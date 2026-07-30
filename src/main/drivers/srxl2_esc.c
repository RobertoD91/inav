/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/srxl2_esc.h"

#include "fc/config.h"

#include "flight/mixer.h"

#include "io/serial.h"

#if defined(USE_SRXL2_ESC)

#define SRXL2_ESC_BAUDRATE                115200
#define SRXL2_ESC_FRAME_TIMEOUT_US        500
#define SRXL2_ESC_INTERVAL_US             10000
#define SRXL2_ESC_HANDSHAKE_INTERVAL_US   50000
// Minimum bus-idle time after the last received byte before we may transmit,
// so we never stomp on an ESC reply in progress (~2 byte times at 115200,
// same value Betaflight uses for its SRXL2 reply quiescence).
#define SRXL2_ESC_QUIESCENCE_US           300
#define SRXL2_ESC_REQUEST_EVERY_N_FRAMES  10

#define SRXL2_HEADER                      0xA6
#define SRXL2_PACKET_TYPE_HANDSHAKE       0x21
#define SRXL2_PACKET_TYPE_CONTROL         0xCD
#define SRXL2_PACKET_TYPE_TELEMETRY       0x80
#define SRXL2_CONTROL_CMD_CHANNEL         0x00

#define SRXL2_RECEIVER_ID                 0x21
#define SRXL2_ESC_ID                      0x40
#define SRXL2_BROADCAST_ID                0xFF
#define SRXL2_RECEIVER_PRIORITY           0x0A
// Per the official SRXL2 spec, baudSupported 0 = 115200 only, 1 = 400k
// capable. We never switch the UART off 115200, so advertising 400k would
// make the ESC change baud on the final broadcast handshake and lose us.
#define SRXL2_RECEIVER_BAUDRATE           0
#define SRXL2_RECEIVER_INFO               0x07
#define SRXL2_RECEIVER_UID                0x27A2C29C

#define XBUS_ESC_ID                       0x20

typedef struct srxl2Handshake_s {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t sourceId;
    uint8_t destId;
    uint8_t priority;
    uint8_t baudrate;
    uint8_t info;
    uint32_t uid;
    uint16_t crc;
} __attribute__((packed)) srxl2Handshake_t;

typedef struct srxl2ChannelData_s {
    int8_t rssi;
    uint16_t frameLosses;
    uint32_t channelMask;
    uint16_t throttle;
    uint16_t reverse;
} __attribute__((packed)) srxl2ChannelData_t;

typedef struct srxl2ControlPacket_s {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t command;
    uint8_t replyId;
    srxl2ChannelData_t channelData;
    uint16_t crc;
} __attribute__((packed)) srxl2ControlPacket_t;

typedef struct srxl2TelemetryPacket_s {
    uint8_t header;
    uint8_t type;
    uint8_t len;
    uint8_t destId;
    uint8_t payload[16];
    uint16_t crc;
} __attribute__((packed)) srxl2TelemetryPacket_t;

typedef struct xbusEsc_s {
    uint8_t identifier;
    uint8_t sid;
    uint16_t rpm;
    uint16_t voltsInput;
    uint16_t tempFet;
    uint16_t currentMotor;
    uint16_t tempBec;
    uint8_t currentBec;
    uint8_t voltageBec;
    uint8_t throttle;
    uint8_t powerOut;
} __attribute__((packed)) xbusEsc_t;

static serialPort_t *srxl2EscPort;
static bool srxl2ReverseActive;
static uint16_t debugTxFrames;
static uint16_t debugRxValidFrames;
static uint16_t debugRxCrcFails;
static uint16_t debugRxBytes;
static uint16_t debugHandshakeReplies;
static uint16_t debugTelemetryFrames;
static timeUs_t lastRxActivityUs;
static bool srxl2EscInitialized;
static uint16_t srxl2MotorValues[MAX_SUPPORTED_MOTORS];
static escSensorData_t srxl2Telemetry[MAX_SUPPORTED_MOTORS];
static uint8_t rxBuffer[sizeof(srxl2TelemetryPacket_t)];
static uint8_t rxBufferPosition;
static timeUs_t lastTxTimeUs;
static uint8_t frameCounter;
static uint8_t escId;
static bool handshakeConfirmed;

static uint16_t swap16(uint16_t value)
{
    return (uint16_t)((value << 8) | (value >> 8));
}

// SRXL2 channel values span the full 16-bit range over +/-150% servo travel
// with 32768 = center (per the official spec comment on SrxlChannelData). A
// Spektrum receiver at -100%/+100% stick sends ~0x2AA0/0xD554, so that band
// is what the ESC's throttle logic expects. Raw 0 is -150% (a sub-900us
// pulse equivalent): Smart ESCs treat it as an invalid throttle and keep
// emitting the no-signal beep even with the bus link up.
#define SRXL2_CHANNEL_LOW                 0x2AA0
#define SRXL2_CHANNEL_HIGH                0xD554

static uint16_t scaleThrottleToSrxl2(uint16_t value)
{
    if (value <= motorConfig()->mincommand) {
        return SRXL2_CHANNEL_LOW;
    }

    // Lowest 2 bits of a channel value are reserved (RFU) - keep them clear.
    return (uint16_t)scaleRange(value, motorConfig()->mincommand, getMaxThrottle(), SRXL2_CHANNEL_LOW, SRXL2_CHANNEL_HIGH) & 0xFFFC;
}

static void srxl2EscSendHandshake(uint8_t destId)
{
    srxl2Handshake_t packet;

    packet.header = SRXL2_HEADER;
    packet.type = SRXL2_PACKET_TYPE_HANDSHAKE;
    packet.len = sizeof(packet);
    packet.sourceId = SRXL2_RECEIVER_ID;
    packet.destId = destId;
    packet.priority = SRXL2_RECEIVER_PRIORITY;
    packet.baudrate = SRXL2_RECEIVER_BAUDRATE;
    packet.info = SRXL2_RECEIVER_INFO;
    packet.uid = SRXL2_RECEIVER_UID;
    packet.crc = swap16(crc16_ccitt_update(0, &packet, sizeof(packet) - sizeof(packet.crc)));

    serialWriteBuf(srxl2EscPort, (const uint8_t *)&packet, sizeof(packet));
    debugTxFrames++;
}

static void srxl2EscSendControlPacket(void)
{
    srxl2ControlPacket_t packet;

    packet.header = SRXL2_HEADER;
    packet.type = SRXL2_PACKET_TYPE_CONTROL;
    packet.len = sizeof(packet);
    packet.command = SRXL2_CONTROL_CMD_CHANNEL;
    packet.replyId = ((frameCounter % SRXL2_ESC_REQUEST_EVERY_N_FRAMES) == 0) ? escId : 0;
    packet.channelData.rssi = 100;
    packet.channelData.frameLosses = 0;
    packet.channelData.channelMask = 0x41;
    packet.channelData.throttle = srxl2MotorValues[0];
    packet.channelData.reverse = srxl2ReverseActive ? SRXL2_CHANNEL_HIGH : SRXL2_CHANNEL_LOW;
    packet.crc = swap16(crc16_ccitt_update(0, &packet, sizeof(packet) - sizeof(packet.crc)));

    serialWriteBuf(srxl2EscPort, (const uint8_t *)&packet, sizeof(packet));
    frameCounter++;
}

static bool srxl2EscValidateFrame(const uint8_t *buffer, uint8_t length)
{
    if (length < 6) {
        return false;
    }

    // CRC is transmitted big-endian (MSB first) on the wire, so read it back the
    // same way and compare against the locally computed value without byte-swapping.
    const uint16_t expected = crc16_ccitt_update(0, buffer, length - 2);
    const uint16_t received = ((uint16_t)buffer[length - 2] << 8) | buffer[length - 1];

    return expected == received;
}

static void srxl2EscParseEscTelemetry(const uint8_t *payload)
{
    xbusEsc_t esc;

    memcpy(&esc, payload, sizeof(esc));

    srxl2Telemetry[0].dataAge = 0;
    srxl2Telemetry[0].temperature = (swap16(esc.tempFet) == 0xFFFF) ? 0 : (int16_t)(swap16(esc.tempFet) / 10);
    srxl2Telemetry[0].voltage = (swap16(esc.voltsInput) == 0xFFFF) ? 0 : (int16_t)swap16(esc.voltsInput);
    // current_motor is reported in 10 mA units, which is exactly the centiampere
    // (0.01 A) unit expected by escSensorData_t, so store it without scaling.
    srxl2Telemetry[0].current = (swap16(esc.currentMotor) == 0xFFFF) ? 0 : (int32_t)swap16(esc.currentMotor);
    // The ESC reports electrical RPM (in 10 RPM units): convert to shaft RPM
    // using the pole count of the attached motor, same as DSHOT telemetry.
    srxl2Telemetry[0].rpm = (swap16(esc.rpm) == 0xFFFF) ? 0 : (uint32_t)swap16(esc.rpm) * 10 * 2 / MAX(2, motorConfig()->motorPoleCount);
}

static void srxl2EscHandleIncomingByte(uint8_t value)
{
    if (rxBufferPosition == 0 && value != SRXL2_HEADER) {
        return;
    }

    if (rxBufferPosition < sizeof(rxBuffer)) {
        rxBuffer[rxBufferPosition++] = value;
    } else {
        rxBufferPosition = 0;
        return;
    }

    if (rxBufferPosition < 3) {
        return;
    }

    const uint8_t expectedLength = rxBuffer[2];
    if (expectedLength == 0 || expectedLength > sizeof(rxBuffer)) {
        rxBufferPosition = 0;
        return;
    }

    if (rxBufferPosition < expectedLength) {
        return;
    }

    if (srxl2EscValidateFrame(rxBuffer, expectedLength)) {
        debugRxValidFrames++;
        if (rxBuffer[1] == SRXL2_PACKET_TYPE_HANDSHAKE && rxBuffer[3] == SRXL2_ESC_ID) {
            if (rxBuffer[4] == 0) {
                // At power-up (or after 50ms of bus silence) the ESC sends an
                // unprompted handshake (sourceId = ESC, destId = 0). Per the
                // official SRXL2 master behaviour, reply immediately with a
                // handshake addressed to it; its addressed reply follows.
                srxl2EscSendHandshake(SRXL2_ESC_ID);
            } else {
                // Handshake addressed back to us: the ESC answered our poll.
                // Per the official SRXL2 device state machine the ESC only
                // enters its running state (and starts accepting throttle) on
                // the final broadcast handshake, so send it now.
                escId = rxBuffer[3];
                debugHandshakeReplies++;
                srxl2EscSendHandshake(SRXL2_BROADCAST_ID);
                handshakeConfirmed = true;
            }
        } else if (rxBuffer[1] == SRXL2_PACKET_TYPE_TELEMETRY && rxBuffer[3] == SRXL2_RECEIVER_ID && rxBuffer[4] == XBUS_ESC_ID) {
            handshakeConfirmed = true;
            debugTelemetryFrames++;
            srxl2EscParseEscTelemetry(&rxBuffer[4]);
        } else if (rxBuffer[1] == SRXL2_PACKET_TYPE_TELEMETRY && rxBuffer[3] == SRXL2_BROADCAST_ID) {
            // Telemetry addressed to 0xFF is the SRXL2 re-handshake request:
            // the ESC lost the link, so restart the discovery sequence.
            handshakeConfirmed = false;
        }
    } else {
        debugRxCrcFails++;
    }

    rxBufferPosition = 0;
}

bool srxl2EscInit(void)
{
    srxl2EscInitialized = false;
    srxl2EscPort = NULL;

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SRXL2_ESC);
    if (!portConfig) {
        return false;
    }

    // SRXL2 is a single-wire half-duplex bus: throttle command and telemetry share
    // one line on the UART TX pin, so the port must be opened in bidirectional mode.
    // Plain SERIAL_BIDIR (open-drain with internal pull-up on F4) is the proven
    // configuration for SRXL2 on STM32 — it is what Betaflight uses for its
    // field-tested SRXL2 driver — and it keeps the line at a defined high level
    // between frames. SERIAL_BIDIR_PP must NOT be used here: the push-pull pin
    // has no pull-up, so the line floats whenever the half-duplex UART releases
    // the driver, feeding noise to the ESC.
    srxl2EscPort = openSerialPort(portConfig->identifier, FUNCTION_SRXL2_ESC, NULL, NULL, SRXL2_ESC_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED | SERIAL_BIDIR);
    if (!srxl2EscPort) {
        return false;
    }

    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        srxl2MotorValues[i] = SRXL2_CHANNEL_LOW;
    }
    memset(srxl2Telemetry, 0xFF, sizeof(srxl2Telemetry));
    rxBufferPosition = 0;
    lastTxTimeUs = 0;
    lastRxActivityUs = 0;
    frameCounter = 0;
    escId = SRXL2_ESC_ID;
    handshakeConfirmed = false;
    srxl2ReverseActive = false;
    debugTxFrames = 0;
    debugRxValidFrames = 0;
    debugRxCrcFails = 0;
    debugRxBytes = 0;
    debugHandshakeReplies = 0;
    debugTelemetryFrames = 0;
    srxl2EscInitialized = true;

    return true;
}

void srxl2EscWriteMotor(uint8_t index, uint16_t value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        srxl2MotorValues[index] = scaleThrottleToSrxl2(value);
    }
}

void srxl2EscSetReverse(bool reverse)
{
    // Transmitted on the SRXL2 reverse channel; when and how the direction
    // change is applied (e.g. only through zero throttle) is governed by the
    // ESC's own firmware.
    srxl2ReverseActive = reverse;
}

void srxl2EscUpdate(timeUs_t currentTimeUs)
{
    if (!srxl2EscInitialized || !srxl2EscPort) {
        return;
    }

    if (serialRxBytesWaiting(srxl2EscPort) > 0) {
        lastRxActivityUs = currentTimeUs;
        while (serialRxBytesWaiting(srxl2EscPort) > 0) {
            debugRxBytes++;
            srxl2EscHandleIncomingByte(serialRead(srxl2EscPort));
        }
    }

    DEBUG_SET(DEBUG_SRXL2, 0, handshakeConfirmed ? 1 : 0);
    DEBUG_SET(DEBUG_SRXL2, 1, debugTxFrames);
    DEBUG_SET(DEBUG_SRXL2, 2, debugRxBytes);
    DEBUG_SET(DEBUG_SRXL2, 3, debugRxValidFrames);
    DEBUG_SET(DEBUG_SRXL2, 4, debugRxCrcFails);
    DEBUG_SET(DEBUG_SRXL2, 5, debugHandshakeReplies);
    DEBUG_SET(DEBUG_SRXL2, 6, debugTelemetryFrames);
    DEBUG_SET(DEBUG_SRXL2, 7, srxl2MotorValues[0]);

    // Half-duplex turnaround: never start transmitting while the ESC may
    // still be replying (this function runs from the scheduler busy-loop, so
    // it re-checks within microseconds).
    if ((currentTimeUs - lastRxActivityUs) < SRXL2_ESC_QUIESCENCE_US) {
        return;
    }

    const timeUs_t txIntervalUs = handshakeConfirmed ? SRXL2_ESC_INTERVAL_US : SRXL2_ESC_HANDSHAKE_INTERVAL_US;
    if ((currentTimeUs - lastTxTimeUs) < txIntervalUs) {
        return;
    }

    lastTxTimeUs = currentTimeUs;

    if (!handshakeConfirmed) {
        srxl2EscSendHandshake(SRXL2_ESC_ID);
    } else {
        srxl2EscSendControlPacket();
        if (frameCounter == 0xFF) {
            frameCounter = 1;
        }
    }
}

bool srxl2EscIsInitialized(void)
{
    return srxl2EscInitialized;
}

bool srxl2EscGetTelemetry(uint8_t index, escSensorData_t *data)
{
    if (!srxl2EscInitialized || !data || index >= MAX_SUPPORTED_MOTORS) {
        return false;
    }

    if (srxl2Telemetry[index].dataAge == ESC_DATA_INVALID) {
        return false;
    }

    *data = srxl2Telemetry[index];
    return true;
}

#else

bool srxl2EscInit(void)
{
    return false;
}

void srxl2EscWriteMotor(uint8_t index, uint16_t value)
{
    UNUSED(index);
    UNUSED(value);
}

void srxl2EscSetReverse(bool reverse)
{
    UNUSED(reverse);
}

void srxl2EscUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

bool srxl2EscIsInitialized(void)
{
    return false;
}

bool srxl2EscGetTelemetry(uint8_t index, escSensorData_t *data)
{
    UNUSED(index);
    UNUSED(data);
    return false;
}

#endif