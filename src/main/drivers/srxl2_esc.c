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

#include "common/crc.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/srxl2_esc.h"

#include "fc/config.h"

#include "flight/mixer.h"

#include "io/serial.h"

#if defined(USE_SERIAL)

#define SRXL2_ESC_BAUDRATE                115200
#define SRXL2_ESC_FRAME_TIMEOUT_US        500
#define SRXL2_ESC_INTERVAL_US             10000
#define SRXL2_ESC_REQUEST_EVERY_N_FRAMES  10

#define SRXL2_HEADER                      0xA6
#define SRXL2_PACKET_TYPE_HANDSHAKE       0x21
#define SRXL2_PACKET_TYPE_CONTROL         0xCD
#define SRXL2_PACKET_TYPE_TELEMETRY       0x80
#define SRXL2_CONTROL_CMD_CHANNEL         0x00

#define SRXL2_RECEIVER_ID                 0x21
#define SRXL2_ESC_ID                      0x40
#define SRXL2_RECEIVER_PRIORITY           0x0A
#define SRXL2_RECEIVER_BAUDRATE           1
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

static uint16_t scaleThrottleToSrxl2(uint16_t value)
{
    if (value <= motorConfig()->mincommand) {
        return 0;
    }

    return (uint16_t)scaleRange(value, motorConfig()->mincommand, getMaxThrottle(), 0, 65532);
}

static void srxl2EscSendHandshake(void)
{
    srxl2Handshake_t packet;

    packet.header = SRXL2_HEADER;
    packet.type = SRXL2_PACKET_TYPE_HANDSHAKE;
    packet.len = sizeof(packet);
    packet.sourceId = SRXL2_RECEIVER_ID;
    packet.destId = SRXL2_ESC_ID;
    packet.priority = SRXL2_RECEIVER_PRIORITY;
    packet.baudrate = SRXL2_RECEIVER_BAUDRATE;
    packet.info = SRXL2_RECEIVER_INFO;
    packet.uid = SRXL2_RECEIVER_UID;
    packet.crc = swap16(crc16_ccitt_update(0, &packet, sizeof(packet) - sizeof(packet.crc)));

    serialWriteBuf(srxl2EscPort, (const uint8_t *)&packet, sizeof(packet));
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
    packet.channelData.reverse = 0;
    packet.crc = swap16(crc16_ccitt_update(0, &packet, sizeof(packet) - sizeof(packet.crc)));

    serialWriteBuf(srxl2EscPort, (const uint8_t *)&packet, sizeof(packet));
    frameCounter++;
}

static bool srxl2EscValidateFrame(const uint8_t *buffer, uint8_t length)
{
    if (length < 6) {
        return false;
    }

    const uint16_t expected = swap16(crc16_ccitt_update(0, buffer, length - 2));
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
    srxl2Telemetry[0].current = (swap16(esc.currentMotor) == 0xFFFF) ? 0 : (int32_t)(swap16(esc.currentMotor) * 10);
    srxl2Telemetry[0].rpm = (swap16(esc.rpm) == 0xFFFF) ? 0 : (uint32_t)swap16(esc.rpm) * 10;
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
        if (rxBuffer[1] == SRXL2_PACKET_TYPE_HANDSHAKE && rxBuffer[3] == SRXL2_ESC_ID) {
            escId = rxBuffer[3];
            handshakeConfirmed = true;
        } else if (rxBuffer[1] == SRXL2_PACKET_TYPE_TELEMETRY && rxBuffer[3] == SRXL2_RECEIVER_ID && rxBuffer[4] == XBUS_ESC_ID) {
            handshakeConfirmed = true;
            srxl2EscParseEscTelemetry(&rxBuffer[4]);
        } else if (rxBuffer[1] == SRXL2_PACKET_TYPE_TELEMETRY && rxBuffer[3] == 0xFF) {
            handshakeConfirmed = false;
        }
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

    srxl2EscPort = openSerialPort(portConfig->identifier, FUNCTION_SRXL2_ESC, NULL, NULL, SRXL2_ESC_BAUDRATE, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!srxl2EscPort) {
        return false;
    }

    memset(srxl2MotorValues, 0, sizeof(srxl2MotorValues));
    memset(srxl2Telemetry, 0xFF, sizeof(srxl2Telemetry));
    rxBufferPosition = 0;
    lastTxTimeUs = 0;
    frameCounter = 0;
    escId = SRXL2_ESC_ID;
    handshakeConfirmed = false;
    srxl2EscInitialized = true;

    return true;
}

void srxl2EscWriteMotor(uint8_t index, uint16_t value)
{
    if (index < MAX_SUPPORTED_MOTORS) {
        srxl2MotorValues[index] = scaleThrottleToSrxl2(value);
    }
}

void srxl2EscUpdate(timeUs_t currentTimeUs)
{
    if (!srxl2EscInitialized || !srxl2EscPort) {
        return;
    }

    while (serialRxBytesWaiting(srxl2EscPort) > 0) {
        srxl2EscHandleIncomingByte(serialRead(srxl2EscPort));
    }

    if ((currentTimeUs - lastTxTimeUs) < SRXL2_ESC_INTERVAL_US) {
        return;
    }

    lastTxTimeUs = currentTimeUs;

    if (!handshakeConfirmed) {
        srxl2EscSendHandshake();
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