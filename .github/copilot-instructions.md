# INAV Copilot Instructions

## Project Overview

INAV is a C (C99/C11) navigation flight controller firmware for STM32 F4/F7/H7 and AT32 MCUs. It evolved from Cleanflight/Baseflight. The codebase targets resource-constrained embedded systems — every decision about memory, abstractions, and code structure reflects that.

## Architecture

Source lives under `src/main/` with these key subsystems:

| Directory | Role |
|---|---|
| `fc/` | Core flight controller: init (`fc_init.c`), main loop (`fc_core.c`), task scheduling (`fc_tasks.c`), MSP handling (`fc_msp.c`) |
| `flight/` | PID controller (`pid.c`), motor mixer (`mixer.c`), altitude/position hold |
| `navigation/` | Waypoint missions, RTH, position estimation |
| `sensors/` | Gyro, accel, baro, GPS, rangefinder — detection, calibration, filtering |
| `drivers/` | Hardware abstraction: SPI, I2C, UART, timers, DMA |
| `config/` | Parameter Group (PG) system for persistent settings |
| `io/` | OSD, LED strips, serial |
| `rx/` | Receiver protocols (CRSF, SBUS, IBUS) |
| `telemetry/` | SmartPort, MAVLink, LTM, CRSF telemetry |
| `target/` | Per-board hardware definitions (`target.h`, `target.c`) |
| `programming/` | Logic conditions and global functions (GUI-programmable) |

**Control flow**: `main.c` → `fc_init.c:init()` → `fc_tasks.c:tasksInit()` → cooperative scheduler. Critical path: Gyro → PID → Mixer → Motors (highest priority).

## Build System

CMake 3.13+ with out-of-source builds. **Ruby is required** for settings generation.

```bash
# Build firmware for a specific target
mkdir build && cd build
cmake ..
make MATEKF722SE

# Build SITL (host simulation)
cmake -DSITL=ON ..
make

# Build and run unit tests (native, no cross-compiler)
mkdir testing && cd testing
cmake -DTOOLCHAIN= ..
make check
```

Each board target lives in `src/main/target/TARGETNAME/` with `target.h` (pin definitions, `#define USE_*` feature flags) and optional `CMakeLists.txt`. Targets register via MCU-specific functions like `target_stm32f405xg()`.

## Settings System (Critical Path)

CLI settings follow this pipeline: **`fc/settings.yaml`** → Ruby script (`src/utils/settings.rb`) → generated `settings_generated.{h,c}`.

In `settings.yaml`:
- **`tables`**: string↔enum mappings (e.g., `off_on` → `{ "OFF", "ON" }`)
- **`constants`**: numeric bounds (e.g., `RPYL_PID_MAX: 255`)
- **`groups`**: map to Parameter Groups — each member defines `field`, `min`, `max`, `default_value`, `type`, `table`, `condition`

After changing settings: run `python src/utils/update_cli_docs.py` to regenerate CLI docs.

## Parameter Groups (PG)

All persistent configuration uses the PG system. Pattern:

```c
// 1. Define struct in header (types use _t suffix)
typedef struct { uint8_t gyro_lpf_hz; uint16_t gyro_kalman_q; } gyroConfig_t;

// 2. Register with defaults (PG ID from config/parameter_group_ids.h)
PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 12);

// 3. Access read-only
gyroConfig()->gyro_lpf_hz

// 4. Access mutable (for CLI/MSP writes)
gyroConfigMutable()->gyro_lpf_hz = 80;
```

**When changing a PG struct, you MUST bump the version number** in the registration macro. CI enforces this via `pg-version-check.yml`.

## Motor Output Protocols

### Architecture Overview

Motor output uses a **function-pointer abstraction** in `drivers/pwm_output.c`. A single `pwmWriteFuncPtr` is assigned at init based on the selected protocol, so `pwmWriteMotor(index, value)` dispatches through it to the correct implementation. Key files:

| File | Role |
|---|---|
| `drivers/pwm_mapping.h` | Protocol enum (`motorPwmProtocolTypes_e`), protocol properties |
| `drivers/pwm_mapping.c` | Timer→motor/servo allocation, init, conflict checking |
| `drivers/pwm_output.h` | Public API: `pwmWriteMotor()`, `pwmCompleteMotorUpdate()`, DShot commands |
| `drivers/pwm_output.c` | All protocol implementations: PWM, OneShot, Multishot, Brushed, DShot |
| `flight/mixer.c` | PID→motor mixing, scaling, calls `pwmWriteMotor()` per motor |
| `drivers/timer.h` | Timer infrastructure: `TCH_t`, `timerHardware_t`, DMA API |

### Available Protocols

```c
typedef enum {
    PWM_TYPE_STANDARD = 0,   // 1-2ms pulse @ 400Hz
    PWM_TYPE_ONESHOT125,     // 125-250µs pulse @ 1kHz
    PWM_TYPE_MULTISHOT,      // 5-25µs pulse @ 2kHz
    PWM_TYPE_BRUSHED,        // 0-100% duty cycle, configurable Hz
    PWM_TYPE_DSHOT150,       // Digital 150kbit/s (3MHz timer, 4kHz update)
    PWM_TYPE_DSHOT300,       // Digital 300kbit/s (6MHz timer, 8kHz update)
    PWM_TYPE_DSHOT600,       // Digital 600kbit/s (12MHz timer, 16kHz update)
} motorPwmProtocolTypes_e;
```

### Analog Protocols (PWM / OneShot125 / Multishot / Brushed)

All analog protocols share the same write function (`pwmWriteStandard()`), which writes directly to the timer CCR register: `*ccr = value * pulseScale + pulseOffset`. They differ only in timing parameters passed to `motorConfigPwm()`:

- **Standard PWM**: `sMin=1ms, sLen=1ms` at 400Hz on a 1MHz timer
- **OneShot125**: `sMin=125µs, sLen=125µs` at 1kHz
- **Multishot**: `sMin=5µs, sLen=20µs` at 2kHz
- **Brushed**: `sMin=0, sLen=0` (full period = 100% scale), user-configurable rate

### DShot (Digital Protocol)

DShot encodes throttle as a **16-bit digital frame**: 11-bit throttle (48–2047) + 1-bit telemetry request + 4-bit CRC. Each bit is a fixed-width pulse where the high-time encodes 0 or 1 (bit period = 20 timer ticks; 0 = 7 ticks high, 1 = 14 ticks high).

**Implementation is always DMA-based** (never bit-banging). Two DMA modes exist:

- **Channel DMA** (default): each timer channel has its own DMA stream. Uses `timerPWMConfigChannelDMA()`.
- **Burst DMA** (`USE_DSHOT_DMAR`): a single DMA stream per timer handles up to 4 channels. Enabled per-target in `target.h` for boards with shared DMA stream constraints.

DMA buffer: 18 elements per motor (16 data bits + 2 reset periods).

**DShot commands** (direction, beeper, etc.) use a circular queue. `sendDShotCommand(cmd)` enqueues; during `pwmCompleteMotorUpdate()`, `executeDShotCommands()` dequeues and sends values 0–47 (the special command range) on all motors with telemetry bit forced high.

### ESC Telemetry

ESC telemetry provides per-motor RPM, voltage, current, and temperature via a **dedicated UART** (not the DShot signal line — INAV does not implement bidirectional DShot). Key files: `sensors/esc_sensor.{h,c}`, consumer in `flight/rpm_filter.c`, `sensors/battery.c`, `io/osd.c`. Feature guard: `USE_ESC_SENSOR`.

**Protocol**: BLHeli_32 / KISS compatible — 10-byte frames at 115200 baud:

| Byte | Content |
|---|---|
| 0 | Temperature (°C, uint8) |
| 1–2 | Voltage (big-endian, centivolt) |
| 3–4 | Current (big-endian, centiamp) |
| 5–6 | Consumption (unused) |
| 7–8 | eRPM (big-endian, uint16) |
| 9 | CRC8 checksum |

**Data flow**:

```
ESC hardware → UART (115200 baud) → escSensorUpdate() [called from fc_core.c]
  │  State machine round-robin: requests telemetry bit on one motor at a time
  │  via pwmRequestMotorTelemetry(motor) → sets DShot packet bit 0
  ▼
escSensorData[MAX_SUPPORTED_MOTORS]   // per-motor: rpm, voltage, current, temp
  │
  ├─► rpm_filter.c: rpmFilterUpdateTask() → per-motor notch filters on gyro
  ├─► battery.c: VOLTAGE_SENSOR_ESC / CURRENT_SENSOR_ESC → vbat, amperage
  ├─► osd.c: OSD_ESC_RPM, OSD_ESC_TEMPERATURE elements
  └─► fc_msp.c: MSP2_INAV_ESC_RPM, MSP2_INAV_ESC_TELEM
```

**DShot telemetry bit integration**: In `prepareDshotPacket()`, bit 0 of the 16-bit DShot frame is the telemetry request flag: `(value << 1) | (requestTelemetry ? 1 : 0)`. `escSensorUpdate()` calls `pwmRequestMotorTelemetry(i)` which sets `motors[i].requestTelemetry = true` for the current motor in round-robin. In **listenOnly mode** (`esc_sensor_listen_only = ON`), BLHeli32 Auto Telemetry sends data without DShot bit requests.

**RPM → eRPM conversion**: `rpm = eRPM * 100 / (motorPoleCount / 2)`. The `motor_poles` setting (default 14) is critical for correct RPM calculation and RPM-based gyro filtering.

**Key data structures**:

```c
escSensorData_t {
    uint8_t  dataAge;       // incremented on timeout, 255 = invalid
    int16_t  temperature;   // °C
    int16_t  voltage;       // centivolt
    int32_t  current;       // mA
    uint32_t rpm;           // mechanical RPM
}

escSensorConfig_t {         // PG-managed
    uint16_t currentOffset; // offset for FC/VTX/cam current draw (mA)
    uint8_t  listenOnly;    // 1 = BLHeli32 Auto Telemetry (no DShot request)
}
```

### Mixer → Motor Output Flow

```
PID loop → motor[i] array (FASTRAM int16_t) → writeMotors() →
  ├─ DShot: scale 1000-2000 → 48-2047 via handleOutputScaling()
  └─ Analog: use value directly (or 3D scaling for reversible)
  → pwmWriteMotor(i, value) per motor
  → pwmCompleteMotorUpdate() [DShot only]:
      prepareDshotPacket() → load DMA buffers → start DMA transfer
```

### Init Flow

```
main() → init() → pwmMotorAndServoInit() →
  ├─ pwmBuildTimerOutputList()    // classify timers as motors/servos/LED
  ├─ pwmInitMotors() →
  │    ├─ pwmMotorPreconfigure()  // assign motorWritePtr based on protocol
  │    └─ per motor: pwmMotorConfig() →
  │         ├─ analog: motorConfigPwm() → pwmOutConfig() → timerConfigBase()
  │         └─ DShot: motorConfigDshot() → pwmOutConfig() + timerPWMConfigChannelDMA()
  └─ pwmInitServos()
```

### Platform-Specific Timer Implementations

| Platform | Timer implementation file | DMA API | Burst DMA |
|---|---|---|---|
| STM32F4 | `drivers/timer_impl_stdperiph.c` | StdPeriph | Supported (per-target) |
| STM32F7/H7 | `drivers/timer_impl_hal.c` | HAL | Supported (per-target) |
| AT32 | `drivers/timer_impl_stdperiph_at32.c` | AT32 DMAMUX | Not used |

`USE_DSHOT` and `USE_DSHOT_DMAR` are per-target `#define`s in `target.h`.

### Key Data Structures

```c
pwmOutputPort_t {
    TCH_t *tch;                          // timer channel handle
    volatile timCCR_t *ccr;              // CCR register for analog write
    float pulseScale, pulseOffset;       // value→µs conversion
    timerDMASafeType_t dmaBuffer[18];    // DMA buffer for DShot
}

pwmOutputMotor_t {
    pwmOutputPort_t *pwmPort;  // physical port (NULL if not timer-based)
    uint16_t value;            // last written value
    bool requestTelemetry;     // DShot telemetry flag
}

motorConfig_t {                // PG-managed persistent config
    uint16_t mincommand;       // disarmed value (typically 1000)
    uint16_t motorPwmRate;     // update frequency (brushed only)
    uint8_t motorPwmProtocol;  // index in motorPwmProtocolTypes_e
    uint16_t digitalIdleOffsetValue;
    uint8_t motorPoleCount;    // for RPM calculation from eRPM
}
```

## Sensors

### Architecture: `drivers/` vs `sensors/` Split

- **`drivers/<sensor>/`** — Hardware abstraction: bus communication (SPI/I2C), chip-specific detection, raw data reads. Contains the device struct (`xxxDev_t`) with function pointers.
- **`sensors/`** — High-level logic: detection orchestration, calibration, filtering, conversion, alignment, health monitoring. Contains the state struct (e.g. `baro_t`) that wraps the dev struct.
- **`sensors/initialisation.c`** — Master init: calls `gyroInit()`, `accInit()`, `baroInit()`, `compassInit()`, `pitotInit()`, `rangefinderInit()`, `opflowInit()`, `temperatureInit()` in sequence. Persists detected sensors to EEPROM.

GPS is a special case: lives in `io/gps.h` (serial protocol, not a bus device).

### Driver Abstraction: Function Pointer Tables

Each sensor type has a `xxxDev_t` struct with function pointers populated during detection:

| Sensor | Dev Struct | Key Function Pointers |
|---|---|---|
| Gyro | `gyroDev_t` | `initFn`, `readFn`, `temperatureFn`, `intStatusFn`, `scale` |
| Accel | `accDev_t` | `initFn`, `readFn`, `acc_1G` |
| Baro | `baroDev_t` | `start_ut`, `get_ut`, `start_up`, `get_up`, `calculate` |
| Compass | `magDev_t` | `init`, `read`, `magAlign` |
| Rangefinder | `rangefinderDev_t` | `init`, `update`, `read`, `maxRangeCm` |
| Pitot | `pitotDev_t` | `start`, `get`, `calculate` |

### Detection Pattern (switch-case with fall-through)

All sensors use the same pattern. Example from baro:

```c
bool baroDetect(baroDev_t *dev, baroSensor_e baroHardwareToUse) {
    switch (baroHardwareToUse) {
    case BARO_AUTODETECT:
    case BARO_BMP085:
#ifdef USE_BARO_BMP085
        if (bmp085Detect(dev)) { baroHardware = BARO_BMP085; break; }
#endif
        if (baroHardwareToUse != BARO_AUTODETECT) break;
        FALLTHROUGH;
    case BARO_MS5611:
        // ...fall through all supported sensors
    }
}
```

Identical pattern for `gyroDetect()`, `compassDetect()`, `pitotDetect()`, etc.

### Sensor Data Flow (Gyro — critical path)

```
Hardware (SPI/I2C) → gyroDev->readFn() → gyroADCRaw[3]
  → subtract gyroZero (calibration)
  → applySensorAlignment() (chip orientation: CW0/90/180/270 + flip)
  → applyBoardAlignment() (rotation matrix from user config)
  → × gyroDev->scale → gyro.gyroADCf[3] (°/s)
  → filter chain: RPM filter → LPF → Dynamic Notch → Kalman
  → IMU fusion → PID controller → Mixer → Motors
```

### Board Alignment

Defined in `sensors/boardalignment.{h,c}`:
- **PG**: `boardAlignment_t` — `rollDeciDegrees`, `pitchDeciDegrees`, `yawDeciDegrees`
- Two levels applied in sequence: per-chip `applySensorAlignment()` (8 orientations) → board-level `applyBoardAlignment()` (rotation matrix)
- Compass has an additional independent alignment layer for external magnetometers

### Supported Hardware

- **Gyro/Accel**: MPU6000, MPU6500, MPU9250, BMI160, BMI088, ICM20689, ICM42605, BMI270, LSM6DXX
- **Baro**: BMP085, MS5611, MS5607, BMP280, BMP388, LPS25H, SPL06, DPS310, 2SMPB-02B
- **Compass**: HMC5883L, AK8975, AK8963, QMC5883L, QMC5883P, LIS3MDL, RM3100, VCM5883, MLX90393, IST8310, IST8308
- **Rangefinder**: VL53L0X, VL53L1X, US42, TOF10102, TeraRanger EVO, USD1, NanoRadar
- **Pitot**: MS4525, ADC, DLVR L10D, Virtual
- **GPS**: u-blox M8+ (auto-config, protocol ≥15.0 required), MSP

### GPS

Lives in `io/gps.{h,c}`. Protocols: `GPS_UBLOX` (primary, auto-config), `GPS_MSP` (remote), `GPS_FAKE` (test). State machine: `GPS_UNKNOWN` → `GPS_INITIALIZING` → `GPS_RUNNING`. Key struct `gpsSolutionData_t`: fixType, numSat, lat/lon (1e-7°), alt (cm), velNED, groundSpeed, groundCourse, eph/epv, hdop.

## Telemetry

### Architecture

Each protocol implements 3 functions: `init<Protocol>Telemetry()`, `check<Protocol>TelemetryState()`, `handle<Protocol>Telemetry()`. All dispatched from `telemetry.c` under `#ifdef USE_TELEMETRY_*` guards. Task: `TASK_TELEMETRY` in `fc_tasks.c`.

### Supported Protocols (11 total)

| Protocol | Files | Key Characteristics |
|---|---|---|
| **SmartPort** | `telemetry/smartport.*` | FrSky S.Port, polled, half-duplex |
| **CRSF** | `telemetry/crsf.*` | Crossfire/ELRS, shares RX port, GPS+battery+attitude+flight mode |
| **MAVLink** | `telemetry/mavlink.*` | v1/v2, bidirectional, waypoint upload/download, configurable rates per stream |
| **LTM** | `telemetry/ltm.*` | Lightweight, G/A/S/O/X/N frames at different rates, shareable port |
| **HoTT** | `telemetry/hott.*` | Graupner, EAM+GPS modules |
| **iBUS** | `telemetry/ibus.*` | FlySky, shareable port with RX |
| **SBUS2** | `telemetry/sbus2.*` | Futaba, 32 slots in SBUS frame gaps |
| **SRXL** | `telemetry/srxl.*` | Spektrum, text generation OSD |
| **GHST** | `telemetry/ghst.*` | ImmersionRC Ghost, shares RX port |
| **Jeti EX Bus** | `telemetry/jetiexbus.*` | Jeti, 128-byte EX frames |
| **SIM** | `telemetry/sim.*` | GSM modem, GPS/failsafe/events via SMS |

### MAVLink Details

- Versions v1 and v2 (configurable). Autopilot type: `GENERIC` or `ARDUPILOT` (ArduCopter/ArduPlane mode mapping).
- **TX streams** with configurable rates: heartbeat, sys_status, attitude, vfr_hud, gps_raw_int, global_position_int, rc_channels, battery_status.
- **RX handlers**: `MISSION_COUNT/ITEM/REQUEST` (full waypoint protocol), `RC_CHANNELS_OVERRIDE`, `COMMAND_INT`, `ADSB_VEHICLE`.

### Port Sharing

CRSF, GHST, SRXL, SBUS2 telemetry shares the RX serial port (bidirectional). LTM and iBUS can share via `TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK`. Other protocols need a dedicated serial port with `FUNCTION_TELEMETRY_*` flag.

## Receivers (RX)

### Abstraction

Each RX protocol populates an `rxRuntimeConfig_t` with function pointers during init:

```c
typedef struct rxRuntimeConfig_s {
    uint8_t channelCount;
    rcReadRawDataFnPtr rcReadRawFn;        // read raw channel value
    rcFrameStatusFnPtr rcFrameStatusFn;    // RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE
    rcProcessFrameFnPtr rcProcessFrameFn;  // optional post-processing
    uint16_t *channelData;                 // channel data buffer
} rxRuntimeConfig_t;
```

### Supported Protocols (15 serial + MSP + SIM)

| Protocol | File | Baud | Notes |
|---|---|---|---|
| **CRSF** | `rx/crsf.c` | 420000 | Crossfire/ELRS, bidirectional |
| **GHST** | `rx/ghst.c` | 420000 | ImmersionRC Ghost |
| **SBUS** | `rx/sbus.c` | 100000 | FrSky, inverted |
| **SBUS Fast** | `rx/sbus.c` | 200000 | Fast variant |
| **SBUS2** | `rx/sbus.c` | 100000 | Futaba, with telemetry slots |
| **FBUS** | `rx/fport2.c` | — | FrSky Bus |
| **F.Port** | `rx/fport.c` | — | FrSky, RC+telemetry single wire |
| **F.Port 2** | `rx/fport2.c` | — | FrSky v2 |
| **iBUS** | `rx/ibus.c` | 115200 | FlySky |
| **SUMD** | `rx/sumd.c` | 115200 | Graupner |
| **Spektrum** | `rx/spektrum.c` | — | DSM 1024/2048 + SRXL |
| **SRXL2** | `rx/srxl2.c` | — | Spektrum v2 |
| **Jeti EX Bus** | `rx/jetiexbus.c` | — | Jeti |
| **MAVLink** | `rx/mavlink.c` | — | RC via `RC_CHANNELS_OVERRIDE` |
| **MSP** | `rx/msp.c` | — | RC via MSP protocol |

### Channel Data Flow

```
RX hardware → UART → protocol driver (decode frame)
  → rcFrameStatusFn() → RX_FRAME_COMPLETE
  → rcReadRawFn() per channel → remap via rcmap[] (AETR default)
  → validate (rx_min_usec..rx_max_usec), hold last valid on invalid
  → rcChannels[].data (1000-2000 PWM range)
  → failsafe check → rc_controls.c → flight modes, PID inputs
```

### RSSI / Link Quality

Sources (auto-detect priority): ADC pin → RX channel → protocol-native (CRSF/GHST) → MSP. Digital protocols provide `rxLinkStatistics_t`: uplinkRSSI (dBm), uplinkLQ (0-100), uplinkSNR (dB), rfMode, txPower (mW).

## Coding Conventions

- **Style**: 4-space indent, K&R braces, `#pragma once` for headers
- **Naming**: types `_t`, enums `_e`, functions `camelCase`, constants `UPPER_SNAKE`, booleans `is*/can*/has*`
- **Memory attributes**: `FASTRAM`, `DMA_RAM`, `STATIC_FASTRAM` — respect these for real-time performance
- **Feature guards**: wrap optional code in `#ifdef USE_FEATURE` / `#endif`
- **Test visibility**: use `STATIC_UNIT_TESTED` instead of `static` when a function needs test access
- **MISRA C**: follow MISRA C guidelines

## Unit Tests

Google Test framework, files in `src/test/unit/*_unittest.cc` (C++). Tests link against the C firmware sources. Disabled tests use `.cc.txt` extension.

## Cross-Platform Considerations

Code must work across STM32F4/F7/H7 and AT32F43x. Use `#if defined(STM32F4)` guards for MCU-specific code. Key differences: F7/H7 have cache management, H7 has complex memory regions (DTCM, SRAM), AT32 has different peripheral APIs. Timer/DMA implementations are split per platform (see Motor Output Protocols section).

## Common Pitfalls

- Breaking PG structure without version bump → CI failure and config corruption
- Breaking MSP protocol → configurator incompatibility
- Ignoring `FASTRAM` placement → real-time performance degradation
- Hardcoding values instead of using PG system
- Forgetting `#ifdef USE_*` guards → build failures on targets without that feature
- Mixing DMA streams across motors/servos without checking `pwm_mapping.c` conflict logic
- PRs must target `maintenance-X.x` branch (current version), never `master`
