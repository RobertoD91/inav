# Spektrum SRXL2 ESC support

INAV can drive a Spektrum Smart ESC (Avian/Firma "Smart" line) directly over its
SRXL2 throttle wire, replacing the PWM signal with a digital half-duplex link
that also carries ESC telemetry (RPM, voltage, current, FET temperature) back
to the flight controller. The flight controller acts as the SRXL2 bus master
(device ID `0x21`, the role a Spektrum receiver normally plays) and the ESC is
the polled device (ID `0x40`).

## Wiring

- Connect the ESC signal wire to the **TX pad** of a free **hardware UART**
  (not SoftSerial). SRXL2 is a single-wire half-duplex bus: the UART runs in
  bidirectional mode on the TX pin and the RX pad of that UART stays unused.
- Common ground between FC and ESC as usual.
- On F4 targets the pin runs open-drain with the internal pull-up
  (the same proven configuration Betaflight uses for SRXL2), so the line
  idles high with no external components.
- Mind target pin sharing: e.g. on MATEKF405SE the TX2 pad (PA2) doubles as
  SoftSerial1 — do not enable SoftSerial on the pad used for the ESC.

## Configuration

Ports tab: set the chosen UART's peripheral function to **Spektrum SRXL2 ESC**
(115200 baud). Outputs/Mixer: set the ESC protocol to **SRXL2**.

CLI equivalent:

```
serial <port> 268435456 115200 115200 0 115200   # function bit 28 = SRXL2_ESC
set motor_pwm_protocol = SRXL2
save
```

Telemetry consumers work as with other ESC telemetry sources, e.g.
`set vbat_meter_type = ESC` to read pack voltage from the ESC.

Current limitations:

- Single ESC (motor index 0). Multi-ESC SRXL2 is not supported.
- RPM is reported as sent by the ESC (no motor-pole scaling applied).
- Available on targets with more than 512KB flash (`USE_SRXL2_ESC`).

### Reversible ESCs

When the motor protocol is SRXL2 a **MOTOR REVERSE** mode appears in the
Modes tab. Assign it to a switch to drive the ESC's reverse channel
(channel 6): switch active = reverse requested. The ESC's own firmware
decides when the direction change is applied (typically only through zero
throttle), so the switch can be armed safely in advance — e.g. for
reverse-thrust landings. INAV's throttle handling is unchanged; this is a
plain pass-through of the direction request.

## Protocol details

The driver (`src/main/drivers/srxl2_esc.c`) implements the master side of the
official Spektrum SRXL2 state machine (reference: SpektrumRC/SRXL2
`spm_srxl.c`):

- **Handshake**: an addressed handshake (`src 0x21 → dst 0x40`) is sent every
  50ms. The ESC's own unprompted handshake (`dst 0x00`, sent after its 50ms
  power-up listen window) is answered immediately. When the ESC replies with a
  handshake addressed back to us, the driver sends the **final broadcast
  handshake (`dst 0xFF`)** — required by the official device state machine to
  enter its Running state — and switches to normal operation.
- **Baud**: the handshake advertises `baudSupported = 0` (115200 only). The
  driver never switches baud, so it must never advertise 400k capability:
  the ESC would switch on the broadcast handshake and drop off the bus.
- **Control frames** (type `0xCD`) are sent at 100Hz with channel mask `0x41`:
  channel 0 = throttle, channel 6 = reverse. Every 10th frame polls the ESC
  for telemetry (`replyID = 0x40`); the ESC answers with a telemetry frame
  (type `0x80`) carrying the standard XBUS ESC telemetry block.
- **Channel values** use the Spektrum convention: the full 16-bit range spans
  ±150% servo travel with 32768 = center. The driver maps
  `mincommand..max_throttle` onto the standard ±100% band
  `0x2AA0..0xD554`. Raw 0 must never be sent as throttle — it decodes as
  −150% (a sub-900µs pulse equivalent) and Smart ESCs reject it as an
  invalid throttle, keeping their no-signal beep even with the bus link up.
- **Half-duplex turnaround**: the driver never starts transmitting within
  300µs of the last received byte, so ESC replies are not stomped. All frames
  use CRC16-CCITT (seed 0) transmitted big-endian.

## Debugging

Set `debug_mode = SRXL2` and watch the debug values (Configurator Sensors tab
or blackbox):

| debug | Meaning |
|-------|---------|
| 0 | Link state: 0 = handshaking, 1 = running |
| 1 | Handshake frames transmitted |
| 2 | Bytes received (includes our own half-duplex echo) |
| 3 | Valid SRXL2 frames received |
| 4 | CRC failures (non-zero = electrical/wiring problem) |
| 5 | Handshake replies received from the ESC |
| 6 | Telemetry frames received from the ESC |
| 7 | Current throttle channel value (~10912 at idle) |

Interpretation guide: debug 1 growing with debug 2 stuck at 0 means nothing is
coming back (wiring); debug 4 growing means a noisy line; debug 5 > 0 with
debug 0 = 0 means a handshake logic problem; debug 6 growing proves the ESC is
in its running state and answering polls.

## Development notes

Issues found and fixed while bringing the driver up on real hardware
(MATEKF405SE + Spektrum Smart ESC), kept here so future work doesn't
rediscover them:

1. **CRC byte order**: the CRC is big-endian on the wire; the RX validator
   must compare without byte-swapping the computed value.
2. **Current units**: the XBUS telemetry block reports current in 10mA units,
   which is already the centiampere unit `escSensorData_t` expects — no
   scaling.
3. **Handshake state machine**: the ESC's unprompted handshake (`dst 0`) is a
   request, not a confirmation — it must be answered, and only the addressed
   reply plus our final `0xFF` broadcast completes the sequence.
4. **Line drive**: plain `SERIAL_BIDIR` (open-drain + pull-up). `SERIAL_BIDIR_PP`
   must not be used: the push-pull pin has no pull-up, so the line floats
   whenever the half-duplex UART releases the driver between frames.
5. **Throttle encoding**: see Channel values above — the ±100% band, never
   raw 0. The symptom of a wrong band is subtle: handshake and telemetry work
   but the ESC keeps its no-throttle beep.
6. **Build system**: `PWM_TYPE_SRXL2` uses no hardware timer
   (`usesHwTimer = false`) and `pwmCompleteMotorUpdate()` must exist on
   non-DSHOT targets too (`USE_DSHOT || USE_SRXL2_ESC` guard).
