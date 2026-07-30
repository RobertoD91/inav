# Add Spektrum SRXL2 Smart ESC support

This adds support for driving a Spektrum Smart ESC directly from a UART using
the SRXL2 protocol, instead of PWM. The ESC signal wire goes to the TX pad of a
free hardware UART and the same wire carries telemetry back (voltage, current,
RPM, FET temperature), so you get Smart ESC telemetry without needing a
Spektrum receiver in between.

The flight controller takes the bus master role, following the state machine of
the official SpektrumRC/SRXL2 reference code: addressed handshake polls, a
reply to the ESC's unprompted handshake at power-up, the final broadcast
handshake, then channel data at 100Hz with a telemetry poll every 10th frame.
Channel values use the normal Spektrum ±100% range (0x2AA0..0xD554) — sending
raw 0 as throttle makes the ESC reject it as invalid and keep beeping, that one
took a while to figure out on the bench.

What's included:

- `SRXL2` motor protocol (`motor_pwm_protocol = SRXL2`), no hardware timer used
- "Spektrum SRXL2 ESC" serial port function for the Ports tab
- ESC telemetry wired into the esc_sensor backend, so `vbat_meter_type = ESC`,
  `current_meter_type = ESC`, OSD ESC RPM/temperature etc. all work as usual.
  RPM is converted from electrical RPM using `motor_poles`.
- A MOTOR REVERSE mode for reversible ESCs. It drives the SRXL2 reverse
  channel; when the direction change is actually applied is up to the ESC
  firmware (it only reverses through zero throttle), so the switch is safe to
  flip in advance, e.g. for reverse-thrust landings.
- `debug_mode = SRXL2` with counters (frames, CRC errors, handshake state)
  that make wiring problems easy to spot from the Sensors tab
- `docs/SRXL2_ESC.md` with wiring, configuration and troubleshooting notes

Bench tested on a MATEKF405SE with an Avian Lite 85A (SPMXAE85A) and a 14 pole
motor, fixed wing configuration with a single motor: handshake, throttle,
voltage/current/RPM telemetry and the reverse switch all working. Not flight
tested yet.

Current limitations:

- single ESC on motor 1 only, no multi-ESC bus support for now
- needs a hardware UART (no SoftSerial), and targets with >512KB flash

A note for review: I picked serial function bit 28 and permanentId 70 for the
new mode box, which were the next free ones at the time of writing. Happy to
renumber if they're already spoken for by another PR.

Companion configurator PR adds the SRXL2 protocol entry, the port function and
the mode name.
