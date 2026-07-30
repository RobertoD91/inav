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

- single ESC on motor 1 only (see below)
- needs a hardware UART (no SoftSerial), and targets with >512KB flash

About the single motor limitation: it's not as arbitrary as it looks. Two ESCs
can't share the wire — Spektrum ESCs all ship with the same fixed device ID
(0x40) and there's no way to change it, so on a shared bus they'd both answer
handshakes and telemetry polls at the same time. And even if the IDs were
different, SRXL2 carries one set of channel data for the whole bus, so every
ESC on it would read the same throttle channel — no differential thrust, which
kills the main reason to have per-motor control in the first place. The real
path to twins is one UART per ESC, each with its own bus and its own throttle
from the mixer. That needs the driver state to become per-instance instead of
the current single global one; it's a reasonable follow-up, but I'd rather land
the single-motor case first since that's what I can actually test on my bench.

A note for review: I picked serial function bit 28 and permanentId 70 for the
new mode box, which were the next free ones at the time of writing. Happy to
renumber if they're already spoken for by another PR.

Companion configurator PR adds the SRXL2 protocol entry, the port function and
the mode name.
