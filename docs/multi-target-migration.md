# Multi-target firmware migration

Baseline: `feature/as5048b-sanity` commit `f21c0dfb1fdb8ad25ff55f3d19a75f2872e9bbce`.
Development branch: `dev`. Keep `main` and the original feature branch unchanged until verified.

## Current state

The application is one ESP-IDF project under `main/`, with compile-time
`RC_TRANSMITTER` in `main/rc_config.h`. Current control traffic is broadcast
on Wi-Fi channel 6, with fixed TX/RX MAC constants and receiver heartbeat.
The TX uses two AS5048B controls. RX uses SA8302 motor outputs and
battery/USB/charging, lights, reset and link diagnostics.

Do not replace this working firmware with the old `main` branch.

## Intended targets

- `transmitter`: existing AS5048B controls, speed mode/expo, charging,
  battery and status LED.
- `rx_skid_steer`: existing mixer/motor timing, SA8302, charging,
  lights, battery and diagnostics. Preserve existing behaviour.
- `rx_conventional`: SA8336D propulsion, SA8301S steering, AS5048B
  steering position feedback, 1S battery; no VUSB/CHG dependencies.

Each target has its own CMake project, `sdkconfig`, build directory,
serial-port setting and VS Code build/flash/monitor task. Common source
is compiled from shared components, never copied between targets.

## Safe migration sequence

1. Extract protocol structures, common ESP-NOW transport, battery sampling,
   LED support and diagnostic utilities without altering packet bytes.
2. Move TX and skid-steer RX entry points into separate projects and remove
   `RC_TRANSMITTER` as a manually edited build switch.
3. Add dedicated VS Code tasks and configurable ports; build both targets
   and compare run-time behaviour with the baseline.
4. Introduce a versioned receiver identity/capabilities handshake.
   Explicitly manage trusted peers; broadcast alone is not authentication.
   Require compatible protocol and neutral throttle before arming.
5. Add conventional RX with motor outputs disabled initially.
6. Verify both H-bridge truth tables, safe startup states and GPIO map.
7. Add AS5048B feedback, field diagnostics, calibration NVS and fault
   latching before enabling closed-loop steering and propulsion.

## Steering safety

Treat missing I2C communication, magnetic field too weak/strong, invalid
sensor diagnostic status, implausible angular jumps, stale readings and
stalled movement as distinct faults. Validate the exact AS5048B register
definitions and magnet thresholds against its datasheet. An invalid
reading must never produce a new steering actuation command. Calibration
is manual and versioned in NVS. Do not seek mechanical stops on boot.

## Acceptance criteria

- Both existing TX and skid-steer RX compile from independent tasks.
- Existing skid-steer controls, radio timing, charging and lighting work
  unchanged on hardware.
- Shared protocol, radio and peripherals are compiled from one source.
- `Build All` builds all supported targets without editing source flags.
- Each target flashes/monitors using its own configurable port.
- Conventional RX remains disarmed until driver truth tables, steering
  sensor validity and calibration are verified.
