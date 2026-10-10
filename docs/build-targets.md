# Independent firmware targets (transitional stage)

Run these in an **ESP-IDF 5.5.2 activated terminal** from the repository root:

```powershell
python tools/target.py transmitter build
python tools/target.py rx_skid_steer build
python tools/target.py rx_conventional build
python tools/target.py all build
python tools/target.py transmitter flash --port COM10
```

VS Code: **Terminal → Run Task**, then select the RC build or flash task.
The flash task prompts for a port; no source flag edits are needed.
You can also set `RC_PORT_TRANSMITTER`, `RC_PORT_RX_SKID_STEER`, or
`RC_PORT_RX_CONVENTIONAL` in the terminal environment.

The transmitter and skid-steer receiver currently compile the *same existing*
`main/*.c` sources, with an isolated per-target compile definition.
This preserves the existing logic while allowing incremental extraction into
shared components. It is not yet a full component refactor. The root ESP-IDF
project remains available as a legacy fallback. The conventional receiver is
an inert placeholder: do not connect motor drivers expecting operation.

Build directories are separate under `build/<target>`; the generated
`sdkconfig` files live in each firmware project. Do not share them.

**Validation outstanding:** ESP-IDF compilation and hardware regression tests
must be performed on the developer machine before considering this migration
verified. The shared radio protocol is not yet changed, and automatic receiver
identification is not implemented.
