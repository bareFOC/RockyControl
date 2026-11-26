# Rocky Studio

Control workstation and embedded firmware for a 28-DoF humanoid robot (plus an 8-DoF Striker preset) with a built-in software simulator. Runs on Desktop, Browser (WebAssembly), and Android, paired with RP2350 controller firmware.

## Running the Workstation

### Desktop

```sh
cargo run -p rk-desktop --release
```

### Browser (WASM)

Requires `trunk` (`0.21.14+`) and `wasm32-unknown-unknown`:

```sh
sh scripts/build.sh
trunk serve --port 3000
```

### Android APK

Requires Android SDK 35, NDK 27.1.12297006, `cargo-apk` 0.10+, JDK 21, and `aarch64-linux-android`:

```sh
bash scripts/build-android.sh --install
```

See `scripts/build-android.sh` for SDK/NDK environment overrides. Current APK limitations: no in-app MCP TCP listener, no gamepad input (`gilrs`), single simulator process, and debug signing only.

## Using Rocky Studio

### Connecting
- **Simulator:** Connects in-process with the bundled 28-DoF humanoid model (`humanoid_bundle.zip`) for safe testing without hardware.
- **Hardware (RP2350):** Connect over USB CDC-ACM serial or USB-NCM Ethernet (`10.42.0.2:7777`). Host and firmware must match wire protocol version (`PROTOCOL_VERSION = 8`).

### Safety Modes
- **Teach (default on boot):** Torque is off; the 3D viewport mirrors live joint angles.
- **Drive:** Torque is enabled. Transitions from `Teach` to `Drive` initialize target angles to current measured positions (bumpless transfer).
- **EStop (`Space` or top-bar button):** Immediately cuts torque and latches until cleared. `ClearFaults` returns strictly to `Teach`.
- **Watchdog & Limits:** A 100 ms host-heartbeat loss triggers `EStop`. Per-joint soft limits, slew limits, and torque/velocity ceilings are enforced on the controller.

### 3D Viewport & Robot Models
- **Navigation:** Drag to orbit, use the top-right ViewCube faces/edges/corners or roll arrows, or reset to isometric home.
- **Visual Feedback:** Live solid meshes track measured angles; in `Drive` mode, a wireframe ghost shows commanded targets; joints tint by thermal/current load.
- **Models:** Switch between the bundled 28-DoF humanoid and 8-DoF Striker presets, or load `.urdf`, `.xml`, or `.zip` (URDF + STL/OBJ meshes) bundles. Models are owned by the workstation application; the MCU does not store or serve URDF files.

### Calibration, Balance, Walking & Recovery
- **Calibration & Tuning:** Zero offsets, joint inversion, soft limits, actuator ceilings, balance gains, UVC gait parameters, and impact thresholds can be adjusted at runtime and saved to RP2350 flash (`CommitFlash`).
- **Balance & Walking:** Enable balance/walk from the control panel or gamepad (`gilrs` on Desktop).
- **Fall Recovery:** On `TILT_FALL_CUTOFF`, the Recovery HUD shows the detected fall orientation and pulses the robot mesh red. Recovery is off by default and requires explicit operator confirmation (**Operator confirmed** or **Get Up**). Get-up motions are open-loop keyframes clamped by soft limits and slew limits; any new fault or E-Stop aborts immediately. Always supervise powered get-up attempts on hardware.

## Verification & Hardware Docs

- Full verification suite: `cargo run -p xtask -- sim-verify`
- Firmware wiring, pinouts, and flashing: [`firmware/README.md`](firmware/README.md)
- Simulation acceptance criteria and replay fixtures: [`docs/SIM_ACCEPTANCE.md`](docs/SIM_ACCEPTANCE.md)
