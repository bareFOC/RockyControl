# Rocky Studio (`rk-*`)

Pure Rust humanoid robotics control workstation and `#![no_std]` embedded firmware monorepo built on `egui` + `storytold/craft-libs` + `three-d` + `urdf-rs` + `k` + `copper-rs` + `embassy-rs`.

Rocky Studio runs the exact same workstation UI on **Desktop** (`apps/rk-desktop`), **Browser WASM** (`apps/rk-web`) and **Android** (`apps/rk-android`), paired with deterministic real-time firmware for **RP2350** (`firmware/rk-rp2350`) and **ESP32-P4** (`firmware/rk-esp32p4`).

> [!WARNING]
> **BREAKING telemetry wire-format change (fall-recovery):** `rk_proto::ImuState` now carries additional full-orientation fields (`yaw_rad`, `yaw_rate_rad_s`, `gravity`). Because the `postcard` + COBS telemetry framing has no protocol-version handshake, the host workstation and firmware must be **rebuilt and flashed in lockstep** — a mismatched host/firmware pair will fail to decode telemetry frames. Always update both sides together (and roll back both together).

## Key Capabilities

- **Full-Bleed 3D Mirror Viewport (`rk-viewport`)**:
  - Embedded `three-d` WebGL2/OpenGL renderer sharing `eframe`'s `glow` context.
  - Always-fit-to-view framing across desktop, tablet landscape, and tablet portrait viewports.
  - Interactive 26-region Fusion 360 / Onshape style **ViewCube** (6 cardinal faces, 12 beveled edges, 8 chamfered corners), direct drag-to-orbit on the cube, 90-degree CCW/CW roll arrows, Home isometric snap, XYZ world-axis triad, and 220 ms eased quaternion slerp transitions.
  - Solid PBR robot rendering tracking live `measured_rad`, translucent cyan wireframe target ghost in `Drive` mode, thermal/current heat tinting above 70% rated current, and click/tap joint picking.
- **Bidirectional Safety Architecture (`rk-proto`, `rk-control`, `rk-multibus`)**:
  - Three explicit authority modes: `Teach` (Robot -> GUI, torque off), `Drive` (GUI -> Robot, torque on), and latched `EStop` (`Space` hotkey or top-bar button).
  - Bumpless transfer on every `Teach -> Drive` transition (`target_rad = measured_rad`).
  - Per-joint soft limits, slew-rate clamping, deadman heartbeat watchdog (200 ms), and deterministic Copper replay verification.
- **URDF & MCU Model Pipeline (`rk-model`, `rk-link`)**:
  - Loads `.urdf`, `.xml`, or `.zip` robot bundles (with embedded STL/OBJ meshes) via upload picker, drag-and-drop, XML paste, or chunked CRC32-verified MCU flash pull (`Pull MCU`).
  - Built-in 12-DoF fighting humanoid sample (`assets/sample_humanoid/rocky_mk1.urdf`).
- **Fall Recovery (`rk-ui`, `rk-model`)**:
  - A pulsing red highlight and a top-right **Recovery HUD** panel make falls unmistakable, showing `FALL DETECTED / STABLE`, the classified orientation, and the ordered recovery state.
  - Four pre-computed get-up motions (Prone, Supine, Left Side, Right Side) end in a stable Guard stance.
- **Desktop & Android Tablet Ergonomics (`rk-ui`)**:
  - Slim CAD workstation shell with floating HUD overlays, in-app Fullscreen toggle, live `egui_plot` telemetry charts, `gilrs` gamepad pilot, and automatic Desktop / Tablet Touch layout switching.

## Workspace Layout

### Host & Browser Workspace (`/Cargo.toml`)

| Crate | Layer | Purpose |
| --- | --- | --- |
| `crates/rk-proto` | Layer 0 (`#![no_std]`) | Wire protocol (`postcard` + `cobs` + CRC32), telemetry/command frames, fault bitflags, chunked MCU model transfer |
| `crates/rk-control` | Layer 1 (`#![no_std]`) | Safety kernel: mode state machine, bumpless transfer, soft limits, slew clamping, deadman watchdog |
| `crates/rk-multibus` | Layer 1b (`#![no_std]`) | BareFOC/I3C register-bus client, control-table wire format and bus-health tracking |
| `crates/rk-harness` | Layer 1c | Fixed-dt deterministic Embassy harness shared by simulator and replay |
| `crates/rk-sim` | Layer 2 | Software-in-the-loop (SIL) humanoid plant, fault injection, scripted experiments and deterministic `.copper` replay |
| `crates/rk-model` | Layer 3 | URDF/ZIP ingestion, `k::Chain<f32>` forward kinematics, calibration, move studio, bounds calculation |
| `crates/rk-link` | Layer 3b | `RobotTransport` abstraction: `SimTransport`, native UDP (`10.42.0.2:7777`), and USB CDC-ACM hook |
| `crates/rk-mcp` | Layer 4 | MCP engine (stdio/HTTP) with headless simulator host and tool dispatch |
| `crates/rk-viewport` | Layer 5 | Embedded `three-d` PBR/wireframe viewport, quaternion always-fit camera, and 26-region ViewCube |
| `crates/rk-ui` | Layer 6 | Shared `egui` + `craft-libs` workstation UI, floating CAD HUD, charts, teach/action studio, responsive engine |
| `apps/rk-desktop` | Layer 7 | Native desktop runner (`eframe`, `craft-atomic` saves, `craft-quit`, native file dialog, fullscreen) |
| `apps/rk-web` | Layer 7 | Browser WASM runner (`eframe` WebGL2, `craft-platform`, native launch splash, Web Fullscreen API) |
| `apps/rk-android` | Layer 7 | Android runner (`eframe` glow/native-activity, app-internal `PlatformSaver`, no desktop-only services) |

### Embedded Firmware Workspace (`/firmware/Cargo.toml`)

| Crate | Target | Purpose |
| --- | --- | --- |
| `firmware/rk-fw-core` | `#![no_std]` | Board-agnostic Embassy `ControlTask`, shared control/replay types, frame codec |
| `firmware/rk-rp2350` | `thumbv8m.main-none-eabihf` | RP2350 firmware (`embassy-rp`) with `usb-cdc` (default) and `usb-net` (CDC-NCM + `embassy-net` UDP) |
| `firmware/rk-esp32p4` | `riscv32imafc-unknown-none-elf` | ESP32-P4 firmware (`esp-hal` + `esp-rtos`) |

See [`firmware/README.md`](firmware/README.md) for wiring, pinouts, and flashing details.

## Quick Start

### Prerequisites

- Rust stable toolchain with `wasm32-unknown-unknown` target
- `trunk` (`0.21.14+`) for Browser WASM builds
- Optional for RP2350 cross-checks: `rustup target add thumbv8m.main-none-eabihf`

### Run Browser Workstation (WASM)

```sh
sh scripts/build.sh
trunk serve --port 3000
```

### Run Desktop Workstation (Native)

```sh
cargo run -p rk-desktop --release
```

### Run Tests & Checks

```sh
# Full host + firmware + cross-target + scripted-experiment gate
cargo run -p xtask -- sim-verify   # `check` is an alias

# Individual steps
cargo test --workspace
sh scripts/lint.sh
cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core
cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf
cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --no-default-features --features usb-net,i3c-servo-bus,imu-lsm6dsv
cargo check --manifest-path firmware/rk-rp2350/Cargo.toml --target thumbv8m.main-none-eabihf --no-default-features --features usb-net,bench-loopback
cargo check -p rk-ui --target wasm32-unknown-unknown
```

See [`docs/SIM_ACCEPTANCE.md`](docs/SIM_ACCEPTANCE.md) for the measured
acceptance matrix, committed fixtures and the exact scope of replay verification.
CI mirrors the gate in `.github/workflows/sim-verify.yml`.

## Android APK

The Android app reuses the shared workstation UI, control kernel and simulator unchanged. Build outputs an `aarch64-linux-android` APK for Android 23+; no host PC is required at runtime.

### Prerequisites

- Android SDK **platform 35** and **build-tools 36.0.0** (`sdkmanager "platforms;android-35" "build-tools;36.0.0"`)
- Android NDK **27.1.12297006** (`sdkmanager "ndk;27.1.12297006"`)
- `cargo-apk` **0.10+** (`cargo install cargo-apk --version 0.10.0`)
- JDK 21 (`keytool` is used to create a local debug keystore)
- Rust target: `rustup target add aarch64-linux-android` (the script checks this)

The script defaults to SDK/NDK paths under `$HOME/Android/Sdk`. Override `ANDROID_HOME`, `ANDROID_NDK_ROOT`, `NDK_TOOLCHAIN` and `CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER` if your toolchain lives elsewhere; see the header of `scripts/build-android.sh`.

### Build

```sh
bash scripts/build-android.sh            # release APK (default)
bash scripts/build-android.sh --check    # cross-check only; no linking, no APK
bash scripts/build-android.sh --debug    # dev-profile APK (faster local iteration)
```

The script prints the resulting APK path. With `cargo-apk` 0.10 the release APK is normally written to `target/release/apk/rk-android.apk`.

### Install

```sh
bash scripts/build-android.sh --install        # build, then `adb install -r <apk>`
# or install an already-built APK by hand:
adb install -r target/release/apk/rk-android.apk
```

Use `adb devices` first to confirm a device is connected with USB debugging enabled.

### Limitations

- **No in-app MCP TCP listener.** The MCP bridge is disabled in the APK; connect an external MCP client to a host-side listener instead. Desktop behavior is unchanged.
- **Gamepad input is inert.** `gilrs` has no Android backend; use the touch UI (or an external MCP client) for control.
- **Single simulator session.** One `CopperSimRunner` exists per process and fixed steps run on the UI thread. Reconnect resets the experiment rather than creating a second runner, and simulation time pauses while the activity is suspended.
- **WORLD / GRF telemetry channels are sim-only** and are not available from real hardware.
- **URDF file picker is not configured.** No SAF/JNI picker is wired up yet, so import relies on the bundled `rocky_mk1.urdf` or the drag-and-drop/XML-paste fallback. Exported replay logs are written to the app-internal files directory; no `zenity` subprocess is used.
- **Unsigned APK only.** No release signing or Play Store AAB is configured. `cargo-apk` 0.10 requires a signing key even for local installs, so the script falls back to the standard Android debug keystore (created under `$HOME/.android` if missing). Treat the output as a debug/dev artifact, not a distributable release.

## GitHub Pages Deployment

The included workflow (`.github/workflows/deploy-pages.yml`) automatically builds the release WASM bundle with `trunk build --release --public-url "./"` and publishes `dist/` to GitHub Pages on push to `main` or `master`.

## Fall Recovery — Operator Behavior & Safety Limitations

When a `TILT_FALL_CUTOFF` latch arrives with a valid IMU, the **Recovery HUD** (top-right of the Control screen) shows `FALL DETECTED`, the classified orientation (Prone / Supine / Left Side / Right Side / Unknown), and the current recovery state. The robot's mesh also pulses red while the alarm is active. The alarm is only cleared once the robot is verified upright (torso tilt below the configured fall-cutoff threshold for a sustained dwell) **and** the get-up has completed or been aborted.

Recovery is **strictly opt-in and off by default**:

- **Auto recovery** checkbox is unchecked on startup. Autonomous recovery starts only when it is enabled *and* the operator has checked **Operator confirmed**, and only after the robot is stable upright for the dwell period.
- **Get Up** button is the explicit operator path: it sets `Operator confirmed` and initiates the same recovery sequence even when Auto recovery is off.

Once consent is satisfied, recovery advances one control frame at a time through an ordered sequence that never bypasses the existing safety interlocks:

1. `reset_estop()` queues a single `ClearFaults` command and returns strictly to `Teach` (torque off).
2. The fault must be observed cleared in telemetry before `Drive` is requested.
3. After one full frame in `Drive` with no active fault, walking is disabled and the orientation-matched get-up move is played.
4. On completion the model returns to `Teach` and the alarm clears only after a stable upright stance is confirmed.

**Safety limitations:**

- Get-up motions are **open-loop keyframes** sampled under existing slew-rate and soft-limit clamps. There is no closed-loop IMU balance or ground-contact / friction model during recovery; success on real hardware depends on the surface and initial pose.
- Any new fall latch or E-Stop during playback **aborts immediately**: playback stops, torque stays cut, consent is cleared, and the state re-latches to `FallLatched` with the newly classified orientation.
- A `Drive` request while a fault is latched is refused by `set_authority_mode()`, and `play_move()` is Drive-only and E-Stop-gated; the watcher relies on these gates rather than bypassing them.
- The operator must supervise powered get-up attempts. Keep servo power off (Teach) during initial bench validation and verify the recovery motion in simulation first.
- Because full-orientation telemetry changed the wire layout, host and firmware must be rebuilt and flashed together (see the breaking-change warning above).
