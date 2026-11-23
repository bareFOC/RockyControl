# Rocky Studio (`rk-*`)

Pure Rust humanoid robotics control workstation and `#![no_std]` embedded firmware monorepo built on `egui` + `storytold/craft-libs` + `three-d` + `urdf-rs` + `k` + `copper-rs` + `embassy-rs`.

Rocky Studio runs the exact same workstation UI on **Desktop** (`apps/rk-desktop`) and **Browser WASM** (`apps/rk-web`), paired with deterministic real-time firmware for **RP2350** (`firmware/rk-rp2350`) and **ESP32-P4** (`firmware/rk-esp32p4`).

## Key Capabilities

- **Full-Bleed 3D Mirror Viewport (`rk-viewport`)**:
  - Embedded `three-d` WebGL2/OpenGL renderer sharing `eframe`'s `glow` context.
  - Always-fit-to-view framing across desktop, tablet landscape, and tablet portrait viewports.
  - Interactive 26-region Fusion 360 / Onshape style **ViewCube** (6 cardinal faces, 12 beveled edges, 8 chamfered corners), direct drag-to-orbit on the cube, 90-degree CCW/CW roll arrows, Home isometric snap, XYZ world-axis triad, and 220 ms eased quaternion slerp transitions.
  - Solid PBR robot rendering tracking live `measured_rad`, translucent cyan wireframe target ghost in `Drive` mode, thermal/current heat tinting above 70% rated current, and click/tap joint picking.
- **Bidirectional Safety Architecture (`rk-proto`, `rk-control`, `rk-cu-nodes`)**:
  - Three explicit authority modes: `Teach` (Robot -> GUI, torque off), `Drive` (GUI -> Robot, torque on), and latched `EStop` (`Space` hotkey or top-bar button).
  - Bumpless transfer on every `Teach -> Drive` transition (`target_rad = measured_rad`).
  - Per-joint soft limits, slew-rate clamping, deadman heartbeat watchdog (200 ms), and deterministic Copper replay verification.
- **URDF & MCU Model Pipeline (`rk-model`, `rk-link`)**:
  - Loads `.urdf`, `.xml`, or `.zip` robot bundles (with embedded STL/OBJ meshes) via upload picker, drag-and-drop, XML paste, or chunked CRC32-verified MCU flash pull (`Pull MCU`).
  - Built-in 12-DoF fighting humanoid sample (`assets/sample_humanoid/rocky_mk1.urdf`).
- **Desktop & Android Tablet Ergonomics (`rk-ui`)**:
  - Slim CAD workstation shell with floating HUD overlays, in-app Fullscreen toggle, live `egui_plot` telemetry charts, `gilrs` gamepad pilot, and automatic Desktop / Tablet Touch layout switching.

## Workspace Layout

### Host & Browser Workspace (`/Cargo.toml`)

| Crate | Layer | Purpose |
| --- | --- | --- |
| `crates/rk-proto` | Layer 0 (`#![no_std]`) | Wire protocol (`postcard` + `cobs` + CRC32), telemetry/command frames, fault bitflags, chunked MCU model transfer |
| `crates/rk-control` | Layer 1 (`#![no_std]`) | Safety kernel: mode state machine, bumpless transfer, soft limits, slew clamping, deadman watchdog |
| `crates/rk-cu-nodes` | Layer 1b (`#![no_std]`) | `copper-rs` tasks (`ControlTask`, `TelemetrySink`, deterministic plant & actuator drivers) |
| `crates/rk-sim` | Layer 2 | Software-in-the-loop (SIL) humanoid plant, fault injection, and deterministic Copper replay runner |
| `crates/rk-model` | Layer 3 | URDF/ZIP ingestion, `k::Chain<f32>` forward kinematics, calibration, move studio, bounds calculation |
| `crates/rk-link` | Layer 3b | `RobotTransport` abstraction: `SimTransport`, native UDP (`10.42.0.2:7777`), and USB CDC-ACM hook |
| `crates/rk-viewport` | Layer 4 | Embedded `three-d` PBR/wireframe viewport, quaternion always-fit camera, and 26-region ViewCube |
| `crates/rk-ui` | Layer 5 | Shared `egui` + `craft-libs` workstation UI, floating CAD HUD, charts, teach/action studio, responsive engine |
| `apps/rk-desktop` | Layer 6 | Native desktop runner (`eframe`, `craft-atomic` saves, `craft-quit`, native file dialog, fullscreen) |
| `apps/rk-web` | Layer 6 | Browser WASM runner (`eframe` WebGL2, `craft-platform`, native launch splash, Web Fullscreen API) |

### Embedded Firmware Workspace (`/firmware/Cargo.toml`)

| Crate | Target | Purpose |
| --- | --- | --- |
| `firmware/rk-fw-core` | `#![no_std]` | Board-agnostic Embassy + `copper-rs` runtime, UART half-duplex servo bus, frame codec |
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
# Host unit & simulation tests
cargo test -p rk-proto -p rk-control -p rk-cu-nodes -p rk-sim -p rk-model -p rk-link -p rk-viewport -p rk-ui

# no_std core check
sh scripts/lint.sh

# Firmware core unit tests & RP2350 cross-checks (USB CDC + USB-NCM UDP)
cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core
cargo check --manifest-path firmware/Cargo.toml -p rk-rp2350 --target thumbv8m.main-none-eabihf
cargo check --manifest-path firmware/Cargo.toml -p rk-rp2350 --target thumbv8m.main-none-eabihf --no-default-features --features usb-net,bench-loopback
```

## GitHub Pages Deployment

The included workflow (`.github/workflows/deploy-pages.yml`) automatically builds the release WASM bundle with `trunk build --release --public-url "./"` and publishes `dist/` to GitHub Pages on push to `main` or `master`.
