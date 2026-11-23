# Proposal

## Why

Rocky Mk1 currently detects a fall (IMU tilt breach) only to hard-latch E-Stop, cut torque, and leave the robot on the ground with no recovery path — the only "stand up" is a simulation-only teleport (`reset_upright()`). Operators have no way to see a fall clearly in the 3D viewer or to bring the robot back to its feet, autonomously or otherwise. This change adds a precise fall-orientation signal, an orientation-specific get-up keyframe library, a pulsing red visual alarm, and a consent-gated autonomous recovery flow.

## What Changes

- **BREAKING** (wire format): Extend `ImuState` in `rk-proto` to carry full orientation — heading/yaw plus a gravity vector — so falls can be classified as prone, supine, left-side, or right-side rather than inferred from 2-axis tilt sign alone. Host and firmware must be rebuilt/flashed in lockstep; there is no protocol version handshake. Frame size remains under `MAX_COBS_FRAME_BYTES = 512`.
- Update the BNO055 driver to decode the already-read heading and gyro-Z plus read the gravity-vector registers; thread new fields through the firmware IMU slot, Copper message flattening, sim plant (synthesized gravity), kinematics (world-Y yaw), protocol validation, and all struct literals/tests.
- Introduce a `FallOrientation` classifier (`Prone`, `Supine`, `SideLeft`, `SideRight`) derived from the gravity vector, used to select the correct get-up motion.
- Add a recovery keyframe library to the Keyframe Action Studio: four pre-computed multi-keyframe `ActionMove`s (one per fall orientation) authored with the existing joint-name `pose()` helper, appended in both `new_default()` and `load_bundle()` so they survive URDF reloads. MCP action lookup is already dynamic.
- Add a pulsing red highlight to the 3D viewport driven by a `fall_event_active` flag on `ViewportOptions` and a monotonic time source (`ui.ctx().input(|i| i.time)`); highest-priority override of mesh albedo/emissive. Background already darkens on E-Stop.
- Add a dedicated Recovery panel (top-right HUD) in the control screen showing fall status, detected orientation, an **Auto Recovery** toggle (default OFF), and an operator **Get Up** button. MCP schema descriptions updated to mention recovery moves.
- Add an app-level fall watcher that latches the fall event, requires a tilt-below-threshold dwell, and — only when auto-recovery is opted in **and** operator-confirmed — performs the legally required multi-frame sequence `reset_estop()` → Teach → (fault clear) → `set_authority_mode(Drive)` → `play_move(getup_idx)`. All existing safety gates remain intact.

## Capabilities

### New Capabilities
- `fall-detection`: Full-orientation IMU acquisition and classification of fall events into prone/supine/left-side/right-side, including the protocol and sensor plumbing needed to observe orientation precisely.
- `recovery-motion`: Pre-computed get-up keyframe sequences per fall orientation in the Keyframe Action Studio, with autonomous, consent-gated triggering and a recovery state machine.
- `fall-visual-alarm`: Visual alarm state in the 3D viewer — pulsing red robot highlight on a detected fall event, plus the recovery HUD panel that exposes status and controls.

### Modified Capabilities
<!-- None: there are no existing specs in this project. -->

## Impact

- **Protocol/wire**: `rk-proto` `ImuState` gains new fields; `decode_mcu_cobs` validation extended; no version/CRC handshake exists, so host + both MCUs must be updated together.
- **Firmware**: `firmware/rk-rp2350/src/imu_bno055.rs` (new register reads), `firmware/rk-fw-core/src/{imu.rs, copper.rs, config.rs, status_led.rs as needed}`, `firmware/rk-rp2350/src/servo_uart.rs` fallback.
- **Host crates**: `rk-control` (gravity-aware fall classification input), `rk-sim` (synthesized gravity/yaw), `rk-model` (recovery library + `KinematicRig` pose helper and yaw-aware transforms), `rk-viewport` (pulse flag/time), `rk-ui` (fall watcher + Recovery panel), `rk-mcp` (schema descriptions).
- **Dependencies**: None new. BNO055 registers `0x2E` (gravity) / `0x20` (quaternion) and existing `0x1A` heading / `0x14` gyro are used.
- **Safety**: Recovery flow preserves all existing E-Stop, watchdog, `TILT_FALL_CUTOFF`, `McpPolicy.drive_armed`, and operator-confirm gates. Autonomous recovery is opt-in and off by default.
