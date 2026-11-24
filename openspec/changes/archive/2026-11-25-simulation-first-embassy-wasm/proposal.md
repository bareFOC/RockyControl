# Proposal

## Why

The app's simulator is a servo-angle rig with synthetic tilt, not a locomotion plant: there is no world-space CoM, no foot contact or ground reaction, the viewport only rotates the robot, and the control loop runs once per egui frame (~60 Hz) instead of the firmware's 200 Hz. As a result "walking" is marching in place. At the same time, the app and firmware have separate control-loop containers. The goal is to make the browser app the primary development target by running the **actual Embassy firmware control application** in WASM with a deterministic clock, backed by a physics plant as the I/O backend — so simulation and hardware share one control path and the hardware build is a backend swap.

## What Changes

- **BREAKING (internal architecture):** Introduce a shared `RobotIo` boundary and a single async `#[embassy_executor::task] control_task` in `rk-fw-core` that runs on both MCU and WASM. Backends: hardware (SPI/I2C/I3C + USB) and simulation (physics plant). The fixed 5 ms `Ticker` + `DirectServoKernel` call becomes the one control path.
- Add a deterministic WASM/native harness using `embassy-time/mock-driver` + `embassy_executor::raw::Executor` with a no-op pender: each simulated tick advances mock time by exactly 5 ms and polls to quiescence. A spike proved this compiles for `wasm32-unknown-unknown`.
- Replace the frame-rate-driven sim step with a fixed-step accumulator in the app (bounded catch-up), so the app runs a true 200 Hz control loop regardless of render FPS; report achieved tick rate honestly.
- Add world-frame state to `SimulatedServoPlant`: CoM x/z position + velocity, per-foot contacts, ground reaction, penetration/slip; derive body motion from leg joint angles via the shared `com_fk`/leg geometry (one source of truth). Replace synthetic walk sway with contact/CoP-driven tilt.
- Translate the viewport root from plant world position so walking is visibly forward motion; wire achieved velocity/displacement/contact into the gait HUD.
- Make walking verifiable end-to-end: sim test that `vx>0` produces forward displacement, alternating contacts, bounded tilt — and that replay of the firmware control task remains bit-exact.
- WASM replay export via `PlatformSaver` and an honest sim-rate indicator.

## Capabilities

### New Capabilities
- `embassy-sim-runtime`: Runs the shared Embassy control application deterministically in WASM/native with a mock clock and a physics I/O backend, while the same task runs on real MCU hardware.
- `locomotion-physics`: World-frame simulation with foot contacts, ground reaction, and body translation that drives both control and 3D visualization.

### Modified Capabilities
- `unified-control-application`: the control application gains a single Embassy task + `RobotIo` boundary shared by firmware and sim.
- `bipedal-walking`: requires end-to-end simulated walking with actual translation and contact.

## Impact

- New/changed: `rk-fw-core` (task + IO trait + harness), `rk-sim` (physics plant), `rk-link` (transport adapter), `rk-ui` (fixed-step driver + HUD), `rk-web` (WASM bootstrap), `rk-rp2350` (hardware IO backend), Cargo manifests ( Embassy features: `mock-driver`, raw executor, no platform feature in harness).
- Risk: WASM+mock-driver has no official example (spike passed); feature unification of `tick-hz-1_000_000`; executor run-queue order must not affect determinism; physics plant must stay deterministic f32.
