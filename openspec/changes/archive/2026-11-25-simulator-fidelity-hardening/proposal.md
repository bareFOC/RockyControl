# Proposal

## Why

RockyControl's simulator shares the exact control kernel and fixed-rate executor with firmware, but several plant/sensor/wire details are wrong or missing. The GRF left/right channels are swapped, velocity commands produce no motion, `post_tick` (LED state machine) never runs in sim, the fixed-step path bypasses COBS wire encoding so wire-error handling is never exercised, there is no clean reproducible "reset experiment", MCP exposes only a subset of fault knobs, and observability lacks the signals (CoM/GRF/contact/tilt-rate) needed to tune walking and balance. These defects make the simulator misleading for controller development even though the kernel path is correct.

## What Changes

- **Fix GRF channel swap** in `rk-sim` telemetry so left/right ground reaction forces map to the correct estimator channels; add a regression test.
- **Honor `JointCommandMode::Velocity`** in the simulator plant using `raw_velocity_rad_s`; add plant + replay tests.
- **Exercise `post_tick` in sim**: implement `SimRobotIo::post_tick` to evaluate the firmware's LED mode state machine (`current_led_mode().is_lit(now_ms)`), expose the flag via `RobotTransport`, and show it in the HUD.
- **Validate the RX wire path in fixed-step sim**: route host commands through `encode_cobs`/`decode_rx` with injectable CRC/version/truncation/fragmentation faults so `rx_crc_errors` and `rx_version_mismatches` are exercised in the app and MCP paths.
- **Add a reset-experiment operation** that resets mock clock, control kernel, plant, faults and UI history together; call it on sim reconnect so tuning runs are reproducible.
- **Expose all `SimFaultInjection` fields through MCP** (`imu_disconnected`, `servo_bus_error`, `external_hand_wave`, `servo_lag_factor`, plus new wire/IMU faults).
- **Add observability**: extend `TelemetryPoint` with achieved CoM x/z, left/right GRF, contact mask and tilt-rate; plot them in the config screen; color per-foot contact/GRF in the viewport.

## Capabilities

### New Capabilities
- `simulator-fidelity`: simulator plant, sensor and wire behavior is faithful enough to validate controller logic and trends without misleading the developer.

### Modified Capabilities
- `persistent-simulation-executor`: gains a reset-experiment operation and the fixed-step wire path honors framing errors.
- `native-mcp-bridge` / `mcp-deterministic-stepping`: full fault-injection surface available through MCP.
- `unified-control-application`: `post_tick` (LED state machine) executes on both firmware and sim paths.

## Impact

Files: `crates/rk-sim/src/lib.rs`, `crates/rk-link/src/lib.rs`, `crates/rk-ui/src/lib.rs`, `crates/rk-ui/src/config_screen.rs`, `crates/rk-ui/src/control_screen.rs`, `crates/rk-viewport/src/lib.rs`, `crates/rk-mcp/src/tools.rs`, `crates/rk-mcp/src/host.rs`, relevant specs. No firmware changes. No upstream bareFOC changes.

Deferred to a follow-up change (medium tier): seeded sensor noise/bias/latency, torque-based actuator dynamics unified with `BenchLoopbackBus`, contact slip/CoP refinement, full-app `step_fixed` end-to-end tests. Large tier (scripted experiment runner, time-origin normalization, targeted multibody effects) remains deferred and is documented in the research report.
