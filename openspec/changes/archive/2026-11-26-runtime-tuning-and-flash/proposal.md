# Change: Runtime Tuning and RP2350 Flash Persistence (`runtime-tuning-and-flash`)

## Why
Currently, several key locomotion, actuator, and impact-detection parameters in `rk-control` (`UvcController` gait/swing limits and gains, per-joint `max_torque_nm` / `max_velocity_rad_s`, and `ImpactConfig`) are compile-time constants or only settable at construction time from `RobotProfile`. Additionally, `ConfigCmd::CommitFlash` only increments a counter in `DirectServoKernel` without persisting calibrated `JointCalibration`, `BalanceGains`, UVC gait config, or `ImpactConfig` to RP2350 QSPI flash across power cycles.

## What Changes
1. **Wire Protocol (`rk-proto`):**
   - Add `UvcGaitConfig` (covering `max_foot_lift`, `max_sway`, `max_dx`, `max_dy`, `max_swing_dx`, `max_swing_dy`, `step_duration_s`, `toe_push_off_gain`, `waist_balance_gain`, `arm_swing_gain`) and `ImpactWireConfig` (`torque_capable`, `k_o`, `threshold_nm`, `debounce_ticks`, `refractory_s`, `current_spike_ma`, `ma_per_nm`) to `rk-proto`.
   - Extend `ConfigCmd` with:
     - `ConfigCmd::SetUvcGait(UvcGaitConfig)`
     - `ConfigCmd::SetActuatorLimits { joint: u8, max_torque_nm: f32, max_velocity_rad_s: f32 }`
     - `ConfigCmd::SetImpactConfig(ImpactWireConfig)`
   - Validate all new `ConfigCmd` float fields in `validate_host_packet`.
   - Bump `PROTOCOL_VERSION` from `7` to `8`.
2. **Control Kernel (`rk-control`):**
   - Store `UvcGaitConfig` inside `UvcController` (initialized from `RobotProfile::leg_geometry` and default gains at boot) and use its fields in UVC stepping, catch-step timing, toe push-off, waist balance assist, and arm swing.
   - Apply `ConfigCmd::SetUvcGait`, `ConfigCmd::SetActuatorLimits` (safely clamped to hardware bounds), and `ConfigCmd::SetImpactConfig` (sanitized via `ImpactConfig::sanitized()`) in `DirectServoKernel::step`.
   - Expose getters/setters on `DirectServoKernel` and `ControlTask` for reading/restoring persisted configuration.
3. **Host Tooling (`rk-model`, `rk-mcp`, `rk-ui`):**
   - Track UVC gait config, actuator limits, and impact config in `RockyStudioModel` (`rk-model`).
   - Expose MCP tools (`uvc_gait_get`, `uvc_gait_set`, `actuator_set_limits`, `impact_get_config`, `impact_set_config`, `flash_commit`) in `rk-mcp`, and tuning controls in `rk-ui`.
4. **RP2350 Flash Persistence (`rk-rp2350` + `rk-fw-core`):**
   - Define a versioned, magic-tagged, CRC32-verified `#![no_std]` flash blob format (`PersistedConfigBlob`) in `rk-fw-core` storing `JointCalibration[0..MAX_JOINTS]`, `BalanceGains`, `UvcGaitConfig`, and `ImpactWireConfig`.
   - In `rk-rp2350`, load persisted config from the last 4 KiB sector of RP2350 QSPI flash at boot before starting `control_task` (falling back cleanly to `RobotProfile` defaults if blank/corrupt).
   - Wire `ConfigCmd::CommitFlash` via an async flash-commit signal/request outside any critical section (so interrupts and USB/control tasks remain Embassy-safe and deterministic) to erase and write the config sector.

## Impact
- `crates/rk-proto`: `PROTOCOL_VERSION` bumped to `8`, new `UvcGaitConfig`, `ImpactWireConfig`, and `ConfigCmd` variants.
- `crates/rk-control`: Runtime-configurable UVC gait, actuator limits, and impact config; deterministic `rk-sim` parity preserved.
- `crates/rk-model`, `crates/rk-mcp`, `crates/rk-ui`: Host state, MCP tools, and UI controls for runtime tuning and flash commit.
- `firmware/rk-fw-core`, `firmware/rk-rp2350`: Versioned CRC32 flash blob and async RP2350 QSPI flash persistence.
- `tests/experiments/*.copper`: Acceptance fixtures regenerated for `PROTOCOL_VERSION = 8`.
