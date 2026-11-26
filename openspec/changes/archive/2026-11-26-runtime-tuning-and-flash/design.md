# Design: Runtime Tuning and RP2350 Flash Persistence

## Wire Protocol (`rk-proto`)
- `PROTOCOL_VERSION` is bumped from `7` to `8` because `ConfigCmd` gains three new variants.
- `UvcGaitConfig` (all `f32`, `#![no_std]`, `Serialize, Deserialize, Clone, Copy, Debug, PartialEq`):
  - `max_foot_lift: f32` (clamped `[0.005, 0.15]`, profile default `leg_geometry.max_foot_lift`)
  - `max_sway: f32` (clamped `[0.005, 0.12]`, profile default `leg_geometry.max_sway`)
  - `max_dx: f32` (clamped `[0.01, 0.25]`, profile default `leg_geometry.max_dx`)
  - `max_dy: f32` (clamped `[0.01, 0.25]`, profile default `leg_geometry.max_dy`)
  - `max_swing_dx: f32` (clamped `[0.01, 0.30]`, profile default `leg_geometry.max_swing_dx`)
  - `max_swing_dy: f32` (clamped `[0.01, 0.30]`, profile default `leg_geometry.max_swing_dy`)
  - `step_duration_s: f32` (clamped `[0.16, 0.45]`, default `0.18`)
  - `toe_push_off_gain: f32` (clamped `[0.0, 0.60]`, default `0.18`)
  - `waist_balance_gain: f32` (clamped `[0.0, 1.0]`, default `0.35`)
  - `arm_swing_gain: f32` (clamped `[0.0, 2.0]`, default `1.0`)
- `ImpactWireConfig` mirrors `rk_control::impact::ImpactConfig` so `rk-proto` remains dependency-free of `rk-control` and `rk-control` maps cleanly between them.
- `ConfigCmd::SetActuatorLimits { joint: u8, max_torque_nm: f32, max_velocity_rad_s: f32 }` updates `calibration[joint].max_torque_nm` (clamped `[0.05, 10.0]`) and `calibration[joint].max_velocity_rad_s` (clamped `[0.5, 30.0]`), bounded to safe hardware ceilings.

## Control Kernel (`rk-control`)
- `UvcController` stores `gait: UvcGaitConfig` alongside `geom: LegGeometry`. At construction (`new_for_profile`), `UvcGaitConfig::from_leg_geometry(&geom)` seeds the 6 geometry limits from `RobotProfile` and the 4 timing/gain fields from defaults, guaranteeing bit-identical default behavior.
- `DirectServoKernel` exposes `uvc_gait_config()`, `set_uvc_gait_config()`, `set_impact_config()`, `set_balance_gains()`, and `set_calibration_slice()` so `ControlTask` and boot-time flash loading can restore all persisted parameters cleanly before the first tick.

## RP2350 Flash Persistence (`rk-fw-core` + `rk-rp2350`)
- `rk-fw-core::flash_config` defines `PersistedConfigBlob` (with `MAGIC = *b"RKCF"`, `VERSION = 1`, `joint_count: u8`, `calibration: [JointCalibrationWire; MAX_JOINTS]`, `balance_gains: BalanceGains`, `uvc_gait: UvcGaitConfig`, `impact: ImpactWireConfig`) serialized with `postcard` into a fixed buffer `[u8; 4096]` with trailing 4-byte IEEE CRC-32 (`rk_proto::crc32`) and length header.
- On RP2350 (`memory.x` defines 2 MiB flash at `0x1000_0000`, `LENGTH = 2048K`):
  - The last 4 KiB sector (`offset = 2048 * 1024 - 4096 = 0x1FF000`) is reserved for config persistence.
  - At boot in `main()`, before spawning `control_task`, `embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH2)` (or `Blocking` at boot before interrupts/DMA tasks start) reads the sector, verifies magic/version/CRC32, and loads the restored config into a static boot config slot consumed by `ControlTask`. If blank (`0xFF`) or corrupt, it falls back cleanly to `RobotProfile` defaults.
  - During runtime, when `DirectServoKernel` processes `ConfigCmd::CommitFlash` (`flash_commit_count` increments), `HardwareRobotIo::post_tick` signals a dedicated async `flash_writer_task` (via `embassy_sync::signal::Signal`) with a snapshot of the current `PersistedConfigBlob`.
  - `flash_writer_task` executes `blocking_erase` + `blocking_write` (or async flash erase/write) outside any `critical_section::with(...)` block, and feeds/coordinates safely with the Embassy executor without disabling interrupts in user critical sections. (Note: `embassy_rp::flash::Flash::new_blocking` or `Flash::new` internally handles XIP-safe flash RAM trampolines without the caller holding a `critical_section`).
