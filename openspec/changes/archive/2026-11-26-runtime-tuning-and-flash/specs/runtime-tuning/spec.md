## ADDED Requirements

### Requirement: Runtime-Configurable UVC Gait, Actuator Limits, and Impact Detection
The wire protocol (`rk-proto`) and control kernel (`rk-control`) SHALL support runtime configuration of UVC gait/swing parameters, per-joint actuator limits, and combat impact detection parameters via `ConfigCmd`, with finite/range validation and `RobotProfile` defaults at boot.

#### Scenario: UVC gait and swing parameters are updated at runtime
- **WHEN** the host sends `ConfigCmd::SetUvcGait(cfg)` with finite values
- **THEN** `DirectServoKernel` validates and clamps each parameter to safe physical bounds and applies them to `UvcController` on the same deterministic tick

#### Scenario: Per-joint actuator limits are updated at runtime
- **WHEN** the host sends `ConfigCmd::SetActuatorLimits { joint, max_torque_nm, max_velocity_rad_s }` for a valid joint index
- **THEN** `DirectServoKernel` clamps `max_torque_nm` and `max_velocity_rad_s` to safe hardware bounds and updates `calibration[joint]`

#### Scenario: Impact detection config is updated at runtime
- **WHEN** the host sends `ConfigCmd::SetImpactConfig(cfg)`
- **THEN** `DirectServoKernel` applies `ImpactConfig::sanitized()` to the active `ImpactDetector`

#### Scenario: Non-finite values in ConfigCmd are rejected
- **WHEN** a `HostToMcu` frame contains `NaN` or `Inf` in `SetUvcGait`, `SetActuatorLimits`, or `SetImpactConfig`
- **THEN** `validate_host_packet` rejects the frame with `CodecError::NonFiniteFloat`

### Requirement: Host Tooling and UI Exposure for Runtime Tuning
`rk-model`, `rk-mcp`, and `rk-ui` SHALL expose the runtime UVC gait, actuator limit, impact config, and flash commit controls to operators and MCP clients.

#### Scenario: MCP tools configure runtime tuning parameters
- **WHEN** an MCP client invokes `uvc_gait_get`, `uvc_gait_set`, `actuator_set_limits`, `impact_get_config`, `impact_set_config`, or `flash_commit`
- **THEN** `RockyMcpTools` updates `RockyStudioModel`, queues the corresponding `ConfigCmd`, and returns the updated configuration

### Requirement: RP2350 QSPI Flash Persistence
The RP2350 firmware (`rk-rp2350`) and shared firmware core (`rk-fw-core`) SHALL persist calibrated `JointCalibration[0..joint_count]`, `BalanceGains`, `UvcGaitConfig`, and `ImpactConfig` to the last sector of RP2350 QSPI flash on `ConfigCmd::CommitFlash` with magic, schema version, and IEEE CRC-32 verification, and restore them at boot before starting `control_task`.

#### Scenario: Blank or corrupt flash falls back to RobotProfile defaults
- **WHEN** the RP2350 boots and the reserved flash sector is erased (`0xFF`) or fails magic/version/CRC-32 verification
- **THEN** firmware falls back cleanly to `RobotProfile` defaults without panicking

#### Scenario: Valid flash config is restored at boot and persisted on CommitFlash
- **WHEN** `ConfigCmd::CommitFlash` is processed by `DirectServoKernel`
- **THEN** `rk-rp2350` writes the CRC32-protected `PersistedConfigBlob` to the reserved QSPI flash sector outside any user critical section, and subsequent boots restore the saved values before starting `control_task`
