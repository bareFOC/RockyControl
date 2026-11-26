## Tasks

- [x] 1. Wire protocol (`rk-proto`): add `UvcGaitConfig`, `ImpactWireConfig`, `ConfigCmd::{SetUvcGait, SetActuatorLimits, SetImpactConfig}`, finite validation, and bump `PROTOCOL_VERSION` to `8`.
- [x] 2. Control kernel (`rk-control`): store and use `UvcGaitConfig` in `UvcController`, handle the new `ConfigCmd` variants in `DirectServoKernel::step`, and expose config getters/setters.
- [x] 3. Host model, MCP, and UI (`rk-model`, `rk-mcp`, `rk-ui`): track new tuning configs in `RockyStudioModel`, add MCP tools (`uvc_gait_get`, `uvc_gait_set`, `actuator_set_limits`, `impact_get_config`, `impact_set_config`, `flash_commit`), and add UI controls.
- [x] 4. Flash persistence (`rk-fw-core`, `rk-rp2350`): implement `PersistedConfigBlob` (magic + version + CRC32) in `rk-fw-core`, boot-time restore before `control_task`, and safe async `CommitFlash` sector write in `rk-rp2350`.
- [x] 5. Regenerate `.copper` acceptance fixtures for `PROTOCOL_VERSION = 8` and run `cargo run -p xtask -- sim-verify` and `cargo run -p xtask -- firmware`.
