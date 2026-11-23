# Design

## Context

See `proposal.md` for motivation and the three spec deltas for required behavior. Current relevant state of the codebase:

- `rk-proto::ImuState` (`crates/rk-proto/src/lib.rs`) carries only `available`, `pitch_rad`, `roll_rad`, `pitch_rate_rad_s`, `roll_rate_rad_s`. Telemetry is serialized with `postcard` + COBS (`MAX_COBS_FRAME_BYTES = 512`) with no protocol version or handshake. `MODEL_SCHEMA_VERSION` and `crc32` cover only the URDF model transfer, not telemetry.
- The BNO055 driver (`firmware/rk-rp2350/src/imu_bno055.rs`) already reads Euler (`0x1A`, Heading/Roll/Pitch) and gyro (`0x14`, X/Y/Z) in two 6-byte bursts but discards heading and gyro-Z. Gravity vector is available at register `0x2E` (page 0, 6 bytes, 1 LSB = 0.01 m/s²); quaternion at `0x20`.
- Fall detection already exists in `DirectServoKernel::step` (`rk-control/src/lib.rs`) and `UvcController::step` (`rk-control/src/uvc.rs`) via `|pitch|/|roll| >= fall_cutoff_rad` (default 0.65 rad), followed by a 40 ms `FallTuck` and an E-Stop latch with `FaultFlags::TILT_FALL_CUTOFF`. Sign of tilt is discarded (`.abs()`).
- The action studio (`rk-model/src/studio.rs`) has `Keyframe`/`ActionMove` with cubic-smoothstep interpolation, a local `pose()` helper that clamps joint overrides to URDF limits, and exactly five combat moves. `play_move()` is Drive-only and E-Stop-gated. `load_bundle()` regenerates the library. The bundled URDF has 21 joints; `MAX_JOINTS = 24`.
- Safety gates: `is_estop_active()` includes `TILT_FALL_CUTOFF`; `reset_estop()` is operator-only and returns to Teach with a queued `ConfigCmd::ClearFaults` (one per packet); `set_authority_mode()` refuses non-EStop while E-Stop active; `build_next_host_packet()` forces wire E-Stop and zero torque mask while faulted; `ingest_telemetry()` re-latches E-Stop. MCP `McpPolicy.drive_armed` defaults false and is cleared on E-Stop.
- `rk-viewport` `ViewportOptions` has `estop_active` and `imu_available/pitch/roll` but no fall flag; mesh materials are per-item (`PhysicalMaterial` with public `albedo`/`emissive`), and heat-tinting already writes them each frame. There is no monotonic time in the render callback, but `ui.ctx().input(|i| i.time)` is available inside `show()`.
- The control screen (`rk-ui/src/control_screen.rs`) draws three HUD panels into `viewport_hud_rects[0..2]` of a 5-slot array; the top-right corner is free.

Constraints: Rust edition 2024, `no_std` for proto/control/firmware crates, deterministic `f32` numerics, strict safety invariants. Firmware uses `embassy` + `copper-rs` task graph with flattened `Encode/Decode/Reflect` message structs.

## Goals / Non-Goals

**Goals:**
- Carry full orientation (yaw + gravity vector) end-to-end from sensor/sim to host with validation.
- Classify falls into four orientations and select a matching pre-computed get-up motion.
- Make falls visually unmistakable via a pulsing red robot highlight and a dedicated Recovery HUD panel.
- Provide opt-in, operator-confirmed autonomous recovery that respects all existing safety gates and the multi-frame reset sequence.
- Keep tasks decomposable into subagent-delegable units with clear file ownership.

**Non-Goals:**
- Closed-loop, IMU-driven dynamic balance during the get-up (recovery motions are open-loop keyframe playback under existing slew/soft-limit clamps).
- MCU-side storage or execution of recovery moves (moves remain host-side in `StudioModel`).
- Adding new external dependencies or changing the Copper task graph topology.
- Changing the existing fall detection thresholds or the protective `FallTuck` behavior.
- Full 3D ground-contact/coefficient-of-friction simulation.

## Decisions

### 1. Extend `ImuState` rather than infer orientation from 2-axis tilt
Add `yaw_rad`, `yaw_rate_rad_s`, and `gravity: [f32; 3]` to `ImuState`. Rationale: the current 2-axis tilt is sign-preserving in sim but clamped at ~77°, so prone vs supine and left vs right cannot be reliably distinguished; the BNO055 already exposes heading/gyro-Z and gravity registers. A gravity vector gives an unambiguous world-down reference for classification. Quaternion was considered but rejected: Euler + gravity is sufficient for classification and integrates with the existing `compute_link_transforms_with_tilt` which builds rotations from Euler axes; quaternion would require a sensor-frame-to-robot-frame remap.

### 2. Sim-only improvements to gravity, yaw in simulation
The LIPM plant tracks only pitch/roll. Add a `yaw_rad` field (driven by a small accumulated synthetic sway, zero on reset) and compute the gravity vector by rotating world-down `(0,-9.81,0)` by the inverse of the same tilt quaternion used in kinematics. This guarantees rendered orientation and sensor reading agree. No wall-clock use (sim avoids `std::time::Instant`).

### 3. Classification from gravity vector, not tilt sign
Classify by rotating the gravity vector into the body frame and comparing the dominant horizontal component: forward → Prone, backward → Supine, lateral sign → Left/Right. If IMU unavailable or gravity magnitude invalid, record `Unknown`. This keeps the classifier independent of the 0.65 rad fall threshold and reusable for other events.

### 4. Recovery library via `default_recovery_library(&rig)`, appended in both constructors
Factor the `pose()` closure into a reusable `KinematicRig::pose_from_overrides()` method. Author four `ActionMove`s (one per orientation), each: tucked/fallen start → roll/push-off → kneel → stand → Guard. Append to `moves` in both `new_default()` and `load_bundle()` to survive URDF reload. Reuse existing `ActionMove`/`Keyframe` serde types and `sample_action_move` interpolation — no new protocol fields for actions. MCP lookup is already dynamic; only schema descriptions change.

### 5. Multi-frame, consent-gated recovery sequence
Introduce a small `RecoveryState` state machine in `RockyStudioApp` (`Idle`, `FallLatched`, `ClearingFault`, `ArmingDrive`, `PlayingGetUp`, `Completed`) and a fall watcher in `update_frame` after `ingest_telemetry()`. Transitions advance one control frame at a time: `reset_estop()` (queues ClearFaults, Teach) → wait for fault clear in telemetry → `set_authority_mode(Drive)` → wait one frame → `walk_cmd.walk_enabled = false` + `play_move(idx)` → on done/re-fall return to Idle/Teach. Autonomous start requires both `auto_recovery_enabled` and an operator confirmation flag; the Get Up button provides explicit operator initiation. Rationale: `ClearFaults` is one-per-packet and `is_estop_active()` includes `TILT_FALL_CUTOFF`, so a single-frame bypass would fail or violate safety. Alternative considered — adding `ControlMode::Recovering` and a new MCU `UvcPhase::GetUp` — was rejected as a much larger protocol/firmware change unnecessary for a first implementation.

### 6. Pulse time from egui context, no viewport signature change
Inside `HumanoidViewport::show()`, read `ui.ctx().input(|i| i.time)` and store it in `ViewportFrameData.time_sec`. Add `fall_event_active: bool` to `ViewportOptions` and frame data. In `SceneResources::render()` tint block, compute `pulse = 0.5 + 0.5*sin(time_sec * TAU * 2.0)` and set `albedo`/`emissive` red as the highest-priority branch before heat/selection tints. Rationale: no timestamp exists in the GL callback; egui's monotonic time avoids threading a new argument through every viewport call site. Both viewport call sites populate the flag from `TILT_FALL_CUTOFF && imu.available`.

### 7. Recovery panel as a fourth HUD panel in the control screen
Add a top-right HUD panel using the spare `viewport_hud_rects[3]` slot, following the existing child-`Ui` pattern. Shows fall status, detected orientation, `auto_recovery_enabled` checkbox (default false), and Get Up button. Do not add a new `AppScreen` variant, avoiding changes to screen routing/top bar/status ribbon.

## Risks / Trade-offs

- [Wire-format breakage between host and firmware] → Restore with lockstep rebuild/flash; document in README and proposal as **BREAKING**. Consider an optional future telemetry `protocol_version` field as a separate change.
- [Open-loop get-up may fail on real hardware with unknown ground contact] → Keep auto recovery opt-in/off by default; require operator confirmation; rely on existing slew clamps and soft limits; abort on any re-latched E-Stop; document limitation.
- [Gravity vector register `0x2E` on BNO055 page 0] → Verify against datasheet during implementation; if unavailable, fall back to quaternion `0x20` and derive gravity, or to Euler-based synthesis. Keep register constants explicit.
- [Frame budget growth] → New fields add ~28 bytes; existing frame is well under 512 bytes; assert frame length in tests.
- [HUD layout crowding] → Top-right corner is currently free; use a compact panel and the existing 5-slot rect array; collapse when the viewport is small using existing responsive helpers.
- [Existing `17 DoF` stale label vs 21-joint URDF] → Author poses by joint name using the 21-joint rig; do not hardcode indices; update UI label opportunistically but not as a requirement.

## Migration Plan

1. Land protocol + firmware + sim changes together (Phase 1) — both MCU targets and host must be rebuilt/flashed in lockstep.
2. Land recovery library (Phase 2), visual alarm (Phase 3), then recovery panel/watcher (Phase 4) as host-only changes.
3. Validate in simulation first (push tests, fall classification, get-up playback), then on bench hardware with servo power off (Teach), then with powered servos under operator supervision.
4. Rollback: revert the commit set and reflash prior firmware; because telemetry layout changed, host and firmware versions must be rolled back together.
