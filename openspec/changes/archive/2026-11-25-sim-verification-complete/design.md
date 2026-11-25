# Design

## Context

Findings: `xtask/src/main.rs` references stale `humanoid-*` crates; no CI workflow runs sim verification; README names a nonexistent `rk-cu-nodes`; no committed experiment fixtures or acceptance doc; `SimFaultSnapshot::default().servo_lag_factor=0.12` vs `SimFaultInjection::default()=0.20`; reset leaves fall/consent state; `SimMultibusPio` has one global bus-error flag and `MultibusBusTracker` is unused in sim; only 8/33 MCP tools have behavior tests; `WalkQuality` is not exposed to UI/MCP (nice-to-have, deferred).

## Goals / Non-Goals

**Goals:** automated gate with committed evidence; fix concrete correctness holes; e2e coverage for untested surfaces; no kernel algorithm changes.

**Non-Goals:** hardware bring-up; quality-metrics UI/MCP exposure; default realistic noise; uneven ground; backlash/thermal; 64B CDC e2e.

## Decisions by phase

**A.** Rewrite `xtask/src/main.rs` steps with `rk-*` names; add `sim-verify` command running host tests + firmware-core + thumb feature matrix + WASM check + scripted experiments via the MCP headless path. Add `.github/workflows/sim-verify.yml` mirroring it. Commit `tests/experiments/walk_nominal.json`, `walk_shove.json`, `fall_recover.json` and a golden `.copper` log; gate asserts `exact_match` and quality bounds. Write `docs/SIM_ACCEPTANCE.md` with the P1–P4 measured matrix and replay-scope caveat. Fix README crate names.

**B.** Set `SimFaultSnapshot::default().servo_lag_factor = 0.20` (`firmware/rk-fw-core/src/replay.rs`) and add a cross-type equality test in rk-sim. Clear `fall_event_active`, `recovery_state`, `fall_orientation`, `operator_confirmed`, `fall_dwell_sec`, `saved_recovery_gains`, `get_up_requested` in `reset_experiment` and reconnect (`rk-ui`); add fall→reset→shove e2e. Add per-joint fault knobs to `SimMultibusPio`: `degraded_joints: [bool; MAX_JOINTS]` (serde default) making that node's transactions fail while others reply; wire `MultibusBusTracker` for timeout/degraded accounting and expose per-joint status in feedback.

**C.** Add MCP tool behavior tests in `rk-mcp/src/tools.rs` (action, gait, telemetry, calibration, URDF/model, estop, IK/joints), a live GUI HTTP listener test driving `run_experiment`/`sim_inject_fault`, fixed-step app e2e for running gate + CBF + impact via typed packets, side-orientation fall/get-up e2e, and staged-model chunked pull with CRC-failure retry. All tests deterministic and gated by existing TEST_LOCKs.

## Risks / Trade-offs

- New per-joint fault knobs change serde/log formats; use `#[serde(default)]` and keep defaults all-healthy.
- MCP tool tests may need small host helpers to construct typed packets; keep them in test modules.
- CI job must install Rust targets; no NDK needed for `cargo check`.
