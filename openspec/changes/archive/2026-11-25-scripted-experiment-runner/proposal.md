# Proposal

## Why

Tuning experiments in the app are currently driven by wall-clock-paced host packets and manually injected shoves/faults, so "I tuned it in the app" is not reproducible in CI. The `.copper` replay log already captures the exact per-tick kernel I/O trace (`ControlTick.rx_cmd`, `wire_errors`, `tx_telem`) and replays bit-exactly, but it does not capture non-kernel events (shoves, fault toggles, recovery progress, virtual hand, reset upright), so it cannot re-drive the plant. This change adds a deterministic scripted experiment runner that records those events alongside the command stream and replays them from a clean reset, exposed via MCP for CI regression.

## What Changes

- Extend `CopperReplayLog` with an optional `events: Vec<TimedSimEvent>` side-channel and a `seed` field (the simulator now has a seeded PRNG). `TimedSimEvent` covers fault changes, shoves, recovery progress, reset-upright and virtual-hand events at a given tick index.
- Capture events at the point they are applied inside `rk-sim` (single source of truth), mirroring how `record_tick` is wired through `RobotIo`.
- Add `CopperSimRunner::run_script(&ExperimentScript) -> CopperReplayLog` that resets the experiment, walks events by tick index, drives `step_harness` with typed packets, and returns a replayable log.
- Add a `run_experiment` MCP tool that takes a JSON script (via the existing confined-file read automation), runs it headlessly, and returns `ReplayVerification`.
- Fix reproducibility holes: reset `StudioModel.tx_seq`/playback/pending config on experiment reset so scripts feed explicit packets rather than frame-derived ones; make `max_recorded_ticks` configurable for long scripts.
- Do **not** change `verify_deterministic_replay` semantics; cross-path time-origin normalization is deliberately deferred until MCU-side replay export exists.

## Capabilities

### New Capabilities
- `scripted-experiment-runner`: record and replay a deterministic experiment (host packet stream + plant events + seed) from a clean reset.

### Modified Capabilities
- `mcp-deterministic-stepping`: gains a `run_experiment` tool for headless CI regression.
- `simulator-fidelity`: gains reproducible reset semantics for host-side command sequencing.

## Impact

Files: `firmware/rk-fw-core/src/replay.rs` + `copper.rs` (additive serde fields), `crates/rk-sim/src/lib.rs`, `crates/rk-mcp/src/{tools.rs,schema.rs,host.rs,headless.rs}`, `crates/rk-ui/src/mcp_bridge.rs`, possibly `crates/rk-model/src/studio.rs` for seq reset. No firmware control-kernel changes; no upstream bareFOC changes. Time-origin normalization is explicitly out of scope.

Deferred: MCU-side `.copper` export (prerequisite for cross-path time normalization), HIL validation.
