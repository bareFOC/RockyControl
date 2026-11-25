# Design

## Context

`.copper` `ControlTick.rx_cmd` already holds the exact typed command the kernel consumed (`copper.rs:139`), so the packet stream needs no new format. What is missing are the non-kernel plant events (shoves/faults/recovery/hand/reset) and host-side sequencer state (`StudioModel.tx_seq`, playback, pending config). `reset_experiment` currently preserves the persistent executor but cannot reset the global mock clock; scripts therefore compare kernel-relevant fields (all `ControlTick`s). Time-origin normalization is deliberately deferred because no MCU-side log export exists.

## Goals / Non-Goals

**Goals:** deterministic script capture/replay; one event enum applied at a single source of truth; headless MCP entry point for CI; no change to replay verification semantics.

**Non-Goals:** MCU log export; cross-path time normalization; changing the control kernel; wall-clock-driven scripts.

## Decisions

1. **Format**: extend `CopperReplayLog` with `#[serde(default)] seed: u64` and `#[serde(default)] events: Vec<TimedSimEvent>`. `TimedSimEvent { tick_index, kind }` where kind = `SetFaults(SimFaultInjection)` | `Shove{pitch,roll}` | `Recovery(Option<f32>)` | `ResetUpright` | `VirtualHand{joint,rad}`. No separate host-packet format — reuse `ticks[].rx_cmd`.
2. **Capture point**: add an optional event recorder to `CopperSimRunner`; the existing transport hooks (`apply_push_impulse`, `sim_faults_mut` writes, `set_recovery_progress`, `reset_upright`, `set_virtual_hand_angle`) record events when recording is active. `sim_faults_mut` returning `&mut` can be wrapped with a recording proxy or events captured on the next `step_harness` by diffing — prefer a small `record_event` call at each hook (explicit, deterministic).
3. **Runner**: `CopperSimRunner::run_script(&ExperimentScript) -> CopperReplayLog`: `reset_experiment()`, then for each tick apply events scheduled for that index, drive `step_harness(1, &[packet])` (or batch contiguous ticks), record ticks. Make `max_recorded_ticks` configurable on the runner/script.
4. **Host reset**: on `reset_experiment`, clear `StudioModel.tx_seq` (or set to the script's starting seq), action playback and pending config; scripts always feed explicit typed packets, never `build_next_host_packet`.
5. **MCP**: `run_experiment` tool in `rk-mcp`, declared in `schema.rs`, taking a script JSON (or path via confined read) and returning `ReplayVerification`. Works in headless and GUI hosts.
6. **Verification**: unchanged `verify_deterministic_replay`. Add `run_experiment` tests: shove@40 + fault@80 run twice → identical ticks and exact replay; old v1 log still verifies.

## Risks / Trade-offs

- `reset_experiment` cannot reset the mock clock (ticker alarm registration), so `ControlTick.now_ms` keeps advancing between runs; scripts compare recorded ticks with their own origin (self-consistent verification), not absolute timestamps.
- Event capture through `&mut SimFaultInjection` is easy to miss; all mutation paths must call `record_event`. Keep the event enum small and add a debug assertion that no fault change occurs during recording without an event.
- Large scripts may exceed the 2400-tick recorder cap; make it configurable and document memory bounds.
