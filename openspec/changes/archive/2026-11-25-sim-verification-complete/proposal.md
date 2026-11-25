# Proposal

## Why

The software simulation now demonstrates good balance and walking with measured acceptance criteria, bit-exact replay, and scripted experiments — but "verified in simulation" is still a manual claim. There is no CI gate (`xtask` uses stale `humanoid-*` crate names, README names a nonexistent crate), no committed experiment fixtures, no single acceptance document, several untested MCP/UI tool surfaces, and two concrete correctness holes (fault-snapshot default mismatch, stale fall state after reset). Before hardware work begins we need an automated, evidence-backed definition of "simulation done".

## What Changes

- **Phase A — gate & source of truth**: repair `xtask` to use current `rk-*` crates and add a `sim-verify` command; add `.github/workflows/sim-verify.yml` running host tests, firmware-core tests, thumb/WASM checks, committed scripted experiments and replay verification; commit golden `tests/experiments/*.json` scripts and a `.copper` log; write `docs/SIM_ACCEPTANCE.md` with the measured P1–P4 matrix and explicit replay-scope statement; fix README's stale test command and workspace table.
- **Phase B — correctness holes**: align `SimFaultSnapshot::default()` with `SimFaultInjection::default()` (lag 0.20) with a cross-type test; clear fall-watcher/consent state on `reset_experiment()` and reconnect with a fall→reset→shove e2e test; implement per-joint servo bus degradation in sim using the shared `MultibusBusTracker` (one joint degraded while others keep replying).
- **Phase C — tool/feature coverage**: add MCP behavior tests for the untested tools (action playback, velocity/gait configure, telemetry, calibration, URDF/model store, estop reset, IK/joint targets); add a live GUI HTTP MCP listener test driving a sim tool; add fixed-step app e2e covering running gate, CBF and impact through scripted packets; add side-orientation fall/get-up e2e; add staged MCU model + chunked pull e2e with CRC failure.

## Capabilities

### New Capabilities
- `sim-verification-gate`: an automated CI gate plus committed fixtures that prove the simulation's acceptance criteria.

### Modified Capabilities
- `scripted-experiment-runner`: golden scripts and a golden `.copper` log are committed and run in CI.
- `mcp-deterministic-stepping`: live GUI HTTP listener drives sim tools end-to-end.
- `simulator-fidelity`: consistent fault defaults, per-joint bus degradation, clean reset state.
- `fall-detection`, `recovery-motion`: side-orientation and reset-flow e2e coverage.

## Impact

Files: `xtask/src/main.rs`, `.github/workflows/sim-verify.yml` (new), `tests/experiments/*` (new), `docs/SIM_ACCEPTANCE.md` (new), `README.md`, `firmware/rk-fw-core/src/replay.rs`, `crates/rk-sim/src/lib.rs`, `crates/rk-ui/src/{lib.rs,mcp_bridge.rs,e2e_*}`, `crates/rk-mcp/src/{tools.rs,http_server.rs}`, `crates/rk-multibus/src/health.rs`, relevant specs. No control-kernel algorithm changes; no hardware work.

Deferred (nice-to-have): `quality_snapshot` MCP tool/HUD card, realistic noise on by default, uneven ground, backlash/thermal derating, 64B CDC framing e2e. Hardware-only work stays out of scope.
