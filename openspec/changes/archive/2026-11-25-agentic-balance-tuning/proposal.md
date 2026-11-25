# Proposal

## Why

An AI agent can already drive the simulator over MCP, but it cannot run an acceptance-equivalent tuning loop: ~5 of the 14 `WalkQuality` metrics are not exposed anywhere, `run_experiment` returns only a replay verdict while ignoring gains set by `balance_set_gains`, there is no reset-to-defaults or named experiment runner, and acceptance bounds are duplicated test literals. This change closes the feedback loop so an agent can autonomously test, tune and verify balance/walking.

## What Changes

- **Phase 1 — feedback loop**: expose `WalkQuality` (all 14 metrics) through `RobotTransport` and a `quality_snapshot` MCP tool/resource; include quality in `run_experiment` responses and make scripted runs use the currently tuned gains (inject `SetBalanceGains` on tick 0).
- **Phase 2 — acceptance & recovery primitives**: add a machine-readable `WalkQualityBounds`/`evaluate()` in `rk-sim` and reuse it in existing tests and the new tool so `{metrics, bounds, violations, pass}` is one response; add `balance_reset_defaults` and `experiment_list`/`experiment_run_named` over the committed fixtures; return structured JSON errors with field/bounds for invalid gain calls.

## Capabilities

### New Capabilities
- `agentic-balance-tuning`: an MCP agent can read full walk quality, evaluate acceptance bounds, reset defaults, and run named experiments deterministically.

### Modified Capabilities
- `scripted-experiment-runner`: `run_experiment` uses tuned gains and reports quality.
- `mcp-deterministic-stepping`: adds quality/acceptance tooling.
- `simulator-fidelity`: `WalkQuality` becomes serde-serializable and bounds are a single source of truth.

## Impact

Files: `crates/rk-sim/src/lib.rs`, `crates/rk-link/src/lib.rs`, `crates/rk-mcp/src/{host.rs,schema.rs,tools.rs,headless.rs}`, `crates/rk-model/src/studio.rs` (reset gains helper if needed), possibly `crates/rk-ui/src/mcp_bridge.rs`. No kernel algorithm changes; no firmware changes.

Deferred: batch `sweep_gains`/`xtask agent-verify --json` (phase 3), sim-only gain guard/auto-revert/preset export/HUD card (phase 4).
