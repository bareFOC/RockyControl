# Design

## Context

Currently: `WalkQuality` (14 fields, `rk-sim/src/lib.rs:2051`) is serde-free and only reachable via `CopperSimRunner::last_walk_quality()`. `telemetry_json()` (`rk-mcp/tools.rs:196-268`) omits world velocity/position, GRF, slip, flight/duty counts. `McpHost::run_experiment` (`host.rs:117-130`) creates a fresh runner with default gains, returns only `ReplayVerification`. Gain bounds are duplicated between `tools.rs:766-788`, `rk-control/src/lib.rs:524-560`, and test literals. `err_result` returns plain strings.

## Decisions

**Phase 1**
1. Derive `Serialize` on `WalkQuality` (`rk-sim`), add `fn walk_quality(&self) -> Option<WalkQuality> { None }` to `RobotTransport` (`rk-link`) with `SimTransport` returning `Some(runner.last_walk_quality())`; mirror as a default `McpHost::walk_quality()`.
2. Add `quality_snapshot` tool + `rocky://quality` resource in `rk-mcp`.
3. Change `McpHost::run_experiment` to build the fresh runner, install the host's current `BalanceGains` via a `SetBalanceGains` config on packet 0 (without mutating the committed script), run, and return `{verification, quality}`; update tool response and tests.

**Phase 2**
4. Add `WalkQualityBounds` (serde, `Default` = documented acceptance values) and `evaluate(&WalkQuality) -> QualityEvaluation {metrics, bounds, violations, pass}` in `rk-sim`; reuse in `quality_snapshot` and refactor `assert_walking_bound`s in rk-mcp tests and key rk-sim assertions onto it.
5. `balance_reset_defaults` tool (host sets `BalanceGains::default()` via existing `update_balance_gains` path + steps).
6. `experiment_list`/`experiment_run_named` over `tests/experiments/*.json` using `AutomationRoots::read_root`; GUI bridge exposes built-in names via `include_str!` or a static list.
7. Structured errors: add a `tool_error_json` helper returning `{error:{code,field,value,bounds}}`; use it for invalid gain writes while keeping existing text fallback for other errors.

## Risks

- Adding a trait method to `RobotTransport` requires implementing it for all transports (default None covers them).
- Changing `run_experiment` return type breaks existing tests; update all callers in the same change.
- Installing gains into script packet 0 must not alter replay determinism; verify with golden fixtures.

## Verification
`cargo test --workspace`, `cargo run -p xtask -- sim-verify`, live MCP quality/acceptance calls against headless server, WASM check.
