# Tasks

> Delegable. Group 1 first; Group 2 after; Group 3 integrates.

## 1. Format + capture + runner (OWNER: sim agent; FIRST)
- [x] 1.1 Extend `CopperReplayLog` with `#[serde(default)] seed` + `events: Vec<TimedSimEvent>` in `firmware/rk-fw-core/src/replay.rs`; define event enum; verify old logs parse. Verify `cargo test --manifest-path firmware/Cargo.toml -p rk-fw-core`.
- [x] 1.2 Add event capture to `CopperSimRunner` in `crates/rk-sim/src/lib.rs` at all plant-event hooks; add `run_script`; make recorder cap configurable. Verify two identical script runs produce identical ticks + exact replay. Verify `cargo test -p rk-sim`.

## 2. MCP + host reset (OWNER: mcp agent; depends on 1)
- [x] 2.1 Add `run_experiment` tool + schema + host method; headless and GUI bridge implementations. Verify `cargo test -p rk-mcp`.
- [x] 2.2 Reset `StudioModel.tx_seq`/playback/pending config on experiment reset so seq is script-driven. Verify `cargo test -p rk-ui`.

## 3. Verification (OWNER: integrator; depends on all)
- [x] 3.1 Full host suite, firmware-core, thumb, WASM, lint, strict validate; update status.
