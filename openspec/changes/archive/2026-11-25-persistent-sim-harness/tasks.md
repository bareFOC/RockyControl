# Tasks

- [x] 1.1 Refactor `rk-sim::SimRobotIo`/`CopperSimRunner` to own a persistent harness + task (leaked boxes), spawned once. Verify `cargo test -p rk-sim`.
- [x] 1.2 Change `rk-link::SimTransport::step_fixed_inner` to drive the persistent harness (queue cmd, advance, drain telemetry) instead of calling `run_harness_continuous` per call. Verify `cargo test -p rk-link -p rk-sim`.
- [x] 1.3 Add test: many sequential `step_fixed` calls in one process (no resets) → no panic, tick indices monotonic, replay exact. Verify `cargo test -p rk-link -p rk-mcp`.
- [x] 1.4 Full verification + remove now-unneeded test clock-reset workarounds where safe.
