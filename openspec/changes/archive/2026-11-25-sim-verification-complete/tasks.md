# Tasks

> Sequential phases. Each phase: delegate → verify green → check off. Final: archive + sync + commit + push.

## A — gate & source of truth (OWNER: build agent)
- [x] A.1 Repair `xtask/src/main.rs` crate names; add `sim-verify` command. Verify `cargo run -p xtask -- check` and `-- sim-verify`.
- [x] A.2 Add `.github/workflows/sim-verify.yml`; commit `tests/experiments/*.json` + golden `.copper` log; assert exact_match and quality bounds.
- [x] A.3 Write `docs/SIM_ACCEPTANCE.md` with measured matrix + replay-scope caveat; fix README stale names. Verify specs validate.

## B — correctness holes (OWNER: fidelity agent; after A)
- [x] B.1 Align `SimFaultSnapshot::default()` lag with `SimFaultInjection::default()`; cross-type equality test.
- [x] B.2 Clear fall/consent/recovery state in reset_experiment and reconnect; fall→reset→shove e2e test.
- [x] B.3 Implement per-joint bus degradation in sim with shared `MultibusBusTracker`; single-joint degraded test. Verify workspace green.

## C — tool/feature coverage (OWNER: coverage agent; after B)
- [x] C.1 MCP tool behavior tests for untested tools.
- [x] C.2 Live GUI HTTP listener test driving a sim tool.
- [x] C.3 Fixed-step app e2e for running/CBF/impact; side-orientation fall/get-up e2e.
- [x] C.4 Staged MCU model chunked pull e2e with CRC failure. Verify full gate.

## Finish (OWNER: integrator)
- [x] D.1 Full verification: workspace x2, firmware-core, thumb matrix, WASM, Android check, specs strict.
- [x] D.2 Archive + sync OpenSpec; commit and push.
