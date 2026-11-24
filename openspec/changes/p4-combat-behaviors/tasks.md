# Tasks

> Subagent-delegable. No back-compat needed.

## 1. Combos and balance tails (OWNER: action-studio agent)
- [ ] 1.1 Add combo ActionMoves (chain Guard/Jab/Cross/Kick + Guard) with balance-restore tails in `rk-model/src/studio.rs`; verify `cargo test -p rk-model`.
- [ ] 1.2 Add impact/fall abort in `ingest_telemetry` handing off to recovery; verify `cargo test -p rk-model -p rk-ui`.

## 2. Offline optimizer (OWNER: tools agent; parallel)
- [ ] 2.1 Add host-side offline impact-aware strike keyframe optimizer (momentum/impulse QP, export ActionMove data). Verify it produces playable keyframes + unit tests.
- [ ] 2.2 Wire optimizer output into Action Studio; verify import.

## 3. Hardware-gated impact detection (OWNER: control agent; parallel)
- [ ] 3.1 Momentum observer (λ̂=0, backward Euler, 0.1–0.3 N·m threshold) gated to torque builds; v1 current-spike heuristic. Verify `cargo test -p rk-control`.

## 4. Validation (OWNER: integrator; depends on all)
- [ ] 4.1 Replay/shove/fall suites + new combo tests; full host suite + firmware-core + thumb checks + lint + strict validate.
- [ ] 4.2 Document v1/v2 scope and deferred learned policy.
