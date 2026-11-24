# Design

## Context

`rk-model/src/studio.rs` already has scripted strikes (Guard/Jab/Cross/Kick) and get-up moves. P1/P2 give an accurate capture point and reactive balance. This stage adds combos and balance tails (v1-feasible) and offline strike optimization (PC). Forceful momentum-observer control is v2-gated; learned residuals are deferred.

## Goals / Non-Goals

**Goals:** combos, balance-restore tails, impact abort, offline optimizer export, v2-gated impact detection. **Non-Goals:** online QP, learned MLP, vision/opponent sensing, v2 hardware bring-up.

## Decisions

1. Combos as `ActionMove`s in `default_combat_library()` chaining existing strikes + Guard; balance tail appended as extra keyframes that drive torso/arms toward a capture-safe pose.
2. Abort on fall/EStop via `ingest_telemetry` → existing recovery state machine.
3. Offline optimizer: host tool using momentum/impulse QP from combat research, exporting `ActionMove` JSON/keyframes into the studio; no runtime solver.
4. Momentum observer in kernel gated on torque-capable builds (`current_ma` + velocity + torque commands); v1 uses current-spike threshold heuristic.
5. Determinism: all new moves are keyframes and pure functions; replay unchanged.

## Risks / Trade-offs
- [Forceful strikes need v2] → v1 gets scripted combos only; document.
- [Optimizer is new PC tool] → keep it simple, export static data.
- [Deferred learned policy] → no opponent sensing; honest scope.
