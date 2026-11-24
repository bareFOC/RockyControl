# Proposal

## Why

The Action Studio already has scripted combat moves (Guard, Jab, Cross, Kick) and get-up moves, but no combos, no impact-aware balance restoration, and no telemetry-triggered abort. These are feasible on v1 (scripted + balance overlay) and deliver the most visible capability for a fighting robot without requiring v2 torque hardware. Forceful impact-aware optimization and learned residuals are v2/PC-only and explicitly deferred to later slices.

## What Changes

- Add scripted **combo** moves (sequences of existing strikes + Guard recovery) to the Action Studio library, with automatic balance-restore tails using the P1/P2 estimator and DCM overlay.
- Add telemetry-triggered abort: cancel playback on fall/EStop/tilt breach and hand off to the existing recovery state machine.
- Add an impact-aware strike keyframe optimizer as an **offline PC tool** (momentum/impulse QP formulation from the combat research) that exports keyframes into the Action Studio — no runtime solver.
- Momentum-observer-based impact detection (0.1–0.3 N·m thresholds, backward Euler, λ̂=0) gated to torque-capable v2 hardware; v1 uses current-spike heuristic.
- Explicitly defer the int8 MLP residual policy (blind without opponent sensing).

## Capabilities
### New Capabilities
- `combat-behaviors`: Scripted combos with balance-restoring tails, impact-aware offline strike optimization, and impact detection, validated by replay and shove tests.

### Modified Capabilities
- `recovery-motion`: combos hand off to existing fall/get-up recovery on abort.

## Impact
- `rk-model/src/studio.rs` (combos, abort), `rk-ui`/`rk-mcp` (combo UI/tools), offline optimizer (new host tool), `rk-control` (v2 impact detection gate).
- Risk: offline optimizer QP is a new PC tool; forceful strikes require v2 for full effect.
