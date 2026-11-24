# Spec Delta

## MODIFIED Requirements

### Requirement: Deterministic walking
All walking state SHALL live in the shared kernel, be driven by fixed-dt inputs, and use only fixed-point-order f32 math so replay remains bit-exact. In simulation the walking loop SHALL be driven at a fixed 200 Hz by the deterministic Embassy harness and SHALL produce observable ground-plane translation.

#### Scenario: Replay of a walking run
- **WHEN** a walking tick log is replayed
- **THEN** gait phase, foot targets, and telemetry match bit-for-bit

#### Scenario: Simulated walking advances the body
- **WHEN** forward walking is commanded in the app
- **THEN** the robot's world position advances and measured CoM velocity is nonzero
