# bipedal-walking Specification

## Purpose

Provides reactive, capture-based bipedal walking with a phase-locked gait clock, Raibert foot placement, and step-timing adaptation, running identically in simulation and firmware.

## Requirements

### Requirement: Phase-locked gait clock
The walking controller SHALL maintain a continuous gait phase accumulator driven by the configured cadence, phase-locked to the existing catch-step state, and SHALL use it to drive support swaps while walking rather than a fixed timer.

#### Scenario: Phase advances while walking
- **WHEN** walking is enabled
- **THEN** the gait phase advances continuously through all controller states and support alternates at π offsets

#### Scenario: Phase locks on catch step
- **WHEN** a catch step is entered
- **THEN** the gait phase is phase-locked to the catch-step progress without a discontinuity

### Requirement: Capture-based foot placement
The controller SHALL compute the next swing-foot landing target from the support-relative capture point and measured CoM velocity, and SHALL write it only to the swing-foot target, never to the stance hip.

#### Scenario: Raibert placement
- **WHEN** CoM velocity deviates from commanded velocity
- **THEN** the landing target is offset by the velocity error divided by the natural frequency, scaled by a configurable gain

#### Scenario: Stance hip untouched
- **WHEN** a new landing target is computed
- **THEN** the stance hip integral remains driven solely by the postural reflex

### Requirement: Step timing adaptation
The controller SHALL shorten the next step when the capture error exceeds the support margin, clamped to a minimum viable duration.

#### Scenario: Large disturbance shortens step
- **WHEN** capture point error exceeds the foot margin
- **THEN** step duration is reduced toward the minimum, enabling faster recovery

### Requirement: Deterministic walking
All walking state SHALL live in the shared kernel, be driven by fixed-dt inputs, and use only fixed-point-order f32 math so replay remains bit-exact.

#### Scenario: Replay of a walking run
- **WHEN** a walking tick log is replayed
- **THEN** gait phase, foot targets, and telemetry match bit-for-bit
