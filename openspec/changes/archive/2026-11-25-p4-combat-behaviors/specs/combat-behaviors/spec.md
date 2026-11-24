# Spec Delta

## Purpose

Provides scripted combat combos with balance-restoring tails, offline impact-aware strike optimization, and impact detection, all validated by replay and shove tests.

## ADDED Requirements

### Requirement: Scripted combos with balance tails
The Action Studio SHALL expose combo actions that chain existing strikes with a Guard recovery and an explicit balance-restore tail driven by the shared estimator.

#### Scenario: Combo returns to guard
- **WHEN** a combo playback finishes
- **THEN** the robot ends in Guard with a stable capture point

#### Scenario: Balance restore during combo
- **WHEN** the capture point leaves the safe margin during a strike
- **THEN** the combo applies a corrective balance tail rather than continuing blind

### Requirement: Impact-triggered abort and recovery
Playback SHALL abort on fall/EStop/tilt breach and hand off to the existing recovery state machine.

#### Scenario: Fall during combo
- **WHEN** a fall is detected during combo playback
- **THEN** playback stops and the get-up recovery takes over

### Requirement: Offline impact-aware optimization
An offline PC tool SHALL optimize strike keyframes for momentum/impulse with a balance-restore constraint and export them into the Action Studio; no runtime solver SHALL be required.

#### Scenario: Optimized strike export
- **WHEN** the optimizer runs on the PC
- **THEN** it produces keyframe actions playable by the MCU without a PC

### Requirement: Hardware-gated impact detection
Momentum-observer impact detection SHALL be active only on torque-capable hardware; other builds use a current-spike heuristic.

#### Scenario: v1 impact heuristic
- **WHEN** running on non-torque hardware
- **THEN** current spikes (not a momentum observer) signal impacts
