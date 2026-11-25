# Spec Delta

## MODIFIED Requirements

### Requirement: Command and telemetry handoff without re-spawn
The persistent control task SHALL receive queued host commands and publish telemetry/recorded ticks through shared state that does not require re-spawning the task. On the fixed-step path, host commands SHALL be exchanged through COBS framing with the same decoder firmware uses, and the system SHALL provide a reset operation that restores mock clock, kernel, plant, faults and history together.

#### Scenario: Host command applied on next step
- **WHEN** a new command is queued between two fixed-step calls
- **THEN** the running control task consumes it on the next tick

#### Scenario: Replay stays bit-exact
- **WHEN** ticks are recorded across many fixed-step calls
- **THEN** replay verification reports an exact match with no per-call resets

#### Scenario: Fixed-step wire errors are observable
- **WHEN** a corrupted host frame is queued between fixed-step calls
- **THEN** the next telemetry snapshot reports the wire error and the control task keeps stepping

#### Scenario: Reset experiment restores initial state
- **WHEN** the reset-experiment operation is invoked
- **THEN** the next command stream produces the same telemetry and replay as a fresh simulation session
