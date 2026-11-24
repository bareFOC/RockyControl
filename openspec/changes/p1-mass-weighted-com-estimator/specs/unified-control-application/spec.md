# Spec Delta

## MODIFIED Requirements

### Requirement: Deterministic host-callable control step
The control wrapper's step function SHALL be pure with respect to global state: all inputs (commands, feedback, wire error counts, time) SHALL be explicit parameters, so it produces identical results for identical inputs on host and MCU. The control application's estimator SHALL use only deterministic, fixed-order kinematic computation (mass-weighted FK, analytic Jacobian, support-relative DCM) and fixed single-precision math.

#### Scenario: Same inputs produce same outputs on host and MCU
- **WHEN** identical command, feedback, wire-error, and time inputs are fed to the control wrapper on the host and on the MCU build
- **THEN** both produce bit-identical telemetry and servo commands

#### Scenario: Replay remains bit-exact after estimator upgrade
- **WHEN** a tick log recorded with the FK-based estimator is replayed
- **THEN** every estimator and telemetry field matches bit-for-bit on a fresh control wrapper
