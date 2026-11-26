# unified-control-application Specification

## Purpose

Guarantees that firmware and simulation execute the same control application code path, with only I/O backends swapped, so that replay verification covers the exact code that runs on hardware and simulation results predict hardware behavior.

## Requirements

### Requirement: Single kernel wrapper
The system SHALL provide exactly one wrapper around the real-time control kernel, hosted in the shared control-application crate, and both firmware and simulation SHALL call that same wrapper to execute a control tick. All simulation callers, including MCP hosts, SHALL use the deterministic fixed-step transport path. The servo bus beneath the wrapper SHALL use BareFOC register-pointer transactions via the shared `rk-multibus` client; no separate bespoke per-joint frame codec SHALL remain.

#### Scenario: Firmware and simulation use the same wrapper
- **WHEN** a control tick is executed in firmware and in simulation
- **THEN** both paths call the same `ControlTask` implementation from the shared control-application crate, with no parallel wrapper in the simulation crate

#### Scenario: No duplicate bus trait
- **WHEN** the simulation implements its servo I/O backend
- **THEN** it implements the same `ServoBusPort`/`RobotIo` contract used by the firmware rather than a separate equivalent trait

#### Scenario: No duplicate task code
- **WHEN** the simulation is built for WASM
- **THEN** it does not contain a parallel copy of the control loop or scheduler

#### Scenario: No legacy frame codec
- **WHEN** the servo bus payload is inspected
- **THEN** the former 16-byte command / 12-byte feedback pack/unpack functions are absent

### Requirement: Deterministic host-callable control step
The control wrapper's step function SHALL be pure with respect to global state: all inputs (commands, feedback, wire error counts, time) SHALL be explicit parameters, so it produces identical results for identical inputs on host and MCU. The control application's estimator SHALL use only deterministic, fixed-order kinematic computation (mass-weighted FK, analytic Jacobian, support-relative DCM) and fixed single-precision math.

#### Scenario: Same inputs produce same outputs on host and MCU
- **WHEN** identical command, feedback, wire-error, and time inputs are fed to the control wrapper on the host and on the MCU build
- **THEN** both produce bit-identical telemetry and servo commands

#### Scenario: Replay remains bit-exact after estimator upgrade
- **WHEN** a tick log recorded with the FK-based estimator is replayed
- **THEN** every estimator and telemetry field matches bit-for-bit on a fresh control wrapper

### Requirement: Replay covers the firmware path
Deterministic replay recording and verification SHALL live in the shared control-application crate and SHALL operate on the same control wrapper that firmware uses, rather than on a simulation-only wrapper.

#### Scenario: Replay verifies firmware wrapper
- **WHEN** a recorded tick log is replayed for verification
- **THEN** verification re-executes the firmware's `ControlTask` and reports an exact match only if its telemetry and commands match the recording bit-for-bit

#### Scenario: Firmware-only wrapper bug is caught
- **WHEN** a change is made to the shared control wrapper's behavior
- **THEN** replay verification detects the behavioral difference, ensuring simulation cannot silently diverge from firmware

### Requirement: Shared transport and model store
The simulation SHALL use the shared host-link transport and flash-model-store implementation rather than parallel reimplementations.

#### Scenario: Simulation uses shared COBS transport
- **WHEN** the simulation exchanges host/MCU frames
- **THEN** it uses the shared control-application crate's COBS framing and mailbox types

#### Scenario: Simulation uses shared model store
- **WHEN** the simulation handles a model-info or model-chunk request
- **THEN** it uses the shared `FlashModelStore` implementation

### Requirement: Profile-parameterized control kernel
`DirectServoKernel` SHALL accept a `&RobotProfile` at construction, scale coupled double-joints via `motor_to_joint_ratio`, compensate chest-mounted IMU readings into the pelvis frame via `com_fk`, and drive toe push-off and 3-DoF waist balance when those roles are present in the profile.

#### Scenario: Chest IMU compensates for waist motion
- **WHEN** the 28-DoF humanoid pitches `waist_pitch` forward while the pelvis remains level
- **THEN** `KinematicEstimator` transforms the chest IMU into the pelvis frame so no false fall or balance tilt is reported

#### Scenario: Coupled double-joint ratio preserves anatomical control
- **WHEN** an anatomical knee or elbow bend `θ_bend` is commanded on a 1:2 coupled joint
- **THEN** the motor command scales by `0.5` and `com_fk` evaluates each half-joint at `±0.5 × θ_bend`

#### Scenario: Toe push-off and SLIP overlay
- **WHEN** UVC walking or SLIP running executes on the 28-DoF humanoid
- **THEN** UVC drives `toe_pitch` push-off and `waist_pitch/roll` balance, and SLIP torque overlay includes the toe joints
