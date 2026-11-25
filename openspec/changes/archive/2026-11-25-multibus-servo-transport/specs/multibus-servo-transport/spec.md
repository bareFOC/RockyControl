# Spec Delta

## Purpose

Servo command and feedback exchange uses BareFOC register-pointer transactions through a shared client, so firmware and simulation exercise the same wire encoding and the servo protocol matches BareFOC firmware.

## ADDED Requirements

### Requirement: Register-pointer servo protocol
Servo commands and feedback SHALL be exchanged as BareFOC register-pointer transactions: a 2-byte big-endian register pointer followed by little-endian payload bytes for writes, and pointer-latch plus burst read for reads. The bespoke 16-byte command / 12-byte feedback frame format SHALL be removed.

#### Scenario: Goal registers written each control tick
- **WHEN** a control tick commands joints
- **THEN** torque/position/velocity/current target registers are written at the canonical BareFOC offsets via `ControlTableClient`

#### Scenario: Feedback burst read
- **WHEN** the controller needs measured joint state
- **THEN** present position/velocity/current registers are burst-read and mapped into `RawServoFeedback`

### Requirement: Shared client across firmware and simulation
The same `rk-multibus` client and register mapping SHALL be used by firmware and the software simulator. Simulation SHALL NOT bypass wire encoding by passing `RawServoCommand` structs directly to the plant.

#### Scenario: Simulator uses real wire encoding
- **WHEN** the simulator exchanges servo commands/feedback
- **THEN** bytes flow through the same register client/transport used by firmware (backed by an in-memory register server or mock I2C)

#### Scenario: Codec round-trip verified
- **WHEN** a command is written to the simulated servo register table
- **THEN** the plant consumes the register values and the feedback registers reflect its state

### Requirement: Bus health isolation
Per-joint timeout and degraded-mask accounting SHALL be preserved when a register transaction fails, without halting transactions to other joints or stalling the control loop.

#### Scenario: One joint fails
- **WHEN** a servo does not acknowledge a register transaction
- **THEN** only that joint is marked degraded and the cumulative timeout count increases

### Requirement: No upstream modification
The BareFOC `barefoc-master` / `barefoc-multibus` crates SHALL be consumed as read-only path dependencies. No source files in the sibling bareFOC repository SHALL be modified by this change.

#### Scenario: Read-only dependency
- **WHEN** the change is built
- **THEN** bareFOC crates are referenced via path dependencies and their source tree is untouched
