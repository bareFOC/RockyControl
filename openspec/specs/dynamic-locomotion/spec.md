# dynamic-locomotion Specification

## Purpose

Provides SLIP-template running with torque-based thrust, flight detection, compliant landings, and CBF safety for the torque-capable hardware generation, with all algorithms deterministic and simulation-verifiable.

## Requirements

### Requirement: SLIP running template
The running controller SHALL generate vertical thrust from a spring-loaded inverted pendulum template scaled to the robot's mass, with target hop height 2–4 cm and stance duration 120–150 ms.

#### Scenario: Hop height scales with mass
- **WHEN** running is enabled on a torque-capable platform
- **THEN** thrust produces a takeoff velocity consistent with the 2–4 cm target and stance stays above 24 control ticks

### Requirement: Flight detection and landing compliance
The controller SHALL detect flight from near-zero motor current plus IMU free-fall signature with debounce, and SHALL ramp landing stiffness over 25–40 ms.

#### Scenario: Flight-to-stance transition
- **WHEN** flight is detected and then stance resumes
- **THEN** landings use compliant torque impedance rather than rigid position hold

### Requirement: CBF safety
A scalar control-barrier filter SHALL constrain torso tilt, CoM velocity, and capture point to safe sets by minimal modification of commands.

#### Scenario: Barrier prevents fall
- **WHEN** a command would push the capture point outside the safe set
- **THEN** the filter modifies the command to keep it within the set

### Requirement: Hardware gating
Running SHALL be gated so non-torque-capable builds are unaffected.

#### Scenario: v1 builds unaffected
- **WHEN** the firmware is built for the non-torque hardware generation
- **THEN** no running code is active and walking/balance behavior is unchanged
