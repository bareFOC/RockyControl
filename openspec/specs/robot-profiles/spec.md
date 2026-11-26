# robot-profiles Specification

## Purpose

Robot-specific kinematics, geometry, coupled-joint ratios, IMU mounting, and multi-bus actuator data are supplied by one compile-time profile shared by firmware and simulator, replacing Rocky Mk1 with the 28-DoF humanoid and 8-DoF Striker.

## Requirements

### Requirement: Compile-time robot profile
The system SHALL define one `no_std` `RobotProfile` data type in `crates/rk-robot-profile` carrying actuator count, URDF link/joint kinematics (with coupled-joint `mimic` mapping and `motor_to_joint_ratio`), `LegGeometry`, `LinkMass`, `JointKin`, `TorqueConstants`, foot/toe indices, IMU link/mounting transform, and per-joint `ServoEndpoint`s. Firmware and simulator SHALL consume the same profile type with zero heap allocation and no runtime XML parsing.

#### Scenario: Profile is compile-time data
- **WHEN** firmware or the simulator is built for a robot
- **THEN** the profile's tables are `const`/`static` with zero heap allocation and no runtime XML parsing

### Requirement: Code-generated model data
Mass, joint-kinematics, and actuator tables SHALL be generated from `assets/robots/*.urdf` and `*.robot.toml` by `xtask codegen-robots` and checked in, with golden tests comparing generated data back to the source URDF. Hand-maintained copies SHALL be removed.

#### Scenario: Generated data matches URDF
- **WHEN** `codegen-robots` runs and the golden test executes
- **THEN** generated masses, joint origins/axes, coupled-joint maps, and total mass match the source URDF within f32 tolerance

### Requirement: One URDF-free robot profile per firmware build
A board firmware build SHALL select exactly one robot profile via mutually exclusive compile-time features (`robot-humanoid` / `robot-striker`), and SHALL NOT embed URDF bytes or mesh assets in the firmware binary.

#### Scenario: Profile feature required
- **WHEN** no robot profile feature (or more than one) is selected
- **THEN** compilation fails with an explicit `compile_error!`

#### Scenario: Firmware remains URDF-free
- **WHEN** firmware is built for any robot profile
- **THEN** no URDF bytes or STL meshes are linked into the binary

### Requirement: Protocol v7 capacity for 28-DoF humanoid
`MAX_JOINTS`, `MAX_IMPACT_JOINTS`, and `MAX_SERVO_JOINTS` SHALL be `36`, `torque_mask` SHALL be `u64`, and `PROTOCOL_VERSION` SHALL be `7`, with deterministic replay fixtures regenerated for the 28-DoF humanoid.

#### Scenario: Replay fixtures verify on wire v7
- **WHEN** committed experiment scripts and `.copper` replay logs are executed against the 28-DoF humanoid profile
- **THEN** all report `exact_match` with zero tick mismatches

### Requirement: Second robot support
The repository SHALL include an 8-DoF Striker profile whose visual/teach path works through the existing URDF loading, with its <17-DoF locomotion limitations explicitly documented.

#### Scenario: Load Striker
- **WHEN** the Striker preset is loaded in the app or built into firmware
- **THEN** the model has 8 joints, correct visuals and teach/position control, and no crash occurs
