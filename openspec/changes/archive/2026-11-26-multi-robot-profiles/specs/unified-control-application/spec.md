# Spec Delta

## ADDED Requirements

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
