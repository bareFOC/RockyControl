# Spec Delta

## ADDED Requirements

### Requirement: 28-DoF keyframe calibration for recovery and combat libraries
`default_recovery_library` and `default_combat_library` SHALL target the 28-DoF humanoid joint set (including `waist_yaw/pitch/roll`, `toe_pitch`, and coupled double-knee/elbow total-bend angles) and SHALL respect all 28-DoF URDF and `humanoid.robot.toml` joint limits.

#### Scenario: Four-orientation get-up succeeds on 28-DoF humanoid
- **WHEN** `fall_recover` or orientation-specific recovery (prone, supine, left-side, right-side) executes in simulation on the 28-DoF humanoid
- **THEN** every keyframe remains within 28-DoF joint limits and the robot returns to a stable standing guard stance

#### Scenario: Combat library respects 28-DoF limits
- **WHEN** `default_combat_library` actions are instantiated and sampled on the 28-DoF humanoid rig
- **THEN** all joint targets remain within the 28-DoF joint limits without self-clipping or over-current faults
