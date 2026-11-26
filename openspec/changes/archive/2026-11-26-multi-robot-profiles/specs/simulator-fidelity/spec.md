# Spec Delta

## ADDED Requirements

### Requirement: Profile-driven plant and two-point foot contact
The simulator SHALL construct its world dynamics, joint plant, torque masks (`u64`), and actuator constants from the active `RobotProfile`, and SHALL model 2-point foot contact (heel + toe) using `com_fk` foot and toe link frames.

#### Scenario: Toe push-off shifts foot contact forward
- **WHEN** the 28-DoF humanoid pitches its toes down during stance push-off
- **THEN** the simulator detects toe contact via toe-link FK and supports ground reaction force at the toe contact point
