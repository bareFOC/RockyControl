# Spec Delta

## ADDED Requirements

### Requirement: REP-103 humanoid frame and 28-DoF walking tuning
The bundled 28-DoF humanoid model SHALL use the ROS REP-103 body frame (`+X` forward, `+Y` left, `+Z` up) with verified joint axis sign conventions, and default `BalanceGains` and `RunningConfig` SHALL be calibrated for `h_com_m ≈ 0.25 m` ($\omega \approx 6.2\text{ rad/s}$) with `toe_pitch` push-off.

#### Scenario: REP-103 forward walk direction
- **WHEN** a positive forward velocity command is issued on the 28-DoF humanoid
- **THEN** the robot walks in `+X`, turns positively around `+Z` (left), and pitches nose-down positively around `+Y`

#### Scenario: Tuned 28-DoF steady walking and shove recovery
- **WHEN** `walk_nominal` and `walk_shove` run with the default 28-DoF `BalanceGains`
- **THEN** steady walking and post-shove recovery satisfy the qualitative walking acceptance bounds without falling or over-current faults
