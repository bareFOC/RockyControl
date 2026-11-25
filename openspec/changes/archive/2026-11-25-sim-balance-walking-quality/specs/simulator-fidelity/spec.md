# Spec Delta

## MODIFIED Requirements

### Requirement: Correct force channel mapping
The simulator SHALL map its simulated left foot contact/GRF to the estimator's left channel and right to right, consistently with its own `foot_contact(left)` accessor. Per-foot GRF SHALL be normalized by the number of loaded feet so single support carries the full body weight and no foot reports adhesive force while airborne.

#### Scenario: Asymmetric contact
- **WHEN** only the simulated left foot carries load
- **THEN** published `grf_left_n` is approximately the full body weight and `grf_right_n` is zero, and the contact mask marks left only

#### Scenario: Single support carries body weight
- **WHEN** the simulator walks with alternating single support
- **THEN** the loaded foot's GRF is within 15% of the robot's weight and flight ticks report zero GRF

#### Scenario: Accumulated slip is observable
- **WHEN** a foot slips during a step
- **THEN** the sim accumulates slip distance for that foot and exposes it for quality assertions

## ADDED Requirements

### Requirement: Realistic disturbance and sensor scenarios
The simulator SHALL support scripted walk-disturbance scenarios (low friction, sensor latency, stance stall, IMU dropout) and a realistic small-humanoid sensor profile with nonzero noise, bias and latency. Shoves SHALL inject a linear CoM impulse into the plant and select the stance leg from actual ground reaction force rather than the controller's phase output.

#### Scenario: Realistic-noise walking
- **WHEN** walking runs with the realistic sensor profile
- **THEN** no over-current or fall fault latches and the quality metrics stay within their acceptance bounds

#### Scenario: Disturbance recovery
- **WHEN** low friction or sensor latency is scripted mid-walk
- **THEN** the controller recovers within two seconds or latches a clean fall with recovery intact
