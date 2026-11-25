# state-estimation Specification

## Purpose

Provides a physically correct, deterministic CoM/CoM-velocity estimate and a support-relative capture point from joint encoders, joint velocities, and link mass properties, identical on host and MCU, as the foundation for balance, walking, and dynamic locomotion.

## Requirements

### Requirement: Mass-weighted CoM from joint state
The estimator SHALL compute the robot's Center of Mass from the joint positions and a constant per-link mass table derived from the robot's mass properties, summed over all links in a fixed parent-first order. It SHALL NOT derive CoM position from IMU tilt alone.

#### Scenario: CoM matches host golden values
- **WHEN** a known joint pose is fed to the estimator
- **THEN** its CoM position equals the host rigid-body FK reference for that pose within f32 tolerance

#### Scenario: Leg joints affect CoM
- **WHEN** only leg joints move while the torso stays level
- **THEN** the estimated CoM changes accordingly

### Requirement: Analytic CoM velocity from joint velocity
The estimator SHALL compute CoM velocity from measured joint velocities via the mass-weighted CoM Jacobian, rather than by finite-differencing position or extrapolating IMU gyro rates.

#### Scenario: Joint velocity produces CoM velocity
- **WHEN** a joint moves with nonzero measured velocity at a fixed pose
- **THEN** CoM velocity equals the mass-weighted Jacobian times that joint velocity

#### Scenario: Host/MCU parity
- **WHEN** identical joint positions and velocities are supplied on host and MCU
- **THEN** both produce bit-identical CoM velocity

### Requirement: Support-relative capture point
The estimator SHALL express the capture point relative to the current support origin using a configurable CoM height $h_{com}$, without base linear velocity. Sensed contact/GRF, when valid, determines the support origin; otherwise the phase-derived mask is used. The default $h_{com}$ SHALL match the model's FK sole-to-CoM height.

#### Scenario: Capturability uses support origin
- **WHEN** the capture point exceeds the support-foot margin
- **THEN** a recovery step is triggered based on the support-relative capture point, not a pelvis-frame proxy

#### Scenario: Fixed CoM height
- **WHEN** `h_com_m` is configured
- **THEN** the DCM natural frequency ω = sqrt(g/h_com_m) and ZMP are computed from that fixed value, independent of arm or torso proxy motion

#### Scenario: Model-consistent default height
- **WHEN** the default balance gains and the bundled URDF mass table are used
- **THEN** `h_com_m` equals the FK sole-to-CoM height at the nominal stance crouch within 0.03 m, and a golden test asserts this

#### Scenario: Sensed contact preferred when valid
- **WHEN** valid sensed contact/GRF is supplied in servo feedback
- **THEN** the estimator's support origin reflects the sensed loaded foot, and the phase-derived mask is used only as fallback

#### Scenario: Support swap is smooth
- **WHEN** support switches between feet with nonzero joint velocity
- **THEN** CoM velocity has no discontinuity spike greater than 0.5 m/s

### Requirement: Deterministic estimator inputs
The estimator SHALL consume bias-compensated attitude shared from the balance controller (lagged by at most one tick) and SHALL keep validity bits gated on IMU availability.

#### Scenario: Bias compensated attitude shared
- **WHEN** the balance controller has calibrated an IMU bias
- **THEN** the estimator uses the bias-compensated tilt for any attitude-dependent path, not raw IMU tilt

#### Scenario: IMU outage invalidates estimation outputs
- **WHEN** the IMU is unavailable
- **THEN** CoM/DCM validity bits remain unset even if joint-based kinematics are available
