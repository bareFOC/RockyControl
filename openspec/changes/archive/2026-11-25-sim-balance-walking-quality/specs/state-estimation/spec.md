# Spec Delta

## MODIFIED Requirements

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
