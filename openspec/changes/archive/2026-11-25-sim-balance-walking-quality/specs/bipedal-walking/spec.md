# Spec Delta

## MODIFIED Requirements

### Requirement: Step timing adaptation
The gait clock SHALL shorten or lengthen the support half-cycle from the Khadiv capture-error estimate during walking as well as during catch steps. A capture breach while `walk_enabled` SHALL produce a measurably shorter next step, and the nominal cadence SHALL apply when there is no capture error.

#### Scenario: Large disturbance shortens step
- **WHEN** capture point error exceeds the foot margin
- **THEN** step duration is reduced toward the minimum, enabling faster recovery

#### Scenario: Walking cadence follows capture error
- **WHEN** a disturbance breaches the capture margin during walking
- **THEN** the next support swap occurs earlier than the nominal half-cycle and the controller recovers without falling

#### Scenario: Nominal cadence undisturbed
- **WHEN** no capture error exists during walking
- **THEN** phase advances at `2π·cadence·dt` as before

### Requirement: Capture-based foot placement
Capturability SHALL be evaluated with a CoM height consistent with the robot's mass model, and the step trigger SHALL NOT fire permanently during steady walking. Foot placement SHALL use sensed contact/GRF when available and the gait-phase fallback otherwise.

#### Scenario: Raibert placement
- **WHEN** CoM velocity deviates from commanded velocity
- **THEN** the landing target is offset by the velocity error divided by the natural frequency, scaled by a configurable gain

#### Scenario: Stance hip untouched
- **WHEN** a new landing target is computed
- **THEN** the stance hip integral remains driven solely by the postural reflex

#### Scenario: Consistent CoM height
- **WHEN** the default balance gains are used with the bundled URDF
- **THEN** `h_com_m` matches the FK-derived sole-to-CoM height at nominal stance crouch within 0.03 m

#### Scenario: Steady walking is capturable
- **WHEN** the robot walks at a bounded commanded velocity
- **THEN** the capturability margin is negative for less than 30% of steady-state ticks

#### Scenario: Sensed contact overrides phase
- **WHEN** the simulator publishes valid sensed contact
- **THEN** the estimator uses it for the support origin instead of the controller's phase-derived mask

## ADDED Requirements

### Requirement: Qualitative walking acceptance
The walking simulation SHALL demonstrate bounded speed tracking, tilt, lateral drift, CoM height, ground reaction force, slip, duty factor and current draw, with quantitative acceptance criteria asserted by the simulator and app-level tests.

#### Scenario: Quality metrics asserted
- **WHEN** the end-to-end walking test runs
- **THEN** it asserts mean speed within 10% of command, peak tilt < 0.08 rad, steady tilt < 0.03 rad, lateral drift < 0.03 m, CoM height within 0.03 m, single-support GRF ≈ body weight, no flight ticks, and no over-current advisory

#### Scenario: Realistic sensor robustness
- **WHEN** a realistic small-humanoid sensor profile is enabled
- **THEN** walking still satisfies the quality criteria without latching fall or over-current faults
