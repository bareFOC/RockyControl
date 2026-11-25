# Spec Delta

## Purpose

The simulator's sensors and actuators are deterministic but imperfect, and its contacts obey unilateral/friction constraints, so controller gains and fault responses tuned in simulation transfer meaningfully to hardware.

## ADDED Requirements

### Requirement: Seeded sensor imperfection
The simulator SHALL generate sensor samples from a deterministic seeded PRNG with configurable IMU noise, bias and latency, current noise, temperature offset and encoder quantization. The same seed and command stream SHALL produce identical samples across runs.

#### Scenario: Repeatable noisy IMU
- **WHEN** two sessions use the same seed and the same command stream
- **THEN** their IMU, current and temperature samples are bit-identical

#### Scenario: Bias and latency configurable
- **WHEN** IMU bias/latency knobs are nonzero
- **THEN** published attitude differs from the true plant attitude by the configured amount and the effect persists across ticks

### Requirement: Torque-limited actuator dynamics
The simulator SHALL compute joint torque from position/velocity error and gravity, clamp it to calibrated limits, integrate angular velocity using effective link inertia, and derive measured current from the applied torque. The firmware bench loopback and simulator SHALL share this plant model.

#### Scenario: Saturated torque
- **WHEN** a large position error demands more torque than the configured limit
- **THEN** torque is clamped and measured current saturates near the corresponding current limit

#### Scenario: Shared plant model
- **WHEN** the same command is applied to the simulator plant and the firmware bench loopback
- **THEN** both advance their joints with the same dynamics for the same dt

### Requirement: Impact detector exercised end-to-end
An applied simulator shove SHALL produce a current spike that the control kernel's impact detector flags, so impact-abort behavior can be validated through the full app loop.

#### Scenario: Shove triggers impact
- **WHEN** a shove impulse is applied while walking or guarding
- **THEN** at least one joint's current jump exceeds the detector threshold and the kernel reports impact

### Requirement: Constrained contacts
The simulator SHALL enforce unilateral contacts, a friction/slip threshold and CoP-inside-support-polygon. It SHALL be able to inject a deliberate disagreement between plant contacts and the controller's scripted contact mask.

#### Scenario: No adhesive ground reaction
- **WHEN** a foot lifts above the contact tolerance
- **THEN** its GRF is zero and it contributes no support polygon point

#### Scenario: Contact disagreement fault
- **WHEN** the contact-disagreement fault is enabled
- **THEN** the controller's contact mask differs from plant contacts and estimator behavior remains stable

### Requirement: Full-app fixed-step validation
The app-level test suite SHALL exercise walking, combos, shove recovery and fall/get-up through `step_fixed` on `RockyStudioApp`, not only through the variable-dt compatibility path.

#### Scenario: Walking displacement through the app
- **WHEN** the app drives a walk command through `step_fixed`
- **THEN** observed root displacement and contact alternation match the simulator-level walking test

#### Scenario: Combo stays upright
- **WHEN** a strike combo plays through `step_fixed`
- **THEN** no fall is reported while equilibrium is maintained
