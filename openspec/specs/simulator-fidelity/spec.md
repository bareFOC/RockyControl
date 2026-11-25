# simulator-fidelity Specification

## Purpose

The simulator's plant, sensor and wire behavior is faithful and observable enough that controller tuning decisions made in the app are not based on incorrect or missing signals.

## Requirements

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

### Requirement: Velocity commands drive the plant
The simulator plant SHALL honor `JointCommandMode::Velocity` by using the commanded `raw_velocity_rad_s`; it SHALL NOT treat velocity-mode joints as held position.

#### Scenario: Velocity-mode motion
- **WHEN** a joint is commanded to velocity mode with a nonzero velocity for several ticks
- **THEN** its measured angle changes in the commanded direction and its reported velocity approaches the command

### Requirement: Post-tick side effects execute in simulation
The simulator SHALL implement `RobotIo::post_tick` and evaluate the same LED mode state machine that firmware evaluates, and SHALL expose the resulting LED state to the host.

#### Scenario: LED state advances each tick
- **WHEN** the simulation advances control ticks
- **THEN** `post_tick` is called once per tick and the exposed LED state matches `current_led_mode().is_lit(now_ms)`

### Requirement: Fixed-step wire errors are simulated
The fixed-step transport path SHALL encode host commands with COBS and decode them through the same framing code as firmware, with injectable corruption so wire-error counters are exercised in app and MCP runs.

#### Scenario: Corrupt frame increments error counter
- **WHEN** a corrupted host frame is injected on the fixed-step path
- **THEN** the next telemetry snapshot reports the corresponding `rx_crc_errors` increment and the control loop keeps running

#### Scenario: Clean frame is unaffected
- **WHEN** no fault is injected
- **THEN** fixed-step decoding succeeds with zero wire errors and bit-exact replay is preserved

### Requirement: Reproducible reset experiment
The system SHALL provide an operation that resets the mock clock, control kernel, plant state, fault injection and UI history together, and SHALL invoke it on simulation reconnect.

#### Scenario: Two identical runs match
- **WHEN** the same scripted command stream is replayed after a reset experiment
- **THEN** the resulting telemetry and recorded replay are identical

### Requirement: Complete fault injection surface
MCP SHALL expose every simulator fault knob that the native UI exposes, including IMU disconnect, servo bus error, external hand wave and servo lag factor.

#### Scenario: MCP fault parity
- **WHEN** a client sets any supported `sim_inject_fault` flag via MCP
- **THEN** the simulator behaves as it does when the same flag is set in the native UI

### Requirement: Balance observability
The simulator SHALL expose achieved CoM position, left/right GRF, contact mask and tilt rate in telemetry samples, plot them in the UI, and visualize per-foot contact/GRF in the viewport.

#### Scenario: Tuning signals visible live
- **WHEN** a developer runs a walking experiment in the app
- **THEN** live plots show achieved CoM and GRF alongside commanded values, and the viewport colors feet by contact/GRF

### Requirement: Realistic disturbance and sensor scenarios
The simulator SHALL support scripted walk-disturbance scenarios (low friction, sensor latency, stance stall, IMU dropout) and a realistic small-humanoid sensor profile with nonzero noise, bias and latency. Shoves SHALL inject a linear CoM impulse into the plant and select the stance leg from actual ground reaction force rather than the controller's phase output.

#### Scenario: Realistic-noise walking
- **WHEN** walking runs with the realistic sensor profile
- **THEN** no over-current or fall fault latches and the quality metrics stay within their acceptance bounds

#### Scenario: Disturbance recovery
- **WHEN** low friction or sensor latency is scripted mid-walk
- **THEN** the controller recovers within two seconds or latches a clean fall with recovery intact
