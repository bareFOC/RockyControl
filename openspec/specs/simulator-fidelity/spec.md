# simulator-fidelity Specification

## Purpose

The simulator's plant, sensor and wire behavior is faithful and observable enough that controller tuning decisions made in the app are not based on incorrect or missing signals.

## Requirements

### Requirement: Correct force channel mapping
The simulator SHALL map its simulated left foot contact/GRF to the estimator's left channel and right to right, consistently with its own `foot_contact(left)` accessor.

#### Scenario: Asymmetric contact
- **WHEN** only the simulated left foot carries load
- **THEN** published `grf_left_n` is nonzero and `grf_right_n` is zero, and the contact mask marks left only

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
