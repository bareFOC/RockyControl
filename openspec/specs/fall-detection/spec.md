# fall-detection Specification

## Purpose

Acquires full-orientation inertial data from the robot's IMU and classifies fall events into prone, supine, left-side, and right-side so downstream recovery behavior can select an appropriate response.

## Requirements

### Requirement: Full orientation telemetry
The system SHALL acquire and transmit torso orientation containing pitch, roll, heading/yaw, angular rates, and a gravity vector, and SHALL mark the sample as available only when a live sensor or simulation source is present. The IMU source tag SHALL identify the actual active sensor (high-rate IMU or legacy IMU), and a high-rate IMU sample SHALL NOT be marked available until its settling period has completed.

#### Scenario: Hardware IMU reports full orientation
- **WHEN** the BNO055 hardware IMU is healthy and sampling
- **THEN** each telemetry sample contains valid pitch, roll, yaw, pitch rate, roll rate, yaw rate, and gravity-vector values and is marked available

#### Scenario: No IMU hardware present
- **WHEN** a board has no IMU or the sensor is disconnected
- **THEN** telemetry reports the IMU as unavailable with neutral orientation values, and no fall classification is attempted

#### Scenario: Legacy IMU is the active source
- **WHEN** the legacy BNO055 is the sensor publishing orientation samples to the control loop
- **THEN** telemetry tags the source as the legacy IMU and does not claim the high-rate IMU is active

#### Scenario: High-rate IMU settling
- **WHEN** the high-rate IMU has started sampling but its settling period has not yet elapsed
- **THEN** the sample is not marked available and is not consumed by fall classification

#### Scenario: High-rate IMU becomes active
- **WHEN** the high-rate IMU driver task is enabled and publishing valid settled samples
- **THEN** telemetry tags the source as the high-rate IMU and the legacy IMU is removed from the control path

### Requirement: Invalid orientation rejection
The system SHALL reject non-finite orientation values at the protocol boundary so corrupt sensor samples cannot influence fall classification.

#### Scenario: Corrupt telemetry frame
- **WHEN** a telemetry frame contains NaN or infinite pitch, roll, yaw, rate, or gravity values
- **THEN** the frame is rejected and the last known-good orientation is retained

### Requirement: Fall orientation classification
The system SHALL classify a fall event as prone, supine, left-side, or right-side from the gravity vector when the IMU is available and a fall threshold has been breached.

#### Scenario: Forward fall
- **WHEN** a fall is detected and the gravity vector indicates the torso tipped forward
- **THEN** the fall is classified as prone

#### Scenario: Backward fall
- **WHEN** a fall is detected and the gravity vector indicates the torso tipped backward
- **THEN** the fall is classified as supine

#### Scenario: Side fall
- **WHEN** a fall is detected and the gravity vector indicates the torso tipped laterally
- **THEN** the fall is classified as left-side or right-side according to the sign of the lateral component

#### Scenario: Orientation unavailable at fall time
- **WHEN** a fall threshold is breached but the IMU is unavailable
- **THEN** the system records a fall event with an unknown orientation rather than guessing a direction

### Requirement: Simulated orientation parity
The simulation source SHALL synthesize a gravity vector consistent with the simulated torso tilt and heading so that fall classification behaves equivalently in simulation and on hardware.

#### Scenario: Simulated forward tilt
- **WHEN** the simulated plant tilts the torso forward under a shove
- **THEN** the published IMU sample's gravity vector corresponds to a forward lean and classifies as prone when over the fall threshold
