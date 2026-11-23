# fall-detection Specification

## Purpose

Acquires full-orientation inertial data from the robot's IMU and classifies fall events into prone, supine, left-side, and right-side so downstream recovery behavior can select an appropriate response.

## Requirements

### Requirement: Full orientation telemetry
The system SHALL acquire and transmit torso orientation containing pitch, roll, heading/yaw, angular rates, and a gravity vector, and SHALL mark the sample as available only when a live sensor or simulation source is present.

#### Scenario: Hardware IMU reports full orientation
- **WHEN** the BNO055 hardware IMU is healthy and sampling
- **THEN** each telemetry sample contains valid pitch, roll, yaw, pitch rate, roll rate, yaw rate, and gravity-vector values and is marked available

#### Scenario: No IMU hardware present
- **WHEN** a board has no IMU or the sensor is disconnected
- **THEN** telemetry reports the IMU as unavailable with neutral orientation values, and no fall classification is attempted

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
