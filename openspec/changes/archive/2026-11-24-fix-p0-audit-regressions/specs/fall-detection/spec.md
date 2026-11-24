# Spec Delta

## MODIFIED Requirements

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
