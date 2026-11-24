# firmware-integration-health Specification

## Purpose

Guarantees that firmware features declared in the build compile, that new board driver modules are actually instantiated on the target rather than left as library-only scaffolding, and that per-joint bus faults are isolated without crashing the control loop.

## Requirements

### Requirement: Declared features compile
Every firmware feature flag declared in the RP2350 board crate SHALL compile successfully, including the optional servo-bus transport features.

#### Scenario: UART servo bus feature builds
- **WHEN** the RP2350 firmware is checked with the `uart-servo-bus` feature enabled
- **THEN** the build completes without struct-field or type errors in the servo bus adapter

### Requirement: Board drivers are instantiated
A board driver module that is added to the firmware core SHALL be instantiated and connected to the target's control loop when its feature is enabled, rather than existing only as compiled-but-unused library code.

#### Scenario: LSM6DSV feature wires the IMU task
- **WHEN** the RP2350 firmware is built with the LSM6DSV IMU feature enabled
- **THEN** an SPI task reads the sensor and publishes orientation samples to the shared IMU slot consumed by the control loop

#### Scenario: I3C servo feature wires the bus hooks
- **WHEN** the RP2350 firmware is built with the I3C servo-bus feature enabled
- **THEN** an I3C board module registers servo-bus hooks that are called by the control loop each tick

### Requirement: Per-joint bus fault isolation
A timeout or CRC error on one servo SHALL increment a per-joint counter and degrade only that joint, without halting transactions to other joints or starving the watchdog feed.

#### Scenario: One servo stops responding
- **WHEN** a single joint misses consecutive I3C replies beyond the degradation threshold
- **THEN** that joint is marked degraded and a bus-error indication is raised while all other joints continue to be serviced

#### Scenario: Transient error recovers
- **WHEN** a degraded joint subsequently replies correctly
- **THEN** its consecutive-timeout counter resets and the cumulative timeout count is retained for diagnostics

### Requirement: Validated driver bring-up state
A high-rate IMU driver SHALL not report a sample as available until its configured settling period has completed, and a sample batch that crosses the settling boundary SHALL transition to available on the expected batch.

#### Scenario: FIFO burst crosses the settling boundary
- **WHEN** the final settling sample falls within a burst of 960 Hz samples
- **THEN** the driver marks the sensor available after that burst without requiring an additional full burst

### Requirement: CI health
The host library test suite and the firmware core test suite SHALL pass with zero failing tests at the change's completion.

#### Scenario: Full test run
- **WHEN** the workspace host-crate tests and the firmware core tests are executed
- **THEN** every test passes, including the estimator validity test, the simulation shove-recovery test, and the FIFO settling test
