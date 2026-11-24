# Spec Delta

## MODIFIED Requirements

### Requirement: Board drivers are instantiated
A board driver module that is added to the firmware core SHALL be instantiated and connected to the target's control loop when its feature is enabled, rather than existing only as compiled-but-unused library code. Hardware I/O performed by such drivers SHALL NOT execute inside a `critical_section` with interrupts disabled, and async drivers SHALL use Embassy's async/DMA APIs rather than blocking calls in async tasks.

#### Scenario: LSM6DSV feature wires the IMU task
- **WHEN** the RP2350 firmware is built with the LSM6DSV IMU feature enabled
- **THEN** an SPI task reads the sensor and publishes orientation samples to the shared IMU slot consumed by the control loop

#### Scenario: I3C servo feature wires the bus hooks
- **WHEN** the RP2350 firmware is built with the I3C servo-bus feature enabled
- **THEN** an I3C board module registers servo-bus hooks that are called by the control loop each tick

#### Scenario: SPI reads do not block the executor
- **WHEN** the LSM6DSV task reads the sensor FIFO over SPI
- **THEN** the transfer is performed with Embassy's async/DMA SPI API and the task yields while waiting, rather than spinning on a blocking transfer

#### Scenario: I3C transactions run outside critical sections
- **WHEN** the I3C servo adapter exchanges frames with the actuators
- **THEN** transactions occur outside any `critical_section` and only short state snapshots/result recordings are protected by critical sections

### Requirement: Declared features compile
Every firmware feature flag declared in the RP2350 board crate SHALL compile successfully, including the optional servo-bus transport features. Mutually exclusive features SHALL be rejected at compile time with a clear diagnostic rather than silently overriding each other.

#### Scenario: UART servo bus feature builds
- **WHEN** the RP2350 firmware is checked with the `uart-servo-bus` feature enabled
- **THEN** the build completes without struct-field or type errors in the servo bus adapter

#### Scenario: Conflicting servo buses are rejected
- **WHEN** both `uart-servo-bus` and `i3c-servo-bus` features are enabled
- **THEN** compilation fails with an explicit diagnostic naming the conflict
