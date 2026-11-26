# firmware-integration-health Specification

## Purpose

Guarantees that firmware features declared in the build compile, that new board driver modules are actually instantiated on the target rather than left as library-only scaffolding, and that per-joint bus faults are isolated without crashing the control loop.

## Requirements

### Requirement: Declared features compile
Each declared firmware feature set SHALL compile for the RP2350 thumb target, reject mutually exclusive features at compile time with a clear diagnostic rather than silently overriding each other, and SHALL NOT include model data that the control path never uses.

#### Scenario: UART servo bus feature builds
- **WHEN** the `uart-servo-bus` feature is enabled
- **THEN** the firmware builds for the thumb target without conflicting servo bus hooks

#### Scenario: Conflicting servo buses are rejected
- **WHEN** both `uart-servo-bus` and `i3c-servo-bus` are enabled
- **THEN** compilation fails with an explicit error

#### Scenario: Hardware build has no embedded URDF
- **WHEN** the hardware feature set (`usb-net,imu-lsm6dsv,i3c-servo-bus`) is built
- **THEN** no URDF bytes are present in `.rodata` and no `FlashModelStore` UDP service is spawned

#### Scenario: Bench plant is opt-in
- **WHEN** a production hardware build is produced without `bench-loopback`
- **THEN** no `BenchLoopbackBus` fallback is linked into the servo bus slot

#### Scenario: Lean release profile
- **WHEN** a release firmware is built
- **THEN** fat LTO is enabled and the flashed size is measurably smaller than the previous thin-LTO baseline

#### Scenario: Still compiles for every feature set
- **WHEN** the xtask thumb feature matrix runs
- **THEN** default CDC, hardware, and USB-Net builds all succeed

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

### Requirement: Multi-bus I3C servo dispatch per robot profile
When `i3c-servo-bus` is enabled, the RP2350 board adapter SHALL support up to 4 registered `PioI3c` bus transports and dispatch each joint transaction to the `(bus_id, node_addr)` specified in the selected `RobotProfile`, while keeping `rk-esp32p4` compiling against `RobotProfile` and `MAX_JOINTS = 36`.

#### Scenario: 28-DoF humanoid dispatches across multiple I3C buses
- **WHEN** `robot-humanoid` and `i3c-servo-bus` are built
- **THEN** each joint reads and writes via its configured `ServoEndpoint { bus_id, node_addr }` without exceeding per-bus 200 Hz tick budget
