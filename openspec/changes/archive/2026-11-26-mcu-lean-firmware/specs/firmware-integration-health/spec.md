# Spec Delta

## MODIFIED Requirements

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
