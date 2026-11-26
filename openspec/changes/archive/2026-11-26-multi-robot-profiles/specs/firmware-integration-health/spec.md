# Spec Delta

## ADDED Requirements

### Requirement: Multi-bus I3C servo dispatch per robot profile
When `i3c-servo-bus` is enabled, the RP2350 board adapter SHALL support up to 4 registered `PioI3c` bus transports and dispatch each joint transaction to the `(bus_id, node_addr)` specified in the selected `RobotProfile`, while keeping `rk-esp32p4` compiling against `RobotProfile` and `MAX_JOINTS = 36`.

#### Scenario: 28-DoF humanoid dispatches across multiple I3C buses
- **WHEN** `robot-humanoid` and `i3c-servo-bus` are built
- **THEN** each joint reads and writes via its configured `ServoEndpoint { bus_id, node_addr }` without exceeding per-bus 200 Hz tick budget
