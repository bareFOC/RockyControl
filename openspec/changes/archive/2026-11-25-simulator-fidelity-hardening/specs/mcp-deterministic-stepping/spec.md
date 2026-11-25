# Spec Delta

## MODIFIED Requirements

### Requirement: Fixed-step MCP stepping
MCP hosts SHALL advance the simulation using the shared fixed 5 ms control-step path rather than the variable-dt compatibility path. Tool semantics (per-tick action playback advancement, config drain, sequence numbers, telemetry sampling) SHALL be preserved. MCP SHALL expose the complete simulator fault set and the reset-experiment operation.

#### Scenario: MCP steps use firmware dt
- **WHEN** an MCP tool advances N ticks
- **THEN** each recorded control tick has dt = 5 ms and now_ms increases by 5

#### Scenario: Returned telemetry remains augmented
- **WHEN** MCP steps the simulator
- **THEN** returned telemetry still includes plant world/contact/GRF channels while recorded replay ticks stay kernel-exact

#### Scenario: Fault injection parity with the UI
- **WHEN** a client sets IMU-disconnect, bus-error, hand-wave or servo-lag faults via MCP
- **THEN** the simulator behaves identically to setting the same faults in the native UI

#### Scenario: Reset experiment via MCP
- **WHEN** a client invokes reset-experiment and then replays a scripted command stream
- **THEN** telemetry and replay match a fresh session given the same stream
