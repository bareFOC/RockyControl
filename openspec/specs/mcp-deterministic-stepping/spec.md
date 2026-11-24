# mcp-deterministic-stepping Specification

## Purpose

MCP tool calls advance the simulation with the same deterministic fixed-step control path as the GUI, so replay verification is bit-exact and headless behavior matches the app.

## Requirements

### Requirement: Fixed-step MCP stepping
MCP hosts SHALL advance the simulation using the shared fixed 5 ms control-step path rather than the variable-dt compatibility path. Tool semantics (per-tick action playback advancement, config drain, sequence numbers, telemetry sampling) SHALL be preserved.

#### Scenario: MCP steps use firmware dt
- **WHEN** an MCP tool advances N ticks
- **THEN** each recorded control tick has dt = 5 ms and now_ms increases by 5

#### Scenario: Returned telemetry remains augmented
- **WHEN** MCP steps the simulator
- **THEN** returned telemetry still includes plant world/contact/GRF channels while recorded replay ticks stay kernel-exact

### Requirement: Bit-exact replay over MCP
`recording_control(start)` → step → `stop` → `verify_replay` SHALL report an exact match for all recorded ticks.

#### Scenario: Replay verification through MCP
- **WHEN** at least 80 ticks are recorded via MCP and then verified
- **THEN** `exact_match` is true and `matched_ticks == total_ticks`

### Requirement: GUI/headless parity
The GUI's in-process MCP host and the headless MCP host SHALL step via the same fixed-step transport method.

#### Scenario: Same tool behavior in GUI and CLI
- **WHEN** the same MCP tool is invoked against the GUI and the headless server
- **THEN** both use the fixed-step control path
