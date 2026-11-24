# persistent-simulation-executor Specification

## Purpose

The simulation owns a long-lived deterministic Embassy executor, allowing repeated fixed-step calls within one process without corrupting global timer state.

## Requirements

### Requirement: One executor per simulation session
A simulation transport SHALL create one deterministic Embassy executor and control task per session and SHALL reuse them for every fixed-step call. No fixed-step call SHALL spawn a new executor against the process-global mock clock.

#### Scenario: Repeated tool calls remain stable
- **WHEN** an MCP client makes many sequential stepping tool calls in one server process
- **THEN** no timer-queue assertion/panic occurs and every call returns a valid response

#### Scenario: Control state persists across calls
- **WHEN** a fixed-step call is followed by another
- **THEN** the kernel's internal state (sequence numbers, tick index, watchdog timers) continues from the previous call rather than resetting

### Requirement: Command and telemetry handoff without re-spawn
The persistent control task SHALL receive queued host commands and publish telemetry/recorded ticks through shared state that does not require re-spawning the task.

#### Scenario: Host command applied on next step
- **WHEN** a new command is queued between two fixed-step calls
- **THEN** the running control task consumes it on the next tick

#### Scenario: Replay stays bit-exact
- **WHEN** ticks are recorded across many fixed-step calls
- **THEN** replay verification reports an exact match with no per-call resets
