# Spec Delta

## Purpose

Experiments can be captured as deterministic scripts (packets, plant events and a PRNG seed) and replayed from a clean reset, so tuning regressions are reproducible in CI.

## ADDED Requirements

### Requirement: Experiment script capture
The system SHALL capture a simulation experiment as a script containing the exact host packet stream, timed plant events (shoves, fault toggles, recovery progress, reset-upright, virtual hand), the seeded sensor-noise configuration, and a clean-reset marker. Host packets SHALL be the typed post-decode commands the kernel consumed, not frame-timing-derived packets.

#### Scenario: Script captures a shove and fault toggle
- **WHEN** an experiment applies a shove at tick 40 and toggles a fault at tick 80
- **THEN** the script records both events with their tick indices and the exact packet consumed on each tick

### Requirement: Deterministic replay from clean reset
The runner SHALL reset the experiment (kernel, plant, faults, wire, recorder and host command sequencing) before replay, then feed the script's packets and events at their recorded tick indices. Re-running the same script SHALL produce bit-identical recorded ticks and a bit-exact `verify_replay` result.

#### Scenario: Two runs of the same script match
- **WHEN** a script is run twice with a reset between runs
- **THEN** both runs report `exact_match` with the same `matched_ticks` and identical telemetry

#### Scenario: Host sequencing reset prevents seq drift
- **WHEN** a script is replayed after an experiment reset
- **THEN** the kernel sees the same `HostToMcu.seq` values as the original run, independent of frame timing

### Requirement: Headless MCP execution
MCP SHALL provide a `run_experiment` tool that executes a JSON script against the headless simulation and returns a `ReplayVerification`. The tool SHALL work over both stdio and HTTP MCP servers.

#### Scenario: CI regression via MCP
- **WHEN** CI calls `run_experiment` with a committed script file
- **THEN** the response reports `exact_match: true` or identifies the first diverging tick

### Requirement: Backward-compatible logs
Additive fields on `CopperReplayLog` SHALL default so existing `.copper` logs (without events/seed) remain parseable and verifiable. `verify_deterministic_replay` semantics SHALL be unchanged.

#### Scenario: Old log still verifies
- **WHEN** a v1 log without the new fields is loaded and replayed
- **THEN** verification succeeds as before
