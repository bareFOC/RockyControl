# agentic-balance-tuning Specification

## Purpose

An MCP agent can measure, evaluate, and reset the simulator's balance/walking quality deterministically, enabling autonomous tuning.

## Requirements

### Requirement: Full walk quality observability
The system SHALL expose all `WalkQuality` metrics as structured JSON through MCP, for both live-stepped and scripted simulation runs. Scripted runs SHALL use the currently tuned `BalanceGains` and SHALL report quality alongside replay verification.

#### Scenario: quality snapshot
- **WHEN** an agent calls `quality_snapshot` after a walk run
- **THEN** the response contains all 14 quality metrics as JSON numbers and `null` on non-sim transports

#### Scenario: tuned scripted run
- **WHEN** an agent sets balance gains and calls `run_experiment` with a walk script
- **THEN** the script executes with those gains and the response includes both `verification` and `quality`

### Requirement: Machine-readable acceptance bounds
Acceptance bounds SHALL be a single serde-serializable definition in the simulator, reused by tests and MCP tools. Tool responses SHALL report metrics, bounds, violations and an overall pass flag.

#### Scenario: Acceptance evaluation
- **WHEN** an agent calls `quality_snapshot` after a nominal walk
- **THEN** the response includes `bounds`, `violations` (empty) and `pass: true`

#### Scenario: Violation detection
- **WHEN** a gain setting produces excessive tilt
- **THEN** the response lists that specific violated bound and reports `pass: false`

### Requirement: Agent recovery primitives
MCP SHALL provide reset-to-defaults for balance gains and named execution of committed experiment fixtures, without requiring agents to author large JSON scripts.

#### Scenario: Reset gains
- **WHEN** an agent calls `balance_reset_defaults`
- **THEN** all 14 balance gains return to their documented defaults on both host and kernel after the next tick

#### Scenario: Named experiments
- **WHEN** an agent calls `experiment_list` then `experiment_run_named` with a listed name
- **THEN** the named committed script runs and returns verification and quality

### Requirement: Structured tuning errors
Invalid gain writes SHALL return machine-readable errors identifying the offending field and its allowed bounds.

#### Scenario: Self-correction
- **WHEN** an agent sets a gain outside its allowed range
- **THEN** the response contains a structured error with `field`, `value`, and `bounds` so the agent can retry
