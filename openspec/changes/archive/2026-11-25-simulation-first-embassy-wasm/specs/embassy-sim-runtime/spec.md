# Spec Delta

## Purpose

Runs the shared Embassy control application deterministically in WASM and native simulation with a mock clock and physics I/O backend, while the identical task runs on real MCU hardware, so simulation results predict firmware behavior.

## ADDED Requirements

### Requirement: One control task across targets
The control application SHALL expose one Embassy task that performs the fixed-period control step, and both the MCU firmware and the WASM/native simulation SHALL execute that same task. Backend-specific sensing and actuation SHALL be reached through one shared I/O trait.

#### Scenario: Same task in browser and firmware
- **WHEN** a control tick is executed in the browser simulator and on the MCU
- **THEN** both call the same Embassy control task with backends selected by build target

#### Scenario: No duplicate control loop
- **WHEN** the simulation advances
- **THEN** it does not use a separate control-loop implementation from the firmware

### Requirement: Deterministic simulated clock
The simulation SHALL advance the Embassy clock by fixed 5 ms steps and poll the executor to quiescence before the next step; task execution SHALL NOT depend on wall-clock time or render frame rate.

#### Scenario: Fixed-rate stepping independent of FPS
- **WHEN** the browser renders at any frame rate
- **THEN** the control application still executes at exactly 200 Hz in simulated time with bounded catch-up per frame

#### Scenario: Reproducible task order
- **WHEN** the same recorded inputs are replayed
- **THEN** the control task produces bit-identical telemetry regardless of host timing

### Requirement: Fixed-step app integration
The app SHALL drive the deterministic harness with a fixed-step accumulator and SHALL report the achieved control tick rate rather than the render frame rate.

#### Scenario: Slow frame does not corrupt simulation
- **WHEN** a frame takes longer than one control period
- **THEN** the accumulator runs multiple fixed steps with a bounded cap and reports the achieved rate

#### Scenario: Honest rate display
- **WHEN** the UI shows link/control rate
- **THEN** it shows the number of control ticks executed per simulated second, not FPS
