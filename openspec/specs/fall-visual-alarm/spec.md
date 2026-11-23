# fall-visual-alarm Specification

## Purpose

Makes fall events unmistakable in the 3D workstation by animating the robot model in pulsing red and providing a dedicated recovery panel that reports fall state and exposes recovery controls.

## Requirements

### Requirement: Pulsing red fall highlight
The 3D viewer SHALL render the robot model in a red highlight that pulses over time whenever a fall event is active, and SHALL take precedence over other per-joint color effects while active.

#### Scenario: Fall event active
- **WHEN** a fall event is active and the robot is visible in the 3D viewer
- **THEN** the robot model is rendered in red with a visible periodic pulse

#### Scenario: Precedence over other tints
- **WHEN** a fall event is active while joint heat tinting or joint selection highlighting would also apply
- **THEN** the red fall highlight overrides those tints

#### Scenario: Fall event cleared
- **WHEN** the fall event is cleared after recovery
- **THEN** the robot returns to its normal materials and color effects

### Requirement: Recovery HUD panel
The control screen SHALL provide a dedicated recovery panel that displays fall status, the detected fall orientation, an auto-recovery enable control that is off by default, and an operator Get Up control.

#### Scenario: Panel shows fall state
- **WHEN** a fall is detected
- **THEN** the recovery panel shows that a fall is active and which orientation was detected

#### Scenario: Auto recovery defaults off
- **WHEN** the operator opens the recovery panel for the first time
- **THEN** auto recovery is shown as disabled

#### Scenario: Operator triggers get-up
- **WHEN** the operator activates the Get Up control
- **THEN** the recovery flow begins using the displayed fall orientation

### Requirement: Alarm visibility across screens
The fall alarm state SHALL be reflected consistently wherever the robot is rendered in the workstation.

#### Scenario: Multiple viewports
- **WHEN** a fall event is active and both the main control viewport and the dashboard preview are visible
- **THEN** both show the robot in the red fall alarm state
