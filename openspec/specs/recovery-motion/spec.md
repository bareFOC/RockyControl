# recovery-motion Specification

## Purpose

Provides pre-computed get-up motions for each fall orientation and a consent-gated recovery flow that safely returns the humanoid robot from a fall to a standing guard stance.

## Requirements

### Requirement: Orientation-specific get-up library
The Keyframe Action Studio SHALL expose a set of pre-computed get-up actions, one for each supported fall orientation (prone, supine, left-side, right-side), each composed of multiple keyframes that transition the robot from a fallen posture through a push-off and kneeling stage to a stable standing guard stance.

#### Scenario: Library available by default
- **WHEN** the action studio initializes with the bundled robot model
- **THEN** all four get-up actions are present and selectable by name

#### Scenario: Library survives model reload
- **WHEN** a new URDF model bundle is loaded into the studio
- **THEN** the four get-up actions remain available

#### Scenario: Get-up poses respect joint limits
- **WHEN** a get-up action is authored or played
- **THEN** every joint command is within the robot model's joint limits and calibration soft limits

### Requirement: Recovery trigger consent
Autonomous recovery SHALL be disabled by default and SHALL only proceed when an operator has enabled an auto-recovery option and confirmed the recovery action.

#### Scenario: Fall with auto recovery disabled
- **WHEN** a fall is detected and auto recovery is not enabled
- **THEN** no recovery motion is started and the robot remains in its safety state

#### Scenario: Operator-initiated get-up
- **WHEN** an operator confirms the Get Up action after a fall
- **THEN** the recovery sequence starts using the motion matching the detected fall orientation

#### Scenario: Autonomous get-up with consent
- **WHEN** auto recovery is enabled, an operator has confirmed consent, and the robot is stable
- **THEN** the recovery sequence starts automatically

### Requirement: Safe recovery sequencing
The recovery flow SHALL clear the latched fall fault, re-enter a torque-enabled drive mode, and start the get-up action in a strict ordered sequence across multiple control frames, never bypassing existing safety interlocks.

#### Scenario: Ordered recovery
- **WHEN** recovery is triggered
- **THEN** the fault is cleared first, then torque-enabled drive mode is entered after the fault clears, and only then is the get-up action played

#### Scenario: Fall fault still latched
- **WHEN** a recovery step is about to start a get-up action while the fall fault is still active
- **THEN** the get-up action is blocked and safety state is preserved

#### Scenario: Re-fall during recovery
- **WHEN** a new fall is detected while a get-up action is playing
- **THEN** playback stops, torque is cut, and the robot returns to its safety state

### Requirement: Recovery completion
The recovery flow SHALL return to a normal idle state only after the get-up action finishes or is aborted, and SHALL leave the robot in a stable standing posture ready for operator command.

#### Scenario: Successful get-up
- **WHEN** a get-up action completes
- **THEN** the robot stands in a guard stance and the recovery state is cleared

#### Scenario: Aborted get-up
- **WHEN** an operator stops a get-up action
- **THEN** playback stops and the robot transitions to a safe non-driving state
