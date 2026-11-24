# Spec Delta

## Purpose

World-frame simulation with foot contacts, ground reaction, and body translation that closes the loop between commanded stepping and visible locomotion, driving both the controller and the 3D viewport.

## ADDED Requirements

### Requirement: World-frame robot state
The simulation SHALL maintain CoM position and velocity in the ground plane, derived from the same joint geometry used by the controller, and SHALL expose them to the controller as measured feedback.

#### Scenario: Stepping produces translation
- **WHEN** forward walking is commanded for several seconds
- **THEN** the simulated CoM advances forward and the controller observes nonzero CoM velocity

#### Scenario: Controller and plant share geometry
- **WHEN** the plant computes CoM and foot positions
- **THEN** it uses the shared kinematic/mass tables rather than a parallel model

### Requirement: Foot contact and ground reaction
The simulation SHALL model per-foot contact state, ground reaction force, penetration, and slip during walking, and SHALL derive body tilt from contact moments instead of scripted gait sway.

#### Scenario: Alternating contacts while walking
- **WHEN** the robot walks
- **THEN** left and right contact flags alternate and ground reaction remains bounded and positive

#### Scenario: No synthetic sway
- **WHEN** gait phase changes without a corresponding contact moment
- **THEN** body tilt does not change solely due to the phase signal

### Requirement: Visible locomotion
The 3D viewport SHALL translate the robot according to simulated world position, and the HUD SHALL display achieved velocity/displacement and contact state alongside commanded values.

#### Scenario: Forward motion is visible
- **WHEN** the robot walks forward in simulation
- **THEN** the rendered root moves forward and the HUD shows achieved vs commanded speed

### Requirement: End-to-end walking verification
An automated test SHALL drive the full simulated control loop with forward walking enabled and assert real translation, alternating contacts, and bounded tilt.

#### Scenario: Walking regression test
- **WHEN** the walking test runs
- **THEN** it fails if the robot marches in place, contacts do not alternate, or tilt exceeds the fall threshold
