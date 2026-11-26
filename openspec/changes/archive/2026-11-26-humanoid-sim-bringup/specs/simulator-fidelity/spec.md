# Spec Delta

## ADDED Requirements

### Requirement: Tuned two-point heel and toe contact fidelity
The simulator's 2-point heel+toe contact parameters SHALL be tuned for the 28-DoF humanoid's foot geometry and mass distribution so `toe_pitch` push-off, heel-strike, and realistic-sensor `walk_nominal` / `walk_shove` runs maintain bounded slip, zero flight ticks, and accurate single-support GRF.

#### Scenario: Two-point push-off without excessive slip
- **WHEN** the 28-DoF humanoid executes `walk_nominal` with `toe_pitch` push-off active
- **THEN** heel and toe contacts share support transitions smoothly with accumulated slip within acceptance bounds and no spurious flight ticks

#### Scenario: Shove recovery under two-point contact
- **WHEN** `walk_shove` injects a lateral or sagittal CoM impulse on the 28-DoF humanoid
- **THEN** two-point foot contact and UVC/SLIP stepping recover balance within the acceptance window without latching a fall
