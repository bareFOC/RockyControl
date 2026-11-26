# Spec Delta

## ADDED Requirements

### Requirement: 28-DoF golden fixtures and sim-verify pass
The committed golden replay fixtures (`walk_nominal.copper`) and scripted experiments (`walk_nominal.json`, `walk_shove.json`, `fall_recover.json`) SHALL reflect the REP-103-normalized, tuned 28-DoF humanoid and SHALL pass `cargo run -p xtask -- sim-verify` with `exact_match` replay verification.

#### Scenario: Golden sim-verify gate passes on 28-DoF humanoid
- **WHEN** `cargo run -p xtask -- sim-verify` runs after 28-DoF bringup
- **THEN** all host, firmware-core, RP2350, WASM, scripted experiment, and `.copper` replay checks pass with zero violations
