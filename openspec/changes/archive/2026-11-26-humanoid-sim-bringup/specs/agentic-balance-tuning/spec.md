# Spec Delta

## ADDED Requirements

### Requirement: 28-DoF humanoid acceptance bounds and gain defaults
The simulator and `rk-mcp` acceptance bounds SHALL be calibrated for the 28-DoF humanoid geometry (`h_com_m ≈ 0.25 m`, `foot_half_length_m`, `foot_margin_m`, and 28-actuator current/torque limits), and `balance_reset_defaults` SHALL restore the tuned 28-DoF defaults.

#### Scenario: Tuned 28-DoF nominal walk passes MCP bounds
- **WHEN** an agent calls `experiment_run_named` for `walk_nominal` or `walk_shove` after `balance_reset_defaults`
- **THEN** `quality_snapshot` reports `violations: []` and `pass: true` against the 28-DoF acceptance bounds

#### Scenario: Gain sweep reflects 28-DoF CoM height
- **WHEN** an agent sweeps `BalanceGains` via `rk-mcp` around the 28-DoF defaults
- **THEN** `h_com_m` and `hip_height_m` bounds permit and validate the `≈ 0.25 m` CoM operating point
