# Tasks

> Subagent-delegable. No back-compat needed.

## 1. SLIP controller (OWNER: control agent)
- [x] 1.1 Add `crates/rk-control/src/slip.rs`: SLIP thrust law, flight state, landing impedance; gate behind RunningConfig. Verify `cargo test -p rk-control`.
- [x] 1.2 Add CBF scalar projections (tilt, CoM vel, capture point) shared with P2. Verify tests.

## 2. Protocol + sim (OWNER: host agent; depends on 1)
- [x] 2.1 Add running config/gains to rk-proto; torque-capable plant mode in rk-sim for validation. Verify `cargo test -p rk-proto -p rk-sim`.
- [x] 2.2 Add running tests (hop height, flight detection, landing ramp, CBF clamp). Verify tests.

## 3. Validation + firmware gating (OWNER: integrator; depends on all)
- [x] 3.1 Full host suite + firmware-core + RP2350 thumb checks (v1 unchanged) + lint + strict validate.
- [x] 3.2 Document that real running requires v2 hardware; defer ESP32-P4 bring-up.
