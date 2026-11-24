# Proposal

## Why

Running is a v2/BLDC + torque-control capability (per the architecture assessment): the v1 N20 gearboxes cannot deliver the ~4× body-weight push-off forces for even a 2–4 cm hop. The ESP32-P4 scaffold must mature before dynamic gaits can be validated. This stage prepares the algorithm layer in the shared kernel so that when v2 hardware is available, running is a configuration, not a rewrite.

## What Changes

- Add a SLIP running template with torque-based leg thrust: spring $k≈670$ N/m, target hop 2–4 cm (takeoff velocity 0.63–0.89 m/s), stance time 120–150 ms (>24 ticks at 200 Hz) — scaled to 1–2 kg.
- Flight detection from near-zero motor current + IMU free-fall signature with 10–15 ms debounce.
- Landing compliance via torque-mode impedance: virtual leg spring 800→4000 N/m over 25–40 ms, damping 40–60 N·s/m.
- Scalar CBF safety projections (torso tilt, CoM velocity, support-relative capture point) shared with P2, with explicit projection (no QP).
- All running code gated behind a `RunningMode` control flag/feature so v1 builds remain unaffected.

## Capabilities
### New Capabilities
- `dynamic-locomotion`: SLIP-template running with thrust control, flight detection, compliant landings, and CBF safety for the torque-capable hardware generation.

### Modified Capabilities
- `bipedal-walking`: shared phase clock and CBF filters from P2 are reused.

## Impact
- `rk-control` (new `slip.rs`, kernel integration, gains), `rk-proto` (running mode/gains), v2 board BSP (torque mode). Depends on ESP32-P4 BSP bring-up (separate track) for hardware validation; simulation validation possible via a torque-capable plant mode.
- Risk: hardware-gated — simulation can validate algorithms, but real running requires v2.
