# Design

## Context

P2 provides walking and a phase clock. Running needs torque commands (v2/BLDC + ESP32-P4), unavailable on v1 N20 gearboxes. The ESP32-P4 scaffold needs bring-up before hardware validation; simulation can validate the algorithms with a torque-capable plant mode.

## Goals / Non-Goals

**Goals:** SLIP thrust control, flight detection, compliant landings, CBF safety, deterministic, hardware-gated. **Non-Goals:** full ESP32-P4 BSP bring-up (separate track), learned policies, vision.

## Decisions

1. New `slip.rs` in rk-control: spring F=k(L0−L), k≈670 N/m, hop 2–4 cm → v_TO 0.63–0.89 m/s, stance 120–150 ms; torque command from thrust.
2. Flight detection: current < threshold + IMU free-fall (|gravity|<~2 m/s² from attitude), 10–15 ms debounce.
3. Landing impedance: virtual spring 800→4000 N/m over 25–40 ms, damping 40–60 N·s/m, torque mode.
4. CBF: scalar projections (tilt, CoM vel, capture point), κ≈3 s⁻¹, no QP.
5. Gate behind `RunningConfig` flag; v1 builds compile with stub/no-op.
6. Validation via torque-capable sim plant; real running requires v2 hardware.

## Risks / Trade-offs
- [Hardware-gated] → simulation only until ESP32-P4 mature.
- [Printed parts fragile] → conservative 2–4 cm hops, CBF margins.
