# Simulation acceptance — Rocky Mk1

This is the single source of truth for what "verified in simulation" means before
hardware work starts. The evidence is committed, reproducible and checked by CI;
it is not a manual claim in a chat log.

## Acceptance matrix

All measurements come from the deterministic 200 Hz software twin
(`CopperSimRunner` + the shared `ControlTask`), with a 21-joint Rocky Mk1 and a
4-second (800-tick) walk at `WalkCmd.vx = 0.5`, `cadence_hz = 1.55`. The
effective velocity command is `vx × 0.5 = 0.25 m/s`. The "fixture" column is
the value produced by the committed script in this tree; it agrees with the
measured P1–P4 results in the second column.

| Behavior | Test / evidence | Measured result (P1–P4) | Committed fixture |
| --- | --- | --- | --- |
| Forward locomotion tracks command | `committed_scripts_replay_exactly_and_meet_quality_bounds` (`walk_nominal.json`) | 0.261 m/s vs 0.25 m/s command (±10%) | 0.2615 m/s |
| Torso stays upright while walking | same test, `WalkQuality::peak_tilt_rad` / `steady_tilt_rad` | peak 0.034 rad, steady 0.028 rad | peak 0.0368 rad, steady 0.0278 rad |
| Lateral drift is bounded | same test, `lateral_drift_m` | 0.025 m (< 0.03 m) | 0.0167 m |
| Single support carries body weight | same test, `single_support_grf_ratio` | 0.944 mg (within ±15%) | 0.9445 mg |
| No steady flight phase | same test, `flight_ticks` | zero steady flight | 1 tick (plant initialization only), zero during gait |
| Actuator current stays below advisory | same test, `max_current_ma` | 1786 mA nominal; 1839 mA with realistic noise | 1786 mA nominal / 1839 mA shove script |
| Capturability margin stays mostly positive | same test, `margin_negative_fraction` | 22% negative (< 30%) | 20.9% nominal / 21.1% shoved |
| Mid-walk shove recovers without latch | same test (`walk_shove.json`), final telemetry fault flags | no fall / over-current / watchdog latch | confirmed; steady tilt 0.0338 rad |
| Fall → get-up → upright | same test (`fall_recover.json`), final telemetry | fall latches, recovery returns upright | peak tilt 1.21 rad, final pitch −0.008 rad, fall fault cleared |
| Deterministic replay | `golden_copper_log_verifies_bit_exactly` | bit-exact | 800/800 ticks, `exact_match: true` |
| Firmware/shared kernel compile | xtask `sim-verify` / CI `thumb-matrix` job | RP2350 default + hardware features | all checks pass |

Quality bounds asserted in the test are the acceptance bands (speed ±10%,
peak tilt < 0.08 rad, steady tilt < 0.03 rad, lateral drift < 0.03 m, single
support GRF within ±15%, flight ≤ 1 initialization tick, current < 1950 mA,
negative margin < 30%, CoM height 0.72 ± 0.03 m). The shove script uses the
disturbance bands (peak tilt < 0.35 rad, steady < 0.05 rad, lateral < 0.08 m).

## What replay verifies — and what it does not

`verify_deterministic_replay` re-executes the recorded `ControlTick` stream
through a fresh shared `ControlTask`: each tick's `rx_cmd`, `rx_state`, wire
error counts, telemetry output, servo command and recorded snapshot must match
bit-for-bit (`f32`). Because firmware and simulator both use this wrapper, a
control-kernel or wrapper-only change cannot silently diverge from the log.

Replay **does not** verify:

- host frame pacing, render-loop timing, UI thread scheduling or wall-clock
  latency — the script is the complete input and every tick is a fixed 5 ms step;
- the software plant's physics fidelity, sensor-noise model or contact model —
  those produce the inputs that are recorded, but replay only re-runs the kernel;
- COBS/UDP/USB framing timing, radio/serial throughput or real I3C bus behavior;
- real IMU bias/drift, motor thermal behavior, gear backlash or ground friction
  on hardware, and therefore not real-world fall recovery success;
- Android/WASM/desktop frame-derived packets — those are excluded from the
  scripted path by construction.

The simulation is a software twin of the control loop, not a substitute for
hardware bring-up. All hardware-only work remains out of scope.

## Committed fixtures

| File | Ticks | Purpose |
| --- | --- | --- |
| `tests/experiments/walk_nominal.json` | 800 | clean-sensor forward walk, P4 quality band |
| `tests/experiments/walk_shove.json` | 800 | realistic sensor profile + shove at tick 300 |
| `tests/experiments/fall_recover.json` | 400 | fall at tick 30, consented get-up sweep, upright end state |
| `tests/experiments/walk_nominal.copper` | 800 | golden replay log generated from the nominal walk |

Scripts use `rk_sim::ExperimentScript` serde shape (`seed`, `events`,
`packets`, `max_ticks`); events are `TimedSimEvent { tick_index, kind }` with
`SimEventKind::{SetFaults, Shove, Recovery, ResetUpright, VirtualHand}`.

The fixtures are loaded through the native MCP headless host's confined-file
`run_experiment` path (`AutomationRoots::read_root` → `file_path` in the tool
arguments). The same tests re-run the script through `CopperSimRunner` to read
`WalkQuality`, since the MCP tool intentionally reports only the replay
verdict. To regenerate fixtures after a deliberate behavior change:

```sh
RK_GENERATE_ACCEPTANCE_FIXTURES=1 cargo test -p rk-mcp --lib \
  acceptance_experiments::generate_fixtures -- --ignored --nocapture
```

## Running the gate

Locally, from the workspace root:

```sh
cargo run -p xtask -- sim-verify   # full acceptance gate (same as `check`)
cargo run -p xtask -- check        # alias of sim-verify
cargo run -p xtask -- no-std       # no_std core crates
cargo run -p xtask -- test         # host tests only
cargo run -p xtask -- wasm         # WASM cross-check only
cargo run -p xtask -- firmware     # firmware-core tests + RP2350 feature matrix
cargo run -p xtask -- experiments  # committed MCP headless experiments + golden replay
```

`sim-verify` runs, in order: `cargo test --workspace`, shared firmware-core
tests, RP2350 thumb checks (default, `usb-net,i3c-servo-bus,imu-lsm6dsv`, and
`usb-net,bench-loopback`), `cargo check -p rk-ui --target wasm32-unknown-unknown`,
and the committed-script tests. The RP2350 crate names its IMU feature
`imu-lsm6dsv` (not `imu-lsm6ds`).

In CI the same steps are split across jobs in
`.github/workflows/sim-verify.yml` (push, pull request, manual dispatch):
`host-tests`, `firmware-core`, `thumb-matrix`, `wasm-check` and `experiments`.
Any failure fails the workflow; `experiments` asserts `exact_match`, recorded
tick counts, quality bounds and golden-log replay.
