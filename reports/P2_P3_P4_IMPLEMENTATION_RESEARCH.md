# P2 / P3 / P4 Implementation-Level Research — Rocky Mk1
### Read-only technical report to inform future OpenSpec proposals (walking → running → combat)

**Date:** 2026-11-24. **Status:** research only; no code changed.
**Basis:** current source (`rk-control/src/{uvc,estimator,lib}.rs`, `rk-model/src/studio.rs`,
`rk-proto/src/lib.rs`) plus the existing reports `ROCKY_ALGORITHM_ASSESSMENT.md`,
`REDUCED_ORDER_WBC_IMPLEMENTATION.md`, `IMPLEMENTATION_CONTROL_RESEARCH.md`,
`HUMANOID_WALKING_RESEARCH_REPORT.md`, `HUMANOID_COMBAT_RESEARCH.md`,
`DYNAMIC_LOCOMOTION_RESEARCH_REPORT.md`.

> **Critical framing.** The "SimpleFOC torque/velocity/position servos over I3C" in the task
> statement is the **v2** actuator stack. The assessment (§1, §6) decided that **v1 = N20 geared
> brushed with software current limiting only** and that torque-mode impedance, momentum
> observers, and running are **v2/BLDC + ESP32-P4 only**. Today the ESP32-P4 is a scaffold
> (no USB, WDT, I3C hooks, PSRAM). Therefore: **P2 is implementable on v1 now; P3 and P4's
> forceful parts are gated on v2 hardware maturity.** Every P3/P4 sub-item below is tagged
> `[v1]`, `[v2]`, or `[PC]` accordingly. Be honest about this in proposals — a P3 proposal
> that ignores the actuator split will be rejected in critique.

---

## 0. Current-code ground truth (what actually exists)

### 0.1 `crates/rk-control/src/uvc.rs` — phase machine, gait phase, foot placement
- `UvcPhase`: `Idle → CrouchInit(0.08s) → CalibrateImu(0.12s) → ReadyWatch →
  ActiveCatchStep → PostStepSettle(0.22s) → ReturnToNeutral(0.42s) → RecoverySettle(0.20s)
  → ReadyWatch`, plus protective `FallTuck` (40 ms then `fall_cutoff_ready`).
- `gait_phase_rad` is **synthetic**: during `ActiveCatchStep` it is `progress*PI` (right support)
  or `PI + progress*PI` (left support) (L544–549). It is *not* a free-running oscillator and
  does not advance in `ReadyWatch`. This is the seam where a real Raibert-style phase clock
  (P2) must attach.
- Step duration: `if wants_walk { (0.5/cadence).clamp(0.16,0.45) } else { 0.18 }` seconds
  (L462–466). With `WalkCmd::default().cadence_hz = 1.55` → **0.32 s** — squarely inside the
  task's 0.2–0.35 s target. `cadence_hz` clamp is `[0.6, 2.8]`.
- Foot placement today: support foot `dxi/dyi` from spherical UVC projection (I-only on tilt,
  with rate lead `theta + 0.3*theta_dot`, L810–821); commanded stride superposed as
  `target_walk_x = vx * max_dx * 0.68 = vx*0.1088 m` (L566); swing foot is a mirror
  `dxis = -dxi + dcm.swing_dx_m` (L580); swing clearance `max_foot_lift*sin(pi*progress)` with
  `max_foot_lift = 0.095 m` (L598). Support swap at landing scales `dxi/dyi *= 0.65` (L609–614).
- **There is no Raibert/capture foot-placement law**, no step-timing adaptation, no LIPM-MPC.
  `LegGeometry::rocky_mk1()`: `thigh=shin=0.32`, `hip_ankle_offset=0.06`,
  `half_hip_spacing=0.095`, `nominal_height=0.62`, `min_height=0.46`, `max_reach=0.635`,
  `max_dx=0.16`, `max_dy=0.15`, `max_swing_dx/dy=0.22`, `max_foot_lift=0.095`, `max_sway=0.075`.
  Note: these are **hip-to-ankle** numbers for a 45 cm robot; the effective CoM height used by
  the estimator is ~0.48 m (`KinematicEstimator::new`), not the report's 0.25 m. See §2.1 for
  the resulting ω discrepancy — this is the single most important numeric decision for P2.

### 0.2 `crates/rk-control/src/estimator.rs` — what state is available
- Lumped-mass CoM from IMU attitude + arm shoulder angles: `com_pos[1] ≈ h_hip*0.82+0.06`,
  `com_y` from `cos(pitch)*cos(roll)`; arm offset from shoulder roll/pitch (L106–134).
- CoM velocity: 0.55×finite-difference + **0.45×gyro lever term** `r*omega_imu`, EMA α=0.30
  at 200 Hz, air-gated on `contact_mask == 0` (L136–178).
- `omega = sqrt(9.81/h_com)`, `xi = c + c_dot/omega`, kinematic ZMP `c - (h/g)*a`,
  both clamped; `foot_half=0.035 m`, `foot_margin=0.028 m`, lateral margin ×2.4 in double
  support (L180–205).
- `DcmSwingHint { margin_breached, swing_dx_m, swing_dy_m }` — **only modulates swing foot**
  (`uvc.rs` L579–582). Explicitly never drives the stance hip (`P1a DOE partitioning`).
- `valid_mask`: v1 sets `COM|DCM|imu_source` but **never `GRF` or `MOMENTUM`** (L223–241).
  This is the exact hook P4's momentum observer must extend (with a new validity bit/gate).

### 0.3 `crates/rk-control/src/lib.rs` — kernel integration
- `DirectServoKernel::step()` (L294) calls estimator (L481) then
  `self.uvc.step_with_dcm(dt, imu, walk, gains, dcm_hint)` (L509–511) whenever
  `uvc_engaged || tilt_exceeded || phase==FallTuck`; blends leg targets with `uvc_blend`
  ramp (8/s normal, 24/s catch/tuck), L526–572. Upper body: waist yaw + shoulder deltas added
  on top of host targets (L567–571).
- Safety is **downstream** of balance: soft limits, slew clamp
  (`BALANCE_SLEW_RAD_PER_SEC = 5.5` when balance/walk active vs base 2.0), torque/velocity
  ceilings, ukemi scaling (`ukemi_torque_scale` 0.15), `JointCommandMode::{Position,Velocity,Torque}`
  with per-joint `torque_nm` (L600–696). `DEFAULT_MAX_TORQUE_NM=1.8`, `DEFAULT_MAX_VELOCITY=8.0`.
- Fall/TILT_FALL_CUTOFF latch, 100 ms deadman, ClearFaults resets UVC + estimator + blend.
- **Any new controller must live inside `DirectServoKernel::step`** (per assessment §2.4 /
  simplify-mcu-runtime archived change), not as a new Copper task; replay remains host-side in
  `rk-fw-core/src/replay.rs` + `rk-sim`, verifying `ControlTask` wrapping this kernel.

### 0.4 `crates/rk-model/src/studio.rs` — action/keyframe system
- `Keyframe { label, duration_ms, angles_rad: [f32; MAX_JOINTS] }`,
  `ActionMove { name, category, keyframes }`; `sample_action_move()` does C1 cubic smoothstep
  between keyframes with `rate_scale = 0.25 + 0.75*speed_limit_pct/100` (L57–107).
- `default_combat_library()` already contains **Guard, Left Jab (3 kf), Right Cross (3 kf),
  Front Kick (4 kf), Neutral Zero** with authored waist-yaw and arm/hip counter-rotation
  (L109–285). `default_recovery_library()` has 4 get-up moves ending at Guard.
- `StudioModel::play_move()` is host-side; `build_next_host_packet()` streams interpolated
  `target_rad` into `HostToMcu` at 200 Hz (L876–933). **No combos, no balance tails on strikes,
  no impact-aware optimization, no telemetry-triggered abort** today. `ingest_telemetry` only
  cancels playback on EStop (L935–941).
- `rk-mcp` has no numeric optimization/training tool — it is host automation/N MCP only.

### 0.5 `rk-proto` wire surface
- `WalkCmd { walk_enabled, balance_enabled, vx, vy, yaw_rate, stance_crouch, step_height,
  cadence_hz }` — normalized `[-1,1]` velocity commands, **not m/s**. Any P2 speed law must
  convert `vx` to m/s or change the wire (breaking change allowed per AGENTS.md).
- `BalanceGains` has `dcm_step_gain`, `foot_half_length_m=0.035`, `foot_margin_m=0.028`,
  `ukemi_*`, `compliance_scale`. New P2/P3/P4 gains can be appended (no backward compat needed).
- `ImuState` already carries `pitch/roll/yaw` rates and `gravity[3]`. `EstimatorTelemetry`
  has `com_pos/vel`, `dcm_xy`, `zmp_xy`, `foot_margin`, `grf_left/right` (always 0 on v1),
  `contact_mask`, `ukemi_active`, `valid_mask`. `RawServoFeedback` has per-joint
  `current_ma`, `velocity_rad_s`, `temp_c`. All needed P3/P4 sensor channels exist.

---

## 1. P2 — Robust walking

### 1.1 Concrete Raibert / capture foot-placement law with this robot's numbers

**Model.** LIPM: `c̈ = ω²(c − p)`, `ω = sqrt(g/h_com)`, `ξ = c + ċ/ω`, `ξ̇ = ω(ξ − p)`.
Two candidate `h` values exist in this repo:
- Report convention: `h≈0.25 m → ω≈6.26 rad/s` (assessment §4.4, WBC report header).
- Current estimator: `com_y ≈ 0.48 m` at nominal hip height 0.62 → `ω≈4.5 rad/s`.

**Recommendation:** do **not** silently reuse 6.3 with the estimator's 0.48 m CoM — that pairs
an inconsistent `ω` with `ξ` and biases foot placement. Either (a) use `ω` derived at runtime
from `com_pos[1]` as the estimator already does (authoritative; `xi` is computed with it), or
(b) if 6.3 is desired for tuning, redefine the estimator's CoM height to ~0.25 m. **P2 proposal
must state which.** The ~6.3 number is appropriate for a true 45 cm robot CoM; the estimator's
0.48 m looks like a hip-height proxy, not a CoM, and may itself deserve a P2 correction.

**Discrete exact DCM step** (WBC report §1.1; Caron `Pendulum::integrateIPM`):
```
phi = exp(ω T)                         // T = step time; precompute table indexed by T
xi_next = p + (xi - p) * phi
c_next  = c*cosh(ωT) + (ċ/ω)*sinh(ωT) - p*(cosh(ωT)-1)
```
With `ωT≈6.26*0.32≈2.0`, `exp(2)≈7.4` — a push DCM error grows 7× per step, so step
placement must be recomputed **every tick**, not once at touchdown.

**Foot placement law** (combine WBC §2.2 and assessment §4.4; numbers use ω=6.26, foot
half-length 0.035 m, usable margin 0.028 m, `T_s=0.32 s`, `max_dx=0.16 m`):
```
x_f = ξ_x + K_v * (ċ_x_des − ċ_x) / ω        // capture-point form
    = c_x + ċ_x*(T_s/2 + K_v/ω) ...           // equivalent Raibert form: c + v*T_s/2 + k_v*Δv
```
With `K_v≈1.5–2.5` (dimensionless), speed error 0.1 m/s → correction `0.1*2/6.26≈3.2 cm`
— exactly one usable foot margin. Commanded speed: `ċ_des = vx * v_max`, with `v_max` chosen
so that `Δx_f = vx*v_max*T_s/2 ≤ max_dx=0.16` → `v_max ≤ 2*0.16/0.32 = 1.0 m/s`. Be
conservative: **`v_max≈0.3–0.5 m/s`** (at 0.5 m/s, stride = 0.16 m = full `max_dx`). This is
the honest answer to "speed limits": current normalized `vx` maps to ~0.5 m/s max, not 1 m/s.
Lateral law mirrors with `max_dy=0.15` and double-support margin `0.028*2.4=0.067 m`.

**Clamp & capturability check:** `x_f ∈ [−max_dx, max_dx]`; if clipped, run the O(9) one-step
capture test (WBC §2.1): evaluate `xi_next` for 3×3 candidate foot positions, require
`|xi_next − p_f| < margin`; if none viable → shorten step (§1.3) and/or invoke ukemi/step
recovery. **Cost: <100 flops/tick.**

**Integration with UVC without breaking determinism** (the key design constraint):
- Keep strict DOF partitioning from P1a: **UVC = stance verticality I-only; ankle = gyro-rate P;
  DCM/Raibert = swing-foot landing only.** P2 replaces the current `swing_dx_m`/`swing_dy_m`
  hint from `estimator.rs` (`DcmSwingHint`) with a computed absolute landing target, but
  **still writes only `dxis/dyis` in `uvc.rs` L580–586**. Do not add a second stance-hip
  integrator.
- Attach a free-running **phase accumulator** `phi += 2π*f*dt` with left/right π offset
  (`cadence_hz` drives `f`; Ijspeert Hopf-lite / WBC §8) in `UvcController` state. On
  `ActiveCatchStep` entry, **phase-lock** `phi` to the existing `gait_phase_rad` calculation so
  replay of old logs is unchanged in structure; when `wants_walk`, let `phi` run continuously
  through `ReadyWatch` and drive support swaps instead of the `progress>=1` timer. Keep
  `gait_phase_rad` as the single telemetry field (`phi mod 2π`) — no wire change needed.
- Determinism: no wall-clock, no `rand`, fixed 200 Hz `dt` from caller, all new state in
  `UvcController`/`DirectServoKernel` structs, all math `f32` + `libm` (same as uvc.rs), all
  gains from `WalkCmd`/`BalanceGains` (wire) not globals. Replay in `rk-fw-core` records
  `ControlTask` I/O only; adding internal state is safe as long as `step()` is a pure function
  of `(dt, cmd, feedback, previous state)`. Add a test that replays a P2 tick log twice and
  compares `McuToHost` bit-for-bit.

### 1.2 Step timing adaptation (Khadiv) — minimal implementation
Khadiv et al. (T-RO 2020, arXiv:1704.01271) key result: **only the next step's timing and
location need optimization**; viability is preserved. Minimal MCU form (WBC §2, walking report §2.3):
```
Given current ξ (relative to current support p0), candidate landing p1 and step time T:
    xi_landing(T) = p1 + (xi - p1)*exp(ω*T)
Capture condition at end of next stance: xi_landing(T) ∈ S(p1), i.e. |xi_landing - p1| ≤ margin.
Solve for maximal admissible T:   T_max = ln(1 + margin/|xi - p1|) / ω
Choose T_cmd = clamp(T_max * safety(0.8), T_min=0.16, T_nom=0.5/cadence).
```
Numerically with ω=6.26, |ξ−p1|=0.10 m, margin=0.028: `T_max=ln(1.28)/6.26≈0.039 s` —
**too short** (<24 ticks); this says the push is uncapturable in one step and must be handled
by hip strategy (CAM) + bigger step or ukemi. With |ξ−p1|=0.03 m: `T_max≈ln(1.93)/6.26≈0.106 s`
→ clamp to 0.16 s. Practical implementation: compute `T_max`, then `T_cmd = clamp(...)`, and if
`T_cmd == T_min` and still uncapturable → set `balance_gains.ukemi` path / FallTuck per existing
logic. **Also propagate `T_cmd` into the swing-foot clearance period** (`sin(pi*progress)` with
`step_dur` replaced by `T_cmd`) so the shortened step is physically executed. Add a
`step_time_s` field to telemetry (new wire field allowed). Complexity: one `logf`/`expf` via
`libm::logf/expf` (deterministic on M33F) or a 32-entry table — ~1 day of work.

### 1.3 Small f32 LIPM-MPC suitable for this MCU
From WBC §4: **cart-table** state `x=[c_x,c_y,ċ_x,ċ_y,c̈_x,c̈_y]`, input = jerk `u=[j_x,j_y]`,
`T=0.08 s`, `N=6` (0.48 s horizon — covers one 0.32 s step), update at **10–20 Hz**
(every 20–40 control ticks), hold first ZMP/CoM target between solves.
Dynamics per axis: `c⁺=c+T ṋ+T²/2 ä+T³/6 u`, `ṋ⁺=ṋ+T ä+T²/2 u`, `ä⁺=ä+T u`;
`p = c − (h/g) ä`. Cost weights `(w_v,w_p,w_j)=(10,1000,1)` (Caron defaults).
Constraints: ZMP box per support phase (4 rows/foot), terminal DCM inside final support
(Wieber/Scianca stability row), `u` box.
Solver: since x/y **separate**, solve two 1-D box-QPs with a tridiagonal Hessian via Thomas
algorithm + capped active-set (≤8 iterations) or ADMM with precomputed Cholesky of
`P+ρAᵀA` (15 iterations, ρ≈10·diag(P)). All arrays static `[N] f32`; scale positions by
`1/foot_half`, velocities by `T/foot_half` so Hessian O(1). Fallback to the analytic
capture law on non-convergence (no NaN/stall). ~10³ flops/solve. **This is [v2-friendly] but
also fits v1** — the blocker is not FLOPs but the need for reliable `ċ`; v1's estimator is
IMU+gyro derived and adequate with the DCM-form variant `x=[ξ_x,ξ_y,p_x,p_y]`, `u= ṗ`,
`N=6..10`, which is even smaller (14–22 vars). Recommend **DCM-form first**, cart-table later.

### 1.4 Tests to write (P2)
1. **Unit (rk-control):** capture law places `dxis` forward under `+vx` and forward lean; clamps
   to `max_dx`; one-step capturability grid rejects unreachable ξ; `T_cmd` shortens under a
   velocity impulse and respects `T_min=0.16`.
2. **Determinism:** replay a P2 tick log twice → `McuToHost` byte-identical; add a wrapper-mutation
   test that fails if the new state is not captured (mirrors existing replay verifier).
3. **Kernel-level:** walking alternates legs and `gait_phase_rad` advances monotonically through
   `ReadyWatch` (new behavior); DCM breach triggers shortened step rather than only swing
   modulation; ukemi/FallTuck still latch after repeated uncapturable pushes.
4. **SIL (`rk-sim`):** shove during single support → recovers within 2 steps without foot
   crossing (`dyi+dyis≥0`, existing anti-collide at uvc.rs L583); speed-step response settles with
   `|ξ−p|<0.028 m`; sweep `vx∈[-1,1]` and assert `|x_f|≤0.16`.
5. **Numeric parity:** `ω` from estimator vs hard-coded 6.26 — assert the value used matches the
   `ξ` definition (the mismatch check from §1.1).
Reuse the existing tests in `uvc.rs` L867–955 and `lib.rs` L952–1056 as templates.

### 1.5 P2 reuse vs build — complexity
- **Reuse:** `KinematicEstimator` (`estimator.rs`) — CoM/ξ/ZMP/margin/hint; `UvcController`
  phase machine + `solve_leg_ik` + swing clearance + anti-collide + support swap; `WalkCmd`/
  `BalanceGains`; `DirectServoKernel::step` blend/safety/ukemi; `rk-fw-core::replay` + `rk-sim`
  harness; existing unit tests.
- **Build new:** a `walking.rs` (or additions to `uvc.rs`) module with (a) free-running phase
  clock, (b) Raibert/capture placement with `v_max` mapping, (c) Khadiv timing law, (d) small
  DCM-form MPC solver with static buffers, (e) telemetry fields for `T_cmd`/`x_f`/MPC status;
  wire additions in `rk-proto`.
- **Estimated complexity:** **6–10 days** for v1 walking (Raibert + timing + tests) ; **+4–6
  days** for the f32 MPC; **+2–3 days** if the estimator CoM height must be corrected.
- **Key risks:** (1) ω/h inconsistency (biggest); (2) DOF fighting if placement is written to
  `dxi/dyi` instead of `dxis/dyis` (regresses P1a); (3) `dt` jitter from I3C stalls breaking
  replay — keep fixed 5 ms tick; (4) shortened steps below ~24 ticks lose touchdown debounce;
  (5) `logf/expf` parity on M33F vs host f32 — verify with the existing `sin_f32` parity test
  pattern.

---

## 2. P3 — Modest running **[v2 only]**

### 2.1 SLIP template scaled to a 2–4 cm hop, 1.5 kg
Assessment §4.8 already corrected the old 20 cm plan. Concrete numbers:
- `m=1.5 kg`, `L_0≈0.22 m` (leg length), `g=9.81`.
- Rest-load spring: `k = mg/(0.1 L_0) = 14.7/0.022 ≈ 670 N/m` (dimensionless
  `K = kL_0/(mg) ≈ 10`).
- Hop height `H=0.03 m` → takeoff velocity `v_z^TO = sqrt(2gH) = sqrt(0.588) ≈ 0.77 m/s`.
  For `H=0.02..0.04`: `v_TO≈0.63..0.89 m/s`.
- Natural frequency `ω_s=sqrt(k/m)=sqrt(670/1.5)=21.1 rad/s`; **full** SLIP stance
  half-period `π/ω_s≈149 ms` (single leg) / ~105 ms (two legs) — **not 75 ms**. Target
  `T_s≈120–150 ms` (>24 ticks at 200 Hz).
- Vertical impulse `J_z = m(v_TO − v_TD)`; symmetric hop `v_TD≈−v_TO` → `J_z≈2mv_TO≈2.3 N·s`,
  average thrust `≈J_z/T_s≈15–19 N` (~1–1.3× body weight — modest, within N20 peak but the
  *bandwidth* is the problem). Energy per hop `E=mgH≈0.44 J` (vs 2.94 J for 20 cm).
- Flight time `T_f=2v_TO/g≈0.13–0.18 s`; duty factor `β=T_s/(T_s+T_f)≈0.45` — a "grounded"
  run, not a sprint.
- Raibert foot placement for running: `x_f = x_hip + v_x*T_s/2 + k_v(v_x−v_des)` with
  `k_v≈0.05–0.2 s`; at `v_x=0.3 m/s`, `T_s=0.14` → `x_f≈0.042 m` ahead of hip, inside
  `max_dx=0.16`. Attitude: `τ_hip = −k_p θ − k_d θ̇` during stance using existing UVC
  pitch/roll + gyro damping (uvc.rs L725–743).
- **Actuator caveat:** 15–19 N average thrust through a ~0.05 m lever → ~0.75–0.95 N·m
  sustained per leg, near the 1.8 N·m ceiling; peak >2 N·m at 120 ms stance. v1 geared N20
  cannot render a clean spring law at 200 Hz — keep running on v2 BLDC torque mode.

### 2.2 Flight detection from IMU + zero motor current
- Signals already in `RawServoFeedback`: `current_ma[21]`, `velocity_rad_s[21]`, `imu.gravity[3]`,
  pitch/roll rates.
- Detector (WBC §6.3, assessment §6.5): 
```
I_leg = Σ_i |current_ma[i]| for leg joints (indices 9..20 on 21-DoF);
flight = (I_leg < I_th) && (|a_m| < 0.3*g)    // a_m from specific force / attitude-derived gravity
confirm with L → L_0 and ċ_z sign; debounce 2–3 ticks (10–15 ms); <5 ms debounce at touchdown.
```
- `I_th ≈ 0.1 * I_stance_peak` (store per-joint baseline at startup). **v2 only**: current→torque
  through BLDC is trustworthy; on v1 N20 gearbox stiction swamp the signal.
- Use the existing `uvc.contact_mask()` (bit 0=right, bit1=left, L327–340 of uvc.rs) as the
  *kinematic* contact prior; the current/IMU detector overrides it during flight. Add a
  `flight_mask` to telemetry (wire addition). Note `contact_mask()` currently returns `0b11`
  in all non-step phases — running needs a true air state (new `UvcPhase::Flight`, or extend
  the mask logic). This is a phase-machine change, so replay tests must be updated.

### 2.3 Landing impedance via torque control (SimpleFOC torque mode) **[v2]**
From IMPLEMENTATION_CONTROL_RESEARCH §3.3 and assessment §4.3 (rescaled!):
- Cartesian/virtual leg spring: `f_v = K_l(L_0 − L) − D_l Ḷ`, `τ = J_legᵀ f_v`.
- `K_l≈800 N/m` touchdown (stores ~1 J at 5 cm compression — ample for 3 cm hop),
  `D_l≈40–60 Ns/m` for `ζ≈0.7` with `m_eff≈0.75 kg`
  (`D ≥ 2ζ√(K_l·m_eff)≈34 Ns/m`).
- Joint-unit conversion with moment arm ~0.05 m: `K_joint≈K_l·0.05²≈2 N·m/rad`,
  `D_joint≈D_l·0.05²≈0.1–0.15 N·m·s/rad` — consistent with the assessment's
  `k_p≈1–5 N·m/rad, k_d≈0.05–0.25`.
- **Ramp stiffening to nominal `K_l≈4000 N/m` over 25–40 ms** for running (NOT the 100–200 ms
  walking ramp: 100 ms would consume ~70% of a 120–150 ms stance). First-order ramp to preserve
  passivity. Higher inner FOC bandwidth *shrinks* the stable impedance region (Focchi 2016) —
  at 200 Hz keep joint `K_p ≲150–300 N·m/rad`, measure I3C latency; if >1 ms, reduce gains.
- Reuse: `solve_leg_ik` gives the leg geometry; `lib.rs` torque-mode path (`JointCommandMode::Torque`,
  `torque_nm[i]` signed) already exists; need a `J_legᵀ` routine (build new, or reuse any FK in
  `rk-model` host-side only — MCU needs its own small planar/3D leg Jacobian). **v1 substitute:**
  software current limiting via `compliance_scale`/`ukemi_torque_scale` is already implemented;
  that is the only compliant touchdown available on v1.

### 2.4 CBF safety shield on tilt / CoM velocity
From WBC §5 (explicit projection, no QP):
- **CoM-velocity CBF:** `h = v_max² − ċ²`, `ċ̈≈ω²(c−p)` affine in ZMP `p` →
  `−2ċ·ω²(c−p) ≥ −κ(v_max²−ċ²)` → linear bound on `p_x`, intersect with support box, clip.
  `κ≈3 s⁻¹`. With `v_max≈0.6 m/s` for running this caps CoM speed before a fall.
- **Capture-point CBF:** `h = ξ_max² − ‖ξ−p_foot‖²`, `ξ̇=ω(ξ−p)` affine in `p` → linear
  constraint on `p` (HULK-style); add as extra row in the §1.3 MPC.
- **Torso-tilt ECBF** (relative degree 2): `h=θ_max²−θ²`, require
  `ḧ+k₁ḣ+k₀h≥0` with `θ̈=a_θ+b_θ u`; solve scalar bound on torque `u` by explicit
  `min/max` clamp (WBC §5.2a). Use `θ_max≈0.5 rad` (< `ukemi_tilt_rad=0.56`,
  < `fall_cutoff_rad=0.65`) so the shield acts **before** ukemi/tuck.
- **Discrete CBF** `h(x_{k+1})≥(1−γ)h(x_k)` with `A=[1,T;0,1]`, `B=[T²/2,T]ᵀ` can be an extra
  linear row in the MPC (WBC §5.2d).
- All are scalar projections — a few flops; fits the "filtered command CBF" pattern, not a QP.
- Reuse: `BalanceGains.fall_cutoff_rad/ukemi_tilt_rad`, estimator `dcm_xy`, `com_vel`;
  implement as a final clamp before the existing soft-limit/slew stage in `lib.rs` L600+.
  **[v1/v2]** — the tilt/CoM shield only needs existing signals, so it can be landed in P2 as a
  safety improvement even before running exists.

### 2.5 P3 reuse vs build — complexity
- **Reuse:** `UvcController` phase machine (extend with `Flight`/`Landing`), `solve_leg_ik`,
  ankle gyro damping, estimator CoM/ξ, `RawServoFeedback.current_ma/velocity`, torque-mode
  command path, `JointCommandMode::Torque`, ukemi/cutoff latch, replay harness.
- **Build new:** SLIP state machine + spring-law torque mapping (needs MCU leg Jacobian `J_leg`),
  flight detector, landing stiffness ramp, CBF projections (can be shared with P2), new
  telemetry fields, v2 BSP work (USB HS / WDT / LED / native I3C hooks / PSRAM) is a **parallel
  prerequisite**, not part of P3 algorithmics.
- **Estimated complexity:** algorithmics **5–8 days** after v2 BSP works; BSP bring-up **3–5
  days** separately; hardware tuning (impact tests, FOC latency measurement) **3–5 days**. **Do
  not bid P3 until v2 runs 21 joints at 200 Hz.**
- **Key risks:** touchdown debounce vs 120 ms stance; gearbox/friction saturating the spring law;
  CBF `κ` too aggressive → stiff gait; `a_m`/gravity lag (must use attitude-derived gravity per
  assessment §6.5, **not** an accel LPF — a 45 cm robot hits the floor in ~0.35 s); replay of
  a flight phase changes `contact_mask()` semantics — update all tests.

---

## 3. P4 — Combat

### 3.1 Offline impact-aware strike keyframe optimization **[PC]**
Host-side tool only (no runtime PC, no camera):
- Formulation from HUMANOID_COMBAT_RESEARCH §2.3 (Wang/Kheddar, arXiv:2001.08454 & 2308.06784):
  velocity jump `Δẋ = −(1+c_r)P_n ẋ⁻` with `P_n=nnᵀ`, articulated-body impulse map
  `Δẋ = Λ⁻¹ I`, `Λ⁻¹=J M⁻¹ Jᵀ`. Predict end-effector jump as affine in `q̈`, then add
  **worst-case linear inequalities** to a whole-body QP: impulsive joint torque
  `Δτ = (1/δt)(Σ J_iᵀ K_i⁻¹) Δẋ`, holding-contact CoP/friction bounds, ZMP bound. Solve a small
  KKT system analytically; no reset maps/hybrid switching.
- Less conservative alternative (Wang 2023): **CoM-velocity-area** zero-step capture region
  `ċ_xy = w z, w=sqrt(g/c_z)`; for this robot with `c_z≈0.25–0.48 m`, `w≈3.6–4.5` → with
  `z∈[−0.028,0.028]` (sagittal foot margin), `|ċ_x|≲0.10–0.13 m/s` post-impact. If a strike's
  predicted recoil exceeds this, the optimizer must add a balance-restore tail / step.
- **Practical tool:** implement a new host binary in `rk-model` (or `rk-sim`) using existing
  `KinematicRig`/URDF FK + `k::Chain`, plus a small dense QP (existing deps? check `Cargo.toml`;
  if none, a hand-coded KKT solve for the ≤12-var problem or add one justified dependency).
  Warm-start from the authored `default_combat_library()` poses (Guard/Jab/Cross/Kick).
- **Export format:** extend `ActionMove`/`Keyframe` (serde already present) with optional
  `impact_metadata`: predicted `Δċ`, `ΔL`, required `CoP` excursion, recoil threshold, and a
  `balance_tail` move reference. Write to the existing bundle/JSON path; validate every pose
  against URDF joint limits (`pose_from_overrides` already clamps) and replay through
  `build_next_host_packet` + `verify_deterministic_replay` before flash.
- Reuse: `KinematicRig`, `pose_from_overrides`, `sample_action_move`, `StudioModel`, replay
  verifier, URDF assets. Build new: optimizer, impact model, export schema, host CLI/MCP tool.
- Complexity: **6–10 days** for a credible offline optimizer + export + replay validation.

### 3.2 Momentum observer impact detection in the kernel **[v2]**
Equation from assessment §4.2c / IMPLEMENTATION_CONTROL_RESEARCH §2:
```
p = M(q) q̇ ;  β = Cᵀ q̇ − G + Aᵀλ̂ + Sᵀ τ_m      (λ̂=0, two-stage, never feed r back)
r_k = [ r_{k−1} + K_O( p_k−p_{k−1} − Δt·β_{k−1}) ] / (1 + K_O Δt)     // backward Euler
impact if ‖r_k‖ > r_th (0.1–0.3 N·m) or ‖r_k−r_{k−1}‖ > ṙ_th, 2–3 sample debounce,
50–100 ms refractory. K_O≈100 s⁻¹ (T_O=10 ms ≈2 ticks; ≥3Ts=15 ms preferred → K_O≈60–100).
```
- **Sign fix mandatory:** `+Aᵀλ`, `+r` inside the integral (critique round 2). Use backward
  Euler (unconditional stability), not `K_OΔt<1` forward Euler.
- `β` needs `Cᵀq̇−G` and `M(q)` (CRBA+RNEA, 50–120k flops/tick, ~1–2 ms at 150 MHz) —
  feasible only with non-blocking I3C. **v1 caveat: stiction/gearbox friction ≫ 0.1 N·m →
  infeasible**; keep the current validity mask (`MOMENTUM` bit never set) on v1.
- Integration: add a `momentum.rs` module in `rk-control`, called from `DirectServoKernel::step`
  after estimator, before UVC; publish `r` norm + impact flag in `EstimatorTelemetry` and set
  the `MOMENTUM` valid bit only on v2. Trigger: on impact, the action player's current strike
  switches to its `balance_tail`, or kernel sets a `balance_recover` flag that UVC honors by
  entering `ActiveCatchStep` (reuse existing P0/P1a recovery).
- Calibration: log `‖r‖` idle vs known shoves; set threshold at 3× idle RMS; K_O diag
  (100 hips/knees, 40–60 ankles). Complexity: **5–8 days** after v2 is live + calibration.

### 3.3 Hip/arm counter-rotation for striking
- Physics: `p = c − z_c/mg·c̈_xy − (1/mg)[−L̇_y, L̇_x]ᵀ`; to get CoP shift `Δp` from momentum
  alone, `L̇^⊥ = mg Δp`. For `Δp=0.01 m`, `T=0.1 s`: `τ_hip≈1.5*9.81*0.01/0.1≈1.47 N·m`
  — feasible. Drive `L̇→0` after impact (`L̇_des=−K_p L`) via torso pitch/roll + arm swing.
- **Already partly present:** `default_combat_library()` bakes waist yaw into Jab/Cross
  (`waist_yaw −0.35 → +0.56`) with mirrored head yaw, and `uvc.rs` `armCont` adds
  `spread/sag_swing` from lateral offset and tilt (L745–758). The kernel already adds waist
  yaw + shoulder deltas to host targets (L567–571).
- Build new: (a) include counter-rotation explicitly in the offline optimizer (§3.1) so
  kinematic momentum cancels rather than relying on hand-authored numbers; (b) on v2, a
  centroidal PD that commands `L̇_des` and maps to torso/arm torques (WBC §4.4) — or keep it
  open-loop on v1 per combat report §12.2. Complexity: **2–4 days** offline-only; **+4 days**
  for v2 centroidal PD.

### 3.4 Scripted combos with automatic balance restore via existing action studio
- Build a `combo` concept on top of `ActionMove`: sequence `[Guard, Left Jab, Right Cross,
  Front Kick, Guard]` using existing `play_move()` one after another (host-side, 200 Hz stream).
- Add a **balance-restore tail** to each strike move (a final keyframe back to Guard already
  exists for Jab/Cross/Kick — but its timing is fixed, not reaction-driven). New: in
  `ingest_telemetry()`, if `|tilt|>θ_abort` or `|dcm|>margin` or momentum impact flag →
  **truncate the current move** and request a restore action (`Guard` or a crouched recovery
  pose) instead of waiting for `done`. `PlaybackState` already tracks `active_keyframe`,
  `progress`, `elapsed_ms` — extend with `abort_requested`.
- Reuse: all of `studio.rs`, `default_combat_library`, `default_recovery_library`,
  `sample_action_move`, `build_next_host_packet`, `ingest_telemetry`; `UvcPhase` catch-step and
  fall-recovery library. Build new: combo sequencer, abort/restore logic, new telemetry checks.
- Complexity: **3–5 days**; can be landed on **v1** because it needs no torque sensing — only
  IMU tilt/DCM which already work. This is the highest-value, lowest-risk P4 item.
- Key risk: abort chatter (need hysteresis + refractory, same as impact detector); keyframe
  playback tempo scaling (`rate_scale = 0.25+0.75*speed%`) must not outrun the balance loop.

### 3.5 Optional tiny int8 MLP residual — training pipeline, obs/action **[PC + MCU]**
- **Observation (proprioceptive only, no camera):** `pitch, roll`, `pitch_rate, roll_rate`,
  normalized `vx/vy` cmd, `com_vel_xy` (or `dcm_xy − foot`), `foot_margin`, `contact_mask`,
  `gait_phase` (sin/cos), `support_foot` one-hot, per-leg joint angles/velocities/currents
  (or a compact subset): ~24–32 inputs. **Action:** residual joint position targets (21) or
  residual gait parameters (step length, cadence, foot offset, waist yaw) — the latter is far
  safer and is the recommendation (RoboStriker's central finding: never explore in raw joint
  torque space; HoloBoxing trains only a compact planner over a frozen controller).
- Training: Isaac Lab / `unitree_rl_gym` recipes adapted to a URDF of Rocky, domain randomization
  over mass (1–2 kg), friction, latency, gearbox/friction, IMU bias; PPO; reward = velocity
  tracking + survival + symmetry + torque/acceleration regularization. Distill to **2×32 or
  3×32 ReLU, int8 quantized**, ~1–3k MACs/step, run at **50 Hz** (every 4th 200 Hz tick) as a
  residual over the analytic UVC/Raibert controller, with the classical path as hard fallback.
- Deployment: new `rk-control::mlp` no_std module with static int8 weight arrays + saturating
  dot products (`i32` accum), generated by a host quantization script in `rk-model`/`tools`.
  Must be **deterministic** (fixed-point, no `f32` nondeterminism) to satisfy replay.
- **Honest recommendation:** given a 3-axis IMU and no opponent sensing, a blind combat policy
  adds little beyond what scripted combos + balance overlay already deliver. **Defer** unless an
  external opponent sensor (IR/beacon/host camera) is added; the report explicitly says not to
  pursue until sensing improves. Complexity if pursued: training **3–5 days GPU**, quantization +
  MCU inference **3–5 days**, safety integration **2 days**.

### 3.6 P4 reuse vs build — complexity summary
| Sub-item | Reuse | Build new | Est. days | HW gate |
|---|---|---|---|---|
| Offline impact-aware keyframes | `KinematicRig`, `studio.rs`, replay | QP/impact tool, export schema | 6–10 | PC |
| Momentum observer impact detect | `estimator` valid_mask, `rk-control` step | `momentum.rs`, CRBA/RNEA, thresholds | 5–8 | **v2** |
| Hip/arm counter-rotation | authored waist/arm deltas, `armCont` | centroidal PD (optional) | 2–4 (+4) | v1 offline / v2 PD |
| Combos + auto balance restore | all of `studio.rs`, recovery library | combo sequencer, abort logic | 3–5 | **v1 OK** |
| Tiny int8 MLP residual | replay, safety overlay | training + quantizer + MCU inference | 8–12 | v1/v2 but defer |
- **Key risks:** impact thresholds dominated by gearbox friction on v1; QP assumes known impact
  bounds; strict ZMP bounds over-conservative for strikes (use CoM-velocity area); learned
  policy without opponent sensing is blind; replay determinism must hold for any new state.

---

## 4. Recommended order within each stage and cross-stage dependencies

```
P2 (v1, now):
  1. Fix/confirm ω–h consistency in estimator (§1.1)         [prereq for everything]
  2. Raibert/capture foot placement into dxis/dyis ONLY + v_max mapping
  3. Phase clock + gait_phase continuity; replay/determinism tests
  4. Khadiv step-timing adaptation (needs 2+3)
  5. Scalar CBF (tilt/CoM-velocity) as filtered command clamp  [also benefits P3/P4]
  6. Small DCM-form f32 MPC (needs 2,4,5; optional stretch)

P3 (v2, after BSP):
  0. ESP32-P4 BSP: USB HS, WDT, LED, I3C hooks, heap → 21-joint 200 Hz torque loop
  1. CBF shield from P2.5 (already available) + flight detector (§2.2)
  2. SLIP spring law + landing impedance ramp (§2.3, needs torque mode)
  3. Raibert running placement (reuse P2 item 2 with T_s=0.12–0.15)
  4. Extend UvcPhase with Flight/Landing; replay updates
  5. Hardware impact tuning (K_l, D_l, debounce, FOC latency)

P4:
  1. Scripted combos + abort/balance-restore via studio (v1, no new sensing) — DO FIRST
  2. Offline impact-aware optimizer + export (PC, feeds better keyframes)
  3. Hip/arm counter-rotation baked into optimized keyframes (reuse P2 DCM/CBF)
  4. Momentum observer impact detection (v2 only, after P3 torque stack)
  5. Optional int8 MLP residual LAST — only if opponent sensing is added
```

**Dependencies:** P2 items 1–3 are a hard prerequisite for both P3 (Raibert placement, CoM
estimator, CBF) and P4 (balance overlay, combos). P3 requires the v2 BSP and torque-mode
impedance; P4's forceful items (3.2, v2 centroidal PD) require P3's torque stack, but P4's
scripted/offline items do **not** and can proceed in parallel with P2. The CBF shield (P2.5,
P3.4) is a shared safety component — implement once in `rk-control` and reuse. Replay/determinism
verification is a gate at every step, not a final task.

**What must be built from scratch overall:** phase clock + placement law + timing adapter +
MPC solver (P2); flight detector + SLIP spring law + leg Jacobian + landing ramp (P3); impact
QP tool + momentum observer + combo sequencer + optional MLP (P4). **What is reused:** the entire
`UvcController` phase/I-K/safety shell, `KinematicEstimator`, `DirectServoKernel::step` wiring,
`JointCommandMode::Torque` path, `ActionMove`/`Keyframe`/`StudioModel`, fall-recovery library,
`BalanceGains`/`WalkCmd` wire, and `rk-fw-core::replay` + `rk-sim` verification.

**Honest bottom line:** P2 is a genuine firmware change of ~1.5–3 weeks on v1 and is the only
stage that can be validated today. P3 is ~2–4 weeks **plus** an unestimated v2 hardware
bring-up that is currently a scaffold — treat the BSP as its own OpenSpec change. P4 splits
into a cheap high-value v1 slice (combos + balance restore, ~1 week) and expensive v2/PC slices
that should be separate proposals. Do not bundle P3/P4 into a P2 proposal.
