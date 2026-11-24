# Implementation-Level Control Research: Torque-Controlled 45 cm Humanoid (RP2350, 200 Hz, f32)

**Scope.** Sensorless (current-based) ground-reaction-force (GRF) estimation, momentum observers,
impedance/virtual-model control, centroidal momentum matrix (CMM) at 200 Hz in f32, and
IMU + leg-kinematics + inferred-contact state estimation. Target: 45 cm tall, 1–2 kg humanoid,
21 DoF, SimpleFOC torque/velocity/position control over I3C, RP2350 @ ~200 MHz (f32; no FPU
double precision advantage), 3-axis IMU only (no magnetometer assumed), flat feet with toes,
no foot force sensors.

> **Priority order for implementation.** (1) momentum-observer residual → contact detection;
> (2) current→torque→jacobian→GRF estimate with constrained-friction-cone projection;
> (3) flat-foot EKF for base pose; (4) impedance / virtual-model control for landing compliance;
> (5) CMM + centroidal PD for balance, only if (1)–(4) are reliable. Items 4–5 are the most
> compute-heavy; at 200 Hz f32 they are feasible but must use the compact formulations given below.

---

## 0. Notation and dynamics conventions

- Generalized coordinates: `q = [q_b; q_j] ∈ R^(6+n)`, `q_b` = floating base pose/velocity
  (position `r ∈ R^3` + unit quaternion `Q ∈ S^3`), `q_j ∈ R^n` joint angles, `n = 21` actuated.
- Spatial / body velocity `v = [ω; v_lin]`; momentum `h = (k; l)` = angular momentum about CoM,
  linear momentum. Ordering conventions differ between papers; keep one convention everywhere.
- Standard floating-base EoM (no foot sensors):

```
M(q) q̈ + C(q, q̇) q̇ + G(q) + Aᵀ(q) λ = Sᵀ (τ_m + τ_ext)          (1)
```

  where `λ` = contact wrences/constraint forces, `A` = contact constraint Jacobian,
  `S = [0_{n×6}  I_{n×n}]` selects actuated joints, `τ_m` = motor torques,
  `τ_ext` = unmodeled/external torques. Identity `Ṁ = C + Cᵀ` holds with the standard
  Christoffel construction.

- Jacobian of a contact point: `J_i(q) = ∂p_i/∂q ∈ R^{3×(6+n)}`; contact force `f_i ∈ R^3`
  enters as `J_iᵀ f_i`. For a flat foot add a contact torque (3D) and use a 6D wrench Jacobian.
- Motor-torque model from FOC current (critical for a current-sensed robot):

```
τ_m = N η_g k_t I_q − τ_fric(q̇_m) − τ_ripple(θ_m) − J_m N² q̈_j   (2)
```

  `N` = gearbox ratio, `η_g` = gearbox efficiency (forward-drive), `k_t` = motor torque
  constant [Nm/A], `I_q` = measured q-axis current, `τ_fric` = Coulomb + viscous friction
  (sign depends on direction), `τ_ripple` = cogging/gearbox ripple (period 2× wave-generator
  frequency for harmonic drives; for SimpleFOC BLDC it is 6× electrical / slot harmonics),
  `J_m N²` = reflected rotor inertia. **Note** η_g is direction-dependent: when torque is
  dissipated through a gearbox (back-driving) use `τ_out = N η_g τ_in` for motoring, but
  `τ_out = N τ_in / η_g` for generation (load driving motor). Using a single η is a dominant
  systematic error source in current-based force estimation (see §1.5).

**Sensor reality check.** For 21 DoF at 1–2 kg, link inertias are tiny (order 10⁻⁴–10⁻³ kg·m²)
and gearbox friction / rotor inertia reflected through `N²` dominate link dynamics. The momentum
observer in §2 is far more robust than direct inverse dynamics on such a platform, because it
integrates torques and does not differentiate encoder position twice. Use the observer, not
`M q̈`, as the primary estimator.

---

## 1. Current-based GRF / contact-force estimation without foot sensors

### 1.1 Algebraic mapping: torques → contact forces via the transposed Jacobian

For a leg in single support with contact wrench `λ ∈ R^6` at the foot, the actuated (joint)
rows of (1) give, neglecting inertial/Coriolis/gravity terms or after compensating them:

```
τ_j = S ( M q̈ + C q̇ + G ) − J_{c,j}ᵀ λ                            (3)
```

with `S = [0 I]`, `J_{c,j} = J_c Sᵀ ∈ R^{6×n}` the joint part of the contact Jacobian. Define
the **generalized momentum residual** `r ∈ R^n` (§2) as the estimate of the total external
joint torque `τ_ext = J_{c,j}ᵀ λ` plus disturbances. Then a point estimate of the contact
wrench is the (weighted) right pseudoinverse:

```
λ̂ = ( W J_{c,j}ᵀ )⁺ W r  =  ( J_{c,j} W J_{c,j}ᵀ )⁻¹ J_{c,j} W r      (4)
```

where `W ⪰ 0` weights joints by current-signal reliability (e.g. `W_ii = 1/σ_τ,i²`, i.e. inverse
variance of the torque estimate). For a fully constrained flat foot `rank(J_{c,j}ᵀ) = 6` requires
at least 6 independent joint torque effects; for a 3-DoF leg only the 3D force is estimable
(moment components are unobservable). A 6-DoF leg (hip 3 + knee + ankle 2) is needed for the
full wrench. For a **point foot / 3D force only**, use `J_{c,j} ∈ R^{3×n}` and `λ̂ ∈ R^3`.

**Null-space ambiguity.** When `rank(Jᵀ) < dim(λ)` the estimate is unique only modulo the
null space `N(Jᵀ)`; regularization/pseudoinverse picks the minimum-norm solution, which
implicitly assumes zero unmodeled joint torques in `N(Jᵀ)`. In practice impose structure:
for a planar 2-DoF leg, `f = (Jᵀ)⁺ τ` is unique; for a 3D leg with a spherical hip, an ankle
torque component lies in the null space of the force-only Jacobian — fix it by assuming
`τ_ankle_abduction ≈ 0` in stance or by including the 6D wrench and using a damped inverse
`λ̂ = Jᵀ (J Jᵀ + ε² I)⁻¹ r`, with `ε` chosen from expected torque noise.

**Equivalent direct formulation (static / quasi-static).** For slow motion with the body
approximated as inertial, summing joint torques weighted by leg-segment moment arms yields
`f ≈ (J_{foot,j}ᵀ)⁺ τ_j` directly [Bledt et al. 2018 — leg-momentum / current-based contact
fusion]. This assumes legs ≪ body mass (poor for a humanoid: legs are ~50 % of mass — see
Payne et al. 2026). Consequently:

> **Design decision for our 1–2 kg humanoid.** Do NOT use the body-fixed assumption. Use the
> full floating-base momentum observer (§2) which works in the world frame, and derive contact
> forces from the constrained-dynamics observer residual (§2.4).

### 1.2 Momentum-based force estimation (De Luca / Wensing line)

The core derivation (De Luca & Mattone 2003/2005; De Luca et al. 2006; Wensing–Orin line):

```
p = M(q) q̇                                                        (5)
ṗ = Ṁ q̇ + M q̈ = (C + Cᵀ) q̇ − (C q̇ + G + Aᵀλ) + Sᵀ(τ_m + τ_ext)
   = Cᵀ q̇ − G − Aᵀλ + Sᵀ(τ_m + τ_ext)                            (6)
   ≡ β + Sᵀ τ_ext,    β ≔ Cᵀ q̇ − G − Aᵀλ + Sᵀ τ_m                 (7)
```

`β` is the **nominal momentum input** and contains no joint accelerations. The observer:

```
p̂̇ = β + Sᵀ r,       r = K_O (p − p̂)                              (8)
ṙ = K_O (τ_ext − r)  ⇒  r_i(s)/τ_ext,i(s) = 1/(1 + T_i s), T_i = 1/K_O,i   (9)
```

i.e. each joint residual is a **first-order low-pass filtered** version of the true external
joint torque, with time constant `T_i = 1/K_O,i`. Larger `K_O` = faster convergence but more
noise amplification. Integrator form (no derivative of q needed):

```
r(t) = K_O [ p(t) − ∫₀ᵗ ( Sᵀ τ_m − β̂(q,q̇) + r ) ds − p(0) ],  r(0)=0   (10)
```

with `β̂ = Ĝ(q) + Ĉ(q,q̇) q̇ − Ṁ̂(q) q̇ + τ̂_fric` (note different sign convention for the
`Ṁ` term depending on how `β` is grouped; be internally consistent). The observer is
decoupled per joint (`K_O` diagonal) and needs no `M⁻¹`, no `q̈`, no force sensor.

For a **floating-base** robot the external wrench at the CoM can be estimated by applying the
same idea to centroidal momentum (Rotella et al. 2015):

```
ċ = l/m                                                           (11a)
l̇ = Σᵢ Fᵢ + m g + ΔF                                              (11b)
k̇ = Σᵢ (pᵢ − c) × Fᵢ + Σᵢ τᵢ + Δτ                                (11c)
ΔḞ = w_ΔF,   Δτ̇ = w_Δτ       (random-walk external wrench)
```

with state `x = [c, l, k, ΔF, Δτ]` and EKF measurement `y = [c_kin, k_kin]` from leg kinematics
+ inertia model. This avoids per-link inertia errors (only total mass `m` appears in prediction)
and yields delay-free push/GRF-offset estimates, at the cost of an EKF (§5.4).

### 1.3 Observability conditions

- **Wrench reconstruction from residual:** `rank(J_cᵀ)` must equal the dimension of the wrench
  being estimated; components in `N(J_cᵀ)` are physically unobservable and cannot be recovered
  from joint-space residuals (MOB-Net, 2024, §8.4; Iskandar et al. 2021). For a free swing leg
  with virtual joints, a 3D force plus contact location (6 unknowns) requires ≥ 6 independent
  Jacobian rows; with multiple simultaneous contacts the stacked Jacobian must maintain rank.
- **Flat-foot contact** gives 6 constraints (3 force + 3 moment) in 3D and removes the need to
  know base position/orientation for the observer (constrained dynamics, Payne et al. 2026).
- **Complementary filtering with gravity:** A single flat foot contact fully constrains base
  pose; only absolute position and yaw remain unobservable for the EKF (Rotella et al. 2014;
  §5.3), which is acceptable for balance.
- **Practical observability at 200 Hz:** with only a 3-axis IMU (no magnetometer) yaw remains
  unobservable; do not attempt to control absolute yaw from estimator output.

### 1.4 Contact detection thresholds and probabilistic fusion

Simple approach (Bledt et al. 2018; Camurri et al. 2017): threshold the estimated GRF and/or
the observer residual norm:

```
contact_i = ( ‖λ̂_i‖ > F_th )  or  ( ‖r_i‖ > r_th );   F_th ≈ 0.2–0.4 m g   (12)
```

For `m = 1.5 kg`, `mg ≈ 14.7 N`, so `F_th ≈ 3–6 N` per foot. Typical impact residuals at
touchdown are orders above steady stance; the residual spike is a **low-latency touchdown
detector** (Payne et al. 2026 §III-A). Liftoff detection via residual alone is weak — use the
relative-velocity constraint `v_rel = (A_l − A_r) q̇` (zero iff both feet flat on ground) or a
foot-height estimate.

Better: probabilistic Markov fusion (Payne et al. 2026; Gökbakan et al. 2025). Define contact
probability vector `P = [p_L, p_R, p_Dual]ᵀ` and per-step transition likelihoods via sigmoids:

```
π_{i,Dual} = σ( (‖τ̂_i,ext‖₂ − τ_t)/τ_b )                           (13)
π_{Dual,i} = σ( (v_rel − v_t)/v_b ) · ‖τ̂_{j,ext}‖/(‖τ̂_i,ext‖+‖τ̂_{j,ext}‖)  (14)
Π = [[1−π_{L,Dual}, 0,            π_{Dual,L}],
     [0,            1−π_{R,Dual}, π_{Dual,R}],
     [π_{L,Dual},   π_{R,Dual},   1−π_{Dual,L}−π_{Dual,R}]]         (15)
P_{k+1} = Π P_k                                                    (16)
```

Switch mode only when a mode's belief exceeds a threshold (e.g. > 0.5). This reduced false
transitions by ~2× vs raw argmax in hardware (Payne et al., 77 % accuracy on a 150 kg exo;
98.4 % in simulation). Gökbakan et al. (2025) use a Bayes filter with torque-likelihood KDEs
and IMU-derived switch probabilities (sigmoid on STFT power of vertical acceleration, offset
8 slope 1; directional probability from median frequency, offset 7.5 slope 2.5), achieving
~91 % takeoff / 71 % landing transition precision with < 100 ms latency on a real robot.
For our platform, the simple sigmoid-threshold + hysteresis version (12)–(16) is recommended
first; the learned-KDE version only if false detections persist.

### 1.5 Error sources (must be calibrated)

1. **Gearbox efficiency / directionality** — up to 20–40 % torque bias; calibrate `η_g⁺`/`η_g⁻`
   by measuring stall current vs. load torque. Dominant error in current-based estimation
   (Wahrburg et al. 2018; De Luca/Mattone current-based collision detection).
2. **Friction / cogging ripple** — Coulomb friction at gearbox output (often 0.1–1 Nm even for
   small gearboxes) and current ripple. Compensate with a friction observer or static friction
   map identified from slow sweeps; Focchi et al. (2016) show ripple creates vibration and
   erroneous residual. On a 1–2 kg robot, a 0.3 Nm unmodeled friction at a hip with lever arm
   0.1 m = 3 N force error — same order as `F_th`; cannot be ignored.
3. **Rotor inertia reflected by N²** — at 200 Hz with fast leg swings, `J_m N² q̈_j` may
   exceed link inertia torque; include it in `β` or subtract from measured current.
4. **IMU acceleration noise & base-velocity error** — momentum observers are sensitive to
   velocity-estimation error (Payne et al. 2026: accuracy degrades with SNR). Use the EKF
   velocity (§5), not raw integrated IMU velocity.
5. **Model parameter errors** (COM offsets, link masses, inertias from URDF) — direct cause of
   `Δc, Δl, Δk` offsets in Rotella et al. (2015); use the Offset Estimator / EWE to absorb them.
6. **Sampling / delay** — at 200 Hz (Ts = 5 ms), the residual filter time constant should be
   `T_O = 1/K_O ≳ 3 Ts = 15 ms` for stability/noise trade-off.

**Practical takeaway:** combine (i) residual threshold for touchdown, (ii) relative foot velocity
for liftoff, (iii) `(Jᵀ)⁺ r` for force magnitude, (iv) friction/efficiency calibration LUT.
Do not trust absolute GRF beyond ~20 % without torque-sensor-grade calibration.

---

## 2. Momentum observer: exact discrete-time implementation at 200 Hz

### 2.1 Continuous-time observer (De Luca & Mattone 2005/2006; Haddadin et al. 2017)

For a fixed-base manipulator (per leg, if using the inertial-body approximation — NOT recommended
for this humanoid but given for completeness):

```
p = M(q) q̇,    β = Cᵀ(q,q̇) q̇ − g(q) + τ_m                          (17)
p̂(0) = p(0),   r(0) = 0
r(t) = K_O [ p(t) − ∫₀ᵗ ( β(q,q̇) + r(s) ) ds − p(0) ]              (18)
p̂̇ = β + r,    ṙ = K_O ( p − p̂ ) = K_O ( τ_ext − r )               (19)
```

For a **floating-base** robot in the world frame (Payne et al. 2026, Eqs. 1–5):

```
p = M q̇,   β = Cᵀ q̇ − G − Aᵀλ + Sᵀ τ_m                            (20)
p̂̇ = β + Sᵀ r,     r = K_O (p − p̂)                                 (21)
```

Here `Aᵀλ` ≈ 0 if no modeled contacts are included (i.e. treat GRF as external disturbance) —
this is the cleanest for contact detection; then `r` contains the entire foot contact wrench
mapped to joint space. If flat-foot constraints are modeled, use the reduced dynamics (§2.4).

### 2.2 Exact Euler discretization (Ts = 5 ms, zero-order hold on τ)

Let `k` index samples, `Δt = 0.005 s`. Integrate (19) with backward/forward Euler:

```
r_k      = r_{k−1} + Δt · K_O ( τ_ext,k−1 − r_{k−1} )   [conceptual]  (22a)
p_k      = M(q_k) q̇_k                                                 (22b)
p̂_k      = p̂_{k−1} + Δt · ( β_{k−1} + r_{k−1} )                       (22c)
r_k      = K_O ( p_k − p̂_k )                                           (22d)
β_k      = Cᵀ(q_k,q̇_k) q̇_k − G(q_k) − Aᵀλ_k + Sᵀ τ_m,k              (22e)
```

This is the explicit "predict then correct" form used in Bledt et al. (2018) and MOB-Net (2024,
`K_O = 100` used there). Equivalently, the integral form (18) discretized exactly over one step:

```
p̂_k = p̂_{k−1} + Δt (β_{k−1} + r_{k−1}),   r_k = K_O(p_k − p̂_k)       (23)
```

`β` must use the **measured/applied** motor torque (current reading × kt × N × η), not just
the commanded torque, whenever available; otherwise use the command as a fallback (Lin et al.
2022 note this approximation increases FPR).

**Stability / display convention.** The discrete first-order filter has pole `1 − K_O Δt`.
Stability of the explicit form requires `0 < K_O Δt < 2`; for monotonic (non-oscillatory)
response require `K_O Δt < 1` → `K_O < 200 s⁻¹` at 200 Hz. Choose `K_O = 60–150 s⁻¹`
(corresponding bandwidth `f_c = K_O/(2π) ≈ 10–24 Hz`, time constant `7–17 ms`) for a 1–2 kg
robot. This comfortably covers expected touchdown rise times (< 50 ms) while suppressing
I3C-current quantization noise.

### 2.3 Tuning `K_O`

- Bandwidth view (9): `r_i/τ_ext,i = 1/(1 + s/K_O,i)` → `K_O,i` is the -3 dB corner in rad/s.
- Noise view: variance of `r` due to torque noise `σ_τ` grows ∝ `K_O` (the residual is a
  proportional gain on integrated noise); choose the largest `K_O` that keeps idle residual
  noise `‖r‖ < 0.3 r_th`.
- Rule of thumb: set `K_O` so the difference between `p` and `p̂` settles in ~4–5 samples
  (20–25 ms) after a known torque step → `K_O ≈ 4/0.02 = 200`, then back off for noise to
  ~100 s⁻¹.
- For 21 joints, use a diagonal `K_O`; joints with noisy current sensors get smaller gains.
  Example: `diag(K_O) = 100·I` for hips/knees, `40–60` for ankle abduction (low torque, noisy).

### 2.4 Constrained (per-contact-mode) observer — robust for humanoids

Payne et al. (2026) avoid assuming the body is inertial by running one observer per hypothesised
contact mode. For each mode `i ∈ {L, R}`, build reduced coordinates `y = Yq` with `A_i q̇ = 0`
(foot fixed to ground), and `H_i = [A_i; Y]⁻¹ [0; I]` so `q̇ = H_i ẏ`. Then:

```
M̃_i = H_iᵀ M H_i,   C̃_i = H_iᵀ C H_i + H_iᵀ M Ḣ_i,   G̃_i = H_iᵀ G   (24)
p_i = M̃_i ẏ_i,   β_i = C̃_iᵀ ẏ_i − G̃_i + τ̃_i,mot                    (25)
τ̂_i,ext = K_O (p_i − p̂_i),    p̂̇_i = β_i + τ̂_i,ext                  (26)
```

Correct contact mode ⇒ `τ̂_i,ext ≈ 0`; a spike in the active-mode residual ⇒ touchdown;
nonzero residuals in both ⇒ dual support; lifetime via `v_rel` (Eq. 14–16). This requires
knowing `A_i` (flat-foot constraints from kinematics) but not base velocity. Our robot with
flat feet + toes is a good fit.

### 2.5 Use as impact detector

Detect impact when any of:

```
‖r_k‖₂ > r_coll ,     or    ‖Δr_k‖ = ‖r_k − r_{k−1}‖ > ṙ_coll      (27)
```

Typical `r_coll ≈ 0.15–0.5 Nm` joint-level for a 1–2 kg robot (impacts produce 1–10 Nm
spikes). Keep a 2–3 sample debounce and a 50–100 ms refractory period after a detected impact
to avoid re-triggering on the residual ringing. For dual-foot detection use the Markov model
(13)–(16). Log `‖r‖²` for offline threshold calibration.

**Literature:** De Luca & Mattone (2003, 2005) [ICRA]; De Luca, Albu-Schäffer, Haddadin &
Hirzinger (2006) [IROS]; Haddadin, De Luca & Albu-Schäffer (2017) [IEEE T-RO 33(6):1292–1312];
Wahrburg et al. (2018) [IEEE T-ASE 15(2):879–886, DOI 10.1109/TASE.2017.2691136];
Payne, Hagen, Garagić & Johnson (2026) [arXiv:2412.03462]; Bledt, Wensing, Ingersoll & Kim
(2018) [ICRA, contact model fusion]; Haddadin et al. (2017) collision survey.

---

## 3. Impedance / admittance control with torque actuators

### 3.1 Exact formulations

**Joint impedance / PD with feedforward** (Focchi et al. 2016; Semini et al. 2015):

```
τ_ref = K_p (q_j,d − q_j) − K_d q̇_j + τ_ff                            (28)
τ_ff  = M_j(q) q̈_j,d + C_j(q,q̇) q̇ + G_j(q)   (inverse-dynamics compensation)  (29)
```

For a robot with inner torque loop of finite bandwidth, the desired mass–spring–damper
behavior is `M_r (ẍ_d − ẍ) + D_r (ẋ_d − ẋ) + K_r (x_d − x) = F_d − F_e`. With computed
inertia shaping (Hogan 1985; Khatib 1987):

```
τ = Jᵀ[ K_p(x_d − x) + K_d(ẋ_d − ẋ) + f_ff ] + ( M J⁻¹ (ẍ_d − J̇ q̇ − J M⁻¹ h) + h − Jᵀ f_e )  (30)
```

Simplified practical **Cartesian impedance** (used on torque-controlled legged robots —
Boaventura et al. 2012/2015, Xin et al. 2020) drops full inertia shaping:

```
τ_j = Jᵀ_{foot}(q) [ K_p (p_d − p) + K_d (v_d − v) + f_ff ] + h(q,q̇) + G(q)   (31)
```

`h = C q̇` (Coriolis), `f_ff` = desired contact force (from virtual model, §3.3) or gravity
compensation. Note the **difference**: joint impedance shapes each joint independently;
Cartesian impedance couples joints through `Jᵀ`, making the foot behave like a spring in task
space. Cartesian is essential for landing compliance because impact energy is absorbed by the
whole leg geometry.

**Admittance control** (for position-/velocity-controlled actuators): invert the same law to
get `x_d = (M_r s² + D_r s + K_r)⁻¹ F_e + x_ref`. With **torque** actuators (our case) use
impedance (31) directly — no admittance outer loop needed.

### 3.2 Passivity, sampling, delay — practical stability rules (Focchi et al. 2016)

Key findings from Focchi, Medrano-Cerda, Boaventura, Frigerio, Semini, Buchli & Caldwell
(2016, *Control Theory and Technology* 12(1):68–81; arXiv:1406.4047):

- Nested-loop architecture: inner PI torque loop (1 kHz in their HyQ) + positive velocity
  compensation + outer impedance PD. **Increasing inner torque-loop bandwidth can shrink
  the stable impedance region**: more loop gain in the inner loop leaves less for the outer
  loop. Do not blindly maximize FOC current-loop gain.
- Sampling at `T_s = 5 ms` (our 200 Hz) is ~5× slower than their 1 ms; the stable
  stiffness region shrinks substantially. Their passivity table shows e.g. `P = 20000 Nm/rad,
  D = 50 Nms/rad` is passive at `T_s = 0.5 ms` but **not** at 2–4 ms. **Conservative rule for
  200 Hz:** keep joint stiffness `K_p ≲ 150–300 Nm/rad` and damping `K_d ≳ 5–20 Nms/rad`,
  and verify by sweeping gains on hardware with the leg in contact.
- Passivity condition (continuous, ideal torque source): `D_r ≻ 0` and `K_r ≻ 0`; with a
  first-order torque loop of bandwidth `ω_t` and one-sample delay, a sufficient condition is
  roughly `ω_t ≫ ω_n = sqrt(K_p/M_eff)` and `D ≥ 2 ζ sqrt(K_p M_eff)` with `ζ ≈ 0.7–1`.
  At 200 Hz, keep `ω_n ≲ 2π·15 ≈ 94 rad/s` → with `M_eff ≈ 0.05 kg·m²`, `K_p ≲ 0.05·94²
  ≈ 440 Nm/rad`; use half of that margin.
- Velocity feedback filter (they average 4 samples) adds delay; avoid heavy filtering at
  200 Hz — use one-pole with `τ_f ≤ Ts/2` or a Savitzky–Golay over 3 samples.
- Use PI torque loop gains `P_t = 0.382 β, I_t = 18 β, β ∈ (0,25)` (their numbers, need
  re-scaling for our motors — treat as structure, not absolute values).

For our SimpleFOC current loop: run the torque (Iq) loop inside SimpleFOC at ≥ 2 kHz on the
motor side if possible, or rely on the FOC driver's current loop; our 200 Hz loop only sends
torque *setpoints* through I3C. I3C communication latency is an extra pure delay — measure it;
if it exceeds ~1 ms, reduce `K_p` further or predict torque one step ahead.

### 3.3 Landing compliance and Virtual Model Control (Pratt et al. 2001)

Virtual Model Control (VMC, Pratt, Chew, Torres, Gill, Ho & Solomon, *IJHR* 5(1):103–129, 2001;
adaptive extension Hu, Pratt, Chew, Herr & Pratt 1998) attaches **imaginary mechanical elements**
(springs, dampers) between points on the robot and the world, computes the virtual force, and
maps to joint torques via a Jacobian:

```
f_v = K_v (x_v,d − x_v) − D_v ẋ_v        (virtual spring–damper force)          (32)
τ   = J_v(q)ᵀ f_v                                                            (33)
```

Example virtual elements for a stance leg (Boaventura et al. 2015, HyQ):

1. **Virtual leg spring** from hip to foot: `l_v = ‖p_hip − p_foot‖`, `f_v = K_l (l_0 − l_v) − D_l ḷ_v`,
   acting along the leg axis `n = (p_hip − p_foot)/l_v`; torque `τ = J_vᵀ (f_v n)` where
   `J_v` is the Jacobian of `l_v`. This absorbs landing impact axially.
2. **Virtual rotational spring-damper at the hip pitch**: maintains body pitch orientation.
3. **Virtual vertical spring at the CoM**: `f_z = K_z(z_d − z_c) − D_z ż_c`, mapped through
   stance-leg Jacobians.

**Landing compliance recipe for our robot:**

- Detect touchdown via residual spike (§2.5) → switch stance leg to **soft virtual leg spring**:
  `K_l ≈ 500–1500 N/m`, `D_l ≈ 10–30 Ns/m` (for `m = 1.5 kg`, a 5 cm compression at
  `K_l = 800 N/m` stores `0.5·800·0.05² ≈ 1 J`, enough to absorb a drop from ~7 cm).
  In joint units for a leg with effective moment arm ~0.05 m, this is `K_joint ≈ K_l · 0.05² ≈
  2–4 Nm/rad` — stay well below the §3.2 stability limit.
- Ramp stiffness from soft to nominal (`K_l → 3000–6000 N/m`) over 100–200 ms as the impact
  decays (residual norm drops below threshold). Use a first-order ramp, not a step, to preserve
  passivity.
- Keep `D_l ≥ 2 ζ sqrt(K_l m_eff)` with `m_eff ≈ m/2 ≈ 0.75 kg` per leg → `D_l ≥ 2·0.7·
  sqrt(800·0.75) ≈ 34 Ns/m`; choose `D_l = 40–60 Ns/m` for critical damping at touchdown.
- Flat feet with toes: exploit the toe as a second virtual contact pivot — model heel and toe
  as two virtual springs during heel-strike / toe-off, switching which is active based on
  estimated CoP from the moment residual `λ̂_moment / λ̂_normal`.

**Gains summary for 1–2 kg humanoid (starting values):**

| Quantity | Swing | Touchdown | Steady stance |
|---|---|---|---|
| Joint K_p [Nm/rad] | 8–15 | 2–5 | 15–40 |
| Joint K_d [Nm·s/rad] | 0.5–1.5 | 3–6 | 2–5 |
| Cartesian K_z [N/m] | — | 600–1000 | 3000–6000 |
| Cartesian D_z [Ns/m] | — | 35–60 | 60–120 |
| Virtual leg K_l [N/m] | — | 800 | 4000 |
| Virtual leg D_l [Ns/m] | — | 50 | 100 |

Scale from mass/length: `K ≈ m g / δ` with allowed deflection `δ ≈ 0.03–0.05 m` →
`K ≈ 1.5·9.81/0.04 ≈ 370 N/m` per leg; double for margin → ~800 N/m. Angular momentum
"gains" scale as `M h` and `M h²` (Wensing & Orin 2016, p. 14 note) — no retuning needed when
scaling robot size.

---

## 4. Centroidal momentum matrix (CMM) at 200 Hz in f32

### 4.1 Definition

Centroidal momentum `h_G = (k_G; l_G) ∈ R^6` (angular about CoM, linear) satisfies:

```
h_G = A_G(q) q̇,     ḣ_G = A_G(q) q̈ +Ȧ_G(q) q̇ ≡ A_G q̈ + b_G        (34)
A_G ∈ R^{6×(6+n)}  (n = 21 → 6×27)                                   (35)
```

(Orin, Goswami & Lee 2013, *Autonomous Robots* 35(2–3):161–176; Orin & Goswami 2008 IROS.)

Newton–Euler centroidal dynamics (used for balance control):

```
m c̈ = Σᵢ fᵢ + m g                                                    (36a)
c × (m(c̈ − g)) + ḱ_G = Σᵢ (pᵢ − c) × fᵢ + Σᵢ τᵢ                     (36b)
```

### 4.2 O(n) algorithm — original recursive method (Orin et al. 2013)

Precompute from URDF: link masses `m_i`, COM offsets `c_i`, rotational inertias `I_i` (3×3,
about link COM, expressed in link frame), parent array `p(i)`, joint free-mode matrices `Φ_i`
(6×dof_i), and transforms `ⁱX_{p(i)}` (spatial transform from link i to parent).

```
1. I_C_i ← I_i for all i                              // composite rigid body init
2. for i = N down to 1:
       I_C_{p(i)} ← I_C_{p(i)} + ⁱXᵀ_{p(i)} I_C_i ⁱX_{p(i)}
3. CoM from I_C_0: extract p_G (translation part of composite inertia)
4. build ^{i}X_G for each link i (transform from G to link i)
5. for i = 1 to N:
       (A_G)_i = ^{i}Xᵀ_G I_C_i Φ_i        // 6×dof_i block
       h_G += (A_G)_i q̇_i
6. I_G = ⁰Xᵀ_G I_C_0 ⁰X_G;   v_G = I_G⁻¹ h_G   (average spatial velocity)
```

Complexity O(N) with a moderate constant; Orin et al. report >3 kHz for a humanoid on a PC.
This is the algorithm from the IROS 2014 workshop slides (Wensing & Orin 2014) / Orin et al.
2013 Table I.

### 4.3 Compact O(n) CMM from the mass matrix (Wensing & Orin 2016 — RECOMMENDED)

If you already compute `M(q)` and `C(q,q̇) q̇` (CRBA + RNEA), you get `A_G` and `b_G = Ȧ_G q̇`
almost for free — **no specialized recursive CMM code needed**:

```
Inputs:  H(q) [=(6+n)×(6+n) mass matrix], C(q,q̇) q̇, U₁ = [I₆  0_{6×n}],
         floating-base kinematics ⁰R₁(q_b), Ψ₁(q_b) (inverse of Φ₁)

H₁₁ = U₁ H U₁ᵀ                                   // 6×6 locked inertia (floating-base block)
I_C₁ = Ψ₁ᵀ H₁₁ Ψ₁                                // composite inertia in base frame
M    = (I_C₁)_{6,6}                               // total mass
¹p_G = [ (I_C₁)_{3,5}, (I_C₁)_{1,6}, (I_C₁)_{2,4} ]ᵀ / M     // CoM position in base frame
¹Xᵀ_G = [ ⁰R₁   0                    ]
        [ 0      ⁰R₁ S(¹p_G)ᵀ        ]      // S(·) = skew-symmetric cross-product matrix
A_G      = ¹Xᵀ_G Ψ₁ᵀ U₁ H                        // 6×(6+n) CMM
b_G = Ȧ_G q̇ = ¹Xᵀ_G Ψ₁ᵀ U₁ ( C q̇ )              // O(1) centroidal bias
```

This is Table 1 of Wensing & Orin (2016), Eqs. (20)–(21). With the common choice
`q_b = v_1` (floating-base spatial velocity as coordinate) → `Φ₁ = I₆`, `Ψ₁ = I₆`, and
`I_C₁ = H₁₁`, so `A_G = ¹Xᵀ_G U₁ H`, `b_G = ¹Xᵀ_G U₁ C q̇`.

**Timing** (Wensing & Orin 2016, Table 3, 30-DoF humanoid): CRBA `H`: 75.8 µs; RNEA `Cq̇`:
26.8 µs; original recursive CMM: 63.2 µs; Table-1 `A_G`: **10.5 µs**; Table-1 bias: **0.8 µs**.
Even with a ~10× slower RP2350 f32, that is ~110 µs — < 3 % of a 5 ms budget. This is the
strong argument for using the mass-matrix route.

**f32 practical notes:**
- Store `I_C₁` as f32; the skew extraction for `p_G` is numerically fine. Normalize the
  quaternion each tick before building `⁰R₁`.
- `S(a) = [[0,−a_z,a_y],[a_z,0,−a_x],[−a_y,a_x,0]]`.
- Only the **actuated columns** `A_G Sᵀ ∈ R^{6×21}` are needed for torque control; the base
  columns are used to map GRF to `ḣ_G` (Eq. 36).
- Precompute URDF constants at startup: `m_i, c_i, I_i, p(i), Φ_i, ⁱX_{p(i)}`; never parse URDF
  at runtime. Critically, validate total mass `M` against scale measurement — a 5 % mass error
  directly biases linear-momentum control by 5 %.

### 4.4 Centroidal momentum control (balance)

Commanded momentum rates (Wensing & Orin 2016, Eqs. 28–29; see also benchmark paper Sample et al.
2022, Eq. 5):

```
l̇_G,c = M [ c̈_d + K_D,l ( ċ_d − ċ ) + K_P,l ( c_d − c ) ]             (37)
k̇_G,c = k̇_G,d + K_D,k ( k_G,d − k_G )      (often k_G,d = 0 for walking)  (38)
```

with `K_D,l ≈ 2 ζ_l ω_l I`, `K_P,l = ω_l² I` (critical damping ζ ≈ 1), `ω_l ≈ 3–6 rad/s`
(for a 45 cm robot natural timescale ~0.2–0.4 s); `K_D,k` similar with `ω_k ≈ 5–10 rad/s`.
Note the units: `K_P,l` [1/s²], `l̇` [N]; `K_D,k` [1/s].

Desired CoM acceleration `c̈_d` can come from a divergent-component-of-motion (DCM) /
capture-point controller: `ξ = c + ċ/ω`, `ω = sqrt(g/z_c)`, with `z_c ≈ 0.25 m` → `ω ≈ 6.3
rad/s`; `ξ̇ = ω(ξ − ξ_d)` drives `c̈_d`.

**Mapping to joint torques — hierarchical QP or projection.** The underactuated part of (1)
is the first 6 rows; with contact forces `f_i` as decision variables:

```
min_{q̈, τ, f_i}  ‖ A_G Sᵀ τ − [l̇_c; k̇_c] ‖²_W + task regularizers
s.t.   M q̈ + C q̇ + G + Aᵀλ = Sᵀ τ,                                   (39)
       S q̈ = q̈_j,    foot accelerations = 0 (stance),
       λ_i in friction cone:  |f_x| ≤ μ f_z, f_z ≥ 0, |τ_z| ≤ μ' f_z.
```

With **no QP solver** on the MCU: use the operational-space projection:

```
q̈_j,d = (A_G Sᵀ)⁺ ( ḣ_c − Ȧ_G q̇ − (base rows of dynamics) )          (40)
τ_j   = S [ M q̈_d + C q̇ + G ] − J_cᵀ λ̂
```

or simply the **virtual model / centroidal PD** combination: compute desired net GRF from
(36): `Σf = m(c̈_d + g)`, and desired net moment about CoM from `k̇_c` minus gravity/contact
moment; distribute to stance feet with a simple heuristic (split by support polygon / CoP),
then `τ = J_stanceᵀ f_d + τ_swing_impedance`. This is the VMC approach of Pratt et al. (2001)
and Xin et al. (2020) and is the recommended first implementation for 200 Hz f32.

**Priority ordering** (Wensing & Orin 2016; Sample et al. 2022): feet contacts > CoM > CAM &
pose. Regulate CAM with a pose task (arm swing) when `k_d = 0` — otherwise over-constraining
CAM produces unnatural/stiff behavior (Wensing & Orin 2016 Figs. 8–9).

---

## 5. State estimation: EKF/complementary filter (Rotella et al. 2014) + practical simplification

### 5.1 Rotella et al. (2014) full formulation — arXiv:1402.5450

**State** (two feet; extend trivially):
`x = [ r, v, Q, p₁, p₂, b_f, b_ω, z₁, z₂ ]`
- `r`: IMU position (world); `v`: base linear velocity;
- `Q`: quaternion world→base; `C = C(Q)`;
- `p_i`: world position of foot `i`; `b_f, b_ω`: accelerometer/gyro biases;
- `z_i`: quaternion world→foot `i` (flat-foot orientation constraint — **key humanoid extension**).

**Process model (continuous):**

```
ṙ   = v                                                             (41a)
v̇   = a = Cᵀ ( f̃ − b_f − w_f ) + g                                 (41b)
Q̇   = ½ [ ω̃ − b_ω − w_ω ; 0 ] ⊗ Q                                  (41c)
ṗ_i = Cᵀ w_{p,i}       (foot fixed in world while in contact)       (41d)
ḃ_f = w_{bf},   ḃ_ω = w_{bω}                                        (41e)
ż_i = ½ [ w_{z,i} ; 0 ] ⊗ z_i   (allows small foot rotational slip)  (41f)
```

`f̃, ω̃` = raw accelerometer/gyro; `w_*` = white noise processes; foot pose random walks.

**Measurement model (from joint encoders + forward kinematics):**

```
s_p,i = C (p_i − r) + n_p,i                                         (42a)  // foot pos in base frame
s_z,i = exp(n_q,i) ⊗ Q ⊗ z_i⁻¹                                      (42b)  // foot orientation in base frame
```

Eq. (42b) is the flat-foot rotational constraint: it makes roll/pitch and **gyro bias**
observable (point-foot filter could not).

**Discrete prediction at 200 Hz (zero-order hold on IMU):**

```
r̂⁻_{k+1} = r̂⁺_k + Δt v̂⁺_k + (Δt²/2)( Ĉ⁺ᵀ_k f̂_k + g )             (43a)
v̂⁻_{k+1} = v̂⁺_k + Δt ( Ĉ⁺ᵀ_k f̂_k + g )                            (43b)
Q̂⁻_{k+1} = exp(Δt ω̂_k) ⊗ Q̂⁺_k                                     (43c)
p̂⁻_{i,k+1} = p̂⁺_{i,k},   b̂⁻ = b̂⁺,   ẑ⁻ = ẑ⁺                        (43d)
f̂_k = f̃ − b̂⁺_f,   ω̂_k = ω̃ − b̂⁺_ω
```

**Quaternion handling:** exponential map `exp(ω) = [ sin(‖ω‖/2)·ω/‖ω‖ ; cos(‖ω‖/2) ]`;
state error uses minimal rotation vector `φ ∈ so(3)`; innovation for a quaternion measurement
`e = log(s ⊗ z⁻¹) ∈ R³`; correction `Q⁺ = exp(Δφ) ⊗ Q⁻` (Eqs. 3–6 of paper).

**Linearized error-state Jacobians** `δx = [δr, δv, δφ, δp, δb_f, δb_ω, δθ]`:

```
F_c = [ 0   I     0          0   0    0   0
        0   0  −Cᵀ f×       0  −Cᵀ  0   0
        0   0  −ω×          0   0   −I   0
        0   0     0          0   0    0   0
        0   0     0          0   0    0   0
        0   0     0          0   0    0   0
        0   0     0          0   0    0   0 ]

L_c = diag{−Cᵀ, −I, Cᵀ, I, I, I}

H_c = [ −C   0   (C(p−r))×   C   0    0    0
          0   0      I      0   0    0  −C[Q⊗z⁻¹] ]
```

with discretizations `F_k ≈ I + F_c Δt`, `Q_k ≈ F_k L_c Q_c L_cᵀ F_kᵀ Δt`, `R_k ≈ R_c/Δt`.
Full equations (25)–(31) in the paper.

**Contact switching:** drop a foot's measurement equations when not in contact, and set its
foothold and foot-orientation process-noise variances `w_p, w_z` to very large values → its
pose uncertainty balloons; on re-contact the measurement re-initializes foot pose. No separate
filter models needed. During aerial phase the filter reduces to pure IMU integration.

**Observability (their Table III):** with flat-foot contacts, rank loss is only 2 (ω = 0),
1 (ω ⊥ Cg), or 0 (generic rotation) — vs. up to 5 for point feet. Absolute position and yaw
are always unobservable; **flat feet additionally render gyro bias observable**.

**Reported performance** (1 kHz, simulated SARCOS humanoid, realistic noise): flat-foot vs
point-foot RMS position error 0.0042 m vs 0.0088 m; yaw error 0.0055 rad vs 0.0379 rad
(Table V). Noise params: `w_f = 0.00078 m/s²/√Hz`, `w_ω = 0.000523 rad/s/√Hz`,
`n_p = 0.01 m`, `n_z = 0.01 rad`.

### 5.2 Practical simplification for our 3-axis-IMU + 1–2 kg robot

A full 27-state error EKF with quaternion foot states is probably too heavy for 200 Hz f32 on
RP2350 alongside control. **Recommended architecture:**

1. **Complementary filter for orientation** (Mahony/Madgwick style) at ~200–500 Hz using
   accelerometer (gravity) + gyro. Yaw not corrected (no magnetometer) — accept drift.
   Alternatively run the EKF's quaternion + bias sub-block (states `Q, b_ω, b_f`, 10 error
   states) only.
2. **Leg-odometry position/velocity correction** at contact: when foot `i` is inferred in
   contact (§1.4), form the position innovation `e_p = p_i,world(FK) − p̂_i,world` and apply a
   gain `K_p ≈ 0.1–0.3` per step to correct `r̂, v̂`; reset `p̂_i` when contact is lost.
   This is exactly Rotella's measurement (42a) with a hand-tuned gain instead of a full EKF.
   Flat-foot orientation innovation `e_R = log(Q_FK ⊗ Q̂⁻¹)` corrects roll/pitch with
   `K_R ≈ 0.05–0.15`.
3. **Foot contacts** tracked via Markov mode probabilities (§1.4) from the momentum observer;
   feed mode to the filter as process-noise switch (infinite variance = free foot).
4. **Use only `C(Q)` from the complementary filter** in the momentum observer's `β` (Coriolis
   term uses q̇, not base velocity, so base-velocity error enters only via M(q) terms — minor).

If compute allows, implement the **error-state EKF** with reduced state
`δx = [δr(3), δv(3), δφ(3), δp_L(3), δp_R(3), δb_f(3), δb_ω(3)]` (21 states), skipping foot
orientation quaternions (flat-foot orientation used as a direct roll/pitch update instead).
Matrix ops are 21×21 — feasible at 200 Hz in f32 if you exploit the block-sparsity of `F`
(most blocks zero; `H` is 6×21 per foot).

### 5.3 Momentum/GRF estimation filter (Rotella et al. 2015 — optional add-on)

If you need delay-free GRF for balance instead of the noisy algebraic estimate, implement the
**Momentum Estimator / External Wrench Estimator** from Rotella, Herzog, Schaal & Righetti
(2015, arXiv:1507.04401, ICHR 2015):

```
state x = [c, l, k, ΔF, Δτ]
process: ċ = l/m;  l̇ = ΣFᵢ + mg + ΔF;  k̇ = Σ(pᵢ−c)×Fᵢ + Στᵢ + Δτ;  ΔḞ,Δτ̇ = white
measurement y = [c_kin, k_kin] from leg FK + inertia model
```

`m` = total mass only (no link inertia errors in prediction); the EKF absorbs model errors as
`ΔF, Δτ`. Report: accurate constant, configuration-dependent, and impulsive wrenches during
walking; superior to LIPM-based filters. Complexity: 15-state EKF with linear dynamics (cheap).
Integrate GRF from the current-based estimator (§1) as measurements `Fᵢ` — this is the cleanest
way to fuse current-based forces with inertial dynamics on our platform.

**Noise parameters they used** (1 kHz, scale to 200 Hz by `σ_disc = σ_cont/√Δt`):
`q_θ = 3.16e-6 rad/√Hz`, `q_F = 0.063 N/√Hz`, `q_τ = 0.00316 Nm/√Hz`;
discrete `σ_θ = 1e-4 rad`, `σ_F = 2 N`, `σ_τ = 0.1 Nm`. For our lighter robot scale forces by
~1/10 (e.g. `σ_F ≈ 0.2–0.5 N`).

### 5.4 Integration architecture (one 5 ms tick)

```
1. Read IMU (3-axis accel+gyro), encoder angles (21), motor currents (21) over I3C.
2. q̇_j ← filtered encoder difference; q̇_b ← complementary filter gyro.
3. FK: foot positions/velocities, contact Jacobian J_i, constraints A_i.
4. Momentum observer (§2): compute β, integrate p̂, residual r; threshold→contact mode (§1.4).
5. GRF estimate λ̂_i = (J_iᵀ)⁺ W r (§1.1), project to friction cone; feed to §5.3 if used.
6. State estimator (§5.2): propagate IMU; update from stance-foot FK + flat-foot orientation;
   output r, v, Q (and c, l, k if §5.3 enabled).
7. Control: finite-state machine (swing/stance/landing) → virtual-model / impedance law (§3.3)
   → τ_ref; optional centroidal PD (§4.4) → stance-force distribution → τ = Jᵀ f + τ_imp.
8. Send torque setpoints via I3C; log residual norms and contact modes.
```

Budget estimate (RP2350 f32, ~10× slower than PC numbers): FK/Jacobians ~100 µs; momentum
observer (21-DoF, needs `Cᵀ q̇`) ~150–300 µs; mass matrix via CRBA ~800 µs–1.5 ms; CMM from
mass matrix ~100 µs; complementary filter ~50 µs; impedance/VMC ~100 µs. Comfortably fits
5 ms if CRBA is called once per tick and reused; a full 21-state EKF adds ~0.5–1 ms —
feasible but profile before committing.

---

## 6. Key references (titles, authors, years, URLs)

1. **De Luca, A. & Mattone, R.** "Actuator failure detection and isolation using generalized
   momenta." *Proc. IEEE ICRA*, 2003, pp. 634–639.
2. **De Luca, A. & Mattone, R.** "Sensorless robot collision detection and hybrid force/motion
   control." *Proc. IEEE ICRA*, 2005, pp. 999–1004. DOI: 10.1109/ROBOT.2005.1570247.
3. **De Luca, A., Albu-Schäffer, A., Haddadin, S. & Hirzinger, G.** "Collision detection and safe
   reaction with the DLR-III lightweight manipulator arm." *Proc. IEEE/RSJ IROS*, 2006,
   pp. 1623–1630. URL: `http://www.diag.uniroma1.it/~deluca/pHRI_elective/IROS06_ADL_etal_CollisionDetectionReaction_DLRIII.pdf`
4. **Haddadin, S., De Luca, A. & Albu-Schäffer, A.** "Robot collisions: a survey on detection,
   isolation, and identification." *IEEE T-RO* 33(6), 2017, 1292–1312.
5. **Wahrburg, A., Bös, J., Listmann, K.D., Dai, F., Matthias, B. & Ding, H.** "Motor-Current-Based
   Estimation of Cartesian Contact Forces and Torques for Robotic Manipulators and Its
   Application to Force Control." *IEEE T-ASE* 15(2), 2018, 879–886. DOI: 10.1109/TASE.2017.2691136.
6. **Payne, J.J., Hagen, D.A., Garagić, D. & Johnson, A.M.** "Multi-Momentum Observer Contact
   Estimation for Bipedal Robots." 2026. `https://arxiv.org/abs/2412.03462`
7. **Bledt, G., Wensing, P.M., Ingersoll, S. & Kim, S.** "Contact model fusion for event-based
   locomotion in unstructured terrains." *Proc. IEEE ICRA*, 2018, pp. 4399–4406.
8. **Camurri, M., Fallon, M., Bazeille, S., Radulescu, A., Barasuol, V., Caldwell, D.G. & Semini,
   C.** "Probabilistic contact estimation and impact detection for state estimation of
   quadruped robots." *IEEE RA-L* 2(2), 2017, 1023–1030.
9. **Gökbakan, U.B., Dümbgen, F. & Caron, S.** "A Data-driven Contact Estimation Method for
   Wheeled-Biped Robots." 2025. `https://arxiv.org/abs/2410.12345`
10. **Lin, T.-Y., Zhang, R., Yu, J. & Ghaffari, M.** "Legged Robot State Estimation using
    Invariant Kalman Filtering and Learned Contact Events." *CoRL 2022*, PMLR 164.
11. **Focchi, M., Medrano-Cerda, G.A., Boaventura, T., Frigerio, M., Semini, C., Buchli, J. &
    Caldwell, D.G.** "Robot Impedance Control and Passivity Analysis with Inner Torque and
    Velocity Feedback Loops." *Control Theory and Technology* 12(1), 2016, 68–81.
    `https://arxiv.org/abs/1406.4047`
12. **Pratt, J., Chew, C.-M., Torres, A., Gill, P., Ho, A. & Solomon, D.** "Virtual Model Control:
    An Intuitive Approach for Bipedal Locomotion." *Int. J. Humanoid Robotics* 5(1), 2001,
    103–129. (Adaptive version: Hu, J., Pratt, J., Chew, C.-M., Herr, H. & Pratt, G., 1998.)
13. **Semini, C., Barasuol, V., Boaventura, T., Frigerio, M., Focchi, M., Caldwell, D.G. & Buchli,
    J.** "Towards versatile legged robots through active impedance control." *IJRR* 34(7), 2015.
14. **Xin, G., Wolfslag, W.J., Lin, H.-C., Tiseo, C. & Mistry, M.** "An Optimization-Based
    Locomotion Controller for Quadruped Robots Leveraging Cartesian Impedance Control."
    *Frontiers in Robotics and AI* 7:48, 2020.
15. **Hogan, N.** "Impedance Control: An Approach to Manipulation." *ASME J. Dynamic Systems,
    Measurement and Control* 107, 1985. **Khatib, O.** "A unified approach for motion and force
    control of robot manipulators: the operational space formulation." *IEEE JRA* 3(1), 1987.
16. **Orin, D.E., Goswami, A. & Lee, S.-H.** "Centroidal dynamics of a humanoid robot."
    *Autonomous Robots* 35(2–3), 2013, 161–176. DOI: 10.1007/s10514-013-9341-4.
17. **Orin, D.E. & Goswami, A.** "Centroidal momentum matrix of a humanoid robot: Structure and
    properties." *Proc. IEEE/RSJ IROS*, 2008, 653–659.
18. **Wensing, P.M. & Orin, D.E.** "Improved Computation of the Humanoid Centroidal Dynamics and
    Application for Whole-Body Control." *Int. J. Humanoid Robotics* 13(1), 2016, 1550039.
    URL: `http://www.cs.cmu.edu/~cga/z/Wensing_IJHR_2016.pdf`
19. **Sample, A. et al.** "Benchmarking Whole-Body Controllers on the TALOS Humanoid Robot."
    *Frontiers in Robotics and AI*, 2022 (centroidal dynamics / DCM / AM task equations).
20. **Rotella, N., Bloesch, M., Righetti, L. & Schaal, S.** "State Estimation for a Humanoid
    Robot." *Proc. IEEE/RSJ IROS*, 2014, 952–958. `https://arxiv.org/abs/1402.5450`
21. **Bloesch, M., Hutter, M., Hoepflinger, M.A., Leutenegger, S., Gehring, C., Remy, C.D. &
    Siegwart, R.** "State estimation for legged robots — consistent fusion of leg kinematics and
    IMU." *Robotics: Science and Systems*, 2012 (base filter extended by Rotella).
22. **Rotella, N., Herzog, A., Schaal, S. & Righetti, L.** "Humanoid Momentum Estimation Using
    Sensed Contact Wrenches." *IEEE-RAS Humanoids*, 2015. `https://arxiv.org/abs/1507.04401`
23. **MOB-Net** (2024): "Limb-modularized Uncertainty Torque Learning of Humanoids for
    Sensorless External Torque Estimation." `https://arxiv.org/abs/2402.11221` (rank analysis
    §8.4, `K_O = 100` discretized MOB baseline).
24. **Iskandar, M., Eiberger, O., Albu-Schäffer, A., De Luca, A. & Dietrich, A.** "Collision
    Detection, Identification, and Localization on the DLR SARA Robot with Sensing Redundancy."
    *Proc. IEEE ICRA*, 2021 (momentum residual Eq. 3–5, wrench pseudoinverse).
25. **Focchi, M. et al.** (same as #11) — stability-region study explicitly covering sampling
    time, torque-loop bandwidth, velocity-filter length, and passivity limits; essential reading
    before choosing impedance gains at 200 Hz.

---

## 7. Recommended implementation sequence (concrete milestones)

1. **Calibrate current→torque** (efficiency, friction, ripple, rotor inertia) on a single joint;
   verify against a known load. Without this, all force estimates are garbage.
2. **Implement residual observer** (Eqs. 22–23) for each joint at 200 Hz; log `‖r‖`; set
   `K_O ≈ 100 s⁻¹`; calibrate touchdown threshold from drop tests.
3. **Contact state machine** with sigmoids + hysteresis (Eqs. 13–16); add `v_rel` liftoff test.
4. **Complementary filter + leg odometry** (§5.2); validate base roll/pitch within 0.05 rad
   during walking by motion capture or manual reference.
5. **GRF estimate** via `(Jᵀ)⁺ r` with friction-cone projection; use for CoP estimate on flat
   feet (`x_CoP ≈ −λ̂_m,y/λ̂_z` etc.).
6. **Virtual model / Cartesian impedance** for landing (§3.3); tune soft→stiff ramp on a single
   leg drop test before full walking.
7. **CMM via mass-matrix route** (§4.3) + centroidal PD (§4.4) with conservative gains and
   pose regularization; add only after 1–6 are stable.
8. **Optional:** full error-state EKF (§5.1/§5.2) and momentum/GRF filter (§5.3) if drift or
   force noise limits performance.

**Avoid these pitfalls:** (a) using `M q̈` from double-differentiated encoders at 200 Hz;
(b) assuming the body is an inertial frame for leg force estimation on a humanoid;
(c) setting impedance `K_p` high just because torque control is "stiff" — at 5 ms sampling the
stable region shrinks dramatically; (d) trusting absolute yaw from a 3-axis IMU;
(e) ignoring gearbox efficiency/friction — they are the largest error source in current-based
force estimation at this scale.
