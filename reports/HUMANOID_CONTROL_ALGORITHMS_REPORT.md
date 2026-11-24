# Humanoid Control Algorithms: Balance, Walking, Running & Combat
### An extensive technical survey (literature through 2026) with a feasibility assessment for Rocky Mk1 on an RP2350-class MCU

**Scope.** This report surveys the algorithmic landscape for humanoid balance, walking, running, and combat/fighting competitions, from foundational ZMP/LIPM work through 2026 learned whole-body controllers. For every family it gives the core idea, key equations, strengths/weaknesses, computational cost, and real-robot track record. It closes with a comparative synthesis and a concrete, tiered recommendation for a 21-DoF humanoid (Rocky Mk1) on an RP2350 (dual Cortex-M33 ~150 MHz, no hardware FP64, ~520 KB SRAM) running at 200 Hz with f32 math and a 2-axis IMU.

**Sources.** Web search was unavailable/rate-limited in this environment, so citations were discovered and verified via the arXiv API, OpenAlex API, individual arXiv abs pages, publisher portals, and Semantic Scholar (2026-11-24), supplemented by established knowledge-base references. Items marked **[KB]** are from the knowledge base and are not freshly verified. Some 2026 preprints are future-dated and should be treated as provisional. The per-topic reports contain verification appendices with specific corrections to commonly-miscited papers; key corrections are reflected below. Detailed per-topic reports with full reference lists live alongside this document:
- `HUMANOID_WALKING_RESEARCH_REPORT.md`
- `DYNAMIC_LOCOMOTION_RESEARCH_REPORT.md`
- `HUMANOID_COMBAT_RESEARCH.md`

---

## 0. The common mathematical backbone

Nearly all model-based humanoid control reduces the full multibody dynamics to a low-dimensional **template**:

- **ZMP (Zero-Moment Point)** — the ground point where the resultant GRF produces zero horizontal moment. If it lies inside the support polygon, contact is physically realizable. With angular momentum $\dot L$ about the CoM $c$ at height $z_c$:
  $$p = c - \frac{z_c}{mg}\ddot c + \frac{1}{mg}\begin{bmatrix}-\dot L_y \\ \dot L_x\end{bmatrix}$$
  Foundation: Vukobratović & Borovac, *IJHR* 2004 (orig. 1972).
- **LIPM (Linear Inverted Pendulum Model)** — point mass at fixed height $h$, massless telescoping leg. With $\omega=\sqrt{g/h}$:
  $$\ddot x = \omega^2(x-p_x), \qquad p = c - \ddot c/\omega^2$$
  Kajita et al., ICRA 2003 (preview control); Kajita, *Humanoid Robots* 2014.
- **Capture Point / DCM (Divergent Component of Motion)** — define $\xi = c + \dot c/\omega$ (the point where the CoM would stop if the ZMP were placed there). The system splits into a stable mode ($c$) and an unstable first-order mode ($\xi$):
  $$\dot\xi = \omega(\xi-p)$$
  Capture point: Pratt, Carff, Drakunov & Goswami, Humanoids 2006. DCM: Englsberger, Ott & Albu-Schäffer, *IEEE T-RO* 31(2), 2015 (DOI 10.1109/TRO.2015.2405592).

Balance control becomes: **regulate $\xi$ into the support region** (ankle strategy) or **move the support foot to $\xi$** (step strategy). This single idea unifies standing balance, walking, and push recovery — and its O(1) cost makes it the backbone of any MCU-feasible controller.

**Variants:** 3LP (Faraji & Ijspeert, arXiv:1605.03036), variable-height VHIP (Caron et al., "Capturability-based Pattern Generation for Walking with Variable Height," *T-RO* 2019, arXiv:1801.07022), angular-momentum LIP/ALIP (Dosunmu-Ogunbi, arXiv:2307.02448), hybrid LIP + actuated SLIP (Xiong & Ames, arXiv:2008.02435), LIPM-with-flywheel (Kasaei, arXiv:1906.02017), support-relative DCM (Zhou et al. 2026, arXiv:2608.00500 — **computable from encoders + 2-axis IMU without base linear velocity**).

---

## 1. Balance & standing control

### 1.1 Classical stack
- **Ankle / hip / step strategies** (Pratt lineage): ankle = CoP shift inside foot (small disturbances); hip = centroidal angular momentum via torso/arms ($\xi$ outside support but stepping impossible); step = relocate foot beyond $\xi$ (large pushes).
- **DCM tracking + step timing/location adjustment**: Englsberger's DCM reference $\dot\xi^{ref}=\omega(\xi^{ref}-\xi^{des})$ with desired ZMP/CMP from tracking error. Atlas hardware: Griffin, Wiedebach et al., IROS 2017 (arXiv:1703.00477). Step timing adaptation (Khadiv et al., arXiv:1704.01271) shows optimizing only the *next* step's timing and location is sufficient for viability — computationally cheap. Position-controlled servos: SURENA-Mini capture-point feedback (arXiv:1710.10598); iCub torque-controlled stepping (arXiv:1705.10579).

### 1.2 Centroidal dynamics & whole-body control (WBC)
- **Centroidal momentum matrix** $h=A(q)\dot q$, $\dot h = [mg+\sum f_i;\ \sum(r_i-c)\times f_i+\tau_i]$ — Orin, Goswami & Lee, *Autonomous Robots* 2013; efficient CMM — Wensing & Orin 2014.
- **Operational-space control** — Khatib 1987.
- **Hierarchical QP / inverse-dynamics WBC**: $\min_{\ddot q,f,\tau}\sum w_i\|J_i\ddot q+\dot J_i\dot q-\ddot x_i^{des}\|^2$ subject to dynamics, friction cones, torque limits. Atlas: Kuindersma et al., *Autonomous Robots* 2016. HRP-4 whole-body admittance: Caron et al., arXiv:1809.07073. Modern comparisons: Sovukluk et al., IROS 2025 (arXiv:2507.18502); kinodynamic fabrics — Adu-Bredu et al., arXiv:2303.04279 (faster/more reactive than QP-WBC).

### 1.3 MPC for balance
Convex QP over $\xi,p$ horizon $N$. Linear LIPM-MPC: Wieber 2006; Shafiee-Ashtiani (arXiv:1702.08742); Daneshmand variable-horizon (arXiv:2010.08198); contingency MPC (arXiv:2404.12455). Nonlinear centroidal MPC / trajectory optimization: TAMOLS (Jenelten, arXiv:2206.14049).

### 1.4 Learning-based balance
- Push recovery: Yang et al. hierarchical DRL on impedance (arXiv:2002.02991); iCub emergence of strategies (arXiv:2104.14534); Atalante reactive stepping (arXiv:2203.01148).
- **Learned whole-body controllers (2023–2026)**: causal transformer policies — Radosavovic et al., *Science Robotics* 2024 (arXiv:2303.03381) and rough-terrain follow-up (arXiv:2410.03654); Humanoid-Gym (arXiv:2404.05695); ASAP real-world residual (arXiv:2502.01143); FastSAC 15-min sim-to-real (arXiv:2512.01996); RPL (arXiv:2602.03002); VIRAL (arXiv:2511.15200); PolySim (arXiv:2510.01708); BFM-Zero (arXiv:2511.04131); HULK with capture-point CBF (arXiv:2610.08970). *BeyondMimic* (Luo et al., ~2025) — **[KB] unverified**, confirm before formal citation.
- **Domain randomization** (mass/inertia/friction/motor delay/IMU noise) + teacher–student distillation is standard practice.

### 1.5 Estimation & disturbance observers
- EKF: IMU propagate + leg-encoder kinematic correct under flat-foot contact — Rotella et al., IROS 2014 (arXiv:1402.5450). Torque-based centroidal EKF (arXiv:2202.12574). SG-ODOM stability-gated contact (arXiv:2611.20710).
- **Momentum observer**: generalized-momentum residual $r=K(p(t)-\int(S^\top\tau+J_c^\top\hat f+C^\top-\tau_f)dt-p(0))$ converges to external wrench — De Luca lineage; MOB-Net learned correction (arXiv:2402.11221).
- **Impedance/admittance control** (Hogan 1985): $M_d\ddot e+D_d\dot e+K_de=F_{ext}$. Ankle impedance sets "ankle-strategy" stiffness; whole-body admittance converts wrench errors to CoM/pose corrections; virtually all learned humanoid controllers run PD/impedance servos (500–1000 Hz) beneath a 50–200 Hz policy — impedance is the compliance buffer that makes sim-to-real work.

---

## 2. Walking & gait generation

### 2.1 Pattern generators
- **Kajita preview control** (ICRA 2003): cart-table model with CoM jerk input; LQ preview controller with integral action, state feedback, and a weighted sum of future ZMP references (~1.6–2 s preview). Gains precomputed offline → tiny online cost. Proven on HRP-2P/2/4.
- **Footstep planning**: MIQP over safe regions (Deits & Tedrake 2014); multi-contact eCMP WPG (Caron & Kheddar, arXiv:1607.08729); rough-terrain nonlinear predictive control (arXiv:1703.00688).

### 2.2 Capture-point / DCM gait
- Capture-point stepping (Pratt 2006) with swing-time correction $x_f \approx \xi - \dot x T_{swing}/2$.
- DCM trajectory generation (Englsberger 2013/2015) with VRP $\mathbf r=c-\ddot c/\omega^2$ and eCMP; analytic exponential interpolation between step setpoints.
- Step timing adaptation — Khadiv et al., "Walking Control Based on Step Timing Adaptation," *T-RO* 2020 (arXiv:1704.01271); key result: only the **next** step needs optimization (ideal for low-compute robots). Predictive foot placement with timing (Missura & Bennewitz, IROS 2017). Push-recovery MPC+CP lead author is **Shafiee-Ashtiani** (Khadiv co-author), SURENA III (arXiv:1612.08034).

### 2.3 Templates, HZD, MPC, RL
- **SLIP / virtual pendulum (Raibert 1986; Full & Koditschek 1999)** — spring-mass template for running/walking; foot-placement law $x_{foot}=x_{com}+v_{com}T/2+k_v e_v$, hopping-height via thrust impulse.
- **Hybrid zero dynamics / virtual constraints** (Westervelt, Grizzle, Koditschek, *IEEE TAC* 2003): exponentially stable periodic orbits via Poincaré analysis; formal stability, MABEL fast running (Sreenath et al., *IJRR* 2013); **CLF + compliant HZD — Reher & Ames 2021** (gait library + CLF-QP with proofs, Cassie, *not* Westerberg); compliant-ground HZD (Luo et al. 2025).
- **MPC walking**: **Scianca et al., *T-RO* 2020, DOI 10.1109/TRO.2019.2958483** (tested on NAO and HRP-4); **Ding, Khazoom, Chignoli & Kim 2022** orientation-aware MPC uses an aSRBM + task-space controller (not a point-mass flywheel); Caron open-source LIPM walking controller (github.com/stephane-caron/lipm_walking_controller).
- **RL walking**: Rudin et al., "Learning to Walk in Minutes" (arXiv:2109.11978); Radosavovic et al. (Science Robotics 2024); Unitree RL Gym; residual/parameterized locomotion (Li et al., ICRA 2021; IJRR 2024 arXiv:2401.16889 — Li/Peng/Abbeel/Levine/Berseth/Sreenath, Cassie, dual-history); Digit sim-to-real RL (Castillo, Weng, Zhang & Hereid — first on Digit without domain randomization); 2025 G1 walk–run (RuN, Li et al., 0–2.5 m/s) and getting-up (He, Dong, Chen & Gupta, RSS 2025).
- **CPGs** (Matsuoka/Hopf oscillators; Ijspeert review 2008): cheap phase generators; useful as a gait clock on an MCU, but lack formal stability — typically paired with feedback.

### 2.4 Deployed systems
ASIMO/HRP (ZMP+LIPM); Atlas (DCM + step adjustment + WBC); TORO (3D DCM + hierarchical ID); iCub (capture point + momentum control); Digit (ALIP + WBC + RL); Unitree G1/H1, Figure, Tesla Optimus (predominantly learning-based WBC with PD/impedance low level).

---

## 3. Running & dynamic locomotion

- **SLIP template** (Blickhan 1989; Farley, Glasheen & McMahon 1993): stance as a linear spring $F=k(L_0-L)$; flight ballistic. Dimensionless parameters (Froude number) govern gait. Approximate analytic solutions (Saranli 2010; Uyanık, Morgül & Saranli, *IEEE T-RO* 2015); differential flatness (Chen, Wensing & Zhang, arXiv:1901.05840); aSLIP/vSLIP over rough terrain (Li et al. 2023, arXiv:2109.05714).
- **Raibert hopping control**: $x_{foot}=x_{com}+v_{com}T/2+k_v e_v$; vertical thrust sets apex height; attitude via hip torque. Directly translatable to a small biped's running template.
- **Template-and-anchor** methodology (Full & Koditschek 1999): SLIP as template, a specific mechanical morphology as anchor.
- **HZD running**: virtual constraints + Poincaré stability; MABEL's fast running; climbing/extending to 3D.
- **MPC**: MIT Cheetah 3 convex MPC (Di Carlo et al., IROS 2018, DOI 10.1109/IROS.2018.8594448); WBIC (Kim et al. 2019); representation-free MPC (Ding et al., *T-RO* 2021); contact-timing optimization.
- **RL / sim-to-real**: PPO (Schulman 2017), SAC (Haarnoja 2018); dynamics randomization (Peng 2017); massively parallel training (Rudin 2022); AMP (Peng 2021), DreamWaQ (Nahrendra 2023), CALM/CaT (Chane-Sane 2024); real-world humanoid locomotion (Radosavovic 2024); unified walk/run/recovery via state-dependent AMP (Lu et al. 2026, arXiv:2605.18611); chasing running via retargeted RL (Olkin et al. 2026, arXiv:2603.25902).
- **Fall recovery / getting up**: FRASA (Gaspard et al., ICRA 2025), HiFAR multi-stage curriculum (Chen et al. 2025, arXiv:2502.20061), cross-morphology get-up (Spraggett 2025, arXiv:2512.12230) — relevant to the get-up work already in this repo.
- **Actuation matters**: series-elastic actuators (SEA), quasi-direct-drive (QDD), and foot compliance set the achievable bandwidth; RL policies assume fast torque/current loops.
- **Safety**: control barrier functions (CBFs) — collision-cone CBF (Tayal & Kolathaya 2023), multi-layered CBF+MPC (Grandia et al. 2021), safe RL (Yang et al. 2022), Agile-but-Safe (He et al., RSS 2024).

---

## 4. Combat / humanoid fighting

**Competition landscape.** RoboOne (Japan, 2002–present) rewards falling the opponent and fast get-up; the 2025 Hangzhou "Mecha Fighting Series" (teleoperated Unitree G1) and the 2025–2026 World Humanoid Robot Games / URKL leagues show striking with scoring for punches, kicks, and knockdowns. Recurring lesson: **reliable fall detection + fast get-up matter more than punching power**, and strikes are often "soft pushes" when balance is not managed.

### 4.1 Physics of striking
- **Resolved momentum control** (Kajita et al., IROS 2003): prescribe linear/angular momentum trajectories; pseudo-inverse distributes them across arm + support leg → whole-body coordination; demonstrated on HRP-2.
- **Centroidal convex momentum** (Ponton et al. 2016) for multi-contact impulse planning.
- **Impact-aware QP** (Wang, Tanguy, Gergondet & Kheddar, arXiv:2001.08454; 2023 T-RO preprint arXiv:2308.06784): velocity jump $\Delta\dot x=-(1+c_r)P_n\dot x^-$; predict post-impact state inside the QP; zero-step capture-region / CoM-velocity-area criteria are less conservative than strict ZMP bounds.
- **Push recovery for impact**: CAM hip strategies (Schuller et al. 2021; Kim et al. 2022); robust "bracing for impact" control (Yang et al. 2026, arXiv:2505.11495); active stabilization of impact motions with unknown reaction forces (Yi et al. 2012).

### 4.2 Learning for combat
- RoboSumo / emergent complexity (Al-Shedivat 2018; Bansal 2018) — adversarial self-play produces pushing and recovery behaviors.
- Sim-to-real soccer (Haarnoja et al., *Science Robotics* 2024, arXiv:2304.13653); i-Sim2Real; humanoid-gym.
- **2025–2026 autonomous boxing**: RoboStriker (arXiv:2601.22517), SLMP (arXiv:2603.01294), HoloBoxing (arXiv:2610.32909), RPG (arXiv:2604.21355) — all use **a low-level tracking/WBC controller and a compact latent/task-space high-level policy**; never raw joint-torque exploration.
- **Safe aggression** via CBF/HJ reachability — computationally heavy, mostly shown on mobile manipulators.

### 4.3 Perception, compliance, rules
Vision-based anticipation of punches; variable-stiffness actuators / SEA for absorbing and delivering impulse; impedance for contact buffering. Rules (weight classes, ring-out, knockdown/8–10 count, striking restrictions) directly reward robust get-up and stable recovery over raw power.

---

## 5. Comparative synthesis

| Family | Core idea | Strengths | Weaknesses | Compute | Hardware record |
|---|---|---|---|---|---|
| ZMP criterion | GRF center in support | rigorous tipping test | not a controller | trivial | universal |
| LIPM / preview | point mass, fixed height | analytic, O(1), industrial | height/momentum mismatch | trivial | ASIMO, HRP |
| Capture point / DCM | $\xi=c+\dot c/\omega$ | first-order balance metric; unifies strategies | needs $\dot c$, constant height | trivial | Atlas, TORO, iCub |
| DCM + step timing | reference $\xi$, ZMP/CMP feedback | push recovery, capturability | model mismatch | trivial–low | Atlas, HRP-4, SURENA |
| Centroidal/WBC/HQP | prioritized task-space QP | handles contacts, torque limits | model + solver heavy | high (PC) | Atlas, HRP-4, TORO |
| Linear MPC (LIPM) | receding-horizon QP | constraints, preview | model mismatch, solve time | med (x86) | Bolt, Atlas |
| Nonlinear MPC / TO | full centroidal + contacts | highest fidelity | heavy, local minima | high (10–50 ms) | ANYmal, Digit |
| HZD / hybrid | virtual constraints, Poincaré | formal stability | design-heavy | low–med | MABEL, Cassie |
| SLIP / Raibert | spring template | simple running/foot placement | template mismatch | trivial | Raibert's hoppers, MABEL |
| CPG phase clock | oscillator | cheap, natural phase | no stability guarantee | trivial | many small bipeds |
| RL policy (MLP) | learned proprioceptive map | robust, minimal modeling | training cost, opacity | low at inference (10⁴ MAC) | G1, Digit |
| RL transformer/foundation | history-conditioned attention | general, in-context | huge train/infer | high | G1, Berkeley Humanoid |
| Momentum observer | generalized-momentum residual | sensorless disturbance estimation | drift, detection delay | low $O(n)$ | iCub, arms |
| EKF state estimation | IMU + kinematic correction | no force sensors | contact/observability | low–med | HRP, Atlas, Digit |
| Impedance/admittance | mass-spring-damper law | compliance, sim-to-real interface | tuning, passivity | trivial | universal |

---

## 6. Recommendation for Rocky Mk1 (RP2350, 21 DoF, 2-axis IMU, 200 Hz, f32)

**Hardware reality.** 5 ms/tick; no FP64 (software-emulated → avoid `double`); 520 KB SRAM fits small matrices and a tiny NN but not dense QP workspaces; a 2-axis IMU gives pitch/roll + rates but no yaw gyro and no base linear velocity.

### Feasible on-chip at 200 Hz (Tier 1)
1. **Joint PD/impedance loops** — trivial; the compliance buffer for everything else.
2. **Complementary filter / 4-state EKF** for pitch/roll (IMU + leg-encoder contact) — ~10⁴ flops.
3. **Support-relative DCM balance law** (the single best choice): compute $\xi_{rel}=\xi-p_{foot}$ using the support-relative formulation (arXiv:2608.00500) so no base velocity is needed. Desired ZMP/CMP:
   $$p_d = \xi_{rel} + k_1(\xi_{rel}-\xi_{ref}) + k_2\dot L_{hip}$$
   Convert to ankle/hip position or torque offsets. A few dozen flops.
4. **Strategy selection by capturability** — compare $\xi$ to support bounds; ankle → hip → step.
5. **CPG-lite gait clock** — phase accumulator or Hopf oscillator, L/R offset $\pi$; analytic LIPM CoM reference with precomputed sinh/cosh tables; Raibert/capture foot placement $x_f=\xi+k_v(\dot x-\dot x_{des})+\dot x T_s/2$; swing foot Bézier arc; ZMP clamp + step shortening à la Khadiv.
6. **Small fixed-iteration f32 LIPM-MPC** (N≈4–8, 2 vars/step, hand-coded active-set/ADMM) — borderline but feasible with precomputed matrices; start with a single-step DCM correction law.
7. **2-axis momentum observer residual** for push detection; **quantized int8 MLP** (2×32/3×32) mapping IMU+encoders → residual joint targets or gait parameters (~1–3k MACs) — feasible.
8. **CBF safety shield** on CoM velocity/tilt (scalar analytic barrier) for "safe aggression".

### Requires an external PC (Tier 2)
Full CMM + inverse-dynamics WBC/HQP QP, nonlinear MPC / trajectory optimization, transformer policies (Radosavovic-style), vision/LiDAR odometry, and all RL training. If a companion SBC exists, run LIPM-MPC / footstep planning there at 50–100 Hz and send $\xi_{ref}$, foot targets, and gains down to the RP2350, which handles IK, PD, CPG, and safety.

### Practical roadmap keyed to this repository
- **Now (already built):** fall detection, classified get-up library, consent-gated recovery state machine, pulsing red alarm, keyframe action studio — this maps directly onto the RoboOne lesson that fall detection + fast get-up are decisive.
- **Next (P1):** support-relative DCM balance overlay on `rk-control`, ankle/hip blending, capturability-based strategy switching, CPG-lite walk phase generator replacing/augmenting the current oscillator, ZMP clamp + step shortening.
- **Then (P2):** impact-aware punching authored offline (host-side QP/trajectory optimization, momentum-shaped strikes with torso counter-rotation), input-shaping of keyframes to suppress residual vibration, scripted "combo" moves with automatic balance restore.
- **Later (P3, if a host computer + richer sensing are added):** offline-trained distilled MLP / latent-space boxing policy (RoboStriker/SLMP/HoloBoxing recipe) as a residual on the analytic controller, with CBF safety shield; quantized int8 inference on MCU.

**Why this split works:** it mirrors the resource-constrained humanoid lineage (NimbRo-OP2X, SURENA-Mini, RoboOne/DARwIn-OP, 21-DoF IJHR walking engine) — closed-form LIPM/DCM + capturability strategy switching + impedance low level is interpretable, debuggable, has a decades-long real-robot record, and fits the 5 ms f32 budget with large margin. Learning is layered on later as a performance enhancer, never as the sole safety-critical authority.

---

## 7. Selected references (full lists in the per-topic reports)

- Vukobratović & Borovac 2004 (ZMP) — worldscientific.com/doi/10.1142/S0219843604000635
- Kajita et al. 2003 (preview control) — doi:10.1109/ROBOT.2003.1241826
- Pratt et al. 2006 (capture point) — ieeexplore.ieee.org/document/4115602
- Englsberger et al. 2015 (DCM) — doi:10.1109/TRO.2015.2405592
- Khadiv et al. 2020 (step timing) — arxiv.org/abs/1704.01271
- Caron et al. — arxiv.org/abs/1801.07022, arxiv.org/abs/1809.07073; tutorial scaron.info/robotics/how-do-biped-robots-walk.html
- Orin, Goswami & Lee 2013 (CMM); Khatib 1987 (operational space)
- Kuindersma et al. 2016 (Atlas WBC); Di Carlo et al. 2018 (Cheetah 3 MPC) doi:10.1109/IROS.2018.8594448
- Westervelt, Grizzle & Koditschek 2003 (HZD) doi:10.1109/TAC.2002.806653
- Raibert 1986 (*Legged Robots That Balance*); Full & Koditschek 1999 (templates)
- Rudin et al. 2022 — arxiv.org/abs/2109.11978; Radosavovic et al. 2024 — arxiv.org/abs/2303.03381
- Wang, Tanguy & Kheddar — arxiv.org/abs/2001.08454, arxiv.org/abs/2308.06784
- Haarnoja et al. 2024 (soccer) — arxiv.org/abs/2304.13653; RoboStriker/SLMP/HoloBoxing 2026 — arxiv.org/abs/2601.22517 / 2603.01294 / 2610.32909
- Rotella et al. 2014 (EKF) — arxiv.org/abs/1402.5450; Zhou et al. 2026 (support-relative CP) — arxiv.org/abs/2608.00500
