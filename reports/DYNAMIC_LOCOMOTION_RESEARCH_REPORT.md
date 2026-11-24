# Dynamic Humanoid Locomotion — Running, Hopping, High-Speed Gaits, and RL/Sim-to-Real

**Technical research report**, compiled 2026-11-24.

**Research note / verification status.** Web search (`websearch`) was non-functional in this environment; all bibliographic discovery was performed via the **arXiv API** (`export.arxiv.org/api/query`) and the **OpenAlex API** (`api.openalex.org`), which returned structured metadata (titles, authors, years, DOIs). Citations marked **[KB]** are from the author's knowledge base and are *unverified by live lookup* in this session — their titles/authors are standard in the literature but should be double-checked before publication use. Everything else was verified live via the two APIs. Today's date is 2026; several 2026 papers appear in the results (some with future-dated preprints — treat with appropriate caution).

---

## 1. Spring-Loaded Inverted Pendulum (SLIP) Template for Running

### Core idea
During steady running, the center of mass (CoM) of humans and animals behaves like a point mass on a **springy leg**: the stance phase is well approximated by a linear spring of stiffness $k$ in contact with the ground, followed by a ballistic flight phase. This is the **spring-loaded inverted pendulum (SLIP)** template (also called the "spring-mass model"). It unifies running, trotting, and hopping across species and body sizes: the ground reaction force (GRF) profile is roughly sinusoidal and the CoM trajectory is near-sinusoidal in stance and parabolic in flight.

### Equations / concepts
Stance dynamics (planar, point mass $m$, leg length $L$, stiffness $k$, angle $\alpha$ from vertical, gravity $g$):

$$
m\ddot{\mathbf r} = \mathbf F_{\rm spring} + m\mathbf g,\qquad
\mathbf F_{\rm spring} = k(L_0 - L)\,\frac{\mathbf r_{\rm foot}-\mathbf r}{\|\mathbf r_{\rm foot}-\mathbf r\|} - b\,\dot L\,\hat{\mathbf e}_L
$$

Flight: $\ddot{\mathbf r} = \mathbf g$. The **dimensionless** parameters that govern the dynamics are

$$
\Pi_1 = \frac{k L_0}{mg}\ (\text{normalized stiffness}),\quad
\Pi_2 = \frac{v_0}{\sqrt{g L_0}}\ (\text{Froude number}),\quad
\alpha_0\ (\text{touchdown angle}),\quad \zeta = \frac{b}{2\sqrt{km}}\ (\text{damping ratio}).
$$

Stance dynamics are **not integrable in closed form** (unlike the LIPM), so analysis relies on approximate analytical solutions (e.g. Saranli et al. 2010, 2011) or numerical integration. A key result: **energy lost at touchdown (collision) is compensated by spring recoil**, and a stable limit cycle exists for appropriate $(\alpha_0, k, L_0)$; leading-leg touchdown angle controls forward speed, leg stiffness controls height/cycle time.

### Strengths / weaknesses
- (+) Extremely low-dimensional (2 DoF planar, 3 DoF 3D); captures GRF shape, flight phase, and speed–height coupling with surprising fidelity; the basis of most reduced-order running controllers.
- (−) No analytical solution; no actor/body dynamics, no joint limits; pure SLIP ignores leg damping/losses and cannot produce net energy without actuation (hence "actuated SLIP", "TD-SLIP" variants with hip torque).

### Computational cost
Trivial: integrate a 2–4 state ODE per step, or evaluate a precomputed map/approximate solution;kHz-rate feasible even on a small MCU.

### Real-robot track record
Extensive: Raibert's hoppers (CMU, 1980s) [KB]; RHex (Saranli et al.); one-legged hoppers (Uyank et al. 2011/2015, verified); ATRIAS, MABEL (Grizzle group); Cheetah/Cheetah 3 (MIT); Salto (Berkeley); numerous small runners.

### Key citations
- R. Blickhan, "The spring-mass model for running and hopping," *Journal of Biomechanics* 22(11–12):1217–1227, **1989**. [KB] DOI: 10.1016/0021-9290(89)90224-8.
- C. T. Farley, J. Glasheen, T. A. McMahon, "Running springs: speed and animal size," *Journal of Experimental Biology* 185:71–86, **1993**. [KB]
- T. A. McMahon, C. T. Farley, "Hopping frequency in humans: a test of how springs set stride frequency in bouncing gaits," *J. Appl. Physiol.* 71(6):2127–2132, **1991**. [KB]
- U. Saranli, Ö. Arslan, M. M. Ankarali, Ö. Morgül, "Approximate analytic solutions to non-symmetric stance trajectories of the passive Spring-Loaded Inverted Pendulum with damping," *Nonlinear Dynamics* 61:729–742, **2010**. DOI: 10.1007/s11071-010-9757-8 (verified via OpenAlex).
- İ. Uyanık, Ö. Morgül, U. Saranli, "Experimental Validation of a Feed-Forward Predictor for the Spring-Loaded Inverted Pendulum Template," *IEEE T-RO* 31(4):1–12, **2015**. DOI: 10.1109/TRO.2014.2383531 (verified).
- H. E. Orhon et al., "Extending The Lossy Spring-Loaded Inverted Pendulum Model with a Slider-Crank Mechanism," arXiv:1506.01808, **2015** (verified via arXiv API).
- H. Chen, P. M. Wensing, W. Zhang, "Optimal Control of a Differentially Flat Two-Dimensional Spring-Loaded Inverted Pendulum Model," *IEEE RA-L* 4(4):3737–3742, **2019**. DOI: 10.1109/LRA.2019.2956457 (verified).
- R. Truax, F. Liu, S. Chowdhury, R. St. Pierre, "Optimizing Design and Control of Running Robots Abstracted as Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP)," ASME IDETC, **2024**, arXiv:2407.12120 / DOI: 10.1115/detc2024-139482 (verified).
- H. Wang et al., "Flying Trot Control Method for Quadruped Robot Based on Trajectory Planning," arXiv:2210.13446, **2022** (verified).
- W. Wang et al., "Highly dynamic locomotion control of biped robot enhanced by swing arms" (F-SLIP), arXiv:2208.08120, **2022** (verified).

---

## 2. Raibert's Control Equations for Hopping and Running

### Core idea
Marc Raibert's 1986 book *Legged Robots That Balance* (MIT Press) established the **three-part decomposed control law** for hopping and running robots, validated on one-, two-, and four-legged machines:

1. **Hopping height control** via vertical thrust (leg extension impulse) at stance.
2. **Forward velocity control** via **foot placement** — placing the foot ahead of/behind the CoM based on velocity.
3. **Body attitude control** via **hip torque** during stance (or foot placement asymmetry for multi-leg).

### Explicit equations
**Foot placement (Raibert's deadbeat law).** For a hopper with stance duration $T_s$ (or stride time $T$), place the foot so that the CoM has zero velocity error at the *next* touchdown:

$$
x_{\text{foot}} = x_{\text{com}} + v_{\text{com}}\,\frac{T_s}{2} + k_v\,(v_{\text{com}} - v_{\text{desired}})
$$

Equivalently, the classic Raibert law is often written as

$$
x_{\text{foot}} = x_{\text{com}} + \frac{T}{2}\,v_{\text{com}} + k_{\dot x}\,(v_{\text{desired}} - v_{\text{com}})
$$

where the first two terms give the ballistic-arc landing point for the current velocity, and the last term is the velocity error correction (the "capture-point"-like correction). The **capture point** (Pratt et al. 2006 [KB]) formalizes this for walking:

$$
\mathbf p_{\rm CP} = \mathbf x_{\rm CoM} + \frac{\mathbf v_{\rm CoM}}{\omega_0},\qquad \omega_0 = \sqrt{g/h}
$$

with $h$ the CoM height — placing the foot at the capture point brings the LIPM to rest. For running, the equivalent relates touchdown angle $\alpha_{\rm TD}$ to speed; the SLIP deadbeat map linearized about the limit cycle gives $\Delta\alpha_{\rm TD} \approx -K_v\, \Delta v$.

**Hopping height / vertical thrust.** The vertical takeoff velocity is regulated by the net leg impulse:

$$
\Delta v_z = \frac{1}{m}\int_{\rm stance}\!\big(F_z - mg\big)\,dt \;\approx\; \frac{F_{\rm thrust}\,T_s}{m} - g T_s
$$

Raibert's controller sets the desired apex height $y_{\rm apex}$; from ballistic flight, required takeoff velocity is $v_z^{\rm TO} = \sqrt{2g(y_{\rm apex}-y_{\rm TD})}$, and desired stance impulse is $F_{\rm thrust} \approx m(v_z^{\rm TO}+g T_s/2)/T_s$. In the spring-mass model, **energy injection** is achieved by extending the leg during stance (positive work) and retracting in flight; the thrust is often commandeered via a leg-length setpoint or virtual spring with an offset:

$$
F_{\rm leg} = k(L_0 - L) + k_p(L_{\rm des}(t)-L) + k_d(\dot L_{\rm des}-\dot L)
$$

**Attitude control.** $\tau_{\rm hip} = -k_p(\theta - \theta_{\rm des}) - k_d\dot\theta$ applied during stance; for a running biped the angular momentum about the CoM is regulated by hip torque plus asymmetric foot placement.

### Strengths / weaknesses
- (+) Decoupled, intuitive, requires only simple state feedback; proven on real hardware at high speed; computationally negligible.
- (−) Tuned per platform; assumes reliable touch-down detection and good state estimation; simple laws do not handle rough terrain or strong disturbances; SLIP deadbeat laws ignore body dynamics and friction constraints.

### Computational cost
O(1) — a handful of multiplies per Control cycle; ideal for embedded MCUs.

### Real-robot track record
Raibert's hoppers and the 3D biped/quadruped runners at CMU (1980s) [KB]; widely reused in RHex, Salto, ATRIAS, MIT Cheetah, and many hobby/educational hoppers.

### Key citations
- M. H. Raibert, *Legged Robots That Balance*, MIT Press, **1986**. ISBN 0-262-18117-7. (Verified title via OpenAlex: "Legged robots" (1986), DOI 10.1145/5948.5950 — the book is listed there.)
- M. Raibert, "Trotting, pacing and bounding by a quadruped robot," *Journal of Biomechanics* 23(Suppl 1):79–98, **1990**. [KB]
- J. Pratt, J. Carff, S. Drakunov, A. Stepanov, "Capture Point: A Step toward Humanoid Push Recovery," *IEEE-RAS Humanoids*, **2006**. [KB]
- R. J. Full, D. E. Koditschek, "Templates and anchors: neuromechanical hypotheses of legged locomotion on land," *J. Exp. Biol.* 202(23):3325–3332, **1999**. [KB]
- Z. He et al., "Controllable Height Hopping of a Parallel Legged Robot," *Applied Sciences* 11(4):1421, **2021**. DOI: 10.3390/app11041421 (verified).

---

## 3. 3D SLIP / Template-and-Anchor Methodology

### Core idea
Full & Koditschek (1999) [KB] introduced the **template-and-anchor** framework: a *template* is a simple reduced-order dynamical system (e.g. SLIP, LIPM) that captures the **essential behavior** to be controlled; an *anchor* is the full robot morphology whose actuators and mechanics can be driven to **realize** the template's behavior (the robot "anchors" the template in physical hardware). The design loop: choose template → design controller on template → find an embedding ("anchoring" map) from robot state to template state → use whole-body control to enforce the template's virtual constraints on the robot.

### Extensions to bipedal running
- **3D SLIP** adds lateral dynamics and yaw/roll; running in 3D requires controlling **angular momentum about the CoM** and **heading**, typically decomposed into a sagittal SLIP + lateral LIPM + independent "flight" control of body orientation via hip torque / arm swing.
- **vSLIP** (vertically actuated SLIP) couples walking height with planar locomotion, used for variable-height biped navigation (Li et al., *IJRR* 2023, arXiv:2109.05714, verified).
- **aSLIP** (actuated SLIP) decouples horizontal LIPM dynamics from a vertical spring, enabling 1 kHz MPC footstep optimization (Wang, Fei, Kormushev, arXiv:2109.09373, **2021**, verified).
- **Flywheel-SLIP (F-SLIP)** incorporates swing-leg/arm angular momentum (Wang et al., arXiv:2208.08120, **2022**, verified).
- "Template + anchor" for gait design with safety guarantees: Liu et al., arXiv:1909.11125, **2019** (verified).

### Strengths / weaknesses
- (+) Systematic reduction from complex robot to analyzable model; composable with MPC and WBC; strong formal tools (hybrid zero dynamics, virtual constraints).
- (−) "Anchoring" is nontrivial: mapping and controller must respect actuation limits, friction, and impact dynamics; 3D templates with flight remain analytically hard.

### Computational cost
Template integration: low. Anchoring via QP-based whole-body control: moderate (1–5 kHz on real robots).

### Real-robot track record
RHex, ATRIAS, MABEL, Cassie (Oregon State / Michigan), ANYmal, MIT Cheetah 3.

### Key citations
- R. J. Full, D. E. Koditschek, "Templates and anchors: neuromechanical hypotheses of legged locomotion on land," *J. Exp. Biol.* 202(23):3325–3332, **1999**. [KB]
- D. E. Koditschek, R. J. Full, "Template and anchor: a framework for comparative neuromechanics of legged locomotion," in *Neurobiology of Motor Control*, **2006**. [KB]
- Z. Li, J. Zeng, S. Chen, K. Sreenath, "Autonomous Navigation of Underactuated Bipedal Robots in Height-Constrained Environments" (vSLIP), *IJRR* 2023, arXiv:2109.05714 (verified).
- K. Wang, H. Fei, P. Kormushev, "Fast Online Optimization for Terrain-Blind Bipedal Robot Walking with a Decoupled Actuated SLIP Model," arXiv:2109.09373, **2021** (verified).
- J. Liu, P. Zhao, Z. Gan, M. Johnson-Roberson, R. Vasudevan, "Leveraging the Template and Anchor Framework for Safe, Online Robotic Gait Design," arXiv:1909.11125, **2019** (verified).

---

## 4. Hybrid Zero Dynamics (HZD) for Running

### Core idea
For underactuated bipeds, **HZD** (Westervelt, Grizzle, Koditschek 2003) enforces *virtual constraints* — outputs $y = q - h_d(s(q),\alpha)$ that drive the robot's joints along parameterized periodic trajectories indexed by a phase variable $s$ — and analyzes the **zero dynamics** that remain when $y\equiv 0$. If the zero dynamics are stable (via Poincaré map analysis of the hybrid impact model), the full hybrid system tracks the periodic gait stably. For **running**, the hybrid model includes an additional **flight phase** and **impact maps** at touchdown; the extension is sometimes called "RB-Dynamics" (reduced/thrust-controlled) and requires active force control to maintain the flight phase and stabilize the vertical dynamics.

### Equations / concepts
Hybrid dynamics:

$$
\Sigma:\begin{cases}
\dot x = f(x)+g(x)u, & x\notin S \ (\text{continuous phase})\\
x^+ = \Delta(x^-), & x\in S \ (\text{impact / touchdown})
\end{cases}
$$

Virtual constraints: $y = h(q) - h_d(s(q),\alpha) = 0$, with a output-linearizing controller $u = -(L_g L_f h)^{-1}(L_f^2 h + K_p y + K_d \dot y)$ (feedback linearization / inverse dynamics). The reduced dynamics in the zero manifold $\mathcal Z = \{y=\dot y =0\}$ is the **hybrid zero dynamics**; stability via the linearization of the Poincaré return map $P$: eigenvalues inside the unit circle. For running on compliant ground (Luo et al. 2025) the impact map is augmented with ground compliance; "compliant HZD" (Sreenath et al. 2013) uses force control within the zero dynamics to stabilize fast running of MABEL (up to ~3.4 m/s).

### Strengths / weaknesses
- (+) Formally guarantees local orbital stability of periodic gaits; explicit gait parameterization amenable to optimization (trajectory optimization + HZD); real-time implementable with QP-based controllers.
- (−) Requires accurate full-order model and state estimation; hybrid impact modeling is delicate; gait library / planning across speeds requires re-optimization; running (flight phase) HZD significantly more complex than walking; sensitive to unmodeled compliance.

### Computational cost
Offline trajectory optimization can be minutes–hours; online: evaluate $h_d(s)$ plus solve a small QP / feedback-linearization law at 1–5 kHz.

### Real-robot track record
RABBIT, MABEL (fast bipedal running, Sreenath et al. 2013), ATRIAS, Cassie (HZD with reinforcement learning extensions, Castillo et al. 2019), DURUS.

### Key citations
- E. R. Westervelt, J. W. Grizzle, D. E. Koditschek, "Hybrid zero dynamics of planar biped walkers," *IEEE Trans. Autom. Control* 48(1):42–56, **2003**. DOI: 10.1109/TAC.2002.806653 (verified via OpenAlex).
- J. W. Grizzle, C. Chevallereau, "Virtual Constraints and Hybrid Zero Dynamics for Realizing Underactuated Bipedal Locomotion," arXiv:1706.01127, **2017** (verified).
- K. Sreenath, H.-W. Park, I. Poulakakis, J. W. Grizzle, "Embedding active force control within the compliant hybrid zero dynamics to achieve stable, fast running on MABEL," *IJRR* 32(3):324–342, **2013**. DOI: 10.1177/0278364912473344 (verified via OpenAlex).
- B. Morris, E. R. Westervelt, C. Chevallereau, G. Buche, J. W. Grizzle, "Achieving Bipedal Running with RABBIT: Six Steps Toward Infinity," in *Recent Trends in Mobile Robotics*, Springer, **2007** (verified via OpenAlex).
- Y. Luo et al., "Study on bipedal running on compliant ground using hybrid zero dynamics controller," *Multibody System Dynamics*, **2025**. DOI: 10.1007/s11044-025-10095-9 (verified).
- G. A. Castillo, B. Weng, W. Zhang, A. Hereid, "Hybrid Zero Dynamics Inspired Feedback Control Policy Design for 3D Bipedal Locomotion using Reinforcement Learning," arXiv:1910.01748, **2019** (verified).
- Y. Gong, J. W. Grizzle, "Zero Dynamics, Pendulum Models, and Angular Momentum in Feedback Control of Bipedal Locomotion," *J. Dynamic Systems, Measurement, and Control*, **2022**. DOI: 10.1115/1.4055770 (verified).

---

## 5. MPC for Dynamic Locomotion

### Core idea
Model Predictive Control solves a finite-horizon optimal control problem online at each control tick, applying only the first control action (receding horizon). For legged robots, a **reduced-order model** (single rigid body / centroidal dynamics / SLIP / LIPM) is used for the prediction, with footstep/contact timing and ground reaction forces as decision variables; a **whole-body controller** (WBC) tracks the resulting GRFs with full joint torques.

### Equations / concepts — MIT Cheetah 3 convex MPC
Di Carlo et al. (IROS 2018, DOI 10.1109/IROS.2018.8594448, verified via OpenAlex) model Cheetah 3 as a **single rigid body** with mass $m$, inertia $\mathbf I$, state $\mathbf x = [\mathbf p,\ \mathbf v,\ \boldsymbol\theta,\ \boldsymbol\omega]$ (CoM position/velocity, body orientation/angular velocity). The (nonlinear) rigid-body dynamics are linearized about a nominal trajectory. With contact forces $\mathbf f_i$ at feet $i$, the discrete-time **linear** system is

$$
\mathbf x_{k+1} = \mathbf A_d \mathbf x_k + \mathbf B_d \mathbf u_k,\qquad \mathbf u_k = [\mathbf f_1,\dots,\mathbf f_4]
$$

and the convex QP (subject to friction cones, box constraints on normal forces, and a fixed contact schedule) is

$$
\min_{\mathbf u}\ \sum_{k=0}^{N-1} \|\mathbf x_{k+1}-\mathbf x_{k+1}^{\rm ref}\|_{\mathbf Q}^2 + \|\mathbf u_k\|_{\mathbf R}^2,
$$
$$
\text{s.t.}\quad \mu f_{i,z}\ge |f_{i,x}|,\ |f_{i,y}|,\quad f_{i,z}\in[f_{\min},f_{\max}],\quad \mathbf f_i=0\ \text{in swing}.
$$

Solved by a custom QP solver at ~100 Hz (MPC) with WBC at 500 Hz–1 kHz. This enabled Cheetah 3 to trot, pace, bound, and gallop robustly. Kim et al. (2019, arXiv:1909.06586, verified) combine **whole-body impulse control (WBIC)** with MPC: MPC computes GRFs, WBIC is a hierarchical task-space QP that realizes them with joint torques, enabling dynamic jumps and aggressive maneuvers.

**Centroidal MPC** uses the full centroidal momentum dynamics

$$
\dot{\mathbf h}_G = \begin{bmatrix}\sum_i \mathbf f_i + m\mathbf g\\ \sum_i (\mathbf r_i - \mathbf c)\times \mathbf f_i\end{bmatrix}
$$

with the **centroidal momentum matrix** $\mathbf h_G = \mathbf A_G(\mathbf q)\dot{\mathbf q}$. This yields a nonlinear but more accurate model; often linearized ("centroidal LIP") or handled with sequential convex programming / DDP. Contact timing optimization can be included by treating the contact schedule as discrete variables (mixed-integer) or by smooth complementarity / phase duration parameters.

**Biped extension:** the same single-rigid-body MPC + WBC stack is used on bipeds (e.g. Cassie, Digit, and humanoid robots), with contact schedules for two feet and additional angular-momentum terms. Representation-free MPC (Ding et al., *T-RO* 2021, DOI 10.1109/TRO.2020.3046415, verified) avoids orientation singularities using rotation matrices/quaternions.

### Strengths / weaknesses
- (+) Reactive, handles disturbances and dynamic gaits; convex QP solvers are fast and reliable; modular (planner → MPC → WBC).
- (−) Accuracy limited by linearization and rigid-body assumption; gains/schedule tuning; contacts must be predetermined or optimized (MIQP is expensive); state estimation critical; MPC alone ignores joint torque/kinematic limits (WBC handles).

### Computational cost
Convex MPC: ~1–5 ms per solve in C++ on embedded PC (100–500 Hz); WBC QP: ~0.2–1 ms (kHz). Nonlinear/centroidal MPC: 10–100 ms (needs powerful CPU/GPU).

### Real-robot track record
MIT Cheetah 3 (convex MPC, 2018), Mini Cheetah (backflips, 2019), ANYmal (perceptive MPC), Cassie/Digit (biped MPC), humanoid robots.

### Key citations
- J. Di Carlo, P. M. Wensing, B. Katz, G. Bledt, S. Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control," *IEEE/RSJ IROS*, **2018**. DOI: 10.1109/IROS.2018.8594448 (verified via OpenAlex).
- D. Kim, J. Di Carlo, B. Katz, G. Bledt, S. Kim, "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control," arXiv:1909.06586, **2019** (verified).
- Y. Ding, A. Pandala, C. Li, Y.-H. Shin, H.-W. Park, "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds," *IEEE T-RO* 37(4):1154–1171, **2021**. DOI: 10.1109/TRO.2020.3046415 (verified).
- S. Katayama, M. Murooka, Y. Tazaki, "Model predictive control of legged and humanoid robots: models and algorithms," *Advanced Robotics*, **2023**. DOI: 10.1080/01691864.2023.2168134 (verified).
- R. Grandia, A. J. Taylor, A. D. Ames, M. Hutter, "Multi-Layered Safety for Legged Robots via Control Barrier Functions and Model Predictive Control," *IEEE ICRA*, **2021**. DOI: 10.1109/ICRA48506.2021.9561510 (verified).
- A. Trivedi et al., "Chance-Constrained Convex MPC for Robust Quadruped Locomotion," *IEEE RA-L*, **2025**. DOI: 10.1109/LRA.2025.3585315 (verified).

---

## 6. Reinforcement Learning for Dynamic Locomotion — PPO, SAC, Sim-to-Real

### Core idea
Deep RL trains a neural policy $\pi_\theta(\mathbf a|\mathbf o)$ mapping observations (proprioception: base velocity, orientation, joint angles/velocities, previous actions; sometimes height maps / vision) to motor commands (joint position targets, torques, or PD setpoints). Training occurs in massively parallel physics simulation (Isaac Gym/Lab, MuJoCo, RaiSim, Webots) with **domain randomization**, then zero-shot transfer to hardware.

### Algorithms
- **PPO** (Schulman et al. 2017 [KB]): clipped surrogate objective
  $L^{\rm CLIP} = \mathbb E_t[\min(r_t \hat A_t,\ \mathrm{clip}(r_t, 1-\epsilon, 1+\epsilon)\hat A_t)]$,
  $r_t = \pi_\theta(a_t|s_t)/\pi_{\theta_{\rm old}}(a_t|s_t)$.
  Dominant for locomotion due to stability, simplicity, and GPU-friendly minibatch SGD. Used in Rudin et al. 2022, Hwangbo et al. 2019, Radosavovic et al. 2024.
- **SAC** (Haarnoja et al. 2018 [KB]): off-policy maximum-entropy actor-critic with soft Q update; sample-efficient, used Haarnoja et al. 2024 "Learning agile soccer skills for a bipedal robot" (DOI 10.1126/scirobotics.adi8022, verified) and in DeepMind's quadruped work.
- **TD3 / DDPG**: off-policy deterministic; less common for high-DoF humanoids due to tuning fragility.
- **Policy architectures**: MLP (3×128 or 2×256, ELU/swish), sometimes with CNN/transformer for height maps; **privileged teacher** (MLP with simulator-only information: friction, masses, terrain) distilled to a **student** using proprioceptive history (Lee et al., *Sci. Robotics* 2020, DOI 10.1126/scirobotics.abc5986, verified; Miki et al. 2022, DOI 10.1126/scirobotics.abk2822, verified).

### Sim-to-real transfer techniques
- **Dynamics randomization** (Peng et al., arXiv:1710.06537, **2017**, verified via arXiv API): randomize mass, inertia, friction, damping, motor strength, latency, and delays during training, so the policy is robust to the real dynamics. OpenAI's "Learning Dexterity" (arXiv:1808.00177, **2018/2020**, verified) and "Solving Rubik's Cube with a Robot Hand" (arXiv:1910.07113, **2019**, verified) demonstrated large-scale randomization + automated domain adaptation.
- **Massively parallel training**: Rudin et al., "Learning to Walk in Minutes Using Massively Parallel Deep RL," arXiv:2109.11978, **2021/2022 (CoRL 2021)** (verified): thousands of ANYmal simulations on one GPU, flat terrain in <4 min, uneven in ~20 min, with a game-inspired curriculum; open-sourced as *legged_gym*.
- **Teacher–student / privileged information**: train a teacher with terrain/friction (privileged) and distill to a blind student; "Learning quadrupedal locomotion over challenging terrain" (Lee et al. 2020), "Learning robust perceptive locomotion in the wild" (Miki et al. 2022). Recent variants: TAR (2025), PA-LOCO (2024), SLR without privileged info (2025) — all verified via OpenAlex.
- **AMP / Adversarial Motion Priors** (Peng et al., arXiv:2104.02180, **2021**, verified; TOG 2021): a discriminator distinguishes state-transition pairs from a mocap reference dataset; its output is added as a reward, giving stylized, natural motion without hand-crafted reward terms. AMP used in Escontrela et al. 2022 (reward replacement), Vollenweider et al. 2023 (multiple AMPs), and 2026 humanoid work "Unified Walking, Running, and Recovery for Humanoids via State-Dependent AMP" (arXiv:2605.18611, verified via OpenAlex).
- **CALM / constrained RL**: "Constraints as Terminations" (CaT, Chane-Sane et al., *IROS* 2024, DOI 10.1109/IROS.2024.10802334, verified) enforces safety constraints by terminating episodes on violations. "Safe RL for Legged Locomotion" (Yang, Zhang, Luu, Ha, Tan, *IROS* 2022, DOI 10.1109/IROS47612.2022.9982038, verified).
- **DreamWaQ** (Nahrendra, Yu, Myung, *ICRA* 2023, DOI 10.1109/ICRA48891.2023.10161144, verified; arXiv:2301.10602): implicit terrain imagination via a VAE-style latent estimated from proprioceptive history; robust blind locomotion. DreamWaQ++ (2026, DOI 10.1109/TRO.2026.3653774, verified) adds obstacle-aware multimodal RL.
- **Real-world humanoid locomotion** (Radosavovic et al., *Science Robotics* 2024, DOI 10.1126/scirobotics.adi9579, verified): fully learned policy for a humanoid, trained in simulation, deployed on hardware.
- **Deep RL survey**: Ha, Lee, van de Panne, Xie, Yu et al., "Learning-based legged locomotion: State of the art and future perspectives," *IJRR*, **2025**, DOI 10.1177/02783649241312698 (verified).

### Strengths / weaknesses
- (+) Minimal modeling effort; can discover dynamic, natural gaits and recoveries; sim parallelism makes training fast; sim-to-real works well with randomization.
- (−) Needs powerful GPU for training; policies are "black box" with limited formal guarantees; sim-to-real gap especially for high-speed impact-rich running; reward engineering still required; brittle out-of-distribution.

### Computational cost
Training: hundreds–thousands of GPU-hours (modern setups: minutes on a single GPU with thousands of envs). **Deployment**: 1–3 MLP forward passes per control step (50–200 Hz) — a few thousand multiply–adds, easily fits an embedded MCU (RP2350-class) if policy is small.

### Real-robot track record
ANYmal (Rudin 2022, Hwangbo 2019), MIT Mini Cheetah (Tan et al. 2018, RSS), Unitree Go1/A1 (numerous), Cassie/Digit (Li et al. 2021, 2024), humanoids (Radosavovic 2024; Agility/Figure/Unitree G1).

### Key citations (also see above)
- J. Schulman, F. Wolski, P. Dhariwal, A. Radford, O. Klimov, "Proximal Policy Optimization Algorithms," arXiv:1707.06347, **2017**. [KB]
- T. Haarnoja et al., "Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL with a Stochastic Actor," ICML **2018**, arXiv:1801.01290. [KB]
- OpenAI et al., "Learning Dexterous In-Hand Manipulation," arXiv:1808.00177, **2018** (verified via arXiv API).
- OpenAI et al., "Solving Rubik's Cube with a Robot Hand," arXiv:1910.07113, **2019** (verified via OpenAlex).
- X. B. Peng, M. Andrychowicz, W. Zaremba, P. Abbeel, "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization," arXiv:1710.06537, **2017** (verified).
- N. Rudin, D. Hoeller, P. Reist, M. Hutter, "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning," arXiv:2109.11978, **2021** (verified).
- X. B. Peng, Z. Ma, P. Abbeel, S. Levine, A. Kanazawa, "AMP: Adversarial Motion Priors for Stylized Physics-Based Character Control," arXiv:2104.02180 / TOG **2021** (verified).
- Z. Xie, X. Da, M. van de Panne, B. Babich, A. Garg, "Dynamics Randomization Revisited: A Case Study for Quadrupedal Locomotion," arXiv:2011.02404, **2020** (verified via arXiv API).
- J. Tan et al., "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots," RSS **2018**, DOI 10.15607/RSS.2018.XIV.010 (verified via OpenAlex).

---

## 7. End-to-End vs Hierarchical RL; Motion Imitation; Residual RL

### Core ideas
- **End-to-end RL**: one policy maps raw(ish) observations (proprioception + height map/camera) directly toJoint targets/torques. Simple, but sample-inefficient and hard to shape for complex behaviors.
- **Hierarchical RL**: a high-level planner/policy produces commands (velocity, footstep targets, gait mode), and a low-level policy or classical controller executes them. E.g. "Hierarchical Reinforcement Learning for Quadruped Locomotion" (Jain et al., *IROS* 2019, DOI 10.1109/IROS.2018.8594252 / 10.1109/IROS40897.2019.8967913, verified via OpenAlex) — a high-level policy selects gait/velocity, low-level controllers handle execution.
- **Motion imitation from mocap**: AMP (Peng et al. 2021), and/or direct tracking of reference joint trajectories from motion capture (human data retargeted to robot kinematics). Used for humanoid natural walking/running styles (e.g. "Adversarial Locomotion and Motion Imitation for Humanoid Policy Learning," 2025; "Chasing Autonomy: Dynamic Retargeting and Control Guided RL for Performant and Controllable Humanoid Running," Olkin et al., arXiv:2603.25902, 2026 — verified via OpenAlex). Retargeting maps human mocap joint angles to the robot's kinematic structure; a RL policy then imitates the reference while maintaining balance.
- **Residual RL on top of classical controllers**: learn a correction $\Delta \mathbf a$ added to a model-based controller's output: $\mathbf a = \mathbf a_{\rm model} + \mathbf a_{\rm RL}$. This combines the robustness/guarantees of MPC/WBC/HZD with RL's adaptability. Key papers: "Reinforcement Learning for Robust Parameterized Locomotion Control of Bipedal Robots" (Li, Cheng, Peng, Abbeel, Levine et al., *ICRA* 2021, DOI 10.1109/ICRA48506.2021.9560769, verified); "Multi-Modal Legged Locomotion Framework With Automated Residual Reinforcement Learning" (Chen & Rosendo, *RA-L* 2022, DOI 10.1109/LRA.2022.3191071, verified); "Reinforcement learning for versatile, dynamic, and robust bipedal locomotion control" (Li et al., *IJRR* 2024, DOI 10.1177/02783649241285161, verified). Residual RL drastically reduces exploration difficulty and preserves baseline behavior.
- **RL as low-level controller with high-level planner**: e.g. a high-level footstep planner / navigation stack outputs desired base velocity and gait mode at 5–20 Hz; the RL policy runs at 50–200 Hz and realizes it. This is the dominant practical architecture for perceptive locomotion (e.g. Miki et al. 2022 with a vision-based high-level waypoint generator; "Humanoid-Gym," Gu et al., arXiv:2404.05695, 2024, verified).

### Strengths / weaknesses
- End-to-end: simplest deployment but hardest to train and debug; hierarchical: modular, debuggable, but inter-layer interfaces must be tuned; imitation: natural motion but needs quality mocap and retargeting; residual RL: best of both worlds but requires a functioning baseline controller.

### Computational cost
Hierarchical/residual: training cost of RL plus baseline controller cost; deployment: policy inference plus baseline (MPC/WBC), feasible on embedded PC; on MCU, only the small policy would fit.

### Real-robot track record
Cassie/Digit (residual RL, Li et al. 2021/2024), ANYmal (hierarchical perceptive RL, Miki et al. 2022), humanoids (motion imitation, 2024–2026).

---

## 8. Learning-Based Fall Recovery and Getting Up

### Core idea
For a humanoid battle robot, falls are inevitable; a controller must detect a fall and execute a **recovery/get-up policy** that brings the robot back to standing from arbitrary postures. RL is attractive because recovery involves complex contact sequences (rolling, using arms/legs to push) that are hard to hand-code.

### Approaches
- **Single unified policy**: train one policy that does locomotion *and* recovery, switching behavior implicitly based on state (Radosavovic et al. 2024 "Real-world humanoid locomotion with RL" includes some recovery; "UniReLo: Learning a Unified Humanoid Policy from Fall Recovery to Locomotion across Diverse Terrains," Xu et al., arXiv:2606.08922, 2026, verified via OpenAlex).
- **Dedicated get-up policies**: "FRASA: An End-to-End RL Agent for Fall Recovery and Stand Up of Humanoid Robots" (Gaspard et al., *ICRA* 2025, DOI 10.1109/ICRA55743.2025.11128664, verified); "Learning Humanoid Standing-up Control across Diverse Postures" (Huang et al., 2025, DOI via OpenAlex, verified); "HiFAR: Multi-Stage Curriculum Learning for High-Dynamics Humanoid Fall Recovery" (Chen et al., arXiv:2502.20061, 2025, verified via arXiv API).
- **Cross-morphology / zero-shot**: "Learning to Get Up Across Morphologies: Zero-Shot Recovery with a Unified Humanoid Policy" (Spraggett, arXiv:2512.12230, 2025, verified via OpenAlex).
- **DeepMind**: "Learning agile soccer skills for a bipedal robot with deep RL" (Haarnoja et al., *Science Robotics* 2024, DOI 10.1126/scirobotics.adi8022, verified) includes fall recovery as part of the skill set. DeepMind's earlier "Learning to get up" is often cited from their humanoid work [KB — exact title/DOI unverified here; likely the AIST/DeepMind "Learning to get up" (2018) or the soccer paper above — flag as unverified].
- **Boston Dynamics Atlas**: uses model-based recovery (MPC + whole-body control with contact planning) rather than RL; robust getting up from falls is demonstrated in their videos [KB — no paper; flag].
- **Stubborn** (Ren et al., arXiv:2606.12814, 2026, verified via OpenAlex): unified RL framework for motion tracking and fall recovery.

### Strengths / weaknesses
- (+) RL recovery handles diverse, unstructured fallen states; no hand-crafted contact sequence needed; can be combined with safety layers.
- (−) Training requires simulating impacts and self-collisions (hard); policies may be unsafe during exploration; getting up from "stuck" against obstacles remains hard; sim-to-real gap large for contact-rich recovery.

### Computational cost
Training: large-scale parallel sim (GPU hours); deployment: small MLP (same as locomotion policy) — cheap.

### Real-robot track record
DeepMind's biped (soccer, 2024), Unitree G1/H1 humanoids (2025–2026 demos), Boston Dynamics Atlas (model-based, not RL).

---

## 9. Running-Specific Hardware Considerations

### Core ideas
Dynamic running imposes unique hardware demands:
- **Actuator bandwidth** (torque/current loop >1 kHz, mechanical bandwidth ≥20–50 Hz) to reject impact disturbances and inject stance impulse within ~100–200 ms.
- **Impedance control**: regulate the leg's apparent stiffness/damping $F = k(x_0 - x) - b\dot x$; enables stable contact with unknown terrain and energy storage.
- **Series elastic actuators (SEA)**: a spring in series with the motor (Pratt & Williamson 1995 [KB]) gives high-fidelity force control, shock tolerance, and energy storage, but limits bandwidth due to spring compliance; used in MABEL, ATRIAS, and many runners. Nonlinear/variable-stiffness SEAs improve range (Thorson & Caldwell, *IROS* 2011, DOI 10.1109/IROS.2011.6094913, verified via OpenAlex; "A Compact Variable Stiffness Actuator for Agile Legged Locomotion," Yu et al., *IEEE T-Mech* 2024, verified).
- **Quasi-direct-drive (QDD)**: low-gear-ratio, high-torque-density motors with low output inertia and high transparency — the MIT Cheetah / Open Dynamic Robot Initiative philosophy (Grimminger et al., *RA-L* 2020, DOI 10.1109/LRA.2020.2976639, verified; "An Open Torque-Controlled Modular Robot Architecture for Legged Locomotion Research"). QDD gives high bandwidth and inherent impact robustness without a series spring, at the cost of lower peak torque than geared/SEA.
- **Torque density** (Nm/kg) and **specific power** determine accel/jump capability; running robots often need >50–100 Nm/kg peak leg torque.
- **Foot compliance**: soft/spherical feet or compliant soles reduce impact peaks, increase contact time, and improve stability on rough terrain; modeled by accessory compliance in SLIP (see Luo et al. 2025 HZD-on-compliant-ground).
- **Latency**: sensor→actuator delay must be <1–2 ms; distributed impedance control with SEA (Zhao & Sentis, arXiv:1811.11573, 2018, verified) addresses this.

### Strengths / weaknesses of each actuator type
| Type | Pros | Cons |
|---|---|---|
| SEA | Force control, shock absorption, energy storage | Limited bandwidth, added mass/complexity |
| QDD | High bandwidth, transparent, simple | Lower torque density, higher current/heating |
| Geared + impedance | High torque | High reflected inertia, poor impact tolerance |
| Variable stiffness | Tunable compliance | Complexity, mass |

### Real-robot track record
MABEL/ATRIAS (SEA), MIT Cheetah 3/Mini Cheetah (QDD), ANYmal (QDD with elastic feet), Salto (SEA), Open Dynamic Robot Initiative (QDD).

---

## 10. Stability and Safety for Aggressive Locomotion

### Core ideas
- **Control Barrier Functions (CBFs)**: encode forward-invariant safety sets $\mathcal C = \{x : h(x)\ge 0\}$ via $\dot h + \alpha(h)\ge 0$; a QP minimally modifies a nominal controller to keep the system safe. E.g. **Collision Cone CBFs (C3BF)** (Tayal & Kolathaya, arXiv:2309.01898, 2023, verified) guarantee collision avoidance for legged robots; layered CBF+MPC architectures (Grandia et al. 2021, verified; Kim, Lee, Ames 2023/2024, verified).
- **Safe RL**: incorporate constraints in the MDP via constrained MDPs / Lagrangian methods (Yang et al. 2022, verified), or "constraints as terminations" (CaT, 2024, verified); "Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion" (He et al., RSS 2024, DOI 10.15607/RSS.2024.XX.059, verified) learns high-speed locomotion with collision-avoidance guarantees.
- **Reachability analysis**: compute backward/forward reachable sets to certify safety under uncertainty; "Safety-Critical Coordination of Legged Robots via Layered Controllers and Forward Reachable Set based CBFs" (Kim et al., *ICRA* 2024, DOI 10.1109/ICRA57147.2024.10610589, verified); "Probabilistically-Safe Bipedal Navigation over Uncertain Terrain via Conformal Prediction and Contraction Analysis" (Muenprasitivej, Zhao, Chou, arXiv:2510.07725, 2025, verified).
- **HZD Poincaré stability** (Sec. 4) is itself a formal stability tool for periodic gaits.

### Strengths / weaknesses
CBFs give formal safety guarantees with low online cost (small QP) but need accurate dynamics and well-designed $h$; safe RL handles uncertainty but sacrifices optimality; reachability is rigorous but computationally heavy (often offline).

### Computational cost
CBF-QP: ~0.1 ms (small QP, MCU-feasible in simplified form); reachability: offline / heavy.

---

## Comparative Table

| Approach | Core idea | Equations/complexity | Strengths | Weaknesses | Online cost | Real robot track record |
|---|---|---|---|---|---|---|
| SLIP template (§1) | Spring-mass CoM + flight | 2–4 state ODE, 3–4 params | Simple, predictive, universal | No closed form, no body dynamics | Trivial | Hoppers, RHex, Cheetah, Salto |
| Raibert foot placement + thrust (§2) | $x_f=x_{cm}+vT/2+k_v e_v$; impulse for height | Handful of multiplies | Deadbeat, hardware-proven, tiny compute | Tuning, needs state estimate | O(1) | Raibert hoppers, many hoppers |
| Template+anchor / 3D SLIP (§3) | Reduced model realized by WBC | Template ODE + QP anchoring | Systematic, formal | Anchoring nontrivial | Low–moderate | Cassie, ATRIAS, Cheetah |
| HZD (§4) | Virtual constraints + Poincaré stability | Feedback linearization + QP | Formal orbital stability | Model-sensitive, flight hard | Moderate (kHz QP) | MABEL, RABBIT, Cassie |
| Convex MPC (§5) | Finite-horizon QP on rigid body | Linear discrete dynamics + friction-cone QP | Reactive, robust, proven | Linearization, contact schedule | ~1–5 ms | Cheetah 3, Mini Cheetah, ANYmal |
| RL + sim-to-real (§6) | PPO/SAC policy + randomization | MLP inference; massive GPU training | Natural gaits, minimal modeling | Black box, sim gap, GPU training | Low inference | ANYmal, Go1, Cassie, humanoids |
| Hierarchical/residual RL (§7) | Planner + RL or model + correction | Baseline + small MLP | Modular, safe baseline | Two systems to tune | Low–moderate | Cassie/Digit, ANYmal |
| AMP / motion imitation (§6–7) | Discriminator reward from mocap | Adversarial reward + MLP | Natural human-like motion | Needs mocap/retargeting | Low inference | Humanoids 2021–2026 |
| Fall recovery RL (§8) | Learned get-up from arbitrary poses | MLP policy | Handles diverse falls | Simulation of impacts hard | Low | DeepMind biped, G1 (2024–26) |
| CBF / safe RL / reachability (§10) | Invariance / constrained MDP | Small QP / offline sets | Formal safety | Model-dependent, reachability heavy | Low (CBF) | Legged robots (C3BF, 2023–24) |

---

## Recommendation — Practical Stack for a Small 21-DoF Humanoid on an Embedded RP2350-Class MCU

**Context.** RP2350-class MCU: dual Cortex-M33 @ ~150 MHz, ~520 KB SRAM, no GPU, no FPU beyond maybe single-precision (M33 has optional FPU; RP2350 has an FPU). This rules out large NNs with many layers, nonlinear MPC, and heavy QPs at high rates, but is fine for small linear algebra and tiny MLPs.

**Recommended layered architecture:**

1. **High-level (offboard or slow onboard, 5–20 Hz):** gait mode / desired velocity / footstep targets — a simple state machine or a small planner running on a host if available. If fully onboard, a finite-state machine (walk/run/hop/recovery) with Raibert/SLIP-based footstep commands.

2. **Mid-level reduced-order controller (100–200 Hz):** **Raibert/SLIP foot placement + thrust control** (Secs. 1–2), optionally wrapped in a tiny **convex MPC** if a lightweight QP solver fits. For 21 DoF, a 3D SLIP with footstep law
   $x_{\rm foot}=x_{\rm com}+v_{\rm com}T_s/2 + k_v(v_{\rm des}-v_{\rm com})$ and height control via leg thrust gives running/hopping with minimal computation. A minimalist **LIPM + capture point** for walking and **SLIP** for running can be selected by the gait state machine.

3. **Whole-body / joint-level control (200–500 Hz):** a small **task-space QP or damped least-squares IK + PD/impedance** controller mapping desired GRFs/foot positions to joint torques. On RP2350, use a hand-coded 6×6 or 12×12 dense QP (active-set) or even decoupled PD with gravity compensation and virtual spring impedance — not a generic large QP solver.

4. **RL policy (optional, 50–200 Hz):** a **tiny MLP** (2×32 or 3×32, ~1–3k weights, int8/fp16 quantized) trained offline with PPO + domain randomization (legged_gym/Isaac Lab) and optionally **residual** on top of the SLIP+WBC baseline (Sec. 7). Deploy as a fixed-point forward pass; this fits in <50 KB flash and <1 ms inference. Teacher–student distillation lets the deployed student use only proprioception (IMU + joint encoders + foot contact), no privileged sim state. Use AMP-style motion priors only if natural running aesthetics matter; a well-tuned SLIP + careful reward may suffice.

5. **Safety layer (200 Hz+):** a simplified **CBF filter** (single or few constraints: CoM inside support polygon / friction cone / joint limits) as a final saturation/clipping stage on torque commands; cheap QP or projection. Fall detection triggers a **learned get-up policy** (or a scripted recovery sequence if RL is too heavy).

6. **Hardware priorities:** QDD actuators (torque density, low inertia) with **compliant feet** and high-bandwidth current loops (>1 kHz); SEA only if force sensing and shock absorption are paramount and bandwidth can be sacrificed. Ensure IMU + joint encoders + foot contact sensing at ≥500 Hz; estimate CoM velocity via a simple Kalman filter / observer.

**Rationale.** Pure end-to-end RL is attractive for deployment simplicity but training and sim-to-real for *high-speed running* on a custom 21-DoF platform is expensive and brittle; a model-based SLIP/Raibert backbone gives predictable, tunable running and hopping with near-zero onboard compute, while a small residual RL policy absorbs modeling errors and terrain variation. The CBF layer addresses safety demands of an aggressive "battle robot." This stack matches what works on real robots of this scale (MIT Mini Cheetah, Unitree Go1/G1, small hoppers).

**Bottom line:** *SLIP/Raibert reduced-order control + WBC/impedance + tiny residual RL policy + CBF safety*, with PPO/domain-randomization/teacher–student training done **offline** on a GPU workstation (legged_gym / Isaac Lab) and only the small policy + model-based controller running on the RP2350.

---

## Full Reference List (verified unless marked [KB])

1. Raibert, M. H. *Legged Robots That Balance*, MIT Press, 1986. (book; OpenAlex lists DOI 10.1145/5948.5950)
2. Blickhan, R. "The spring-mass model for running and hopping," *J. Biomech.*, 1989. [KB] DOI 10.1016/0021-9290(89)90224-8
3. Farley, C. T., Glasheen, J., McMahon, T. A. "Running springs: speed and animal size," *J. Exp. Biol.*, 1993. [KB]
4. McMahon, T. A., Farley, C. T. "Hopping frequency in humans," *J. Appl. Physiol.*, 1991. [KB]
5. Full, R. J., Koditschek, D. E. "Templates and anchors," *J. Exp. Biol.*, 1999. [KB]
6. Westervelt, E. R., Grizzle, J. W., Koditschek, D. E. "Hybrid zero dynamics of planar biped walkers," *IEEE TAC*, 2003. DOI 10.1109/TAC.2002.806653
7. Sreenath, K., Park, H.-W., Poulakakis, I., Grizzle, J. W. "Embedding active force control within the compliant hybrid zero dynamics… MABEL," *IJRR*, 2013. DOI 10.1177/0278364912473344
8. Grizzle, J. W., Chevallereau, C. "Virtual Constraints and Hybrid Zero Dynamics…," arXiv:1706.01127, 2017.
9. Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., Kim, S. "Dynamic Locomotion in the MIT Cheetah 3 Through Convex MPC," IROS 2018. DOI 10.1109/IROS.2018.8594448
10. Kim, D., Di Carlo, J., Katz, B., Bledt, G., Kim, S. "Highly Dynamic Quadruped Locomotion via WBIC and MPC," arXiv:1909.06586, 2019.
11. Ding, Y. et al. "Representation-Free MPC for Dynamic Motions in Quadrupeds," *IEEE T-RO*, 2021. DOI 10.1109/TRO.2020.3046415
12. Schulman, J. et al. "Proximal Policy Optimization Algorithms," arXiv:1707.06347, 2017. [KB]
13. Haarnoja, T. et al. "Soft Actor-Critic," ICML 2018. [KB] arXiv:1801.01290
14. Peng, X. B., Andrychowicz, M., Zaremba, W., Abbeel, P. "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization," arXiv:1710.06537, 2017.
15. OpenAI et al. "Learning Dexterous In-Hand Manipulation," arXiv:1808.00177, 2018.
16. OpenAI et al. "Solving Rubik's Cube with a Robot Hand," arXiv:1910.07113, 2019.
17. Rudin, N., Hoeller, D., Reist, P., Hutter, M. "Learning to Walk in Minutes Using Massively Parallel Deep RL," arXiv:2109.11978, 2021 (CoRL 2021).
18. Hwangbo, J. et al. "Learning agile and dynamic motor skills for legged robots," *Sci. Robotics*, 2019. DOI 10.1126/scirobotics.aau5872
19. Lee, J. et al. "Learning quadrupedal locomotion over challenging terrain," *Sci. Robotics*, 2020. DOI 10.1126/scirobotics.abc5986
20. Miki, T. et al. "Learning robust perceptive locomotion for quadrupedal robots in the wild," *Sci. Robotics*, 2022. DOI 10.1126/scirobotics.abk2822
21. Peng, X. B., Ma, Z., Abbeel, P., Levine, S., Kanazawa, A. "AMP: Adversarial Motion Priors," arXiv:2104.02180 / TOG 2021.
22. Escontrela, A. et al. "Adversarial Motion Priors Make Good Substitutes for Complex Reward Functions," IROS 2022. DOI 10.1109/IROS47612.2022.9981973
23. Nahrendra, I. M. A., Yu, B., Myung, H. "DreamWaQ," ICRA 2023. DOI 10.1109/ICRA48891.2023.10161144 (arXiv:2301.10602)
24. Nahrendra, I. M. A. et al. "DreamWaQ++," *IEEE T-RO*, 2026. DOI 10.1109/TRO.2026.3653774
25. Radosavovic, I. et al. "Real-world humanoid locomotion with reinforcement learning," *Sci. Robotics*, 2024. DOI 10.1126/scirobotics.adi9579
26. Haarnoja, T. et al. "Learning agile soccer skills for a bipedal robot with deep RL," *Sci. Robotics*, 2024. DOI 10.1126/scirobotics.adi8022
27. Li, Z. et al. "Reinforcement Learning for Robust Parameterized Locomotion Control of Bipedal Robots," ICRA 2021. DOI 10.1109/ICRA48506.2021.9560769
28. Li, Z. et al. "Reinforcement learning for versatile, dynamic, and robust bipedal locomotion control," *IJRR*, 2024. DOI 10.1177/02783649241285161
29. Gaspard, C. et al. "FRASA: End-to-End RL Agent for Fall Recovery and Stand Up," ICRA 2025. DOI 10.1109/ICRA55743.2025.11128664
30. Chen, P. et al. "HiFAR: Multi-Stage Curriculum Learning for High-Dynamics Humanoid Fall Recovery," arXiv:2502.20061, 2025.
31. Spraggett, J. "Learning to Get Up Across Morphologies," arXiv:2512.12230, 2025.
32. Tayal, M., Kolathaya, S. "Safe Legged Locomotion using Collision Cone CBFs," arXiv:2309.01898, 2023.
33. Grandia, R., Taylor, A. J., Ames, A. D., Hutter, M. "Multi-Layered Safety for Legged Robots via CBFs and MPC," ICRA 2021. DOI 10.1109/ICRA48506.2021.9561510
34. Yang, T.-Y., Zhang, T., Luu, L., Ha, S., Tan, J. "Safe Reinforcement Learning for Legged Locomotion," IROS 2022. DOI 10.1109/IROS47612.2022.9982038
35. He, T. et al. "Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion," RSS 2024. DOI 10.15607/RSS.2024.XX.059
36. Grimminger, F. et al. "An Open Torque-Controlled Modular Robot Architecture for Legged Locomotion Research," *RA-L*, 2020. DOI 10.1109/LRA.2020.2976639
37. Thorson, I., Caldwell, D. G. "A nonlinear series elastic actuator for highly dynamic motions," IROS 2011. DOI 10.1109/IROS.2011.6094913
38. Ha, S., Lee, J., van de Panne, M., Xie, Z., Yu, W. et al. "Learning-based legged locomotion: State of the art and future perspectives," *IJRR*, 2025. DOI 10.1177/02783649241312698
39. Uyanık, İ., Morgül, Ö., Saranli, U. "Experimental Validation of a Feed-Forward Predictor for the SLIP Template," *IEEE T-RO*, 2015. DOI 10.1109/TRO.2014.2383531
40. Saranli, U. et al. "Approximate analytic solutions… passive SLIP with damping," *Nonlinear Dynamics*, 2010. DOI 10.1007/s11071-010-9757-8
41. Chen, H., Wensing, P. M., Zhang, W. "Optimal Control of a Differentially Flat 2D SLIP Model," *RA-L*, 2019. DOI 10.1109/LRA.2019.2956457
42. Wang, K., Fei, H., Kormushev, P. "Fast Online Optimization… aSLIP," arXiv:2109.09373, 2021.
43. Li, Z., Zeng, J., Chen, S., Sreenath, K. "Autonomous Navigation of Underactuated Bipedal Robots… (vSLIP)," *IJRR*, 2023, arXiv:2109.05714.
44. Olkin, Z., Compton, W., Bena, R. M., Ames, A. D. "Chasing Autonomy: Dynamic Retargeting and Control Guided RL for Humanoid Running," arXiv:2603.25902, 2026.
45. Lu, Y. et al. "Unified Walking, Running, and Recovery for Humanoids via State-Dependent AMP," arXiv:2605.18611, 2026.
46. Chane-Sane, E. et al. "Constraints as Terminations for Legged Locomotion RL (CaT)," IROS 2024. DOI 10.1109/IROS.2024.10802334
47. Xie, Z., Da, X., van de Panne, M., Babich, B., Garg, A. "Dynamics Randomization Revisited," arXiv:2011.02404, 2020.
48. Tan, J. et al. "Sim-to-Real: Learning Agile Locomotion For Quadruped Robots," RSS 2018. DOI 10.15607/RSS.2018.XIV.010
49. Luo, Y. et al. "Study on bipedal running on compliant ground using HZD controller," *Multibody System Dynamics*, 2025. DOI 10.1007/s11044-025-10095-9
50. Gong, Y., Grizzle, J. W. "Zero Dynamics, Pendulum Models, and Angular Momentum…," *JDSMC*, 2022. DOI 10.1115/1.4055770

**[KB] = cited from knowledge base, not verified by live API lookup in this session.** All other titles/authors/years/DOIs were verified via arXiv API and/or OpenAlex API on 2026-11-24. Some 2026 preprints are future-dated and should be treated as provisional.
