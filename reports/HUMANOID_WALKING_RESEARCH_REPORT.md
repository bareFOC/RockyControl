# Humanoid Robot Walking & Gait Generation: A Technical Survey (through 2026)

**Scope.** This report surveys the major families of algorithms for bipedal/humanoid gait generation, balance, and control, from classical ZMP preview control through modern learning-based approaches. For each family: core idea, key equations, strengths/weaknesses, computational cost, and real-robot track record. It closes with a comparative table and a concrete recommendation for a small 21-DoF humanoid on an RP2350-class embedded MCU (no heavy floating-point solver).

---

## 0. Common mathematical backbone

Nearly all model-based walking controllers share the same reduction of the full multibody dynamics to a low-dimensional template.

**Assumptions (Caron's "walking cookbook"):**
1. Joint torques are sufficient to realize desired motions.
2. Angular momentum about the CoM is negligible (arms counter-swing locked/small).
3. CoM height is constant.

Under these, the full dynamics reduce to the **3D Linear Inverted Pendulum Model (LIPM)** (Kajita et al., IROS 2001):

$$\ddot{p}_G = \omega^2 (p_G - p_Z), \qquad \omega^2 = \frac{g}{h}$$

where $p_G$ is the horizontal CoM position, $p_Z$ the ZMP (zero moment point / zero tilting moment point), and $h$ the constant CoM height. Equivalently $p_Z = p_G - \ddot{p}_G/\omega^2$. The ZMP must remain inside the support polygon $\mathcal{S}$. Intuitively, walking is controlled falling: shift ZMP behind CoM to accelerate forward, then ahead to decelerate.

**DCM / capture-point decomposition.** Define the **divergent component of motion (DCM)**, also called the instantaneous capture point (ICP) or capture point (CP):

$$\xi = p_G + \frac{\dot{p}_G}{\omega}$$

Then the LIPM splits into a *stable* and an *unstable* first-order system:

$$\dot{p}_G = \omega(\xi - p_G) \quad \text{(stable)}, \qquad \dot{\xi} = \omega(\xi - p_Z) \quad \text{(unstable)}$$

If the ZMP $p_Z$ is held fixed at $p$, then $\xi(t) = p + (\xi_0-p)e^{\omega t}$ and the CoM converges to $p$. The point at which the robot can come to rest is the **capture point**: place foot at $\xi$ (modulo swing time and angular momentum). This decomposition underlies Secs. 2 and 7.

*Key reference:* Stéphane Caron, "How do biped robots walk?", https://scaron.info/robotics/how-do-biped-robots-walk.html (excellent accessible derivation, includes ZMP, LIPM, DCM, MPC, and preview-control formulas).

---

## 1. Pattern generators / trajectory generation

### 1.1 ZMP-based preview control (Kajita et al., 2003)

**Paper:** S. Kajita, F. Kanehiro, K. Kaneko, K. Fujiwara, K. Harada, K. Yokoi, H. Hirukawa, "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point," *Proc. IEEE ICRA 2003*, pp. 1620–1627. DOI: 10.1109/ROBOT.2003.1241826. PDF: https://mzucker.github.io/swarthmore/e91_s2013/readings/kajita2003preview.pdf ; summary: https://peasant98.github.io/digest/kajita2003-preview-control

**Core idea.** Treat walking pattern generation as a *ZMP-tracking servo*. Take the CoM jerk as control input $u=\dddot{x}$; with state $(x,\dot{x},\ddot{x})$ the cart-table model ($p_Z = x - (z_c/g)\ddot{x}$) becomes a linear strictly-proper system. A discrete LQ **preview controller** (after Katayama et al.) minimizes

$$J = \sum \left[ Q_e (p_Z^{\text{ref}}[k]-p_Z[k])^2 + R u[k]^2 \right]$$

and combines (i) integral action on ZMP error, (ii) state feedback, and (iii) a weighted sum of *future* ZMP references over $N_L$ steps. The CoM must begin moving *before* the ZMP steps to the next foot — hence preview.

**Equations/concepts.** Discrete augmented system $\begin{bmatrix}x_{k+1}\\e_{k+1}\end{bmatrix} = \begin{bmatrix}A&0\\-C&A\end{bmatrix}\begin{bmatrix}x_k\\e_k\end{bmatrix}+\begin{bmatrix}B\\-D\end{bmatrix}u_k+\begin{bmatrix}0\\I\end{bmatrix}p_{k}^{\text{ref}}$ with optimal control $u_k = -K x_k - K_e\sum_{i=0}^{k}e_i + \sum_{j=1}^{N_L} f_j\, p_{k+j}^{\text{ref}}$. Preview gain becomes negligible beyond ~2 s; ~1.6 s is enough in practice.

**Multibody correction.** A second preview loop takes the ZMP error of the full multibody model (computed via inverse dynamics after IK places the pelvis on the CoM trajectory) and feeds it back, delayed by the preview period, as a compensation. This reduced max ZMP error on HRP-2P from 2.3 cm (x) / 1.6 cm (y) to 1.2 cm / 0.4 cm.

**Strengths.** Simple, globally optimal LQ solution (gains precomputed offline → tiny online cost: a few matrix-vector products per step). Arbitrary foot placements; can walk on stepping stones (unlike earlier inverted-pendulum methods that moved the feet to keep balance). Proven on real robots.

**Weaknesses.** Footstep locations, step timing, and ZMP reference must be fixed in advance; single CoM height plane $z_c$ for all steps; no inherent push recovery (requires outer feedback layers); relies on assumptions 1–3 above.

**Computational cost.** Very low online: a fixed-gain recursion (state feedback + finite preview sum, $N_L\approx 100$–$200$ samples). Suitable even for microcontrollers.

**Track record.** HRP-2P (simulation with spiral stairs in the original paper), then HRP-2 and HRP-4 real robots (AIST); widely reused as the "classic" WPG. Also basis of Honda-style ZMP walking.

### 1.2 LIPM + MPC generalization; footstep planning

**Wieber (2006)** recast preview control as finite-horizon linear MPC with inequalities (ZMP inside support polygon): P.-B. Wieber, "Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Weak Perturbations," *IEEE-RAS Humanoids 2006*, DOI: 10.1109/ICHR.2006.321375. Extensions: footstep adaptation (Herdt et al., IROS 2010, DOI: 10.1109/IROS.2010.5654429), CoM height variation (Caron et al., Humanoids 2015).

**Footstep planning.**
- R. Deits & R. Tedrake, "Footstep planning on uneven terrain with mixed-integer convex optimization," *IEEE-RAS Humanoids 2014*, pp. 279–286. PDF: https://groups.csail.mit.edu/robotics-center/public_papers/Deits14b.pdf — uses IRIS safe regions + MIQP to choose footsteps in clutter; solved in seconds, not for inner-loop MCU use.
- J. Caron & A. Kheddar, "Multi-contact Walking Pattern Generation based on Model Preview Control of 3D COM Accelerations," *Humanoids 2016*, arXiv:1607.08729 (https://arxiv.org/abs/1607.08729). Introduces eCMP and multi-contact ZMP support areas.
- S. Caron, A. Kheddar, et al., "Dynamic walking over rough terrains by nonlinear predictive control of the floating-base inverted pendulum," *IROS 2017*, arXiv:1703.00688 (https://arxiv.org/pdf/1703.00688v2.pdf) — auto step durations over rough terrain.
- Sequential/reactive footstep MPC: K. Narkhede et al., "A Sequential MPC Approach to Reactive Planning for Bipedal Robot Footsteps," 2022 (https://par.nsf.gov/servlets/purl/10376311).

---

## 2. Capture-point / DCM-based gait

### 2.1 Capture point (Pratt et al., 2006)

**Paper:** J. Pratt, J. Carff, S. Drakunov, A. Goswami, "Capture Point: A Step Toward Humanoid Push Recovery," *6th IEEE-RAS Int. Conf. on Humanoid Robots*, Dec 2006, pp. 200–207. PDF: http://ambarish.com/paper/Pratt_Goswami_Humanoids2006.pdf

**Core idea.** The instantaneous capture point $\xi=x+\dot{x}/\omega$ is where the robot must step to "capture" itself (bring CoM to rest) neglecting angular momentum and assuming fixed ZMP. If $\xi$ lies inside the base of support, no step needed (ankle/hip strategy); otherwise must step so that the new foot lands at/around $\xi$. Accounts for swing time: $\xi_{\text{step}} = \xi_{\text{desired}} - \dot{x} T_{\text{swing}}$ roughly. (Extended to variable CoM height by **Caron, Escande, Lanari, Mallein, "Capturability-based Pattern Generation for Walking with Variable Height," IEEE T-RO 2019, DOI 10.1109/TRO.2019.2923971, arXiv:1801.07022** — **verified title/authors/venue**; generalizes capturability to the variable-height inverted pendulum for uneven terrain.)

**Strengths.** Extremely cheap (a few flops), intuitive, directly yields foot placement from measured state; ideal for reactive stepping and push recovery.

**Weaknesses.** LIPM assumptions; ignores angular momentum, finite foot size (unless using capturability regions — see Caron et al., arXiv:1801.07022, verified title "Capturability-based Pattern Generation for Walking with Variable Height," https://arxiv.org/abs/1801.07022), and swing-foot dynamics; point-foot abstraction.

### 2.2 DCM trajectory generation (Englsberger et al., 2013/2015)

**Papers:**
- J. Englsberger, C. Ott, A. Albu-Schäffer, "Three-dimensional bipedal walking control using Divergent Component of Motion," *IROS 2013*, pp. 404–411. PDF: https://elib.dlr.de/87255/1/IROS_2013_FinalSubmission_JEnglsberger.pdf
- J. Englsberger, C. Ott, A. Albu-Schäffer, "Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion," *IEEE Trans. Robotics* 31(2):355–368, 2015, DOI: 10.1109/TRO.2015.2405592. (DLR / TUM; authors/venue verified at https://portal.fis.tum.de/en/publications/three-dimensional-bipedal-walking-control-based-on-divergent-comp . The abstract emphasizes encoding external forces via eCMP/VRP, continuous leg-force profiles, toe-off in double support, and verification in simulation **and experiments** — plotting the framework as experimental rather than simulation-only.)

**Core idea.** Extend the 1D DCM decomposition to 3D. Introduce the **enhanced centroidal moment pivot (eCMP)** $\mathbf{p}$ (generalized ZMP accounting for centroidal angular momentum) and the **virtual repellent point (VRP)** $\mathbf{r} = \mathbf{c} - \ddot{\mathbf{c}}/\omega^2$ (a ZMP-like point that can lie outside the support polygon, allowing "non-physical" but DCM-consistent commands). The governing equations (with constant height $z_c$):

$$\dot{\boldsymbol{\xi}} = \omega(\boldsymbol{\xi} - \mathbf{r}), \qquad \dot{\mathbf{c}} = \omega(\boldsymbol{\xi} - \mathbf{c})$$

**DCM trajectory generation:** design a smooth DCM reference that exponentially interpolates between successive eCMP setpoints while each foot is in support: $\boldsymbol{\xi}(t) = \mathbf{r} + (\boldsymbol{\xi}(t_0)-\mathbf{r}) e^{\omega(t-t_0)}$ during single support, with boundary conditions enforced by solving for appropriate VRP trajectories. In double support the DCM is steered from the old eCMP to the new one. The CoM follows stably.

**Strengths.** Provides an analytic, closed-form, computationally trivial gait generator with explicit stability semantics; separates stable CoM tracking from unstable DCM planning; combines naturally with MPC and with step adjustment; works on torque-controllable humanoids (TORO).

**Weaknesses.** Constant-height/nonzero-angular-momentum approximations; VRP may leave support polygon (must be mapped to feasible CoP); step timing/location still planned by an outer layer; DCM tracking assumes accurate state estimation.

### 2.3 Step timing adaptation for push recovery (Khadiv, Missura, Pratt lineage)

**Papers:**
- M. Khadiv, A. Herzog, S. A. A. Moosavian, L. Righetti, "Step timing adjustment: A step toward generating robust gaits," *IEEE-RAS Humanoids 2016*; extended journal version **"Walking Control Based on Step Timing Adaptation," *IEEE Trans. Robotics* (2020), arXiv:1704.01271** (https://arxiv.org/abs/1704.01271 — verified). Key result (from the verified abstract): it is **sufficient to optimize only the next step's timing and location** (not a multi-step horizon), which remains computationally simple while guaranteeing that any viable state stays viable; validated in simulation on a humanoid with **passive ankles** under pushes and foot slippage. This makes it particularly attractive for low-compute robots.
- M. Missura and M. Bennewitz, "Predictive foot placement for push recovery with step timing adjustment," IROS 2017 / related works.
- M. Khadiv et al., "Smooth trajectory generation and push-recovery based on Divergent Component of Motion," *IROS 2018*, DOI: 10.1109/IROS.2018.8594424.
- **M. Shafiee-Ashtiani, A. Yousefi-Koma, M. Shariat-Panahi, M. Khadiv**, "Push Recovery of a Humanoid Robot Based on Model Predictive Control and Capture Point," arXiv:1612.08034 (2016), https://arxiv.org/abs/1612.08034 — **verified**: a single MPC regulates the CP by modulating ZMP (ankle strategy, CP inside support) and CMP (hip strategy, CP outside support); demonstrated on an abstract model of the **SURENA III** humanoid under severe pushes, even with the support polygon shrunk to a point or line. (Note: Khadiv is a co-author, not the lead author.)

**Core idea.** When a push makes the required capture step too long or infeasible, *shortening the step time* is as important as relocating the foot. The nonlinear timing optimization is approximated as a linear/QP problem: given the current CP error, solve for the foot location $p$ and next step duration $T$ such that the capture condition holds at the end of the (shortened) step. This yields larger recoverable pushes than step-location-only strategies.

**Strengths.** Explicit formalism for recovery region; cheap linearized form suitable for online use; combinable with ankle/hip strategies.

**Weaknesses.** Approximation error vs full nonlinear solution; assumes SLIP/LIPM validity during the perturbed step; requires reliable CoM/CP estimation at high rate.

---

## 3. Virtual pendulum / SLIP and Raibert templates

**Papers:**
- M. H. Raibert, *Legged Locomotion and Robotics*, MIT Press; foundational papers: "Dynamically Stable Legged Locomotion," CMU-RI-TR-83-13 (1983), https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/raibert_marc_h_1983_1/raibert_marc_h_1983_1.pdf ; "Legged robots," *ASME J. Dynamic Systems, Measurement and Control* 108(3), 1986, http://www.sci.brooklyn.cuny.edu/~goetz/agents/courses/corc3303/papers/a4-raibert-1986.pdf
- R. J. Full & D. E. Koditschek, "Templates and anchors: neuromechanical hypotheses of legged locomotion on land," *J. Exp. Biology* 202, 1999 — the "template" concept.
- G. Schaal / M. A. Schwind / others on SLIP control; e.g. "Leg-adjustment strategies for stable running in three dimensions," *Bioinspiration & Biomimetics* 7(3):036002, 2012, https://doi.org/10.1088/1748-3182/7/3/036002.

**Core idea.** A running/walking legged robot is abstracted as a point mass on a massless, springy leg — the **spring-loaded inverted pendulum (SLIP)**. During stance the CoM vaults over the foot like an inverted pendulum with a spring; during flight it is ballistic. Raibert's celebrated **three-part control** decomposes the problem:
1. **Foot placement** controls forward velocity: $x_f = \frac{\dot{x} T_s}{2} + k_{\dot x}(\dot x - \dot x^{\text{des}})$ (land the foot ahead of where the CoM would land passively).
2. **Leg length / thrust** controls vertical height and hopping.
3. **Body attitude** control via hip torque during stance.

The "virtual leg" abstraction (Raibert) treats a multi-leg as one effective springy leg in the sagittal plane. For **walking** the vaulting inverted pendulum (compass/SLIP with double support) is the template; for **running** the ballistic SLIP.

**Strengths.** Biologically grounded; extremely simple, robust control laws; explains empirical gait patterns (foot placement, leg stiffness); works without heavy computation or exact models; natural for compliant actuators.

**Weaknesses.** Template-to-anchor mapping (full robot → SLIP) is not exact; spring dynamics and impact losses require tuning; SLIP is harder to analyze formally than LIPM; mainly validated on hoppers/runners and simplified robots, less so on full-size torque-controlled humanoids.

**Computational cost.** Minimal (a handful of gains + an ODE or even deadbeat foot-placement law).

**Track record.** Raibert's monopod/biped/quadruped hoppers (CMU, 1980s), many running robots and quadrupeds; used as a conceptual foundation for virtual-constraint/HZD work (the "SLIP-like" virtual constraints of Grizzle's group).

---

## 4. Hybrid zero dynamics (HZD) + virtual constraints + CLFs

**Papers (selection):**
- J. W. Grizzle et al., "Hybrid zero dynamics of planar biped walkers," *IEEE Trans. Automatic Control* 48(1):42–56, 2003.
- A. D. Ames, K. Galloway, J. W. Grizzle, "Control Lyapunov Functions and Hybrid Zero Dynamics," *Proc. IEEE CDC 2012*, https://grizzle.robotics.umich.edu/files/CDC12_1692_FI.pdf.
- A. D. Ames, K. Galloway, K. Sreenath, J. W. Grizzle, "Rapidly exponentially stabilizing control Lyapunov functions and hybrid zero dynamics," *IEEE Trans. Automatic Control* 59(4):876–891, 2014, DOI: 10.1109/TAC.2014.2309331.
- **J. Reher and A. D. Ames**, "Control Lyapunov Functions for Compliant Hybrid Zero Dynamic Walking," arXiv:2107.04241, 2021, https://arxiv.org/abs/2107.04241 — **verified (correction: authors are Jenna Reher and Aaron D. Ames, NOT Westerberg)**. End-to-end realization on **Cassie** with a compliant (series-elastic) model: a gait library of compliant walking motions is compiled and parameterized, CLFs are synthesized with formal stability proofs, and coupled with inverse dynamics into optimization-based QP torque controllers; 3D walking at different speeds and terrains is demonstrated.
- "Virtual Constraints and Hybrid Zero Dynamics for Realizing Underactuated Bipedal Locomotion," arXiv:1706.01127 (https://arxiv.org/abs/1706.01127) — metadata **unverified** (API rate-limited); treat citation details with caution. Survey: Ames et al., *Automatica* survey PDF http://ames.caltech.edu/AUTOMATICA_SurveyPaper_final.pdf.

**Name note (requested "Westerberg"):** In the HZD/CLF literature the relevant Michigan/Caltech names are **Ames, Grizzle, Galloway, Sreenath, Reher, Westervelt** (e.g., Westervelt et al., "Design and control of dynamic bipedal robot locomotion," CRC Press 2018). I could not verify a "Westerberg" HZD paper; the compliant-HZD-CLF paper requested is by **Reher & Ames (2021)**, corrected above.
- Galloway et al., "Torque Saturation in Bipedal Robotic Walking Through Control Lyapunov Function-Based Quadratic Programs," 2015, https://grizzle.robotics.umich.edu/files/CLF_ConvexOptim.pdf.

**Core idea.** Model walking as a **hybrid system**: continuous single-support dynamics $\dot{x}=f(x)+g(x)u$ plus discrete impact map $x^+=h(x^-)$ at foot strike. Define **virtual constraints** — outputs $y = h(x) - h_d(\alpha(q),\tau(s))$ that encode desired joint trajectories as functions of a monotonically advancing phase variable $s$ (e.g., hip angle) and a small set of parameters $\alpha$ (step length, speed, etc.). Drive $y\to 0$ by feedback (input–output linearization, or better, a **control Lyapunov function**). When $y\equiv0$, motion is confined to a low-dimensional **zero dynamics manifold**; if its reduced dynamics (the HZD) has a stable periodic orbit, the full-order walking gait is provably (locally exponentially) stable — a formal stability guarantee rare in robotics.

**CLF-QP.** Rather than exact feedback linearization (which needs precise model and can demand huge torques), pick a rapidly exponentially stabilizing CLF $V = y^\top P y$ and solve at each control tick the pointwise QP:

$$\min_{u} \; \tfrac12 u^\top Wu \quad \text{s.t.} \quad \dot V(y,u) \le -\epsilon V(y)$$

subject also to torque limits and friction cones (the "CLF-QP" with slack variables). This is a small dense QP solvable in ~0.1–1 ms with dedicated solvers.

**Strengths.** The only family with rigorous, constructive **formal stability guarantees** for underactuated 3D walking; handles underactuation and compliant actuators; well-developed toolchain (FROST, MABEL/Cassie/Agility). CLF-QP integrates constraints gracefully.

**Weaknesses.** Requires an accurate full-order model with contact/impact model, careful virtual-constraint design, and an offline optimization to shape gaits; impact models are sensitive; QP feasible only near the designed gait; extending to diverse terrain/footholds is harder than with MPC/RL; steep learning curve.

**Computational cost.** Medium–high online (small dense QP at ~200–500 Hz) plus offline trajectory optimization; full-state estimation required.

**Track record.** MABEL (Michigan, fast running), **Cassie** (Agility Robotics, robust indoor/outdoor walking, 2017–2021), ATRIAS, and influence on Digit's early controllers; DLR/two-legged testbeds.

---

## 5. MPC-based walking

MPC solves, at each tick, a finite-horizon optimal control problem over the reduced-order (LIPM/centroidal) dynamics with inequalities (ZMP in support polygon, foot reachability, friction), applies the first input, and re-solves (receding horizon) — giving inherent reactivity.

### 5.1 Linear MPC (MIT, Wieber, Caron, Scianca)

- **MIT MPC:** **Y. Ding, C. Khazoom, M. Chignoli, S. Kim**, "Orientation-Aware Model Predictive Control with Footstep Adaptation for Dynamic Humanoid Walking," arXiv:2205.15443 (2022), https://arxiv.org/abs/2205.15443 — **verified (correction: authors are Ding, Khazoom, Chignoli, Kim; NOT Ding/Stanger-Jones/Kim, and NOT a point-mass/flywheel model)**. Uses the **augmented single rigid body model (aSRBM)** so the MPC reasons about torso orientation dynamics and stepping in one optimization; footstep locations are decision variables, both MPC and the **task-space controller (TSC)** are QPs. Verified abstract reports increased robustness to external torque disturbances vs. point-mass models (esp. large torso angular excursions) and MIT Humanoid traversal of a wave field. Related: "Implementation of Convex Optimization Control on the MIT Humanoid," ICRA 2024 workshop (https://icra-2024-humanoid.github.io/docs/assets/papers/cvx-opt-mit-humanoid.pdf).
- **Scianca et al. "IS-MPC":** N. Scianca, D. De Simone, L. Lanari, G. Oriolo, "MPC for Humanoid Gait Generation: Stability and Feasibility," *IEEE T-RO 2020*, **DOI 10.1109/TRO.2019.2958483**, arXiv:1901.08505 (https://arxiv.org/abs/1901.08505 — **verified**). Uses a dynamically extended LIP with **ZMP velocities** as inputs, adds an **explicit stability constraint** linking future ZMP velocities to the current state so the CoM trajectory stays bounded w.r.t. the ZMP — no terminal-cost tuning; generates footsteps with timing online. The verified abstract discusses the MPC "tail" and terminal constraints, proves recursive feasibility conditions and stability, and reports simulation **and experimental results on NAO and HRP-4**. (Earlier lineage: "A MPC walking framework..." ICRA 2017.)
- **Caron et al. qpmpc / lipm_walking_controller:** Stéphane Caron's open-source LIPM MPC walking stack (https://github.com/stephane-caron/lipm_walking_controller , https://github.com/stephane-caron/qpmpc ), used on HRP-2 and others.

**Core equations (discrete LIPM):** state $x_k=[p_G,\dot p_G]^\top$, $x_{k+1}=A x_k + B p_{Z,k}$ with $A=\begin{bmatrix}1&T\\ \omega^2 T&1\end{bmatrix}$, $B=\begin{bmatrix}-\omega^2 T^2/2\\ -\omega^2 T\end{bmatrix}$ (for constant ZMP over the step). MPC minimizes $\sum \|p_G-\bar p_G\|_Q^2+\|p_Z-\bar p_Z\|_R^2$ s.t. $p_Z\in$ support polygon, footstep kinematic constraints, and (for IS-MPC) capture/stability inequalities.

**Strengths.** Reactivity to disturbances; automatic footstep adjustment; feasibility inequalities; well-understood convex QP solvers (OSQP, qpOASES, CVXGEN). Linear formulation is fast and reliable.

**Weaknesses.** Reduced-order model ignores limb inertia/angular momentum (partly addressed by adding a "flywheel" centroidal momentum state); coordinate and mode-sequence handling (single vs double support, variable timing) complicates the QP; fixed contact schedule unless mixed-integer/extended.

**Cost.** Small dense QP (tens to a few hundred variables), 50–200 Hz — feasible on an embedded Linux SBC; borderline but possible on a fast MCU with a hand-written active-set solver.

### 5.2 Centroidal / nonlinear MPC (Boston Dynamics-style, MIT NMPC)

- **Caron & Kheddar (2017)** — floating-base inverted pendulum NMPC for rough terrain, arXiv:1703.00688.
- **MIT NMPC:** E. Stanger-Jones, S. Kim et al., "Tailoring Solution Accuracy for Fast Whole-Body Model Predictive Control of Legged Robots," *IEEE RA-L 2022/2023*, https://biomimetics.mit.edu/publications/tailoring-solution-accuracy — ADMM-based SQP solver runs a **32-step horizon, 2004 variables, 3768 constraints** at high rate on the MIT Humanoid, allowing complex crossed-leg/arm motions for balance.
- **Centroidal MPC:** J. Carpentier & N. Mansard, "Multicontact Locomotion of Legged Robots via Centroidal MPC," *IEEE T-RO 2018*; uses the centroidal dynamics + (approximate) centroidal inertia, solved via DDP.

**Boston Dynamics Atlas** is widely understood to use a hierarchical **MPC (centroidal/reduced model) + whole-body controller (WBC)** stack; see Boston Dynamics blog "Large Behavior Models and Atlas Find New Footing" (https://bostondynamics.com/blog/large-behavior-models-atlas-find-new-footing/) and RAI Institute collaboration videos. Exact implementation is proprietary.

**Cost.** High: requires Newton/Hessian computations, linear algebra libraries, GPUs or multicore CPUs; not MCU-feasible.

### 5.3 Footstep MPC
Gradev/Graf et al. (e.g., "Robust Walking based on MPC with Online Adapted Step Duration and Foot Placement," IROS 2018) and the sequential MPC of Narkhede et al. (2022) explicitly optimize footstep positions/durations within the MPC; mixed-integer formulations (Deits & Tedrake 2014) handle discrete footholds.

---

## 6. Reinforcement learning for walking

### 6.1 End-to-end and sim-to-real foundations
- **X. Peng, M. Andrychowicz, W. Zaremba, P. Abbeel, "Sim-to-Real Transfer of Robotic Control with Dynamics Randomization,"** *ICRA 2018*, arXiv:1710.06537 (https://arxiv.org/abs/1710.06537) — randomizes 95 physical parameters (masses, inertias, friction, latencies, gains) during training so policies transfer zero-shot from simulation to a real **3D biped** (and a manipulator). The canonical sim-to-real recipe.
- **J. Hwangbo et al., "Learning agile and dynamic motor skills for legged robots,"** *Science Robotics* 4(26), 2019 — actuator-network + domain randomization, deployed on ANYmal.
- **N. Rudin, D. Hoeller, P. Reist, M. Hutter, "Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning,"** *CoRL 2022*, arXiv:2109.11978 (https://arxiv.org/abs/2109.11978) — Isaac Gym + adapted PPO, ~4096 parallel robots, flat terrain in <4 min, rough terrain in ~20 min on one GPU; also trained Cassie in simulation. Code: https://leggedrobotics.github.io/legged_gym/ . This collapsed training time by orders of magnitude and made RL the default path for new legged platforms.
- **J. Lee et al., "Learning quadrupedal locomotion over challenging terrain,"** *Science Robotics* 5(47), 2020; **T. Miki et al., "Learning robust perceptive locomotion for quadrupedal robots in the wild,"** *Science Robotics* 7(62), 2022 — teacher–student distillation with privileged terrain info; legged robots traverse real trails.

### 6.2 RL on humanoids (2023–2026 state of the art)
- **I. Radosavovic, T. Xiao, B. Zhang, T. Darrell, J. Malik, K. Sreenath, "Real-world humanoid locomotion with reinforcement learning,"** *Science Robotics* 9(89):eadi9579, April 2024 (https://science.org/doi/10.1126/scirobotics.adi9579 ; project page https://learning-humanoid-locomotion.github.io ; PDF https://hybrid-robotics.berkeley.edu/publications/ScienceRobotics2024_Learning_Humanoid_Locomotion.pdf). A **causal transformer** policy trained with RL on a **Digit** humanoid; directly maps proprioceptive history to joint commands; demonstrates robust outdoor walking, getting up from falls, and sim-to-real transfer at scale. This is a landmark: fully learning-based locomotion on a full-size humanoid in the real world.
- **Z. Li, X. B. Peng, P. Abbeel, S. Levine, G. Berseth, K. Sreenath, "Reinforcement Learning for Versatile, Dynamic, and Robust Bipedal Locomotion Control,"** arXiv:2401.16889 (2024), **accepted IJRR 2024** — **verified**. A unified RL controller for walking, running, jumping, standing on **Cassie**; uses a **dual-history (long- and short-term I/O history) architecture** and task randomization; demos include a 400 m dash, standing long jumps and high jumps.
- **G. A. Castillo, B. Weng, W. Zhang, A. Hereid, "Robust Feedback Motion Policy Design Using Reinforcement Learning on a 3D Digit Bipedal Robot,"** arXiv:2103.15309 (2021) — **verified (correction: authors are Castillo, Weng, Zhang, Hereid)**. A hierarchical cascade controller combining learning with feedback regulation; reduced state/action spaces; **first learning-based policy transferred to Digit hardware without dynamics randomization or curriculum learning** (per the verified abstract). Agility now uses NVIDIA Isaac Lab + a "whole-body control foundation model" for Digit (https://agilityrobotics.com/content/training-a-whole-body-control-foundation-model ; NVIDIA case study: https://nvidia.com/en-us/case-studies/agility-robotics-digit-humanoid-robot). Digit has >65,000 h of real production deployment (company claims, 2024–25).
- **Figure AI** — "Natural Humanoid Walk Using Reinforcement Learning" (https://figure.ai/news/reinforcement-learning-walking) and **Helix 02** (2025/26, https://figure.ai/news/helix-02): System 0 is a learned whole-body controller trained on >1000 h of human motion + sim-to-real RL, with human-like heel-strike gait. Reinforcement learning is explicitly credited for the natural walking.
- **Unitree H1/G1** — open-source RL stack: **unitreerobotics/unitree_rl_gym** (https://github.com/unitreerobotics/unitree_rl_gym), PPO-based, supports Go2/H1/H1_2/G1; extensive community demos (parkour, gets-up). 2025 research (all trained in sim, deployed zero-shot on **Unitree G1**): **"RuN: Residual Policy for Natural Humanoid Locomotion," arXiv:2509.20696 (Q. Li, C. Zhu, Y. Wu, X. Yuan, Z. Zhang, J. Yang, Y. Liu, 2025 — verified)** — decouples a Conditional Motion Generator prior from a lightweight RL residual, achieving natural gaits and smooth walk–run transitions over 0–2.5 m/s; **"Learning Getting-Up Policies for Real-World Humanoid Robots," arXiv:2502.12152 (X. He, R. Dong, Z. Chen, S. Gupta, RSS 2025 — verified)** — two-phase curriculum for getting up on flat/deformable/slippery surfaces and slopes; "Learning Humanoid Standing-up Control across..." RSS 2025 (https://www.roboticsproceedings.org/rss21/p064.pdf — unverified).
- **Tesla Optimus** — no peer-reviewed controller paper; Tesla's public descriptions emphasize end-to-end neural networks learned from human video, with RL for locomotion and manipulation ("How Tesla Optimus Learns," https://theroboticlife.com/tesla-optimus-gen-3-learning; see also LiPS, arXiv:2503.08349, https://arxiv.org/html/2503.08349v1, which uses Optimus-like designs). A 2025 paper "RL from Physical Feedback: Aligning Large Motion Models with Humanoid Control" (arXiv:2506.12769) cites Optimus/G1 as motivating platforms.
- **2025–26 frontier:** diffusion-based policies (DiffuseLoco, CoRL 2024, https://hybrid-robotics.berkeley.edu/publications/CoRL2024_DiffuseLoco.pdf), residual + MPC hybrids ("Residual MPC: Blending RL with GPU-Parallelized MPC," arXiv:2510.12717), and whole-body VLA traversal (Figure Helix 02, Berkeley Science Robotics 2026 preprint https://hybrid-robotics.berkeley.edu/publications/ScienceRobotics2026_HumanoidLocomotion.pdf).

**Core idea.** Learn a policy $\pi_\theta(o_t)\to a_t$ (often joint position targets + PD, or torques) from proprioception (and optionally height maps / vision), with rewards for velocity tracking, regularities (torques, accelerations, foot clearance, symmetry), and domain randomization for sim-to-real. PPO dominates (on-policy, robust to randomization); off-policy (SAC) and transformers/diffusion are emerging.

**Strengths.** No explicit model or stability proof needed; emerges robust, natural gaits; handles rough terrain, pushes, and sim-to-real with randomization; dramatically shorter development time with modern GPU pipelines; handles full 20–30 DoF including arms.

**Weaknesses.** Needs massive simulation compute for *training* (GPU cluster/workstation, hours–days) and a high-fidelity simulator plus careful reward engineering; limited formal guarantees; sim-to-real gap (actuator dynamics, delays, contacts); poor out-of-distribution behavior; policies are "black boxes", hard to certify; **but inference is cheap** — a small MLP/transformer at 50–100 Hz.

**Cost.** Training: high (GPU). **Deployment: low** — a 2–4 layer MLP (a few 10k MACs per step) easily runs on an MCU; transformers are heavier. The training burden is an offline, one-time cost.

---

## 7. Reactive walking / push recovery while walking

The "三个层次" of balance (Pratt):
1. **Ankle strategy** — shift ZMP within support foot (small disturbances).
2. **Hip strategy** — rotate upper body / swing limbs to shift CoM (medium).
3. **Stepping strategy** — take a step (capture step) to a new foot location (large).

**Key works:**
- Pratt et al. 2006 (capture point, Sec. 2.1) — formalizes "capture step."
- **M. Khadiv, A. Herzog, et al.** — step timing + location optimization (Sec. 2.3), and **M. Khadiv, S. A. A. Moosavian, A. Herzog, L. Righetti, "Push recovery of a humanoid robot based on model predictive control and capture point,"** arXiv:1612.08034 (https://arxiv.org/html/1612.08034v1) — MPC cost that blends ankle (ZMP modulation) and hip (CMP modulation) strategies with CP constraints.
- **M. Missura & M. Bennewitz**, "Predictive foot placement and step timing for bipedal walking," and related — predict the *opportunity* for a step: whether a swing foot can land in time and where; use CP and predicted disturbances.
- **Boaventura et al., "Walking control with push recovery" / "Capture point step adjustment"** — ICRA/IROS 2017–2019 (e.g., "Dynamic balance control for biped robots: an optimization approach to generate safe foot placements using capture point," IROS 2018).
- **J. Caron, A. Kheddar, "Walking pattern generators designed for physical collaboration,"** ICRA 2016 — WPG that accepts external contact forces.
- **"An MPC Walking Framework With External Contact Forces,"** arXiv:1712.09308 (https://arxiv.org/html/1712.09308v1) — plans external contact forces (e.g., pushing a cart/hand contact) in linear MPC.
- **J. Yi et al., "A Hybrid Walk Controller for Resource-Constrained Humanoid Robots,"** 2013 (PDF https://romela.org/wp-content/uploads/2015/05/2013_a_hybrid_walk_controller_for_resource-constrained_humanoid_robots.pdf) — switches between an analytic LIPM reactive controller and a preview/ZMP controller; directly relevant to small robots.
- **"Walking Engine Using ZMP Criterion and Feedback Control for Child-Sized Humanoid Robot,"** *IJHR* 2016 — a 21-DoF child-sized humanoid with ZMP gait generator + feedback, DOI 10.1142/S0219843616500213 (highly relevant to the target platform).

**Core concepts.** Estimate the CP error $\Delta\xi$ after a push. If $|\Delta\xi|$ below foot support radius → ankle strategy (move ZMP to foot edge). Else compute a capture step: choose new foot position $p$ and step duration $T$ satisfying $p \approx \xi(T) = p_Z + (\xi_0-p_Z)e^{\omega T}$ (plus swing-time and margin). Khadiv's key result: optimizing $T$ (shortening it) expands the recoverable push region substantially. Combine with virtual repellant point limits for feasibility.

**Strengths.** Mathematically grounded; cheap; integrates with all LIPM/DCM/MPC frameworks; essential for real robustness.

**Weaknesses.** Relies on CP estimate (noisy CoM velocity, impacts); ignores angular momentum from the push (may capture with body spin); limited by physical step length, swing time, and friction.

---

## 8. Gait phase oscillators / Central Pattern Generators (CPGs)

**Papers:**
- K. Matsuoka, "Mechanisms of pattern generation based on neural modeling," PhD thesis / IEEE 1990; "Sustained oscillations generated by mutually inhibiting neurons with adaptation," *Biological Cybernetics* 52, 1985.
- **Y. Fukuoka, H. Kimura, J.-A. Wang, "Design of a three-dimensional quadruped locomotion controller based on a central pattern generator and sensory feedback,"** *Autonomous Robots* 14, 2003 — actuators/neurons with sensory feedback; representative CPG+reflex work.
- **G. Taga, Y. Yamaguchi, H. Shimizu, "Self-organized locomotion of an infant robot by the dynamic interaction of neural oscillators and the environment,"** 1991/1994; and Taga's "A model of the neuro-musculo-skeletal control system of human bipedal locomotion" — classic coupled-oscillator human walking.
- **M. A. Lewis et al., "Experimental Studies of a Neural Oscillator for Biped Locomotion with QRIO,"** *ICRA 2005*, DOI: 10.1109/robot.2005.1570183 (https://doi.org/10.1109/robot.2005.1570183) — Matsuoka oscillators on Sony QRIO.
- **A. J. Ijspeert, "Central pattern generators for locomotion control in animals and robots: a review,"** *Neural Networks* 21(4), 2008 — canonical review. Hopf oscillators: e.g., "Biped Locomotion Control through a Biomimetic CPG-based Controller," *J. Intelligent & Robotic Systems* 2016 (https://link.springer.com/10.1007/s10846-016-0407-3) compares Matsuoka vs Hopf.
- **"A Hybrid CPG-ZMP control system for stable walking of a simulated flexible spine humanoid robot,"** 2010 (PubMed 20031370) — hybrid CPG + ZMP.

**Core idea.** Generate rhythmic joint patterns from coupled nonlinear oscillators instead of precomputed trajectories. **Matsuoka oscillator** (two mutually inhibiting neurons with adaptation):

$$\tau \dot u_i = -u_i - \beta v_i - \gamma [u_j]^+ + s_i + \text{feedback}, \qquad \tau' \dot v_i = -v_i + [u_i]^+$$

with output $y_i = [u_i]^+ - [u_j]^+$ and tonic input $s_i$ controlling frequency/amplitude. **Hopf oscillator** (normal form of Andronov–Hopf bifurcation):

$$\dot x = \alpha(\mu - r^2)x - \omega y,\qquad \dot y = \alpha(\mu-r^2)y + \omega x$$

with phase $\phi=\text{atan2}(y,x)$, amplitude $r$ — phase and amplitude separately controllable, making it popular for phase-based control. Coupling via phases locks left/right legs with a $\pi$ phase offset; sensory feedback (foot contact, CoP error) entrains the oscillator (frequency/phase locking). Outputs map to joint trajectories or to a phase variable $\phi\in[0,2\pi)$ driving virtual constraints.

**Strengths.** O(10) scalar ODEs; extremely cheap, inherently smooth and periodic; easy phase resets and entrainment; graceful degradation/sensory coupling; forgiving of model errors; ideal for tiny MCUs.

**Weaknesses.** Hard to guarantee ZMP feasibility/stability formally; no systematic way to specify exact foot placements or adapt timing to arbitrary terrain; parameter tuning is empirical; weaker real-robot track record on full-size humanoids (QRIO, small humanoids, some research platforms) vs ZMP/MPC/RL; often used as a *phase clock* rather than the whole controller.

**Cost.** Minimal (a few hundred flops per control tick). Attractive for MCU.

---

## 9. Trajectory optimization / direct collocation

**Papers:**
- **H. Dai, A. Ames et al., "Versatile Locomotion Planning and Control for Humanoid Robots" (TOWR+)**, arXiv/IJRR 2021 (https://pmc.ncbi.nlm.nih.gov/articles/PMC8414409) — trajectory optimization for walking robots plus whole-body control.
- **M. Posa et al. (DRACO team, CMU/USC/UT Austin)** — contact-implicit and hybrid trajectory optimization: "Contact-implicit trajectory optimization using variational inference," *IJRR 2020* (https://www.ri.cmu.edu/app/uploads/2020/06/0278364919849235.pdf); "ContactNets" (https://par.nsf.gov/biblio/10273128); **DRACO 3** (Apptronik/UT Austin), "Variable Inertia Model Predictive Control for Fast Bipedal Maneuvers," arXiv:2407.16811 (https://arxiv.org/html/2407.16811v2) and robot description arXiv:2210.00961 (https://arxiv.org/pdf/2210.00961) — proximal actuation, rolling contacts, dynamic walking/jumping.
- **"Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive Control,"** arXiv:2309.01813 (https://arxiv.org/html/2309.01813v3) — surprisingly simple inverse-dynamics formulation enabling real-time contact-implicit MPC.
- **A. Hereid et al., "Direct collocation framework for optimizing hybrid zero dynamics,"** ICRA 2016 / IEEE T-RO 2018 — optimizes virtual-constraint parameters offline for HZD gaits (FROST toolbox).
- **MIT Underactuated Robotics, Ch. 17 "Planning and Control through Contact"** (https://underactuated.mit.edu/contact.html) — excellent tutorial distinguishing fixed-mode-sequence vs contact-implicit optimization.

**Core idea.** Discretize the full (reduced or full-order) dynamics and contact constraints into a nonlinear program (NLP) solved by direct collocation / SNOPT / IPOPT / CasADi:

$$\min_{x_k,u_k,\lambda_k} \sum_k \ell(x_k,u_k) \quad \text{s.t. } x_{k+1}=f(x_k,u_k,\lambda_k),\; \lambda_k\in\mathcal{K}, \; g(x_k)\ge0$$

**Contact-implicit** variants optimize contact locations/timings and modes jointly (via complementarity or smooth approximations) instead of fixing them a priori — powerful but nonconvex and expensive.

**Strengths.** Can exploit full robot dynamics, complex contacts, uneven terrain, momentum, and underactuation; produces high-performance, physically consistent gaits; HZD parameter optimization gives provably stable gaits.

**Weaknesses.** Nonconvex, local minima, expensive; mode sequencing combinatorial; historically offline (minutes), though recent work reaches real-time MPC. Requires good initial guesses and differentiation tooling.

**Cost.** High (NLP with hundreds/thousands of variables; seconds–minutes offline for full ined; ~10–100 ms per MPC step for the latest contact-implicit approaches on workstation CPUs).

---

## 10. Practical deployed walking systems (what algorithms they use)

| Platform | Known / reported approach | Notes |
|---|---|---|
| **Honda P2 / ASIMO** (1996–2018) | **ZMP-based walking** with predictive control ("i-WALK," posture/ZMP feedback). CoM trajectory planned so the zero-moment point stays in the support polygon; locked arms/bent knees to satisfy LIPM assumptions. | Honda technical review Vol.13-1 (https://global.honda/en/technical-review/PDF/013-1/013-1e.pdf); "All-New ASIMO" (https://hondanews.eu/eu/en/media/pressreleases/35508/all-new-asimo). The archetypal ZMP success story. |
| **HRP-2 / HRP-4** (AIST) | **Kajita preview-control ZMP WPG** + LIPM tracking + whole-body inverse kinematics; later HRP-4 stair climbing with whole-body admittance control (arXiv:1809.07073). | Kajita's papers use HRP-2P/2/4 as the primary testbeds; open-source stack (https://github.com/stephane-caron/lipm_walking_controller). |
| **Boston Dynamics Atlas** (2013–2026) | Hierarchical **MPC + whole-body control** (centroidal/reduced-order planner → QP WBC for torque tracking); teleoperation and behaviors layered on top (BD blog: https://bostondynamics.com/blog/large-behavior-models-atlas-find-new-footing/). Recent 2024–26 work adds **whole-body RL** (e.g., wheeled "Atlas on a Roll" combining classical motor control + whole-body RL) and TRI collaboration on large behavior models. Exact details proprietary. | Most dynamic full-size humanoid demos (parkour, backflips with electric Atlas). |
| **Agility Digit** | Early: **HZD / virtual constraints** (Cassie lineage) with RL-augmented feedback (arXiv:2103.15309). Current: **RL-based whole-body control foundation model** trained in NVIDIA Isaac Lab, with production deployment (65k+ hours, company claim). | https://agilityrobotics.com/content/training-a-whole-body-control-foundation-model ; https://nvidia.com/en-us/case-studies/agility-robotics-digit-humanoid-robot . |
| **Figure 01/02/03** | **RL walking controller** (end-to-end neural net, heel-strike human-like gait) + **Helix** VLA for upper-body/task semantics; Helix 02 (2025–26) "System 0" learned whole-body controller trained on >1000 h human motion + sim-to-real RL. | https://figure.ai/news/reinforcement-learning-walking ; https://figure.ai/news/helix-02 . |
| **Tesla Optimus** | End-to-end neural networks learned from human video + RL for locomotion/manipulation; details not published in peer-reviewed form. | https://theroboticlife.com/tesla-optimus-gen-3-learning ; IEEE Spectrum expert commentary (https://spectrum.ieee.org/robotics-experts-tesla-bot-optimus/particle-16). |
| **Unitree H1 / G1** | **PPO-based RL** policies trained in Isaac Gym/Isaac Lab with domain randomization and deployed zero-shot; open-source `unitree_rl_gym`. Research extensions: get-up policies, residual policies, visual sim-to-real. | https://github.com/unitreerobotics/unitree_rl_gym ; arXiv:2502.12152, arXiv:2509.20696. |
| **MIT Humanoid** | **Convex/NMPC (LMPC + ADMM-SQP whole-body NMPC) + WBC**, with footstep location & timing adaptation; CBF-based self-collision avoidance; recent Residual-MPC blends RL with GPU-parallel MPC. | arXiv:2205.15443; https://biomimetics.mit.edu/publications/tailoring-solution-accuracy ; https://icra-2024-humanoid.github.io/posts/2024/05/paper-3 . |

**Industry trend (2024–26):** Model-based MPC+WBC remains the backbone for the most dynamic, controllable full-size platforms (Atlas, MIT Humanoid), while **RL-based whole-body policies dominate new commercial humanoids** (Figure, Digit, Unitree, Optimus) because they scale with data and generalize to messy environments; the frontier is **hybrid** (RL priors/residuals + MPC, VLA + learned whole-body control).

---

## 11. Comparative table

| Family | Core idea | Stability guarantee | Online cost | Offline/training cost | Terrain adapt. | Push recovery | Real-robot record |
|---|---|---|---|---|---|---|---|
| **ZMP preview (Kajita 2003)** | LQ preview servo tracks planned ZMP | Feasibility (ZMP in support) | **Very low** (fixed gains) | Low (precompute gains) | Limited (planned) | Needs outer loop | HRP-2/4, ASIMO lineage |
| **LIPM MPC (Wieber/Scianca/Caron/Ding)** | Receding-horizon QP over LIPM | Intrinsic stability (IS-MPC) or terminal cost | **Low–medium** (small QP) | Low | Good (footsteps online) | Good | HRP-2, MIT Humanoid |
| **DCM/capture point (Pratt/Englsberger/Khadiv)** | Steer unstable DCM; step to capture point | Conditional (capture condition) | **Very low** (closed form) | Low | Good (with adaptation) | **Excellent** | DLR TORO, many bipeds |
| **SLIP / Raibert template** | Springy virtual leg + foot-placement law | Empirical / limited | **Very low** | Low | Moderate | Moderate (reactive) | Hoppers, runners, quadrupeds |
| **HZD + CLF-QP (Grizzle/Ames/Reher)** | Virtual constraints + CLF-QP on hybrid model | **Formal (Lyapunov, hybrid)** | Medium (small QP + state est.) | High (gait optimization) | Limited per gait | Moderate | MABEL, Cassie, ATRIAS |
| **Centroidal/NMPC (MIT/Carpentier, Atlas-style)** | Full centroidal/full-body NMPC + WBC | Feasibility-based | **High** (ADMM/SQP, GPU/CPU) | Medium | Excellent | Excellent | Atlas, MIT Humanoid |
| **RL (Peng/Rudin/Radosavovic/Unitree/Figure)** | Learned policy + domain randomization | None formal | **Low at inference** (MLP) | **Very high** (GPU training) | Excellent | Excellent (learned) | Digit, Figure, Unitree G1, ANYmal |
| **CPG / oscillators (Matsuoka/Hopf)** | Coupled nonlinear oscillators + feedback | Entrainment only | **Very low** | Low (tuning) | Poor–moderate | Moderate (reflexes) | QRIO, small humanoids |
| **Trajectory optimization / collocation (DRACO/FROST/TOWR+)** | NLP over full dynamics & contacts | Local optimality | High (offline), emerging real-time | Very high | Excellent (planned) | Re-planning | DRACO 3, research robots |

---

## 12. Recommendation for a small 21-DoF humanoid on an RP2350-class MCU (no heavy FP solver)

**Constraints.** RP2350 (dual Cortex-M33 @150 MHz, ~520 KB SRAM, **no hardware FPU** — FP emulated in software, expensive), likely needs a control loop of 100–200 Hz, limited state estimation, small foot soles, modest actuator bandwidth. QPs with dense linear algebra, NMPC, and neural-network inference with large matrices are unrealistic; even modest floating-point must be used sparingly (fixed-point where possible).

**Recommended architecture — a tiered, mostly closed-form stack:**

1. **Gait clock / phase generator (CPG-lite).** A single **Hopf oscillator** or even a simple phase accumulator $\phi\leftarrow\phi+2\pi f T$ with left/right offset $\pi$ produces the gait phase. Cost: a few dozen flops per tick. This replaces a heavyweight pattern generator and gives natural phase resets for stepping. (Sec. 8; Ijspeert 2008 review.)

2. **Footstep planner with analytic LIPM + capture-point logic.** Keep a small **state machine** (double support / single support left / single support right). Compute the capture point $\xi = x + \dot x/\omega$ each tick (two multiplies and an add, $\omega$ precomputed). Use **Kajita-style analytic LIPM** or DCM exponential interpolation to generate the CoM reference: with fixed ZMP $p$ during a step, $x(t) = p + (x_0-p)\cosh(\omega t) + \frac{\dot x_0}{\omega}\sinh(\omega t)$; precompute $\cosh/\sinh$ via a small lookup table or a few Taylor terms. Choose foot placement from Raibert/capture law: $x_f = \xi + k_v(\dot x - \dot x_{\text{des}}) + \tfrac{1}{2}\dot x T_s$ (Secs. 2–3). This is the hybrid reactive/preview controller of Yi et al. (2013) applied to a small robot.

3. **ZMP feasibility check, not a QP.** Compute the desired ZMP $p_Z = x - \ddot x/\omega^2$ and clamp it into the support polygon (a few `min`/`max` operations). If it exits the foot, trigger a capture step (shorten step timing à la Khadiv). Optional: a 2-tap preview of the next ZMP waypoint for smoother CoM (Kajita's preview idea reduced to a short FIR sum with precomputed gains — no online Riccati).

4. **Inverse kinematics + joint control.** Map CoM/pelvis/foot trajectories to 21 joint targets with **analytic IK** for simple leg chains (or a tiny dampedleast-squares with 3×3 matrices in fixed point), then PD control each joint. Track swing foot with a smooth Bézier/sine arc in phase $\phi$.

5. **Reactive layer (push recovery).** Compare $\xi$ with the support foot bounds each tick: ankle strategy via CoP shift (ZMP clamp), else step adjustment (move next foot target toward $\xi$, shorten $T_s$). Cost: trivial.

6. **Optional off-board training / on-board inference (if allowed).** If an RL policy is desired for robustness (uneven ground), **train off-board** with massive parallelism (Rudin et al. 2022 / `unitree_rl_gym` recipes) and deploy a **tiny MLP** (2×32 or 3×32 ReLU, ~1–3k MACs/step) that outputs *residual joint targets or gait parameters* (step length, phase speed, foot offset) on top of the classical controller. Quantize to int8 and run at 50 Hz — feasible on M33 with SIMD/table tricks. Keep the LIPM/IK fallback as a safety net. Avoid transformers and large networks.

**What to avoid:** dense QP/MPC solvers, full centroidal NMPC, contact-implicit TO, large neural nets, and full multibody inverse dynamics on the MCU. Those belong on a companion SBC if the robot has one (a Raspberry Pi/CM4 running MPC at 100–200 Hz sending CoM/foot targets to the RP2350, which handles IK, PD, and the CPG phase clock).

**Why this works.** The combination of (i) closed-form LIPM/DCM motion, (ii) capture-point foot placement, (iii) a phase oscillator clock, and (iv) fixed/precomputed gains is exactly the family with the lowest computational cost and a decades-long real-robot record on small and full-size humanoids (Kajita/AIST; Yi et al. 2013 for resource-constrained robots; the 21-DoF walking engine of IJHR 2016). It provides predictable, debuggable behavior with formal-ish feasibility semantics, while RL can be layered on later purely as a performance enhancer trained off-board.

---

## 13. Key references (URLs)

- Kajita et al., "Biped Walking Pattern Generation by using Preview Control of ZMP," ICRA 2003 — https://doi.org/10.1109/ROBOT.2003.1241826 ; PDF https://mzucker.github.io/swarthmore/e91_s2013/readings/kajita2003preview.pdf
- Kajita et al., "The 3D Linear Inverted Pendulum Mode," IROS 2001.
- Pratt, Carff, Drakunov, Goswami, "Capture Point," Humanoids 2006 — http://ambarish.com/paper/Pratt_Goswami_Humanoids2006.pdf
- Englsberger, Ott, Albu-Schäffer, "3D Bipedal Walking Based on DCM," IEEE T-RO 31(2), 2015 — DOI 10.1109/TRO.2015.2405592 ; IROS 2013 PDF https://elib.dlr.de/87255/1/IROS_2013_FinalSubmission_JEnglsberger.pdf
- Khadiv et al., "Walking Control Based on Step Timing Adaptation," IEEE T-RO 2020 — https://arxiv.org/abs/1704.01271 (verified)
- Khadiv et al., "Push recovery based on MPC and capture point" — https://arxiv.org/html/1612.08034v1
- Raibert, "Legged robots," 1986 — http://www.sci.brooklyn.cuny.edu/~goetz/agents/courses/corc3303/papers/a4-raibert-1986.pdf
- Full & Koditschek, "Templates and anchors," J. Exp. Biol. 202, 1999.
- Ames, Galloway, Grizzle, "Control Lyapunov Functions and Hybrid Zero Dynamics," CDC 2012 — https://grizzle.robotics.umich.edu/files/CDC12_1692_FI.pdf
- **Reher & Ames**, "Control Lyapunov Functions for Compliant Hybrid Zero Dynamic Walking," 2021 — https://arxiv.org/abs/2107.04241 (verified)
- Wieber, "Trajectory Free Linear MPC for Stable Walking," Humanoids 2006 — DOI 10.1109/ICHR.2006.321375
- Scianca et al., "MPC for Humanoid Gait Generation: Stability and Feasibility," T-RO 2020, DOI 10.1109/TRO.2019.2958483 — https://arxiv.org/abs/1901.08505 (verified; NAO + HRP-4)
- Caron & Kheddar, "Multi-contact WPG based on Model Preview Control of 3D CoM Accelerations," Humanoids 2016 — https://arxiv.org/abs/1607.08729
- Caron's open-source LIPM walking controller — https://github.com/stephane-caron/lipm_walking_controller
- Ding, Khazoom, Chignoli, Kim, "Orientation-Aware MPC with Footstep Adaptation," arXiv:2205.15443 (2022) — https://arxiv.org/abs/2205.15443 (verified; aSRBM, TSC)
- Stanger-Jones & Kim, "Tailoring Solution Accuracy for Fast Whole-Body MPC," RA-L — https://biomimetics.mit.edu/publications/tailoring-solution-accuracy
- Deits & Tedrake, "Footstep planning on uneven terrain with mixed-integer convex optimization," Humanoids 2014 — https://groups.csail.mit.edu/robotics-center/public_papers/Deits14b.pdf
- Peng et al., "Sim-to-Real Transfer with Dynamics Randomization," ICRA 2018 — https://arxiv.org/abs/1710.06537
- Rudin et al., "Learning to Walk in Minutes," CoRL 2022 — https://arxiv.org/abs/2109.11978 ; code https://leggedrobotics.github.io/legged_gym/
- Radosavovic et al., "Real-world humanoid locomotion with reinforcement learning," Science Robotics 9(89), 2024 — https://science.org/doi/10.1126/scirobotics.adi9579 ; PDF https://hybrid-robotics.berkeley.edu/publications/ScienceRobotics2024_Learning_Humanoid_Locomotion.pdf
- Castillo, Weng, Zhang, Hereid, "Robust Feedback Motion Policy Design Using RL on a 3D Digit Bipedal Robot," 2021 — https://arxiv.org/abs/2103.15309 (verified)
- Unitree RL Gym — https://github.com/unitreerobotics/unitree_rl_gym
- Ijspeert, "CPGs for locomotion control in animals and robots: a review," Neural Networks 21(4), 2008.
- Fukuoka et al., "3D quadruped locomotion controller based on CPG + sensory feedback," Autonomous Robots 14, 2003.
- Yi et al., "A Hybrid Walk Controller for Resource-Constrained Humanoid Robots," 2013 — https://romela.org/wp-content/uploads/2015/05/2013_a_hybrid_walk_controller_for_resource-constrained_humanoid_robots.pdf
- "Walking Engine Using ZMP Criterion and Feedback Control for Child-Sized Humanoid Robot," IJHR 2016 (21-DoF) — DOI 10.1142/S0219843616500213
- TOWR+ / Dai et al., "Versatile Locomotion Planning and Control for Humanoid Robots," 2021 — https://pmc.ncbi.nlm.nih.gov/articles/PMC8414409
- MIT Underactuated Robotics, Ch. 17 (contact) — https://underactuated.mit.edu/contact.html
- DRACO 3 / variable-inertia MPC — https://arxiv.org/html/2407.16811v2
- Boston Dynamics Atlas blog — https://bostondynamics.com/blog/large-behavior-models-atlas-find-new-footing/
- Figure RL walking — https://figure.ai/news/reinforcement-learning-walking ; Helix 02 — https://figure.ai/news/helix-02
- Agility/Digit WBC foundation model — https://agilityrobotics.com/content/training-a-whole-body-control-foundation-model
- Honda P2/ASIMO ZMP control — https://global.honda/en/technical-review/PDF/013-1/013-1e.pdf
- Caron, "How do biped robots walk?" (tutorial) — https://scaron.info/robotics/how-do-biped-robots-walk.html

---

## 14. Citation verification status (added after rate-limited search)

Web search, the arXiv API (`export.arxiv.org/api/query`), and Semantic Scholar's graph API were **rate-limited (HTTP 429)** in this environment. Verification was therefore performed by fetching individual arXiv abstract pages and publisher/portal pages (which remained accessible), plus two successful Semantic Scholar lookups. Status:

**Verified via arXiv abs pages / publisher portals / Semantic Scholar (titles, authors, years, venues corrected in the text above):**
- Englsberger, Ott, Albu-Schäffer, IEEE T-RO 31(2):355–368, 2015, DOI 10.1109/TRO.2015.2405592 (TUM portal).
- **Reher & Ames** (not Westerberg), "Control Lyapunov Functions for Compliant Hybrid Zero Dynamic Walking," arXiv:2107.04241, 2021.
- **Scianca, De Simone, Lanari, Oriolo**, arXiv:1901.08505, T-RO 2020, DOI 10.1109/TRO.2019.2958483 (NAO, HRP-4).
- **Ding, Khazoom, Chignoli, Kim**, arXiv:2205.15443, 2022 (aSRBM + TSC, not a point-mass flywheel).
- **Khadiv, Herzog, Moosavian, Righetti**, arXiv:1704.01271, "Walking Control Based on Step Timing Adaptation," T-RO 2020 (next-step-only optimization, viability guarantee, passive-ankle sim).
- **Shafiee-Ashtiani, Yousefi-Koma, Shariat-Panahi, Khadiv**, arXiv:1612.08034 (2016; SURENA III model; Khadiv is co-author).
- **Caron, Escande, Lanari, Mallein**, arXiv:1801.07022, "Capturability-based Pattern Generation for Walking with Variable Height," T-RO 2019, DOI 10.1109/TRO.2019.2923971.
- Caron & Kheddar, arXiv:1607.08729 (Humanoids 2016, HRP-4).
- **Rudin, Hoeller, Reist, Hutter**, arXiv:2109.11978 (CoRL 2021/2022) — Semantic Scholar.
- **Radosavovic, Xiao, Zhang, Darrell, Malik, Sreenath**, arXiv:2303.03381 (2023; Science Robotics 2024, eadi9579) — arXiv abs + web search.
- **Li, Peng, Abbeel, Levine, Berseth, Sreenath**, arXiv:2401.16889 (IJRR 2024; Cassie; dual-history).
- **Castillo, Weng, Zhang, Hereid**, arXiv:2103.15309 (2021).
- **He, Dong, Chen, Gupta**, arXiv:2502.12152 (RSS 2025, G1 getting-up).
- **Q. Li et al.**, arXiv:2509.20696 (RuN, 2025, G1, 0–2.5 m/s).
- Zhang et al., arXiv:2503.08349 (LiPS, 2025) — Semantic Scholar.
- Kajita et al. 2003 (DOI + MIT-hosted PDF), Pratt et al. 2006 (author-hosted PDF), Raibert 1983/1986 (CMU/ASME PDFs), Honda/HRP technical pages, Boston Dynamics/Figure/Agility/Unitree pages — web search results (HTTP 200) as noted inline.

**Unverified / lower-confidence (flagged, treat details cautiously):**
- arXiv:1706.01127 ("Virtual Constraints and HZD...") — could not fetch metadata.
- arXiv:2506.12769 (RL from Physical Feedback), arXiv:2105.08328 (Blind bipedal stair traversal), arXiv:2210.00961 / 2407.16811 (DRACO), arXiv:2309.01813 (contact-implicit ID-TO), arXiv:1902.06770, arXiv:1702.08742 — no independent metadata check; titles/years from search snippets only.
- The exact internal controller architectures of Boston Dynamics Atlas, Tesla Optimus, Figure, and Agility's current foundation model are **not publicly documented in peer-reviewed form**; statements in Sec. 10 are from company blogs/press and third-party reporting, and are explicitly labeled as reported/inferred.
- No "Westerberg" HZD paper could be verified; the requested compliant-HZD-CLF paper is Reher & Ames (2021) — see name note in Sec. 4.
