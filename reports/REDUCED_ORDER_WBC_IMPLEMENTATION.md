# Reduced-Order Control & Task-Space WBC: Implementation-Level Design
### Technical design appendix for a 45 cm / 1–2 kg humanoid on RP2350 @ 200 Hz, f32, no-alloc, no camera

**Audience.** Control firmware implementers (`rk-control`, `rk-cu-nodes`, `firmware/rk-rp2350`).
**Scope.** Seven topics: (1) LIPM/DCM balance with CoM-Jacobian state, (2) capturability and strategy switching, (3) task-space priority / WBC on MCU, (4) small f32 LIPM-MPC, (5) CBF safety filters, (6) SLIP/Raibert running, (7) I3C bus budget.
**Sources.** Verified via arXiv API, publisher pages, and the Caron `lipm_walking_controller` source on GitHub (2026-11-24). Items marked **[KB]** are from established knowledge base (textbook/standard results) and are given as implementation facts rather than freshly verified citations. Web search was rate-limited; arXiv API and raw-source fetches were used instead.

> All numbers below are derived for: mass $m=1.5$ kg (range 1–2), leg length $\ell\approx0.22$ m, nominal CoM height $h\approx0.25$ m (45 cm robot), $g=9.81$ m/s². Then $\omega=\sqrt{g/h}=\sqrt{39.24}=6.26$ rad/s (period $2\pi/\omega\approx1.0$ s). At $h=0.20$ m, $\omega=7.0$ rad/s. Control period $T_s=5$ ms. Use f32 throughout.

---

## 1. LIPM / DCM balance control, implementable detail

### 1.1 Core equations

LIPM: point mass at fixed height $h$, massless telescoping leg, ground contact point $p$ (ZMP/CoP).

$$\omega=\sqrt{\frac{g}{h}},\qquad \lambda=\omega^2=\frac{g}{h}$$

$$\ddot c = \omega^2(c-p) \quad\text{(horizontal, per axis)}$$

$$p = c - \frac{\ddot c}{\omega^2} \quad\text{(ZMP from CoM accel)}$$

Capture point / DCM (Pratt et al., Humanoids 2006; Englsberger et al., T-RO 2015):

$$\xi = c + \frac{\dot c}{\omega},\qquad \dot\xi = \omega(\xi-p)$$

**Exact discrete solution** (this is literally what Caron's `Pendulum::integrateIPM` does — verified from source): with VRP $r=p-g/\lambda$ (note: in the planar horizontal case $g$ term vanishes and $r=p$; in Caron's 3D code $g$ is the world gravity vector):

$$c_{k+1} = c_k\cosh(\omega T) + \frac{\dot c_k}{\omega}\sinh(\omega T) - r(\cosh(\omega T)-1)$$
$$\dot c_{k+1} = \dot c_k\cosh(\omega T) + \omega(c_k-r)\sinh(\omega T)$$
$$\ddot c_{k} = \lambda(c_k-p)+g$$

For **small $\omega T$** (here $6.26\times0.005=0.031$), use the cheap approximations $\cosh\approx1+u^2/2$, $\sinh\approx u+u^3/6$ (error $<10^{-6}$), or precompute a 64-entry table of $(\cosh,\sinh)$ indexed by step size.

DCM discrete integration is even simpler. The exact solution of $\dot\xi=\omega(\xi-p)$ for constant $p$ over one step:

$$\xi_{k+1} = p + (\xi_k-p)e^{\omega T}\approx \xi_k + \omega T(\xi_k-p)\left(1+\tfrac{\omega T}{2}\right)$$

With $\omega T=0.031$, a forward-Euler step $\xi_{k+1}=\xi_k+T\omega(\xi_k-p)$ has relative error $\sim1.6\%$ per step, negligible for a 5 ms tick with feedback; use RK2 or the exponential form for the MPC prediction.

### 1.2 CoM position and velocity from leg encoders (Jacobian)

There is no camera and no base velocity sensor. The floating base is reconstructed from **contacts + IMU** (the "kinematic floating base observer", Caron's `FloatingBaseObserver::estimatePosition`):

1. IMU gives base orientation $R_{0,m}$ (2-axis IMU → pitch/roll only; yaw held from model, exactly as Caron does: `orientation_ = rpyToMat(mRPY(0), mRPY(1), cRPY(2))`).
2. Assume the support foot pose is fixed on the ground: $X_{0,s}=$ known contact frame. With leg joint vector $q$ and forward kinematics $X_{0,s}=f(q)$, the base pose is $X_{0,b}=X_{0,s}\,(X_{s,b}(q))^{-1}$. Caron interpolates two feet with `leftFootRatio_`: $X_{0,\text{anchor}}=\text{interp}(X_{0,L},X_{0,R},\alpha)$.
3. Once the base pose is known, CoM position is from forward kinematics: $c=\text{com}(q_{\text{full}})$.
4. **CoM velocity** from the weighted CoM Jacobian:

$$\dot c = J_{\text{com}}(q)\,\dot q,\qquad J_{\text{com}}=\begin{bmatrix}\partial c_x/\partial q & \cdots\end{bmatrix}\in\mathbb{R}^{3\times n}$$

For a floating base, augment with 6 virtual joints: $q_a=[x_b,y_b,z_b,\phi,\theta,\psi,q_1\ldots q_n]$, and $J_{\text{com}}=[I_{3}\ 0_{3\times3}\ J_{\text{rel}}]$ when expressed at the CoM (the base translation columns are identity). Practical small-robot implementation:
   - **Per leg geometric Jacobian** of the CoM relative to the support foot: $J_{\text{leg}}\in\mathbb{R}^{3\times n_{\text{leg}}}$ computed from the leg's link mass-weighted Jacobians,
   $$\dot c = \frac{1}{m}\sum_{\text{links }i} m_i\,J_i(q)\,\dot q$$
   where $J_i$ is the 3×n positional Jacobian of link $i$'s CoM. With $n=21$ and 2 legs, this is ~3×21 multiply-accumulates, trivial at 200 Hz.
   - **Low-pass filter**: Caron uses `LowPassVelocityFilter` / EMA on $\dot c$ because encoder differentiation is noisy; use $\dot c_f \leftarrow (1-a)\dot c_f + a\,\dot c_{\text{raw}}$ with $a=T_s/(T_f+T_s)$, $T_f\approx20$ ms.
   - **Air detection**: $\|\sum f_z\| < \text{threshold}$ (Caron: `MIN_DSP_FZ=15 N`; for 1.5 kg robot set $\approx0.1\,mg\approx1.5$ N) → freeze $\dot c$ filter and hold base pose, since contact kinematics are invalid.
   - **Support-relative DCM** (Zhou et al. 2026, arXiv:2608.00500): $\xi_{\text{rel}}=\xi-p_{\text{foot}}$ computed from encoders + 2-axis IMU **without base linear velocity** — recommended for this robot. Since $\xi=c+\dot c/\omega$ needs $\dot c$, the relative form computes $\dot c$ purely from the contact-constrained leg Jacobian as above, which is exactly what encoder+IMU fusion provides; no global position drift.

### 1.3 Desired ZMP/CMP from DCM tracking error (Caron's `Stabilizer`, verified source)

Define errors $e_\xi=\xi^{ref}-\xi$, $e_p=p^{ref}-p$ (both horizontal), and a leaky integral $\bar e_\xi$ (EMA with time constant $T_i=5$ s) and derivative (stationary-offset filter $T_d=1$ s). The desired CoM acceleration:

$$\ddot c^{des}=\ddot c^{ref}+\omega\left(k_p e_\xi + ( \dot c^{ref}-\dot c )\right)+\omega k_i\bar e_\xi + \omega k_d \dot e_\xi$$

where $\dot e_\xi=\omega(e_\xi-e_p)$ (Caron's `dcmDerivator_.update(omega*(dcmError_ - zmpError_))`). Desired force: $f^{des}=m(\ddot c^{des}-g)$. Desired ground moment about measured CoM: $\tau^{des}=c_{\text{meas}}\times f^{des}$ (note: source comment says using measured rather than reference CoM fixed issue #28).

**Pole-placement gains** (Caron `polePlacement_`, from Morisawa et al. 2014). Given desired closed-loop poles $-\alpha,-\beta,-\gamma<0$ and ZMP actuator lag frequency $f_l$:

$$k_p^{zmp}=\frac{\alpha\beta+\beta\gamma+\gamma\alpha+\omega f_l}{\omega f_l},\quad k_i^{zmp}=-\frac{\alpha\beta\gamma}{\omega f_l},\quad k_d^{zmp}=-\frac{\alpha+\beta+\gamma+f_l-\omega}{\omega f_l}$$
$$k_p=\omega(k_p^{zmp}-1),\quad k_i=\omega T_i\,k_i^{zmp},\quad k_d=\omega\,k_d^{zmp}$$

Default poles in source: $(\alpha,\beta,\gamma,f_l)=(10,5,1,10)$ Hz; for the small robot scale these down: $\omega=6.26$, try $(\alpha,\beta,\gamma,f_l)=(6,3,0.8,8)$ → compute offline, store as constants. Clamp magnitudes (Caron maxima: $k_p\le20$, $k_i\le100$, $k_d\le2$; for small robot start $k_p\approx1$–3, $k_i\approx1$–5, $k_d\approx0$–0.5).

Equivalently, in DCM-error feedback form $\dot e_\xi=-K e_\xi$ with $K$ chosen by pole placement, and the ZMP command:

$$p^{des}=p^{ref}+k_p^{zmp}e_\xi+k_i^{zmp}\bar e_\xi+k_d^{zmp}\dot e_\xi$$

which is the same controller expressed as a ZMP correction rather than a CoM-acceleration correction. This is the form to use if you distribute via ankle torque/CMP.

### 1.4 Ankle vs hip strategy and the CMP offset

With nonzero centroidal angular momentum $L$ about the CoM, the generalized ZMP/CMP relation (Vukobratović; Orin et al. 2013):

$$p = c - \frac{z_c}{mg}\ddot c_{xy} - \frac{1}{mg}\begin{bmatrix}-\dot L_y\\ \dot L_x\end{bmatrix}
= c - \frac{\ddot c_{xy}}{\omega^2} - \frac{\dot L_{xy}^\perp}{mg}$$

Define **CMP** (centroidal moment pivot) $r_{\text{CMP}} = p + \dot L^\perp/(mg)$. A hip/torso strategy changes $\dot L$ instead of $p$: to move the ground CoP by $\Delta p$ using momentum only, require

$$\dot L^{\perp} = mg\,\Delta p\quad\Rightarrow\quad \Delta L_y = -mg\,\Delta p_x,\ \ \Delta L_x = mg\,\Delta p_y$$

Over one step $T$: $\tau_{\text{hip}}\approx\Delta L/T \approx mg\,\Delta p/T$. For $m=1.5$ kg, $\Delta p=1$ cm, $T=0.1$ s: $\tau_{\text{hip}}\approx1.5\cdot9.81\cdot0.01/0.1=1.47$ N·m. Feasible for 1–2 kg actuators.

**Distribution between ankle torque and hip momentum** (implementable): given desired CoP correction $\Delta p=(\Delta p_x,\Delta p_y)$ from §1.3:
1. Let ankle CoP authority be bounded by foot half-length $X$ and half-width $Y$ (for a 45 cm robot foot: $X\approx0.035$ m, $Y\approx0.018$ m). Clamp $\Delta p_{\text{ankle}}=\text{clip}(\Delta p,-X_{\text{margin}},X_{\text{margin}})$ with margin $\approx0.7X$ to avoid tipping.
2. Residual $\Delta p_{\text{hip}}=\Delta p-\Delta p_{\text{ankle}}$; convert to torso/angular-momentum change: $\dot L^{\perp}=mg\,\Delta p_{\text{hip}}$; command via upper-body/joint torques $\tau_{\text{hip}}=J_{\text{torso}}^{T}\Lambda\,\Delta\dot L$ or simply a torso pitch/roll torque PD.
3. If $\|\Delta p_{\text{hip}}\|$ exceeds available momentum rate (e.g. $|\dot L_z|\le L_{\max}$), the remainder can only be realized by a step (§2).

A practical blending scalar: $\beta\in[0,1]$, $\Delta p = \beta\,\Delta p_{\text{ankle}}+(1-\beta)\,\Delta p_{\text{CMP}}$, with $\beta=1$ for small disturbances and $\beta\to0$ as $\|\xi\|$ approaches capturability limits. This is the "ankle → hip → step" escalation.

### 1.5 Caron LIPM walking controller architecture (verified from source)

The open-source controller (`github.com/stephane-caron/lipm_walking_controller`, archived 2019; HRP-4, stair climbing, Airbus demo) runs under `mc_rtc` + `Tasks` QP. Components (from `Controller.h`, Doxygen overview, `Stabilizer.cpp`, `ModelPredictiveControl.h`):

| Stage | Class | Function |
|---|---|---|
| State estimation | `FloatingBaseObserver` | base pose from IMU orientation + interpolated support-foot kinematics (see §1.2) |
| Wrench measurement | `NetWrenchObserver` | net contact wrench from foot force sensors |
| Velocity estimation | `LowPassVelocityFilter` | filtered CoM velocity |
| Gait plan | `FootstepPlan`, `PlanInterpolator` | contact sequence, SSP/DSP durations, interpolated preview |
| Pattern generation | `ModelPredictiveControl` (copra, QLD QP) | 16-step linear MPC, $T=0.1$ s, jerk input |
| Reference reduced state | `Pendulum` | analytic IPM integration, holds $c,\dot c,\ddot c,p,\omega$ |
| Balance feedback | `Stabilizer` | DCM PID → desired wrench; ZMP-compensation control (ZMPCC); foot-force difference control |
| Wrench distribution | `Stabilizer::distributeWrench` / `saturateWrench` | 12-var / 6-var QP with contact wrench cone (CWC) inequalities |
| Whole-body tracking | `mc_tasks::CoMTask`, `CoPTask`, orientation tasks | IK/ID QP solved by `Tasks` library |

`Controller::run()` sequence (per control cycle): observe floating base → update preview (MPC, every 0.1 s) → update pendulum reference → stabilizer computes task targets → `mc_control::fsm::Controller::run()` (FSM states: Initial → Standing → SingleSupport → DoubleSupport) → QP → torques/position targets.

**Wrench distribution QP (exact, from source).** Variables $x=[w_L;w_R]\in\mathbb{R}^{12}$ (spatial wrenches at world origin). Cost rows:
- $|w_L+w_R-w^{des}|^2$ (weight $\sqrt{w_{\text{net}}}$),
- $|A_{\text{ankle}} w|^2$ with diagonal $(1,1,10^{-4},10^{-3},10^{-3},10^{-4})$ in ankle frame (weight $\sqrt{w_{\text{ankle}}}$),
- $|(1-\alpha)f_{L,z}-\alpha f_{R,z}|^2$ (force-ratio, weight $\sqrt{w_{\text{fratio}}}$).

Constraints ($16+16+2$ rows): CWC per foot (`wrenchFaceMatrix_`, 16 inequalities, see below) and $f_z\ge15$ N each. Q matrix $Q=A^\top A$, $c=-A^\top b$, solved with `eigen-quadprog`. Single support reduces to a 6-var QP projecting onto one CWC.

**Contact wrench cone (16×6 H-rep, verified source).** For sole half-length $X$, half-width $Y$, friction $\mu$, rows over $(m_x,m_y,m_z,f_x,f_y,f_z)$:

$$\pm f_x+\mu f_z\le0,\quad \pm f_y+\mu f_z\le0,\quad \pm m_x-Y f_z\le0,\quad \pm m_y-X f_z\le0$$
$$\pm\mu m_x\pm\mu m_y - m_z - Y f_z - X f_z-(X+Y)\mu f_z\le0\text{ (4 rows)}$$
$$\pm\mu m_x\pm\mu m_y + m_z - Y f_z - X f_z-(X+Y)\mu f_z\le0\text{ (4 rows)}$$

For a no-force-sensor small robot, **replace this with a pure ZMP support-polygon check** plus estimated $f_z\approx mg$ split by contact: $|p_x|\le X$, $|p_y|\le Y$, and inferred friction from commanded accelerations $\mu\ge\|\ddot c_{xy}\|/g$.

**ZMP computation from a wrench** (source): $p = p_{\text{frame}} + \frac{n\times(m_{\text{frame}})}{n\cdot f}$ where $m_{\text{frame}}=m_0-p_{\text{frame}}\times f$, $n=$ contact normal.

**ZMP compensation control (ZMPCC)** (source): leaky-integrate CoM offset from ZMP error:
$$v_{\text{new}}=-R_{c}^{0}\,\text{Adm}\circ(R_{0}^{c}(p_{\text{dist}}-p_{\text{meas}})),\quad \Delta c = \text{LeakyInt}(v_{\text{new}}),\quad c^{task}=c^{ref}+\Delta c$$
with $\text{MAX\_ZMPCC\_COM\_OFFSET}=0.05$ m (for the small robot clamp to $\approx0.01$ m). This absorbs model error without changing the reduced plan.

---

## 2. Capturability regions and strategy switching

### 2.1 Capture region / N-step capturability

**One-step capturable set** (Caron et al., "Capturability-based analysis and control of legged locomotion", IJRR 2012; "Capturability-based pattern generation for walking with variable height", T-RO 2019, arXiv:1801.07022): the set of DCM states $\xi$ for which there exists an admissible next foot placement $p_f$ (inside reachable workspace $\mathcal{F}$ and within step time $T_{\text{SSP}}$) such that the pendulum can be captured at $p_f$. With DCM evolution $\xi(t)=p_f+(\xi_0-p_f)e^{\omega t}$, the state at end of step $T$:

$$\xi(T)=p_f+(\xi_0-p_f)e^{\omega T}$$

Capture at landing requires $\xi(T)=p_f$ → only possible if $\xi_0=p_f$; otherwise the **capture point after the step** must lie inside the next support polygon. The one-step capture region is the set of $\xi_0$ for which $\exists p_f\in\mathcal{F}$ with $\xi(T)\in\mathcal{S}_{\text{next}}$. In the LIPM with fixed $h$ and foot position, the **capture region boundary** is (Caron 2012):

$$\xi_x^{\max} = p_{\max}+\frac{(p_{\max}-p_{\text{current}})}{e^{\omega T}-1}\ \text{(generalized form; solve numerically per axis)}$$

The implementable test (no geometry library): for each candidate $p_f$ on a small grid (e.g. 3×3 within reach), simulate the exponential DCM step above (one multiply) and test containment in the next support box. $O(9)$ f32 ops, trivial.

**N-step capturability** (Caron 2012; "Capture steps", Missura & Bennewitz IJHR 2020, arXiv:2011.02793): recurse the reachability set backward $N$ times. In the variable-height generalization (Caron et al. 2019, arXiv:1801.07022) the capture-input map is computed by a tailored optimizer fast enough for MPC. On the MCU use the **fixed-height approximation** and a precomputed table: $N=1,2,3$ boundaries as functions of $\xi$ offset and step time, or just the one-step test with a safety margin.

### 2.2 Ankle vs step decision (concrete thresholds)

Let support polygon half-extents $X,Y$ (e.g. $X=0.035$ m, $Y=0.018$ m) and define margin $m=0.2$ (80% usable). Decision per tick:

1. **Ankle strategy** if $\|\xi-p_{\text{current}}\|_{\infty,\text{scaled}}\le(1-m)(X,Y)$: use CoP/DCM feedback §1.3 with ankle authority only.
2. **Hip strategy** if $\xi$ is outside ankle range but within $\rho_h$ of support and a single step is not yet required: use CMP offset via $\dot L$ (§1.4). Practical threshold: $\|\Delta p_{\text{hip}}\|\le 0.02$ m → $\dot L \le m g\cdot0.02=0.29$ N·m·s, achievable over $T=0.1$ s with $\tau\le2.9$ N·m.
3. **Step strategy** if $\xi\notin$ ankle+hip authority, or the one-step capturability test fails. Foot target:

$$p_f = \xi + k_v(\dot c-\dot c^{des})\,\frac{1}{\omega}\ \text{(dimensionless }k_v\text{)} \quad\text{or}\quad p_f = \xi - \frac{\dot c}{2}T_{\text{SSP}} + k_v\,\Delta v$$

Raibert-style form (Pratt 2006; Missura 2020): $x_f = x_{\text{com}}+\dot x\,T_{\text{SSP}}/2+k_v(\dot x-\dot x^{des})$; the DCM form $x_f=\xi+k_v(\dot x-\dot x^{des})/\omega$ is equivalent up to the ballistic term. Clamp $p_f$ to reachable workspace; if clipped and still uncapturable, shorten the step (Khadiv et al., T-RO 2020, arXiv:1704.01271 — only the *next* step needs optimization).

**Small-robot numbers.** With $\omega=6.26$ and foot $X=0.035$ m, ankle authority is $\approx\pm0.028$ m of CoP. A push producing $\dot c=0.1$ m/s gives $\xi$ shift $=0.1/6.26=1.6$ cm → within ankle range (ankle suffices). $\dot c=0.3$ m/s → $\xi=4.8$ cm → outside foot → needs a step. This is the practical switching boundary: **ankle for $\dot c\lesssim0.15$ m/s, step for $\dot c\gtrsim0.2$ m/s** for a 45 cm robot with 3.5 cm foot. Tune against capture-step tests.

---

## 3. Task-space priority control / operational space control on an MCU

### 3.1 DLS / null-space projection formulation

**Operational space control** (Khatib 1987 **[KB]**): for task coordinate $x=f(q)$, Jacobian $J=\partial x/\partial q$, task-space inertia $\Lambda=(J M^{-1}J^\top)^{-1}$ (damped for singularities: $\Lambda=(J M^{-1}J^\top+\epsilon I)^{-1}$):

$$\tau = J^\top\Lambda\left(\ddot x^{des}-\dot J\dot q\right) + N(q)\,\tau_{\text{lower}},\qquad N=I-J^\top\bar J^\top,\ \bar J = M^{-1}J^\top\Lambda$$

This is torque-level inverse dynamics; requires $M(q)$ and Coriolis $C(q,\dot q)$. Full version: $\tau=M(q)\ddot q^{des}+C(q,\dot q)\dot q+g(q)$ with $\ddot q^{des}=J^\dagger(\ddot x^{des}-\dot J\dot q)+N\ddot q_{\text{post}}$.

**Damped least squares (DLS)** avoids explicit $\Lambda$: $J^\dagger=J^\top(JJ^\top+\lambda^2 I)^{-1}$, and $N=I-J^\dagger J$. For a 3-D task, $(JJ^\top+\lambda^2I)$ is 3×3 → closed-form inverse. Cost ~$3\times n\times 3$ flops per level. This is the version to implement at 200 Hz.

**Hierarchical stack** (Siciliano & Slotine **[KB]**): for tasks 1..k with Jacobians $J_1,\dots,J_k$ and null-space projectors $N_1=I-J_1^\dagger J_1$, $N_2=N_1(I-J_2^{\dagger|1}J_2)$, etc., the stacked solution:

$$\ddot q = J_1^\dagger e_1 + N_1\left[J_2^\dagger(e_2-\dot J_2\dot q) + N_2\left(J_3^\dagger(\cdots)+N_3\ddot q_{\text{post}}\right)\right]$$

where $e_i=\ddot x_i^{des}-\dot J_i\dot q$ and $J_i^{\dagger|i-1}=(J_i N_{i-1})^\dagger$ (task Jacobian projected into the previous null space). For 21 DoF and 3-D tasks, each projector is a 21×21 matrix — **do not form it**; apply projectors to vectors: $N v = v - J^\top\bar J^\top v$ with a 3×3 solve. Memory O(n) per stack level.

**Recommended 4-level stack** for Rocky (f32, 200 Hz):
1. **Foot contact / support constraint** — 6-D (position+orientation) of stance foot, $\dot x=0,\ddot x=0$; highest priority; treat as hard constraint (also builds the floating-base closure, §3.3).
2. **CoM / DCM tracking** — 2-D horizontal CoM accel from §1.3 (or DCM ZMP command mapped to CoM accel $\ddot c=\omega^2(c-p^{des})$).
3. **Attitude** — 2-D torso pitch/roll (from 2-axis IMU), desired $0$; weight moderate.
4. **Posture** — full 21-D joint PD to nominal half-sit pose in the residual null space: $\tau_{\text{post}}=K_p(q_0-q)-K_d\dot q$.

Use **weighted DLS** (task-space weights diagonal $W_i$) in level 3/4 to soften trade-offs.

### 3.2 Task-space inverse dynamics vs operational space

- **Operational space** (Khatib 1987): control law in task coordinates with dynamically consistent $\Lambda$; needs inverse dynamics but gives decoupled task dynamics and exact null-space torque injection. Best when you have torque control and a good $M,C,g$ model.
- **Task-space inverse dynamics / hierarchical QP** (Kuindersma et al., Autonomous Robots 2016; Caron's `Tasks`): solve $\min_{\ddot q,f,\tau}\sum_i w_i\|J_i\ddot q+\dot J_i\dot q-\ddot x_i^{des}\|^2$ s.t. dynamics $M\ddot q+C\dot q+g=S\tau+J_c^\top f$, friction cones, torque limits. Strict priority via lexicographic null spaces. More general (handles contacts/limits) but needs a dense QP solver — not feasible on RP2350 at 200 Hz.

For the MCU, use the **operational space / DLS recursion with a simplified dynamics model** (§3.3), not a general QP. Cost: per level a 3×3 Cholesky/inverse (~60 flops) + a few 21-vectors; total ~$10^4$ flops → <0.5 ms on Cortex-M33 at 150 MHz.

### 3.3 Floating base without full dynamics

Use the **6-DoF virtual joint** approach **[KB]**: augment $q_a=[q_b; q]$ with $q_b=(x,y,z,\phi,\theta,\psi)$ and a diagonal fictitious mass/inertia $M_b=\text{diag}(m,\ldots,m,I,\ldots,I)$ (set $m=1.5$, $I\approx m\ell^2/12\approx0.006$). Then:
- Build $J_i$ w.r.t. $q_a$ for each task (the foot/Jacobian functions already support a floating base).
- Add the **contact constraint as a task of priority 1**: $J_c\ddot q_a+\dot J_c\dot q_a=0$ for the stance foot (6 rows). This closes the kinematic loop and determines the base acceleration from leg accelerations.
- Gravity compensation: add $g(q)$ as a feed-forward bias in the lowest level or compute it from a simplified centroidal model ($g_{\text{base}}=[0,0,-mg]$ plus link contributions), avoiding full $C(q,\dot q)$.
- Coriolis: neglect or use a **feedback linearization residual** folded into the joint PD (for a 1–2 kg robot at 0.2–0.5 m/s, $C\dot q$ terms are small relative to gravity and motor authority). Alternatively precompute a sparse $C(q,\dot q)$ at 20 Hz.

This gives a torque command $\tau=S^\top(M\ddot q_a^{des}+g)$ with $S=[0\ I_{21}]$ selecting actuated joints. If motors are position/velocity controlled over I3C (SimpleFOC torque/velocity/position modes), convert desired torque to a velocity/position offset via the actuator impedance: $\Delta q = \tau/K_{\text{eff}}$ or $\dot q_{\text{cmd}}=\tau/D_{\text{eff}}$, with $K_{\text{eff}}\approx$ joint stiffness from gear ratio and current limit.

### 3.4 MIT WBIC structure (Kim et al., 2019)

The Whole-Body Impulse Controller (WBIC; Kim, Di Carlo, Chignoli, Kim, "High-speed dynamically diverse running of humanoid robots", 2019/2020; used on MIT Humanoid) is a **three-tier hierarchical QP with strict priority**:

1. **Floating-base dynamics / reaction-force QP** — solves for contact reaction forces $f$ and base acceleration $\ddot q_b$ satisfying the centroidal/floating-base dynamics for desired CoM linear momentum $\dot h_G^{des}$ and angular momentum $\dot L_G^{des}$ (plus contact constraints, friction cones). This is a small QP (~12–18 vars).
2. **Actuated joint QP** — with $f$ fixed, solve joint accelerations/torques tracking swing-foot and torso orientation tasks while satisfying contact kinematics.
3. **Task/allocation hierarchy** — additional tasks (e.g. swing foot, arm posture) in lower priority null spaces; follow-up work ("WBLC", Kim et al. 2019, arXiv:1901.08100) relaxes contact constraints for smooth switching.

CoM trajectories for WBIC come from a convex MPC (Di Carlo et al., IROS 2018, DOI 10.1109/IROS.2018.8594448) and foot placement from Raibert/ capture-point; this is the canonical "MPC → WBIC → PD" stack. **MCU-feasible simplifications:**
- Replace QP tier 1 with the analytic ground-reaction-force solve from §1.3/§1.4 (CMP/ZMP → $f^{des}=m(\ddot c-g)$, $\tau_H = c\times f^{des}+\dot L^{des}$), and a diagonal redistribution of $f_z$ across feet by the support ratio $\alpha$.
- Replace tier 2/3 QPs with the DLS/null-space recursion §3.1 and a diagonal joint-space PD for posture.
- Freeze $M,\Lambda,J$ at a nominal half-sit configuration and precompute $J^\dagger$ / projector matrices at startup (or at 20 Hz): "practical simplified version feasible at 200 Hz with precomputed matrices" — store a 21×21 nominal mass matrix $M_0$, its Cholesky, and task Jacobians at $q_0$; update only the 3×3 task inverses online.

---

## 4. Small f32 LIPM-MPC for a hand-coded solver

### 4.1 Exact QP (Scianca et al., T-RO 2020; Caron's copra-based MPC)

**State** (choice A, 6-D cart-table): $x_k=[c_x,c_y,\dot c_x,\dot c_y,\ddot c_x,\ddot c_y]$; **input** $u_k=[\dddot c_x,\dddot c_y]$ (CoM jerk) — exactly Caron's `STATE_SIZE=6`, `INPUT_SIZE=2`, $T=0.1$ s, $N=16$. Dynamics per axis:

$$c_{k+1}=c_k+T\dot c_k+\tfrac{T^2}{2}\ddot c_k+\tfrac{T^3}{6}u_k,\quad \dot c_{k+1}=\dot c_k+T\ddot c_k+\tfrac{T^2}{2}u_k,\quad \ddot c_{k+1}=\ddot c_k+T u_k$$

ZMP from state: $p_k = c_k - (h/g)\,\ddot c_k$ (Caron's `zmpFromState_` maps state order $(c_x,c_y,\dot c_x,\dot c_y,\ddot c_x,\ddot c_y)$ via row $[1,0,0,0,-h/g,0]$). Two DCM-like quantities must not be conflated:
- **Standard DCM / capture point** (used throughout §1–2): $\xi=c+\dot c/\omega$ (velocity form).
- **Caron's cart-table MPC terminal quantity**: his `dcmFromState_ = [1,0,\omega^{-1},0,0,0;\ 0,1,0,\omega^{-1},0,0]` indexes **state slot 2 = $\ddot c$**, extracting $c+\ddot c/\omega$. This is used only for the MPC terminal stability constraint, not the balance feedback $\xi$ of §1.

For the DCM-form MPC of §4.2 use the velocity definition $\xi=c+\dot c/\omega$. The cart-table state $(c,\dot c,\ddot c)$ is sufficient for either; keep the indexing straight in code (one f32 vector, two extraction rows).

**Cost** (Caron defaults, verified):
$$J=\sum_{k=0}^{N} w_v\|\dot c_k-\dot c_k^{ref}\|^2 + w_p\|p_k-p_k^{ref}\|^2 + \sum_{k=0}^{N-1} w_j\|u_k\|^2,\quad (w_v,w_p,w_j)=(10,1000,1)$$

**Constraints** (linear box/H-rep per step):
- ZMP in support polygon: $A_{\text{supp}}(k)\,p_k\le b_{\text{supp}}(k)$ (support switches at known step times; 4 rows/foot for a rectangle).
- **Terminal stability constraint** (Wieber 2006; Scianca IS-MPC 2020): terminal DCM inside final support and/or ZMP inside final support — guarantees boundedness of CoM relative to ZMP. Scianca's IS-MPC uses **ZMP velocity as input** and an explicit "intrinsically stable" constraint linking future ZMP velocities to current state so the CoM/ZMP stay bounded; this adds no extra decision variables beyond the ZMP rate sequence.

**Decision variables** for Caron: $N\!+\!1$ states ×2 axes; with $N=16,T=0.1$ s: horizon 1.6 s. Scianca uses a similar preview horizon (1–2 s).

### 4.2 DCM-form MPC (smaller)

Use $x=[\xi_x,\xi_y,p_x,p_y]$, dynamics $\xi_{k+1}=p_k+(\xi_k-p_k)e^{\omega T}$, input $u_k=\dot p_k$ (ZMP velocity), constraint $p_k\in\mathcal{S}(k)$. Cost $\sum\|\xi_k-\xi_k^{ref}\|^2+\rho\|u_k\|^2$. With $N=6$–$10$ and $T=0.05$–$0.1$ s, variables $=2(N+1)$ (e.g. 14–22). Scianca IS-MPC uses exactly the ZMP-velocity-input form; for DCM form see Caron's tutorial **[KB]**.

### 4.3 Solving on Cortex-M33 in f32

**Practical choice:** $N=6$, $T=0.08$ s (0.48 s horizon), update at 10–20 Hz (every 5–10 control ticks), hold the first ZMP/CoM target between solves. This is enough for one upcoming step on a 0.2–0.5 m/s small walker (step time ~0.3–0.5 s); Caron's 16×0.1 s is for slower HRP-4.

Hand-coded solver options:
1. **Fixed-iteration active-set** (primal/dual) on the box-constrained QP. For the 2-axis separable problem, solve **two independent 1-D QPs** (x and y), each with $N$ variables, tridiagonal Hessian from the double integrator. A dedicated tridiagonal solver (Thomas algorithm) is $O(N)$ with ~$10N$ f32 ops. Active-set on ZMP bounds adds ~$2$–$5$ iterations of re-solving with clamped variables; cap at 8 iterations. Total ~$10^3$ flops.
2. **ADMM**: minimize $\tfrac12 z^\top P z + q^\top z$ s.t. $Az\in\mathcal{C}$ (ZMP boxes). Iterations:
$$z^{k+1}=\text{cholSolve}(P+\rho A^\top A,\ -q+\rho A^\top(v^k-w^k)),\quad v^{k+1}=\Pi_{\mathcal{C}}(Az^{k+1}+w^k),\quad w^{k+1}=w^k+Az^{k+1}-v^{k+1}$$
where $\Pi_{\mathcal{C}}$ is component-wise clipping to the support boxes. Precompute the Cholesky of $P+\rho A^\top A$ once (it depends only on $T,N,\rho$, not on the state!). Fixed 12–20 iterations; store all arrays statically (`[N]` f32 stacks). ADMM converges slowly for tight tolerances but 15 iterations with $\rho\approx10\cdot\text{diag}(P)$ gives cm-level ZMP accuracy — adequate.
3. **Conjugate gradient on the box-QP** (Goldfarb-Idnani is overkill). For separable x/y and short horizon, active-set is simpler.

**f32 care:** scale positions by $1/X_{\text{foot}}$ and velocities by $T/X_{\text{foot}}$ so Hessian entries are $O(1)$; avoid accumulating $e^{\omega T}$ powers by using precomputed $\phi=e^{\omega T}$ table; clamp all integrators; check residual and fall back to last solution / capture-point law on non-convergence. No dynamic allocation — all matrices as `const` arrays compiled for fixed $N$.

**Citation anchors:** Scianca, De Simone, Lanari, Oriolo, "MPC for Humanoid Gait Generation: Stability and Feasibility", IEEE T-RO 36(3), 2020, DOI 10.1109/TRO.2019.2958483 (arXiv:1901.08505) — IS-MPC, NAO/HRP-4; Caron et al. "Capturability-based pattern generation..." T-RO 2019 (arXiv:1801.07022) — fast tailored solver for VHIP; Wieber 2006 — trajectory-free MPC; Khadiv et al. 2020 (arXiv:1704.01271) — step timing adaptation.

---

## 5. Control Barrier Functions (CBF) for safety

### 5.1 Continuous-time CBF condition (Ames et al. **[KB]**)

For dynamics $\dot x=f(x)+g(x)u$ and safe set $\mathcal{C}=\{x:h(x)\ge0\}$, $h$ is a CBF if $\exists\alpha\in\mathcal{K}_\infty$ s.t.

$$L_f h(x)+L_g h(x)\,u \ge -\alpha(h(x))$$

where $L_fh=\nabla h^\top f$, $L_gh=\nabla h^\top g$. **CBF-QP safety filter**:

$$u^\ast=\arg\min_{u}\|u-u^{nom}\|_2^2\quad\text{s.t. } L_fh+L_gh\,u\ge -\alpha(h)$$

With relative degree $r>1$ (e.g. tilt affected by torque through acceleration), use **exponential CBF (ECBF)**: define $\eta_1=h,\eta_2=\dot h,\dots,\eta_r=h^{(r-1)}$, and require

$$h^{(r)} + K_\alpha\begin{bmatrix}h&\dot h&\dots&h^{(r-1)}\end{bmatrix}^\top\ge0$$

with $K_\alpha$ chosen so $s^r+K_{\alpha,r-1}s^{r-1}+\dots+K_{\alpha,0}$ is Hurwitz. Paredes & Hereid (arXiv:2311.08409) use ECBFs in an inverse-dynamics QP on Digit, enforcing ZMP, friction cone, and torque constraints with high-relative-degree safety sets. Khazoom et al. (arXiv:2207.00692) combine CBF with WBC on MIT Humanoid; Ahn, Lee & Sentis (arXiv:1906.03812, RA-L 2020) use step capturability + CBF for safe learning.

### 5.2 Simple scalar CBFs appropriate for the MCU

**(a) Torso tilt CBF.** Let $\theta$ = torso pitch from 2-axis IMU, $\dot\theta$ its rate, and torque command $u=\tau_\theta$. With $\ddot\theta = a_\theta+b_\theta u$ (identified $a_\theta,b_\theta$) and safe set $h=\theta_{\max}^2-\theta^2\ge0$ (or $h=\theta_{\max}-|\theta|$), relative degree 2 → ECBF:

$$\ddot h + k_1\dot h+k_0 h\ge0,\quad \dot h=-2\theta\dot\theta,\quad \ddot h=-2(\dot\theta^2+\theta\ddot\theta)$$
$$\Rightarrow -2\theta(a_\theta+b_\theta u) \ge 2\dot\theta^2 - k_1(2\theta\dot\theta)-k_0(\theta_{\max}^2-\theta^2)$$

Solve for $u$ with $u\in[u_{\min},u_{\max}]$: one scalar bound → **explicit projection** (no QP):
$$u^\ast = \min\left(u_{\max},\max\left(u_{\min},\frac{2\dot\theta^2 - k_1(2\theta\dot\theta)-k_0 h - 2\theta a_\theta}{-2\theta b_\theta}\right)\right)$$
Guard against $\theta\approx0$ (division by zero) by using a smooth barrier $h=\theta_{\max}-\theta^2/\theta_{\max}$ with nonzero gradient at 0, or use $h=\theta_{\max}^2-\theta^2$ and treat the singular case by reverting to PD.

**(b) CoM velocity CBF.** $h=v_{\max}^2-\dot c^2\ge0$ with $\ddot c\approx\omega^2(c-p)$ = affine in CoP/ZMP command $p$. Relative degree 1 if $u=p$ directly:

$$\nabla h^\top f+\nabla h^\top g\,p = -2\dot c\,\omega^2(c-p)\ge-\alpha(v_{\max}^2-\dot c^2)$$

→ linear bound on $p_x$; intersect with the support-polygon box → clip $p^{cmd}$ to the interval. This is a **CBF-projected ZMP command**, trivial in f32. Choose $\alpha(h)=\kappa h$ with $\kappa\approx2$–$5$ s⁻¹ (timescale 0.2–0.5 s, consistent with the 5 ms tick).

**(c) Capture point CBF.** $h=\xi_{\max}^2-\|\xi-p_{\text{foot}}\|^2$; since $\dot\xi=\omega(\xi-p)$ is affine in $p$, this is relative degree 1 and yields a linear constraint on $p$: $-2(\xi-p)\omega(\xi-p_{\text{foot}})\ge-\alpha h$. Used in HULK (arXiv:2610.08970) as a capture-point CBF during RL training, reducing DCM excursion 35.7%.

**(d) Discrete-time CBF** ( Peng, Paredes, Castillo & Hereid, arXiv:2411.03619, 2024): for the sampled LIP gait use $h(x_k)\ge0$ and the linearized discrete condition $h(x_{k+1})\ge(1-\gamma)h(x_k)$ with $x_{k+1}=Ax_k+Bu_k$, giving a **linear** constraint $A_h x_k+B_h u_k\ge-\gamma h(x_k)$ — perfect for the f32 MPC/QP of §4 (add as an extra row). For a 1-DOF tilt with $x=[\theta,\dot\theta]$, $A=\begin{bmatrix}1&T\\0&1\end{bmatrix}$, $B=[T^2/2,T]^\top$.

**Implementation recommendation:** use (b)+(c) as continuous ZMP command projection (a few flops), and (d) as 1–2 extra inequality rows in the §4 MPC. Keep (a) as a final torque-level clamp. These are "filtered command" CBFs, not a QP solver — appropriate for 200 Hz f32 no-alloc.

---

## 6. SLIP / Raibert running equations for 1–2 kg

### 6.1 SLIP template (Blickhan 1989; Full & Koditschek 1999)

Stance: point mass $m$ on a massless leg modeled as linear spring $k$ with rest length $L_0$, hip at the mass. Force along leg direction $e=(p_{\text{foot}}-c)/L$:

$$F_s = k(L_0-L)\,\mathbf{e},\qquad L=\|c-p_{\text{foot}}\|,\quad F_s\ge0\text{ (no pull)}$$

Equations of motion (gravity $g$):

$$m\ddot c = k(L_0-L)\,\mathbf{e} - m g\,\hat z,\qquad \dot L = -\dot c\cdot\mathbf{e}\ \text{(foot fixed in stance)}$$

Flight: ballistic $\ddot c=-g\hat z$, $\dot c_z(t)=\dot c_z(0)-gt$, $c_z(t)=c_z(0)+\dot c_z(0)t-\tfrac12 gt^2$. Apex when $\dot c_z=0$; apex height $H=c_z(0)+\dot c_z(0)^2/(2g)$. Dimensionless spring stiffness $K=kL_0/(mg)$ and Froude number $Fr=v^2/(gL_0)$ govern gait (Saranli 2010; Uyanık, Morgül & Saranli, T-RO 2015 give approximate analytic stance solutions; Chen, Wensing & Zhang, arXiv:1901.05840 give differential flatness).

**Parameter selection for 1.5 kg, $L_0=0.22$ m.** To support weight with 10% compression at rest: $k\approx mg/(0.1L_0)=14.7/0.022=670$ N/m. For a 20 cm apex hop ($H=0.2$ m), take-off velocity $\dot c_z=\sqrt{2gH}=1.98$ m/s. Stance half-duration for a linear spring ~$\pi/(2\omega_s)$, $\omega_s=\sqrt{k/m}=\sqrt{670/1.5}=21.1$ rad/s → $T_s\approx0.075$ s. Flight time $T_f=2\dot c_z/g=0.40$ s. Duty factor $\beta=T_s/(T_s+T_f)\approx0.16$ — typical for running.

### 6.2 Raibert control (Raibert 1986 **[KB]**)

Three independent controllers:
1. **Foot placement (forward speed):** at touchdown,
$$x_{\text{foot}} = x_{\text{hip}} + \frac{T_s}{2}\dot x + k_v(\dot x-\dot x^{des})$$
where $T_s$ = stance duration (the $T_s/2\dot x$ term cancels stance velocity to leave zero at mid-stance). $k_v\approx0.05$–$0.2$ s (tune). For the SLIP with flight, use $T_{\text{stride}}/2$ instead of $T_s/2$ in some formulations; the mid-stance neutral point is $x_{\text{foot}}=x_{\text{com}}+\dot x T_{\text{stance}}/2$.
2. **Hopping height (vertical):** thrust impulse $\Delta v_z = \sqrt{2g(H^{des}-H_{apex})}$ or, in terms of stance energy, set leg stiffness/neutral length: $L_0^{cmd}=L_0+\frac{m}{k}(\ddot c_z^{des}+g)$, and command net vertical impulse $J_z=\int (F_z-mg)dt=m(\dot c_z^{to}-\dot c_z^{td})$. For symmetric hopping $\dot c_z^{to}=-\dot c_z^{td}=v_z$: $J_z=2mv_z$, and required energy per step $E=2mgH$. With $H=0.2$ m, $E=2\cdot1.5\cdot9.81\cdot0.2=5.9$ J per stride.
3. **Attitude (body pitch):** hip torque during stance $\tau_\theta=-k_p\theta-k_d\dot\theta$ transfers angular momentum to the body; or shift CoP within foot.

**MCU implementation:** CPG/phase clock from §2; during stance run spring-force PD on the leg (SimpleFOC torque mode: $\tau_{\text{joint}}=J_{\text{leg}}^\top F_s$); during flight retract leg to preload length; at predicted touchdown compute Raibert foot target and place leg via IK. Use $\dot x$ from the contact Jacobian estimator (§1.2) — no camera.

### 6.3 Flight phase detection from IMU + zero contact current

No force sensors: use **motor current as proxy** plus IMU:
- In stance, leg joints carry load → `SimpleFOC` current (Iq) has a sustained DC component $\approx\tau_{\text{load}}/K_t$; in flight, joints move ballistically with near-zero load current (only inertia/friction).
- Detector: $I_{\text{leg}}=\sum_i|I_{q,i}|$ (or the vertical-projected joint torque from Jacobian transpose). Declare flight if $I_{\text{leg}}<I_{\text{th}}$ for $>2$ ms **and** IMU specific force $|a_m-g|<\epsilon$ with $\epsilon\approx0.1g$ (free fall → accelerometer reads ~0). Combine: $\text{flight} = (I_{\text{leg}}<I_{\text{th}})\land(\|a_m\|<0.3g)$.
- Confirm with leg extension $L\to L_0$ and vertical velocity sign; debounce 2–3 ticks (10–15 ms) to avoid chatter at touchdown.
- For $m=1.5$ kg and $k\approx670$ N/m, stance current is large; threshold $I_{\text{th}}\approx0.1\,I_{\text{stance,peak}}$ works. Store per-joint current baselines at startup.

---

## 7. I3C bus budget analysis (21 SimpleFOC nodes)

### 7.1 Bus parameters

I3C (MIPI I3C v1.1.1 **[KB]**): SDR push-pull up to **12.5 MHz** (typical 6–10 MHz with 21 nodes and long traces); HDR modes (HDR-DDR 2×, HDR-TSL/TSP) higher but complex. Useful throughput in SDR: $12.5\text{ MHz}/8\approx1.56$ Mbyte/s raw, minus framing/ACK/turnaround overhead ~20–30% → ~1.0–1.2 MB/s usable. I²C-compatible 0.4–1 MHz legacy devices may share the bus.

**Frame overhead** (SDR, per private transaction): START (address, ~1 byte), command/target address, data bytes, parity/ACK, STOP; practical cost ≈ **6–8 bytes overhead** per register read/write transaction depending on controller (e.g. Raspberry Pi Pico / RP2350 PIO I3C controller). For bulk reads with repeated-start, overhead amortizes.

### 7.2 Per-joint data per 5 ms cycle

Command (host → node): mode byte + setpoint. For SimpleFOC torque/velocity/position:
- mode (1 B) + target (f32 = 4 B) + optional feedforward/kp (another 4 B) ⇒ **6–10 B payload**; with 8 B overhead → **~14–18 B per joint per cycle** for a single write transaction.

Readback (node → host): position (i32 ticks, 4 B) + velocity (f32 or i32, 4 B) + current Iq (f32 or i16, 2–4 B) + status/flags (1–2 B) ⇒ **12–15 B payload**; plus overhead → **~18–23 B per joint**.

**200 Hz full-state exchange:** 21 joints × (write ~16 B + read ~21 B) = 21×37 ≈ **777 B/cycle**, ×200 Hz = **155 kB/s**. This is **~15% of a 1 MB/s usable 12.5 MHz bus** — fits comfortably even at 6 MHz (~0.5 MB/s, 31% load). Even with worst-case 2× overhead, 200 Hz fits. Note: at 12.5 MHz, 777 B takes ~0.62 ms of bus time per 5 ms tick (12% duty); with 6 MHz ~1.3 ms (26%).

### 7.3 Bus design recommendations

- **Broadcast commands** for synchronized mode changes / emergency stop / timestamp (I3C broadcast address 0x7C) — 1 transaction for all 21 nodes. Use I3C **broadcast with synchronous push-pull** for the per-cycle "sample now" trigger so all nodes latch setpoints simultaneously (jitter <1 µs).
- **Per-device reads** for telemetry: 21 private reads. Alternatively **IBI (in-band interrupt)** for fault reporting: a node asserts SDA low during SCL high after STOP; controller does a retarget read. Use IBI for over-current/over-temperature events instead of polling — saves bandwidth and gives event latency ~10–50 µs.
- **Dynamic address assignment (DAA):** I3C assigns 7-bit dynamic addresses at bus init using each target's 48-bit Provisional ID (PID) + random 24-bit seed; this allows hot-plug and avoids address conflicts (21 nodes can't share static I²C addresses). RP2350's I3C controller (via PIO or the SMI-based controller) supports DAA in host-role; assign addresses at startup and store in a table keyed by PID → joint ID.
- **Push-pull vs open-drain:** SDR uses push-pull (totem-pole) after the open-drain START/address phase → fast edges, low crosstalk, supports 12.5 MHz. Keep stub lengths short (<10 cm per node) or use a managed multi-drop topology; at 21 nodes, bus capacitance is the limiting factor — 12.5 MHz may derate to 6 MHz; still fits.
- **Latency budget:** each private transaction ~ (8+16 B)/(12.5 Mbit/s) ≈ 15 µs; 21 reads ≈ 0.32 ms + turnaround ≈ 0.4 ms. Well within 5 ms. Even at 2 MHz it's ~2.5 ms — borderline; choose ≥4 MHz.
- **Top-level plan:** 1 broadcast write/tick (mode + common timestamp) + 21 unicast writes (setpoints) + 21 unicast reads (state), or combine command+readback per node in a **combined write-then-read** (repeated START) transaction to halve framing overhead → ~12 B/joint overhead. Target ~500 B/tick total. Reserve ~10% bus time for IBI/DAA/hotplug.

---

## 8. Consolidated recommendation (implementation order)

**Tier 1 (now, 200 Hz f32):** contact-Jacobian CoM estimator (§1.2), support-relative DCM balance + ankle/CMP distribution (§1.3–1.4), capturability-based ankle/hip/step switch (§2), CBF ZMP/torso clamps (§5), CPG clock + Raibert foot placement (§6), I3C 200 Hz exchange (§7). No dynamics model needed beyond leg Jacobians and a diagonal mass.

**Tier 2 (next):** DLS/null-space 4-level stack with a 6-DoF virtual floating base and contact task at priority 1 (§3), frozen $M_0$ at nominal pose → precomputed projectors. ~$10^4$ flops.

**Tier 3 (if budget allows):** N=6 f32 active-set LIPM-MPC at 10–20 Hz producing ZMP reference and step timing (§4); Scianca IS-MPC stability row.

**Requires host PC:** full CMM/WBIC QP (Kim et al. 2019), variable-height capturability MPC, nonlinear TO, learned policies.

**Key numbers to hard-code:** $m=1.5$ kg, $h=0.25$ m, $\omega=6.26$ rad/s, foot $(X,Y)=(0.035,0.018)$ m, $\mu\approx0.6$, $T_s=5$ ms, $k_p\approx1$–3, $k_i\approx5$, $k_d\approx0$, DCM error clamp 0.02 m (scaled from Caron's 0.05 m for HRP-4), ZMPCC offset clamp 0.01 m, CBF $\kappa=3$ s⁻¹, SLIP $k\approx670$ N/m, $L_0=0.22$ m, I3C ≥4 MHz.

---

## References (authors, title, venue, identifier)

- Vukobratović & Borovac, "Zero-moment point — thirty five years of its life", IJHR 2(1), 2004 (ZMP).
- Kajita et al., "A realtime pattern generator for biped walking", ICRA 2003, DOI 10.1109/ROBOT.2003.1241826; Kajita, *Introduction to Humanoid Robotics*, 2014.
- Pratt, Carff, Drakunov & Goswami, "Capture Point: A Step toward Humanoid Push Recovery", Humanoids 2006, DOI 10.1109/ICHR.2006.333745 (capture point).
- Englsberger, Ott & Albu-Schäffer, "Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion", IEEE T-RO 31(2), 2015, DOI 10.1109/TRO.2015.2405592.
- Caron, Escande, Lanari, Mallein, "Capturability-based Pattern Generation for Walking with Variable Height", IEEE T-RO 35(3), 2019, DOI 10.1109/TRO.2019.2923971, arXiv:1801.07022.
- Caron et al., "Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control", 2018, HAL hal-01875387; code: github.com/stephane-caron/lipm_walking_controller (archived 2019; Doxygen scaron.info/doc/lipm_walking_controller).
- Caron, Pham & Kheddar, "Multi-contact stabilization of a humanoid robot for realizing dynamic contact transitions on non-coplanar surfaces", 2019; contact wrench cone derivation HAL hal-02108449.
- Khadiv et al., "Walking Control Based on Step Timing Adaptation", IEEE T-RO 36(3), 2020, arXiv:1704.01271.
- Missura, Bennewitz & Behnke, "Capture Steps: Robust Walking for Humanoid Robots", IJHR 16(6), 2020, arXiv:2011.02793.
- Scianca, De Simone, Lanari & Oriolo, "MPC for Humanoid Gait Generation: Stability and Feasibility", IEEE T-RO 36(3), 2020, DOI 10.1109/TRO.2019.2958483, arXiv:1901.08505.
- Wieber, "Trajectory free linear model predictive control for stable walking in the presence of strong perturbations", Humanoids 2006.
- Khatib, "A unified approach for motion and force control of robot manipulators: the operational space formulation", IEEE JRA 3(1), 1987 (OSC) **[KB]**.
- Orin, Goswami & Lee, "Centroidal momentum matrix of a humanoid robot: Structure and properties", Autonomous Robots 34(3), 2013.
- Kuindersma et al., "Optimization-based locomotion planning, estimation, and control design for the Atlas humanoid robot", Autonomous Robots 40(3), 2016.
- Kim, Di Carlo, Chignoli, Kim, "High-speed dynamically diverse running of humanoid robots" (WBIC), 2019/2020; whole-body locomotion control (WBLC) arXiv:1901.08100.
- Di Carlo et al., "Dynamic Legged Locomotion with LIPM and Convex MPC", IROS 2018, DOI 10.1109/IROS.2018.8594448.
- Ames, Grizzle, Tabuada, "Control barrier function based quadratic programs for safety critical systems", IEEE TAC 62(8), 2017 **[KB]**; Ames et al., "Control Barrier Functions: Theory and Applications", ECC 2019 **[KB]**.
- Paredes & Hereid, "Safe Whole-Body Task Space Control for Humanoid Robots", arXiv:2311.08409 (2023).
- Khazoom, Gonzalez-Diaz, Ding & Kim, "Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions", arXiv:2207.00692 (2022).
- Ahn, Lee & Sentis, "Data-Efficient and Safe Learning for Humanoid Locomotion Aided by a Dynamic Balancing Model", IEEE RA-L 5(2), 2020, arXiv:1906.03812.
- Peng, Paredes, Castillo & Hereid, "Real-Time Safe Bipedal Robot Navigation using Linear Discrete Control Barrier Functions", arXiv:2411.03619 (2024).
- Dang et al., "HULK: Learning Whole-Body Forceful Loco-Manipulation for Humanoids", arXiv:2610.08970 (2026) — capture-point CBF.
- Blickhan, "The spring-mass model for running and hopping", J. Biomechanics 22, 1989; Full & Koditschek, "Templates and anchors: neuromechanical hypotheses of legged locomotion", J. Exp. Biology 202, 1999 (SLIP/template).
- Raibert, *Legged Robots That Balance*, MIT Press, 1986 **[KB]**.
- Saranli, "Approximate analytic solutions to the spring loaded inverted pendulum dynamics", 2010; Uyanık, Morgül & Saranli, "Approximate analytical solutions for the SLIP dynamics", IEEE T-RO 31(4), 2015.
- Chen, Wensing & Zhang, "Differential flatness of the spring loaded inverted pendulum", arXiv:1901.05840.
- MIPI Alliance, "I3C Bus Interface Specification v1.1.1" **[KB]** (12.5 MHz SDR, DAA, IBI, push-pull).
- Zhou et al., "Support-relative capture point / DCM", arXiv:2608.00500 (2026) — encoder+2-axis-IMU DCM without base velocity.
