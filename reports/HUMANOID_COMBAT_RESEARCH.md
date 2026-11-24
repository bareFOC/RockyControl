# Algorithms and Control Strategies for Humanoid Robot Combat / Fighting Competitions
### An extensive technical survey (literature through 2026) with a feasibility assessment for a 21‑DoF Rocky Mk1–class robot on an RP2350‑class MCU with a 2‑axis IMU

**Scope note.** "Combat" here means *sport* fighting (boxing, kickboxing, freestyle fighting, sumo/wrestling, robot soccer/boxing demos), not lethal or weaponized systems. Everything below concerns balance, impact, perception, and game-theoretic control of legged humanoids under deliberate physical contact. DARPA SubT is included only as a contrast case: it is a search-and-rescue autonomy competition, not a fighting competition, but it contributes relevant lessons on learned/teleoperated autonomy under degraded sensing and single-operator supervision.

---

## 0. Executive summary

1. **The field changed character in 2024–2026.** Until ~2023, humanoid combat was dominated by *teleoperation* (Unitree G1 at the 2025 Hangzhou "Mecha Fighting Series", RoboOne hobby robots since 2002). From 2025–2026, hierarchical *learning* systems with motion priors (RoboStriker, SLMP, HoloBoxing, RPG) demonstrated autonomous boxing with sim-to-real transfer on Unitree G1 / ENGINEAI platforms. The recurring architectural lesson: **never let a high-level strategic policy explore in raw joint-torque space** — collapse is immediate. All successful systems use a low-level tracking/whole-body controller and a compact latent/task-space high-level policy.

2. **The core physics is impulse + angular momentum management.** A punch is a momentum-transfer event: generate distal link velocity with whole-body coordination, then absorb the reaction impulse without breaking stance contacts. The cleanest formalisms are (a) resolved/centroidal momentum control (Kajita et al. 2003), (b) impact-aware QP with predicted state jumps and ZMP/CoP constraints (Wang, Tanguy, Gergondet, Kheddar 2020), and (c) zero-step capture-region / CoM-velocity-area balance criteria (Wang, Tanguy, Kheddar 2023), which are markedly less conservative than strict ZMP bounds.

3. **Push recovery is a mature, well-solved subproblem** for moderate disturbances: capture point/step recovery (Pratt et al. 2006; Missura & Behnke 2019), centroidal angular momentum (CAM) hip strategies (Schuller et al. 2021; Kim et al. 2022), and reduced-order MPC/robust H∞ style controllers (Yang et al. 2026 "Bracing for Impact"). These are directly reusable for combat impacts.

4. **Safety and aggression are in tension**; CBF/HJ-reachability formalisms provide the only composable guarantees ("safe aggression"), but they are computationally heavy and mostly demonstrated on mobile manipulators, not full humanoids in fights.

5. **For a small 21-DoF humanoid on RP2350 (dual M33 @150 MHz, 520 kB SRAM, no FPU guarantee for heavy math, 2-axis IMU):** full learned policies, online QP whole-body control, and vision pipelines are **not feasible onboard**. The practical architecture is **offline-authored scripted combat moves + a lightweight real-time balance overlay** (LIPM/capture-point + complementary-filter attitude from the 2-axis IMU + open-loop keyframe upper-body strikes with reactive abort/restore), with all heavy planning/tuning done offline on the host workstation. This maps directly onto the existing `rk-control` safety kernel, `rk-model` move studio, and the existing fall-recovery architecture in this repository.

---

## 1. Robot boxing / fighting hardware and control

### 1.1 RoboOne (Japan, 2002–present)

- **Core idea.** Hobbyist/amateur bipedal humanoid fighting: an autonomous "demonstration/qualification" stage followed by 1-vs-1 matches. Created by Terukazu Nishimura, inspired by Honda ASIMO and US combat robots but explicitly non-destructive.
  - Sources: [ROBO-ONE official rules](https://www.robo-one.com/en/abouts/view/rule); [Wikipedia ROBO-ONE](https://en.wikipedia.org/wiki/ROBO-ONE); [France24 "Japan's black belt robots" (2010)](https://observers.france24.com/en/20100323-japan-black-belt-robot-wars-robo-one); [Society of Robots Robo-One 2010 report](https://www.societyofrobots.com/robo-one_2010.shtml); [ITmedia 9th Robo-One report (2006)](https://www.itmedia.co.jp/news/article/0603/23/1060323006).
- **Rules → control priorities.** A fall after a valid attack is a "down"; 3 downs loses; stand up within a 10-count or lose; accidental falls ("slip") don't count; an "owaza" aerial throw counts as 2 downs. Weight classes introduced at Robo-One 11 (2007). Matches ~3 min. From 2004, attacks involving hand/high-kick contact are permitted if the robot recovers within 10 counts. From the 9th event (2006), the same "sacrifice throw" motion cannot score twice.
- **Control implication.** The decisive capability is **reliable fall detection + fast get-up**, more than punching power. Early robots "just standing" evolved into "sacrifice throws" because toppling the opponent is easier than delivering a clean strike.
- **Strengths:** extremely low barrier to entry, driven rapid improvement in hobby bipeds; rules reward robustness. **Weaknesses:** subjective judging, semi/manual radio control accepted historically, small robots (~40 cm, e.g. DARwIn-OP class). **Track record:** 100+ entrants per event by 2004–2006 (~126 at 6th event), continuous series to present.

### 1.2 Humanoid boxing events 2025–2026

- **World's first humanoid fighting tournament, Hangzhou, 25 May 2025** (CMG "Mecha Fighting Series" / "World Robot Tournament – Mecha Combat Arena"): four teams teleoperating Unitree G1 (1.32 m, 35 kg, 7-DoF arms); demonstration + 3×2 min rounds; punch = 1 point, kick = 3 points, fall = −5, fail to rise in 8 s = −10 and round ends. Moves demonstrated: straight punches, hooks, side kicks, aerial spin kicks, get-ups. Observed weakness: strikes often "soft pushes", occasional jabs into empty space, balance losses after kicks.
  - Sources: [Live Science (2025-06-18)](https://www.livescience.com/technology/robotics/china-pits-rival-humanoids-against-each-other-in-worlds-first-robot-boxing-tournament); [Asia Times (2025-05-27)](https://asiatimes.com/2025/05/china-stages-first-ever-humanoid-robot-kickboxing-match); [People's Daily (2025-05-26)](https://en.people.cn/n3/2025/0526/c90000-20319423.html); [Interesting Engineering](https://interestingengineering.com/innovation/robot-boxing-china-humanoids-wins).
- **World Humanoid Robot Games, Beijing 2025 & 2026.** 2026 edition: 666 teams, 2,056 robots, 51 events, 16 countries. Combat became official in 2026 with 3×2 min rounds, weight classes 40/58/80 kg, scoring mirroring human combat (effective strike 1 pt, sweep/high kick 3 pt, knockdown 6 pt if opponent rises within 10 s; failure loses the round). 2026 required full autonomy in soccer/dance/track and scored autonomy with a coefficient; kickboxing still allowed a human in the loop.
  - Sources: [Global Times 2026-08-26](https://www.globaltimes.cn/page/202608/1369121.shtml); [Robonomy WHG page](https://robonomy.net/pages/world-humanoid-robot-games); [Seoul Economic Daily 2026-08-23](https://en.sedaily.com/world/2026/08/23/humanoid-robots-box-sprint-past-usain-bolt-at-beijing-games); [Guardian 2025-08-15](https://www.theguardian.com/world/2025/aug/15/china-world-humanoid-robot-games-advances-limitations).
- **EngineAI / URKL professional league (2025–2026).** URKL V2.0.0 rules: one-on-one striking, registered EngineAI T800 platforms, 3×5 min rounds, Wi-Fi disabled, control software submitted for inspection, one 3-min technical timeout per round, registered reserves, KO by failure to continue.
  - Source: [RobotBoxe "How Does Robot Fighting Work?" (2026)](https://robotboxe.com/guides/how-robot-fighting-works).
- **Sony QRIO / SDR (historical).** Not a boxing robot per se, but the canonical small humanoid with integrated running/jumping, external-force adaptation, and fall recovery — a direct ancestor of the control stack a fighter needs. 0.58 m, ~7 kg, 38 DoF, 3×64-bit RISC, Aperios/OPEN-R; "Real-time Integrated Adaptive Motion Control" (SDR-4X, 2002), first autonomous bipedal running humanoid (2003). Demonstrated kicking a ball and getting back up after being pushed at CMU (2005).
  - Sources: [Nagasaka, "Sony QRIO", in Humanoid Robotics: A Reference, Springer 2019, pp. 187–200](https://www.cs.columbia.edu/~allen/S19/SonyQRIO.pdf); [Sony product milestones](https://www.sony.com/en/SonyInfo/CorporateInfo/History/sonyhistory-j.html); [CMU news 2005](https://www.cmu.edu/cmnews/extra/050128_qrio.html); [IEEE Spectrum "QRIO: The Robot That Could" (2004)](https://spectrum.ieee.org/qrio-the-robot-that-could); [Impress 2003 running announcement](https://pc.watch.impress.co.jp/docs/2003/1218/sony.htm).
- **"Robot Boxing League" / university boxing:** no single canonical academic league named "The Robot Boxing League" appears in the literature; the closest institutional lineage is (i) RoboOne/RoboGames, (ii) robot soccer (RoboCup humanoid league, FIRA), and (iii) the 2025–2026 Chinese events described above. RoboCup/FIRA are relevant because they drove fast vision-based bipedal motion; CMU won the RoboCup Legged league in 1998 and 2002 with QRIO-adjacent expertise (Veloso). [CMU source above.]

### 1.3 Control challenges unique to fighting

- **Dynamic punching:** accelerate distal mass quickly while the floating base is underactuated; the reaction impulse must be routed through the legs into the floor.
- **Balance under impact:** post-impact CoM velocity and centroidal angular momentum can jump in milliseconds; feet may break contact or slip.
- **Rapid direction changes:** require ankle/hip/step strategies and often a guarded, lowered-CoM stance.
- **Uncertainty:** opponent location, impact timing, contact normal, restitution, and compliance are all poorly known. This is why robust/worst-case formulations dominate.

---

## 2. Punching / striking dynamics

### 2.1 Resolved momentum control (Kajita, Harada et al.)

**Paper:** S. Kajita, F. Kanehiro, K. Kaneko, K. Fujiwara, K. Harada, K. Yokoi, H. Hirukawa, *"Resolved Momentum Control: Humanoid Motion Planning based on the Linear and Angular Momentum"*, IROS 2003, DOI 10.1109/IROS.2003.1248880. [IEEE](https://ieeexplore.ieee.org/document/1248880); [PDF](https://ieeexplore.ieee.org/iel5/8832/27959/01248880.pdf); Japanese journal version, JRSJ 22(6):772–779, 2004; [ROBOGAKU overview](https://robogaku.jp/en/history/locomotion/L-2004-2.html). 19th RSJ Best Paper Award 2005.

**Core idea.** Choose total linear momentum **P** and angular momentum **L** of the whole robot, then solve for base and joint velocities that realize them. With generalized coordinatesq = (base, joints), the total momentum is linear in velocities:

$$ \begin{bmatrix}\mathbf P\\ \mathbf L\end{bmatrix} = \mathbf{M}_P(\mathbf q)\,\dot{\mathbf q} $$

and for a given momentum reference, joint/base velocities follow by a weighted pseudo-inverse:

$$ \dot{\mathbf q} = \mathbf{M}_P^{\#}\begin{bmatrix}\mathbf P^{ref}\\ \mathbf L^{ref}\end{bmatrix} $$

with leg–ground contact constraints enforced in the same linear system. Rate-of-change relates to ground reaction:

$$ \dot{\mathbf P} = \sum \mathbf f_{ext} - m\mathbf g,\qquad \dot{\mathbf L}_{CoM} = \sum (\mathbf r_i - \mathbf c)\times \mathbf f_i + \boldsymbol\tau_i $$

**Striking interpretation.** To punch: prescribe a momentum trajectory with a short, high peak along the punch direction; the pseudo-inverse distributes it across the arm and the support leg(s), producing whole-body coordination automatically. Angular momentum reference can be chosen to cancel torso spin (arm counter-swing, hip counter-rotation).

**Strengths:** unified framework for kicks, punches, walking; demonstrated on HRP-2 real hardware. **Weaknesses:** momentum reference design is manual; no explicit contact switching or impact dynamics; instantaneous, near-zero contact velocity assumption. **Cost:** one pseudo-inverse of a small matrix — cheap enough for a 2000s PC, still too heavy for RP2350 with 21 DoF if done in floating point at 200 Hz.

### 2.2 Centroidal momentum convex formulation (Ponton, Herzog, Schaal, Righetti)

**Paper:** B. Ponton, A. Herzog, S. Schaal, L. Righetti, *"A convex model of humanoid momentum dynamics for multi-contact motion generation"*, Humanoids 2016. [IEEE](https://ieeexplore.ieee.org/document/7803371).

Convexifies the centroidal dynamics + contact constraints so multi-contact (e.g. punch while braced) trajectories can be optimized reliably. Directly relevant for "maximum impulse" planning.

### 2.3 Impact-aware whole-body QP (Wang, Tanguy, Gergondet, Kheddar)

**Papers:**
- Y. Wang, A. Tanguy, P. Gergondet, A. Kheddar, *"Impact-aware humanoid robot motion generation with a quadratic optimization controller"*, arXiv:2001.08454 (2020). [HTML](https://arxiv.org/html/2001.08454v2).
- Y. Wang, A. Tanguy, A. Kheddar, *"Impact-Aware Multi-Contact Balance Criteria"*, IEEE T‑Ro preprint arXiv:2308.06784 (2023). [HTML](https://arxiv.org/html/2308.06784).

**Core equations.**

Velocity jump through coefficient of restitution $c_r$ and contact normal projector $P_{\vec n}=\vec n\vec n^\top$:

$$ \Delta\dot{\mathbf x}=\dot{\mathbf x}^+-\dot{\mathbf x}^-=-(1+c_r)P_{\vec n}\dot{\mathbf x}^- $$

Predicted end-effector jump as a function of the QP decision variable $\ddot{\mathbf q}$:

$$ \Delta\dot{\mathbf x}_{k+1}=P_\Delta\big(\mathbf J_k\Delta t\,\ddot{\mathbf q}_k + \dot{\mathbf J}_k\Delta t^2\ddot{\mathbf q}_k + \mathbf J_k\dot{\mathbf q}_k + \dot{\mathbf J}_k\dot{\mathbf q}_k\Delta t\big) $$

Operational-space impulse propagation (articulated-body inertia $\Lambda^{-1}=\mathbf J\mathbf M^{-1}\mathbf J^\top$):

$$ \Delta\dot{\mathbf x}=\Lambda^{-1}\mathbf I,\qquad \mathbf J\,\Delta\dot{\mathbf q}=\Lambda^{-1}\mathbf I $$

These are wrapped in a least-norm equality QP solved analytically via a KKT system, giving linear maps $\Delta\dot{\mathbf q}^*=\mathbf K_{\Delta\dot q}^{-1}\Delta\dot{\mathbf x}_{m+1}$, $\mathbf I_i^*=\mathbf K_i^{-1}\Delta\dot{\mathbf x}_{m+1}$. Those maps become *linear inequality constraints* in the main whole-body QP:

- joint velocity bound: $\mathcal J_{\Delta\dot q}\ddot{\mathbf q}\Delta t \le \bar{\dot{\mathbf q}}-\dot{\mathbf q}-\mathcal J_{\Delta\dot q}\dot{\mathbf q}$, etc.
- impulsive joint torque: $\Delta\boldsymbol\tau = \frac{1}{\delta t}\big(\sum_i \mathbf J_i^\top \mathbf K_i^{-1}\big)\Delta\dot{\mathbf x}_{m+1}$, bounded analogously;
- holding contacts: CoP constraint $\mathbf A_{c2}\mathcal J_f\ddot{\mathbf q}\frac{\Delta t}{\delta t}\le-\mathbf A_c\mathbf F-\mathbf A_{c2}\mathcal J_f\dot{\mathbf q}\frac1{\delta t}$, friction cone via $P_\mu=\mathbf I-\vec n\vec n^\top-\mu\vec n\vec n^\top$;
- ZMP bound: $\mathbf A_Z\sum_i(\mathbf A_i\mathcal J_{f_i})\ddot{\mathbf q}\frac{\Delta t}{\delta t}\le-\mathbf A_Z\big(\sum_i\mathbf F_i+\frac1{\delta t}\sum_i(\mathbf A_i\mathcal J_{f_i})\dot{\mathbf q}\big)$.

**No reset maps, no hybrid switching, no knowledge of impact timing/location needed** — only worst-case upper bounds. HRP-4 results: hit a concrete wall at 0.35 m/s with peak impulsive force ~130 N; strict ZMP bound forced 0.11 m/s. Box-grabbing at 0.15 m/s, post-impact force regulated to 60 N with admittance.

The follow-up 2023 paper shows that **strict ZMP is over-conservative for strikes**: ZMP temporarily left the support polygon at 0.345 m/s yet HRP-4 did not fall, because post-impact CoM velocity (−0.16 m/s) stayed inside the zero-step capture region. It defines **CoM velocity area** $\mathcal X_{[\dot c_{xy}]}$ from the contact wrench cone + torque limits + LIPM assumptions, with boundary

$$ \dot{\mathbf c}_{xy}=w\,\mathbf z,\quad w=\sqrt{g/c_z} $$

and for the stable standing region $\underline z_x\le c_x+\dot c_x/w\le\bar z_x$. With $c_z=0.78$ m and $z\in[-0.13,0.13]$ m: $\dot c_x\in[-0.4608,0.4608]$ m/s. Then it restricts the entire predicted post-impact CoM velocity set (a convex polyhedron) inside that area.

**Strengths:** rigorous, real-robot validated, less conservative than ZMP. **Weaknesses:** 3D frictional impact is not analytically determinable (needs numerical integration), assumes high-stiffness kinematic control and known sustained contacts/geometry; impact duration estimation (Hertz stiffness) remains hard. **Cost:** two nested QPs (predictor + controller), ray-shooting projection — workstation-grade, not MCU-grade.

### 2.4 Other striking works

- **Y. Wang et al., "Maximum Impulse Approach to Soccer Kicking for Humanoid Robots," arXiv:2412.01480 (2024)** — analytic 4-phase kick with constant-acceleration trajectories; kick duration modulates step frequency; integrated with ZMP gait; validated on NimbRo-OP2X. [arXiv](https://arxiv.org/abs/2412.01480).
- **D. Marew, N. Perera, S. Yu, S. Roelker, D. Kim, "A Biomechanics-Inspired Approach to Soccer Kicking for Humanoid Robots," Humanoids 2024** — 25-DoF PresToe, 7-DoF legs with actuated toe, instep kicks >11 m/s in full dynamics simulation. [IEEE](https://ieeexplore.ieee.org/document/10769964).
- **S.-J. Yi, B.-T. Zhang, D. Hong, D. D. Lee, "Active Stabilization of a Humanoid Robot for Impact Motions with Unknown Reaction Forces," 2012** — treats unknown punch reaction as a disturbance; combined whole-body torso trajectory + biomechanically motivated ankle/hip/step push recovery; validated in Webots and on DARwIn-OP (45 cm, 2.8 kg, 100 Hz, Dynamixel position servos). **This is the closest prior art to a small humanoid punching with a minimalist controller.** [PDF](https://romela.org/wp-content/uploads/2015/05/Active-Stabilization-of-a-Humanoid-Robot-for-Impact-Motions-with-Unknown-Reaction-Forces.pdf).
- **HITTER (2025)** — hierarchical model-based planner + RL whole-body controller for humanoid table tennis, 106 consecutive rallies with a human; coordinated waist rotation during strikes. [arXiv:2508.21043](https://arxiv.org/html/2508.21043v2).
- **High-speed impact-resilient teleoperation (2024)** — IHMC Nadia, momentum-based QP WBC + cycloidal actuators; VR punching into a punching bag; ~70 ms operator-to-pose latency. [arXiv:2409.04639](https://arxiv.org/html/2409.04639).
- **H2O (2024)** — RGB-camera real-time whole-body teleoperation including boxing, with RL tracker zero-shot sim-to-real. [arXiv:2403.04436](https://arxiv.org/html/2403.04436).

---

## 3. Balance under external impact / push recovery

### 3.1 Capture point (Pratt, Carff, Drakunov, Goswami)

**Paper:** J. Pratt, J. Carff, S. Drakunov, A. Goswami, *"Capture Point: A Step Toward Humanoid Push Recovery"*, 6th IEEE-RAS Humanoids, 2006, DOI 10.1109/ICHR.2006.321385. [IEEE](https://ieeexplore.ieee.org/document/4115602).

**Core idea / equations.** Extend LIPM with a flywheel body. The capture point is where the CoM would come to rest if the support were moved there instantly:

$$ \xi = \mathbf c + \frac{\dot{\mathbf c}}{\omega},\qquad \omega=\sqrt{g/c_z} $$

LIPM dynamics $\ddot{\mathbf c}=\omega^2(\mathbf c-\mathbf z)$; the **capture region** is the set of foot placements from which the robot can stop. Adding rotational inertia (flywheel = swinging arms/torso) enlarges the capture region substantially — the human hip/ankle strategies in miniature. Intersection of capture region and base of support decides ankle vs. step strategy.

**Strengths:** principled "when and where to step". **Weaknesses:** point-mass approximation; needs step execution; flywheel extension adds complexity. **Cost:** tiny — closed form, ideal for embedded.

### 3.2 Instantaneous capture point in momentum control (Koolen & Pratt)

**Paper:** T. Koolen, J. Pratt, *"Exploiting instantaneous capture point dynamics in momentum-based control"*, 2013. [PDF](https://www.cmu.edu/dynamic-walking/files/abstracts/Koolen_2013_DW.pdf). ICP trajectories give a physically achievable $\dot{\mathbf P}$ with CMP inside the support polygon; leftover CoP–CMP offset can serve posture control.

### 3.3 Capture steps (Missura & Behnke)

**Paper:** M. Missura, S. Behnke, *"Capture Steps: Robust Walking for Humanoid Robots"*, IJHR 2020. [PDF](https://www.ais.uni-bonn.de/papers/IJHR_2020_Missura.pdf).

**Notable for small robots:** torso IMU + joint encoders only — **no force/torque sensors, no precise model, only approximate link lengths**, compliant low-gain position-controlled actuators. 58 pushes without controller → 22 falls; 97 pushes with controller → 18 falls (stronger pushes applied). One of the strongest push-recovery results for humanoids despite minimal sensing — **highly relevant to a 2-axis IMU robot**.

### 3.4 Centroidal angular momentum (CAM) hip strategy

- **Schuller, Mesesan, Englsberger, Lee, Ott, "Online Centroidal Angular Momentum Reference Generation and Motion Optimization for Humanoid Push Recovery," IEEE RA‑L 6(3):5689, 2021.** [PDF](https://elib.dlr.de/144076/2/Online_Centroidal_Angular_Momentum_Reference_Generation_and_Motion_Optimization_for_Humanoid_Push_Recovery.pdf). TORO: pushes up to 65 N / 18 N·s (0.227 m/s) rejected by bending forward and swinging arms back; motion optimizer trades CAM tracking vs. posture with kinematic limits.
- **Kim, Lim, Park, Park, "Humanoid Balance Control using Centroidal Angular Momentum based on Hierarchical Quadratic Programming," IROS 2022.** [IEEE](https://ieeexplore.ieee.org/document/9981036). HQP prioritizes CAM tracking over return-to-pose within a capture-point feedback framework.
- **Chang, Hsu, Cheng, Huang, "Humanoid Robot Push-Recovery Strategy Based on CMP Criterion and Angular Momentum Regulation," 2015.** [PDF](https://www.chinganc.com/docs/chang2015humanoid.pdf). CoM angular momentum as impedance system + Kalman CoM estimator + CMP stepping; CoG momentum decayed to zero in 3–4 s after 3.5 kg bag pushes (<80 N·m).
- **Stephens, "Push Recovery Control for Force-Controlled Humanoid Robots," CMU PhD 2011.** [PDF](https://www.cs.cmu.edu/~bstephe1/papers/thesis.pdf). PR-MPC with capture-point objective; notes hip strategy generates momentum that must be removed quickly; $\dot{\mathbf H}_{des}=-K_p\mathbf H$ dissipates it.
- **Yang, Werner, Ghansah, Ames, "Bracing for Impact: Robust Humanoid Push Recovery and Locomotion with Reduced Order Models," arXiv:2505.11495 (v1 May 2025; v2 June 2026)** — reduced-order robust control for bracing. [arXiv](https://arxiv.org/html/2505.11495v2).
- **Xie et al., "Dynamic Balancing of Humanoid Robot with Proprioceptive Actuation," Micromachines 13(9):1458, 2022** — Walker3 (1.6 m, 43 kg, 12 DoF, IMU + foot CoP, EtherCAT), hierarchical WBC with RBDL; 5 kg ball impulses on torso. [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC9500612/).
- **Castano et al., "Benchmarking Dynamic Balancing Controllers for Humanoid Robots," Robotics 11:114, 2022** — CoM stabilizer, CP controller, attitude controller on REEM-C; useful comparison of simple vs. complex. [PDF](https://burjcdigital.urjc.es/server/api/core/bitstreams/21bde903-35c6-473d-8f27-b2c357b94bd9/content).
- **FPGA balance control (Liu et al., Appl. Sci. 10:2699, 2020)** — evidence that even small FPGAs/MCUs can run dedicated balance loops. [MDPI](https://www.mdpi.com/2076-3417/10/8/2699).

**Takeaway for combat:** combine (i) ankle/CoP strategy for small taps, (ii) CAM arm/torso swing for medium hits, (iii) capture-point step for large hits, (iv) fall detection + get-up for the rest. Missura & Behnke shows (i)–(iii) work without force sensors.

---

## 4. Whole-body impact-aware control (hybrid dynamics, contact-aware, task-space)

Covered substantially in §2.2–2.3. Additional points:

- **Continuous-impact vs hybrid.** Wang et al. explicitly avoid reset maps/hybrid control by adding impact-aware inequality constraints to a continuous-time QP. The alternative literature uses hybrid zero dynamics (HZD) for walking; these require precise event detection and are brittle for unprepared combat impacts.
- **Task-space impact control:** operational-space inertia $\Lambda^{-1}=\mathbf J\mathbf M^{-1}\mathbf J^\top$ is the central object; impulse at one end-effector propagates to all others through its off-diagonal blocks.
- **3D frictional impacts:** Routh's graphical approach is exact in 2D; in 3D, tangential impulses require numerical integration (Stronge 2000; Stewart & Trinkle lineage), which is incompatible with an online WBC — hence worst-case bounding is preferred.
- **Impact-aware bimanual catching** (Yan, Stouraitis, Moura, Xu, arXiv:2403.17249, 2024) uses compliance models and indirect force control for >3.5 m/s objects; $\Delta\mathbf v=\boldsymbol\Lambda/M$, $\Delta\boldsymbol\omega=\mathbf I^{-1}\mathbf r^\times\boldsymbol\Lambda$. [HTML](https://arxiv.org/html/2403.17249v1).

---

## 5. Reinforcement learning for combat / aggressive behaviors

### 5.1 RoboSumo lineage

**Paper:** M. Al-Shedivat, T. Bansal, Y. Burda, I. Sutskever, I. Mordatch, P. Abbeel, *"Continuous Adaptation via Meta-Learning in Nonstationary and Competitive Environments"*, ICLR 2018. [arXiv:1710.03641](https://arxiv.org/html/1710.03641); [OpenReview](https://openreview.net/forum?id=Sk2u1g-0-); [code](https://github.com/openai/robosumo).
- Introduced **RoboSumo**: MuJoCo 1-vs-1 wrestling with Ant/Bug/Spider morphologies; observations include own positions/velocities, opponent positions, contact forces ("tactile"); continuous actions; **iterated adaptation games** with policy updates between rounds; meta-learners dominate reactive baselines; population evolves by TrueSkill.
- **Related:** T. Bansal, J. Pachocki, S. Sidor, I. Sutskever, I. Mordatch, *"Emergent Complexity via Multi-Agent Competition"*, ICLR 2018 [arXiv:1710.03748](https://arxiv.org/pdf/1710.03748) — competitive self-play produces behaviors far more complex than the environment. A. Grover et al., *"Learning Policy Representations in Multiagent Systems"*, ICML 2018 [PMLR](https://proceedings.mlr.press/v80/grover18a/grover18a.pdf) — opponent embeddings as privileged info improve win rates in RoboSumo.
- **Important caveat:** the user's mention of "Bansal et al. 2018 — From imagination to reality: learning goal-directed locomotion and pushing" conflates two threads. There is a 2018 Google Brain paper *"From Imagination to Reality: Deep Reinforcement Learning for Robotic Manipulation"* context and Tan et al.'s *"Sim-to-Real: Learning Agile Locomotion For Quadruped Robots"* (RSS 2018, [arXiv:1804.10332](https://arxiv.org/html/1804.10332)) which uses domain randomization, perturbation forces, compact observation space, actuator modeling and latency handling. The RoboSumo competition environment itself is Al-Shedivat et al. 2018. Both lineages matter: RoboSumo for adversarial self-play, Tan et al. for sim-to-real.

### 5.2 Hierarchical latent-space combat (the 2026 state of the art)

- **RoboStriker — three versions in 2026:**
  - *Hierarchical Decision-Making for Autonomous Humanoid Boxing*, arXiv:2601.22517 (Jan 2026), ICML-listed. [HTML](https://arxiv.org/html/2601.22517v1).
  - *Robust Policy Gating ...* companion lineage.
  - *Latent-Space Strategic Games for Autonomous Humanoid Boxing*, arXiv:2608.16195 (Aug 2026). [HTML](https://arxiv.org/html/2608.16195).
  - Authors: K. Yin, K. Liu, Z. Cao, W. Dong, W. Zeng, T. Zhang, Q. Zhang, J. Wang, J. Pang, Y. Li, M. Zhou, W. Zhang (SJTU / Shanghai AI Lab).
  - **Pipeline:** (1) single-agent motion tracker trained on 46 Xsens clips ≈14 min of professional boxing (mirrored augmentation) in Isaac Lab; (2) distill skills into a spherical/topologically bounded latent manifold (Gaussian distribution projected onto unit hypersphere); (3) **Latent-Space Neural Fictitious Self-Play (LS-NFSP)** with AMP warmup; two-player zero-sum Markov game with approximate-Nash interpretation over decoder-reachable actions. Deployed on real humanoid robots; wins/striking efficiency exceed raw-action-space baselines; catastrophic balance failures greatly reduced.
- **SLMP — Spherical Latent Motion Prior**, J. Tan et al., HKUST-GZ / Oxford, arXiv:2603.01294 (2026). [HTML](https://arxiv.org/html/2603.01294v1). Two-hour human combat motion-capture dataset (502 clips); distillation + discriminator + local semantic consistency; avoids VAE information loss and AMP mode collapse; two-agent combat with simple rule-based rewards; validated on Unitree G1 and ENGINEAI PM01 models in simulation (human-like physically plausible combat). ~8 h combat-policy training post-tracker on 2× RTX 4090 (per HoloBoxing's comparison table).
- **HoloBoxing — 7 minutes of single-GPU training**, X. Ma, B. Zhang, B. Li, Y. Bao, Z. Liu, Y. Wang (Horizon Robotics), arXiv:2610.32909 (Oct 2026). [HTML](https://arxiv.org/html/2610.32909v2); [code](https://github.com/HorizonRobotics/HoloMotion/tree/master/holoboxing). Freeze a pretrained HoloMotion v2 whole-body controller; train only a compact planner that emits 32-dim actions → 225 future body-point targets; measured-state imitation initializes it before RL. ~7 min additional training (excluding controller pretraining); simulation-only, no real-robot transfer yet. Demonstrates that once motor capability exists, *task* adaptation is cheap.
- **RPG — Robust Policy Gating**, Y. Xin et al., Tsinghua/Shanghai AI Lab, arXiv:2604.21355 (2026). [HTML](https://arxiv.org/html/2604.21355v2). Smooth multi-skill transitions for humanoid fighting; validated on Unitree G1 hardware. Addresses the "same button doesn't give the same move" problem seen in teleoperated fights.

### 5.3 Adjacent competitive RL with real robots

- **Haarnoja et al., "Learning Agile Soccer Skills for a Bipedal Robot with Deep RL," Science Robotics 9, adi8022 (2024).** [arXiv:2304.13653](https://arxiv.org/html/2304.13653); [Science](https://www.science.org/doi/10.1126/scirobotics.adi8022). Two-stage teacher distillation → self-play with a snapshot pool (first quarter only, plus untrained agent) → zero-shot transfer to Robotis OP3. 0.69 m/s walk vs 0.27 scripted; get-up 0.9 s vs 2.5 s; penalty success 70% real vs 90% sim. Critic conditioned on opponent ID to handle partial observability. **Direct precedent for small humanoids.**
- **Abeyruwan et al., "i-Sim2Real: RL of Robotic Policies in Tight Human-Robot Interaction Loops," CoRL 2023** — alternating sim/real refinement of both human model and policy via Blackbox Gradient Sensing; table tennis rallies 22 avg / 150 best. [PMLR 205](https://proceedings.mlr.press/v205/abeyruwan23a.html).
- **Humanoid-Gym, arXiv:2404.05695 (2024)** — Isaac Gym → MuJoCo sim-to-sim → zero-shot real transfer on XBot-S/L. [HTML](https://arxiv.org/html/2404.05695v2).
- **"Bridging the Reality Gap," IEEE RAM 32(1):49–58 (2025)** — systematic ablation of dynamics randomization, state history, noise/bias/delay, state selection, perturbations, network size on one platform. [IEEE](https://ieeexplore.ieee.org/document/10795482).
- **Air hockey self-play (Orsula, arXiv:2406.00518, 2024)** — shows non-self-play agents overfit to the baseline (14.5:0.1 vs baseline) but lose 0.7:14.3 to self-play agents. [HTML](https://arxiv.org/html/2406.00518).
- **"Ready-to-React" (ICLR 2025)** — diffusion predictor + online decoder for two-character boxing reaction generation; reduces error accumulation. [PDF](https://proceedings.iclr.cc/paper_files/paper/2025/file/cbd2851fe02a9d5d46278a53ede53e99-Paper-Conference.pdf).

**Strengths/weaknesses/cost:** RL gives emergent tactics and rapid adaptation but requires GPU-scale training (hours; ~8 h for SLMP, minutes for HoloBoxing *given a pretrained controller*), careful reward shaping, domain randomization, and a safety fallback. Sim-to-real of contact-rich fighting remains fragile — jabs into empty air and falls were common even in 2026 events.

---

## 6. Game-theoretic / adversarial control

- **Minimax / zero-sum Markov games:** RoboStriker formalizes combat as a two-player zero-sum Markov game in latent action space with approximate-Nash interpretation under regularity + approximate best-response assumptions. Classical NFSP (Heinrich et al. 2015) and PSRO (Lanctot et al. 2017) underpin snapshot-pool self-play.
- **HJ reachability:** *"Refining Control Barrier Functions through Hamilton-Jacobi Reachability"*, Tonkens & Herbert, arXiv:2204.12507 (2022) — refineCBF uses dynamic programming to turn candidate CBFs into valid, less conservative CBFs with monotonic safety guarantees. [HTML](https://arxiv.org/html/2204.12507v2). HJ reachability computes optimal safety value functions for adversarial opponents; game-theoretic HJ in *belief space* appears in *"Deception Game"*, arXiv:2309.01267 (2023). [HTML](https://arxiv.org/html/2309.01267v2).
- **Adversarially robust CBFs:** Garg et al., *"Advances in the Theory of CBFs"*, Annual Reviews in Control 57:100945 (2024) — sampled-data, high-relative-degree, adversarial inputs; sufficient invariance condition splits normal vs adversarial agents:

$$ \sum_{j\in\mathcal A}\big(L_{f_j}h_j + L_{g_j}h_j u_j^{\max} + L_{\phi_j}h_j\big) + \sum_{i\in\mathcal N}\big(L_{f_i}h_i + L_{g_i}h_i u_i + L_{\phi_i}h_i\big) \le -\alpha(h) $$

  [PDF](https://par.nsf.gov/servlets/purl/10569324). Also *"Multirobot Adversarial Resilience Using CBFs"*, IEEE T‑Ro 2024 [ACM](https://dl.acm.org/doi/10.1109/TRO.2023.3341570); FT‑NCBF for sensor attacks, arXiv:2402.18677 [HTML](https://arxiv.org/html/2402.18677).
- **Humanoid CBF-WBC:** Khazoom, Gonzalez-Diaz, Ding, Kim, *"Humanoid Self-Collision Avoidance Using WBC with CBFs"*, Humanoids 2022. CBF-QP: $u^*=\arg\min_u\frac12\|u-k(x)\|^2$ s.t. $\dot h\ge-\alpha_1 h$; integrates with LMPC and a centroidal angular momentum task. [HTML](https://arxiv.org/html/2207.00692v2); [MIT DSpace](https://dspace.mit.edu/entities/publication/1208949f-4e4a-4bf4-b909-fd2f83aa762a). Also Paredes & Hereid, *"Safe Whole-Body Task Space Control for Humanoid Robots"*, arXiv:2311.08409 (ECBF formulation). [PDF](https://arxiv.org/pdf/2311.08409).

**Practical reading:** CBFs are the right tool for "hit hard but never step outside the ring / never exceed joint limits / never let CoM velocity leave capture region" — i.e. **safe aggression filters** wrapped around an aggressive nominal controller. Cost: a small QP per cycle; feasible on a host PC, marginal on RP2350. A scalar CBF on CoM-velocity (capture-point) or torso tilt can be implemented analytically on an MCU.

---

## 7. Perception and reaction for combat

- **Human pose estimation from RGB:** H2O (2024) uses off-the-shelf 3D human pose estimator + RL tracker for real-time whole-body teleoperation including boxing. [arXiv:2403.04436](https://arxiv.org/html/2403.04436).
- **Punch anticipation:** *"Punch Anticipation in a Karate Combat with Computer Vision"*, ACM 2021 — OpenCV motion filter reduced human response time 0.925 s → 0.859 s and improved punch-side prediction 64.6% → 82.9%. [ACM](https://dl.acm.org/doi/fullHtml/10.1145/3450614.3461688).
- **Biomechanics of anticipation:** Limballe et al., *"Virtual reality boxing: gaze-contingent blur..."*, Frontiers in Sports and Active Living 6:1430719 (2024) — elite boxers gaze at trunk/head and use peripheral low-spatial-frequency motion cues; anticipation is more important than raw reaction speed. [Frontiers](https://www.frontiersin.org/journals/sports-and-active-living/articles/10.3389/fspor.2024.1430719/full).
- **Vision latency:** IHMC teleoperation reports ~70 ms operator-input-to-robot-pose; ping pong used to stress the full perception-action loop. [arXiv:2409.04639](https://arxiv.org/html/2409.04639).
- **Scoring/analysis CV:** Compubox Reimagined (CNN hit detection + YOLO-Pose + LSTM punch classification, 97% train accuracy) [PDF](https://shangar.ddns.net/media/papers/Compubox_3.pdf); real-time combat-sport HAR (Quinn & Corcoran, 2022) [PDF](https://papers.academic-conferences.org/index.php/icair/article/download/930/912/3029).
- **Ready-to-React** (ICLR 2025) provides a diffusion-based opponent-motion predictor suitable for anticipatory defense. [PDF](https://proceedings.iclr.cc/paper_files/paper/2025/file/cbd2851fe02a9d5d46278a53ede53e99-Paper-Conference.pdf).

**Takeaway:** at combat distances, sub-100 ms perception-to-action is essential; a 2-axis IMU robot cannot do vision onboard — use an external host/camera or hardwire opponent detection (IR/beacon/contact switch), or fight on pre-programmed timing.

---

## 8. Compliant / impedance control for impact

- **Hogan (1984)** impedance control — regulate dynamic relation between trajectory and interaction force: $Z_{imp}=-\tau_{ext}/\dot q$ (Laplace). SEAs convert force control into position control via spring deflection.
- **Pratt & Williamson (1995) SEA** — "stiffness isn't everything"; spring filters shock, measures torque, lowers backdrive impedance. 25 of 44 DoF of NASA Valkyrie are SEAs; chosen because DRC tasks required compliant interaction rather than high-gain rigid control. [Valkyrie actuator control, 2014](https://sites.utexas.edu/hcrl/files/2016/01/jfr-valkyrie-actuator-control-final.pdf); [SEA review 2024](https://ieeexplore.ieee.org/document/10668290).
- **Variable impedance actuators (VIACTORS)** — Albu-Schäffer et al., *"Variable Impedance Actuators: Moving the Robots of Tomorrow"*, IROS 2012. [PDF](https://elib.dlr.de/80680/1/1944.pdf). Applications explicitly include **striking and impact response** (hammering fingers, striking with a baseball on the arm), energy storage for kicks/throws, robustness to impacts. DLR VS-Joint: 2× kick speed vs stiff joint (6.35 vs 3.06 m/s); QA-Joint stiffness change 3%→97% in 0.15 s. DLR David/neoDavid use VSAs throughout arms/hands; VUB VERONICA uses MACCEPA. [Humanoid.guide VSA report 2026](https://humanoid.guide/variable-stiffness-actuators-humanoid-robots).
- **Active impedance control overview**, PMC 2018 — cascade outer impedance loop over SEA; reduced torque bandwidth is the main drawback. [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC6211161).
- **Key timing result:** Wang et al. note that even a VSA needs >10 ms to generate torque counterbalancing impulsive torques — **hardware absorbs the impact instant; controllers act before/after**. Hence compliance is a hardware design decision, not a software fix for a rigid robot.

**For combat:** compliance is a double-edged sword — a soft joint stores/releases energy for faster strikes but reduces positional bandwidth and punch accuracy. A small hobby humanoid with position servos cannot render variable impedance; emulate compliance in software (low-gain/velocity-limited deceleration after contact) or add mechanical padding.

---

## 9. Stability during attacks (ZMP/CoP, angular momentum compensation)

- **ZMP/CMP/LIPM** fundamentals as in §2–3. ZMP: $z_x=-\sum\tau_y/\sum f_n$, $z_y=\sum\tau_x/\sum f_n$ (co-planar); CoP analogous per contact. CMP: ground reference where $\dot L=0$.
- **Angular momentum compensation:** arm swing, torso counter-rotation, hip strategy. Schuller et al. generate CAM by bending forward and arms back; Kim et al. prioritize CAM tracking in HQP; Stephens sets $\dot H_{des}=-K_p H$ to dissipate. RoboStriker/HITTER implicitly learn waist rotation during strikes.
- **"Dynamic balance during whole-body striking":** Kajita's resolved momentum control handles kicking on HRP-2; Yi et al. 2012 explicitly uses "exaggerated weight shifts" to impart impact forces and then stabilizes via push-recovery; Wang et al. 2023 provides the CoM-velocity-area criterion for non-coplanar contacts.
- **Rule-of-thumb:** keep CoM low and inside support polygon during wind-up; drive angular momentum toward zero over a strike cycle (the punch's +L must be cancelled by legs/torso −L); after impact catch CoM velocity within $-w z_{max}\le\dot c\le w z_{max}$ or step.

---

## 10. Rule sets and design trade-offs of humanoid battle competitions

| Competition | Format | Scoring | Key control priority |
|---|---|---|---|
| **RoboOne** (2002–) | Biped, 3 min, 3 downs; 10-count get-up; "owaza" throw = 2 downs; weight classes since 2007 | Downs | Fall detection + fast get-up; stable sacrifice throws |
| **CMG Mecha Fighting / Hangzhou 2025** | Unitree G1, teleop, 3×2 min | Punch 1, kick 3, fall −5, no rise in 8 s −10 | Latency, full-body coordination, self-balancing after kicks |
| **World Humanoid Robot Games 2026** | 40/58/80 kg, 3×2 min, teleop or autonomous (autonomy scored) | Strike 1, sweep/high kick 3, knockdown 6, 10-count | Autonomy coefficient pushes toward learned policies; robustness to falls |
| **URKL / EngineAI T800 (2025–26)** | 3×5 min, Wi-Fi off, inspected software, 1 timeout/round, reserves | KO / rounds | Certified deterministic control, hardware endurance |
| **RoboCup Humanoid / FIRA** | Soccer, 2v2/3v3, autonomy required | Goals | Fast vision, gait stability, team coordination — transferred to combat |
| **DARPA SubT (2019–21)** | Multi-robot search, single human supervisor, degraded comms/perception | Artifacts found | Autonomy under uncertainty, human-supervisor handoff — *not combat*, but a model for teleop→autonomy transition |

**Sources:** RoboOne rules [official](https://www.robo-one.com/en/abouts/view/rule); Hangzhou scoring [Asia Times](https://asiatimes.com/2025/05/china-stages-first-ever-humanoid-robot-kickboxing-match); WHG 2026 [Robonomy](https://robonomy.net/pages/world-humanoid-robot-games), [Global Times](https://www.globaltimes.cn/page/202608/1369121.shtml); URKL [RobotBoxe](https://robotboxe.com/guides/how-robot-fighting-works); SubT [JPL NeBula](https://ai.jpl.nasa.gov/public/projects/subT), [CERBERUS Science Robotics 2022](https://www.science.org/doi/full/10.1126/scirobotics.abp9742), [THI human-robot teams 2024](https://dl.acm.org/doi/full/10.1145/3665319).

**How rules shape control:** point systems favoring high kicks reward *large-impulse, high-risk* moves; knockdown bonuses reward *recovery*; 10-counts reward get-up sequences; autonomy coefficients reward *learned policies*; weight classes cap achievable impulse (a 21-DoF small robot should fight in a lighter class where its relative momentum is competitive).

---

## 11. Comparative table

| Approach | Core idea | Key equations/concepts | Strengths | Weaknesses | Compute cost | Real-robot track record |
|---|---|---|---|---|---|---|
| **Resolved momentum control** (Kajita 2003) | Specify P, L; solve for joint/base velocities | $\mathbf P,\mathbf L=\mathbf M_P\dot q$; pseudo-inverse | Unified kicks/walking; elegant | Manual momentum refs; no impact model; near-zero contact velocity | Low (small pseudoinverse) | HRP-2 kicks/walking |
| **Impact-aware QP** (Wang 2020) | Predict state jumps; add worst-case constraints to WBC | $\Delta\dot x=-(1+c_r)P_n\dot x^-$; $\Delta\dot x=\Lambda^{-1}I$; CoP/friction/ZMP inequalities | No reset maps; max contact velocity with guaranteed feasibility | Nested QP; assumes known impact bounds; conservative ZMP | High (2 QPs @ 100+ Hz) | HRP-4 wall impact 0.35 m/s, 133 N impulse |
| **CoM-velocity-area / capture region** (Wang 2023) | Bound entire post-impact CoM velocity set inside zero-step capture region | $\dot c=w z$, $w=\sqrt{g/c_z}$; contact wrench cone | Far less conservative than ZMP; handles non-coplanar contacts | Kinematic-controlled assumption; 3D friction needs numerics | Medium-high (ray shooting) | HRP-4 ramp push recovery, wall impact |
| **Capture point / steps** (Pratt 2006) | Step to $\xi=c+\dot c/\omega$ | LIPM + flywheel; capture region | Closed form; principled when-to-step | Approximate model | **Very low** | Sims + many humanoids |
| **Capture steps** (Missura 2019) | Analytic capture step, no F/T sensors | Torso IMU + joint angles only; compliant actuators | Minimal sensing, strong results | Less precise on slippery terrain | **Very low** | DARwIn/OP2 class, 97 pushes |
| **CAM hip strategy** (Schuller 2021, Kim 2022) | Generate/refuse CAM to reject pushes | $\dot L$ from push torque; HQP priority | Works where stepping impossible | Must dissipate momentum; torque limits | Medium | TORO 65 N / 18 N·s; DYROS-JET |
| **Push-recovery MPC / robust** (Stephens 2011, Yang 2026) | PR-MPC with capture objective; reduced-order robust | $J=w_1\|X_{CAP}-X_{CAP}^{goal}\|^2+\dots$ | Optimal foot placement | Model heavy | High | Sims + force-controlled humanoids |
| **CBF / HJ safety filter** (Khazoom 2022, Garg 2024, Tonkens 2022) | Forward-invariant safe set; minimally modify nominal input | $\dot h\ge-\alpha h$; CBF-QP; HJ value iteration | Formal safety with aggressive nominal policy | Conservative; synthesis hard; heavy | Medium-high | Mobile robots, humanoid self-collision (sim) |
| **Adversarial self-play RL** (RoboSumo 2018, Emergent Complexity 2018) | Population vs population, meta-adaptation | Zero-sum Markov game; PPO; TrueSkill | Emergent complex tactics | Simulation-only, morphology-specific | Training: GPU-hours; inference: small MLP | No real robot (MuJoCo) |
| **Latent-space MARL boxing** (RoboStriker 2026, SLMP 2026) | Track human mocap → latent prior → latent NFSP | Spherical latent manifold; NFSP; AMP warmup | Real robot boxing; avoids joint-space collapse | Long training pipeline; mocap dataset needed | Training: hours, 1–2 GPUs; inference: 2 MLPs | Unitree G1 / ENGINEAI PM01 (sim + real for RoboStriker) |
| **Frozen-controller task adaptation** (HoloBoxing 2026) | Train only compact planner over frozen WBC | 32-D action → 225 body-point targets; imitation init | ~7 min extra training | Sim-only; relies on pretrained controller | Tiny at inference | Simulation only |
| **Policy gating** (RPG 2026) | Smooth switching among learned fighting skills | Gate network + motion transitions | Robust multi-action fighting on hardware | Needs skill library | Low-medium | Unitree G1 real |
| **Teleoperation + WBC** (IHMC 2024, H2O 2024) | Human provides intent, robot WBC executes | Momentum QP; RGB pose estimator; prediction | High-quality strikes today | Human latency; not autonomous | Host PC required | Nadia punching bag; H2O boxing |
| **Compliant / SEA / VSA** (Pratt 1995, Albu-Schäffer 2012) | Hardware compliance absorbs/stores impact | $Z_{imp}$; spring deflection as torque sense | Shock tolerance, explosive strikes | Reduced bandwidth, extra mass | N/A (hardware) | Valkyrie, DLR David/neoDavid |
| **Scripted keyframes + balance overlay** (RoboOne-era, this repo's fall recovery) | Precomputed motions + reactive stabilizer | LIPM/IMU tilt+rate; fuzzy/PD/CP | Deterministic, MCU-feasible, testable | No tactical adaptation; brittle to novel opponents | **Very low** | Thousands of RoboOne matches; DARwIn-OP punching |

---

## 12. What's practical for a small 21-DoF humanoid (Rocky Mk1 style) on RP2350-class MCU with 2-axis IMU

### 12.1 Hardware budget reality

- **RP2350:** dual Cortex-M33 (or Hazard3) @150 MHz, **520 kB SRAM**, 8 kB OTP, up to 16 MB QSPI flash/PSRAM. No hardware double-precision FPU; a 21-DoF floating-base operational-space inertia matrix + KKT solve at 200 Hz is out of reach. A compact LIPM/attitude controller at 100–200 Hz is comfortable. [RP2350 datasheet](https://pip.raspberrypi.com/documents/RP-008373-DS-2-rp2350-datasheet.pdf).
- **2-axis IMU:** gives pitch/roll-ish tilt and rate (no yaw). Enough for sagittal/lateral balance state estimation (complementary filter, $\theta\approx\arctan(a_x/a_z)$ fused with gyro), **not** for 3D momentum/footstep planning or vision-free opponent localization.
- **Repository context:** `rocky_mk1.urdf` ships a 12-DoF fighting humanoid sample; existing safety kernel has Teach/Drive/EStop, bumpless transfer, soft limits, slew clamping, 200 ms deadman heartbeat, and open-loop keyframe fall recovery with four poses. All host-side tooling (URDF, `k::Chain` FK, move studio, SIL, deterministic Copper replay) is on the desktop/WASM app, not the MCU. This is the right division of labor.

### 12.2 Feasible now (on-MCU)

1. **Scripted combat moves with a balance overlay** — the recommended primary architecture.
   - Offline author jab / cross / hook / body punch / low kick / guard / sidestep / duck keyframes in the host Move Studio; export joint-space trajectories with velocity/acceleration limits.
   - On MCU, overlay a **reactive balance controller**:
     - Complementary-filter pitch/roll from the 2-axis IMU (like Yi 2012 and Missura 2019 — no F/T sensors needed).
     - LIPM-based CoM estimate in sagittal/lateral planes; capture point $\xi=c+\dot c/\omega$; keep $\xi$ within support polygon by ankle/hip offsets and, if available, a single reactive step.
     - Scalar CBF-like guard: abort/soften a strike if $|\theta|>\theta_{max}$ or $|\dot\theta|>\omega_{max}$ or if $\xi$ leaves a safety margin.
     - Angular-momentum compensation baked into keyframes: torso counter-rotation and arm counter-swing on every punch; this is the cheapest way to get Kajita-style coordination without runtime optimization.
   - Post-impact: detect anomaly via IMU tilt-rate spike; switch to a precomputed "recover guard" keyframe, reuse existing fall-recovery state machine. This is exactly the Yi et al. (2012) approach on a DARwIn-OP-class robot.
   - **Computational cost:** a few thousand FMAs per control tick; 100–200 Hz is realistic in `no_std` Rust.

2. **Lightweight "combat state machine"** on the host, not MCU: guard → approach → jab combo → retreat → recover. Game-theoretic/adversarial reasoning is unnecessary at this scale; a finite-state machine with a few opponent-distance thresholds captures most of the value of RoboStriker-style tactics without the training pipeline.

3. **Opponent sensing:** *not* onboard vision. Use an external camera + host pose estimator (H2O-style) or simple IR/ultrasonic range + manual teleop (gamepad path already exists via `gilrs`). The host sends high-level intents; the MCU executes scripted moves with balance overlay.

### 12.3 Feasible as a hybrid (host + MCU)

- **Offline trajectory optimization with impact-aware constraints, executed open-loop on MCU.** Use the host workstation (Rust `rk-sim`, URDF, `k::Chain`) to compute momentum-consistent striking trajectories with analytically bounded impulses (Wang-style constraints), then stream joint trajectories. This captures much of the benefit of whole-body impact-aware control without running QP onboard.
- **Deterministic Copper replay verification** already exists in this repo — ideal for validating that a combat move never violates soft limits/slew rates before flashing.
- **Distillation of a small MLP policy** trained in simulation on the host (~32 inputs → 21 outputs) *could* run on RP2350 via fixed-point/int8 inference (tens of kB, <1 ms), but only if paired with the hard safety overlay; pure RL policies without a tracker prior collapse in raw joint space (RoboStriker's central finding). Given a 2-axis IMU and no vision, such a policy would be blind to the opponent — recommend **not** pursuing until sensing improves.

### 12.4 Not feasible onboard (do not attempt)

- Online whole-body QP with impulse prediction (§2.3), HJ reachability, CBF-QP over full dynamics.
- Self-play MARL training (obviously), but also *inference* of transformer/large latent-space decoders with real-time constraints.
- Vision-based punch anticipation, YOLO pose, diffusion response predictors — host or external compute only.
- True variable impedance striking — requires hardware SEA/VSA redesign.

### 12.5 Recommended feature set, prioritized

| Priority | Feature | Method | Where |
|---|---|---|---|
| P0 | Fall detection + reliable get-up | Existing latch + 4 keyframes + IMU dwell | MCU + host |
| P0 | Safe guard stance with lowered CoM | Static keyframe, soft limits | MCU |
| P0 | Jab / cross with counter-rotation | Keyframes + IMU tilt abort overlay | MCU |
| P1 | Hook / body punch / low kick with capture-point recovery | Offline momentum-aware trajectories, keyframe execution | Host authoring, MCU exec |
| P1 | Reactive step / ankle strategy on medium hits | Capture point $\xi=c+\dot c/\omega$, 2-axis IMU | MCU |
| P2 | Teleoperated combos with host-side state machine | Gamepad → host → intent → MCU | Host |
| P2 | Impact-aware trajectory optimization offline | Wang-style QP on host, export | Host |
| P3 | Small distilled policy + latent prior | RoboStriker/HoloBoxing recipe, sim-trained | Host train, MCU int8 infer |
| P3 | CBF safety shield on CoM velocity/tilt | Scalar analytic barrier | MCU |
| — | Onboard vision, online WBC QP, self-play | Not feasible | — |

**Bottom line:** For Rocky Mk1 on RP2350 with a 2-axis IMU, the winning architecture is **scripted strikes + a rigorous lightweight balance/recovery overlay**, authored and validated with the existing desktop/WASM toolchain, with optional host-side teleoperation and offline impact-aware optimization. This is the same recipe that produced thousands of competitive RoboOne robots and the DARwIn-OP punching results, and it is the only one whose compute and sensing fit the bill. Learned latent-space boxing (RoboStriker/SLMP/HoloBoxing) is the research frontier and a sensible P3 if the platform later gains a host computer, depth/vision, foot force sensing, and a richer IMU.

---

## 13. Citation verification note

Web search and the arXiv API were intermittently rate-limited in this environment. All arXiv IDs cited above were therefore verified by fetching their `/abs/` pages directly (titles, first authors, submission dates). Verified results:

| ID | Verified title (first author, date) | Used in report |
|---|---|---|
| 2601.22517 | RoboStriker: Hierarchical Decision-Making for Autonomous Humanoid Boxing (Yin, 2026-01-30) | ✅ |
| 2608.16195 | RoboStriker: Latent-Space Strategic Games for Autonomous Humanoid Boxing (Yin, 2026-08-17) | ✅ |
| 2610.32909 | HoloBoxing: A Simple Recipe for Humanoid Boxing with 7 Minutes of Single-GPU Training (Ma, 2026-10-23) | ✅ |
| 2603.01294 | Spherical Latent Motion Prior for Physics-Based Simulated Humanoid Control (Tan, 2026-03-01) | ✅ |
| 2604.21355 | RPG: Robust Policy Gating for Smooth Multi-Skill Transitions in Humanoid Fighting (Xin, 2026-04-23) | ✅ |
| 2505.11495 | Bracing for Impact: Robust Humanoid Push Recovery and Locomotion with Reduced Order Models (Yang, v1 2025-05-16, v2 2026) | ✅ (date corrected) |
| 2409.04639 | High-Speed and Impact Resilient Teleoperation of Humanoid Robots (Bertrand, 2024-09-06) | ✅ |
| 2403.04436 | Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation (He, 2024-03-07) | ✅ |
| 2508.21043 | HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning (Su, 2025-08-28) | ✅ |
| 2304.13653 | Learning Agile Soccer Skills for a Bipedal Robot with Deep RL (Haarnoja, 2023-04-26) | ✅ |
| 2311.08409 | Safe Whole-Body Task Space Control for Humanoid Robots (Paredes, 2023-10-12) | ✅ |
| 2207.00692 | Humanoid Self-Collision Avoidance Using WBC with CBFs (Khazoom, 2022-07-01) | ✅ |
| 2001.08454 / 2308.06784 | Impact-aware humanoid motion generation with a QP controller (Wang, 2020) / Impact-Aware Multi-Contact Balance Criteria (Wang, 2023) | ✅ |
| 2412.01480 / 2403.17249 | Maximum Impulse Approach to Soccer Kicking (Ficht, 2024) / Impact-Aware Bimanual Catching (Yan, 2024) | ✅ (author list corrected) |
| 1710.03641 / 1710.03748 | Continuous Adaptation via Meta-Learning... (Al-Shedivat, 2017) / Emergent Complexity via Multi-Agent Competition (Bansal, 2017) | ✅ (ICLR 2018 publications; arXiv 2017 preprints) |
| 1804.10332 | Sim-to-Real: Learning Agile Locomotion For Quadruped Robots (Tan, 2018) | ✅ |
| 2512.21219 / 2406.00518 / 2204.12507 / 2309.01267 | ESP32-C3 CoP feedback / Air hockey self-play / refineCBF / Deception Game | ✅ |

**Corrections made:** (a) "Bracing for Impact" v1 date is May 2025 (v2 2026); (b) the bimanual catching paper's author list is Yan, Stouraitis, Moura, Xu, not only "Yan & Xu"; (c) a reference-list entry seen in RPG (`arXiv:2507.08303`) is actually **"Keep on Going: Learning Robust Humanoid Motion Skills via Selective Adversarial Training" (Zhang et al., 2025)** — *not* "Learning Robust Motion Skills via Critical Adversarial Attacks for Humanoid Robots" as paraphrased in RPG's bibliography; this paper is not relied upon in the report's conclusions.

**Not independently verified (flagged):** exact quotation of the user-supplied title *"From imagination to reality: learning goal-directed locomotion and pushing"* — no paper with that exact title was found; §5.1 explains the likely conflation with RoboSumo (Al-Shedivat et al. 2018) and Sim-to-Real agile locomotion (Tan et al. 2018). Event scoring details in §1.2 and §10 come from news sources (Live Science, Asia Times, Global Times, Robonomy, RobotBoxe, People's Daily) that were fetched directly; they are secondary sources, not official rule PDFs.

---

## 14. Reference index (URLs)

**Competitions / hardware**
- RoboOne rules: https://www.robo-one.com/en/abouts/view/rule ; history: https://en.wikipedia.org/wiki/ROBO-ONE ; 2010 report: https://www.societyofrobots.com/robo-one_2010.shtml ; 2010 France24: https://observers.france24.com/en/20100323-japan-black-belt-robot-wars-robo-one
- Hangzhou 2025 fighting: https://www.livescience.com/technology/robotics/china-pits-rival-humanoids-against-each-other-in-worlds-first-robot-boxing-tournament ; https://asiatimes.com/2025/05/china-stages-first-ever-humanoid-robot-kickboxing-match ; https://en.people.cn/n3/2025/0526/c90000-20319423.html
- WHG 2026: https://www.globaltimes.cn/page/202608/1369121.shtml ; https://robonomy.net/pages/world-humanoid-robot-games ; https://en.sedaily.com/world/2026/08/23/humanoid-robots-box-sprint-past-usain-bolt-at-beijing-games
- URKL rules: https://robotboxe.com/guides/how-robot-fighting-works
- Sony QRIO: https://www.cs.columbia.edu/~allen/S19/SonyQRIO.pdf ; https://www.sony.com/en/SonyInfo/CorporateInfo/History/sonyhistory-j.html ; https://www.cmu.edu/cmnews/extra/050128_qrio.html ; https://spectrum.ieee.org/qrio-the-robot-that-could
- DARPA SubT: https://ai.jpl.nasa.gov/public/projects/subT ; https://www.science.org/doi/full/10.1126/scirobotics.abp9742 ; https://www.cs.cmu.edu/news/2022/link-subt-lessons

**Momentum / impact / balance**
- Kajita et al. 2003: https://ieeexplore.ieee.org/document/1248880 ; https://ieeexplore.ieee.org/iel5/8832/27959/01248880.pdf ; https://robogaku.jp/en/history/locomotion/L-2004-2.html
- Ponton et al. 2016: https://ieeexplore.ieee.org/document/7803371
- Wang et al. 2020: https://arxiv.org/html/2001.08454v2
- Wang, Tanguy, Kheddar 2023: https://arxiv.org/html/2308.06784
- Pratt et al. 2006 capture point: https://ieeexplore.ieee.org/document/4115602
- Koolen & Pratt 2013: https://www.cmu.edu/dynamic-walking/files/abstracts/Koolen_2013_DW.pdf
- Missura & Behnke 2019: https://www.ais.uni-bonn.de/papers/IJHR_2020_Missura.pdf
- Schuller et al. 2021: https://elib.dlr.de/144076/2/Online_Centroidal_Angular_Momentum_Reference_Generation_and_Motion_Optimization_for_Humanoid_Push_Recovery.pdf
- Kim et al. 2022: https://ieeexplore.ieee.org/document/9981036
- Chang et al. 2015: https://www.chinganc.com/docs/chang2015humanoid.pdf
- Stephens 2011: https://www.cs.cmu.edu/~bstephe1/papers/thesis.pdf
- Yang et al. 2026: https://arxiv.org/html/2505.11495v2
- Yi et al. 2012: https://romela.org/wp-content/uploads/2015/05/Active-Stabilization-of-a-Humanoid-Robot-for-Impact-Motions-with-Unknown-Reaction-Forces.pdf
- Ficht & Behnke 2024: https://arxiv.org/abs/2412.01480 ; Marew et al. 2024: https://ieeexplore.ieee.org/document/10769964
- IHMC teleop 2024: https://arxiv.org/html/2409.04639 ; H2O 2024: https://arxiv.org/html/2403.04436 ; HITTER 2025: https://arxiv.org/html/2508.21043v2
- Yan & Xu 2024 catching: https://arxiv.org/html/2403.17249v1
- Xie et al. 2022 Walker3: https://pmc.ncbi.nlm.nih.gov/articles/PMC9500612/ ; Castano et al. 2022: https://burjcdigital.urjc.es/server/api/core/bitstreams/21bde903-35c6-473d-8f27-b2c357b94bd9/content

**RL / games / safety**
- Al-Shedivat et al. 2018 RoboSumo: https://arxiv.org/html/1710.03641 ; code https://github.com/openai/robosumo
- Bansal et al. 2018 emergent complexity: https://arxiv.org/pdf/1710.03748
- Grover et al. 2018 policy representations: https://proceedings.mlr.press/v80/grover18a/grover18a.pdf
- Tan et al. 2018 sim-to-real: https://arxiv.org/html/1804.10332
- Haarnoja et al. 2024 soccer: https://arxiv.org/html/2304.13653 ; https://www.science.org/doi/10.1126/scirobotics.adi8022
- i-Sim2Real 2023: https://proceedings.mlr.press/v205/abeyruwan23a.html
- Humanoid-Gym 2024: https://arxiv.org/html/2404.05695v2 ; reality-gap ablation 2025: https://ieeexplore.ieee.org/document/10795482
- RoboStriker 2026: https://arxiv.org/html/2601.22517v1 ; https://arxiv.org/html/2608.16195
- SLMP 2026: https://arxiv.org/html/2603.01294v1 ; HoloBoxing 2026: https://arxiv.org/html/2610.32909v2 ; RPG 2026: https://arxiv.org/html/2604.21355v2
- Air hockey self-play 2024: https://arxiv.org/html/2406.00518 ; Ready-to-React 2025: https://proceedings.iclr.cc/paper_files/paper/2025/file/cbd2851fe02a9d5d46278a53ede53e99-Paper-Conference.pdf
- CBF survey Garg et al. 2024: https://par.nsf.gov/servlets/purl/10569324 ; refineCBF 2022: https://arxiv.org/html/2204.12507v2 ; Deception Game 2023: https://arxiv.org/html/2309.01267v2 ; Khazoom et al. 2022: https://arxiv.org/html/2207.00692v2 ; Paredes & Hereid 2023: https://arxiv.org/pdf/2311.08409

**Perception / compliance / MCU**
- Punch anticipation: https://dl.acm.org/doi/fullHtml/10.1145/3450614.3461688 ; VR boxing gaze: https://www.frontiersin.org/journals/sports-and-active-living/articles/10.3389/fspor.2024.1430719/full ; Compubox: https://shangar.ddns.net/media/papers/Compubox_3.pdf
- VIACTORS: https://elib.dlr.de/80680/1/1944.pdf ; VSA report 2026: https://humanoid.guide/variable-stiffness-actuators-humanoid-robots ; Valkyrie SEA: https://sites.utexas.edu/hcrl/files/2016/01/jfr-valkyrie-actuator-control-final.pdf ; impedance overview: https://pmc.ncbi.nlm.nih.gov/articles/PMC6211161
- RP2350 datasheet: https://pip.raspberrypi.com/documents/RP-008373-DS-2-rp2350-datasheet.pdf ; ESP32-C3 CoP balance 2025: https://arxiv.org/html/2512.21219v1 ; FPGA balance 2020: https://www.mdpi.com/2076-3417/10/8/2699
