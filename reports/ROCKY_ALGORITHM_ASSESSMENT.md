# Rocky Mk1 — Detailed Algorithm & Architecture Assessment
### Control feasibility, firmware integration, and staged roadmap for a 45 cm torque-controlled humanoid

**Status:** refined through all hardware clarifications and a full technical critique (2026-11-24).
**Companion documents:** `HUMANOID_*_RESEARCH_REPORT.md`, `DYNAMIC_LOCOMOTION_RESEARCH_REPORT.md` (literature); `IMPLEMENTATION_CONTROL_RESEARCH.md`, `REDUCED_ORDER_WBC_IMPLEMENTATION.md` (equations/implementation); `LSM6DSV` IMU research (this revision).

## Critique response log — all issues accepted and fixed

| Critique | Resolution in this document |
|---|---|
| v1 N20 gears make current-GRF / observer / active impedance physically impossible | §6 split: **P1a = kinematic DCM + UVC on v1**; torque/observer stack is **P1b = v2 only** (§4.2c, 4.3, 4.5 marked v2-only) |
| Gains 10–30× too large for 1–2 kg | §4.3 rescaled: $k_p\approx1\!-\!5$ N·m/rad, $k_d\approx0.05\!-\!0.25$ N·m·s/rad |
| Observer threshold 5–15 N·m too large | §4.2c rescaled to **0.1–0.3 N·m** (and noted infeasible on v1) |
| Momentum observer sign flip (double-counted contact) | §4.2c corrected to consistent $+A^\top\lambda$ / $+\hat\lambda$ sign |
| 3.5 cm foot geometry ambiguity | §4.4 clarified: 7 cm total foot, 3.5 cm half-length, ±2.8 cm authority |
| Task-space formula conflated velocity-IK with acceleration/torque WBC | §4.5 rewritten with correct null-space recursion and explicit velocity-vs-acceleration distinction |
| Pipeline cross-references shifted | §4.1 diagram renumbered |
| 20 cm SLIP hop energetically impossible / timing contradiction | §4.8 reduced to **2–4 cm hop**, $T_s\approx120$–150 ms, short debounce, ≤100 ms landing ramp; v2 only |
| I3C harness 50 pF limit | User confirmed harness **already validated on hardware**; note kept on overlapped transfers + per-joint timeouts |
| BNO055 100 Hz stale ticks / double filtering / magnetometer | §6.5: **LSM6DSV16X** on dedicated SPI @960 Hz as primary; BNO055 demoted to heading reference |
| Wire protocol no version/CRC | §2.5 hardening: `protocol_version`, CRC16/32, per-joint torque/velocity clamps |
| FLOP budget skewed both ways | §7 corrected: filter ~100 flops; full v2 stack 50k–120k flops/tick (1–2 ms) with non-blocking I3C |
| ESP32-P4 maturity | Kept as secondary BSP track behind RP2350 stability |
| UVC lesson (small geared humanoids) | Adopted as the v1 strategy in §6 (P1a), citing Dr. Guero's near-identical 40 cm platform |

**Decisions confirmed by user:** custom current loop exists on each N20; I3C harness already validated on hardware; split P1 into P1a/P1b; LSM6DSV proposed as IMU replacement/augmentation.

### Round 2 critique — all issues accepted and fixed

| Critique | Resolution |
|---|---|
| `ImuState.gravity` from accel LPF lags falls by 0.5–1 s | §6.5: gravity computed **from complementary-filter attitude with zero lag** ($g_{body}$ formula) |
| Wrong WHO_AM_I (0x6B/0x6C) and 960→200 Hz aliasing | §6.5: **WHO_AM_I = 0x70** (verified against ST datasheet + ArduPilot/PX4); FS bits in CTRL6/CTRL8; **FIFO burst-read ~5 samples/tick** or gyro LPF1 |
| Momentum observer missing `+r` in integral → drifts | §4.2c: `+r(τ)` added inside integral; **backward-Euler** form (unconditionally stable) |
| $\hat\lambda$ circularity | §4.2c: explicit two-stage pipeline — observer runs with $\hat\lambda=0$, then contact-mode projection of $r$ |
| UVC and kinematic DCM fight over hip/stride DOFs | §6 P1a item 4: strict partitioning — UVC = stance verticality I-only; ankle = gyro-rate P; DCM = swing-foot placement only; ukemi relaxation |
| No current-limit semantics for v1 position mode | §3: in Position/Velocity mode `torque_nm[i]` is the per-joint **saturation limit** (v1 compliance); Torque mode = v2 |
| Copper `Copy` stack growth | §3: `size_of` unit test + stack high-water check added to P1a |
| Telemetry validity + CRC placement | §2.5: `valid_mask` bits; CRC over raw payload **before COBS**, checked after; bad frames dropped/counted |
| BNO055 yaw still unreliable | §6.5: BNO055 **removed from control path**; gyro-Z integration for turning |
| Stale §1 rows, broken §2.6 ref, §4.5 notation, §4.8 ramp, §8 typo | All fixed (see §1 table, §4.2d→§6.5, §4.5 corrected recursion, §4.8 25–40 ms ramp, §8 rewritten) |

---

## 1. Confirmed platform

| Item | Value |
|---|---|
| Form factor | 1/4-scale Dummy 13 humanoid, **~45 cm**, **1–2 kg**, 3D printed, flat feet with functional toes |
| DoF | 21 (J_WAIST_YAW … R_ANKLE_ROLL), `MAX_JOINTS = 24` |
| Actuators | **v1: N20 geared brushed with custom per-joint current loop**; **v2: geared BLDC** |
| Servo electronics | **v1: custom N20 current loop** (software current limiting for compliance; no full torque cascade through gearbox); **v2: SimpleFOC full torque → velocity → position cascade** with position/velocity/current readback |
| Servo bus | **Shared I3C bus** — **v1 RP2350 uses a PIO-based I3C driver (already work done, harness validated)**; ESP32-P4 has native I3C (unconfigured in scaffold) |
| IMU | **v1 primary: ST LSM6DSV16X** 6-axis (dedicated SPI1, 960 Hz raw gyro+accel, DRDY INT1); BNO055 9-DoF (I2C0, 100 Hz fused) demoted to slow heading reference only |
| MCU v1 | **RP2350** (Cortex-M33F @ ~150 MHz, hard-float `thumbv8m.main-none-eabihf`), 512 KB SRAM (64 KiB cu29 heap), 2 MiB flash configured; USB CDC-ACM or CDC-NCM transport at 200 Hz |
| MCU v2 | **ESP32-P4** (400 MHz RV32IMAFC, single-precision FPU), native I3C, USB-OTG HS intended; **scaffold only** today (96 KiB heap, no PSRAM/flash config, no USB/WD/LED/servo bus wired) |
| Host PC | Rocky Control app — **offline only** (training, trajectory optimization). **No camera at runtime.** |
| Contact sensing | **v1: CPG phase + IMU impact/step detection** (no force estimation through N20 gears); **v2: current-based GRF + momentum observer (BLDC)** |
| Control rate | **200 Hz** (5 ms) on both MCUs; 100 ms host deadman; 250 ms HW watchdog (RP2350 implemented, ESP32-P4 TODO) |

**Decision taken:** raise `MAX_COBS_FRAME_BYTES` from 512 → **1024** so telemetry can carry per-joint velocities, a full estimator block, and debug data.

---

## 2. Current firmware baseline (what the new stack must integrate with)

### 2.1 Shared firmware core — `firmware/rk-fw-core/`
- `config.rs`: `CONTROL_LOOP_HZ = 200`, `CONTROL_PERIOD_MS = 5`, `CONTROL_DT_SEC = 0.005`, `HW_WATCHDOG_TIMEOUT_MS = 250`, `DEFAULT_JOINT_COUNT = 21`; USB VID `0x1209`, PIDs `0x524B` (CDC) / `0x524E` (NCM); MCU IP `10.42.0.2`, host `10.42.0.1`, control UDP **7777**, model UDP **7778**.
- `copper.rs`:
  - `HostCmdMsg` (L48–77), `ServoFeedbackMsg` (L200–218), `ControlStepMsg` (L264–304) — `Clone+Copy+Default+Serialize+Deserialize+Encode+Decode+Reflect`.
  - `DirectServoCuTask` — `CuTask` with `Input = input_msg!('m, HostCmdMsg, ServoFeedbackMsg)`, `Output = output_msg!(ControlStepMsg)`; calls `kernel.step(now_ms, dt, cmd, fb)` (L424), derives LED mode, fills telemetry.
  - `HostLinkBridge` — `rx_cmd`/`tx_telem` channels, publishes `to_telemetry()` to `HOST_LINK_MAILBOX`.
  - `ServoBusBridge` — `rx_state` / `tx_cmd`; `send` calls `SERVO_BUS_SLOT.write_command(&step_msg.to_servo_cmd())`; `receive` calls `SERVO_BUS_SLOT.read_feedback(dt)` and **overwrites `fb.imu` with `IMU_SLOT.latest()`**.
- `imu.rs`: `IMU_SLOT: ImuSlot` (`Mutex<RefCell<ImuSnapshot>>`); `publish` validates all 7 IMU floats finite; `latest()` returns `None` until first valid sample.
- `servo_bus.rs` — **the I3C plug-in point**:
  ```rust
  pub struct ServoBusHooks {
      pub read_feedback: fn(f32) -> RawServoFeedback,
      pub write_command: fn(&RawServoCommand),
  }
  pub struct ServoBusSlot { bench: Mutex<RefCell<BenchLoopbackBus>>, hooks: Mutex<RefCell<Option<ServoBusHooks>>> }
  pub static SERVO_BUS_SLOT: ServoBusSlot = ServoBusSlot::new();
  ```
  Board drivers register hooks exactly as the UART stub does (`servo_uart.rs` L85–88). **The I3C SimpleFOC driver registers here — no Copper graph change needed.**
- `transport.rs`: `CobsStreamFramer`, `HostLinkMailbox` (latest-wins), framing helpers.
- `status_led.rs`: `StatusLedMode::{TeachHeartbeat, DriveSolid, FaultStrobe}`.

### 2.2 RP2350 BSP — `firmware/rk-rp2350/`
- `main.rs`: `#[copper_runtime(config = "../copperconfig.ron")] struct RockyRp2350App`; 64 KiB buddy heap; `embassy_rp::init(Default::default())` (~150 MHz); GPIO25 LED; `Ticker::every(5 ms)` drives `running_app.run_one_iteration()`; **error → log + skip watchdog feed**; HW watchdog 250 ms fed after clean iteration.
- Peripherals: USB (CDC-ACM default / CDC-NCM alt), I2C0 @ 400 kHz for BNO055 (GPIO4 SDA / GPIO5 SCL), UART0 @ 1 Mbps placeholder servo (GPIO0/1), GPIO25 LED, WATCHDOG.
- `imu_bno055.rs`: 100 Hz task, gravity registers `0x2E` → quaternion `0x20` → Euler fallbacks; publishes to `IMU_SLOT`.
- `servo_uart.rs`: **stub** — cached feedback, 8-byte placeholder command. Replaced by the PIO I3C SimpleFOC driver.
- `.cargo/config.toml`: `thumbv8m.main-none-eabihf`, runner `probe-rs run --chip RP235x`; `memory.x`: RAM 512 KB @ `0x20000000`, FLASH 2 MiB @ `0x10000000`.

### 2.3 ESP32-P4 BSP — `firmware/rk-esp32p4/` (scaffold)
- `main.rs`: `#[esp_hal_embassy::main]`, `esp_alloc::heap_allocator!(size: 96*1024)`, `esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()))`, TIMG0 as embassy time source only.
- **TODOs (verbatim, main.rs L62–67):**
  1. Spawn embassy-usb CDC-ACM/NCM HS driver pushing to `HOST_LINK_MAILBOX` / draining telemetry at 200 Hz.
  2. Arm `timg0.wdt` hardware watchdog (250 ms) and feed after each Copper iteration.
  3. Drive status LED GPIO from `current_led_mode().is_lit(now_ms)`.
- **Not present:** native I3C peripheral config, servo bus hooks, PSRAM/flash config, partition table. Target `riscv32imafc-unknown-none-elf`, runner `espflash flash --monitor`.
- **Migration principle:** share 100% of `rk-proto`, `rk-control`, `rk-fw-core`, and `copperconfig.ron`; only the BSP differs. The PIO I3C driver written for RP2350 must be reimplemented with the ESP32-P4 I3C peripheral (or reused via an external bridge) for v2.

### 2.4 Copper graph — `firmware/copperconfig.ron`
One task (`servo_ctrl` = `DirectServoCuTask`, `joint_count: 21`), two bridges (`host_link`, `servo_bus`), four connections. **No rate declarations** — 200 Hz imposed by the Embassy ticker. A new estimator/controller should live **inside `DirectServoKernel::step` (rk-control)** and expose results via telemetry, not as a new Copper task — preserving the graph, both MCUs, and cu29 reflection.

### 2.5 Wire budget — `rk-proto`
`MAX_COBS_FRAME_BYTES` 512 → **1024** (decision). Current `McuToHost` ≈ 350 B raw + COBS ≈ 355–365 B; with 1024 B we add:
- `joint_velocity_rad_s: [f32; 24]` ≈ 96 B
- `EstimatorTelemetry` (CoM pos/vel, GRF L/R, contact mask, ZMP, momentum) ≈ 82–85 B
- debug headroom for ~500 B more.
All COBS buffers (`usb_cdc.rs`, `usb_net.rs`, `transport.rs`, host decoders) must be updated in lockstep. **HARDENING REQUIRED (part of P1a):** the protocol has no version/CRC handshake. Once `HostCmdMsg` carries direct torque commands, a mismatched build or corrupted packet can command violent full-scale torque. Add:
1. A **2-byte `protocol_version`** in both frame types with a compile-time constant checked on decode;
2. **CRC-16/CCITT-FALSE or CRC-32 (IEEE) computed over the raw payload before COBS encoding** and checked after COBS decode. Bad frames are **dropped + counted**, not allowed to starve the 250 ms watchdog on a single bit flip — the 100 ms host deadman handles sustained dropouts;
3. **Explicit physical range clamping** per joint ($|\tau|\le\tau_{max}$, $|\dot q|\le\dot q_{max}$ from calibration) beyond `is_finite()`;
4. `EstimatorTelemetry` gains a **`valid_mask: u32`** (bits: `COM`, `GRF`, `MOMENTUM`, `LSM6DSV_OK`, `BNO055_OK`) so host logging never confuses v1's disabled GRF (`0.0 N`) with free-fall (`0.0 N`).
`decode_mcu_cobs` validation must cover all new finite floats.

---

## 3. Critical hardware/actuation gaps and their fixes

| Gap | Impact | Fix (part of P1) |
|---|---|---|
| `RawServoCommand` has only `torque_mask` + `raw_target_rad` — no torque/velocity command | Cannot use SimpleFOC torque/velocity cascade from MCU | Extend to `command_mode: u8` per joint + `torque_nm: [f32;24]` + `velocity_rad_s: [f32;24]`. **Per-mode contract:** in `Position`/`Velocity` mode, `torque_nm[i]` is the **per-joint current/torque saturation limit** (clamped to `[0, tau_max_i]`) — this is how v1 gets dynamic compliance from its custom N20 current loop without a torque command; in `Torque` mode (v2) it is the commanded torque. Propagate through `HostCmdMsg`/`ControlStepMsg`/`to_servo_cmd()` and the I3C driver. |
| `RawServoFeedback` has no joint velocity | Momentum observer / WBC damping impossible | Add `velocity_rad_s: [f32;24]` from SimpleFOC readback. (Also in telemetry per 1024 B decision.) |
| No I3C in `rk-fw-core` abstractions | Driver cannot plug in cleanly | Add `i3c_simplefoc` board module registering `ServoBusHooks`; PIO I3C on RP2350 (existing work, harness validated), native I3C on ESP32-P4 later. Add **per-joint I3C timeout counters + error logging** so one noisy servo doesn't crash the controller. |
| Copper messages are `Clone + Copy` and growing | Silent stack/heap pressure on RP2350 | Adding `[f32;24]` velocity + torque and `[u8;24]` mode grows `HostCmdMsg` to ~350–400 B, `ControlStepMsg` to ~500+ B; passing all three by value through `DirectServoCuTask` adds 2–4 KB stack and expands the 64 KiB heap pools. Add a unit test printing `core::mem::size_of::<T>()` for all three messages and check stack high-water marks early in P1a. |
| ESP32-P4 scaffold incomplete | v2 cannot run yet | Port BSP: USB, WDT, LED, I3C hooks, PSRAM/heap config. |
| No foot contact sensor | ZMP/COP must be inferred | Current-based GRF + IMU step detection (Section 4.2). |

---

## 4. Detailed algorithm stack (on-MCU, 200 Hz, f32, no-alloc)

### 4.1 Per-tick pipeline (target design)

```
I3C readback (pos/vel/current/temp per joint)  ──┐
IMU (LSM6DSV high-rate gyro + accel)            ──┼──► Estimator (§4.2)
HostCmdMsg (mode, walk, gains)                  ──┘        │
                                                          ▼
                              Reference generator (DCM/LIPM/CPG, §4.4)
                                                          ▼
                     v1: kinematic DCM + UVC reflex (§6)  |  v2: task-space WBC (§4.5) + impedance (§4.3)
                                                          ▼
                                     Safety filter (CBF §4.7, clamps, watchdog)
                                                          ▼
                                     RawServoCommand (torque/velocity/position) → I3C write
                                                          ▼
                                     McuToHost telemetry (1024 B COBS)
```

### 4.2 State estimation

**a) CoM kinematics.** From URDF link masses $m_i$ and local COM offsets $r_i$:
$$c = \frac{1}{M}\sum_i m_i\,(p_i + R_i r_i), \qquad \dot c = J_{com}(q)\,\dot q$$
with $J_{com}=\frac{1}{M}\sum_i m_i J_i$. Now feasible because **joint velocity feedback exists**. EMA-filter $\dot c$ ($\alpha\approx0.3$ at 200 Hz) and air-gate (zero $\dot c$ correction when no contact).

**b) Current-based GRF / contact inference.** Motor torque $\tau_i \approx k_{t,i} I_i$ (after gearbox: $\tau_{joint}=N\eta\,k_t I$, with direction-dependent $\eta$). For a leg with contact force $f$ at the foot and Jacobian $J_{foot}$:
$$J_{foot}^\top f = \tau_{leg} - M_{leg}(q)\ddot q - C_{leg} - G_{leg} \;\Rightarrow\; \hat f = (J_{foot}^\top)^\dagger \big(\tau_{leg} - \text{dynamics terms}\big)$$
Simpler practical form: use the **momentum observer residual** (below) as the external/contact wrench estimate; convert to per-foot vertical force with a noise-weighted pseudoinverse. Observability requires non-singular leg Jacobian (avoid fully extended leg) — enforce via crouch. **Scale-corrected thresholds (1–2 kg body):** static foot load is ~7–15 N; a plausible contact threshold is **3–6 N** (as originally stated), but debounce must be **short (5–10 ms, 1–2 ticks)** for walking and even shorter for running — the earlier 20–30 ms is too slow. Flat feet + toes give a clear support polygon including toe-off. **v1 caveat:** with N20 high-ratio gears, current→GRF is dominated by stiction; this path is **v2/BLDC only** (see §6).

**c) Discrete momentum observer (De Luca / Wensing) — sign- and stability-corrected.** Generalized momentum $p=M(q)\dot q$ with contact wrench $\lambda$ and selection matrix $S$:
$$\dot p = C^\top\dot q - G + A^\top\lambda + S^\top(\tau_m + \tau_{ext})$$
($A^\top = J_c^\top$ maps contact forces to generalized forces; **$+A^\top\lambda$** since $\lambda$ is force applied *to the robot*). Correct **continuous** observer (note **$+r$ inside the integral**, without which $\dot r=K_O S^\top\tau_{ext}$ is a pure integrator that drifts unbounded on any constant bias):
$$r(t) = K_O\Big(p(t) - p(0) - \int_0^t \big(C^\top\dot q - G + A^\top\hat\lambda + S^\top\tau_m + r(\tau)\big)\,d\tau\Big)$$
so that $r \to S^\top\tau_{ext}$ in steady state. **Backward-Euler discrete form** (unconditionally stable for any $K_O>0$, avoids the old $K_O\Delta t<1$ boundary):
$$r_k = \frac{1}{1 + K_O\Delta t}\Big(r_{k-1} + K_O\big(p_k - p_{k-1} - \Delta t\,(C^\top\dot q - G + A^\top\hat\lambda + S^\top\tau_m)\big)\Big)$$
**Resolving the $\hat\lambda$ circularity (explicit two-stage pipeline):** §4.2b proposed using $r$ as the contact estimate while §4.2c subtracts $A^\top\hat\lambda$ inside the observer — feeding $r$-derived $\hat\lambda$ back would create a closed loop. Instead: (1) run the **unconstrained observer with $\hat\lambda = 0$**, so $r \to A^\top\lambda + S^\top\tau_{ext}$; (2) partition $r$ into contact vs external disturbance using **contact-mode projection** (stance-leg Jacobian subspace vs its null space / upper-body joints, Payne et al. 2024). Never feed $r$ back as $\hat\lambda$. Tuning: $K_O\approx100\,\mathrm{s}^{-1}$. **Scale-corrected impact threshold:** a shove on 1.5 kg produces only **~0.1–0.5 N·m** joint-space torque, not 5–15 N·m (larger than $mgh\approx3.7$ N·m). Use **0.1–0.3 N·m** with debounce — requires sub-0.02 N·m model/friction accuracy, **infeasible on v1 N20 gears; v2 BLDC only**.

**d) Attitude filter.** 4-state complementary filter (pitch/roll + gyro bias) using the IMU's gravity-derived tilt as reference and gyro integration; **~100 flops** (not 10⁴). A reduced Rotella-style EKF (base pose/velocity + contact-constrained leg updates) can be added later; the full 27-state EKF is feasible on 512 KB RAM but start simpler. **Do not stack a second low-pass filter on an already-fused gravity vector** (see §6.5 for the LSM6DSV recommendation).

### 4.3 Impedance / virtual model control
With torque command available:
$$\tau = J^\top\big(K_p(x_d-x) + K_d(\dot x_d-\dot x) + f_{ff}\big) + \tau_{posture}$$
Joint-level: $\tau_i = k_{p,i}(q_{d,i}-q_i)+k_{d,i}(\dot q_{d,i}-\dot q_i)+\tau_{ff,i}$.
**Scale-corrected gains for 1–2 kg:** the whole gravitational toppling moment is only $mgh\approx3.7$ N·m, so the earlier $k_p=20\!-\!80$ N·m/rad, $k_d=1\!-\!5$ N·m·s/rad are **10–30× too large**. Use **$k_p\approx1\!-\!5$ N·m/rad, $k_d\approx0.05\!-\!0.25$ N·m·s/rad** as starting values (e.g. 0.05 rad error → ~0.1 N·m, not 4 N·m). **Landing compliance:** a virtual leg spring $k\approx800$ N/m is reasonable for absorbing ~1 J, but ramp stiffening over **≤100 ms** so it completes within a stance phase (see §4.8). Note: higher inner torque-loop bandwidth *shrinks* the stable impedance region (Focchi et al.) — tune with real SimpleFOC loop delay. **v1 caveat:** active torque impedance requires a current loop; on v1 use *software current limiting* on position/velocity targets instead.

### 4.4 LIPM / DCM / capturability
- $\omega=\sqrt{g/h}$ with $h\approx0.25$ m → $\omega\approx6.3$ rad/s (unstable mode doubles every ~0.11 s).
- Capture point $\xi=c+\dot c/\omega$; $\dot\xi=\omega(\xi-p)$. Discrete IPM integration (cosh/sinh) with precomputed tables.
- Desired ZMP from DCM error: $p_d = \xi - \frac{1}{\omega}\dot\xi_{ref} + k_\xi(\xi-\xi_{ref})$; pole-placement gains from Caron's stabilizer.
- Ankle vs hip: $\dot L^\perp = mg\,\Delta p$ — a ZMP shift $\Delta p$ can be produced by centroidal angular momentum via torso/arms. Use ankle torque for small $\Delta p$ inside foot, hip for larger.
- Capturability: **foot geometry clarified** — assume a **7 cm total foot length** (3.5 cm half-length) with ~2.8 cm usable ankle/CoP authority after an ~80% safety margin; a 3.5 cm *total* foot would give only ±1.75 cm. Confirm the printed foot length before encoding the support polygon. If $|\xi|$ exceeds ankle authority → step. Foot placement $x_f=\xi+k_v(\dot c-\dot c_{des})+\frac{1}{2}\dot c\,T_s$; step shortening (Khadiv) when needed.

### 4.5 Task-space priority controller (feasible simplified WBC) — **v2 / BLDC only**
4-level **velocity-level** DLS/null-space IK stack (no 21×21 dense matrices):
1. Foot contact / stance constraints (highest priority)
2. CoM / ZMP tracking
3. Torso attitude
4. Posture / joint limits (null space)

Correct strict-priority recursion (project secondary Jacobian into the null space of the first task):
$$\dot q_1 = J_1^\dagger \dot x_1,\quad N_1 = I - J_1^\dagger J_1,\quad \bar J_2 = J_2 N_1$$
$$\dot q = \dot q_1 + \bar J_2^\dagger\big(\dot x_2 - J_2\dot q_1\big) + N_2(\dots),\qquad N_2 = N_1\big(I - \bar J_2^\dagger \bar J_2\big)$$
so level 3 stays in the intersection of both null spaces. **Avoid dense $n\times n$ matrices:** forming $N_1=I-J_1^\dagger J_1$ directly allocates a $21\times21$ matrix; compute $\bar J_2 = J_2 - (J_2 J_1^\dagger)J_1$ using the smaller task-dimension products instead.
**Correction:** the previous single-line formula omitted the null-space projection of $J_2$ and the task-2 residual $(\dot x_2 - J_2\dot q_1)$; without them, non-orthogonal tasks corrupt level-2 tracking. This stack maps desired **velocities** to joint velocities; **acceleration/torque-level WBC** additionally needs $M(q), C, G$ and is a separate formulation (MIT WBIC style) — do not conflate them. ~10⁴ flops at 200 Hz. Full CMM $A(q)={}^1X_G^\top\Psi_1^\top U_1 H$ (Wensing & Orin 2016) is computable in ~10 µs-class on MCU-class f32. **v1 alternative:** kinematic DCM + UVC postural reflex (§6) — no dynamics/WBC.

### 4.6 Small f32 LIPM-MPC (optional, P2+)
Cart-table state $(x,\dot x,\ddot x)$, jerk input, horizon $N=6$, $T=0.08$ s, cost weights ~10/1000/1 (Scianca/Caron). Hand-coded fixed-iteration active-set or ADMM with precomputed factorizations; update at 10–20 Hz, track with the 200 Hz analytic controller.

### 4.7 CBF safety filter
Scalar barriers with explicit projection (no QP needed): torso tilt $|\theta|\le\theta_{max}$, CoM velocity, capture point inside support, $h(x)\ge0$, $\dot h \ge -\kappa h$, $\kappa\approx3$ s⁻¹. Filter commanded torque/velocity by minimal modification satisfying constraints.

### 4.8 Running (P3) — **scaled down to v1/v2 reality**
The earlier "20 cm hop" is arithmetically correct ($v=\sqrt{2g(0.20)}=1.98$ m/s) but physically unrealistic: for $m=1.5$ kg it demands $E=mg\Delta h\approx2.94$ J, average push-off force $\approx64$ N (~4.3× body weight), peak forces >90 N and >100 W — beyond N20s and printed joints. Also, the SLIP half-period with $k=670$ N/m is $\pi\sqrt{m/k}\approx149$ ms single-leg (~105 ms two-leg), **not 75 ms**; 75 ms stance is only 15 ticks at 200 Hz, conflicting with touchdown debounce and landing ramps. **Revised target: 2–4 cm hop** ($T_s\approx120$–150 ms, >24 ticks), modest thrust, short (<5 ms) debounce, and a **25–40 ms landing-compliance ramp for running** (a ≤100 ms ramp consumes 70–80% of a 120–150 ms stance; reserve 50–100 ms ramps for walking touchdowns). Raibert law: $x_f=x_{com}+v_{com}T/2+k_v e_v$; hop height via thrust impulse; flight detection from near-zero motor current + IMU free-fall signature. Running is a **v2/BLDC** goal; v1 targets robust walking only.

---

## 5. Offline PC role (no runtime PC, no camera)
- Train tiny int8 MLP residual (obs: pos+vel+current+IMU; action: torque/position residuals) with domain randomization; deploy as 50 Hz residual over analytic controller.
- Momentum-optimized strike keyframes (Wang/Kheddar impact-aware QP) with balance-restore tails, exported into the Action Studio.
- Gait reference libraries, gain tuning, simulator validation.

---

## 6. Staged roadmap — split by actuator generation (critique §5)

**P0 (done):** fall detection, classified get-up library, recovery state machine, red alarm.

**P1a — v1 firmware foundation + kinematic/UVC balance (N20 + RP2350):**
1. Wire hardening: `MAX_COBS_FRAME_BYTES` → 1024; extend `RawServoCommand`/`RawServoFeedback` + Copper messages with torque/velocity/current; add `protocol_version`, CRC, and per-joint torque/velocity clamps.
2. PIO I3C SimpleFOC driver on RP2350 via `ServoBusHooks` (existing work); **per-joint timeout counters and error logging** so one noisy servo doesn't crash the controller. Verify 21-joint 200 Hz exchange. I3C harness signal integrity is **already validated on hardware** (user confirmation).
3. Kinematic estimator only: CoM position/velocity from FK + joint velocity; attitude from the IMU (see §6.5). **No current-based GRF, no momentum observer on v1.**
4. **Kinematic DCM + UVC postural reflex — clean DOF partitioning (avoids controller fighting):** Dr. Guero's UVC is a **pure integral (I-only)** reflex — he explicitly notes adding P/D to UVC itself destabilizes it because it maintains verticality, not a fixed equilibrium. Two integrators on the same hip/stride DOFs would fight, especially since v1 CoM velocity $\dot c$ assumes a flat non-slipping stance foot (breaks under shove). Partition strictly:
   - **Stance torso verticality → UVC only:** pure I of LSM6DSV pitch/roll into grounded hip pitch/roll or stride offset (`dxi/dyi`), support alternation (`jikuasi`) from CPG phase. Keep a crouched nominal leg length (e.g. 170 mm vs 180 mm max) to avoid knee singularity at touchdown; leak/anti-windup clamp on `dxi/dyi` so it resets when lifted or fallen.
   - **High-frequency vibration damping → ankle rate-gyro P-term:** small proportional feedback from raw LSM6DSV gyro rate onto ankle pitch/roll — separate from UVC's hip integrator.
   - **Capturability / stepping → kinematic DCM on swing-foot placement ONLY:** use DCM error strictly to modulate the *next* foot landing point $x_f$ and trigger a recovery step when tilt/DCM leaves the ±2.8 cm foot margin — never double-drive the stance hip.
   - **Fall relaxation ("ukemi"):** once tilt passes the irrecoverable angle (before P0 recovery triggers), drop joint torque/current limits immediately to protect printed links and N20 gears (matches current-limited hip shock absorption).
5. Software current limiting for impact compliance; verify shove recovery in sim, bench loopback, then hardware.
6. **Bench-test N20 current-vs-load linearity** to decide if any current-based sensing is viable later; do not assume it.

**P1b — v2 torque/observer stack (BLDC + ESP32-P4 + high-rate IMU):**
- Closed-loop SimpleFOC torque control; current-based GRF (sign-corrected, §4.2b); discrete momentum observer with 0.1–0.3 N·m thresholds; active impedance with rescaled gains; task-space priority WBC; LIPM-MPC. Review the §3 sign fix and §4 scalings before coding.

**P2 — Robust walking (v1 kinematic + v2 torque variants):** phase clock with IMU foot-strike detection; Raibert/capture foot placement; step timing adaptation; small f32 LIPM-MPC (v2).

**P3 — Running (v2/BLDC only):** reduced 2–4 cm SLIP hop, flight detection, short debounce, landing impedance, CBF shield.

**P4 — Combat:** offline strike keyframes with balance tails; impact detection (v2); hip/arm counter-rotation combos; optional tiny MLP residual.

**BSP parallel track (secondary):** ESP32-P4 TODOs — USB HS, TIMG0 WDT, LED, native I3C hooks, PSRAM/heap config — kept strictly behind RP2350 stability (esp-hal I3C/USB-HS/PSRAM support is less mature than embassy-rp).

### 6.5 IMU strategy for v1 (LSM6DSV16X) — detailed
The BNO055's **fusion is hard-locked to 100 Hz** (10 ms latency floor ≈ the 5 ms control period) and its NDOF magnetometer is easily disturbed by 21 nearby motors — precisely the critique's concern. **Recommendation: use an ST LSM6DSV16X as the sole primary IMU on v1; remove the BNO055 from the control path entirely (keep only as an optional slow diagnostic / power it down).** Neither UVC nor kinematic DCM needs absolute magnetic yaw — use integrated gyro-Z for turning (3°/h bias ≈ 0.05°/min, far cleaner than a magnet-disturbed compass).

**Why LSM6DSV16X (DS13510 Rev4 / AN5763 Rev4):**
- Gyro noise **2.8 mdps/√Hz** (HP mode, independent of ODR/FS); at 200 Hz that's ~28 mdps RMS → ~0.14° integration error per step — excellent. Bias instability ~3°/h; zero-rate ±1 dps, drift ±0.006 dps/°C.
- Accel noise **60 µg/√Hz** (HP). ODR up to **7.68 kHz** (vs BNO055's 100 Hz fused). Interfaces: SPI / I²C / **I3C v1.1 @12.5 MHz**; on-chip FSM @960 Hz, SFLP fusion ≤240 Hz, **4.5 KB FIFO**, pedometer/sig-motion/free-fall interrupts, 0.65 mA.
- **No magnetometer** → no motor-field yaw jumps.

**Integration (concrete, verified against ST DS13510 Rev4 / AN5763 Rev4 and ArduPilot/PX4 drivers):**
- Hardware: LSM6DSV16X on **dedicated SPI1** on RP2350 (not the I3C servo bus — avoids IBI arbitration delaying a gyro sample), **INT1 (DRDY) → GPIO IRQ**, mounted near torso CoM. Remove/power-down BNO055 on v1.
- **`WHO_AM_I` (0x0F) = 0x70** (confirmed for both 16X and 32X; earlier 0x6B/0x6C guesses were wrong). 16X vs 32X distinguished by `CTRL8` bit2 at reset (must-be-0 on 16X, must-be-1 on 32X). Retry the ID read 2–3× — the first SPI transaction after power-up can return garbage (PX4 practice).
- **Register map:** ODR/mode in `CTRL1` (0x10, accel) / `CTRL2` (0x11, gyro); **full-scale bits are in `CTRL6` (0x15, gyro `FS_G[3:0]`) and `CTRL8` (0x17, accel `FS_XL[1:0]`)**, not CTRL1/CTRL2 as on older LSM6DSO parts. `CTRL3` (0x12) holds BDU(bit6)/IF_INC(bit2)/SW_RESET(bit0). Gyro LPF1 bandwidth in `CTRL6`, enable via `CTRL7` (0x16) `LPF1_G_EN`. Data `0x22–0x27` gyro / `0x28–0x2D` accel; scales 35 mdps/LSB @±1000 dps, 0.244 mg/LSB @±8 g. 4.5 KB FIFO.
- **Anti-aliasing / 960→200 Hz downsampling:** a single instantaneous 960 Hz sample in the 200 Hz tick aliases motor/gearbox vibration >100 Hz into the balance gains. Preferred: run 960 Hz into the **hardware FIFO**, **burst-read ~5 accumulated samples once per 5 ms tick over SPI1 DMA**, integrate all 5 into the complementary filter and average gyro rate for damping — cuts SPI/IRQ switches 5×. Alternative: enable gyro LPF1 (~70–90 Hz). Discard ~6 settling samples / wait ~50 ms after config.
- **Gravity from attitude, NOT an accel LPF (critical fix):** LPF of raw acceleration with τ=0.5–1 s would lag fast falls by up to a second — a 45 cm robot hits the floor in ~0.35 s, so fall detection would see a stale "upright" gravity mid-fall. Compute gravity **directly from the complementary-filter pitch/roll with zero lag**:
  $$g_{body} = 9.81\,[-\sin\theta,\ \cos\theta\sin\phi,\ \cos\theta\cos\phi]^\top$$
  Spikes are still rejected through the gyro path. Compatible with the `fall-detection/spec.md` classifier.
- Balance uses **raw averaged gyro rate** (lowest latency); complementary filter ~100 flops, **no second filter stacked on fused gravity**.
- Config: add `imu-lsm6dsv` feature; publish `ImuState` to `IMU_SLOT` at ≥200 Hz with `available`, pitch/roll from the filter, yaw by gyro integration, rates from averaged gyro, and attitude-derived gravity. **No Copper bridge change** — `copper.rs:624–626` already overrides each 200 Hz feedback frame with `IMU_SLOT.latest()`. Hold last gyro ≤20 ms before declaring balance unavailable.

---

## 7. Compute / memory budget (RP2350) — corrected
- **Corrected FLOP estimate:** a 4-state complementary filter is ~100 flops, not 10⁴. Conversely, floating-base kinematics, RNEA ($C^\top\dot q+G$), CRBA ($M(q)$ for the momentum observer), CMM $A(q)$, and 4-level DLS pseudoinverse across 21 DoF realistically cost **50,000–120,000 flops/tick** (v2 stack), not 20–30k. With generated sparse Featherstone routines that is ~1–2 ms of the 5 ms tick at 150 MHz hard-FPU — **fits, provided I3C transfers are non-blocking (PIO/DMA overlapped)** rather than stalling `ServoBusBridge` for ~1.5 ms. v1's kinematic/UVC stack is far cheaper (~few thousand flops).
- RAM: 512 KB total, 64 KiB cu29 heap; estimator/controller state < 20 KB — comfortable.
- I3C budget: ~777 B/cycle at 200 Hz ≈ 155 kB/s (~15% of a 1 MB/s usable 12.5 MHz bus); harness already validated on hardware. Keep transfers overlapped and each node timeout-bounded.

---

## 8. Bottom line (revised)
The critique is accepted: **v1 (N20 geared brushed + 100 Hz BNO055) cannot support current-based GRF, momentum observers, active torque impedance, or 20 cm SLIP hopping.** The clean architecture (control inside `DirectServoKernel::step`, `ServoBusHooks` BSP decoupling, 1024 B wire with version/CRC/clamps) stays, but the algorithm stack is **split by actuator generation**:
- **v1 / P1a:** wire hardening + LSM6DSV16X (960 Hz FIFO-burst, attitude-derived gravity, no BNO055 in the loop) + **kinematic DCM + UVC postural reflex with strict DOF partitioning** (UVC = stance verticality I-only, ankle = gyro-rate damping P, DCM = swing-foot placement only, ukemi torque relaxation) + software current limiting via per-mode torque-limit fields.
- **v2 / P1b:** sign/stability-corrected momentum observer (with $\hat\lambda=0$ two-stage projection), rescaled impedance, task-space WBC, LIPM-MPC, and modest running — on BLDC + ESP32-P4.

Vision, large learned policies, and dense QP/NMPC remain excluded, with their outputs precomputed offline. Immediate next step: **OpenSpec proposal for P1a**.
