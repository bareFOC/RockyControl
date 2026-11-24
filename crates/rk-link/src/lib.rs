//! Layer 4 — `RobotTransport` trait & Stage 1 Sim / Stage 2 Serial backends (`rk-link`).

use rk_control::JointCalibration;
use rk_proto::{
    ConfigCmd, HostToMcu, MAX_JOINTS, McuToHost, ModelFormat, ModelRequest, ModelResponse,
};
use rk_sim::{CopperReplayLog, CopperSimRunner, SimFaultInjection};

/// Fixed period of one control tick (5 ms / 200 Hz).
///
/// Mirrors `rk_fw_core::config::CONTROL_DT_SEC`; `rk-link` intentionally does not
/// depend on `rk-fw-core` directly, so keep these two values in sync. The
/// deterministic harness in `rk-sim` already asserts the firmware value.
pub const FIXED_CONTROL_STEP_SEC: f32 = 0.005;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportKind {
    /// Desktop `copper-rs` simulation runtime or Browser `#![no_std]` WASM safety kernel + plant.
    SimKernel,
    /// Wired USB CDC-ACM virtual serial (`postcard` COBS frames).
    UsbSerialStage2,
    /// Wired USB Networking (CDC-NCM Ethernet-over-USB, UDP `10.42.0.2:7777`).
    UsbNetworkingUdp,
}

impl TransportKind {
    pub fn label(self) -> &'static str {
        match self {
            #[cfg(target_arch = "wasm32")]
            Self::SimKernel => "Sim (WASM Kernel)",
            #[cfg(not(target_arch = "wasm32"))]
            Self::SimKernel => "Sim (Copper-rs)",
            Self::UsbSerialStage2 => "USB CDC-ACM (Wired)",
            Self::UsbNetworkingUdp => "USB Net UDP 10.42.0.2:7777",
        }
    }
}

pub struct LinkStats {
    pub packet_rate_hz: f32,
    pub rtt_ms: f32,
    pub link_errors: u64,
    pub connected: bool,
}

pub trait RobotTransport {
    fn kind(&self) -> TransportKind;

    /// Legacy, variable-dt exchange path.
    ///
    /// Kept for the MCP bridge, headless harnesses and compatibility tests. This
    /// is a **separate timing domain**: it performs one caller-paced tick per
    /// call and its reported rate is derived from the caller's frame dt. The
    /// interactive app uses [`RobotTransport::step_fixed`] instead.
    fn step_exchange(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> Option<McuToHost>;

    /// Runs exactly `steps` deterministic 5 ms ticks through the shared Embassy
    /// control task and returns the last telemetry frame.
    ///
    /// `frame_dt_sec` is real elapsed frame time, used only to measure the
    /// achieved ticks-per-second reported by [`RobotTransport::stats`]; it never
    /// enters the control path. The host command is sampled once by the caller
    /// and applied to every tick (one-shot config commands only on the first
    /// tick). Non-sim transports fall back to one compat exchange because their
    /// 200 Hz loop is owned by firmware, not by the host app.
    fn step_fixed(
        &mut self,
        steps: usize,
        host_pkt: &HostToMcu,
        frame_dt_sec: f32,
    ) -> Option<McuToHost> {
        let _ = steps;
        self.step_exchange(frame_dt_sec, host_pkt)
    }

    fn stats(&self) -> LinkStats;
    fn sim_faults_mut(&mut self) -> Option<&mut SimFaultInjection> {
        None
    }
    fn sim_faults(&self) -> Option<&SimFaultInjection> {
        None
    }
    fn set_virtual_hand_angle(&mut self, _joint: usize, _raw_rad: f32) {}
    fn virtual_hand_angles(&self) -> Option<&[f32; MAX_JOINTS]> {
        None
    }
    fn inject_corrupted_frame(&mut self) {}
    /// Injects a combat shove impulse into the simulated torso (Sim mode only).
    fn apply_push_impulse(&mut self, _pitch_impulse_rad_s: f32, _roll_impulse_rad_s: f32) {}
    /// Resets the simulated torso upright at 0 deg pitch/roll (Sim mode only).
    fn reset_upright(&mut self) {}
    /// Updates active get-up recovery progress (`Some(0.0..=1.0)`) for simulated ground push-off.
    fn set_recovery_progress(&mut self, _progress: Option<f32>) {}
    fn is_recording(&self) -> bool {
        false
    }
    fn start_recording(&mut self) {}
    fn stop_recording(&mut self) -> Option<CopperReplayLog> {
        None
    }
    fn recorded_log(&self) -> Option<&CopperReplayLog> {
        None
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn save_copper_log(&self, _path: &std::path::Path) -> std::io::Result<()> {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "not supported by transport",
        ))
    }
    /// Sends a single `ModelRequest` (`GetInfo` or `ReadChunk`) to the MCU onboard model store.
    fn exchange_model(&mut self, _req: ModelRequest) -> Option<ModelResponse> {
        None
    }
    /// Stages a URDF / kinematics blob into the simulated MCU flash store (Sim mode only).
    fn stage_sim_mcu_model(
        &mut self,
        _name: &str,
        _format: ModelFormat,
        _blob: Vec<u8>,
        _joint_count: u8,
    ) -> bool {
        false
    }
}

/// Stage 1 Software Simulation transport backed by `CopperSimRunner` (runs on both Desktop and WASM).
pub struct SimTransport {
    runner: CopperSimRunner,
    /// Achieved control ticks per wall-clock second (200.0 when the sim keeps up).
    rate_hz_smooth: f32,
    rtt_ms: f32,
}

impl SimTransport {
    /// Reset the global deterministic clock (tests / fresh sessions only).
    pub fn reset_mock_clock(&self) {
        self.runner.reset_mock_clock();
    }

    pub fn new(
        joint_count: usize,
        initial_cal: &[JointCalibration],
        initial_raw_pose: [f32; MAX_JOINTS],
    ) -> Self {
        Self {
            runner: CopperSimRunner::new(joint_count, initial_cal, initial_raw_pose),
            rate_hz_smooth: 200.0,
            rtt_ms: 1.4,
        }
    }

    /// Advances the simulated MCU by `steps` fixed 5 ms control ticks.
    ///
    /// Host commands are sampled once per rendered frame and repeated on every
    /// tick so the deadman watchdog behaves as it does on the 200 Hz wire;
    /// one-shot `ConfigCmd`s are applied only on the first tick. The simulated
    /// heartbeat-drop fault suppresses all host commands for the frame, matching
    /// the legacy `step_exchange` behaviour. COBS encoding of host commands is
    /// intentionally bypassed here — `run_harness` queues typed commands
    /// directly — while MCU telemetry still round-trips through COBS in
    /// `RobotIo::publish_telemetry`.
    fn step_fixed_inner(
        &mut self,
        steps: usize,
        host_pkt: &HostToMcu,
        frame_dt_sec: f32,
    ) -> Option<McuToHost> {
        if steps == 0 {
            return None;
        }

        // Achieved rate: ticks actually executed per real second. This is 200 Hz
        // while catch-up keeps up and drops below that when the UI cap is hit.
        let inst_hz = (steps as f32 / frame_dt_sec.max(0.001_f32)).clamp(0.0, 200.0);
        self.rate_hz_smooth = self.rate_hz_smooth * 0.9 + inst_hz * 0.1;

        let dropped = self.runner.plant.faults().drop_heartbeat;
        self.rtt_ms = if dropped {
            0.0
        } else {
            1.2 + ((host_pkt.seq.wrapping_mul(17) % 10) as f32) * 0.09
        };

        // Keep the running gate in sync with the sampled command, as `tick()` did.
        self.runner.plant.set_running_config(host_pkt.walk.running);

        let mut cmds = Vec::with_capacity(steps);
        if !dropped {
            let mut later = *host_pkt;
            later.config = ConfigCmd::None;
            cmds.push(*host_pkt);
            cmds.extend(std::iter::repeat(later).take(steps - 1));
        }

        // The runner owns one persistent executor + spawned control task for its
        // whole lifetime. Each call only queues commands and advances that same
        // task; no new executor or task is created against the process-global
        // mock clock, and kernel state / tick index keep advancing.
        let telem = self.runner.step_harness(steps, &cmds);

        if dropped { None } else { Some(telem) }
    }
}

impl RobotTransport for SimTransport {
    fn kind(&self) -> TransportKind {
        TransportKind::SimKernel
    }

    fn step_exchange(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> Option<McuToHost> {
        // Compat timing domain: one caller-paced variable-dt tick, so the legacy
        // rate here is still frame-derived. See `step_fixed` for the real loop.
        let inst_hz = (1.0_f32 / dt_sec.max(0.001_f32)).clamp(1.0, 500.0);
        self.rate_hz_smooth = self.rate_hz_smooth * 0.9 + inst_hz * 0.1;
        let dropped = self.runner.plant.faults().drop_heartbeat;
        self.rtt_ms = if dropped {
            0.0
        } else {
            1.2 + ((host_pkt.seq.wrapping_mul(17) % 10) as f32) * 0.09
        };
        Some(self.runner.tick(dt_sec, host_pkt))
    }

    fn step_fixed(
        &mut self,
        steps: usize,
        host_pkt: &HostToMcu,
        frame_dt_sec: f32,
    ) -> Option<McuToHost> {
        self.step_fixed_inner(steps, host_pkt, frame_dt_sec)
    }

    fn stats(&self) -> LinkStats {
        let dropped = self.runner.plant.faults().drop_heartbeat;
        LinkStats {
            packet_rate_hz: if dropped { 0.0 } else { self.rate_hz_smooth },
            rtt_ms: self.rtt_ms,
            link_errors: self.runner.link_errors(),
            connected: !dropped,
        }
    }

    fn sim_faults_mut(&mut self) -> Option<&mut SimFaultInjection> {
        Some(self.runner.plant.faults_mut())
    }

    fn sim_faults(&self) -> Option<&SimFaultInjection> {
        Some(self.runner.plant.faults())
    }

    fn set_virtual_hand_angle(&mut self, joint: usize, raw_rad: f32) {
        self.runner.plant.set_virtual_hand_angle(joint, raw_rad);
    }

    fn virtual_hand_angles(&self) -> Option<&[f32; MAX_JOINTS]> {
        Some(self.runner.plant.virtual_hand_angles())
    }

    fn inject_corrupted_frame(&mut self) {
        self.runner.inject_corrupted_frame();
    }

    fn apply_push_impulse(&mut self, pitch_impulse_rad_s: f32, roll_impulse_rad_s: f32) {
        self.runner
            .apply_push_impulse(pitch_impulse_rad_s, roll_impulse_rad_s);
    }

    fn reset_upright(&mut self) {
        self.runner.reset_upright();
    }

    fn set_recovery_progress(&mut self, progress: Option<f32>) {
        self.runner.set_recovery_progress(progress);
    }

    fn is_recording(&self) -> bool {
        self.runner.is_recording()
    }

    fn start_recording(&mut self) {
        self.runner.start_recording();
    }

    fn stop_recording(&mut self) -> Option<CopperReplayLog> {
        self.runner.stop_recording()
    }

    fn recorded_log(&self) -> Option<&CopperReplayLog> {
        self.runner.recorded_log()
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn save_copper_log(&self, path: &std::path::Path) -> std::io::Result<()> {
        self.runner.save_copper_log_to_path(path)
    }

    fn exchange_model(&mut self, req: ModelRequest) -> Option<ModelResponse> {
        self.runner.exchange_model_request(req)
    }

    fn stage_sim_mcu_model(
        &mut self,
        name: &str,
        format: ModelFormat,
        blob: Vec<u8>,
        joint_count: u8,
    ) -> bool {
        self.runner.stage_mcu_model(name, format, blob, joint_count);
        true
    }
}

/// Native desktop wired USB Networking (CDC-NCM) UDP transport (`10.42.0.2:7777`).
///
/// Exchanges identical `postcard` COBS-framed `HostToMcu` / `McuToHost` datagrams with
/// `firmware/rk-rp2350` (`--no-default-features --features usb-net,bench-loopback`).
#[cfg(not(target_arch = "wasm32"))]
pub struct UdpTransport {
    socket: std::net::UdpSocket,
    remote_addr: std::net::SocketAddr,
    tx_buf: [u8; rk_proto::MAX_COBS_FRAME_BYTES],
    rx_buf: [u8; rk_proto::MAX_COBS_FRAME_BYTES],
    link_errors: u64,
    rate_hz_smooth: f32,
    last_rx_ok: bool,
}

#[cfg(not(target_arch = "wasm32"))]
impl UdpTransport {
    pub fn connect(remote: &str) -> std::io::Result<Self> {
        let remote_addr: std::net::SocketAddr = remote.parse().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("invalid addr: {e}"))
        })?;
        let socket = std::net::UdpSocket::bind("0.0.0.0:0")?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            remote_addr,
            tx_buf: [0u8; rk_proto::MAX_COBS_FRAME_BYTES],
            rx_buf: [0u8; rk_proto::MAX_COBS_FRAME_BYTES],
            link_errors: 0,
            rate_hz_smooth: 0.0,
            last_rx_ok: false,
        })
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl RobotTransport for UdpTransport {
    fn kind(&self) -> TransportKind {
        TransportKind::UsbNetworkingUdp
    }

    fn step_exchange(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> Option<McuToHost> {
        if let Ok(encoded) = rk_proto::encode_cobs(host_pkt, &mut self.tx_buf) {
            if self.socket.send_to(encoded, self.remote_addr).is_err() {
                self.link_errors = self.link_errors.saturating_add(1);
            }
        } else {
            self.link_errors = self.link_errors.saturating_add(1);
        }

        // Drain all queued UDP telemetry datagrams and keep the freshest packet
        let mut latest = None;
        loop {
            match self.socket.recv_from(&mut self.rx_buf) {
                Ok((n, _)) if n > 0 => {
                    match rk_proto::decode_mcu_cobs(&mut self.rx_buf[..n]) {
                        Ok(telem) => latest = Some(telem),
                        Err(_) => self.link_errors = self.link_errors.saturating_add(1),
                    }
                }
                Ok(_) => break,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => {
                    self.link_errors = self.link_errors.saturating_add(1);
                    break;
                }
            }
        }

        if latest.is_some() {
            let inst_hz = (1.0_f32 / dt_sec.max(0.001_f32)).clamp(1.0, 500.0);
            self.rate_hz_smooth = self.rate_hz_smooth * 0.9 + inst_hz * 0.1;
            self.last_rx_ok = true;
        }
        latest
    }

    fn stats(&self) -> LinkStats {
        LinkStats {
            packet_rate_hz: self.rate_hz_smooth,
            rtt_ms: 0.5,
            link_errors: self.link_errors,
            connected: self.last_rx_ok,
        }
    }

    fn exchange_model(&mut self, req: ModelRequest) -> Option<ModelResponse> {
        // Query dedicated MCU Flash Model Service on UDP port 7778 (control port + 1)
        let model_addr = std::net::SocketAddr::new(
            self.remote_addr.ip(),
            self.remote_addr.port().saturating_add(1),
        );
        let sock = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
        let _ = sock.set_read_timeout(Some(std::time::Duration::from_millis(80)));
        let mut tx = [0u8; rk_proto::MAX_MODEL_COBS_FRAME_BYTES];
        let wire = rk_proto::encode_model_req_cobs(&req, &mut tx).ok()?;
        sock.send_to(wire, model_addr).ok()?;
        let mut rx = [0u8; rk_proto::MAX_MODEL_COBS_FRAME_BYTES];
        let (n, _) = sock.recv_from(&mut rx).ok()?;
        rk_proto::decode_model_resp_cobs(&mut rx[..n]).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_proto::ControlMode;
    use std::sync::Mutex;

    // The Embassy mock clock and task pool are process-global in this workspace.
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn sim() -> SimTransport {
        SimTransport::new(6, &[JointCalibration::default(); MAX_JOINTS], [0.0; MAX_JOINTS])
    }

    /// The app-facing path must drive the shared Embassy task for exactly `n`
    /// fixed ticks, and report achieved ticks per wall second (not render FPS).
    #[test]
    fn step_fixed_runs_n_fixed_ticks_and_reports_achieved_rate() {
        let _guard = TEST_LOCK.lock().unwrap();
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();
        let pkt = HostToMcu {
            seq: 7,
            mode: ControlMode::Teach,
            ..Default::default()
        };

        let telem = sim.step_fixed(4, &pkt, 0.020).expect("telemetry frame");
        assert_eq!(telem.active_mode, ControlMode::Teach);

        let log = sim.stop_recording().expect("recorded replay");
        assert_eq!(log.ticks.len(), 4);
        for (i, tick) in log.ticks.iter().enumerate() {
            assert!((tick.dt_sec - FIXED_CONTROL_STEP_SEC).abs() < 1e-6);
            assert_eq!(tick.now_ms, ((i + 1) * 5) as u32);
            assert_eq!(tick.rx_cmd.map(|c| c.seq), Some(7));
        }
        // 4 ticks in 20 ms of real time == sustained 200 Hz.
        assert!((sim.stats().packet_rate_hz - 200.0).abs() < 0.01);
    }

    /// When the frame cannot supply enough real time for 200 Hz, the reported
    /// rate must fall below 200 rather than echoing the render frame rate.
    #[test]
    fn step_fixed_reports_rate_shortfall() {
        let _guard = TEST_LOCK.lock().unwrap();
        let mut sim = sim();
        let pkt = HostToMcu::default();
        for _ in 0..8 {
            // 2 ticks per 20 ms frame == 100 Hz achieved.
            sim.step_fixed(2, &pkt, 0.020);
        }
        assert!(sim.stats().packet_rate_hz < 160.0);
    }

    /// A one-shot config command sampled once per frame is applied on the first
    /// fixed tick only; later ticks see a clean repeated motion command.
    #[test]
    fn step_fixed_applies_one_shot_config_once() {
        let _guard = TEST_LOCK.lock().unwrap();
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();
        let cfg = ConfigCmd::SetZeroOffset {
            joint: 0,
            zero_rad: 0.25,
        };
        let pkt = HostToMcu {
            seq: 3,
            config: cfg,
            ..Default::default()
        };
        sim.step_fixed(3, &pkt, 0.015);
        let log = sim.stop_recording().expect("recorded replay");
        assert_eq!(log.ticks.len(), 3);
        assert_eq!(log.ticks[0].rx_cmd.expect("cmd").config, cfg);
        for tick in &log.ticks[1..] {
            assert_eq!(tick.rx_cmd.expect("cmd").config, ConfigCmd::None);
        }
    }
}
