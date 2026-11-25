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
    /// Status LED state from the firmware LED mode state machine (Sim mode only).
    /// Default false for wired transports that have no simulated LED side effect.
    /// Evaluated by `post_tick` outside the recorded kernel tick stream.
    fn sim_led_on(&self) -> bool {
        false
    }
    /// Resets the simulation experiment (kernel, plant, faults, wire and history)
    /// while preserving the persistent executor. No-op for wired transports.
    fn reset_experiment(&mut self) {}
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

/// Deterministic subset of [`SimFaultInjection`] applied to the host -> MCU wire.
#[derive(Clone, Copy, Default)]
struct WireRxFaults {
    crc_error: bool,
    version_mismatch: bool,
    truncated: bool,
    fragmented: bool,
}

impl WireRxFaults {
    fn from_injection(faults: &SimFaultInjection) -> Self {
        Self {
            crc_error: faults.rx_crc_error,
            version_mismatch: faults.rx_version_mismatch,
            truncated: faults.rx_truncated_frame,
            fragmented: faults.rx_fragmented_frame,
        }
    }
}

/// Flips one COBS *data* byte (never a code byte or the delimiter) so the
/// decoded payload/CRC changes while the frame stays structurally valid.
///
/// `CobsStreamFramer` then reports `CodecError::CrcMismatch` through the shared
/// mailbox, exactly as a bit-flipped USB CDC-ACM transfer would on hardware.
fn flip_cobs_data_byte(frame: &mut [u8]) {
    // The final byte of an encoded frame is the 0x00 COBS delimiter.
    let end = frame.len().saturating_sub(1);
    let mut idx = 0usize;
    while idx < end {
        let code = frame[idx] as usize;
        if code == 0 {
            // Malformed frame: leave it to the decoder to count the error.
            return;
        }
        idx += 1;
        let data_end = (idx + code - 1).min(end);
        if data_end > idx {
            let target = idx + (data_end - idx) / 2;
            // COBS data bytes are nonzero; keep the result nonzero so framing
            // is unchanged and only the CRC check fails.
            frame[target] = if frame[target] == 1 {
                0x02
            } else {
                frame[target] ^ 1
            };
            return;
        }
        // `code == 1` (or a truncated final run): the next wire byte is the
        // following group's code byte.
        idx = data_end;
    }
}

/// Encodes `host_pkt` with the real `rk_proto::encode_cobs`, optionally injects
/// a deterministic wire fault, then decodes it through the *same*
/// `CobsStreamFramer` / `decode_rx` path firmware uses.
///
/// Returns the typed command the MCU would actually have received, or `None`
/// when the frame was rejected. Codec errors land in the runner's local
/// mailbox and are copied into the next telemetry tick by `step_control_tick`.
fn decode_wire_host_cmd(
    runner: &mut CopperSimRunner,
    host_pkt: &HostToMcu,
    faults: WireRxFaults,
    wire: &mut [u8; rk_proto::MAX_COBS_FRAME_BYTES],
) -> Option<HostToMcu> {
    // The protocol version lives inside the postcard payload that the CRC
    // covers. Inject a mismatch by encoding a packet with the wrong version:
    // the frame is structurally valid with a correct CRC, and the firmware's
    // `validate_host_packet` rejects it as `VersionMismatch`. This avoids
    // hard-coding a byte offset into a postcard payload.
    let mut pkt = *host_pkt;
    if faults.version_mismatch {
        pkt.protocol_version = rk_proto::PROTOCOL_VERSION.wrapping_add(1);
    }

    let len = match rk_proto::encode_cobs(&pkt, wire) {
        Ok(encoded) => encoded.len(),
        Err(_) => return None,
    };

    if faults.crc_error {
        flip_cobs_data_byte(&mut wire[..len]);
    }

    if faults.truncated && len >= 2 {
        // Drop the final data byte but retain the COBS delimiter. The shared
        // framer sees a complete-but-invalid frame, counts the framing error
        // and resynchronises immediately, so the next clean tick is unaffected.
        wire[len - 2] = 0x00;
        return runner.decode_host_frame(&mut wire[..len - 1]);
    }

    if faults.fragmented {
        // Exercise the streaming accumulator across several pushes, as USB
        // CDC-ACM delivers ~120-byte packets in 64-byte chunks. The command
        // arrives on the last chunk; partial chunks never emit a packet.
        let mut decoded = None;
        for chunk in wire[..len].chunks_mut(16) {
            if let Some(pkt) = runner.decode_host_frame(chunk) {
                decoded = Some(pkt);
            }
        }
        decoded
    } else {
        runner.decode_host_frame(&mut wire[..len])
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
    /// Host commands are sampled once per rendered frame, encoded with the real
    /// `encode_cobs`, decoded through the same `CobsStreamFramer` / `decode_rx`
    /// path firmware uses, then repeated on every tick so the deadman watchdog
    /// behaves as it does on the 200 Hz wire; one-shot `ConfigCmd`s are applied
    /// only on the first tick. Injected CRC/version/truncation/fragmentation
    /// faults corrupt the wire before decoding, and the resulting codec errors
    /// reach telemetry via the shared mailbox. The heartbeat-drop fault still
    /// suppresses all host commands for the frame, matching the legacy
    /// `step_exchange` behaviour. MCU telemetry round-trips through COBS in
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

        // Snapshot the wire knobs before taking `&mut self.runner` for decoding.
        let wire_faults = {
            let f = self.runner.plant.faults();
            WireRxFaults::from_injection(f)
        };

        // The wire is the single source of truth: even a clean frame is encoded
        // and decoded before a typed command reaches the harness. A rejected
        // frame supplies no command at all (same as a dropped heartbeat), so
        // the control loop keeps ticking and the deadman logic can react.
        let decoded = if dropped {
            None
        } else {
            let mut wire = [0u8; rk_proto::MAX_COBS_FRAME_BYTES];
            decode_wire_host_cmd(&mut self.runner, host_pkt, wire_faults, &mut wire)
        };

        let mut cmds = Vec::with_capacity(steps);
        if let Some(first) = decoded {
            let mut later = first;
            later.config = ConfigCmd::None;
            cmds.push(first);
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

    fn sim_led_on(&self) -> bool {
        self.runner.sim_led_on()
    }

    fn reset_experiment(&mut self) {
        self.runner.reset_experiment();
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
    use rk_sim::verify_deterministic_replay;
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

    /// A clean COBS frame must decode bit-exactly to the sampled packet, and
    /// the fixed-step path must remain replay-exact through the real wire.
    #[test]
    fn step_fixed_clean_wire_frame_is_bit_exact_and_replay_exact() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();

        let mut target_rad = [0.0_f32; MAX_JOINTS];
        for (i, value) in [0.1_f32, -0.2, 0.3, -0.4, 0.5, -0.6].iter().enumerate() {
            target_rad[i] = *value;
        }
        let pkt = HostToMcu {
            seq: 42,
            mode: ControlMode::Drive,
            torque_mask: 0b11_1111,
            speed_limit_pct: 33,
            target_rad,
            command_mode: [1; MAX_JOINTS],
            velocity_rad_s: [0.25; MAX_JOINTS],
            config: ConfigCmd::SetZeroOffset {
                joint: 2,
                zero_rad: 0.3,
            },
            ..Default::default()
        };

        // Decode the same bytes the fixed-step path uses, directly through the
        // runner's shared stream framer.
        let mut wire = [0u8; rk_proto::MAX_COBS_FRAME_BYTES];
        let decoded = decode_wire_host_cmd(&mut sim.runner, &pkt, WireRxFaults::default(), &mut wire);
        assert_eq!(decoded, Some(pkt));
        assert_eq!(sim.runner.wire_error_counts(), (0, 0));

        sim.start_recording();
        let telem = sim.step_fixed(4, &pkt, 0.020).expect("telemetry frame");
        assert_eq!(telem.active_mode, ControlMode::Drive);
        assert_eq!(telem.rx_crc_errors, 0);
        assert_eq!(telem.rx_version_mismatches, 0);

        let log = sim.stop_recording().expect("recorded replay");
        assert_eq!(log.ticks.len(), 4);
        assert_eq!(log.ticks[0].rx_cmd, Some(pkt));
        let mut later = pkt;
        later.config = ConfigCmd::None;
        for tick in &log.ticks[1..] {
            assert_eq!(tick.rx_cmd, Some(later));
            assert_eq!(tick.wire_errors, (0, 0));
        }
        let check = verify_deterministic_replay(&log);
        assert!(check.exact_match, "fixed-step wire replay must be exact: {check:?}");
        assert_eq!(check.matched_ticks, check.total_ticks);
    }

    /// A flipped COBS byte must increment `rx_crc_errors` in telemetry and the
    /// next clean tick must still deliver its command (no framer cross-talk).
    #[test]
    fn step_fixed_crc_fault_counts_error_and_recovers() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();

        let pkt = HostToMcu {
            seq: 9,
            ..Default::default()
        };
        sim.sim_faults_mut().expect("sim faults").rx_crc_error = true;
        let telem = sim.step_fixed(2, &pkt, 0.010).expect("telemetry frame");
        assert_eq!(telem.rx_crc_errors, 1, "CRC fault must reach kernel telemetry");
        assert_eq!(telem.rx_version_mismatches, 0);
        assert!(sim.runner.link_errors() >= 1, "framer must record the bad frame");
        let log = sim.recorded_log().expect("recording active");
        assert!(log.ticks.iter().all(|t| t.rx_cmd.is_none()));

        sim.sim_faults_mut().expect("sim faults").rx_crc_error = false;
        let clean = HostToMcu {
            seq: 10,
            ..Default::default()
        };
        let next = sim.step_fixed(1, &clean, 0.005).expect("telemetry frame");
        assert_eq!(next.rx_crc_errors, 1, "counts are cumulative, not reset");
        let log = sim.recorded_log().expect("recording active");
        assert_eq!(log.ticks.last().expect("tick").rx_cmd, Some(clean));
    }

    /// A wrong `protocol_version` on the wire must increment only the version
    /// mismatch counter, without a CRC or framing error.
    #[test]
    fn step_fixed_version_fault_counts_only_version_mismatch() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();

        let pkt = HostToMcu {
            seq: 11,
            ..Default::default()
        };
        sim.sim_faults_mut().expect("sim faults").rx_version_mismatch = true;
        let telem = sim.step_fixed(1, &pkt, 0.005).expect("telemetry frame");
        assert_eq!(telem.rx_version_mismatches, 1);
        assert_eq!(telem.rx_crc_errors, 0);
        assert!(sim.runner.link_errors() >= 1);
        let log = sim.recorded_log().expect("recording active");
        assert!(log.ticks.last().expect("tick").rx_cmd.is_none());

        sim.sim_faults_mut()
            .expect("sim faults")
            .rx_version_mismatch = false;
        let clean = HostToMcu {
            seq: 12,
            ..Default::default()
        };
        let next = sim.step_fixed(1, &clean, 0.005).expect("telemetry frame");
        assert_eq!(next.rx_version_mismatches, 1);
        let log = sim.recorded_log().expect("recording active");
        assert_eq!(log.ticks.last().expect("tick").rx_cmd, Some(clean));
    }

    /// A truncated frame must be counted as a framing error, never panic, and
    /// must not poison the next clean frame.
    #[test]
    fn step_fixed_truncated_fault_counts_frame_error_and_recovers() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();

        let pkt = HostToMcu {
            seq: 13,
            ..Default::default()
        };
        sim.sim_faults_mut().expect("sim faults").rx_truncated_frame = true;
        let telem = sim.step_fixed(1, &pkt, 0.005).expect("telemetry frame");
        assert_eq!(telem.rx_crc_errors, 0);
        assert_eq!(telem.rx_version_mismatches, 0);
        assert!(sim.runner.link_errors() >= 1, "framer must count the short frame");
        let log = sim.recorded_log().expect("recording active");
        assert!(log.ticks.last().expect("tick").rx_cmd.is_none());

        sim.sim_faults_mut().expect("sim faults").rx_truncated_frame = false;
        let clean = HostToMcu {
            seq: 14,
            ..Default::default()
        };
        let next = sim.step_fixed(1, &clean, 0.005).expect("telemetry frame");
        assert_eq!(next.rx_crc_errors, 0);
        assert_eq!(next.rx_version_mismatches, 0);
        let log = sim.recorded_log().expect("recording active");
        assert_eq!(log.ticks.last().expect("tick").rx_cmd, Some(clean));
    }

    /// Splitting a frame across several framer pushes must still deliver the
    /// command, with no wire errors and an exact replay.
    #[test]
    fn step_fixed_fragmented_fault_reassembles_command() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();

        let pkt = HostToMcu {
            seq: 15,
            mode: ControlMode::Drive,
            torque_mask: 0b11,
            target_rad: [0.2; MAX_JOINTS],
            ..Default::default()
        };
        let mut wire = [0u8; rk_proto::MAX_COBS_FRAME_BYTES];
        let decoded = decode_wire_host_cmd(
            &mut sim.runner,
            &pkt,
            WireRxFaults {
                fragmented: true,
                ..Default::default()
            },
            &mut wire,
        );
        assert_eq!(decoded, Some(pkt));
        assert_eq!(sim.runner.wire_error_counts(), (0, 0));

        sim.sim_faults_mut().expect("sim faults").rx_fragmented_frame = true;
        sim.start_recording();
        let telem = sim.step_fixed(3, &pkt, 0.015).expect("telemetry frame");
        assert_eq!(telem.active_mode, ControlMode::Drive);
        assert_eq!(sim.runner.wire_error_counts(), (0, 0));

        let log = sim.stop_recording().expect("recorded replay");
        assert_eq!(log.ticks[0].rx_cmd, Some(pkt));
        let mut later = pkt;
        later.config = ConfigCmd::None;
        for tick in &log.ticks[1..] {
            assert_eq!(tick.rx_cmd, Some(later));
        }
        assert!(verify_deterministic_replay(&log).exact_match);
    }

    /// Heartbeat-drop suppression still wins over wire faults: no frame is put
    /// on the wire, so no wire error is counted for that frame.
    #[test]
    fn step_fixed_dropped_heartbeat_still_suppresses_wire_faults() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut sim = sim();
        sim.reset_mock_clock();
        sim.start_recording();

        let pkt = HostToMcu {
            seq: 16,
            ..Default::default()
        };
        {
            let faults = sim.sim_faults_mut().expect("sim faults");
            faults.drop_heartbeat = true;
            faults.rx_crc_error = true;
            faults.rx_version_mismatch = true;
        }
        assert!(sim.step_fixed(2, &pkt, 0.010).is_none());
        assert_eq!(sim.runner.wire_error_counts(), (0, 0));
        let log = sim.recorded_log().expect("recording active");
        assert!(log.ticks.iter().all(|t| t.rx_cmd.is_none()));
    }
}
