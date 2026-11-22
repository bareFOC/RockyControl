//! Layer 4 — `RobotTransport` trait & Stage 1 Sim / Stage 2 Serial backends (`humanoid-link`).

use humanoid_control::JointCalibration;
use humanoid_proto::{HostToMcu, MAX_JOINTS, McuToHost, ModelFormat, ModelRequest, ModelResponse};
use humanoid_sim::{CopperReplayLog, CopperSimRunner, SimFaultInjection};

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
    fn step_exchange(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> Option<McuToHost>;
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
    rate_hz_smooth: f32,
    rtt_ms: f32,
}

impl SimTransport {
    pub fn new(
        joint_count: usize,
        initial_cal: &[JointCalibration],
        initial_raw_pose: [f32; MAX_JOINTS],
    ) -> Self {
        Self {
            runner: CopperSimRunner::new(joint_count, initial_cal, initial_raw_pose),
            rate_hz_smooth: 60.0,
            rtt_ms: 1.4,
        }
    }
}

impl RobotTransport for SimTransport {
    fn kind(&self) -> TransportKind {
        TransportKind::SimKernel
    }

    fn step_exchange(&mut self, dt_sec: f32, host_pkt: &HostToMcu) -> Option<McuToHost> {
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
/// `firmware/humanoid-rp2350` (`--no-default-features --features usb-net,bench-loopback`).
#[cfg(not(target_arch = "wasm32"))]
pub struct UdpTransport {
    socket: std::net::UdpSocket,
    remote_addr: std::net::SocketAddr,
    tx_buf: [u8; humanoid_proto::MAX_COBS_FRAME_BYTES],
    rx_buf: [u8; humanoid_proto::MAX_COBS_FRAME_BYTES],
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
            tx_buf: [0u8; humanoid_proto::MAX_COBS_FRAME_BYTES],
            rx_buf: [0u8; humanoid_proto::MAX_COBS_FRAME_BYTES],
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
        if let Ok(encoded) = humanoid_proto::encode_cobs(host_pkt, &mut self.tx_buf) {
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
                    match humanoid_proto::decode_cobs::<McuToHost>(&mut self.rx_buf[..n]) {
                        Ok((telem, _)) => latest = Some(telem),
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
        let mut tx = [0u8; humanoid_proto::MAX_MODEL_COBS_FRAME_BYTES];
        let wire = humanoid_proto::encode_model_req_cobs(&req, &mut tx).ok()?;
        sock.send_to(wire, model_addr).ok()?;
        let mut rx = [0u8; humanoid_proto::MAX_MODEL_COBS_FRAME_BYTES];
        let (n, _) = sock.recv_from(&mut rx).ok()?;
        humanoid_proto::decode_model_resp_cobs(&mut rx[..n]).ok()
    }
}
