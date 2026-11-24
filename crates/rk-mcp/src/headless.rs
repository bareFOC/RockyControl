//! Headless `McpHost` backed directly by `StudioModel` + `RobotTransport` for native CLI & tests.

use std::collections::VecDeque;
#[cfg(not(target_arch = "wasm32"))]
use std::path::Path;
use std::path::PathBuf;

use rk_link::{FIXED_CONTROL_STEP_SEC, RobotTransport, SimTransport, TransportKind};
use rk_model::{AssetBundle, StudioModel};
use rk_proto::ControlMode;

use crate::host::{McpActivityEntry, McpHost, McpPolicy};
use crate::tools::query_mcu_model_summary;

/// Optional canonical directory confinement matching `storytold/craft-contrib`
/// `--automation-read-root` and `--automation-write-root` flags.
#[derive(Clone, Debug, Default)]
pub struct AutomationRoots {
    pub read_root: Option<PathBuf>,
    pub write_root: Option<PathBuf>,
}

impl AutomationRoots {
    /// Resolves `rel_or_abs` strictly inside `root` after canonicalizing existing path prefixes.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn resolve_read(&self, path: &str) -> Result<PathBuf, String> {
        let Some(root) = &self.read_root else {
            return Err("File reading disabled: pass --automation-read-root <dir> to allow reading URDF files from disk.".to_string());
        };
        let canon_root = std::fs::canonicalize(root)
            .map_err(|e| format!("Invalid --automation-read-root '{}': {e}", root.display()))?;
        let candidate = if Path::new(path).is_absolute() {
            PathBuf::from(path)
        } else {
            canon_root.join(path)
        };
        let canon_file = std::fs::canonicalize(&candidate)
            .map_err(|e| format!("Cannot read '{}': {e}", candidate.display()))?;
        if !canon_file.starts_with(&canon_root) {
            return Err(format!(
                "Path '{}' escapes --automation-read-root '{}'",
                path,
                canon_root.display()
            ));
        }
        Ok(canon_file)
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn resolve_write(&self, path: &str) -> Result<PathBuf, String> {
        let Some(root) = &self.write_root else {
            return Err("File writing disabled: pass --automation-write-root <dir> to allow exporting files to disk.".to_string());
        };
        let canon_root = std::fs::canonicalize(root)
            .map_err(|e| format!("Invalid --automation-write-root '{}': {e}", root.display()))?;
        let candidate = if Path::new(path).is_absolute() {
            PathBuf::from(path)
        } else {
            canon_root.join(path)
        };
        let parent = candidate
            .parent()
            .ok_or_else(|| "Output path must include a filename".to_string())?;
        let file_name = candidate
            .file_name()
            .ok_or_else(|| "Output path must include a filename".to_string())?;
        let canon_parent = std::fs::canonicalize(parent)
            .map_err(|e| format!("Output directory '{}' does not exist: {e}", parent.display()))?;
        let final_path = canon_parent.join(file_name);
        if !final_path.starts_with(&canon_root) {
            return Err(format!(
                "Path '{}' escapes --automation-write-root '{}'",
                path,
                canon_root.display()
            ));
        }
        Ok(final_path)
    }
}

/// Standalone headless host driving `StudioModel` + `RobotTransport` without a GUI window.
pub struct HeadlessHost {
    model: StudioModel,
    transport: Box<dyn RobotTransport + Send>,
    connected: bool,
    screen: &'static str,
    policy: McpPolicy,
    #[allow(dead_code)]
    roots: AutomationRoots,
    pub activity: VecDeque<McpActivityEntry>,
}

impl HeadlessHost {
    pub fn new(policy: McpPolicy, roots: AutomationRoots) -> Self {
        let model = StudioModel::new_default();
        let transport: Box<dyn RobotTransport + Send> = Box::new(SimTransport::new(
            model.rig.joints.len(),
            &model.calibration,
            model.rig.default_pose(),
        ));
        Self {
            model,
            transport,
            connected: false,
            screen: "dashboard",
            policy,
            roots,
            activity: VecDeque::with_capacity(64),
        }
    }
}

impl McpHost for HeadlessHost {
    fn model(&self) -> &StudioModel {
        &self.model
    }

    fn model_mut(&mut self) -> &mut StudioModel {
        &mut self.model
    }

    fn transport(&self) -> &dyn RobotTransport {
        &*self.transport
    }

    fn transport_mut(&mut self) -> &mut dyn RobotTransport {
        &mut *self.transport
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn connect(&mut self, kind: TransportKind) -> Result<String, String> {
        self.model.set_authority_mode(ControlMode::Teach);
        self.model.walk_cmd.walk_enabled = false;
        self.model.walk_cmd.vx = 0.0;
        self.model.walk_cmd.vy = 0.0;
        self.model.walk_cmd.yaw_rate = 0.0;

        match kind {
            TransportKind::SimKernel => {
                let sim = SimTransport::new(
                    self.model.rig.joints.len().max(1),
                    &self.model.calibration,
                    self.model.rig.default_pose(),
                );
                self.transport = Box::new(sim);
                self.connected = true;
                self.step_ticks(2, 0.01);
                Ok("Connected to Simulated Rocky MCU Twin in safe Teach mode.".to_string())
            }
            #[cfg(not(target_arch = "wasm32"))]
            TransportKind::UsbNetworkingUdp => {
                let udp = rk_link::UdpTransport::connect("10.42.0.2:7777")
                    .map_err(|e| format!("USB-Net UDP bind failed: {e}"))?;
                self.transport = Box::new(udp);
                self.connected = true;
                Ok("Connected RP2350 USB-Net UDP -> 10.42.0.2:7777 (Teach mode).".to_string())
            }
            _ => Err(format!("Transport '{}' is not available in this build.", kind.label())),
        }
    }

    fn disconnect(&mut self) {
        self.model.set_authority_mode(ControlMode::Teach);
        self.model.walk_cmd.walk_enabled = false;
        self.model.walk_cmd.vx = 0.0;
        self.model.walk_cmd.vy = 0.0;
        self.model.walk_cmd.yaw_rate = 0.0;
        self.connected = false;
        self.screen = "dashboard";
    }

    fn load_bundle(&mut self, bundle: AssetBundle) -> Result<String, String> {
        self.model.load_bundle(bundle)?;
        if self.transport.kind() == TransportKind::SimKernel {
            let sim = SimTransport::new(
                self.model.rig.joints.len().max(1),
                &self.model.calibration,
                self.model.rig.default_pose(),
            );
            self.transport = Box::new(sim);
            if self.connected {
                self.step_ticks(2, 0.01);
            }
        }
        Ok(self.model.status_message.clone())
    }

    fn sync_mcu_model(&mut self) -> Result<String, String> {
        if !self.connected {
            return Err("Connect to a robot first via robot_connect.".to_string());
        }
        query_mcu_model_summary(&mut *self.transport)
    }

    fn active_screen_name(&self) -> &'static str {
        self.screen
    }

    fn navigate_screen(&mut self, screen: &str) -> Result<(), String> {
        self.screen = match screen {
            "dashboard" => "dashboard",
            "control" => "control",
            "config" => "config",
            other => return Err(format!("unknown screen `{other}`")),
        };
        Ok(())
    }

    fn set_camera_preset(&mut self, _preset: &str) -> Result<(), String> {
        Ok(())
    }

    fn apply_joint_angle(&mut self, joint_idx: usize, rad: f32) -> Result<(), String> {
        if joint_idx >= self.model.rig.joints.len() {
            return Err(format!("joint index {joint_idx} out of range"));
        }
        let cal = self.model.calibration[joint_idx];
        let clamped = rad.clamp(cal.min_rad, cal.max_rad);
        self.model.selected_joint = joint_idx;
        self.model.playback = None;
        if self.model.requested_mode == ControlMode::Drive {
            self.model.target_rad[joint_idx] = clamped;
        } else {
            let raw = rk_control::calibrated_to_raw(clamped, &cal);
            self.transport.set_virtual_hand_angle(joint_idx, raw);
            self.model.target_rad[joint_idx] = clamped;
        }
        Ok(())
    }

    fn set_walk_override(&mut self, vx: f32, vy: f32, yaw_rate: f32, enabled: bool) {
        self.model.walk_cmd.vx = vx.clamp(-1.0, 1.0);
        self.model.walk_cmd.vy = vy.clamp(-1.0, 1.0);
        self.model.walk_cmd.yaw_rate = yaw_rate.clamp(-1.0, 1.0);
        self.model.walk_cmd.walk_enabled = enabled;
    }

    fn step_ticks(&mut self, ticks: usize, dt_sec: f32) {
        if ticks == 0 {
            return;
        }
        // One `step_fixed` call keeps a single deterministic harness/executor
        // alive for all `ticks`. Stepping one tick at a time would spawn one
        // executor per tick against the process-global Embassy mock clock and
        // leave stale timer wakers behind. Advance playback by the same total
        // time as the old per-tick packet building.
        let pkt = self.model.build_next_host_packet(dt_sec * ticks as f32);
        if let Some(telem) = self
            .transport
            .step_fixed(ticks, &pkt, FIXED_CONTROL_STEP_SEC * ticks as f32)
        {
            self.model.ingest_telemetry(telem);
        }
    }

    fn policy(&self) -> McpPolicy {
        self.policy
    }

    fn set_drive_armed(&mut self, armed: bool) {
        self.policy.drive_armed = armed;
    }

    fn trigger_estop_and_disarm(&mut self) {
        self.policy.drive_armed = false;
        self.model.trigger_estop();
    }

    fn reset_estop_upright(&mut self) {
        if let Some(f) = self.transport.sim_faults_mut() {
            f.drop_heartbeat = false;
        }
        self.transport.reset_upright();
        self.model.reset_estop();
    }

    fn record_activity(&mut self, entry: McpActivityEntry) {
        self.activity.push_back(entry);
        while self.activity.len() > 64 {
            self.activity.pop_front();
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn read_confined_file(&self, path: &str) -> Result<Vec<u8>, String> {
        let safe_path = self.roots.resolve_read(path)?;
        std::fs::read(&safe_path).map_err(|e| format!("Failed reading '{}': {e}", safe_path.display()))
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn write_confined_file(&self, path: &str, bytes: &[u8]) -> Result<String, String> {
        let safe_path = self.roots.resolve_write(path)?;
        std::fs::write(&safe_path, bytes)
            .map_err(|e| format!("Failed writing '{}': {e}", safe_path.display()))?;
        Ok(safe_path.display().to_string())
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use crate::test_support::TEST_LOCK;

    #[test]
    fn automation_roots_confine_paths_and_block_traversal() {
        let base = std::env::temp_dir().join("rk_mcp_root_test");
        let _ = std::fs::create_dir_all(&base);
        let inside = base.join("sample.urdf");
        std::fs::write(&inside, b"<robot name=\"ok\"/>").unwrap();

        let roots = AutomationRoots {
            read_root: Some(base.clone()),
            write_root: Some(base.clone()),
        };
        assert!(roots.resolve_read("sample.urdf").is_ok());
        assert!(roots.resolve_read("../etc/passwd").is_err());

        let _ = std::fs::remove_file(&inside);
        let _ = std::fs::remove_dir_all(&base);
    }

    #[test]
    fn step_ticks_records_fixed_5ms_ticks_and_keeps_augmented_telemetry() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        host.connect(TransportKind::SimKernel).expect("sim connect");

        host.transport_mut().start_recording();
        host.step_ticks(20, 0.01);
        let log = host
            .transport_mut()
            .stop_recording()
            .expect("fixed ticks were recorded");

        assert_eq!(log.ticks.len(), 20);
        for pair in log.ticks.windows(2) {
            assert_eq!(pair[0].dt_sec, FIXED_CONTROL_STEP_SEC);
            assert_eq!(pair[1].now_ms - pair[0].now_ms, 5);
        }
        assert_eq!(log.ticks.last().unwrap().dt_sec, FIXED_CONTROL_STEP_SEC);

        // Recorded ticks stay kernel-exact; the telemetry returned to the host is
        // still augmented with the plant's WORLD/GRF channels.
        let valid = host.model().latest_telemetry.estimator.valid_mask;
        assert_ne!(valid & rk_proto::estimator_valid::WORLD, 0);
        assert_ne!(valid & rk_proto::estimator_valid::GRF, 0);
    }
}
