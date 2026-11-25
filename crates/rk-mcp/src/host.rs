//! Platform-agnostic host facade (`McpHost`) and Guarded Autonomy policy (`McpPolicy`).

use rk_fw_core::replay::{ReplayVerification, verify_deterministic_replay};
use rk_link::{RobotTransport, TransportKind};
use rk_model::{AssetBundle, StudioModel};
use rk_proto::ConfigCmd;
use rk_sim::{CopperSimRunner, ExperimentScript, WalkQuality};
use serde::{Deserialize, Serialize};

/// Outcome of a scripted experiment: the bit-exact replay verdict together with
/// the walk-quality metrics accumulated by the simulator during that run.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct ExperimentResult {
    pub verification: ReplayVerification,
    pub quality: Option<WalkQuality>,
}

/// Guarded autonomy policy controlling external MCP agent permissions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct McpPolicy {
    /// Master MCP enable switch (enabled by default; toggleable from Dashboard Home screen).
    pub enabled: bool,
    /// Opt-in operator guard for energized `Drive` mode and active locomotion (defaults to `false`;
    /// automatically reset to `false` whenever E-Stop triggers).
    pub drive_armed: bool,
}

impl Default for McpPolicy {
    fn default() -> Self {
        Self {
            enabled: true,
            drive_armed: false,
        }
    }
}

/// Outcome classification recorded in the live MCP activity log.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum McpCallOutcome {
    Ok,
    Guarded,
    Error,
}

impl McpCallOutcome {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Guarded => "GUARDED",
            Self::Error => "ERR",
        }
    }
}

/// One timestamped audit entry for the Rocky Studio MCP drawer activity feed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct McpActivityEntry {
    pub seq: u64,
    pub method: String,
    pub summary: String,
    pub outcome: McpCallOutcome,
    pub detail: String,
}

/// Bridge trait implemented by both live `RockyStudioApp` (3D browser/desktop UI)
/// and standalone `HeadlessHost` (`rk-mcp` CLI binary).
pub trait McpHost {
    fn model(&self) -> &StudioModel;
    fn model_mut(&mut self) -> &mut StudioModel;

    fn transport(&self) -> &dyn RobotTransport;
    fn transport_mut(&mut self) -> &mut dyn RobotTransport;

    /// Accumulated walk-quality metrics of the active transport, if it measures
    /// them. Mirrors [`RobotTransport::walk_quality`] for host implementations.
    fn walk_quality(&self) -> Option<WalkQuality> {
        self.transport().walk_quality()
    }

    fn is_connected(&self) -> bool;
    fn connect(&mut self, kind: TransportKind) -> Result<String, String>;
    fn disconnect(&mut self);

    fn load_bundle(&mut self, bundle: AssetBundle) -> Result<String, String>;
    fn sync_mcu_model(&mut self) -> Result<String, String>;

    fn active_screen_name(&self) -> &'static str;
    fn navigate_screen(&mut self, screen: &str) -> Result<(), String>;
    fn set_camera_preset(&mut self, preset: &str) -> Result<(), String>;

    fn apply_joint_angle(&mut self, joint_idx: usize, rad: f32) -> Result<(), String>;
    fn set_walk_override(&mut self, vx: f32, vy: f32, yaw_rate: f32, enabled: bool);
    /// Advances the simulation by `ticks` deterministic fixed 5 ms control steps
    /// (200 Hz), matching the GUI's shared fixed-step control path.
    ///
    /// `dt_sec` is the caller-provided per-tick value used to build the host
    /// packet (`dt_sec * ticks` advances action playback by the same total time);
    /// it does not change the fixed 5 ms control tick duration.
    fn step_ticks(&mut self, ticks: usize, dt_sec: f32);

    fn policy(&self) -> McpPolicy;
    fn set_drive_armed(&mut self, armed: bool);
    fn trigger_estop_and_disarm(&mut self);
    fn reset_estop_upright(&mut self);

    /// Restarts the simulation from clean construction inputs: fresh control
    /// kernel, plant, fault flags, wire state and recorder, plus cleared host
    /// history where the host owns any.
    ///
    /// The default asks the simulation transport to reset in place, preserving
    /// its persistent Embassy executor. The GUI bridge overrides this to also
    /// drop its telemetry history. Host packet sequencing is reset in both
    /// paths so scripted runs do not inherit frame-derived `tx_seq` values.
    /// Wired transports do not support a reset.
    fn reset_experiment(&mut self) -> Result<String, String> {
        if self.transport().kind() != TransportKind::SimKernel {
            return Err("Reset experiment is only supported on the Simulation transport.".to_string());
        }
        self.transport_mut().reset_experiment();
        self.model_mut().reset_experiment_state();
        Ok("Simulation experiment reset: kernel, plant, faults, wire, recorder and host sequencing are clean.".to_string())
    }

    /// Executes a deterministic [`ExperimentScript`] on a fresh software twin
    /// and verifies the resulting `.copper` log bit-exactly.
    ///
    /// The script owns its whole input stream (typed host packets, timed plant
    /// events and PRNG seed), so this path never builds frame-derived packets.
    /// Host-side sequencing is reset first to keep seq values independent of
    /// prior UI activity. The default works for both the headless CLI host and
    /// the live GUI bridge without requiring transport-specific access.
    ///
    /// The script is cloned before use: tick 0 is rewritten with the host's
    /// current tuned `BalanceGains` (`ConfigCmd::SetBalanceGains`) so scripted
    /// runs exercise the same gains as live tuning, and caller-owned scripts are
    /// never mutated. A config already on packet 0 is replaced (there is one
    /// config slot per packet), so other configs belong on later ticks. The
    /// response carries both replay verification and the run's walk quality.
    fn run_experiment(&mut self, script: &ExperimentScript) -> Result<ExperimentResult, String> {
        if self.transport().kind() != TransportKind::SimKernel {
            return Err("Scripted experiments require the Simulation transport.".to_string());
        }
        self.model_mut().reset_experiment_state();
        let model = self.model();
        let gains = model.balance_gains;
        let joint_count = model.rig.joints.len().max(1);
        let calibration = model.calibration.clone();
        let pose = model.rig.default_pose();

        let mut script = script.clone();
        if let Some(first) = script.packets.first_mut() {
            first.config = ConfigCmd::SetBalanceGains(gains);
        }

        let mut runner = CopperSimRunner::new(joint_count, &calibration, pose);
        let log = runner.run_script(&script);
        Ok(ExperimentResult {
            verification: verify_deterministic_replay(&log),
            quality: Some(runner.last_walk_quality()),
        })
    }

    fn record_activity(&mut self, entry: McpActivityEntry);

    /// Optional canonical file read confined to `--automation-read-root` (native CLI only).
    fn read_confined_file(&self, _path: &str) -> Result<Vec<u8>, String> {
        Err("File-path reading is only available on the native rk-mcp CLI with --automation-read-root.".to_string())
    }

    /// Optional canonical file write confined to `--automation-write-root` (native CLI only).
    fn write_confined_file(&self, _path: &str, _bytes: &[u8]) -> Result<String, String> {
        Err("File-path writing is only available on the native rk-mcp CLI with --automation-write-root.".to_string())
    }
}
