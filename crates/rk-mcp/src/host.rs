//! Platform-agnostic host facade (`McpHost`) and Guarded Autonomy policy (`McpPolicy`).

use rk_link::{RobotTransport, TransportKind};
use rk_model::{AssetBundle, StudioModel};
use serde::{Deserialize, Serialize};

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
