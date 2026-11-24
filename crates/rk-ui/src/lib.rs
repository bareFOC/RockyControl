//! Layer 5 — Rocky Studio GUI (`rk-ui`).
//!
//! Unified touch-first 3-screen app shared identically across Desktop, Browser WASM, and Tablet:
//! - [`dashboard_screen`]: Home screen (Connect to Robot -> Live Robot Stats + Control / Config Launchers)
//! - [`control_screen`]: Full-bleed 3D robot with Walk Pad, WASD/Gamepad, MCU Upper-Body Vertical Balance, and Combat Shove tests
//! - [`config_screen`]: Visual 3D direct joint/arm dragging (FK & `k` Limb IK), 3D protractor arc, floating visual joint config bar, and on-demand 3-tab setup drawer
//! - [`top_bar`]: Contextual navigation, Teach/Drive authority toggle, always-pinned E-Stop, and safety banners
//! - [`urdf_panel`], [`calibration_tab`], [`sim_tab`]: Focused engineering tabs inside the Config & Calibration drawer
//! - [`responsive`]: Unified touch-first ergonomics adapting automatically between Landscape and Portrait

mod action_studio;
mod calibration_tab;
mod config_screen;
mod control_screen;
mod dashboard_screen;
mod mcp_bridge;
mod mcp_drawer;
mod responsive;
mod sim_tab;
mod top_bar;
mod urdf_panel;

use std::collections::VecDeque;

use action_studio::ActionCategoryFilter;

use craft_keymap::{Active, Bindable, Binding, Keymap, Scope, Scopes};
use craft_ui::theme::WidgetPalette;
use egui::{Color32, CornerRadius, Key, RichText, Stroke, Vec2};
pub use mcp_bridge::McpUiState;
#[cfg(not(target_arch = "wasm32"))]
pub use mcp_bridge::NativeMcpListenerStatus;
use rk_link::{RobotTransport, SimTransport, TransportKind};
use rk_model::{AssetBundle, StudioModel};
use rk_fw_core::replay::ReplayVerification;
use rk_proto::{ControlMode, FallOrientation, FaultFlags, ModelFormat};
use rk_viewport::{HumanoidViewport, ViewportInteraction};
pub use responsive::{FormFactor, UiMetrics};
use responsive::apply_responsive_style;
use urdf_panel::McuModelSyncState;
use web_time::Instant;

const TELEMETRY_HISTORY_LEN: usize = 180;
const CMD_ESTOP: &str = "safety.estop";

/// Fixed control period of the shared Embassy task (5 ms / 200 Hz).
const CONTROL_STEP_SEC: f32 = rk_fw_core::config::CONTROL_DT_SEC;
/// Bounded catch-up: at most this many fixed ticks are executed in one frame.
const MAX_CONTROL_STEPS_PER_FRAME: usize = 4;
/// Upper bound on buffered frame time (~10 ticks); excess is dropped so a long
/// stall (tab backgrounded, debugger pause) cannot trigger a catch-up spiral.
const MAX_CONTROL_ACCUM_SEC: f32 = 0.05;
/// Sustained low-angular-rate settle dwell required after a fall before recovery starts.
const RECOVERY_STABLE_DWELL_SEC: f32 = 0.45;
/// Maximum pitch/roll angular rate (rad/s) considered settled on the floor or upright.
const RECOVERY_SETTLED_RATE_RAD_S: f32 = 0.35;
/// Temporary tilt cutoff (rad, ~77 deg) while pushing off the ground during get-up playback.
/// Still trips immediately on a violent re-shove or operator E-Stop.
const RECOVERY_PLAYBACK_CUTOFF_RAD: f32 = 1.35;

/// Ordered, multi-frame fall-recovery sequence owned by [`RockyStudioApp`].
///
/// Every transition is driven by fresh telemetry in `update_fall_watcher()` so
/// no safety interlock is bypassed in a single frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RecoveryState {
    #[default]
    Idle,
    /// A `TILT_FALL_CUTOFF` (or E-Stop during playback) has been latched.
    FallLatched,
    /// `ClearFaults` queued via `reset_estop()`; waiting for the fault to clear in telemetry.
    ClearingFault,
    /// Fault clear observed; `Drive` requested, waiting one control frame before playing.
    ArmingDrive,
    /// Get-up keyframes are playing with torque enabled.
    PlayingGetUp,
    /// Playback finished; returning to `Teach` and waiting for a stable upright stance.
    Completed,
}

impl RecoveryState {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::FallLatched => "FALL LATCHED",
            Self::ClearingFault => "CLEARING FAULT",
            Self::ArmingDrive => "ARMING DRIVE",
            Self::PlayingGetUp => "GETTING UP",
            Self::Completed => "COMPLETED",
        }
    }
}

#[cfg(target_arch = "wasm32")]
thread_local! {
    static PENDING_WASM_DROPS: std::cell::RefCell<Vec<(String, Vec<u8>)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// Top-level app navigation screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppScreen {
    Dashboard,
    Control,
    Config,
}

/// Focused 4-tab engineering drawer inside the Config & Calibration screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigDrawerTab {
    BalanceGait,
    Actions,
    Joints,
    Setup,
}

impl ConfigDrawerTab {
    pub const fn label(self) -> &'static str {
        match self {
            Self::BalanceGait => "Balance",
            Self::Actions => "Actions",
            Self::Joints => "Joints",
            Self::Setup => "Setup",
        }
    }
}

/// Preset combat push test impulse intensities for verifying upper-body vertical balance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PushStrength {
    Jab,
    Shove,
    Heavy,
}

impl PushStrength {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Jab => "Jab",
            Self::Shove => "Shove",
            Self::Heavy => "Heavy",
        }
    }

    pub const fn impulse_rad_s(self) -> f32 {
        match self {
            Self::Jab => 1.05,
            Self::Shove => 1.65,
            Self::Heavy => 2.35,
        }
    }
}

/// Platform file I/O hook supplied by `apps/rk-desktop` (`craft-atomic` + native picker)
/// or `apps/rk-web` (`craft-platform::browser` + pure-Rust `web-sys` `<input type="file">`).
pub trait PlatformSaver {
    fn save_file(&mut self, filename: &str, mime: &str, bytes: &[u8]) -> Result<String, String>;

    /// Opens the platform's file picker for `.urdf`, `.xml`, or `.zip` robot bundles.
    fn request_open_urdf_file(&mut self) -> Result<(), String> {
        Err("File picker not configured on this platform — drag & drop a .urdf/.zip file instead."
            .to_string())
    }

    /// Drains any files chosen asynchronously via `request_open_urdf_file`.
    fn poll_opened_files(&mut self) -> Vec<(String, Vec<u8>)> {
        Vec::new()
    }

    /// Returns true if the host desktop window or browser document is currently fullscreen.
    fn is_fullscreen(&self, _ctx: &egui::Context) -> bool {
        false
    }

    /// Toggles fullscreen mode on the host window or browser document.
    fn toggle_fullscreen(&mut self, _ctx: &egui::Context) -> Result<bool, String> {
        Err("Fullscreen mode is not supported on this platform.".to_string())
    }
}

#[derive(Default)]
pub struct NoopSaver;

impl PlatformSaver for NoopSaver {
    fn save_file(&mut self, filename: &str, _mime: &str, _bytes: &[u8]) -> Result<String, String> {
        Ok(format!("Prepared '{filename}'"))
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TelemetryPoint {
    pub(crate) t_sec: f64,
    #[allow(dead_code)]
    pub(crate) err_deg: f64,
    #[allow(dead_code)]
    pub(crate) current_ma: f64,
    #[allow(dead_code)]
    pub(crate) temp_c: f64,
    pub(crate) pitch_deg: f64,
    pub(crate) roll_deg: f64,
}

/// Complete Rocky Studio UI state shared identically between Desktop, Web, and Tablet.
pub struct RockyStudioApp {
    pub model: StudioModel,
    pub transport: Box<dyn RobotTransport>,
    pub viewport: HumanoidViewport,
    pub connected: bool,
    pub active_screen: AppScreen,
    pub config_drawer_tab: ConfigDrawerTab,
    pub show_config_drawer: bool,
    pub show_quick_balance_tune: bool,
    pub show_action_studio: bool,
    pub(crate) action_category_filter: ActionCategoryFilter,
    pub push_strength: PushStrength,
    keymap: Keymap,
    keymap_dispatcher: craft_keymap::egui::Dispatcher,
    keymap_platform: craft_keymap::Platform,
    theme_applied: bool,
    last_frame: Instant,
    pub(crate) last_dt_sec: f32,
    /// Unconsumed real frame time awaiting conversion to fixed 5 ms control ticks.
    sim_control_accum_sec: f32,
    /// Achieved control ticks per wall second, reported by the sim transport.
    pub(crate) control_rate_hz: f32,
    /// True when the bounded catch-up cap dropped ticks in the latest frame.
    pub(crate) sim_catchup_cap_hit: bool,
    sim_time_sec: f64,
    pub(crate) history: VecDeque<TelemetryPoint>,
    gilrs: Option<gilrs::Gilrs>,
    pub(crate) active_gamepad_name: Option<String>,
    pub(crate) last_gamepad_event: String,
    gamepad_vx: f32,
    gamepad_vy: f32,
    gamepad_yaw: f32,
    pub(crate) pad_vx: f32,
    pub(crate) pad_vy: f32,
    pub(crate) pad_yaw: f32,
    pub(crate) replay_result: Option<ReplayVerification>,
    /// Serialized stopped replay log, ready for platform export (download on WASM).
    pub(crate) copper_export_json: Option<String>,
    calibration_import_buf: String,
    pub(crate) show_import_dialog: bool,
    pub(crate) paste_urdf_buf: String,
    pub(crate) show_paste_urdf_dialog: bool,
    pub(crate) mcu_sync: McuModelSyncState,
    pub(crate) staged_sim_model: Option<(String, ModelFormat, Vec<u8>, u8)>,
    pub(crate) ui_metrics: UiMetrics,
    applied_form_factor: Option<FormFactor>,
    pub(crate) viewport_hud_rects: [egui::Rect; 5],
    pub(crate) saver: Box<dyn PlatformSaver>,
    pub(crate) selected_transport_ui: TransportKind,
    pub mcp: McpUiState,
    /// Native-only consumer half of the HTTP→egui MCP bridge.
    #[cfg(not(target_arch = "wasm32"))]
    mcp_bridge_rx: Option<rk_mcp::McpBridgeRx>,
    /// Visual alarm / latch state for the most recent fall event.
    pub fall_event_active: bool,
    /// Classification captured from telemetry when the fall latched.
    pub fall_orientation: FallOrientation,
    /// Standing opt-in for autonomous recovery (off by default).
    pub auto_recovery_enabled: bool,
    /// Per-event operator confirmation (set by the confirm checkbox or Get Up button).
    pub operator_confirmed: bool,
    /// Ordered recovery sequence state.
    pub recovery_state: RecoveryState,
    /// Seconds of continuous settled state since the last fall latch.
    pub fall_dwell_sec: f32,
    /// True when the Get Up button (not the autonomous path) initiated this sequence.
    get_up_requested: bool,
    /// Operator's original `(BalanceGains, balance_enabled)` saved while get-up plays.
    saved_recovery_gains: Option<(rk_proto::BalanceGains, bool)>,
}

impl RockyStudioApp {
    pub fn new(saver: Box<dyn PlatformSaver>) -> Self {
        let model = StudioModel::new_default();
        let transport: Box<dyn RobotTransport> = Box::new(SimTransport::new(
            model.rig.joints.len(),
            &model.calibration,
            model.rig.default_pose(),
        ));

        let platform = craft_keymap::Platform::Linux;
        let keymap = build_rocky_keymap(platform);
        let gilrs = gilrs::Gilrs::new().ok();

        let mut viewport = HumanoidViewport::new();
        viewport.fit_to_rig(&model.rig);

        let default_metrics = UiMetrics::resolve(Vec2::new(1440.0, 900.0));

        Self {
            model,
            transport,
            viewport,
            connected: false,
            active_screen: AppScreen::Dashboard,
            config_drawer_tab: ConfigDrawerTab::BalanceGait,
            show_config_drawer: false,
            show_quick_balance_tune: false,
            show_action_studio: false,
            action_category_filter: ActionCategoryFilter::All,
            push_strength: PushStrength::Shove,
            keymap,
            keymap_dispatcher: craft_keymap::egui::Dispatcher::new(),
            keymap_platform: platform,
            theme_applied: false,
            last_frame: Instant::now(),
            last_dt_sec: 0.016,
            sim_control_accum_sec: 0.0,
            control_rate_hz: 200.0,
            sim_catchup_cap_hit: false,
            sim_time_sec: 0.0,
            history: VecDeque::with_capacity(TELEMETRY_HISTORY_LEN + 4),
            gilrs,
            active_gamepad_name: None,
            last_gamepad_event: "No controller input yet".to_string(),
            gamepad_vx: 0.0,
            gamepad_vy: 0.0,
            gamepad_yaw: 0.0,
            pad_vx: 0.0,
            pad_vy: 0.0,
            pad_yaw: 0.0,
            replay_result: None,
            copper_export_json: None,
            calibration_import_buf: String::new(),
            show_import_dialog: false,
            paste_urdf_buf: String::new(),
            show_paste_urdf_dialog: false,
            mcu_sync: McuModelSyncState::default(),
            staged_sim_model: None,
            ui_metrics: default_metrics,
            applied_form_factor: None,
            viewport_hud_rects: [egui::Rect::NOTHING; 5],
            saver,
            selected_transport_ui: TransportKind::SimKernel,
            mcp: McpUiState::default(),
            #[cfg(not(target_arch = "wasm32"))]
            mcp_bridge_rx: None,
            fall_event_active: false,
            fall_orientation: FallOrientation::Unknown,
            auto_recovery_enabled: false,
            operator_confirmed: false,
            recovery_state: RecoveryState::Idle,
            fall_dwell_sec: 0.0,
            get_up_requested: false,
            saved_recovery_gains: None,
        }
    }

    /// Connects the selected robot transport in safe `Teach` mode (Torque OFF) and queries MCU model metadata.
    pub fn connect_active_transport(&mut self) {
        self.model.set_authority_mode(ControlMode::Teach);
        self.model.walk_cmd.walk_enabled = false;
        self.model.walk_cmd.vx = 0.0;
        self.model.walk_cmd.vy = 0.0;
        self.model.walk_cmd.yaw_rate = 0.0;

        match self.selected_transport_ui {
            TransportKind::SimKernel => {
                let mut sim = SimTransport::new(
                    self.model.rig.joints.len().max(1),
                    &self.model.calibration,
                    self.model.rig.default_pose(),
                );
                if let Some((name, fmt, blob, joints)) = &self.staged_sim_model {
                    sim.stage_sim_mcu_model(name, *fmt, blob.clone(), *joints);
                }
                self.transport = Box::new(sim);
                self.connected = true;
                self.sim_control_accum_sec = 0.0;
                self.sim_catchup_cap_hit = false;
                self.query_mcu_model_info();
                // Prime one fixed telemetry frame so Dashboard stats immediately populate
                let pkt = self.model.build_next_host_packet(0.01);
                if let Some(telem) = self.transport.step_fixed(1, &pkt, CONTROL_STEP_SEC) {
                    self.model.ingest_telemetry(telem);
                }
                self.control_rate_hz = self.transport.stats().packet_rate_hz;
                self.viewport.fit_to_rig(&self.model.rig);
                self.model.status_message =
                    "Connected to Simulated Rocky MCU Twin in safe Teach mode.".to_string();
            }
            #[cfg(not(target_arch = "wasm32"))]
            TransportKind::UsbNetworkingUdp => {
                match rk_link::UdpTransport::connect("10.42.0.2:7777") {
                    Ok(udp) => {
                        self.transport = Box::new(udp);
                        self.connected = true;
                        self.query_mcu_model_info();
                        self.model.status_message =
                            "Connected RP2350 USB-Net UDP -> 10.42.0.2:7777 (Teach mode)."
                                .to_string();
                    }
                    Err(err) => {
                        self.connected = false;
                        self.model.status_message = format!("USB-Net UDP bind failed: {err}");
                    }
                }
            }
            _ => {
                self.connected = false;
                self.model.status_message =
                    "Selected transport is not available on this build.".to_string();
            }
        }
    }

    /// Safely drops to `Teach` mode, stops walking, and returns to the disconnected Dashboard Home.
    pub fn disconnect_transport(&mut self) {
        self.model.set_authority_mode(ControlMode::Teach);
        self.model.walk_cmd.walk_enabled = false;
        self.model.walk_cmd.vx = 0.0;
        self.model.walk_cmd.vy = 0.0;
        self.model.walk_cmd.yaw_rate = 0.0;
        self.pad_vx = 0.0;
        self.pad_vy = 0.0;
        self.pad_yaw = 0.0;
        self.restore_recovery_gains();
        self.transport.set_recovery_progress(None);
        self.fall_event_active = false;
        self.fall_orientation = FallOrientation::Unknown;
        self.recovery_state = RecoveryState::Idle;
        self.fall_dwell_sec = 0.0;
        self.get_up_requested = false;
        self.connected = false;
        self.active_screen = AppScreen::Dashboard;
        self.show_config_drawer = false;
        self.model.status_message = "Disconnected from robot. Returned to Dashboard.".to_string();
    }

    /// Injects a deterministic directional shove strong enough to knock Rocky into
    /// the requested fall orientation (`Prone`, `Supine`, `SideLeft`, `SideRight`) in simulation.
    pub fn trigger_simulated_fall(&mut self, orientation: FallOrientation) {
        if self.model.requested_mode == ControlMode::Teach && !self.model.is_estop_active() {
            self.model.set_authority_mode(ControlMode::Drive);
        }
        self.model.walk_cmd.walk_enabled = false;
        self.model.playback = None;
        let (dp, dr) = match orientation {
            FallOrientation::Prone => (6.0_f32, 0.0_f32),
            FallOrientation::Supine => (-6.0_f32, 0.0_f32),
            FallOrientation::SideLeft => (0.0_f32, -6.0_f32),
            FallOrientation::SideRight => (0.0_f32, 6.0_f32),
            FallOrientation::Unknown => (6.0_f32, 0.0_f32),
        };
        self.transport.apply_push_impulse(dp, dr);
        self.model.status_message = format!(
            "Simulated {} fall impulse injected — watching IMU threshold breach.",
            orientation.label()
        );
    }

    fn restore_recovery_gains(&mut self) {
        if let Some((orig_gains, orig_balance_enabled)) = self.saved_recovery_gains.take() {
            self.model.update_balance_gains(orig_gains);
            self.model.walk_cmd.balance_enabled = orig_balance_enabled;
        }
    }

    /// Explicit operator manual reset (stands sim upright, resets E-Stop to Teach, and clears fall alarm).
    pub fn manual_reset_upright_and_clear(&mut self) {
        if let Some(f) = self.transport.sim_faults_mut() {
            f.drop_heartbeat = false;
        }
        self.transport.set_recovery_progress(None);
        self.restore_recovery_gains();
        self.transport.reset_upright();
        if self.model.is_estop_active() {
            self.model.reset_estop();
        }
        self.fall_event_active = false;
        self.fall_orientation = FallOrientation::Unknown;
        self.recovery_state = RecoveryState::Idle;
        self.fall_dwell_sec = 0.0;
        self.operator_confirmed = false;
        self.get_up_requested = false;
    }

    /// Applies a newly parsed `AssetBundle` (from upload, drop, paste, preset, or MCU pull),
    /// refreshes the 3D scene and camera bounds, and resets the simulation plant cleanly.
    pub fn apply_loaded_bundle(&mut self, bundle: AssetBundle) {
        if let Err(err) = self.model.load_bundle(bundle) {
            self.model.status_message = err;
            return;
        }
        if self.selected_transport_ui == TransportKind::SimKernel {
            let mut sim = SimTransport::new(
                self.model.rig.joints.len().max(1),
                &self.model.calibration,
                self.model.rig.default_pose(),
            );
            if let Some((name, fmt, blob, joints)) = &self.staged_sim_model {
                sim.stage_sim_mcu_model(name, *fmt, blob.clone(), *joints);
            }
            self.transport = Box::new(sim);
        }
        self.viewport.notify_rig_changed();
        self.viewport.fit_to_rig(&self.model.rig);
    }

    /// Applies a direct joint angle edit (from 3D dragging or the visual joint bar) in either Drive or Teach mode.
    pub(crate) fn apply_direct_joint_angle(&mut self, joint_idx: usize, rad: f32) {
        if joint_idx >= self.model.rig.joints.len() || self.model.is_estop_active() {
            return;
        }
        let cal = self.model.calibration[joint_idx];
        let clamped = rad.clamp(cal.min_rad, cal.max_rad);
        self.model.selected_joint = joint_idx;
        self.model.playback = None;

        if self.model.requested_mode == ControlMode::Drive {
            self.model.target_rad[joint_idx] = clamped;
        } else {
            // In Teach mode (Torque OFF), visually hand-pose the simulated physical robot
            let raw = rk_control::calibrated_to_raw(clamped, &cal);
            self.transport.set_virtual_hand_angle(joint_idx, raw);
            self.model.target_rad[joint_idx] = clamped;
        }
    }

    /// Applies selection, FK joint drag, or `k` Jacobian Limb IK drag returned from the 3D viewport.
    pub(crate) fn apply_viewport_interaction(&mut self, inter: &ViewportInteraction) {
        if let Some(picked_idx) = inter.picked_joint {
            self.model.selected_joint = picked_idx;
            if let Some(j) = self.model.rig.joints.get(picked_idx) {
                self.model.status_message = format!(
                    "Selected #{:02} {} ({}) — drag robot to pose, or drag empty space / ViewCube to rotate view.",
                    picked_idx, j.label, j.name
                );
            }
        }

        if let Some((joint_idx, angle_rad)) = inter.dragged_joint {
            self.apply_direct_joint_angle(joint_idx, angle_rad);
            if let Some(j) = self.model.rig.joints.get(joint_idx) {
                self.model.status_message = format!(
                    "FK Drag: {} -> {:+.1} deg",
                    j.label,
                    angle_rad.to_degrees()
                );
            }
        } else if let Some((ref link_name, solved)) = inter.ik_pose {
            let dof = self.model.rig.joints.len().min(rk_proto::MAX_JOINTS);
            self.model.playback = None;
            for idx in 0..dof {
                let cal = self.model.calibration[idx];
                let clamped = solved[idx].clamp(cal.min_rad, cal.max_rad);
                if self.model.requested_mode == ControlMode::Drive {
                    self.model.target_rad[idx] = clamped;
                } else {
                    let raw = rk_control::calibrated_to_raw(clamped, &cal);
                    self.transport.set_virtual_hand_angle(idx, raw);
                    self.model.target_rad[idx] = clamped;
                }
            }
            self.model.status_message = format!("IK Limb Reach: dragging '{link_name}'");
        }
    }

    pub fn update_frame(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();

        if !self.theme_applied {
            apply_rocky_theme(&ctx);
            self.keymap_platform = craft_keymap::egui::platform(&ctx);
            self.keymap = build_rocky_keymap(self.keymap_platform);
            self.theme_applied = true;
        }

        // Resolve unified touch-first ergonomics for Landscape / Portrait
        let viewport_size = ctx.content_rect().size();
        self.ui_metrics = UiMetrics::resolve(viewport_size);
        self.viewport.pick_radius_scale = self.ui_metrics.viewport_pick_scale;
        self.viewport.touch_mode = true;
        if self.applied_form_factor != Some(self.ui_metrics.form_factor) {
            apply_responsive_style(&ctx, &self.ui_metrics);
            self.applied_form_factor = Some(self.ui_metrics.form_factor);
        }

        // 1. High-priority Spacebar E-Stop via craft-keymap (disabled only while typing in a text field)
        let typing = ctx.egui_wants_keyboard_input();
        let mut estop_pressed = false;
        ctx.input_mut(|input| {
            let active = Active {
                typing,
                ..Active::NONE
            };
            for hit in self
                .keymap_dispatcher
                .dispatch(input, &self.keymap, &active)
            {
                if hit.command.id == CMD_ESTOP {
                    estop_pressed = true;
                }
            }
        });
        if estop_pressed && self.connected {
            self.mcp.on_estop_disarm();
            self.model.trigger_estop();
        }

        // 2. Handle drag-and-drop URDF / .zip bundles directly onto the window
        #[cfg(not(target_arch = "wasm32"))]
        let dropped: Vec<(String, Vec<u8>)> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| {
                    let bytes = f.bytes().ok()?;
                    let name = f
                        .path()
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| "uploaded.urdf".to_string());
                    Some((name, bytes))
                })
                .collect()
        });
        #[cfg(target_arch = "wasm32")]
        let dropped: Vec<(String, Vec<u8>)> = {
            let files = ctx.input(|i| i.raw.dropped_files.clone());
            for f in files {
                let name = f
                    .path()
                    .file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_else(|| "uploaded.urdf".to_string());
                let ctx_clone = ctx.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    if let Ok(bytes) = f.bytes_async().await {
                        PENDING_WASM_DROPS.with(|q| q.borrow_mut().push((name, bytes)));
                        ctx_clone.request_repaint();
                    }
                });
            }
            PENDING_WASM_DROPS.with(|q| std::mem::take(&mut *q.borrow_mut()))
        };
        let mut incoming_files = self.saver.poll_opened_files();
        incoming_files.extend(dropped);
        for (name, bytes) in incoming_files {
            match AssetBundle::from_bytes(&name, &bytes) {
                Ok(bundle) => self.apply_loaded_bundle(bundle),
                Err(err) => {
                    self.model.status_message = err;
                }
            }
        }

        // 3. Advance real-time control loop, keyboard WASD, gamepad, and MCU pull only while connected
        let now = Instant::now();
        let raw_dt_sec = now.duration_since(self.last_frame).as_secs_f32();
        let dt_sec = raw_dt_sec.clamp(0.004, 0.05);
        self.last_frame = now;
        self.last_dt_sec = dt_sec;

        if self.connected {
            self.step_mcu_model_pull();
            self.poll_gamepad();
            self.update_walk_command_from_inputs(&ctx, typing);

            // Fixed-step accumulator: convert real frame time into whole 5 ms
            // control ticks, run at most MAX_CONTROL_STEPS_PER_FRAME of them,
            // and keep the remainder for the next frame. The cap means a slow
            // frame slows simulated time rather than corrupting determinism.
            // Use the unclamped (but upper-bounded) frame time here: clamping a
            // very fast frame up to 4 ms would drive the sim faster than real time.
            self.sim_control_accum_sec = (self.sim_control_accum_sec
                + raw_dt_sec.min(MAX_CONTROL_ACCUM_SEC))
            .min(MAX_CONTROL_ACCUM_SEC);
            let available_steps = (self.sim_control_accum_sec / CONTROL_STEP_SEC) as usize;
            let steps = available_steps.min(MAX_CONTROL_STEPS_PER_FRAME);
            self.sim_control_accum_sec -= steps as f32 * CONTROL_STEP_SEC;
            self.sim_catchup_cap_hit = available_steps > steps;

            self.sim_time_sec += dt_sec as f64;
            // Host command/UI state is sampled exactly once per rendered frame;
            // the transport repeats it across the fixed ticks of this frame.
            let host_pkt = self.model.build_next_host_packet(dt_sec);
            if steps > 0 {
                if let Some(telem) = self.transport.step_fixed(steps, &host_pkt, dt_sec) {
                    self.model.ingest_telemetry(telem);
                    if self.model.is_estop_active() {
                        self.mcp.on_estop_disarm();
                    }
                }
            }
            // Achieved control rate (ticks/wall-second), not render FPS.
            self.control_rate_hz = self.transport.stats().packet_rate_hz;
            // Advance the consent-gated, multi-frame fall recovery sequence only after
            // fresh telemetry has been ingested so every safety gate is observed.
            self.update_fall_watcher(dt_sec);

            let j = self
                .model
                .selected_joint
                .min(self.model.rig.joints.len().saturating_sub(1));
            let err_deg = (self.model.target_rad[j] - self.model.latest_telemetry.measured_rad[j])
                .to_degrees() as f64;
            self.history.push_back(TelemetryPoint {
                t_sec: self.sim_time_sec,
                err_deg,
                current_ma: self.model.latest_telemetry.current_ma[j] as f64,
                temp_c: self.model.latest_telemetry.temp_c[j] as f64,
                pitch_deg: self.model.latest_telemetry.imu.pitch_rad.to_degrees() as f64,
                roll_deg: self.model.latest_telemetry.imu.roll_rad.to_degrees() as f64,
            });
            while self.history.len() > TELEMETRY_HISTORY_LEN {
                self.history.pop_front();
            }
        } else {
            self.sim_control_accum_sec = 0.0;
            self.sim_catchup_cap_hit = false;
            self.sim_time_sec += dt_sec as f64;
            self.active_screen = AppScreen::Dashboard;
        }

        // 3b. WASM polls the host bridge here. On native the bridge is drained
        // from DesktopWrapper::logic so hidden windows still service requests.
        #[cfg(target_arch = "wasm32")]
        self.pump_mcp_bridge(&ctx);

        // 4. Contextual Top Toolbar
        let top_h = self.ui_metrics.top_bar_height;
        egui::Panel::top("rocky_top_toolbar")
            .exact_size(top_h)
            .frame(
                egui::Frame::new()
                    .fill(Color32::from_rgb(13, 18, 26))
                    .inner_margin(egui::Margin::symmetric(10, 4))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(30, 40, 56))),
            )
            .show(ui, |ui| {
                self.render_top_toolbar(ui);
            });

        // 5. Quiet Bottom Status Ribbon
        let status_h = self.ui_metrics.status_bar_height;
        egui::Panel::bottom("rocky_status_bar")
            .exact_size(status_h)
            .frame(
                egui::Frame::new()
                    .fill(Color32::from_rgb(12, 16, 23))
                    .inner_margin(egui::Margin::symmetric(12, 2))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(28, 36, 50))),
            )
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(
                        RichText::new(&self.model.status_message)
                            .size(11.5)
                            .color(Color32::from_rgb(168, 184, 205)),
                    );
                    // Honest control-rate indicator: achieved fixed ticks per
                    // second, with an explicit warning when catch-up is capped.
                    if self.connected && viewport_size.x >= 560.0 {
                        let shortfall = self.sim_catchup_cap_hit || self.control_rate_hz < 190.0;
                        let color = if shortfall {
                            Color32::from_rgb(251, 191, 36)
                        } else {
                            Color32::from_rgb(74, 222, 128)
                        };
                        let text = if self.sim_catchup_cap_hit {
                            format!(
                                "CTRL {:.0} Hz  ·  CATCH-UP CAP (frames < 200 Hz)",
                                self.control_rate_hz
                            )
                        } else {
                            format!("CTRL {:.0} Hz", self.control_rate_hz)
                        };
                        ui.label(RichText::new(text).size(11.0).color(color));
                    }
                    if viewport_size.x >= 720.0 {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let screen_tag = match self.active_screen {
                                AppScreen::Dashboard => "DASHBOARD",
                                AppScreen::Control => "CONTROL",
                                AppScreen::Config => "CONFIG & CAL",
                            };
                            ui.label(
                                RichText::new(format!(
                                    "{screen_tag}  ·  {} ({} DoF)  ·  {}",
                                    self.model.rig.robot_name,
                                    self.model.rig.joints.len(),
                                    self.ui_metrics.form_factor.badge()
                                ))
                                .size(11.0)
                                .color(Color32::from_rgb(115, 132, 156)),
                            );
                        });
                    }
                });
            });

        // 6. Central Workspace: Dedicated 3-Screen Router (Dashboard | Control | Config & Calibration)
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(Color32::from_rgb(11, 14, 20)))
            .show(ui, |ui| match self.active_screen {
                AppScreen::Dashboard => self.render_dashboard_screen(ui),
                AppScreen::Control => self.render_control_screen(ui),
                AppScreen::Config => self.render_config_screen(ui),
            });

        // Optional Calibration JSON import modal
        if self.show_import_dialog {
            let mut open = self.show_import_dialog;
            egui::Window::new("Import Calibration JSON")
                .open(&mut open)
                .collapsible(false)
                .default_width(440.0)
                .show(&ctx, |ui| {
                    ui.label("Paste exported calibration JSON below:");
                    egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                        ui.add(
                            egui::TextEdit::multiline(&mut self.calibration_import_buf)
                                .font(egui::TextStyle::Monospace)
                                .desired_width(f32::INFINITY),
                        );
                    });
                    ui.horizontal(|ui| {
                        if ui.button("Apply Calibration").clicked() {
                            match self.model.import_calibration_json(&self.calibration_import_buf) {
                                Ok(()) => self.show_import_dialog = false,
                                Err(err) => self.model.status_message = err,
                            }
                        }
                        if ui.button("Cancel").clicked() {
                            self.show_import_dialog = false;
                        }
                    });
                });
            if !open {
                self.show_import_dialog = false;
            }
        }

        // Optional Paste URDF XML modal
        self.render_paste_urdf_modal(&ctx);

        // Slide-over MCP Agent Drawer
        self.render_mcp_drawer(&ctx);

        // Visual drag-and-drop overlay when hovering a .urdf/.zip file over the window
        let hovered_files = ctx.input(|i| i.raw.hovered_files.len());
        if hovered_files > 0 {
            let screen_rect = ctx.content_rect();
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                ui.scope_id().with("rocky_drop_overlay"),
            ));
            painter.rect_filled(screen_rect, 0.0, Color32::from_black_alpha(180));
            painter.rect_stroke(
                screen_rect.shrink(18.0),
                CornerRadius::same(10),
                Stroke::new(2.0, Color32::from_rgb(56, 189, 248)),
                egui::StrokeKind::Inside,
            );
            painter.text(
                screen_rect.center(),
                egui::Align2::CENTER_CENTER,
                "Drop .urdf / .xml / .zip robot file to parse & render in 3D Canvas",
                egui::FontId::proportional(20.0),
                Color32::from_rgb(224, 242, 254),
            );
        }

        ctx.request_repaint();
    }

    /// Advances the fall-recovery state machine from freshly ingested telemetry.
    ///
    /// Safety invariants mirrored here:
    /// - nothing leaves `Teach` or clears faults unless `ClearFaults` was queued first;
    /// - `Drive` is entered only after `is_estop_active()` is false in telemetry;
    /// - a get-up move is played only while `requested_mode == Drive` and no fault is latched;
    /// - any new fall / E-Stop during playback cuts torque and re-latches immediately.
    fn update_fall_watcher(&mut self, dt_sec: f32) {
        let telem = &self.model.latest_telemetry;
        let fall_fault = telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF) && telem.imu.available;
        let estop_active = self.model.is_estop_active();
        let imu_ok = telem.imu.available;
        let tilt = telem.imu.pitch_rad.abs().max(telem.imu.roll_rad.abs());
        let rate = telem
            .imu
            .pitch_rate_rad_s
            .abs()
            .max(telem.imu.roll_rate_rad_s.abs());
        let nominal_cutoff = self
            .saved_recovery_gains
            .map(|(g, _)| g.fall_cutoff_rad)
            .unwrap_or(self.model.balance_gains.fall_cutoff_rad);
        let upright = imu_ok && tilt < nominal_cutoff;
        let settled = imu_ok && rate < RECOVERY_SETTLED_RATE_RAD_S;
        let orientation = FallOrientation::from_wire(telem.balance.fall_orientation);
        // An action abort in `StudioModel::ingest_telemetry` latches this signal so
        // recovery can take over even when the MCU has not yet published a fall fault.
        let aborted = self.model.playback_aborted;
        if fall_fault && self.fall_orientation == FallOrientation::Unknown {
            self.fall_orientation = orientation;
        }

        // 1. Latch a new fall (or abort an in-progress get-up on a re-fall / E-Stop).
        match self.recovery_state {
            RecoveryState::Idle | RecoveryState::Completed => {
                if fall_fault || aborted {
                    self.model.playback_aborted = false;
                    self.transport.set_recovery_progress(None);
                    self.restore_recovery_gains();
                    self.fall_event_active = true;
                    if fall_fault || orientation != FallOrientation::Unknown {
                        self.fall_orientation = orientation;
                    }
                    self.fall_dwell_sec = 0.0;
                    // Preserve standing opt-in; require fresh confirmation unless auto-recovery is armed.
                    self.operator_confirmed = false;
                    self.get_up_requested = false;
                    self.recovery_state = RecoveryState::FallLatched;
                    if fall_fault {
                        self.model.status_message = format!(
                            "FALL DETECTED ({}). Click Get Up or confirm Auto Recovery to stand up.",
                            self.fall_orientation.label()
                        );
                    } else {
                        self.model.status_message = format!(
                            "ACTION ABORTED ({}). Recovery standby — click Get Up or confirm Auto Recovery.",
                            self.fall_orientation.label()
                        );
                    }
                }
            }
            RecoveryState::PlayingGetUp => {
                if fall_fault || estop_active {
                    self.model.playback_aborted = false;
                    self.model.stop_playback();
                    self.transport.set_recovery_progress(None);
                    self.restore_recovery_gains();
                    // Explicitly cut torque even if the MCU has not yet mirrored the fault.
                    self.model.trigger_estop();
                    self.model.walk_cmd.walk_enabled = false;
                    self.fall_event_active = true;
                    if fall_fault {
                        self.fall_orientation = orientation;
                    }
                    self.fall_dwell_sec = 0.0;
                    self.operator_confirmed = false;
                    self.get_up_requested = false;
                    self.recovery_state = RecoveryState::FallLatched;
                    self.model.status_message =
                        "Get-up ABORTED by new fall / E-Stop — torque OFF.".to_string();
                    return;
                }
            }
            RecoveryState::FallLatched
            | RecoveryState::ClearingFault
            | RecoveryState::ArmingDrive => {
                // An operator E-Stop (without a new fall) also aborts an armed sequence.
                if estop_active && !fall_fault && self.recovery_state != RecoveryState::FallLatched {
                    self.model.playback_aborted = false;
                    self.model.recovery_playback = false;
                    self.transport.set_recovery_progress(None);
                    self.restore_recovery_gains();
                    self.model.walk_cmd.walk_enabled = false;
                    self.fall_dwell_sec = 0.0;
                    self.operator_confirmed = false;
                    self.get_up_requested = false;
                    self.recovery_state = RecoveryState::FallLatched;
                    self.model.status_message =
                        "Recovery aborted by E-Stop. Torque OFF.".to_string();
                    return;
                }
            }
        }

        // 2. Track dwell:
        //    - While FallLatched: wait for the robot to settle (stop tumbling/bouncing on the
        //      floor, or sit upright) so recovery can start directly from a fallen pose.
        //    - After playback (Completed): require genuine upright attitude before clearing alarm.
        if self.fall_event_active {
            let dwell_ok = match self.recovery_state {
                RecoveryState::FallLatched => settled,
                _ => upright && !fall_fault,
            };
            if dwell_ok {
                self.fall_dwell_sec += dt_sec;
            } else {
                self.fall_dwell_sec = 0.0;
            }
        }

        // 3. Ordered state transitions — at most one per control frame.
        match self.recovery_state {
            RecoveryState::FallLatched => {
                let consent = self.get_up_requested
                    || (self.auto_recovery_enabled && self.operator_confirmed);
                if consent && self.fall_dwell_sec >= RECOVERY_STABLE_DWELL_SEC {
                    self.operator_confirmed = false;
                    self.get_up_requested = false;
                    // Save operator gains and temporarily disable UVC balance & raise fall cutoff
                    // BEFORE ClearFaults so the fallen torso (~65-71 deg) can push off without re-tripping.
                    if self.saved_recovery_gains.is_none() {
                        self.saved_recovery_gains =
                            Some((self.model.balance_gains, self.model.walk_cmd.balance_enabled));
                    }
                    self.model.walk_cmd.balance_enabled = false;
                    let mut rec_gains = self.model.balance_gains;
                    rec_gains.fall_cutoff_rad = RECOVERY_PLAYBACK_CUTOFF_RAD;
                    self.model.update_balance_gains(rec_gains);
                    self.model.reset_estop();
                    self.recovery_state = RecoveryState::ClearingFault;
                    self.model.status_message =
                        "Recovery step 1/3: ClearFaults queued; waiting for fault clear."
                            .to_string();
                }
            }
            RecoveryState::ClearingFault => {
                if !estop_active {
                    self.model.set_authority_mode(ControlMode::Drive);
                    self.recovery_state = RecoveryState::ArmingDrive;
                    self.model.status_message =
                        "Recovery step 2/3: fault clear observed; Drive requested.".to_string();
                }
            }
            RecoveryState::ArmingDrive => {
                if self.model.requested_mode == ControlMode::Drive && !estop_active {
                    self.model.walk_cmd.walk_enabled = false;
                    if let Some(idx) = self.recovery_move_index() {
                        self.model.play_move(idx);
                        // Elevated tilt during get-up belongs to this state machine,
                        // not to the action-abort path in `ingest_telemetry`.
                        self.model.playback_aborted = false;
                        self.model.recovery_playback = true;
                        self.transport.set_recovery_progress(Some(0.0));
                        self.recovery_state = RecoveryState::PlayingGetUp;
                        self.model.status_message = format!(
                            "Recovery step 3/3: playing '{}'.",
                            self.model.moves[idx].name
                        );
                    } else {
                        // No matching motion: fail safe instead of playing the wrong move.
                        self.restore_recovery_gains();
                        self.model.set_authority_mode(ControlMode::Teach);
                        self.recovery_state = RecoveryState::Completed;
                        self.model.status_message = format!(
                            "No get-up motion for {} — staying in Teach.",
                            self.fall_orientation.label()
                        );
                    }
                }
            }
            RecoveryState::PlayingGetUp => {
                self.model.walk_cmd.walk_enabled = false;
                if let Some(pb) = &self.model.playback {
                    self.transport.set_recovery_progress(Some(pb.progress));
                } else {
                    self.transport.set_recovery_progress(None);
                    self.model.recovery_playback = false;
                    self.restore_recovery_gains();
                    self.model.set_authority_mode(ControlMode::Teach);
                    self.recovery_state = RecoveryState::Completed;
                    self.model.status_message =
                        "Get-up finished — returning to Teach; verifying upright stance."
                            .to_string();
                }
            }
            RecoveryState::Completed => {
                if upright && !fall_fault && !estop_active {
                    self.fall_event_active = false;
                    self.fall_orientation = FallOrientation::Unknown;
                    self.fall_dwell_sec = 0.0;
                    self.recovery_state = RecoveryState::Idle;
                    self.model.status_message =
                        "Recovery complete — stable guard stance, alarm cleared.".to_string();
                }
            }
            RecoveryState::Idle => {}
        }
    }

    /// Index of the pre-computed get-up move matching the latched fall orientation.
    fn recovery_move_index(&self) -> Option<usize> {
        let name = match self.fall_orientation {
            FallOrientation::Prone => "Get Up Prone",
            FallOrientation::Supine => "Get Up Supine",
            FallOrientation::SideLeft => "Get Up Left Side",
            FallOrientation::SideRight => "Get Up Right Side",
            FallOrientation::Unknown => return None,
        };
        self.model.moves.iter().position(|mv| mv.name == name)
    }

    fn update_walk_command_from_inputs(&mut self, ctx: &egui::Context, typing: bool) {
        let mut kb_vx = 0.0_f32;
        let mut kb_vy = 0.0_f32;
        let mut kb_yaw = 0.0_f32;

        if self.active_screen == AppScreen::Control && !typing && !self.model.is_estop_active() {
            ctx.input(|i| {
                if i.key_down(Key::W) || i.key_down(Key::ArrowUp) {
                    kb_vx += 1.0;
                }
                if i.key_down(Key::S) || i.key_down(Key::ArrowDown) {
                    kb_vx -= 1.0;
                }
                if i.key_down(Key::A) || i.key_down(Key::ArrowLeft) {
                    kb_vy += 1.0;
                }
                if i.key_down(Key::D) || i.key_down(Key::ArrowRight) {
                    kb_vy -= 1.0;
                }
                if i.key_down(Key::Q) {
                    kb_yaw += 1.0;
                }
                if i.key_down(Key::E) {
                    kb_yaw -= 1.0;
                }
            });
        }

        let total_vx = (kb_vx + self.pad_vx + self.gamepad_vx).clamp(-1.0, 1.0);
        let total_vy = (kb_vy + self.pad_vy + self.gamepad_vy).clamp(-1.0, 1.0);
        let total_yaw = (kb_yaw + self.pad_yaw + self.gamepad_yaw).clamp(-1.0, 1.0);
        let has_input =
            total_vx.abs() > 0.05 || total_vy.abs() > 0.05 || total_yaw.abs() > 0.05;

        if has_input && !self.model.is_estop_active() {
            // Human input always supersedes any prior MCP walk velocity override
            self.mcp.mcp_walk_override = None;
            self.model.walk_cmd.walk_enabled = true;
            if self.model.requested_mode == ControlMode::Teach {
                self.model.set_authority_mode(ControlMode::Drive);
            }
            self.model.walk_cmd.vx = total_vx;
            self.model.walk_cmd.vy = total_vy;
            self.model.walk_cmd.yaw_rate = total_yaw;
        } else if let Some((vx, vy, yaw, enabled)) = self.mcp.mcp_walk_override {
            if self.model.is_estop_active() || !self.mcp.policy.drive_armed {
                self.mcp.mcp_walk_override = None;
                self.model.walk_cmd.vx = 0.0;
                self.model.walk_cmd.vy = 0.0;
                self.model.walk_cmd.yaw_rate = 0.0;
            } else {
                self.model.walk_cmd.vx = vx;
                self.model.walk_cmd.vy = vy;
                self.model.walk_cmd.yaw_rate = yaw;
                self.model.walk_cmd.walk_enabled = enabled;
            }
        } else {
            self.model.walk_cmd.vx = total_vx;
            self.model.walk_cmd.vy = total_vy;
            self.model.walk_cmd.yaw_rate = total_yaw;
        }
    }

    fn poll_gamepad(&mut self) {
        let Some(gilrs) = self.gilrs.as_mut() else {
            return;
        };

        while let Some(gilrs::Event { id, event, .. }) = gilrs.next_event() {
            let name = gilrs.gamepad(id).name().to_string();
            self.active_gamepad_name = Some(name);

            match event {
                gilrs::EventType::ButtonPressed(btn, _) => {
                    self.last_gamepad_event = format!("Button {:?} pressed", btn);
                    match btn {
                        gilrs::Button::South => self.model.play_move(0),
                        gilrs::Button::West => self.model.play_move(1),
                        gilrs::Button::East => self.model.play_move(2),
                        gilrs::Button::North => self.model.play_move(3),
                        gilrs::Button::Start | gilrs::Button::Select => {
                            self.mcp.on_estop_disarm();
                            self.model.trigger_estop();
                        }
                        _ => {}
                    }
                }
                gilrs::EventType::AxisChanged(axis, val, _) => {
                    let v = if val.abs() > 0.14 { val } else { 0.0 };
                    self.last_gamepad_event = format!("Axis {:?} = {:.2}", axis, v);
                    match axis {
                        gilrs::Axis::LeftStickY => self.gamepad_vx = v,
                        gilrs::Axis::LeftStickX => self.gamepad_vy = -v,
                        gilrs::Axis::RightStickX => self.gamepad_yaw = -v,
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}

fn build_rocky_keymap(platform: craft_keymap::Platform) -> Keymap {
    let mut catalog = Vec::new();
    if let Ok(space) = Binding::parse("Space") {
        catalog.push(
            Bindable::new(CMD_ESTOP, "Emergency Stop", Scope::app())
                .with_path(vec!["Safety".to_string(), "E-Stop".to_string()])
                .with_default(space),
        );
    }
    Keymap::new(catalog, Scopes::new(), platform)
}

fn apply_rocky_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = Color32::from_rgb(13, 17, 25);
    visuals.window_fill = Color32::from_rgb(17, 23, 33);
    visuals.extreme_bg_color = Color32::from_rgb(9, 12, 18);
    visuals.faint_bg_color = Color32::from_rgb(19, 25, 36);
    visuals.selection.bg_fill = Color32::from_rgb(14, 116, 144);
    visuals.selection.stroke = Stroke::new(1.0, Color32::from_rgb(103, 232, 249));

    let palette = WidgetPalette {
        noninteractive: Color32::from_rgb(16, 22, 32),
        inactive: Color32::from_rgb(24, 32, 46),
        hovered: Color32::from_rgb(33, 45, 64),
        active: Color32::from_rgb(14, 116, 144),
        open: Color32::from_rgb(28, 38, 54),
    };
    palette.apply_to(
        &mut visuals.widgets,
        CornerRadius::same(5),
        Stroke::new(1.0, Color32::from_rgb(218, 228, 242)),
    );
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(34, 46, 64));
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(44, 58, 80));
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(56, 189, 248));

    ctx.set_visuals(visuals);
}

#[cfg(test)]
mod fall_recovery_tests {
    use super::*;
    use rk_proto::{ConfigCmd, fall_orientation};

    fn new_app() -> RockyStudioApp {
        RockyStudioApp::new(Box::new(NoopSaver))
    }

    /// Pushes a synthetic telemetry frame into the model, mirroring what
    /// `ingest_telemetry()` observes after a `step_exchange()` in the real loop.
    fn telemetry(app: &mut RockyStudioApp, fall: bool, pitch: f32, roll: f32, code: u8) {
        let t = &mut app.model.latest_telemetry;
        t.imu.available = true;
        t.imu.pitch_rad = pitch;
        t.imu.roll_rad = roll;
        t.balance.fall_orientation = code;
        if fall {
            t.faults.insert(FaultFlags::TILT_FALL_CUTOFF);
            t.active_mode = ControlMode::EStop;
        } else {
            t.faults.remove(FaultFlags::TILT_FALL_CUTOFF);
            t.active_mode = ControlMode::Teach;
        }
    }

    fn latch(app: &mut RockyStudioApp, code: u8) {
        telemetry(app, true, 1.2, 0.0, code);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::FallLatched);
        assert!(app.fall_event_active);
    }

    /// Advances stable-upright frames until the dwell elapses and the first
    /// transition out of `FallLatched` occurs (one control frame at a time).
    fn settle_upright(app: &mut RockyStudioApp, code: u8) {
        for _ in 0..60 {
            telemetry(app, false, 0.02, 0.0, code);
            app.update_fall_watcher(0.016);
            if app.recovery_state != RecoveryState::FallLatched {
                break;
            }
        }
    }

    /// Drives a consented fall through ClearFaults -> Drive -> get-up playback.
    fn drive_to_playback(app: &mut RockyStudioApp, code: u8) {
        latch(app, code);
        app.auto_recovery_enabled = true;
        app.operator_confirmed = true;
        settle_upright(app, code);
        assert_eq!(app.recovery_state, RecoveryState::ClearingFault);
        assert_eq!(app.model.requested_mode, ControlMode::Teach);
        assert!(app
            .model
            .pending_config
            .contains(&ConfigCmd::ClearFaults));

        // Fault still latched in telemetry -> cannot arm Drive yet.
        telemetry(app, true, 1.2, 0.0, code);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::ClearingFault);
        assert_eq!(app.model.requested_mode, ControlMode::Teach);

        // Fault clear observed in telemetry -> Drive is requested.
        telemetry(app, false, 0.02, 0.0, code);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::ArmingDrive);
        assert_eq!(app.model.requested_mode, ControlMode::Drive);

        // One full control frame later -> get-up playback starts.
        telemetry(app, false, 0.02, 0.0, code);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::PlayingGetUp);
        assert!(app.model.playback.is_some());
        assert!(!app.model.walk_cmd.walk_enabled);
    }

    #[test]
    fn auto_disabled_does_not_start_recovery() {
        let mut app = new_app();
        latch(&mut app, fall_orientation::PRONE);
        // No consent flags set: auto recovery is off by default.
        settle_upright(&mut app, fall_orientation::PRONE);
        assert_eq!(app.recovery_state, RecoveryState::FallLatched);
        assert_eq!(app.model.requested_mode, ControlMode::Teach);
        assert!(app.model.pending_config.is_empty());
        assert!(app.model.playback.is_none());
        assert!(app.fall_event_active);
    }

    #[test]
    fn ordered_recovery_reaches_completed_and_clears_alarm() {
        let mut app = new_app();
        drive_to_playback(&mut app, fall_orientation::PRONE);

        // Playback finished -> return to Teach and wait for stable upright.
        app.model.playback = None;
        telemetry(&mut app, false, 0.02, 0.0, fall_orientation::PRONE);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::Completed);
        assert_eq!(app.model.requested_mode, ControlMode::Teach);
        assert!(app.fall_event_active, "alarm must persist until upright");

        // Dwell already satisfied -> alarm clears and state returns to Idle.
        telemetry(&mut app, false, 0.02, 0.0, fall_orientation::PRONE);
        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::Idle);
        assert!(!app.fall_event_active);
        assert_eq!(app.fall_orientation, FallOrientation::Unknown);
    }

    #[test]
    fn re_fall_during_get_up_aborts_and_cuts_torque() {
        let mut app = new_app();
        drive_to_playback(&mut app, fall_orientation::PRONE);

        telemetry(&mut app, true, -1.1, 0.0, fall_orientation::SUPINE);
        app.update_fall_watcher(0.016);

        assert!(app.model.playback.is_none());
        assert_eq!(app.model.requested_mode, ControlMode::EStop);
        assert!(!app.model.walk_cmd.walk_enabled);
        assert_eq!(app.recovery_state, RecoveryState::FallLatched);
        assert_eq!(app.fall_orientation, FallOrientation::Supine);
        assert!(!app.operator_confirmed);
        assert!(app.fall_event_active);
    }

    #[test]
    fn get_up_button_works_with_auto_disabled() {
        let mut app = new_app();
        latch(&mut app, fall_orientation::SIDE_LEFT);
        // Explicit Get Up initiation does not require the auto opt-in.
        app.operator_confirmed = true;
        app.get_up_requested = true;
        settle_upright(&mut app, fall_orientation::SIDE_LEFT);
        assert_eq!(app.recovery_state, RecoveryState::ClearingFault);
        assert!(app
            .model
            .pending_config
            .contains(&ConfigCmd::ClearFaults));
    }

    /// Pushes a synthetic telemetry frame through the real `ingest_telemetry` path
    /// so the abort signal is produced exactly as it is in the live control loop.
    fn ingest(app: &mut RockyStudioApp, fall: bool, pitch: f32, roll: f32, code: u8) {
        let mut t = app.model.latest_telemetry;
        t.imu.available = true;
        t.imu.pitch_rad = pitch;
        t.imu.roll_rad = roll;
        t.balance.fall_orientation = code;
        if fall {
            t.faults.insert(FaultFlags::TILT_FALL_CUTOFF);
            t.active_mode = ControlMode::EStop;
        } else {
            t.faults.remove(FaultFlags::TILT_FALL_CUTOFF);
            t.active_mode = ControlMode::Teach;
        }
        app.model.ingest_telemetry(t);
    }

    #[test]
    fn action_abort_hands_off_to_fall_recovery() {
        let mut app = new_app();
        app.model.set_authority_mode(ControlMode::Drive);
        let combo = app
            .model
            .moves
            .iter()
            .position(|m| m.category == "Combo")
            .expect("combo exists");
        app.model.play_move(combo);
        assert!(app.model.playback.is_some());

        // Tilt breach before the MCU latches a fall fault.
        ingest(&mut app, false, 0.62, 0.0, fall_orientation::UNKNOWN);
        assert!(app.model.playback.is_none());
        assert!(app.model.playback_aborted);

        app.update_fall_watcher(0.016);
        assert_eq!(app.recovery_state, RecoveryState::FallLatched);
        assert!(!app.model.playback_aborted, "handoff signal is consumed once");
        assert!(app.model.playback.is_none(), "no playback may fight recovery");
        assert!(app.fall_event_active);
        assert!(app.model.status_message.contains("ACTION ABORTED"));
    }

    #[test]
    fn get_up_playback_is_not_aborted_by_its_own_tilt() {
        let mut app = new_app();
        drive_to_playback(&mut app, fall_orientation::PRONE);
        assert!(app.model.recovery_playback);

        ingest(&mut app, false, 1.10, 0.0, fall_orientation::UNKNOWN);
        assert!(app.model.playback.is_some());
        assert!(!app.model.playback_aborted);

        // A genuine new fall still cuts get-up playback and re-latches.
        ingest(&mut app, true, 1.20, 0.0, fall_orientation::PRONE);
        app.update_fall_watcher(0.016);
        assert!(app.model.playback.is_none());
        assert_eq!(app.recovery_state, RecoveryState::FallLatched);
        assert_eq!(app.fall_orientation, FallOrientation::Prone);
    }
}

/// Headless, real-plant E2E scenario for task 7.3 (separate file to keep the
/// synthetic-telemetry state-machine tests above focused).
#[cfg(test)]
#[path = "e2e_fall_recovery_test.rs"]
mod e2e_fall_recovery_test;

#[cfg(all(test, not(target_arch = "wasm32")))]
mod native_mcp_bridge_tests {
    use std::thread;
    use std::time::Duration;

    use egui::Context;
    use rk_mcp::mcp_bridge_channel;
    use serde_json::json;

    use super::*;

    #[test]
    fn attached_bridge_replies_to_tools_list() {
        let mut app = RockyStudioApp::new(Box::new(NoopSaver));
        let (tx, rx) = mcp_bridge_channel(|| {});
        app.attach_mcp_bridge(rx);

        // `submit` blocks on the oneshot reply, so enqueue from a worker thread.
        let worker = thread::spawn(move || {
            tx.submit(
                json!({
                    "jsonrpc": "2.0",
                    "id": 1,
                    "method": "tools/list",
                    "params": {},
                }),
                Duration::from_secs(5),
            )
        });

        // One pump drains the queue, dispatches against the live app, and replies.
        app.pump_mcp_bridge(&Context::default());

        let reply = worker
            .join()
            .expect("bridge submit worker panicked")
            .expect("tools/list reply")
            .expect("tools/list reply body");
        assert!(reply
            .get("result")
            .and_then(|r| r.get("tools"))
            .and_then(serde_json::Value::as_array)
            .is_some());
    }
}
