//! Layer 5 — Rocky Studio GUI (`rk-ui`).
//!
//! Unified touch-first 3-screen app shared identically across Desktop, Browser WASM, and Tablet:
//! - [`dashboard_screen`]: Home screen (Connect to Robot -> Live Robot Stats + Control / Config Launchers)
//! - [`control_screen`]: Full-bleed 3D robot with Walk Pad, WASD/Gamepad, MCU Upper-Body Vertical Balance, and Combat Shove tests
//! - [`config_screen`]: Visual 3D direct joint/arm dragging (FK & `k` Limb IK), 3D protractor arc, floating visual joint config bar, and on-demand 3-tab setup drawer
//! - [`top_bar`]: Contextual navigation, Teach/Drive authority toggle, always-pinned E-Stop, and safety banners
//! - [`urdf_panel`], [`calibration_tab`], [`sim_tab`]: Focused engineering tabs inside the Config & Calibration drawer
//! - [`responsive`]: Unified touch-first ergonomics adapting automatically between Landscape and Portrait

mod calibration_tab;
mod config_screen;
mod control_screen;
mod dashboard_screen;
mod responsive;
mod sim_tab;
mod top_bar;
mod urdf_panel;

use std::collections::VecDeque;

use craft_keymap::{Active, Bindable, Binding, Keymap, Scope, Scopes};
use craft_ui::theme::WidgetPalette;
use egui::{Color32, CornerRadius, Key, RichText, Stroke, Vec2};
use rk_link::{RobotTransport, SimTransport, TransportKind};
use rk_model::{AssetBundle, StudioModel};
use rk_proto::{ControlMode, ModelFormat};
use rk_sim::ReplayVerification;
use rk_viewport::{HumanoidViewport, ViewportInteraction};
pub use responsive::{FormFactor, UiMetrics};
use responsive::apply_responsive_style;
use urdf_panel::McuModelSyncState;
use web_time::Instant;

const TELEMETRY_HISTORY_LEN: usize = 180;
const CMD_ESTOP: &str = "safety.estop";

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

/// Focused 3-tab engineering drawer inside the Config & Calibration screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigDrawerTab {
    BalanceGait,
    Joints,
    Setup,
}

impl ConfigDrawerTab {
    pub const fn label(self) -> &'static str {
        match self {
            Self::BalanceGait => "Balance",
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
    pub push_strength: PushStrength,
    keymap: Keymap,
    keymap_dispatcher: craft_keymap::egui::Dispatcher,
    keymap_platform: craft_keymap::Platform,
    theme_applied: bool,
    last_frame: Instant,
    pub(crate) last_dt_sec: f32,
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
            push_strength: PushStrength::Shove,
            keymap,
            keymap_dispatcher: craft_keymap::egui::Dispatcher::new(),
            keymap_platform: platform,
            theme_applied: false,
            last_frame: Instant::now(),
            last_dt_sec: 0.016,
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
                self.query_mcu_model_info();
                // Prime one telemetry frame so Dashboard stats immediately populate
                let pkt = self.model.build_next_host_packet(0.01);
                if let Some(telem) = self.transport.step_exchange(0.01, &pkt) {
                    self.model.ingest_telemetry(telem);
                }
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
        self.connected = false;
        self.active_screen = AppScreen::Dashboard;
        self.show_config_drawer = false;
        self.model.status_message = "Disconnected from robot. Returned to Dashboard.".to_string();
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
                    "Selected #{:02} {} ({}) — drag directly to rotate or use ViewCube to orient.",
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
        let dt_sec = now.duration_since(self.last_frame).as_secs_f32().clamp(0.004, 0.05);
        self.last_frame = now;
        self.last_dt_sec = dt_sec;

        if self.connected {
            self.step_mcu_model_pull();
            self.poll_gamepad();
            self.update_walk_command_from_inputs(&ctx, typing);

            self.sim_time_sec += dt_sec as f64;
            let host_pkt = self.model.build_next_host_packet(dt_sec);
            if let Some(telem) = self.transport.step_exchange(dt_sec, &host_pkt) {
                self.model.ingest_telemetry(telem);
            }

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
            self.active_screen = AppScreen::Dashboard;
        }

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
            self.model.walk_cmd.walk_enabled = true;
            if self.model.requested_mode == ControlMode::Teach {
                self.model.set_authority_mode(ControlMode::Drive);
            }
        }

        self.model.walk_cmd.vx = total_vx;
        self.model.walk_cmd.vy = total_vy;
        self.model.walk_cmd.yaw_rate = total_yaw;
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
