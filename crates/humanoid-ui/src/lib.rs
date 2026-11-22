//! Layer 5 — Rocky Studio GUI (`humanoid-ui`).
//!
//! Built directly on `egui 0.37` + `storytold/craft-libs` (`craft-ui`, `craft-keymap`,
//! `craft-settings`) + `egui_plot` + `gilrs` + `humanoid-viewport` (`three-d`).
//!
//! Organized into focused panel submodules:
//! - [`top_bar`]: Persistent safety toolbar, transport selector, and Android Tablet adaptive layout
//! - [`viewport_panel`]: 3D Mirror Viewport panel, E-Stop banner, and camera presets
//! - [`urdf_panel`]: URDF loader, kinematic tree inspector, and MCU flash sync
//! - [`calibration_tab`]: Joint calibration, soft limits, precision nudge, and jog sliders
//! - [`teach_tab`]: Teach & Action Studio keyframe capture, move library, and touch combat pad
//! - [`pilot_tab`]: `gilrs` manual pilot, live `egui_plot` charts, and deterministic Copper replay
//! - [`sim_tab`]: Simulation fault injection and virtual physical robot hand-posing
//! - [`responsive`]: Desktop + Android Tablet ergonomics resolver

mod calibration_tab;
mod pilot_tab;
mod responsive;
mod sim_tab;
mod teach_tab;
mod top_bar;
mod urdf_panel;
mod viewport_panel;

use std::collections::VecDeque;

use craft_keymap::{Active, Bindable, Binding, Keymap, Scope, Scopes};
use craft_ui::docking::{DockArea, DockContent, DockStyle, Layout, Node};
use craft_ui::layout::{SplitAxis, SplitSize};
use craft_ui::theme::WidgetPalette;
use egui::{Color32, CornerRadius, RichText, Stroke, Vec2};
use humanoid_link::{RobotTransport, SimTransport, TransportKind};
use humanoid_model::{AssetBundle, StudioModel};
use humanoid_proto::{ControlMode, ModelFormat};
use humanoid_sim::ReplayVerification;
use humanoid_viewport::HumanoidViewport;
pub use responsive::{FormFactor, UiMetrics, UiProfileMode};
use responsive::apply_responsive_style;
use serde::{Deserialize, Serialize};
use urdf_panel::McuModelSyncState;
use web_time::Instant;

const TELEMETRY_HISTORY_LEN: usize = 180;
const CMD_ESTOP: &str = "safety.estop";

#[cfg(target_arch = "wasm32")]
thread_local! {
    static PENDING_WASM_DROPS: std::cell::RefCell<Vec<(String, Vec<u8>)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DockPanel {
    Viewport3D,
    UrdfModel,
    CalibrationJog,
    ActionStudio,
    PilotHealth,
    SimFaultHand,
}

impl DockPanel {
    pub const INSPECTOR_PANELS: [Self; 5] = [
        Self::UrdfModel,
        Self::CalibrationJog,
        Self::ActionStudio,
        Self::PilotHealth,
        Self::SimFaultHand,
    ];

    pub fn title(self) -> &'static str {
        match self {
            Self::Viewport3D => "3D Mirror Viewport",
            Self::UrdfModel => "URDF & MCU Model",
            Self::CalibrationJog => "Calibration & Jog",
            Self::ActionStudio => "Teach & Action Studio",
            Self::PilotHealth => "Pilot & Health",
            Self::SimFaultHand => "Sim Fault & Hand",
        }
    }

    pub fn short_label(self) -> &'static str {
        match self {
            Self::Viewport3D => "3D View",
            Self::UrdfModel => "URDF / MCU",
            Self::CalibrationJog => "Joints",
            Self::ActionStudio => "Moves",
            Self::PilotHealth => "Pilot",
            Self::SimFaultHand => "Sim",
        }
    }
}

/// Platform file I/O hook supplied by `apps/humanoid-desktop` (`craft-atomic` + native picker)
/// or `apps/humanoid-web` (`craft-platform::browser` + pure-Rust `web-sys` `<input type="file">`).
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
    pub(crate) err_deg: f64,
    pub(crate) current_ma: f64,
    pub(crate) temp_c: f64,
}

/// Complete Rocky Studio UI state shared identically between Desktop and Browser WASM.
pub struct RockyStudioApp {
    pub model: StudioModel,
    pub transport: Box<dyn RobotTransport>,
    pub viewport: HumanoidViewport,
    pub dock_layout: Layout<DockPanel>,
    keymap: Keymap,
    keymap_dispatcher: craft_keymap::egui::Dispatcher,
    keymap_platform: craft_keymap::Platform,
    theme_applied: bool,
    last_frame: Instant,
    sim_time_sec: f64,
    pub(crate) history: VecDeque<TelemetryPoint>,
    gilrs: Option<gilrs::Gilrs>,
    pub(crate) active_gamepad_name: Option<String>,
    pub(crate) last_gamepad_event: String,
    pub(crate) replay_result: Option<ReplayVerification>,
    pub(crate) new_move_name: String,
    calibration_import_buf: String,
    pub(crate) show_import_dialog: bool,
    pub(crate) paste_urdf_buf: String,
    pub(crate) show_paste_urdf_dialog: bool,
    pub(crate) mcu_sync: McuModelSyncState,
    pub(crate) staged_sim_model: Option<(String, ModelFormat, Vec<u8>, u8)>,
    pub ui_profile_mode: UiProfileMode,
    pub(crate) ui_metrics: UiMetrics,
    applied_form_factor: Option<FormFactor>,
    seen_touch_input: bool,
    pub(crate) active_tablet_panel: DockPanel,
    pub(crate) tablet_full_3d: bool,
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

        let dock_layout = default_dock_layout();
        let platform = craft_keymap::Platform::Linux;
        let keymap = build_rocky_keymap(platform);

        let gilrs = gilrs::Gilrs::new().ok();

        let mut viewport = HumanoidViewport::new();
        viewport.fit_to_rig(&model.rig);

        let default_metrics =
            UiMetrics::resolve(UiProfileMode::Auto, Vec2::new(1440.0, 900.0), false);

        let mut app = Self {
            model,
            transport,
            viewport,
            dock_layout,
            keymap,
            keymap_dispatcher: craft_keymap::egui::Dispatcher::new(),
            keymap_platform: platform,
            theme_applied: false,
            last_frame: Instant::now(),
            sim_time_sec: 0.0,
            history: VecDeque::with_capacity(TELEMETRY_HISTORY_LEN + 4),
            gilrs,
            active_gamepad_name: None,
            last_gamepad_event: "No controller input yet".to_string(),
            replay_result: None,
            new_move_name: "Custom Combo".to_string(),
            calibration_import_buf: String::new(),
            show_import_dialog: false,
            paste_urdf_buf: String::new(),
            show_paste_urdf_dialog: false,
            mcu_sync: McuModelSyncState::default(),
            staged_sim_model: None,
            ui_profile_mode: UiProfileMode::Auto,
            ui_metrics: default_metrics,
            applied_form_factor: None,
            seen_touch_input: false,
            active_tablet_panel: DockPanel::UrdfModel,
            tablet_full_3d: false,
            saver,
            selected_transport_ui: TransportKind::SimKernel,
        };
        app.query_mcu_model_info();
        app
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

    pub fn update_frame(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();

        if !self.theme_applied {
            apply_rocky_theme(&ctx);
            self.keymap_platform = craft_keymap::egui::platform(&ctx);
            self.keymap = build_rocky_keymap(self.keymap_platform);
            self.theme_applied = true;
        }

        // Detect touch input and resolve Desktop vs Android Tablet ergonomics for this frame
        if ctx.input(|i| i.any_touches() || i.multi_touch().is_some()) {
            self.seen_touch_input = true;
        }
        let viewport_size = ctx.content_rect().size();
        self.ui_metrics =
            UiMetrics::resolve(self.ui_profile_mode, viewport_size, self.seen_touch_input);
        self.viewport.pick_radius_scale = self.ui_metrics.viewport_pick_scale;
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
        if estop_pressed {
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

        // 2b. Step active chunked MCU model transfer (if pulling from MCU flash)
        self.step_mcu_model_pull();

        // 3. Poll gilrs gamepad input (Desktop + Browser Gamepad API)
        self.poll_gamepad();

        // 4. Advance real-time control loop & transport
        let now = Instant::now();
        let dt_sec = now.duration_since(self.last_frame).as_secs_f32().clamp(0.004, 0.05);
        self.last_frame = now;
        self.sim_time_sec += dt_sec as f64;

        let host_pkt = self.model.build_next_host_packet(dt_sec);
        if let Some(telem) = self.transport.step_exchange(dt_sec, &host_pkt) {
            self.model.ingest_telemetry(telem);
        }

        // Record telemetry sample for live egui_plot charts
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
        });
        while self.history.len() > TELEMETRY_HISTORY_LEN {
            self.history.pop_front();
        }

        // 5. Render persistent Top Toolbar (adapts height & controls for Desktop vs Android Tablet)
        let top_h = self.ui_metrics.top_bar_height;
        egui::Panel::top("rocky_top_toolbar")
            .exact_size(top_h)
            .frame(
                egui::Frame::new()
                    .fill(Color32::from_rgb(15, 20, 28))
                    .inner_margin(egui::Margin::symmetric(
                        if self.ui_metrics.touch_mode { 10 } else { 12 },
                        6,
                    ))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(34, 44, 60))),
            )
            .show(ui, |ui| {
                self.render_top_toolbar(ui);
            });

        // 6. Render quiet bottom status ribbon
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
                            ui.label(
                                RichText::new(format!(
                                    "URDF: {} ({} DoF)  ·  {}",
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

        // 7. Render Workspace (Desktop Docking vs Android Tablet Touch Split)
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(Color32::from_rgb(11, 14, 20)))
            .show(ui, |ui| {
                if self.ui_metrics.touch_mode {
                    self.render_tablet_workspace(ui);
                } else {
                    let style = DockStyle::from_ui(ui);
                    let layout_snapshot = self.dock_layout.clone();
                    let mut content = RockyDockContent { app: self };
                    let out = DockArea::new(ui.make_persistent_id("rocky_workspace_dock"))
                        .show_content(ui, &layout_snapshot, &style, &mut content);
                    if !out.actions.is_empty() {
                        let _ = self
                            .dock_layout
                            .apply_all(out.actions, |_| Default::default());
                    }
                }
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

        // Visual drag-and-drop overlay when hovering a .urdf/.zip file over the workstation
        let hovered_files = ctx.input(|i| i.raw.hovered_files.len());
        if hovered_files > 0 {
            let screen_rect = ctx.content_rect();
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                ui.id().with("rocky_drop_overlay"),
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
                gilrs::EventType::AxisChanged(axis, val, _) if val.abs() > 0.15 => {
                    self.last_gamepad_event = format!("Axis {:?} = {:.2}", axis, val);
                    if self.model.requested_mode == ControlMode::Drive
                        && !self.model.is_estop_active()
                    {
                        let dof = self.model.rig.joints.len();
                        match axis {
                            gilrs::Axis::LeftStickX if dof > 0 => {
                                self.model.target_rad[0] = (val * 0.95).clamp(-1.05, 1.05);
                            }
                            gilrs::Axis::LeftStickY if dof > 3 => {
                                self.model.target_rad[3] = (-0.8 + val * 1.1).clamp(-2.4, 1.3);
                            }
                            gilrs::Axis::RightStickY if dof > 6 => {
                                self.model.target_rad[6] = (-0.8 + val * 1.1).clamp(-2.4, 1.3);
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
}

struct RockyDockContent<'a> {
    app: &'a mut RockyStudioApp,
}

impl DockContent<DockPanel> for RockyDockContent<'_> {
    fn body(&mut self, ui: &mut egui::Ui, panel: &DockPanel) {
        match panel {
            DockPanel::Viewport3D => self.app.render_viewport_panel(ui),
            DockPanel::UrdfModel => {
                egui::Frame::new()
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| self.app.render_urdf_mcu_panel(ui));
            }
            DockPanel::CalibrationJog => {
                egui::Frame::new()
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| self.app.render_calibration_panel(ui));
            }
            DockPanel::ActionStudio => {
                egui::Frame::new()
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| self.app.render_action_studio_panel(ui));
            }
            DockPanel::PilotHealth => {
                egui::Frame::new()
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| self.app.render_pilot_health_panel(ui));
            }
            DockPanel::SimFaultHand => {
                egui::Frame::new()
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| self.app.render_sim_fault_panel(ui));
            }
        }
    }

    fn logical_label(&self, panel: &DockPanel) -> String {
        panel.title().to_string()
    }
}

fn default_dock_layout() -> Layout<DockPanel> {
    Layout {
        root: Some(Node::Split {
            axis: SplitAxis::Horizontal,
            size: SplitSize::Ratio(0.62),
            first: Box::new(Node::Tabs {
                panels: vec![DockPanel::Viewport3D],
                active: 0,
            }),
            second: Box::new(Node::Tabs {
                panels: vec![
                    DockPanel::UrdfModel,
                    DockPanel::CalibrationJog,
                    DockPanel::ActionStudio,
                    DockPanel::PilotHealth,
                    DockPanel::SimFaultHand,
                ],
                active: 0,
            }),
        }),
        floating: Vec::new(),
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
        CornerRadius::same(4),
        Stroke::new(1.0, Color32::from_rgb(218, 228, 242)),
    );
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(34, 46, 64));
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(44, 58, 80));
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(56, 189, 248));

    ctx.set_visuals(visuals);
}
