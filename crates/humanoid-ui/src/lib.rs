//! Layer 5 — Rocky Studio GUI (`humanoid-ui`).
//!
//! Built directly on `egui 0.37` + `storytold/craft-libs` (`craft-ui`, `craft-keymap`,
//! `craft-settings`) + `egui_plot` + `gilrs` + `humanoid-viewport` (`three-d`).

use std::collections::VecDeque;

use craft_keymap::{Active, Bindable, Binding, Keymap, Scope, Scopes};
use craft_ui::docking::{DockArea, DockContent, DockStyle, Layout, Node};
use craft_ui::layout::{SplitAxis, SplitSize};
use craft_ui::theme::WidgetPalette;
use egui::{Color32, CornerRadius, RichText, Stroke, Vec2};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};
use humanoid_link::{RobotTransport, SimTransport, TransportKind};
use humanoid_model::{ActionMove, AssetBundle, Keyframe, LimbGroup, StudioModel};
use humanoid_proto::{ControlMode, FaultFlags, HEAT_TINT_THRESHOLD_MA};
use humanoid_sim::{ReplayVerification, verify_deterministic_replay};
use humanoid_viewport::{CameraPreset, HumanoidViewport};
use serde::{Deserialize, Serialize};
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
    CalibrationJog,
    ActionStudio,
    PilotHealth,
    SimFaultHand,
}

impl DockPanel {
    pub fn title(self) -> &'static str {
        match self {
            Self::Viewport3D => "3D Mirror Viewport",
            Self::CalibrationJog => "Calibration & Jog",
            Self::ActionStudio => "Teach & Action Studio",
            Self::PilotHealth => "Pilot & Health",
            Self::SimFaultHand => "Sim Fault & Hand",
        }
    }
}

/// Platform export hook supplied by `apps/humanoid-desktop` (`craft-atomic`)
/// or `apps/humanoid-web` (`craft-platform::browser`).
pub trait PlatformSaver {
    fn save_file(&mut self, filename: &str, mime: &str, bytes: &[u8]) -> Result<String, String>;
}

#[derive(Default)]
pub struct NoopSaver;

impl PlatformSaver for NoopSaver {
    fn save_file(&mut self, filename: &str, _mime: &str, _bytes: &[u8]) -> Result<String, String> {
        Ok(format!("Prepared '{filename}'"))
    }
}

#[derive(Clone, Copy, Debug)]
struct TelemetryPoint {
    t_sec: f64,
    err_deg: f64,
    current_ma: f64,
    temp_c: f64,
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
    history: VecDeque<TelemetryPoint>,
    gilrs: Option<gilrs::Gilrs>,
    active_gamepad_name: Option<String>,
    last_gamepad_event: String,
    replay_result: Option<ReplayVerification>,
    new_move_name: String,
    calibration_import_buf: String,
    show_import_dialog: bool,
    saver: Box<dyn PlatformSaver>,
    selected_transport_ui: TransportKind,
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

        Self {
            model,
            transport,
            viewport: HumanoidViewport::new(),
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
            saver,
            selected_transport_ui: TransportKind::SimKernel,
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
        for (name, bytes) in dropped {
            match AssetBundle::from_bytes(&name, &bytes) {
                Ok(bundle) => {
                    if let Err(err) = self.model.load_bundle(bundle) {
                        self.model.status_message = err;
                    } else {
                        self.transport = Box::new(SimTransport::new(
                            self.model.rig.joints.len(),
                            &self.model.calibration,
                            self.model.rig.default_pose(),
                        ));
                        self.viewport.notify_rig_changed();
                    }
                }
                Err(err) => {
                    self.model.status_message = err;
                }
            }
        }

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
        let j = self.model.selected_joint.min(self.model.rig.joints.len().saturating_sub(1));
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

        // 5. Render persistent Top Toolbar
        egui::Panel::top("rocky_top_toolbar")
            .exact_size(46.0)
            .frame(
                egui::Frame::new()
                    .fill(Color32::from_rgb(15, 20, 28))
                    .inner_margin(egui::Margin::symmetric(12, 6))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(34, 44, 60))),
            )
            .show(ui, |ui| {
                self.render_top_toolbar(ui);
            });

        // 6. Render quiet bottom status ribbon (operational feedback only)
        egui::Panel::bottom("rocky_status_bar")
            .exact_size(26.0)
            .frame(
                egui::Frame::new()
                    .fill(Color32::from_rgb(12, 16, 23))
                    .inner_margin(egui::Margin::symmetric(12, 4))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(28, 36, 50))),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(&self.model.status_message)
                            .size(12.0)
                            .color(Color32::from_rgb(168, 184, 205)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(format!(
                                "URDF: {} ({} DoF)  ·  Drop .urdf / .zip onto viewport to load",
                                self.model.rig.robot_name,
                                self.model.rig.joints.len()
                            ))
                            .size(11.5)
                            .color(Color32::from_rgb(115, 132, 156)),
                        );
                    });
                });
            });

        // 7. Render Craft-UI transactional Docking Workspace in the CentralPanel
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(Color32::from_rgb(11, 14, 20)))
            .show(ui, |ui| {
                let style = DockStyle::from_ui(ui);
                let layout_snapshot = self.dock_layout.clone();
                let mut content = RockyDockContent { app: self };
                let out = DockArea::new(ui.make_persistent_id("rocky_workspace_dock")).show_content(
                    ui,
                    &layout_snapshot,
                    &style,
                    &mut content,
                );
                if !out.actions.is_empty() {
                    let _ = self.dock_layout.apply_all(out.actions, |_| Default::default());
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

        // Request smooth continuous repaint for live 60 Hz simulation & telemetry
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
                        gilrs::Button::South => self.model.play_move(0), // Guard
                        gilrs::Button::West => self.model.play_move(1),  // Left Jab
                        gilrs::Button::East => self.model.play_move(2),  // Right Cross
                        gilrs::Button::North => self.model.play_move(3), // Front Kick
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
                        match axis {
                            gilrs::Axis::LeftStickX => {
                                // Waist yaw
                                self.model.target_rad[0] = (val * 0.95).clamp(-1.05, 1.05);
                            }
                            gilrs::Axis::LeftStickY => {
                                // Left shoulder pitch
                                self.model.target_rad[3] = (-0.8 + val * 1.1).clamp(-2.4, 1.3);
                            }
                            gilrs::Axis::RightStickY => {
                                // Right shoulder pitch
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

    fn render_top_toolbar(&mut self, ui: &mut egui::Ui) {
        let stats = self.transport.stats();
        let estop = self.model.is_estop_active();
        let volts = (self.model.latest_telemetry.bus_voltage_mv as f32) / 1000.0;

        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 10.0;

            // Brand mark
            ui.label(
                RichText::new("ROCKY STUDIO")
                    .strong()
                    .size(16.0)
                    .color(Color32::from_rgb(236, 244, 255)),
            );

            ui.separator();

            // Transport selector
            let prev_transport = self.selected_transport_ui;
            egui::ComboBox::from_id_salt("transport_select")
                .selected_text(self.selected_transport_ui.label())
                .width(168.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.selected_transport_ui,
                        TransportKind::SimKernel,
                        TransportKind::SimKernel.label(),
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    ui.selectable_value(
                        &mut self.selected_transport_ui,
                        TransportKind::UsbNetworkingUdp,
                        TransportKind::UsbNetworkingUdp.label(),
                    );
                    #[cfg(target_arch = "wasm32")]
                    ui.add_enabled(
                        false,
                        egui::Button::new("USB Net UDP (Desktop Native)"),
                    );
                    ui.add_enabled(
                        false,
                        egui::Button::new("USB CDC-ACM Serial (RP2350)"),
                    );
                });
            if self.selected_transport_ui != prev_transport {
                // Safety rule: switching links always forces Teach (Torque OFF)
                self.model.set_authority_mode(ControlMode::Teach);
                match self.selected_transport_ui {
                    TransportKind::SimKernel => {
                        self.transport = Box::new(SimTransport::new(
                            self.model.rig.joints.len(),
                            &self.model.calibration,
                            self.model.rig.default_pose(),
                        ));
                        self.model.status_message =
                            "Switched to Software Sim Kernel in Teach mode.".to_string();
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    TransportKind::UsbNetworkingUdp => {
                        match humanoid_link::UdpTransport::connect("10.42.0.2:7777") {
                            Ok(udp) => {
                                self.transport = Box::new(udp);
                                self.model.status_message =
                                    "Connected USB-NCM UDP link -> 10.42.0.2:7777 (Teach mode)."
                                        .to_string();
                            }
                            Err(err) => {
                                self.selected_transport_ui = TransportKind::SimKernel;
                                self.model.status_message =
                                    format!("USB-NCM UDP bind failed: {err}");
                            }
                        }
                    }
                    _ => {
                        self.selected_transport_ui = TransportKind::SimKernel;
                    }
                }
            }

            ui.separator();

            // Clean unboxed monospace telemetry readout
            let volt_color = if self
                .model
                .latest_telemetry
                .faults
                .contains(FaultFlags::UNDER_VOLTAGE)
            {
                Color32::from_rgb(248, 113, 113)
            } else {
                Color32::from_rgb(134, 239, 172)
            };

            ui.label(
                RichText::new(format!("{:>4.0} Hz", stats.packet_rate_hz))
                    .monospace()
                    .size(12.5)
                    .color(Color32::from_rgb(192, 206, 224)),
            );
            ui.label(RichText::new("·").color(Color32::from_rgb(76, 92, 116)));
            ui.label(
                RichText::new(format!("{:.1} ms", stats.rtt_ms))
                    .monospace()
                    .size(12.5)
                    .color(Color32::from_rgb(192, 206, 224)),
            );
            ui.label(RichText::new("·").color(Color32::from_rgb(76, 92, 116)));
            ui.label(
                RichText::new(format!("Err {}", stats.link_errors))
                    .monospace()
                    .size(12.5)
                    .color(if stats.link_errors > 0 {
                        Color32::from_rgb(251, 191, 36)
                    } else {
                        Color32::from_rgb(148, 163, 184)
                    }),
            );
            ui.label(RichText::new("·").color(Color32::from_rgb(76, 92, 116)));
            ui.label(
                RichText::new(format!("{:.2} V", volts))
                    .monospace()
                    .strong()
                    .size(12.5)
                    .color(volt_color),
            );

            ui.separator();

            // Segmented Authority Toggle: Teach | Drive (greyed out when E-Stop active)
            ui.add_enabled_ui(!estop, |ui| {
                let is_teach = self.model.requested_mode == ControlMode::Teach;
                let is_drive = self.model.requested_mode == ControlMode::Drive;

                let teach_btn = egui::Button::new(
                    RichText::new("Teach (Robot -> GUI)")
                        .size(12.5)
                        .color(if is_teach {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(156, 172, 194)
                        }),
                )
                .fill(if is_teach {
                    Color32::from_rgb(22, 112, 146)
                } else {
                    Color32::from_rgb(24, 31, 44)
                })
                .corner_radius(CornerRadius::same(4));

                if ui.add(teach_btn).clicked() {
                    self.model.set_authority_mode(ControlMode::Teach);
                }

                let drive_btn = egui::Button::new(
                    RichText::new("Drive (GUI -> Robot)")
                        .size(12.5)
                        .color(if is_drive {
                            Color32::from_rgb(14, 18, 24)
                        } else {
                            Color32::from_rgb(156, 172, 194)
                        }),
                )
                .fill(if is_drive {
                    Color32::from_rgb(245, 158, 11)
                } else {
                    Color32::from_rgb(24, 31, 44)
                })
                .corner_radius(CornerRadius::same(4));

                if ui.add(drive_btn).clicked() {
                    self.model.set_authority_mode(ControlMode::Drive);
                }
            });

            ui.separator();

            // Global Speed Limiter (10% - 100%, default 25%)
            ui.label(
                RichText::new("Speed")
                    .size(12.0)
                    .color(Color32::from_rgb(156, 172, 194)),
            );
            ui.add(
                egui::Slider::new(&mut self.model.speed_limit_pct, 10..=100)
                    .suffix("%")
                    .max_decimals(0),
            );

            // Right-aligned E-Stop & explicit Reset E-Stop
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let estop_btn = egui::Button::new(
                    RichText::new("E-STOP [Space]")
                        .strong()
                        .size(13.0)
                        .color(Color32::WHITE),
                )
                .fill(Color32::from_rgb(212, 34, 44))
                .stroke(Stroke::new(1.0, Color32::from_rgb(252, 130, 136)))
                .min_size(Vec2::new(124.0, 30.0))
                .corner_radius(CornerRadius::same(4));

                if ui.add(estop_btn).clicked() {
                    self.model.trigger_estop();
                }

                if estop {
                    let reset_btn = egui::Button::new(
                        RichText::new("Reset E-Stop -> Teach")
                            .strong()
                            .size(12.5)
                            .color(Color32::from_rgb(15, 22, 32)),
                    )
                    .fill(Color32::from_rgb(251, 191, 36))
                    .min_size(Vec2::new(152.0, 30.0))
                    .corner_radius(CornerRadius::same(4));

                    if ui.add(reset_btn).clicked() {
                        if let Some(f) = self.transport.sim_faults_mut() {
                            f.drop_heartbeat = false;
                        }
                        self.model.reset_estop();
                    }
                }
            });
        });
    }

    fn render_viewport_panel(&mut self, ui: &mut egui::Ui) {
        let estop = self.model.is_estop_active();
        let faults = self.model.latest_telemetry.faults;

        // Prominent Red Safety Banner when E-Stop is latched
        if estop {
            egui::Frame::new()
                .fill(Color32::from_rgb(138, 22, 30))
                .inner_margin(egui::Margin::symmetric(12, 6))
                .stroke(Stroke::new(1.0, Color32::from_rgb(248, 113, 113)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!(
                                "E-STOP LATCHED — TORQUE OFF  |  Faults: {:?}",
                                faults
                            ))
                            .strong()
                            .size(12.5)
                            .color(Color32::WHITE),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button("Reset E-Stop -> Teach").clicked() {
                                if let Some(f) = self.transport.sim_faults_mut() {
                                    f.drop_heartbeat = false;
                                }
                                self.model.reset_estop();
                            }
                        });
                    });
                });
        } else if !faults.is_empty() {
            egui::Frame::new()
                .fill(Color32::from_rgb(92, 54, 14))
                .inner_margin(egui::Margin::symmetric(12, 5))
                .stroke(Stroke::new(1.0, Color32::from_rgb(245, 158, 11)))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new(format!("Active Advisory / Faults: {:?}", faults))
                            .strong()
                            .size(12.0)
                            .color(Color32::from_rgb(254, 243, 199)),
                    );
                });
        }

        // Compact 3D Viewport control ribbon
        egui::Frame::new()
            .fill(Color32::from_rgb(16, 21, 30))
            .inner_margin(egui::Margin::symmetric(10, 5))
            .stroke(Stroke::new(1.0, Color32::from_rgb(30, 40, 56)))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("Camera:")
                            .size(12.0)
                            .color(Color32::from_rgb(148, 163, 184)),
                    );
                    for (label, preset) in [
                        ("Iso", CameraPreset::Iso),
                        ("Front", CameraPreset::Front),
                        ("Side", CameraPreset::Side),
                        ("Top", CameraPreset::Top),
                    ] {
                        if ui.small_button(label).clicked() {
                            self.viewport.orbit.apply_preset(preset);
                        }
                    }
                    ui.separator();
                    ui.checkbox(&mut self.viewport.show_grid, "Grid");

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let show_ghost =
                            self.model.latest_telemetry.active_mode == ControlMode::Drive;
                        let legend = if show_ghost {
                            "Solid = Measured  ·  Cyan Wireframe = Target Ghost  ·  Click link to select"
                        } else {
                            "Solid = Measured Encoders (Ghost hidden in Teach/E-Stop)  ·  Click link to select"
                        };
                        ui.label(
                            RichText::new(legend)
                                .size(11.5)
                                .color(Color32::from_rgb(138, 154, 178)),
                        );
                    });
                });
            });

        // Embedded three-d 3D viewport via PaintCallback
        let show_ghost = self.model.latest_telemetry.active_mode == ControlMode::Drive;
        if let Some(picked) = self.viewport.show(
            ui,
            &self.model.rig,
            &self.model.bundle,
            &self.model.latest_telemetry.measured_rad,
            &self.model.target_rad,
            &self.model.latest_telemetry.current_ma,
            self.model.selected_joint,
            show_ghost,
            estop,
        ) {
            self.model.selected_joint = picked;
            if let Some(j) = self.model.rig.joints.get(picked) {
                self.model.status_message = format!("Selected joint: {} ({})", j.label, j.name);
            }
        }
    }

    /// Panel 1 — Calibration & Jog
    fn render_calibration_panel(&mut self, ui: &mut egui::Ui) {
        let is_drive =
            self.model.requested_mode == ControlMode::Drive && !self.model.is_estop_active();

        ui.horizontal(|ui| {
            if ui.button("Save Calibration").clicked() {
                match self.model.export_calibration_json() {
                    Ok(json) => {
                        match self.saver.save_file(
                            "rocky_calibration.json",
                            "application/json",
                            json.as_bytes(),
                        ) {
                            Ok(msg) => self.model.status_message = msg,
                            Err(err) => self.model.status_message = err,
                        }
                    }
                    Err(err) => self.model.status_message = err,
                }
            }
            if ui.button("Import JSON").clicked() {
                self.show_import_dialog = true;
            }
            if ui.button("Commit Flash").clicked() {
                self.model
                    .pending_config
                    .push(humanoid_proto::ConfigCmd::CommitFlash);
                self.model.status_message = "Queued ConfigCmd::CommitFlash to MCU.".to_string();
            }
        });

        ui.add_space(6.0);

        // Selected joint detailed calibration card
        let sel = self
            .model
            .selected_joint
            .min(self.model.rig.joints.len().saturating_sub(1));
        if let (Some(jmeta), Some(cal)) = (
            self.model.rig.joints.get(sel).cloned(),
            self.model.calibration.get(sel).copied(),
        ) {
            let measured_rad = self.model.latest_telemetry.measured_rad[sel];
            let applied_rad = self.model.latest_telemetry.applied_target_rad[sel];
            let raw_rad = humanoid_control::calibrated_to_raw(measured_rad, &cal);
            let ma = self.model.latest_telemetry.current_ma[sel];
            let temp = self.model.latest_telemetry.temp_c[sel];

            egui::Frame::new()
                .fill(Color32::from_rgb(18, 24, 34))
                .corner_radius(CornerRadius::same(6))
                .inner_margin(egui::Margin::same(10))
                .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("#{:02} {}", sel, jmeta.label))
                                .strong()
                                .size(14.0)
                                .color(Color32::from_rgb(103, 232, 249)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(
                                RichText::new(format!("{} mA · {} °C", ma, temp))
                                    .monospace()
                                    .size(12.0)
                                    .color(if ma >= HEAT_TINT_THRESHOLD_MA {
                                        Color32::from_rgb(248, 113, 113)
                                    } else {
                                        Color32::from_rgb(168, 184, 205)
                                    }),
                            );
                        });
                    });

                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(format!(
                            "Cal: {:+6.1}°  |  Raw: {:+6.1}°  |  Applied: {:+6.1}°",
                            measured_rad.to_degrees(),
                            raw_rad.to_degrees(),
                            applied_rad.to_degrees()
                        ))
                        .monospace()
                        .size(12.0)
                        .color(Color32::from_rgb(212, 224, 240)),
                    );

                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("Set Current as Zero").clicked() {
                            self.model.set_current_as_zero(sel);
                        }
                        let mut inv = cal.inverted;
                        if ui.checkbox(&mut inv, "Invert Sign").changed() {
                            self.model.toggle_invert(sel);
                        }
                        let mut torque_on = ((self.model.torque_mask >> sel) & 1) == 1;
                        if ui.checkbox(&mut torque_on, "Torque").changed() {
                            self.model.set_joint_torque(sel, torque_on);
                        }
                    });

                    ui.add_space(4.0);
                    let mut min_deg = cal.min_rad.to_degrees();
                    let mut max_deg = cal.max_rad.to_degrees();
                    ui.horizontal(|ui| {
                        ui.label("Soft Min:");
                        let r1 = ui.add(
                            egui::DragValue::new(&mut min_deg)
                                .speed(1.0)
                                .range(-180.0..=180.0)
                                .suffix("°"),
                        );
                        ui.label("Max:");
                        let r2 = ui.add(
                            egui::DragValue::new(&mut max_deg)
                                .speed(1.0)
                                .range(-180.0..=180.0)
                                .suffix("°"),
                        );
                        if r1.changed() || r2.changed() {
                            self.model
                                .update_soft_limits(sel, min_deg.to_radians(), max_deg.to_radians());
                        }
                    });
                });
        }

        ui.add_space(6.0);
        ui.label(
            RichText::new(if is_drive {
                "Joint Jog Sliders (Active — Drive Mode)"
            } else {
                "Joint Angles (Read-Only Mirror — Teach / E-Stop)"
            })
            .size(12.0)
            .color(Color32::from_rgb(148, 163, 184)),
        );
        ui.separator();

        egui::ScrollArea::vertical()
            .id_salt("cal_joint_list")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for group in LimbGroup::ALL {
                    let indices: Vec<usize> = self
                        .model
                        .rig
                        .joints
                        .iter()
                        .filter(|j| j.group == group)
                        .map(|j| j.index)
                        .collect();
                    if indices.is_empty() {
                        continue;
                    }
                    egui::CollapsingHeader::new(
                        RichText::new(group.label())
                            .strong()
                            .color(Color32::from_rgb(203, 213, 225)),
                    )
                    .default_open(true)
                    .show(ui, |ui| {
                        for idx in indices {
                            let label = self.model.rig.joints[idx].label.clone();
                            let cal = self.model.calibration[idx];
                            let measured_deg =
                                self.model.latest_telemetry.measured_rad[idx].to_degrees();
                            let ma = self.model.latest_telemetry.current_ma[idx];
                            let selected = self.model.selected_joint == idx;

                            ui.horizontal(|ui| {
                                if ui
                                    .selectable_label(
                                        selected,
                                        RichText::new(label).size(12.0),
                                    )
                                    .clicked()
                                {
                                    self.model.selected_joint = idx;
                                }

                                let mut deg = if is_drive {
                                    self.model.target_rad[idx].to_degrees()
                                } else {
                                    measured_deg
                                };
                                let lo = cal.min_rad.to_degrees();
                                let hi = cal.max_rad.to_degrees();
                                ui.add_enabled_ui(is_drive, |ui| {
                                    if ui
                                        .add(
                                            egui::Slider::new(&mut deg, lo..=hi)
                                                .suffix("°")
                                                .max_decimals(1),
                                        )
                                        .changed()
                                    {
                                        self.model.playback = None;
                                        self.model.target_rad[idx] = deg.to_radians();
                                        self.model.selected_joint = idx;
                                    }
                                });
                                if ma >= HEAT_TINT_THRESHOLD_MA {
                                    ui.label(
                                        RichText::new(format!("{}mA", ma))
                                            .monospace()
                                            .size(11.0)
                                            .color(Color32::from_rgb(248, 113, 113)),
                                    );
                                }
                            });
                        }
                    });
                }
            });
    }

    /// Panel 2 — Teach & Action Studio
    fn render_action_studio_panel(&mut self, ui: &mut egui::Ui) {
        let is_teach =
            self.model.requested_mode == ControlMode::Teach && !self.model.is_estop_active();
        let is_drive =
            self.model.requested_mode == ControlMode::Drive && !self.model.is_estop_active();

        ui.horizontal(|ui| {
            let cap_btn = egui::Button::new("Capture Keyframe (Teach)");
            if ui.add_enabled(is_teach, cap_btn).clicked() {
                self.model.capture_keyframe();
            }
            if ui.button("Export Moves").clicked() {
                if let Ok(json) = serde_json::to_string_pretty(&self.model.moves) {
                    match self.saver.save_file(
                        "rocky_moves.json",
                        "application/json",
                        json.as_bytes(),
                    ) {
                        Ok(msg) => self.model.status_message = msg,
                        Err(err) => self.model.status_message = err,
                    }
                }
            }
        });

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ui.text_edit_singleline(&mut self.new_move_name);
            if ui.button("+ New Move").clicked() && !self.new_move_name.trim().is_empty() {
                let name = self.new_move_name.trim().to_string();
                self.model.moves.push(ActionMove {
                    name,
                    category: "Custom".to_string(),
                    keyframes: vec![Keyframe {
                        label: "Start Pose".to_string(),
                        duration_ms: 450,
                        angles_rad: self.model.latest_telemetry.measured_rad,
                    }],
                });
                self.model.selected_move = self.model.moves.len() - 1;
            }
        });

        ui.add_space(6.0);
        ui.label(
            RichText::new("Combat Move Library")
                .strong()
                .size(12.5)
                .color(Color32::from_rgb(192, 206, 224)),
        );

        let move_count = self.model.moves.len();
        for idx in 0..move_count {
            let name = self.model.moves[idx].name.clone();
            let cat = self.model.moves[idx].category.clone();
            let kf_len = self.model.moves[idx].keyframes.len();
            let is_selected = self.model.selected_move == idx;
            let is_playing = self
                .model
                .playback
                .as_ref()
                .is_some_and(|p| p.move_idx == idx);

            ui.horizontal(|ui| {
                if ui
                    .selectable_label(
                        is_selected,
                        RichText::new(format!("{name}  ·  {cat} ({kf_len} kf)")).size(12.5),
                    )
                    .clicked()
                {
                    self.model.selected_move = idx;
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if is_playing {
                        if ui.button("Stop").clicked() {
                            self.model.stop_playback();
                        }
                    } else if ui
                        .add_enabled(is_drive, egui::Button::new("Play"))
                        .clicked()
                    {
                        self.model.play_move(idx);
                    }
                });
            });
        }

        if let Some(pb) = &self.model.playback {
            ui.add_space(6.0);
            ui.add(
                egui::ProgressBar::new(pb.progress)
                    .text(format!(
                        "Playing KF #{} ({:.0}%)",
                        pb.active_keyframe + 1,
                        pb.progress * 100.0
                    ))
                    .fill(Color32::from_rgb(245, 158, 11)),
            );
        }

        ui.separator();
        ui.label(
            RichText::new("Keyframes (Cubic Hermite Interpolation)")
                .strong()
                .size(12.5)
                .color(Color32::from_rgb(192, 206, 224)),
        );

        egui::ScrollArea::vertical()
            .id_salt("keyframe_editor_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                let sel_move = self.model.selected_move;
                if let Some(mv) = self.model.moves.get_mut(sel_move) {
                    let mut remove_idx = None;
                    for (k_idx, kf) in mv.keyframes.iter_mut().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("#{:02}", k_idx + 1))
                                    .monospace()
                                    .color(Color32::from_rgb(103, 232, 249)),
                            );
                            ui.add(egui::TextEdit::singleline(&mut kf.label).desired_width(120.0));
                            ui.add(
                                egui::DragValue::new(&mut kf.duration_ms)
                                    .range(80..=5000)
                                    .speed(10)
                                    .suffix(" ms"),
                            );
                            if ui.small_button("Del").clicked() {
                                remove_idx = Some(k_idx);
                            }
                        });
                    }
                    if let Some(rm) = remove_idx {
                        if mv.keyframes.len() > 1 {
                            mv.keyframes.remove(rm);
                        }
                    }
                }
            });
    }

    /// Panel 3 — Manual Pilot (`gilrs`), Live `egui_plot` Charts & Deterministic Replay
    fn render_pilot_health_panel(&mut self, ui: &mut egui::Ui) {
        let is_drive =
            self.model.requested_mode == ControlMode::Drive && !self.model.is_estop_active();

        egui::ScrollArea::vertical()
            .id_salt("pilot_health_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                // Gamepad status (gilrs)
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("Manual Pilot (gilrs Gamepad)")
                                .strong()
                                .size(13.0)
                                .color(Color32::from_rgb(226, 232, 240)),
                        );
                        let pad_text = self
                            .active_gamepad_name
                            .as_deref()
                            .unwrap_or("No USB/Bluetooth controller detected (press a button to wake)");
                        ui.label(
                            RichText::new(pad_text)
                                .size(12.0)
                                .color(Color32::from_rgb(148, 163, 184)),
                        );
                        ui.label(
                            RichText::new(format!("Last event: {}", self.last_gamepad_event))
                                .monospace()
                                .size(11.5)
                                .color(Color32::from_rgb(103, 232, 249)),
                        );
                        ui.label(
                            RichText::new(
                                "Bindings: A=Guard · X=Left Jab · B=Right Cross · Y=Front Kick · Start=E-Stop",
                            )
                            .size(11.0)
                            .color(Color32::from_rgb(125, 142, 166)),
                        );
                        ui.add_space(4.0);
                        ui.horizontal_wrapped(|ui| {
                            for (idx, label) in [
                                (0, "Guard"),
                                (1, "Left Jab"),
                                (2, "Right Cross"),
                                (3, "Front Kick"),
                            ] {
                                if ui
                                    .add_enabled(is_drive, egui::Button::new(label))
                                    .clicked()
                                {
                                    self.model.play_move(idx);
                                }
                            }
                        });
                    });

                ui.add_space(8.0);

                // Live egui_plot charts for selected joint
                let sel = self
                    .model
                    .selected_joint
                    .min(self.model.rig.joints.len().saturating_sub(1));
                let joint_label = self.model.rig.joints[sel].label.clone();

                ui.label(
                    RichText::new(format!("Live Telemetry — #{:02} {}", sel, joint_label))
                        .strong()
                        .size(13.0)
                        .color(Color32::from_rgb(226, 232, 240)),
                );

                // 1. Tracking Error (deg)
                let err_pts: PlotPoints = self
                    .history
                    .iter()
                    .map(|p| [p.t_sec, p.err_deg])
                    .collect();
                Plot::new("plot_tracking_error")
                    .height(105.0)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .legend(Legend::default())
                    .show(ui, |plot_ui| {
                        plot_ui.line(
                            Line::new("Tracking Err (deg)", err_pts)
                                .color(Color32::from_rgb(56, 189, 248))
                                .width(1.8),
                        );
                    });

                ui.add_space(4.0);

                // 2. Current (mA) with 70% heat-tint reference line
                let ma_pts: PlotPoints = self
                    .history
                    .iter()
                    .map(|p| [p.t_sec, p.current_ma])
                    .collect();
                Plot::new("plot_current_ma")
                    .height(105.0)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .include_y(0.0)
                    .include_y(2400.0)
                    .legend(Legend::default())
                    .show(ui, |plot_ui| {
                        plot_ui.hline(
                            HLine::new("70% Heat Threshold", HEAT_TINT_THRESHOLD_MA as f64)
                                .color(Color32::from_rgb(245, 158, 11)),
                        );
                        plot_ui.line(
                            Line::new("Current (mA)", ma_pts)
                                .color(Color32::from_rgb(248, 113, 113))
                                .width(1.8),
                        );
                    });

                ui.add_space(4.0);

                // 3. Temperature (C)
                let temp_pts: PlotPoints = self
                    .history
                    .iter()
                    .map(|p| [p.t_sec, p.temp_c])
                    .collect();
                Plot::new("plot_temp_c")
                    .height(95.0)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .include_y(20.0)
                    .include_y(85.0)
                    .legend(Legend::default())
                    .show(ui, |plot_ui| {
                        plot_ui.line(
                            Line::new("Temp (°C)", temp_pts)
                                .color(Color32::from_rgb(251, 191, 36))
                                .width(1.8),
                        );
                    });

                ui.add_space(8.0);

                // Deterministic .copper Replay Recorder & Verifier
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("Deterministic Copper Replay Log")
                                .strong()
                                .size(13.0),
                        );
                        let is_rec = self.transport.is_recording();
                        let tick_count = self
                            .transport
                            .recorded_log()
                            .map(|l| l.ticks.len())
                            .unwrap_or(0);

                        ui.horizontal(|ui| {
                            if is_rec {
                                if ui.button("Stop Recording").clicked() {
                                    let _ = self.transport.stop_recording();
                                    self.model.status_message =
                                        format!("Stopped .copper recording ({tick_count} ticks).");
                                }
                            } else if ui.button("Start .copper Recording").clicked() {
                                self.replay_result = None;
                                self.transport.start_recording();
                                self.model.status_message =
                                    "Recording CopperList wire ticks...".to_string();
                            }

                            if ui
                                .add_enabled(tick_count > 0 && !is_rec, egui::Button::new("Verify Replay"))
                                .clicked()
                            {
                                if let Some(log) = self.transport.recorded_log() {
                                    let check = verify_deterministic_replay(log);
                                    self.replay_result = Some(check);
                                    self.model.status_message = format!(
                                        "Deterministic replay: {}/{} ticks exact match.",
                                        check.matched_ticks, check.total_ticks
                                    );
                                }
                            }

                            if ui
                                .add_enabled(tick_count > 0 && !is_rec, egui::Button::new("Export .copper"))
                                .clicked()
                            {
                                if let Some(log) = self.transport.recorded_log() {
                                    if let Ok(bytes) = serde_json::to_vec_pretty(log) {
                                        match self.saver.save_file(
                                            "rocky_session.copper",
                                            "application/json",
                                            &bytes,
                                        ) {
                                            Ok(msg) => self.model.status_message = msg,
                                            Err(err) => self.model.status_message = err,
                                        }
                                    }
                                }
                            }
                        });

                        if let Some(res) = self.replay_result {
                            ui.label(
                                RichText::new(format!(
                                    "Replay check: {}/{} matched ({})",
                                    res.matched_ticks,
                                    res.total_ticks,
                                    if res.exact_match { "BIT-EXACT PASS" } else { "MISMATCH" }
                                ))
                                .monospace()
                                .size(12.0)
                                .color(if res.exact_match {
                                    Color32::from_rgb(134, 239, 172)
                                } else {
                                    Color32::from_rgb(248, 113, 113)
                                }),
                            );
                        }
                    });
            });
    }

    /// Panel 4 — Sim Fault & Virtual-Hand Drawer (Visible in Sim Mode)
    fn render_sim_fault_panel(&mut self, ui: &mut egui::Ui) {
        let sel = self
            .model
            .selected_joint
            .min(self.model.rig.joints.len().saturating_sub(1));
        let sel_label = self.model.rig.joints[sel].label.clone();

        egui::ScrollArea::vertical()
            .id_salt("sim_fault_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                // Fault Injection Suite (Gate 1.6 tests)
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("Hardware Fault Injection")
                                .strong()
                                .size(13.0)
                                .color(Color32::from_rgb(251, 191, 36)),
                        );
                        ui.add_space(4.0);

                        if let Some(faults) = self.transport.sim_faults_mut() {
                            ui.checkbox(
                                &mut faults.drop_heartbeat,
                                "Drop Heartbeat (Tests 100 ms Watchdog)",
                            );
                            ui.checkbox(
                                &mut faults.stalled_joints[sel],
                                format!("Inject Joint Stall on '{}' (>2200 mA)", sel_label),
                            );
                            ui.checkbox(
                                &mut faults.voltage_sag,
                                "Voltage Sag (<10.5 V Under-Voltage)",
                            );
                            ui.checkbox(&mut faults.servo_bus_error, "Servo Bus Error");
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                ui.label("Actuator Response:");
                                ui.add(
                                    egui::Slider::new(&mut faults.servo_lag_factor, 0.03..=0.35)
                                        .max_decimals(2),
                                );
                            });
                        }

                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            if ui.button("Inject Corrupted COBS Frame").clicked() {
                                self.transport.inject_corrupted_frame();
                                self.model.status_message =
                                    "Injected corrupted COBS bytes — silently dropped, link error counter incremented.".to_string();
                            }
                            if ui.button("Step +90° Jump Test").clicked() {
                                if self.model.requested_mode == ControlMode::Drive
                                    && !self.model.is_estop_active()
                                {
                                    self.model.playback = None;
                                    self.model.target_rad[sel] =
                                        (self.model.target_rad[sel] + std::f32::consts::FRAC_PI_2)
                                            .clamp(
                                                self.model.calibration[sel].min_rad,
                                                self.model.calibration[sel].max_rad,
                                            );
                                    self.model.status_message = format!(
                                        "Commanded +90° step on '{}' (MCU slew limiter active).",
                                        sel_label
                                    );
                                } else {
                                    self.model.status_message =
                                        "Switch to Drive mode to run the +90° slew-clamp step test."
                                            .to_string();
                                }
                            }
                        });
                    });

                ui.add_space(8.0);

                // Virtual Physical Robot Hand-Posing (Teach mode Robot -> GUI verification)
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("Virtual Physical Robot (Teach Mode Hand-Posing)")
                                .strong()
                                .size(13.0)
                                .color(Color32::from_rgb(103, 232, 249)),
                        );
                        ui.label(
                            RichText::new(
                                "Move simulated physical limbs while Torque is OFF to test Robot -> GUI mirroring end-to-end.",
                            )
                            .size(11.5)
                            .color(Color32::from_rgb(148, 163, 184)),
                        );

                        if let Some(faults) = self.transport.sim_faults_mut() {
                            ui.add_space(4.0);
                            ui.checkbox(
                                &mut faults.external_hand_wave,
                                "External Hand Wave (Sinusoidal Auto-Pose)",
                            );
                        }

                        ui.add_space(6.0);
                        let hand_copy = self
                            .transport
                            .virtual_hand_angles()
                            .copied()
                            .unwrap_or([0.0; humanoid_proto::MAX_JOINTS]);

                        let is_torque_off = self.model.latest_telemetry.torque_state_mask == 0;
                        if !is_torque_off {
                            ui.label(
                                RichText::new("Switch to Teach mode (Torque OFF) to hand-pose limbs.")
                                    .size(11.5)
                                    .color(Color32::from_rgb(251, 191, 36)),
                            );
                        }

                        ui.add_enabled_ui(is_torque_off, |ui| {
                            for j in &self.model.rig.joints {
                                let mut deg = hand_copy[j.index].to_degrees();
                                let lo = j.default_min_rad.to_degrees();
                                let hi = j.default_max_rad.to_degrees();
                                ui.horizontal(|ui| {
                                    ui.label(RichText::new(&j.label).size(12.0));
                                    if ui
                                        .add(
                                            egui::Slider::new(&mut deg, lo..=hi)
                                                .suffix("°")
                                                .max_decimals(1),
                                        )
                                        .changed()
                                    {
                                        self.transport
                                            .set_virtual_hand_angle(j.index, deg.to_radians());
                                    }
                                });
                            }
                        });
                    });
            });
    }
}

struct RockyDockContent<'a> {
    app: &'a mut RockyStudioApp,
}

impl DockContent<DockPanel> for RockyDockContent<'_> {
    fn body(&mut self, ui: &mut egui::Ui, panel: &DockPanel) {
        match panel {
            DockPanel::Viewport3D => self.app.render_viewport_panel(ui),
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
            size: SplitSize::Ratio(0.64),
            first: Box::new(Node::Tabs {
                panels: vec![DockPanel::Viewport3D],
                active: 0,
            }),
            second: Box::new(Node::Tabs {
                panels: vec![
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
