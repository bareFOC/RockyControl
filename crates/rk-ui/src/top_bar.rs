//! Persistent contextual top safety toolbar and safety fault banners for the 3-screen app.

use egui::{Color32, CornerRadius, Margin, RichText, Stroke, Vec2};
use rk_proto::{ControlMode, FaultFlags};

use crate::{AppScreen, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_top_toolbar(&mut self, ui: &mut egui::Ui) {
        let estop = self.model.is_estop_active();
        let volts = (self.model.latest_telemetry.bus_voltage_mv as f32) / 1000.0;
        let m = self.ui_metrics;
        let is_portrait = m.form_factor.is_portrait();

        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = 8.0;

            if self.active_screen != AppScreen::Dashboard {
                let back_btn = egui::Button::new(
                    RichText::new("< Home")
                        .strong()
                        .size(12.5)
                        .color(Color32::from_rgb(206, 224, 246)),
                )
                .fill(Color32::from_rgb(22, 31, 46))
                .stroke(Stroke::new(1.0, Color32::from_rgb(48, 66, 92)))
                .min_size(Vec2::new(72.0, m.authority_btn_height))
                .corner_radius(CornerRadius::same(5));
                if ui.add(back_btn).clicked() {
                    self.active_screen = AppScreen::Dashboard;
                }
                ui.separator();
            }

            ui.label(
                RichText::new(if m.compact_toolbar { "ROCKY" } else { "ROCKY STUDIO" })
                    .strong()
                    .size(15.5)
                    .color(Color32::from_rgb(236, 244, 255)),
            );

            if self.connected {
                ui.separator();

                // Contextual screen switcher between Control and Config & Calibration
                for (screen, label) in [
                    (AppScreen::Control, "Control"),
                    (
                        AppScreen::Config,
                        if m.compact_toolbar { "Config" } else { "Config & Cal" },
                    ),
                ] {
                    let active = self.active_screen == screen;
                    let nav_btn = egui::Button::new(
                        RichText::new(label)
                            .strong()
                            .size(12.5)
                            .color(if active {
                                Color32::WHITE
                            } else {
                                Color32::from_rgb(160, 178, 202)
                            }),
                    )
                    .fill(if active {
                        Color32::from_rgb(14, 116, 144)
                    } else {
                        Color32::from_rgb(22, 30, 44)
                    })
                    .stroke(Stroke::new(
                        1.0,
                        if active {
                            Color32::from_rgb(56, 208, 248)
                        } else {
                            Color32::from_rgb(40, 54, 74)
                        },
                    ))
                    .min_size(Vec2::new(0.0, m.authority_btn_height))
                    .corner_radius(CornerRadius::same(5));
                    if ui.add(nav_btn).clicked() {
                        self.active_screen = screen;
                    }
                }

                ui.separator();

                // Voltage readout
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
                    RichText::new(format!("{volts:.1} V"))
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
                        RichText::new("Teach")
                            .strong()
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
                    .min_size(Vec2::new(66.0, m.authority_btn_height))
                    .corner_radius(CornerRadius::same(5));
                    if ui.add(teach_btn).clicked() {
                        self.model.set_authority_mode(ControlMode::Teach);
                    }

                    let drive_btn = egui::Button::new(
                        RichText::new("Drive")
                            .strong()
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
                    .min_size(Vec2::new(66.0, m.authority_btn_height))
                    .corner_radius(CornerRadius::same(5));
                    if ui.add(drive_btn).clicked() {
                        self.model.set_authority_mode(ControlMode::Drive);
                    }
                });

                if !is_portrait && !m.compact_toolbar && !estop {
                    ui.separator();
                    ui.label(
                        RichText::new("Speed")
                            .size(12.0)
                            .color(Color32::from_rgb(156, 172, 194)),
                    );
                    ui.scope(|ui| {
                        ui.spacing_mut().slider_width = 76.0;
                        ui.add(
                            egui::Slider::new(&mut self.model.speed_limit_pct, 10..=100)
                                .suffix("%")
                                .max_decimals(0),
                        );
                    });
                }
            }

            // Right-aligned E-Stop (always pinned when connected or tripped), Reset, and Fullscreen
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if self.connected || estop {
                    let estop_text = if m.compact_toolbar {
                        "E-STOP"
                    } else {
                        "E-STOP [Space]"
                    };
                    let estop_btn = egui::Button::new(
                        RichText::new(estop_text)
                            .strong()
                            .size(13.0)
                            .color(Color32::WHITE),
                    )
                    .fill(Color32::from_rgb(212, 34, 44))
                    .stroke(Stroke::new(1.2, Color32::from_rgb(252, 130, 136)))
                    .min_size(m.estop_size)
                    .corner_radius(CornerRadius::same(5));

                    if ui.add(estop_btn).clicked() {
                        self.mcp.on_estop_disarm();
                        self.model.trigger_estop();
                    }

                    if estop {
                        let reset_btn = egui::Button::new(
                            RichText::new("Reset -> Teach")
                                .strong()
                                .size(12.0)
                                .color(Color32::from_rgb(15, 22, 32)),
                        )
                        .fill(Color32::from_rgb(251, 191, 36))
                        .min_size(Vec2::new(108.0, m.estop_size.y))
                        .corner_radius(CornerRadius::same(5));

                        if ui.add(reset_btn).clicked() {
                            if let Some(f) = self.transport.sim_faults_mut() {
                                f.drop_heartbeat = false;
                            }
                            self.transport.reset_upright();
                            self.model.reset_estop();
                        }
                    }
                }

                let is_fs = self.saver.is_fullscreen(ui.ctx());
                let fs_label = if is_fs { "Exit Full" } else { "Full" };
                let fs_btn = egui::Button::new(
                    RichText::new(fs_label)
                        .size(11.5)
                        .color(if is_fs {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(186, 210, 236)
                        }),
                )
                .fill(if is_fs {
                    Color32::from_rgb(14, 116, 144)
                } else {
                    Color32::from_rgb(22, 30, 44)
                })
                .min_size(Vec2::new(0.0, m.authority_btn_height))
                .corner_radius(CornerRadius::same(5));
                if ui
                    .add(fs_btn)
                    .on_hover_text("Toggle fullscreen workstation window")
                    .clicked()
                {
                    match self.saver.toggle_fullscreen(ui.ctx()) {
                        Ok(now_fs) => {
                            self.model.status_message = if now_fs {
                                "Entered fullscreen workstation mode.".to_string()
                            } else {
                                "Exited fullscreen mode.".to_string()
                            };
                        }
                        Err(err) => {
                            self.model.status_message = err;
                        }
                    }
                }

                let mcp_enabled = self.mcp.policy.enabled;
                let mcp_armed = self.mcp.policy.drive_armed;
                let recent_pulse = (self.sim_time_sec - self.mcp.last_activity_sim_time) < 1.2;
                let (mcp_lbl, mcp_txt, mcp_fill, mcp_stroke) = if !mcp_enabled {
                    (
                        "MCP OFF",
                        Color32::from_rgb(142, 156, 176),
                        Color32::from_rgb(22, 28, 38),
                        Color32::from_rgb(44, 56, 74),
                    )
                } else if mcp_armed {
                    (
                        "MCP ARMED",
                        Color32::from_rgb(254, 243, 199),
                        Color32::from_rgb(120, 74, 14),
                        Color32::from_rgb(251, 191, 36),
                    )
                } else if self.mcp.drawer_open || recent_pulse {
                    (
                        "MCP ON",
                        Color32::WHITE,
                        Color32::from_rgb(14, 116, 144),
                        Color32::from_rgb(103, 232, 249),
                    )
                } else {
                    (
                        "MCP ON",
                        Color32::from_rgb(125, 224, 248),
                        Color32::from_rgb(18, 42, 58),
                        Color32::from_rgb(40, 128, 160),
                    )
                };
                let mcp_btn = egui::Button::new(
                    RichText::new(mcp_lbl).strong().size(11.5).color(mcp_txt),
                )
                .fill(mcp_fill)
                .stroke(Stroke::new(1.0, mcp_stroke))
                .min_size(Vec2::new(0.0, m.authority_btn_height))
                .corner_radius(CornerRadius::same(5));
                if ui
                    .add(mcp_btn)
                    .on_hover_text("Open MCP Agent Access panel, setup snippets & live tool log")
                    .clicked()
                {
                    self.mcp.drawer_open = !self.mcp.drawer_open;
                }
            });
        });
    }

    /// Prominent safety banner for E-Stop or active advisory faults above the 3D canvas.
    pub(crate) fn render_safety_banners(&mut self, ui: &mut egui::Ui) {
        let estop = self.model.is_estop_active();
        let faults = self.model.latest_telemetry.faults;

        if estop {
            egui::Frame::new()
                .fill(Color32::from_rgb(138, 22, 30))
                .inner_margin(Margin::symmetric(12, 6))
                .stroke(Stroke::new(1.0, Color32::from_rgb(248, 113, 113)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        let reason = if faults.contains(FaultFlags::TILT_FALL_CUTOFF) {
                            "FALL CUTOFF LATCHED — TORQUE OFF"
                        } else {
                            "E-STOP LATCHED — TORQUE OFF"
                        };
                        ui.label(
                            RichText::new(format!("{reason}  |  Faults: {:?}", faults))
                                .strong()
                                .size(12.5)
                                .color(Color32::WHITE),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.button("Reset Upright -> Teach").clicked() {
                                if let Some(f) = self.transport.sim_faults_mut() {
                                    f.drop_heartbeat = false;
                                }
                                self.transport.reset_upright();
                                self.model.reset_estop();
                            }
                        });
                    });
                });
        } else if !faults.is_empty() {
            egui::Frame::new()
                .fill(Color32::from_rgb(92, 54, 14))
                .inner_margin(Margin::symmetric(12, 5))
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
    }
}
