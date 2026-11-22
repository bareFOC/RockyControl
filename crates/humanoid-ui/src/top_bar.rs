//! Persistent top safety toolbar, transport selector, and Android Tablet adaptive workspace layout.

use egui::{Color32, CornerRadius, RichText, Stroke, Vec2};
use humanoid_link::{RobotTransport, SimTransport, TransportKind};
use humanoid_proto::{ControlMode, FaultFlags};

use crate::responsive::UiProfileMode;
use crate::{DockPanel, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_transport_selector(&mut self, ui: &mut egui::Ui, width: f32) {
        let prev_transport = self.selected_transport_ui;
        egui::ComboBox::from_id_salt("transport_select")
            .selected_text(self.selected_transport_ui.label())
            .width(width)
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
                ui.add_enabled(false, egui::Button::new("USB Net UDP (Desktop Native)"));
                ui.add_enabled(false, egui::Button::new("USB CDC-ACM Serial (RP2350)"));
            });
        if self.selected_transport_ui != prev_transport {
            // Safety rule: switching links always forces Teach (Torque OFF)
            self.model.set_authority_mode(ControlMode::Teach);
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
                    self.query_mcu_model_info();
                    self.model.status_message =
                        "Switched to Software Sim Kernel in Teach mode.".to_string();
                }
                #[cfg(not(target_arch = "wasm32"))]
                TransportKind::UsbNetworkingUdp => {
                    match humanoid_link::UdpTransport::connect("10.42.0.2:7777") {
                        Ok(udp) => {
                            self.transport = Box::new(udp);
                            self.query_mcu_model_info();
                            self.model.status_message =
                                "Connected USB-NCM UDP link -> 10.42.0.2:7777 (Teach mode)."
                                    .to_string();
                        }
                        Err(err) => {
                            self.selected_transport_ui = TransportKind::SimKernel;
                            self.model.status_message = format!("USB-NCM UDP bind failed: {err}");
                        }
                    }
                }
                _ => {
                    self.selected_transport_ui = TransportKind::SimKernel;
                }
            }
        }
    }

    pub(crate) fn render_top_toolbar(&mut self, ui: &mut egui::Ui) {
        let stats = self.transport.stats();
        let estop = self.model.is_estop_active();
        let volts = (self.model.latest_telemetry.bus_voltage_mv as f32) / 1000.0;
        let m = self.ui_metrics;
        let is_portrait = m.form_factor.is_portrait();

        ui.horizontal_centered(|ui| {
            ui.spacing_mut().item_spacing.x = if m.touch_mode { 8.0 } else { 10.0 };

            ui.label(
                RichText::new(if m.compact_toolbar { "ROCKY" } else { "ROCKY STUDIO" })
                    .strong()
                    .size(if m.touch_mode { 15.5 } else { 16.0 })
                    .color(Color32::from_rgb(236, 244, 255)),
            );

            ui.separator();

            if !is_portrait {
                self.render_transport_selector(ui, if m.touch_mode { 132.0 } else { 168.0 });
                ui.separator();
            }

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

            if m.show_full_telemetry_inline {
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
            }
            ui.label(
                RichText::new(format!("{:.1} V", volts))
                    .monospace()
                    .strong()
                    .size(if m.touch_mode { 13.0 } else { 12.5 })
                    .color(volt_color),
            );

            ui.separator();

            // Segmented Authority Toggle: Teach | Drive (greyed out when E-Stop active)
            ui.add_enabled_ui(!estop, |ui| {
                let is_teach = self.model.requested_mode == ControlMode::Teach;
                let is_drive = self.model.requested_mode == ControlMode::Drive;

                let teach_label = if m.compact_toolbar {
                    "Teach"
                } else {
                    "Teach (Robot -> GUI)"
                };
                let drive_label = if m.compact_toolbar {
                    "Drive"
                } else {
                    "Drive (GUI -> Robot)"
                };

                let teach_btn = egui::Button::new(
                    RichText::new(teach_label)
                        .strong()
                        .size(if m.touch_mode { 13.5 } else { 12.5 })
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
                .min_size(Vec2::new(
                    if m.touch_mode { 76.0 } else { 0.0 },
                    m.authority_btn_height,
                ))
                .corner_radius(CornerRadius::same(4));

                if ui.add(teach_btn).clicked() {
                    self.model.set_authority_mode(ControlMode::Teach);
                }

                let drive_btn = egui::Button::new(
                    RichText::new(drive_label)
                        .strong()
                        .size(if m.touch_mode { 13.5 } else { 12.5 })
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
                .min_size(Vec2::new(
                    if m.touch_mode { 76.0 } else { 0.0 },
                    m.authority_btn_height,
                ))
                .corner_radius(CornerRadius::same(4));

                if ui.add(drive_btn).clicked() {
                    self.model.set_authority_mode(ControlMode::Drive);
                }
            });

            if !is_portrait {
                ui.separator();
                ui.label(
                    RichText::new("Speed")
                        .size(12.0)
                        .color(Color32::from_rgb(156, 172, 194)),
                );
                ui.scope(|ui| {
                    ui.spacing_mut().slider_width = if m.touch_mode { 68.0 } else { 108.0 };
                    ui.add(
                        egui::Slider::new(&mut self.model.speed_limit_pct, 10..=100)
                            .suffix("%")
                            .max_decimals(0),
                    );
                });
            }

            // Right-aligned E-Stop (always pinned), Reset E-Stop, Full-3D toggle (Tablet), and UI Profile toggle
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let estop_text = if m.compact_toolbar {
                    "E-STOP"
                } else {
                    "E-STOP [Space]"
                };
                let estop_btn = egui::Button::new(
                    RichText::new(estop_text)
                        .strong()
                        .size(if m.touch_mode { 14.0 } else { 13.0 })
                        .color(Color32::WHITE),
                )
                .fill(Color32::from_rgb(212, 34, 44))
                .stroke(Stroke::new(1.2, Color32::from_rgb(252, 130, 136)))
                .min_size(m.estop_size)
                .corner_radius(CornerRadius::same(4));

                if ui.add(estop_btn).clicked() {
                    self.model.trigger_estop();
                }

                if estop {
                    let reset_text = if m.compact_toolbar {
                        "Reset -> Teach"
                    } else {
                        "Reset E-Stop -> Teach"
                    };
                    let reset_btn = egui::Button::new(
                        RichText::new(reset_text)
                            .strong()
                            .size(12.5)
                            .color(Color32::from_rgb(15, 22, 32)),
                    )
                    .fill(Color32::from_rgb(251, 191, 36))
                    .min_size(Vec2::new(
                        if m.compact_toolbar { 112.0 } else { 152.0 },
                        m.estop_size.y,
                    ))
                    .corner_radius(CornerRadius::same(4));

                    if ui.add(reset_btn).clicked() {
                        if let Some(f) = self.transport.sim_faults_mut() {
                            f.drop_heartbeat = false;
                        }
                        self.model.reset_estop();
                    }
                }

                let profile_label = match self.ui_profile_mode {
                    UiProfileMode::Auto => {
                        if m.touch_mode {
                            "Auto · Touch"
                        } else {
                            "Auto · Desk"
                        }
                    }
                    UiProfileMode::Desktop => "Desktop",
                    UiProfileMode::Tablet => "Touch",
                };
                let profile_btn = egui::Button::new(
                    RichText::new(profile_label)
                        .size(11.5)
                        .color(Color32::from_rgb(186, 210, 236)),
                )
                .fill(Color32::from_rgb(22, 30, 44))
                .min_size(Vec2::new(0.0, m.authority_btn_height.min(34.0)))
                .corner_radius(CornerRadius::same(4));
                if ui
                    .add(profile_btn)
                    .on_hover_text("Cycle ergonomics profile: Auto -> Tablet Touch -> Desktop")
                    .clicked()
                {
                    self.ui_profile_mode = self.ui_profile_mode.next();
                    self.model.status_message =
                        format!("UI ergonomics profile: {}", self.ui_profile_mode.label());
                }

                if m.touch_mode {
                    let full_btn = egui::Button::new(
                        RichText::new(if self.tablet_full_3d {
                            "Show Inspector"
                        } else {
                            "Full 3D"
                        })
                        .strong()
                        .size(12.0)
                        .color(if self.tablet_full_3d {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(186, 210, 236)
                        }),
                    )
                    .fill(if self.tablet_full_3d {
                        Color32::from_rgb(14, 116, 144)
                    } else {
                        Color32::from_rgb(24, 32, 46)
                    })
                    .min_size(Vec2::new(0.0, m.authority_btn_height.min(34.0)))
                    .corner_radius(CornerRadius::same(4));
                    if ui.add(full_btn).clicked() {
                        self.tablet_full_3d = !self.tablet_full_3d;
                    }
                }
            });
        });
    }

    /// Touch-optimized workspace for Android Tablets (Landscape horizontal split or Portrait vertical stack,
    /// with thumb-friendly tab bar and instant Full-3D expansion).
    pub(crate) fn render_tablet_workspace(&mut self, ui: &mut egui::Ui) {
        if self.tablet_full_3d {
            self.render_viewport_panel(ui);
            return;
        }

        let avail = ui.available_rect_before_wrap();
        let is_portrait = self.ui_metrics.form_factor.is_portrait();
        let split_ratio = self.ui_metrics.split_ratio_3d;
        let divider_thickness = 2.0;

        let (vp_rect, div_rect, insp_rect) = if is_portrait {
            let vp_h = (avail.height() * split_ratio).max(240.0);
            let vp_rect = egui::Rect::from_min_size(avail.min, Vec2::new(avail.width(), vp_h));
            let div_rect = egui::Rect::from_min_size(
                egui::pos2(avail.min.x, vp_rect.max.y),
                Vec2::new(avail.width(), divider_thickness),
            );
            let insp_rect =
                egui::Rect::from_min_max(egui::pos2(avail.min.x, div_rect.max.y), avail.max);
            (vp_rect, div_rect, insp_rect)
        } else {
            let vp_w = (avail.width() * split_ratio).max(320.0);
            let vp_rect = egui::Rect::from_min_size(avail.min, Vec2::new(vp_w, avail.height()));
            let div_rect = egui::Rect::from_min_size(
                egui::pos2(vp_rect.max.x, avail.min.y),
                Vec2::new(divider_thickness, avail.height()),
            );
            let insp_rect =
                egui::Rect::from_min_max(egui::pos2(div_rect.max.x, avail.min.y), avail.max);
            (vp_rect, div_rect, insp_rect)
        };

        ui.painter()
            .rect_filled(div_rect, 0.0, Color32::from_rgb(36, 48, 66));

        let mut vp_ui = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(vp_rect)
                .layout(egui::Layout::top_down(egui::Align::LEFT)),
        );
        vp_ui.set_clip_rect(vp_rect);
        self.render_viewport_panel(&mut vp_ui);

        let mut insp_ui = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(insp_rect)
                .layout(egui::Layout::top_down(egui::Align::LEFT)),
        );
        insp_ui.set_clip_rect(insp_rect);
        self.render_tablet_inspector(&mut insp_ui);
    }

    fn render_tablet_inspector(&mut self, ui: &mut egui::Ui) {
        let is_portrait = self.ui_metrics.form_factor.is_portrait();
        let tab_h = self.ui_metrics.touch_tab_bar_height;

        if is_portrait {
            egui::Frame::new()
                .fill(Color32::from_rgb(15, 20, 28))
                .inner_margin(egui::Margin::symmetric(10, 5))
                .stroke(Stroke::new(1.0, Color32::from_rgb(28, 38, 54)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        self.render_transport_selector(ui, 155.0);
                        ui.separator();
                        ui.label(
                            RichText::new("Speed")
                                .size(12.5)
                                .color(Color32::from_rgb(156, 172, 194)),
                        );
                        ui.add(
                            egui::Slider::new(&mut self.model.speed_limit_pct, 10..=100)
                                .suffix("%")
                                .max_decimals(0),
                        );
                    });
                });
        }

        // Thumb-friendly 5-tab bar (equal-width touch buttons)
        egui::Frame::new()
            .fill(Color32::from_rgb(14, 19, 27))
            .inner_margin(egui::Margin::symmetric(6, 5))
            .stroke(Stroke::new(1.0, Color32::from_rgb(32, 43, 60)))
            .show(ui, |ui| {
                let count = DockPanel::INSPECTOR_PANELS.len() as f32;
                let spacing = 5.0;
                let total_w = (ui.available_width() - spacing * (count - 1.0)).max(200.0);
                let btn_w = (total_w / count).floor();
                let btn_h = (tab_h - 10.0).max(34.0);

                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = spacing;
                    for panel in DockPanel::INSPECTOR_PANELS {
                        let active = self.active_tablet_panel == panel;
                        let btn = egui::Button::new(
                            RichText::new(panel.short_label())
                                .strong()
                                .size(12.5)
                                .color(if active {
                                    Color32::WHITE
                                } else {
                                    Color32::from_rgb(156, 172, 194)
                                }),
                        )
                        .fill(if active {
                            Color32::from_rgb(14, 116, 144)
                        } else {
                            Color32::from_rgb(21, 28, 40)
                        })
                        .stroke(Stroke::new(
                            1.0,
                            if active {
                                Color32::from_rgb(56, 189, 248)
                            } else {
                                Color32::from_rgb(36, 48, 66)
                            },
                        ))
                        .min_size(Vec2::new(btn_w, btn_h))
                        .corner_radius(CornerRadius::same(5));

                        if ui.add(btn).clicked() {
                            self.active_tablet_panel = panel;
                        }
                    }
                });
            });

        let margin = self.ui_metrics.panel_margin;
        egui::Frame::new()
            .inner_margin(egui::Margin::same(margin))
            .show(ui, |ui| match self.active_tablet_panel {
                DockPanel::UrdfModel => self.render_urdf_mcu_panel(ui),
                DockPanel::CalibrationJog => self.render_calibration_panel(ui),
                DockPanel::ActionStudio => self.render_action_studio_panel(ui),
                DockPanel::PilotHealth => self.render_pilot_health_panel(ui),
                DockPanel::SimFaultHand => self.render_sim_fault_panel(ui),
                DockPanel::Viewport3D => self.render_urdf_mcu_panel(ui),
            });
    }
}
