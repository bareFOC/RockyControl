//! 3D Mirror Viewport panel with E-Stop banner, camera presets, and embedded `three-d` canvas.

use egui::{Color32, RichText, Stroke, Vec2};
use humanoid_proto::ControlMode;
use humanoid_viewport::CameraPreset;

use crate::RockyStudioApp;

impl RockyStudioApp {
    pub(crate) fn render_viewport_panel(&mut self, ui: &mut egui::Ui) {
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

        // Compact 3D Viewport control ribbon (adapts hit targets & adds +/- touch zoom for Android Tablets)
        let touch = self.ui_metrics.touch_mode;
        let ribbon_btn_h = if touch { 34.0 } else { 22.0 };
        let ribbon_text = if touch { 12.5 } else { 11.5 };

        egui::Frame::new()
            .fill(Color32::from_rgb(16, 21, 30))
            .inner_margin(egui::Margin::symmetric(10, if touch { 6 } else { 5 }))
            .stroke(Stroke::new(1.0, Color32::from_rgb(30, 40, 56)))
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    let upload_btn = egui::Button::new(
                        RichText::new("Upload URDF")
                            .strong()
                            .size(ribbon_text)
                            .color(Color32::WHITE),
                    )
                    .fill(Color32::from_rgb(14, 116, 144))
                    .min_size(Vec2::new(0.0, ribbon_btn_h));
                    if ui.add(upload_btn).clicked() {
                        if let Err(err) = self.saver.request_open_urdf_file() {
                            self.model.status_message = err;
                        }
                    }

                    let paste_btn = egui::Button::new(RichText::new("Paste XML").size(ribbon_text))
                        .min_size(Vec2::new(0.0, ribbon_btn_h));
                    if ui.add(paste_btn).clicked() {
                        self.paste_urdf_buf = self.model.bundle.urdf_xml.clone();
                        self.show_paste_urdf_dialog = true;
                    }

                    let pull_btn = egui::Button::new(RichText::new("Pull MCU").size(ribbon_text))
                        .min_size(Vec2::new(0.0, ribbon_btn_h));
                    if ui.add_enabled(!self.mcu_sync.pulling, pull_btn).clicked() {
                        self.start_pull_mcu_model();
                    }

                    ui.separator();

                    for (label, preset) in [
                        ("Iso", CameraPreset::Iso),
                        ("Front", CameraPreset::Front),
                        ("Side", CameraPreset::Side),
                        ("Top", CameraPreset::Top),
                    ] {
                        let btn = egui::Button::new(RichText::new(label).size(ribbon_text))
                            .min_size(Vec2::new(0.0, ribbon_btn_h));
                        if ui.add(btn).clicked() {
                            self.viewport.apply_preset_for_rig(preset, &self.model.rig);
                        }
                    }

                    ui.separator();
                    let up_btn = egui::Button::new(
                        RichText::new(self.model.rig.up_axis.label()).size(ribbon_text),
                    )
                    .min_size(Vec2::new(0.0, ribbon_btn_h));
                    if ui.add(up_btn).clicked() {
                        self.toggle_urdf_up_axis();
                    }
                    ui.checkbox(&mut self.viewport.show_grid, "Grid");
                    ui.checkbox(&mut self.viewport.show_joint_markers, "Joints");

                    let avail_w = ui.available_width();
                    if avail_w > 195.0 {
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let show_ghost =
                                self.model.latest_telemetry.active_mode == ControlMode::Drive;
                            let legend = if touch {
                                if show_ghost {
                                    "Drag Orbit · Tap Joint · Ghost=Target"
                                } else {
                                    "Drag Orbit · Tap Joint · Auto-Fit"
                                }
                            } else if show_ghost {
                                "Solid=Measured · Cyan=Target"
                            } else {
                                "Drop .urdf / .zip on canvas"
                            };
                            ui.label(
                                RichText::new(legend)
                                    .size(11.0)
                                    .color(Color32::from_rgb(138, 154, 178)),
                            );
                        });
                    }
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
}
