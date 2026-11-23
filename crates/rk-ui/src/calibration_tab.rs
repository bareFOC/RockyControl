//! Panel 1 — Joint Calibration, Soft Limits, Precision Nudge, and Limb Jog Sliders.

use egui::{Color32, CornerRadius, RichText, Stroke, Vec2};
use rk_model::LimbGroup;
use rk_proto::{ControlMode, HEAT_TINT_THRESHOLD_MA};

use crate::RockyStudioApp;

impl RockyStudioApp {
    pub(crate) fn render_calibration_panel(&mut self, ui: &mut egui::Ui) {
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
                    .push(rk_proto::ConfigCmd::CommitFlash);
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
            let raw_rad = rk_control::calibrated_to_raw(measured_rad, &cal);
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
                    ui.horizontal_wrapped(|ui| {
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

                    // Precision Step Jog bar (unified touch-first sizing)
                    ui.add_space(6.0);
                    ui.add_enabled_ui(is_drive, |ui| {
                        let step_h = 30.0;
                        ui.horizontal_wrapped(|ui| {
                            ui.label(
                                RichText::new("Nudge:")
                                    .size(11.5)
                                    .color(Color32::from_rgb(148, 163, 184)),
                            );
                            for (label, delta_deg) in [
                                ("-5°", Some(-5.0_f32)),
                                ("-1°", Some(-1.0_f32)),
                                ("0°", None),
                                ("+1°", Some(1.0_f32)),
                                ("+5°", Some(5.0_f32)),
                            ] {
                                let btn = egui::Button::new(RichText::new(label).monospace())
                                    .min_size(Vec2::new(42.0, step_h));
                                if ui.add(btn).clicked() {
                                    self.model.playback = None;
                                    let next = match delta_deg {
                                        Some(d) => self.model.target_rad[sel] + d.to_radians(),
                                        None => 0.0,
                                    };
                                    self.model.target_rad[sel] =
                                        next.clamp(cal.min_rad, cal.max_rad);
                                }
                            }
                        });
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
                            .selectable_label(selected, RichText::new(label).size(12.0))
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
                        if ui
                            .add(
                                egui::Slider::new(&mut deg, lo..=hi)
                                    .suffix("°")
                                    .max_decimals(1),
                            )
                            .changed()
                        {
                            self.apply_direct_joint_angle(idx, deg.to_radians());
                        }
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
    }
}
