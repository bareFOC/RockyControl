//! Panel 4 — Simulation Fault Injection & Virtual Physical Robot Hand-Posing Drawer.

use egui::{Color32, CornerRadius, RichText, Stroke};
use rk_proto::ControlMode;

use crate::RockyStudioApp;

impl RockyStudioApp {
    pub(crate) fn render_sim_fault_panel(&mut self, ui: &mut egui::Ui) {
        let sel = self
            .model
            .selected_joint
            .min(self.model.rig.joints.len().saturating_sub(1));
        let sel_label = self
            .model
            .rig
            .joints
            .get(sel)
            .map(|j| j.label.clone())
            .unwrap_or_else(|| "None".to_string());

        // Fault Injection Suite
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
                            ui.checkbox(
                                &mut faults.imu_disconnected,
                                "Disable Virtual IMU (Always Upright / Top-Up)",
                            );
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
                                if let Some(cal) = self.model.calibration.get(sel) {
                                    if self.model.requested_mode == ControlMode::Drive
                                        && !self.model.is_estop_active()
                                    {
                                        self.model.playback = None;
                                        self.model.target_rad[sel] =
                                            (self.model.target_rad[sel] + std::f32::consts::FRAC_PI_2)
                                                .clamp(cal.min_rad, cal.max_rad);
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
                            .unwrap_or([0.0; rk_proto::MAX_JOINTS]);

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
    }
}
