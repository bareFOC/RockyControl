//! Panel 3 — Manual Pilot (`gilrs` gamepad), live `egui_plot` telemetry charts, and deterministic Copper replay.

use egui::{Color32, CornerRadius, RichText, Stroke};
use egui_plot::{HLine, Legend, Line, Plot, PlotPoints};
use humanoid_proto::{ControlMode, HEAT_TINT_THRESHOLD_MA};
use humanoid_sim::verify_deterministic_replay;

use crate::RockyStudioApp;

impl RockyStudioApp {
    pub(crate) fn render_pilot_health_panel(&mut self, ui: &mut egui::Ui) {
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
                let joint_label = self
                    .model
                    .rig
                    .joints
                    .get(sel)
                    .map(|j| j.label.clone())
                    .unwrap_or_else(|| "Static Model (0 DoF)".to_string());

                ui.label(
                    RichText::new(format!("Live Telemetry — #{:02} {}", sel, joint_label))
                        .strong()
                        .size(13.0)
                        .color(Color32::from_rgb(226, 232, 240)),
                );

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
}
