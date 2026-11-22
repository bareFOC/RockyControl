//! Panel 2 — Teach & Action Studio (keyframe capture, combat move library, and touch combat pad).

use egui::{Color32, RichText, Vec2};
use humanoid_model::{ActionMove, Keyframe};
use humanoid_proto::ControlMode;

use crate::RockyStudioApp;

impl RockyStudioApp {
    pub(crate) fn render_action_studio_panel(&mut self, ui: &mut egui::Ui) {
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
        if self.ui_metrics.touch_mode {
            ui.label(
                RichText::new("Quick Combat Pad (Touch)")
                    .strong()
                    .size(13.0)
                    .color(Color32::from_rgb(251, 191, 36)),
            );
            ui.add_space(4.0);
            let pad_w = ((ui.available_width() - 12.0) * 0.5).max(110.0);
            egui::Grid::new("tablet_combat_pad")
                .num_columns(2)
                .spacing([8.0, 8.0])
                .show(ui, |ui| {
                    for (slot, (idx, label)) in [
                        (0, "Guard Stance"),
                        (1, "Left Jab"),
                        (2, "Right Cross"),
                        (3, "Front Kick"),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        let btn = egui::Button::new(RichText::new(label).strong().size(13.5))
                            .min_size(Vec2::new(pad_w, 44.0));
                        if ui.add_enabled(is_drive, btn).clicked() {
                            self.model.play_move(idx);
                        }
                        if slot % 2 == 1 {
                            ui.end_row();
                        }
                    }
                });
            if self.model.playback.is_some() {
                ui.add_space(6.0);
                let stop_btn = egui::Button::new(
                    RichText::new("Stop Active Move")
                        .strong()
                        .color(Color32::WHITE),
                )
                .fill(Color32::from_rgb(185, 28, 28))
                .min_size(Vec2::new(ui.available_width(), 40.0));
                if ui.add(stop_btn).clicked() {
                    self.model.stop_playback();
                }
            }
            ui.add_space(8.0);
            ui.separator();
        }

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
}
