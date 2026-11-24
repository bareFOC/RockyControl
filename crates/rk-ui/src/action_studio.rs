//! Keyframe Action Studio — Combat & Autonomous Fall-Recovery Motion Library.
//!
//! Shared between the Control Screen's on-demand Action Studio panel and the
//! Config & Calibration drawer's `Actions` tab.

use egui::{Color32, CornerRadius, Margin, RichText, Stroke, Vec2};
use rk_proto::ControlMode;

use crate::RockyStudioApp;

/// Category filter for the Keyframe Action Studio move library.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ActionCategoryFilter {
    #[default]
    All,
    Combat,
    Recovery,
}

impl ActionCategoryFilter {
    pub const fn label(self) -> &'static str {
        match self {
            Self::All => "All",
            Self::Combat => "Combat",
            Self::Recovery => "Recovery",
        }
    }

    pub fn matches(self, category: &str) -> bool {
        match self {
            Self::All => true,
            Self::Recovery => category.eq_ignore_ascii_case("Recovery"),
            Self::Combat => !category.eq_ignore_ascii_case("Recovery"),
        }
    }
}

impl RockyStudioApp {
    /// Renders the Keyframe Action Studio: move library (Combat + Recovery get-up sequences),
    /// per-keyframe timeline, live playback bar, pose preview, and pose capture.
    pub(crate) fn render_action_studio_panel(&mut self, ui: &mut egui::Ui, compact: bool) {
        let estop = self.model.is_estop_active();

        // Header + category filter bar
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("KEYFRAME ACTION STUDIO")
                    .strong()
                    .size(if compact { 11.0 } else { 12.5 })
                    .color(Color32::from_rgb(103, 232, 249)),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for filter in [
                    ActionCategoryFilter::Recovery,
                    ActionCategoryFilter::Combat,
                    ActionCategoryFilter::All,
                ] {
                    let active = self.action_category_filter == filter;
                    let fill = if active {
                        if filter == ActionCategoryFilter::Recovery {
                            Color32::from_rgb(180, 83, 9)
                        } else {
                            Color32::from_rgb(14, 116, 144)
                        }
                    } else {
                        Color32::from_rgb(22, 30, 44)
                    };
                    let btn = egui::Button::new(
                        RichText::new(filter.label())
                            .strong()
                            .size(10.5)
                            .color(if active {
                                Color32::WHITE
                            } else {
                                Color32::from_rgb(156, 174, 198)
                            }),
                    )
                    .fill(fill)
                    .min_size(Vec2::new(0.0, 22.0))
                    .corner_radius(CornerRadius::same(4));
                    if ui.add(btn).clicked() {
                        self.action_category_filter = filter;
                    }
                }
            });
        });

        ui.add_space(4.0);

        // Move selector grid (Combat + Recovery libraries)
        let filtered_indices: Vec<usize> = self
            .model
            .moves
            .iter()
            .enumerate()
            .filter(|(_, mv)| self.action_category_filter.matches(&mv.category))
            .map(|(i, _)| i)
            .collect();

        if !filtered_indices.is_empty() && !filtered_indices.contains(&self.model.selected_move) {
            self.model.selected_move = filtered_indices[0];
        }

        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(5.0, 5.0);
            for idx in &filtered_indices {
                let mv = &self.model.moves[*idx];
                let is_sel = self.model.selected_move == *idx;
                let is_playing = self
                    .model
                    .playback
                    .as_ref()
                    .is_some_and(|pb| pb.move_idx == *idx);
                let is_recovery = mv.category.eq_ignore_ascii_case("Recovery");

                let fill = if is_playing {
                    Color32::from_rgb(22, 134, 96)
                } else if is_sel {
                    if is_recovery {
                        Color32::from_rgb(146, 64, 14)
                    } else {
                        Color32::from_rgb(14, 116, 144)
                    }
                } else if is_recovery {
                    Color32::from_rgb(38, 28, 24)
                } else {
                    Color32::from_rgb(24, 33, 48)
                };

                let stroke = if is_sel || is_playing {
                    Stroke::new(
                        1.2,
                        if is_recovery {
                            Color32::from_rgb(251, 191, 36)
                        } else {
                            Color32::from_rgb(103, 232, 249)
                        },
                    )
                } else {
                    Stroke::new(1.0, Color32::from_rgb(44, 58, 80))
                };

                let btn = egui::Button::new(
                    RichText::new(&mv.name)
                        .strong()
                        .size(11.0)
                        .color(if is_sel || is_playing {
                            Color32::WHITE
                        } else if is_recovery {
                            Color32::from_rgb(253, 224, 171)
                        } else {
                            Color32::from_rgb(206, 222, 240)
                        }),
                )
                .fill(fill)
                .stroke(stroke)
                .min_size(Vec2::new(0.0, 26.0))
                .corner_radius(CornerRadius::same(5));

                if ui.add(btn).clicked() {
                    self.model.selected_move = *idx;
                }
            }
        });

        ui.add_space(6.0);

        // Selected move detail & keyframe timeline card
        let sel_idx = self
            .model
            .selected_move
            .min(self.model.moves.len().saturating_sub(1));
        let Some(mv_snapshot) = self.model.moves.get(sel_idx).cloned() else {
            return;
        };
        let is_recovery = mv_snapshot.category.eq_ignore_ascii_case("Recovery");
        let total_ms: u32 = mv_snapshot.keyframes.iter().map(|k| k.duration_ms).sum();
        let active_pb = self
            .model
            .playback
            .as_ref()
            .filter(|pb| pb.move_idx == sel_idx)
            .cloned();

        egui::Frame::new()
            .fill(Color32::from_rgb(17, 23, 34))
            .corner_radius(CornerRadius::same(6))
            .inner_margin(Margin::same(8))
            .stroke(Stroke::new(
                1.0,
                if is_recovery {
                    Color32::from_rgb(120, 74, 24)
                } else {
                    Color32::from_rgb(40, 56, 78)
                },
            ))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(&mv_snapshot.name)
                            .strong()
                            .size(12.5)
                            .color(Color32::from_rgb(236, 245, 255)),
                    );
                    ui.label(
                        RichText::new(format!(
                            "{} · {} kf · {:.2}s",
                            mv_snapshot.category,
                            mv_snapshot.keyframes.len(),
                            total_ms as f32 / 1000.0
                        ))
                        .monospace()
                        .size(10.5)
                        .color(if is_recovery {
                            Color32::from_rgb(251, 191, 36)
                        } else {
                            Color32::from_rgb(138, 162, 192)
                        }),
                    );
                });

                if let Some(pb) = &active_pb {
                    ui.add_space(4.0);
                    let kf_name = mv_snapshot
                        .keyframes
                        .get(pb.active_keyframe)
                        .map(|k| k.label.as_str())
                        .unwrap_or("Running");
                    ui.add(
                        egui::ProgressBar::new(pb.progress.clamp(0.0, 1.0))
                            .desired_height(14.0)
                            .text(format!(
                                "Step {}/{}: {} ({:.0}%)",
                                pb.active_keyframe + 1,
                                mv_snapshot.keyframes.len().max(1),
                                kf_name,
                                pb.progress * 100.0
                            )),
                    );
                }

                ui.add_space(5.0);

                // Transport buttons: Play / Stop + Capture Current Pose
                ui.horizontal(|ui| {
                    let playing_any = self.model.playback.is_some();
                    if playing_any {
                        let stop_btn = egui::Button::new(
                            RichText::new("Stop")
                                .strong()
                                .size(11.5)
                                .color(Color32::WHITE),
                        )
                        .fill(Color32::from_rgb(185, 28, 28))
                        .min_size(Vec2::new(72.0, 26.0))
                        .corner_radius(CornerRadius::same(5));
                        if ui.add(stop_btn).clicked() {
                            self.model.stop_playback();
                        }
                    } else {
                        let play_lbl = if estop {
                            "Play (E-Stop)"
                        } else {
                            "Play Sequence"
                        };
                        let play_btn = egui::Button::new(
                            RichText::new(play_lbl)
                                .strong()
                                .size(11.5)
                                .color(if estop {
                                    Color32::from_rgb(156, 170, 190)
                                } else {
                                    Color32::WHITE
                                }),
                        )
                        .fill(if estop {
                            Color32::from_rgb(36, 42, 54)
                        } else if is_recovery {
                            Color32::from_rgb(180, 83, 9)
                        } else {
                            Color32::from_rgb(14, 116, 144)
                        })
                        .min_size(Vec2::new(108.0, 26.0))
                        .corner_radius(CornerRadius::same(5));
                        if ui.add_enabled(!estop, play_btn).clicked() {
                            if self.model.requested_mode == ControlMode::Teach {
                                self.model.set_authority_mode(ControlMode::Drive);
                            }
                            self.model.walk_cmd.walk_enabled = false;
                            self.model.play_move(sel_idx);
                        }
                    }

                    let cap_btn = egui::Button::new(RichText::new("+ Capture Pose").size(11.0))
                        .fill(Color32::from_rgb(26, 36, 52))
                        .min_size(Vec2::new(96.0, 26.0))
                        .corner_radius(CornerRadius::same(5));
                    if ui
                        .add(cap_btn)
                        .on_hover_text("Append live robot joint angles as a new keyframe")
                        .clicked()
                    {
                        self.model.capture_keyframe();
                    }
                });

                ui.add_space(5.0);
                ui.separator();

                // Keyframe timeline list with one-click pose preview
                let max_h = if compact { 122.0 } else { 210.0 };
                egui::ScrollArea::vertical()
                    .id_salt(if compact {
                        "action_studio_kf_compact"
                    } else {
                        "action_studio_kf_drawer"
                    })
                    .max_height(max_h)
                    .show(ui, |ui| {
                        for (kf_idx, kf) in mv_snapshot.keyframes.iter().enumerate() {
                            let is_active_kf = active_pb
                                .as_ref()
                                .is_some_and(|pb| pb.active_keyframe == kf_idx);
                            ui.horizontal(|ui| {
                                let row_col = if is_active_kf {
                                    Color32::from_rgb(74, 222, 128)
                                } else {
                                    Color32::from_rgb(206, 222, 240)
                                };
                                ui.label(
                                    RichText::new(format!(
                                        "{}. {} ({} ms)",
                                        kf_idx + 1,
                                        kf.label,
                                        kf.duration_ms
                                    ))
                                    .monospace()
                                    .size(11.0)
                                    .color(row_col),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        let preview_btn = egui::Button::new(
                                            RichText::new("Preview").size(10.0),
                                        )
                                        .min_size(Vec2::new(54.0, 20.0))
                                        .corner_radius(CornerRadius::same(4));
                                        if ui
                                            .add_enabled(!estop, preview_btn)
                                            .on_hover_text(
                                                "Apply this keyframe's joint angles to Rocky",
                                            )
                                            .clicked()
                                        {
                                            self.apply_keyframe_preview(&kf.angles_rad, &kf.label);
                                        }
                                    },
                                );
                            });
                        }
                    });
            });
    }

    fn apply_keyframe_preview(&mut self, angles_rad: &[f32; rk_proto::MAX_JOINTS], label: &str) {
        if self.model.is_estop_active() {
            return;
        }
        self.model.playback = None;
        self.model.walk_cmd.walk_enabled = false;
        let dof = self.model.rig.joints.len().min(rk_proto::MAX_JOINTS);
        for idx in 0..dof {
            let cal = self.model.calibration[idx];
            let clamped = angles_rad[idx].clamp(cal.min_rad, cal.max_rad);
            self.model.target_rad[idx] = clamped;
            if self.model.requested_mode == ControlMode::Teach {
                let raw = rk_control::calibrated_to_raw(clamped, &cal);
                self.transport.set_virtual_hand_angle(idx, raw);
            }
        }
        self.model.status_message = format!("Previewing keyframe pose '{label}'.");
    }
}
