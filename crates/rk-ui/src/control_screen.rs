//! Screen 2 — Control Screen (Full-bleed 3D robot, Walk Pad + WASD/Gamepad,
//! MCU Upper-Body Vertical Balance HUD, Combat Push Tests, and Quick Combat Moves).

use egui::{Color32, CornerRadius, Margin, Pos2, Rect, RichText, Sense, Stroke, UiBuilder, Vec2, pos2, vec2};
use rk_proto::{ControlMode, FaultFlags};
use rk_viewport::ViewportOptions;

use crate::{PushStrength, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_control_screen(&mut self, ui: &mut egui::Ui) {
        self.render_safety_banners(ui);

        let pointer_pos = ui
            .ctx()
            .input(|i| i.pointer.interact_pos().or(i.pointer.hover_pos()));
        let hud_blocked = self.show_paste_urdf_dialog
            || self.show_import_dialog
            || pointer_pos.is_some_and(|p| self.viewport_hud_rects.iter().any(|r| r.contains(p)));

        let telem = self.model.latest_telemetry;
        let estop = self.model.is_estop_active();
        let show_ghost = telem.active_mode == ControlMode::Drive;

        let interaction = self.viewport.show(
            ui,
            self.last_dt_sec,
            &self.model.rig,
            &self.model.bundle,
            &telem.measured_rad,
            &self.model.target_rad,
            &self.model.calibration,
            &telem.current_ma,
            self.model.selected_joint,
            ViewportOptions {
                show_ghost,
                estop_active: estop,
                hud_blocked,
                show_view_cube: true,
                show_protractor: false,
                interactive_drag: !estop,
                imu_pitch_rad: telem.imu.pitch_rad,
                imu_roll_rad: telem.imu.roll_rad,
            },
        );
        self.apply_viewport_interaction(&interaction);

        self.render_control_floating_hud(ui, interaction.rect);
    }

    fn render_control_floating_hud(&mut self, ui: &mut egui::Ui, vp_rect: Rect) {
        self.viewport_hud_rects = [Rect::NOTHING; 5];
        if vp_rect.width() < 220.0 || vp_rect.height() < 180.0 {
            return;
        }

        let pad = 12.0;
        let hud_frame = || {
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(13, 18, 28, 224))
                .corner_radius(CornerRadius::same(8))
                .inner_margin(Margin::same(10))
                .stroke(Stroke::new(
                    1.0,
                    Color32::from_rgba_unmultiplied(52, 70, 96, 200),
                ))
        };

        // 1. Top-Left Floating HUD: IMU Attitude, Vertical Balance & Combat Push Tests
        let att_w = 264.0_f32.min(vp_rect.width() - 175.0).max(200.0);
        let att_area = Rect::from_min_size(
            pos2(vp_rect.left() + pad, vp_rect.top() + pad),
            vec2(att_w, 240.0),
        );
        let mut att_ui = ui.new_child(
            UiBuilder::new()
                .max_rect(att_area)
                .layout(egui::Layout::top_down(egui::Align::LEFT)),
        );
        att_ui.set_clip_rect(vp_rect);
        let att_resp = hud_frame().show(&mut att_ui, |ui| {
            ui.set_width(att_w - 20.0);
            let telem = self.model.latest_telemetry;
            let pitch_deg = telem.imu.pitch_rad.to_degrees();
            let roll_deg = telem.imu.roll_rad.to_degrees();
            let fallen = telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF);
            let recovering = !fallen && (pitch_deg.abs() > 4.5 || roll_deg.abs() > 4.5);

            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("UPPER-BODY BALANCE")
                        .strong()
                        .size(11.0)
                        .color(Color32::from_rgb(138, 162, 192)),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let (badge, col) = if fallen {
                        ("FALLEN", Color32::from_rgb(248, 113, 113))
                    } else if recovering {
                        ("RECOVERING", Color32::from_rgb(251, 191, 36))
                    } else {
                        ("UPRIGHT", Color32::from_rgb(74, 222, 128))
                    };
                    ui.label(RichText::new(badge).strong().size(11.0).color(col));
                });
            });

            ui.add_space(4.0);
            paint_tilt_bar(ui, "Pitch", pitch_deg, 42.0);
            ui.add_space(3.0);
            paint_tilt_bar(ui, "Roll ", roll_deg, 42.0);

            ui.add_space(7.0);
            ui.horizontal(|ui| {
                let bal_on = self.model.walk_cmd.balance_enabled;
                let bal_btn = egui::Button::new(
                    RichText::new(if bal_on { "Balance: ON" } else { "Balance: OFF" })
                        .strong()
                        .size(12.0)
                        .color(if bal_on {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(200, 160, 160)
                        }),
                )
                .fill(if bal_on {
                    Color32::from_rgb(14, 116, 144)
                } else {
                    Color32::from_rgb(64, 30, 36)
                })
                .min_size(Vec2::new(112.0, 28.0))
                .corner_radius(CornerRadius::same(5));
                if ui.add(bal_btn).clicked() {
                    self.model.walk_cmd.balance_enabled = !bal_on;
                    self.model.status_message = if self.model.walk_cmd.balance_enabled {
                        "MCU upper-body vertical balance ENABLED.".to_string()
                    } else {
                        "MCU upper-body vertical balance DISABLED (unassisted test).".to_string()
                    };
                }

                let reset_btn = egui::Button::new(RichText::new("Reset").size(12.0))
                    .min_size(Vec2::new(62.0, 28.0))
                    .corner_radius(CornerRadius::same(5));
                if ui.add(reset_btn).clicked() {
                    self.transport.reset_upright();
                    if self.model.is_estop_active() {
                        self.model.reset_estop();
                    }
                    self.model.status_message = "Reset Rocky upright at 0 deg pitch/roll.".to_string();
                }
            });

            if self.model.requested_mode == ControlMode::Teach && !self.model.is_estop_active() {
                ui.add_space(5.0);
                let drive_cta = egui::Button::new(
                    RichText::new("Engage Drive for Walk & Balance")
                        .strong()
                        .size(11.5)
                        .color(Color32::from_rgb(14, 18, 24)),
                )
                .fill(Color32::from_rgb(245, 158, 11))
                .min_size(Vec2::new(ui.available_width(), 28.0))
                .corner_radius(CornerRadius::same(5));
                if ui.add(drive_cta).clicked() {
                    self.model.set_authority_mode(ControlMode::Drive);
                }
            }

            ui.add_space(7.0);
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("COMBAT SHOVE")
                        .strong()
                        .size(10.5)
                        .color(Color32::from_rgb(138, 162, 192)),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    for strength in [PushStrength::Heavy, PushStrength::Shove, PushStrength::Jab] {
                        let active = self.push_strength == strength;
                        let b = egui::Button::new(
                            RichText::new(strength.label())
                                .size(10.5)
                                .color(if active {
                                    Color32::WHITE
                                } else {
                                    Color32::from_rgb(150, 168, 190)
                                }),
                        )
                        .fill(if active {
                            Color32::from_rgb(14, 116, 144)
                        } else {
                            Color32::from_rgb(22, 30, 44)
                        })
                        .min_size(Vec2::new(0.0, 20.0))
                        .corner_radius(CornerRadius::same(4));
                        if ui.add(b).clicked() {
                            self.push_strength = strength;
                        }
                    }
                });
            });

            ui.add_space(4.0);
            let imp = self.push_strength.impulse_rad_s();
            let btn_w = ((ui.available_width() - 12.0) / 4.0).floor().max(42.0);
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for (lbl, dp, dr) in [
                    ("Push F", imp, 0.0),
                    ("Push B", -imp, 0.0),
                    ("Push L", 0.0, -imp * 0.85),
                    ("Push R", 0.0, imp * 0.85),
                ] {
                    let b = egui::Button::new(RichText::new(lbl).strong().size(11.5))
                        .fill(Color32::from_rgb(38, 46, 64))
                        .min_size(Vec2::new(btn_w, 28.0))
                        .corner_radius(CornerRadius::same(5));
                    if ui.add(b).clicked() {
                        if self.model.requested_mode == ControlMode::Teach
                            && !self.model.is_estop_active()
                        {
                            self.model.set_authority_mode(ControlMode::Drive);
                        }
                        self.transport.apply_push_impulse(dp, dr);
                        self.model.status_message = format!(
                            "Injected {} ({}) into simulated torso.",
                            lbl,
                            self.push_strength.label()
                        );
                    }
                }
            });
        });
        self.viewport_hud_rects[0] = att_resp.response.rect.expand(3.0);

        // 2. Bottom-Left Floating HUD: Touch Walk Pad + Turn Buttons + Viewport Toggles
        let walk_w = 246.0_f32.min(vp_rect.width() * 0.48).max(195.0);
        let walk_h = 204.0;
        let walk_area = Rect::from_min_size(
            pos2(vp_rect.left() + pad, vp_rect.bottom() - walk_h - pad),
            vec2(walk_w, walk_h),
        );
        let mut walk_ui = ui.new_child(
            UiBuilder::new()
                .max_rect(walk_area)
                .layout(egui::Layout::top_down(egui::Align::LEFT)),
        );
        walk_ui.set_clip_rect(vp_rect);
        let walk_resp = hud_frame().show(&mut walk_ui, |ui| {
            ui.set_width(walk_w - 20.0);
            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("WALK PAD")
                        .strong()
                        .size(11.0)
                        .color(Color32::from_rgb(138, 162, 192)),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let walking = self.model.walk_cmd.walk_enabled;
                    let w_btn = egui::Button::new(
                        RichText::new(if walking { "Walk: ON" } else { "Stand" })
                            .strong()
                            .size(11.5)
                            .color(if walking {
                                Color32::from_rgb(14, 18, 24)
                            } else {
                                Color32::from_rgb(190, 206, 224)
                            }),
                    )
                    .fill(if walking {
                        Color32::from_rgb(56, 208, 248)
                    } else {
                        Color32::from_rgb(26, 36, 52)
                    })
                    .min_size(Vec2::new(72.0, 24.0))
                    .corner_radius(CornerRadius::same(4));
                    if ui.add(w_btn).clicked() {
                        if !walking
                            && self.model.requested_mode == ControlMode::Teach
                            && !self.model.is_estop_active()
                        {
                            self.model.set_authority_mode(ControlMode::Drive);
                        }
                        self.model.walk_cmd.walk_enabled = !walking;
                        if !self.model.walk_cmd.walk_enabled {
                            self.pad_vx = 0.0;
                            self.pad_vy = 0.0;
                            self.pad_yaw = 0.0;
                        }
                    }
                });
            });

            ui.add_space(6.0);
            ui.horizontal(|ui| {
                // Interactive Circular Analog Walk Stick (touch & mouse drag)
                let pad_diam = 108.0;
                let (stick_rect, stick_resp) =
                    ui.allocate_exact_size(Vec2::splat(pad_diam), Sense::click_and_drag());
                let center = stick_rect.center();
                let radius = pad_diam * 0.45;

                if stick_resp.dragged() {
                    if let Some(pos) = stick_resp.interact_pointer_pos() {
                        let delta = pos - center;
                        let clamped = if delta.length() > radius {
                            delta.normalized() * radius
                        } else {
                            delta
                        };
                        self.pad_vx = (-clamped.y / radius).clamp(-1.0, 1.0);
                        self.pad_vy = (-clamped.x / radius).clamp(-1.0, 1.0);
                        if !self.model.walk_cmd.walk_enabled {
                            self.model.walk_cmd.walk_enabled = true;
                        }
                        if self.model.requested_mode == ControlMode::Teach
                            && !self.model.is_estop_active()
                        {
                            self.model.set_authority_mode(ControlMode::Drive);
                        }
                    }
                } else {
                    self.pad_vx = 0.0;
                    self.pad_vy = 0.0;
                }

                let p = ui.painter();
                p.circle_filled(center, radius, Color32::from_rgb(17, 25, 38));
                p.circle_stroke(center, radius, Stroke::new(1.2, Color32::from_rgb(56, 78, 108)));
                p.line_segment(
                    [pos2(center.x - radius + 8.0, center.y), pos2(center.x + radius - 8.0, center.y)],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(64, 86, 114, 110)),
                );
                p.line_segment(
                    [pos2(center.x, center.y - radius + 8.0), pos2(center.x, center.y + radius - 8.0)],
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(64, 86, 114, 110)),
                );
                let knob_pos = pos2(
                    center.x - self.model.walk_cmd.vy * radius * 0.85,
                    center.y - self.model.walk_cmd.vx * radius * 0.85,
                );
                p.circle_filled(knob_pos, 18.0, Color32::from_rgb(14, 132, 168));
                p.circle_stroke(knob_pos, 18.0, Stroke::new(1.5, Color32::from_rgb(103, 232, 249)));

                ui.vertical(|ui| {
                    ui.label(
                        RichText::new("TURN")
                            .strong()
                            .size(10.0)
                            .color(Color32::from_rgb(126, 148, 174)),
                    );
                    let mut yaw_hold = 0.0_f32;
                    ui.horizontal(|ui| {
                        let ql = ui.add(
                            egui::Button::new(RichText::new("Q").strong().size(12.5))
                                .min_size(Vec2::new(40.0, 32.0))
                                .corner_radius(CornerRadius::same(5)),
                        );
                        let qr = ui.add(
                            egui::Button::new(RichText::new("E").strong().size(12.5))
                                .min_size(Vec2::new(40.0, 32.0))
                                .corner_radius(CornerRadius::same(5)),
                        );
                        if ql.is_pointer_button_down_on() {
                            yaw_hold = 0.85;
                        } else if qr.is_pointer_button_down_on() {
                            yaw_hold = -0.85;
                        }
                    });
                    self.pad_yaw = yaw_hold;
                    if yaw_hold.abs() > 0.01 {
                        self.model.walk_cmd.walk_enabled = true;
                        if self.model.requested_mode == ControlMode::Teach
                            && !self.model.is_estop_active()
                        {
                            self.model.set_authority_mode(ControlMode::Drive);
                        }
                    }

                    ui.add_space(6.0);
                    ui.label(
                        RichText::new(format!(
                            "vx {:+.2}\nvy {:+.2}",
                            self.model.walk_cmd.vx, self.model.walk_cmd.vy
                        ))
                        .monospace()
                        .size(11.0)
                        .color(Color32::from_rgb(172, 194, 218)),
                    );
                    ui.label(
                        RichText::new(if self.active_gamepad_name.is_some() {
                            "WASD + Pad"
                        } else {
                            "WASD Ready"
                        })
                        .size(10.5)
                        .color(Color32::from_rgb(112, 198, 224)),
                    );
                });
            });

            ui.add_space(5.0);
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                if ui.small_button(self.model.rig.up_axis.label()).clicked() {
                    self.toggle_urdf_up_axis();
                }
                ui.checkbox(&mut self.viewport.show_grid, "Grid");
                ui.checkbox(&mut self.viewport.show_joint_markers, "Joints");
            });
        });
        self.viewport_hud_rects[1] = walk_resp.response.rect.expand(3.0);

        // 3. Bottom-Right Floating HUD: Combat Action Quick Bar & Quick Balance Tuning Popover
        let act_w = 276.0_f32.min(vp_rect.width() - walk_w - pad * 3.0).max(160.0);
        if act_w >= 180.0 {
            let pop_h = if self.show_quick_balance_tune { 226.0 } else { 78.0 };
            let act_area = Rect::from_min_size(
                pos2(vp_rect.right() - act_w - pad, vp_rect.bottom() - pop_h - pad),
                vec2(act_w, pop_h),
            );
            let mut act_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(act_area)
                    .layout(egui::Layout::bottom_up(egui::Align::RIGHT)),
            );
            act_ui.set_clip_rect(vp_rect);
            let act_resp = hud_frame().show(&mut act_ui, |ui| {
                ui.set_width(act_w - 20.0);
                ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("COMBAT MOVES")
                                .strong()
                                .size(10.5)
                                .color(Color32::from_rgb(138, 162, 192)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let tune_btn = egui::Button::new(
                                RichText::new(if self.show_quick_balance_tune {
                                    "Hide PD"
                                } else {
                                    "Tune PD"
                                })
                                .size(11.0),
                            )
                            .fill(if self.show_quick_balance_tune {
                                Color32::from_rgb(14, 116, 144)
                            } else {
                                Color32::from_rgb(26, 36, 52)
                            })
                            .min_size(Vec2::new(62.0, 22.0))
                            .corner_radius(CornerRadius::same(4));
                            if ui.add(tune_btn).clicked() {
                                self.show_quick_balance_tune = !self.show_quick_balance_tune;
                            }
                        });
                    });

                    ui.add_space(4.0);
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing = vec2(5.0, 5.0);
                        let move_count = self.model.moves.len().min(4);
                        for idx in 0..move_count {
                            let name = self.model.moves[idx].name.clone();
                            let b = egui::Button::new(RichText::new(name).strong().size(11.5))
                                .min_size(Vec2::new(0.0, 28.0))
                                .corner_radius(CornerRadius::same(5));
                            if ui.add(b).clicked() {
                                if self.model.requested_mode == ControlMode::Teach
                                    && !self.model.is_estop_active()
                                {
                                    self.model.set_authority_mode(ControlMode::Drive);
                                }
                                self.model.walk_cmd.walk_enabled = false;
                                self.model.play_move(idx);
                            }
                        }
                    });

                    if self.show_quick_balance_tune {
                        ui.separator();
                        let mut gains = self.model.balance_gains;
                        let mut changed = false;
                        ui.spacing_mut().slider_width = (act_w - 118.0).max(76.0);
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.kp_pitch, 0.0..=3.0).text("Kp Pitch"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.kd_pitch, 0.0..=1.0).text("Kd Pitch"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.kp_roll, 0.0..=3.0).text("Kp Roll"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.arm_assist, 0.0..=1.4).text("Arm Assist"))
                            .changed();
                        if changed {
                            self.model.update_balance_gains(gains);
                        }
                    }
                });
            });
            self.viewport_hud_rects[2] = act_resp.response.rect.expand(3.0);
        }
    }
}

fn paint_tilt_bar(ui: &mut egui::Ui, label: &str, deg: f32, max_deg: f32) {
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(format!("{label} {:+5.1}°", deg))
                .monospace()
                .size(11.5)
                .color(Color32::from_rgb(206, 222, 240)),
        );
        let bar_w = ui.available_width().max(64.0);
        let (rect, _) = ui.allocate_exact_size(vec2(bar_w, 10.0), Sense::hover());
        let p = ui.painter();
        p.rect_filled(rect, CornerRadius::same(3), Color32::from_rgb(20, 28, 42));
        let mid_x = rect.center().x;
        p.line_segment(
            [Pos2::new(mid_x, rect.top()), Pos2::new(mid_x, rect.bottom())],
            Stroke::new(1.0, Color32::from_rgb(76, 98, 128)),
        );
        let norm = (deg / max_deg.max(1.0)).clamp(-1.0, 1.0);
        let fill_col = if deg.abs() > 26.0 {
            Color32::from_rgb(248, 113, 113)
        } else if deg.abs() > 9.0 {
            Color32::from_rgb(251, 191, 36)
        } else {
            Color32::from_rgb(56, 208, 248)
        };
        let x0 = mid_x.min(mid_x + norm * (rect.width() * 0.5));
        let x1 = mid_x.max(mid_x + norm * (rect.width() * 0.5));
        if (x1 - x0) > 1.0 {
            p.rect_filled(
                Rect::from_min_max(pos2(x0, rect.top() + 1.5), pos2(x1, rect.bottom() - 1.5)),
                CornerRadius::same(2),
                fill_col,
            );
        }
    });
}
