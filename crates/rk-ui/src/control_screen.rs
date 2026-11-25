//! Screen 2 — Control Screen (Full-bleed 3D robot, Walk Pad + WASD/Gamepad,
//! MCU Upper-Body Vertical Balance HUD, Combat Push Tests, and Quick Combat Moves).

use egui::{Color32, CornerRadius, Margin, Pos2, Rect, RichText, Sense, Stroke, UiBuilder, Vec2, pos2, vec2};
use rk_proto::{ControlMode, FaultFlags, estimator_valid};
use rk_viewport::ViewportOptions;

use crate::{PushStrength, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_control_screen(&mut self, ui: &mut egui::Ui) {

        let pointer_pos = ui
            .ctx()
            .input(|i| i.pointer.interact_pos().or(i.pointer.hover_pos()));
        let hud_blocked = self.show_paste_urdf_dialog
            || self.show_import_dialog
            || (self.mcp.drawer_open
                && pointer_pos.is_some_and(|p| self.mcp.drawer_rect.contains(p)))
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
                imu_available: telem.imu.available,
                imu_pitch_rad: telem.imu.pitch_rad,
                imu_roll_rad: telem.imu.roll_rad,
                fall_event_active: self.fall_event_active
                    || (telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                        && telem.imu.available),
                world_offset: telem.world_offset(),
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

        // 1. Top-Left Floating HUD: IMU Attitude, UVC Vertical Balance & Combat Push Tests
        let att_w = 276.0_f32.min(vp_rect.width() - 175.0).max(200.0);
        let att_area = Rect::from_min_size(
            pos2(vp_rect.left() + pad, vp_rect.top() + pad),
            vec2(att_w, 268.0),
        );
        let sim_led_on = self.transport.sim_led_on();
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
            let phase = telem.balance.uvc_phase;
            let phase_label = rk_proto::uvc_phase::label(phase);

            ui.horizontal(|ui| {
                ui.label(
                    RichText::new("UVC BALANCE")
                        .strong()
                        .size(11.0)
                        .color(Color32::from_rgb(138, 162, 192)),
                );
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Firmware `post_tick` LED state machine, mirrored by the sim
                    // IO and exposed through `RobotTransport::sim_led_on`.
                    let led_col = if sim_led_on {
                        Color32::from_rgb(74, 222, 128)
                    } else {
                        Color32::from_rgb(62, 76, 96)
                    };
                    ui.label(
                        RichText::new("LED")
                            .monospace()
                            .size(10.0)
                            .color(led_col),
                    );
                    let (led_rect, _) = ui.allocate_exact_size(vec2(9.0, 9.0), Sense::hover());
                    ui.painter().circle_filled(led_rect.center(), 4.5, led_col);
                    let (badge, col) = if fallen {
                        ("FALL CUTOFF", Color32::from_rgb(248, 113, 113))
                    } else if phase == rk_proto::uvc_phase::FALL_TUCK {
                        ("TUCK", Color32::from_rgb(248, 113, 113))
                    } else if phase == rk_proto::uvc_phase::ACTIVE_CATCH_STEP {
                        ("STEP", Color32::from_rgb(251, 191, 36))
                    } else if phase == rk_proto::uvc_phase::POST_STEP_SETTLE
                        || phase == rk_proto::uvc_phase::RETURN_TO_NEUTRAL
                        || phase == rk_proto::uvc_phase::RECOVERY_SETTLE
                    {
                        (phase_label, Color32::from_rgb(56, 208, 248))
                    } else if phase == rk_proto::uvc_phase::READY_WATCH {
                        ("READY", Color32::from_rgb(74, 222, 128))
                    } else {
                        (phase_label, Color32::from_rgb(156, 174, 198))
                    };
                    ui.label(RichText::new(badge).strong().size(11.0).color(col));
                    let sup_lbl = if telem.balance.support_foot == 1 {
                        "SUP L"
                    } else {
                        "SUP R"
                    };
                    ui.label(
                        RichText::new(sup_lbl)
                            .monospace()
                            .size(10.5)
                            .color(Color32::from_rgb(138, 162, 192)),
                    );
                });
            });

            ui.add_space(4.0);
            if telem.imu.available {
                paint_tilt_bar(ui, "Pitch", pitch_deg, 38.0);
                ui.add_space(3.0);
                paint_tilt_bar(ui, "Roll ", roll_deg, 38.0);
            } else {
                ui.label(
                    RichText::new("NO IMU · UPRIGHT LOCK (0.0° / 0.0°)")
                        .monospace()
                        .strong()
                        .size(11.0)
                        .color(Color32::from_rgb(148, 166, 190)),
                );
            }

            ui.add_space(4.0);
            ui.label(
                RichText::new(format!(
                    "dX {:+4.0} mm  dY {:+4.0} mm  H {:3.0} mm",
                    telem.balance.foot_dx_m * 1000.0,
                    telem.balance.foot_dy_m * 1000.0,
                    telem.balance.hip_height_m * 1000.0,
                ))
                .monospace()
                .size(10.5)
                .color(Color32::from_rgb(152, 178, 204)),
            );

            ui.add_space(6.0);
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
                    self.manual_reset_upright_and_clear();
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

        // Width of the right-hand column owned by the ViewCube / fall-recovery stack.
        // Computed here so the estimator HUD can be placed strictly to its left.
        let rec_w = 264.0_f32.min(vp_rect.width() - att_w - pad * 3.0);

        // 2. Top-Center-Right Floating HUD: the shared estimator + gait telemetry the
        //    phase-locked, capture-based controller consumes every tick. Right-aligned
        //    against the ViewCube / recovery column without ever covering either.
        let est_w = 250.0_f32.min(vp_rect.width() - att_w - rec_w - pad * 4.0);
        if est_w >= 200.0 {
            let est_left = vp_rect.right() - rec_w - pad * 2.0 - est_w;
            let est_area =
                Rect::from_min_size(pos2(est_left, vp_rect.top() + pad), vec2(est_w, 282.0));
            let mut est_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(est_area)
                    .layout(egui::Layout::top_down(egui::Align::LEFT)),
            );
            est_ui.set_clip_rect(vp_rect);
            let est_resp = hud_frame().show(&mut est_ui, |ui| {
                ui.set_width(est_w - 20.0);
                let telem = self.model.latest_telemetry;
                let bal = telem.balance;
                let est = telem.estimator;
                let valid = est.valid_mask;
                let com_ok = valid & estimator_valid::COM != 0;
                let dcm_ok = valid & estimator_valid::DCM != 0;
                let world_ok = valid & estimator_valid::WORLD != 0;
                // An absent world channel renders as an em dash, not a misleading 0.0.
                let world_ms = |v: f32| fmt_signed_ms(if world_ok { v } else { f32::NAN });
                let world_m = |v: f32| fmt_signed_m(if world_ok { v } else { f32::NAN });

                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("GAIT / ESTIMATOR")
                            .strong()
                            .size(11.0)
                            .color(Color32::from_rgb(138, 162, 192)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let (badge, col) = if bal.walk_active {
                            ("WALK", Color32::from_rgb(56, 208, 248))
                        } else if bal.balance_active {
                            ("BAL", Color32::from_rgb(74, 222, 128))
                        } else {
                            ("IDLE", Color32::from_rgb(156, 174, 198))
                        };
                        ui.label(RichText::new(badge).strong().size(10.5).color(col));
                    });
                });

                ui.add_space(4.0);
                let phase_deg = bal.gait_phase_rad.to_degrees().rem_euclid(360.0);
                let phase_frac = if bal.gait_phase_rad.is_finite() {
                    bal.gait_phase_rad.rem_euclid(std::f32::consts::TAU) / std::f32::consts::TAU
                } else {
                    0.0
                };
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("φ {phase_deg:5.0}°   {phase_frac:.2}"))
                            .monospace()
                            .size(11.5)
                            .color(Color32::from_rgb(206, 222, 240)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let sup = if bal.support_foot == 1 {
                            "SUP L"
                        } else {
                            "SUP R"
                        };
                        let col = if bal.support_foot == 1 {
                            Color32::from_rgb(103, 232, 249)
                        } else {
                            Color32::from_rgb(251, 191, 36)
                        };
                        ui.label(
                            RichText::new(sup)
                                .strong()
                                .monospace()
                                .size(10.5)
                                .color(col),
                        );
                    });
                });
                paint_gait_phase_bar(ui, phase_frac, bal.support_foot);

                // No dedicated wire field for the *adapted* step duration yet, so show the
                // commanded half-cycle period derived from the gait cadence.
                let cadence = self.model.walk_cmd.cadence_hz;
                let step_txt = if cadence.is_finite() && cadence > 0.1 {
                    format!("{:.2}s", 0.5 / cadence)
                } else {
                    "—".to_string()
                };
                ui.label(
                    RichText::new(format!(
                        "uvc {:>7}  ·  Tst {step_txt}",
                        rk_proto::uvc_phase::label(bal.uvc_phase)
                    ))
                    .monospace()
                    .size(10.0)
                    .color(Color32::from_rgb(148, 166, 190)),
                );

                ui.add_space(4.0);
                ui.separator();
                ui.add_space(3.0);

                let dim = Color32::from_rgb(112, 132, 156);
                let bright = Color32::from_rgb(172, 194, 218);
                let com_col = if com_ok { bright } else { dim };
                let dcm_col = if dcm_ok { bright } else { dim };
                ui.label(
                    RichText::new(format!(
                        "CoM {}/{}/{} mm",
                        fmt_signed_mm(est.com_pos_m[0]),
                        fmt_signed_mm(est.com_pos_m[1]),
                        fmt_signed_mm(est.com_pos_m[2])
                    ))
                    .monospace()
                    .size(10.5)
                    .color(com_col),
                );
                ui.label(
                    RichText::new(format!(
                        "Vel {}/{}/{} m/s",
                        fmt_signed_ms(est.com_vel_m_s[0]),
                        fmt_signed_ms(est.com_vel_m_s[1]),
                        fmt_signed_ms(est.com_vel_m_s[2])
                    ))
                    .monospace()
                    .size(10.5)
                    .color(com_col),
                );
                ui.label(
                    RichText::new(format!(
                        "DCM {}/{} mm (lat/sag)",
                        fmt_signed_mm(est.dcm_xy_m[0]),
                        fmt_signed_mm(est.dcm_xy_m[1])
                    ))
                    .monospace()
                    .size(10.5)
                    .color(dcm_col),
                );
                ui.label(
                    RichText::new(format!(
                        "ZMP {}/{} mm",
                        fmt_signed_mm(est.zmp_xy_m[0]),
                        fmt_signed_mm(est.zmp_xy_m[1])
                    ))
                    .monospace()
                    .size(10.5)
                    .color(dcm_col),
                );
                let margin_col = if est.foot_margin_m < 0.0 {
                    Color32::from_rgb(248, 113, 113)
                } else {
                    Color32::from_rgb(74, 222, 128)
                };
                ui.label(
                    RichText::new(format!("Margin {}", fmt_signed_mm(est.foot_margin_m)))
                        .monospace()
                        .size(10.5)
                        .color(if est.foot_margin_m.is_finite() {
                            margin_col
                        } else {
                            dim
                        }),
                );

                // Achieved locomotion from the plant's world telemetry (v6 wire).
                // Without a valid WORLD channel these rows show em dashes rather
                // than suggesting the robot moved when no world estimate exists.
                ui.label(
                    RichText::new(format!(
                        "World x/z {}/{} m · vz {} m/s",
                        world_m(bal.world_pos_m[0]),
                        world_m(bal.world_pos_m[2]),
                        world_ms(bal.world_vel_m_s[2]),
                    ))
                    .monospace()
                    .size(10.5)
                    .color(if world_ok { bright } else { dim }),
                );
                // WalkCmd.vx is normalized [-1, 1]; the kernel's sagittal target is vx * 0.5 m/s.
                let cmd_vx = self.model.walk_cmd.vx;
                ui.label(
                    RichText::new(format!(
                        "Cmd vx {:+.2} ({:+.2} m/s) -> Ach {} m/s",
                        cmd_vx,
                        cmd_vx * 0.5,
                        world_ms(bal.world_vel_m_s[2]),
                    ))
                    .monospace()
                    .size(10.5)
                    .color(if world_ok { bright } else { dim }),
                );

                let contact = match est.contact_mask & 0b11 {
                    0b01 => "R",
                    0b10 => "L",
                    0b11 => "R+L",
                    _ => "—",
                };
                let left = if est.contact_mask & 0b10 != 0 { "ON" } else { "--" };
                let right = if est.contact_mask & 0b01 != 0 { "ON" } else { "--" };
                let ctrl_hz = self.transport.stats().packet_rate_hz;
                let mut channels = String::new();
                if valid & estimator_valid::COM != 0 {
                    channels.push('C');
                }
                if valid & estimator_valid::DCM != 0 {
                    channels.push('D');
                }
                if valid & estimator_valid::GRF != 0 {
                    channels.push('G');
                }
                if valid & estimator_valid::MOMENTUM != 0 {
                    channels.push('M');
                }
                if valid & estimator_valid::LSM6DSV_OK != 0 {
                    channels.push('I');
                }
                if channels.is_empty() {
                    channels.push('—');
                }
                ui.label(
                    RichText::new(format!(
                        "Contact L:{left} R:{right} ({contact}) · {ctrl_hz:.0} Hz"
                    ))
                    .monospace()
                    .size(10.0)
                    .color(Color32::from_rgb(126, 148, 174)),
                )
                .on_hover_text(format!(
                    "Achieved control tick rate: {ctrl_hz:.1} Hz · estimator channels: {channels}"
                ));
            });
            self.viewport_hud_rects[4] = est_resp.response.rect.expand(3.0);
        }

        // 3. Bottom-Left Floating HUD: Touch Walk Pad + Turn Buttons + Viewport Toggles
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

        // 4. Mid-Right Floating HUD (cleanly BELOW the top-right ViewCube):
        //    Fall / Recovery status, consent-gated autonomous trigger, and directional fall test triggers.
        let rec_top = vp_rect.top() + 166.0; // Leaves the 138x158 top-right ViewCube 100% unobstructed
        let mut rec_bottom = vp_rect.top() + 166.0;
        if rec_w >= 190.0 && (vp_rect.bottom() - rec_top) >= 240.0 {
            let rec_area = Rect::from_min_size(
                pos2(vp_rect.right() - rec_w - pad, rec_top),
                vec2(rec_w, 232.0),
            );
            let mut rec_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(rec_area)
                    .layout(egui::Layout::top_down(egui::Align::LEFT)),
            );
            rec_ui.set_clip_rect(vp_rect);
            let rec_stroke = if self.fall_event_active {
                Stroke::new(1.5, Color32::from_rgb(248, 113, 113))
            } else {
                Stroke::new(1.0, Color32::from_rgba_unmultiplied(52, 70, 96, 200))
            };
            let rec_frame = egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(13, 18, 28, 228))
                .corner_radius(CornerRadius::same(8))
                .inner_margin(Margin::same(10))
                .stroke(rec_stroke);

            let rec_resp = rec_frame.show(&mut rec_ui, |ui| {
                ui.set_width(rec_w - 20.0);
                let (status, color) = match self.recovery_state {
                    crate::RecoveryState::PlayingGetUp
                    | crate::RecoveryState::ClearingFault
                    | crate::RecoveryState::ArmingDrive => {
                        ("RECOVERING", Color32::from_rgb(251, 191, 36))
                    }
                    _ if self.fall_event_active => {
                        ("FALL DETECTED", Color32::from_rgb(248, 113, 113))
                    }
                    _ => ("STABLE", Color32::from_rgb(74, 222, 128)),
                };
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("FALL RECOVERY")
                            .strong()
                            .size(11.0)
                            .color(Color32::from_rgb(138, 162, 192)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(status).strong().size(11.0).color(color));
                    });
                });
                ui.label(
                    RichText::new(format!(
                        "Pose: {}  ·  {}",
                        self.fall_orientation.label(),
                        self.recovery_state.label()
                    ))
                    .monospace()
                    .size(10.5)
                    .color(Color32::from_rgb(172, 194, 218)),
                );

                if self.recovery_state == crate::RecoveryState::PlayingGetUp {
                    if let Some(pb) = &self.model.playback {
                        ui.add_space(4.0);
                        let step_lbl = self
                            .model
                            .moves
                            .get(pb.move_idx)
                            .and_then(|mv| mv.keyframes.get(pb.active_keyframe))
                            .map(|kf| kf.label.as_str())
                            .unwrap_or("Standing up");
                        ui.add(
                            egui::ProgressBar::new(pb.progress.clamp(0.0, 1.0))
                                .desired_height(13.0)
                                .text(format!("{step_lbl} ({:.0}%)", pb.progress * 100.0)),
                        );
                    }
                }

                ui.add_space(3.0);
                ui.checkbox(
                    &mut self.auto_recovery_enabled,
                    "Auto recovery (opt-in)",
                );
                ui.checkbox(&mut self.operator_confirmed, "Operator confirmed");
                ui.add_space(3.0);

                let can_get_up = self.fall_event_active
                    && self.recovery_state == crate::RecoveryState::FallLatched;
                let get_up_label = if can_get_up {
                    format!("Get Up ({})", self.fall_orientation.label())
                } else if self.recovery_state == crate::RecoveryState::PlayingGetUp {
                    "Getting Up...".to_string()
                } else {
                    "Get Up (No Fall)".to_string()
                };
                let get_up = egui::Button::new(
                    RichText::new(get_up_label)
                        .strong()
                        .size(12.0)
                        .color(if can_get_up {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(156, 174, 198)
                        }),
                )
                .fill(if can_get_up {
                    Color32::from_rgb(14, 132, 168)
                } else {
                    Color32::from_rgb(26, 36, 52)
                })
                .stroke(if can_get_up {
                    Stroke::new(1.2, Color32::from_rgb(103, 232, 249))
                } else {
                    Stroke::NONE
                })
                .min_size(Vec2::new(ui.available_width(), 27.0))
                .corner_radius(CornerRadius::same(5));
                if ui.add_enabled(can_get_up, get_up).clicked() {
                    self.operator_confirmed = true;
                    self.get_up_requested = true;
                    self.model.status_message = format!(
                        "Get Up ({}) initiated — clearing fault and executing recovery sequence.",
                        self.fall_orientation.label()
                    );
                }

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("SIM FALL")
                            .strong()
                            .size(9.5)
                            .color(Color32::from_rgb(126, 148, 174)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 3.0;
                        for (lbl, orient) in [
                            ("R", rk_proto::FallOrientation::SideRight),
                            ("L", rk_proto::FallOrientation::SideLeft),
                            ("Back", rk_proto::FallOrientation::Supine),
                            ("Front", rk_proto::FallOrientation::Prone),
                        ] {
                            let fb = egui::Button::new(RichText::new(lbl).size(10.0))
                                .fill(Color32::from_rgb(54, 26, 32))
                                .stroke(Stroke::new(1.0, Color32::from_rgb(120, 48, 56)))
                                .min_size(Vec2::new(0.0, 20.0))
                                .corner_radius(CornerRadius::same(4));
                            if ui
                                .add_enabled(!self.fall_event_active, fb)
                                .on_hover_text(format!(
                                    "Simulate {} fall to test IMU alarm & get-up recovery",
                                    orient.label()
                                ))
                                .clicked()
                            {
                                self.trigger_simulated_fall(orient);
                            }
                        }
                    });
                });
            });
            rec_bottom = rec_resp.response.rect.bottom();
            self.viewport_hud_rects[3] = rec_resp.response.rect.expand(3.0);
        }

        // 5. Bottom-Right Floating HUD: Combat Moves Quick Bar, Action Studio & Quick Balance Tuning
        let max_act_w = (vp_rect.width() - walk_w - pad * 3.0).max(160.0);
        let desired_act_w: f32 = if self.show_action_studio { 348.0 } else { 304.0 };
        let act_w = desired_act_w.min(max_act_w);
        if act_w >= 180.0 {
            let max_below_rec = (vp_rect.bottom() - rec_bottom - pad * 2.0).max(110.0);
            let pop_h = if self.show_action_studio {
                348.0_f32.min(max_below_rec).max(180.0)
            } else if self.show_quick_balance_tune {
                268.0_f32.min(max_below_rec).max(140.0)
            } else {
                104.0
            };
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
                            RichText::new("ACTIONS & MOVES")
                                .strong()
                                .size(10.5)
                                .color(Color32::from_rgb(138, 162, 192)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.spacing_mut().item_spacing.x = 4.0;
                            let tune_btn = egui::Button::new(
                                RichText::new(if self.show_quick_balance_tune {
                                    "Hide UVC"
                                } else {
                                    "Tune UVC"
                                })
                                .size(10.5),
                            )
                            .fill(if self.show_quick_balance_tune {
                                Color32::from_rgb(14, 116, 144)
                            } else {
                                Color32::from_rgb(26, 36, 52)
                            })
                            .min_size(Vec2::new(60.0, 22.0))
                            .corner_radius(CornerRadius::same(4));
                            if ui.add(tune_btn).clicked() {
                                self.show_quick_balance_tune = !self.show_quick_balance_tune;
                                if self.show_quick_balance_tune {
                                    self.show_action_studio = false;
                                }
                            }

                            let studio_btn = egui::Button::new(
                                RichText::new(if self.show_action_studio {
                                    "Close Studio"
                                } else {
                                    "Action Studio"
                                })
                                .strong()
                                .size(10.5)
                                .color(if self.show_action_studio {
                                    Color32::from_rgb(14, 18, 24)
                                } else {
                                    Color32::from_rgb(253, 224, 171)
                                }),
                            )
                            .fill(if self.show_action_studio {
                                Color32::from_rgb(245, 158, 11)
                            } else {
                                Color32::from_rgb(44, 34, 24)
                            })
                            .min_size(Vec2::new(82.0, 22.0))
                            .corner_radius(CornerRadius::same(4));
                            if ui.add(studio_btn).clicked() {
                                self.show_action_studio = !self.show_action_studio;
                                if self.show_action_studio {
                                    self.show_quick_balance_tune = false;
                                }
                            }
                        });
                    });

                    ui.add_space(5.0);

                    if self.show_action_studio {
                        self.render_action_studio_panel(ui, true);
                    } else {
                        ui.horizontal_wrapped(|ui| {
                            ui.spacing_mut().item_spacing = vec2(5.0, 5.0);
                            let move_count = self.model.moves.len().min(4);
                            for idx in 0..move_count {
                                let name = self.model.moves[idx].name.clone();
                                let b = egui::Button::new(RichText::new(name).strong().size(11.5))
                                    .min_size(Vec2::new(0.0, 26.0))
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
                    }

                    if self.show_quick_balance_tune {
                        ui.add_space(4.0);
                        ui.separator();
                        let mut gains = self.model.balance_gains;
                        let mut changed = false;
                        ui.spacing_mut().slider_width = (act_w - 118.0).max(76.0);
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.uvc_gain_pitch, 0.0..=0.8).text("UVC Pitch"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.uvc_gain_roll, 0.0..=0.8).text("UVC Roll"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.gyro_damp_pitch, 0.0..=0.35).text("Gyro Damp"))
                            .changed();
                        changed |= ui
                            .add(egui::Slider::new(&mut gains.arm_assist, 0.0..=1.4).text("Arm Assist"))
                            .changed();
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut gains.h_com_m, 0.12..=0.80)
                                    .suffix(" m")
                                    .text("CoM Height"),
                            )
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

/// Signed metric-to-millimetres formatter for estimator rows. An invalid / absent
/// wire channel renders as an em dash instead of a misleading physical `0.0`.
fn fmt_signed_mm(v: f32) -> String {
    if v.is_finite() {
        format!("{:+5.0}", v * 1.0e3)
    } else {
        "    —".to_string()
    }
}

/// Signed m/s formatter for the CoM velocity row (same invalid-channel convention).
fn fmt_signed_ms(v: f32) -> String {
    if v.is_finite() {
        format!("{:+5.2}", v)
    } else {
        "    —".to_string()
    }
}

/// Signed metres formatter for world displacement rows (invalid -> em dash).
fn fmt_signed_m(v: f32) -> String {
    if v.is_finite() {
        format!("{:+5.2}", v)
    } else {
        "    —".to_string()
    }
}

/// Thin gait-phase progress bar with a mid-cycle support-swap tick.
fn paint_gait_phase_bar(ui: &mut egui::Ui, phase_frac: f32, support_foot: u8) {
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 8.0), Sense::hover());
    let p = ui.painter();
    p.rect_filled(rect, CornerRadius::same(3), Color32::from_rgb(20, 28, 42));
    p.line_segment(
        [
            pos2(rect.center().x, rect.top()),
            pos2(rect.center().x, rect.bottom()),
        ],
        Stroke::new(1.0, Color32::from_rgb(76, 98, 128)),
    );
    let col = if support_foot == 1 {
        Color32::from_rgb(103, 232, 249)
    } else {
        Color32::from_rgb(251, 191, 36)
    };
    if phase_frac.is_finite() {
        let frac = phase_frac.clamp(0.0, 1.0);
        let x1 = rect.left() + 1.0 + (rect.width() - 2.0) * frac;
        if x1 > rect.left() + 1.0 {
            p.rect_filled(
                Rect::from_min_max(
                    pos2(rect.left() + 1.0, rect.top() + 1.0),
                    pos2(x1, rect.bottom() - 1.0),
                ),
                CornerRadius::same(2),
                col,
            );
        }
    }
}

#[cfg(test)]
mod gait_hud_tests {
    use super::{fmt_signed_m, fmt_signed_mm, fmt_signed_ms};

    #[test]
    fn estimator_rows_format_finite_and_invalid_channels() {
        assert_eq!(fmt_signed_mm(0.007), "   +7");
        assert_eq!(fmt_signed_mm(-0.024), "  -24");
        assert_eq!(fmt_signed_mm(f32::NAN), "    —");
        assert_eq!(fmt_signed_ms(0.08), "+0.08");
        assert_eq!(fmt_signed_ms(-1.5), "-1.50");
        assert_eq!(fmt_signed_ms(f32::INFINITY), "    —");
        assert_eq!(fmt_signed_m(1.75), "+1.75");
        assert_eq!(fmt_signed_m(-0.2), "-0.20");
        assert_eq!(fmt_signed_m(f32::NAN), "    —");
    }
}
