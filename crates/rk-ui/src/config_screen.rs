//! Screen 3 — Config & Calibration Screen (Visual 3D direct joint/arm dragging,
//! 3D Protractor Rotation Arc, floating bottom-center visual joint calibration bar,
//! and on-demand 3-tab slide-over engineering drawer: Balance & Gait, Joints, Setup).
//!
//! The achieved CoM/GRF/contact plots below use simulator-only WORLD/GRF
//! telemetry channels; hardware v1 leaves them zero/clear, so those traces
//! stay flat at zero on real hardware.

use egui::{Color32, CornerRadius, Margin, Rect, RichText, Stroke, UiBuilder, Vec2, pos2, vec2};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use rk_fw_core::replay::verify_deterministic_replay;
use rk_proto::{BalanceGains, ConfigCmd, ControlMode, estimator_valid};
use rk_viewport::{DragMode, FootContactState, ViewportOptions};

use crate::{ConfigDrawerTab, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_config_screen(&mut self, ui: &mut egui::Ui) {

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

        // WORLD/GRF channels are simulator-only enrichment; hardware v1 leaves
        // them clear/zero. Gate the per-foot marks on the GRF validity bit so the
        // viewport does not render phantom contact on a v1 MCU.
        let grf_valid = telem.estimator.valid_mask & estimator_valid::GRF != 0;
        self.viewport.set_foot_contact_state(FootContactState {
            contact_mask: if grf_valid { telem.estimator.contact_mask } else { 0 },
            grf_left_n: if grf_valid { telem.estimator.grf_left_n } else { 0.0 },
            grf_right_n: if grf_valid { telem.estimator.grf_right_n } else { 0.0 },
        });

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
                show_protractor: true,
                interactive_drag: !estop,
                imu_available: telem.imu.available,
                imu_pitch_rad: telem.imu.pitch_rad,
                imu_roll_rad: telem.imu.roll_rad,
                fall_event_active: self.fall_event_active
                    || (telem.faults.contains(rk_proto::FaultFlags::TILT_FALL_CUTOFF)
                        && telem.imu.available),
                world_offset: telem.world_offset(),
            },
        );
        self.apply_viewport_interaction(&interaction);

        self.render_config_floating_overlays(ui, interaction.rect);
    }

    fn render_config_floating_overlays(&mut self, ui: &mut egui::Ui, vp_rect: Rect) {
        self.viewport_hud_rects = [Rect::NOTHING; 5];
        if vp_rect.width() < 220.0 || vp_rect.height() < 180.0 {
            return;
        }

        let pad = 12.0;
        let hud_frame = || {
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(13, 18, 28, 228))
                .corner_radius(CornerRadius::same(8))
                .inner_margin(Margin::symmetric(10, 7))
                .stroke(Stroke::new(
                    1.0,
                    Color32::from_rgba_unmultiplied(54, 72, 98, 205),
                ))
        };

        // 1. Top-Left Floating Strip: Direct Drag Mode (Auto / FK / IK) + Display + Engineering Drawer Toggle
        let top_w = (vp_rect.width() - 175.0).max(200.0);
        let top_area = Rect::from_min_size(
            pos2(vp_rect.left() + pad, vp_rect.top() + pad),
            vec2(top_w, 46.0),
        );
        let mut top_ui = ui.new_child(
            UiBuilder::new()
                .max_rect(top_area)
                .layout(egui::Layout::left_to_right(egui::Align::Center)),
        );
        top_ui.set_clip_rect(vp_rect);
        let top_resp = hud_frame().show(&mut top_ui, |ui| {
            ui.spacing_mut().item_spacing.x = 6.0;

            ui.label(
                RichText::new("DRAG:")
                    .strong()
                    .size(10.5)
                    .color(Color32::from_rgb(132, 154, 182)),
            );
            for mode in [DragMode::Auto, DragMode::FkJoint, DragMode::LimbIk] {
                let active = self.viewport.drag_mode == mode;
                let btn = egui::Button::new(
                    RichText::new(mode.label())
                        .strong()
                        .size(11.5)
                        .color(if active {
                            Color32::WHITE
                        } else {
                            Color32::from_rgb(164, 182, 206)
                        }),
                )
                .fill(if active {
                    Color32::from_rgb(14, 116, 144)
                } else {
                    Color32::from_rgb(22, 30, 44)
                })
                .min_size(Vec2::new(0.0, 26.0))
                .corner_radius(CornerRadius::same(4));
                if ui.add(btn).clicked() {
                    self.viewport.drag_mode = mode;
                }
            }

            ui.separator();
            if ui.small_button(self.model.rig.up_axis.label()).clicked() {
                self.toggle_urdf_up_axis();
            }
            ui.checkbox(&mut self.viewport.show_grid, "Grid");
            ui.checkbox(&mut self.viewport.show_joint_markers, "Joints");

            ui.separator();
            let drawer_btn = egui::Button::new(
                RichText::new(if self.show_config_drawer {
                    "Hide Setup Drawer"
                } else {
                    "Setup & Gains"
                })
                .strong()
                .size(11.5)
                .color(if self.show_config_drawer {
                    Color32::from_rgb(14, 18, 24)
                } else {
                    Color32::from_rgb(232, 242, 255)
                }),
            )
            .fill(if self.show_config_drawer {
                Color32::from_rgb(245, 158, 11)
            } else {
                Color32::from_rgb(28, 40, 58)
            })
            .min_size(Vec2::new(0.0, 26.0))
            .corner_radius(CornerRadius::same(4));
            if ui.add(drawer_btn).clicked() {
                self.show_config_drawer = !self.show_config_drawer;
            }
        });
        self.viewport_hud_rects[0] = top_resp.response.rect.expand(3.0);

        // 2. Bottom-Center Floating Visual Joint Configuration Bar (updates when clicking any joint)
        if !self.model.rig.joints.is_empty() {
            let j_idx = self
                .model
                .selected_joint
                .min(self.model.rig.joints.len() - 1);
            let drawer_reserve = if self.show_config_drawer { 366.0 } else { 0.0 };
            let bar_w = (vp_rect.width() - drawer_reserve - pad * 2.0)
                .clamp(280.0, 660.0);
            let bar_h = 112.0;
            let bar_x = vp_rect.left()
                + ((vp_rect.width() - drawer_reserve - bar_w) * 0.5).max(pad);
            let bar_area = Rect::from_min_size(
                pos2(bar_x, vp_rect.bottom() - bar_h - pad),
                vec2(bar_w, bar_h),
            );
            let mut bar_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(bar_area)
                    .layout(egui::Layout::top_down(egui::Align::LEFT)),
            );
            bar_ui.set_clip_rect(vp_rect);
            let bar_resp = hud_frame().show(&mut bar_ui, |ui| {
                ui.set_width(bar_w - 20.0);
                let j_label = self.model.rig.joints[j_idx].label.clone();
                let j_name = self.model.rig.joints[j_idx].name.clone();
                let measured_rad = self.model.latest_telemetry.measured_rad[j_idx];
                let current_ma = self.model.latest_telemetry.current_ma[j_idx];
                let cal = self.model.calibration[j_idx];
                let torque_on = (self.model.torque_mask & (1u64 << j_idx)) != 0;

                // Row 1: Joint identity, live angle & current, Torque / Zero / Invert / Reset
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(format!("#{j_idx:02} {j_label}"))
                            .strong()
                            .size(13.5)
                            .color(Color32::from_rgb(103, 232, 249)),
                    );
                    ui.label(
                        RichText::new(format!("({j_name})"))
                            .size(11.0)
                            .color(Color32::from_rgb(130, 150, 176)),
                    );
                    ui.label(
                        RichText::new(format!(
                            "{:+.1}°  ·  {} mA",
                            measured_rad.to_degrees(),
                            current_ma
                        ))
                        .monospace()
                        .size(11.5)
                        .color(Color32::from_rgb(206, 222, 240)),
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let tq_btn = egui::Button::new(
                            RichText::new(if torque_on { "Torque ON" } else { "Torque OFF" })
                                .strong()
                                .size(11.0)
                                .color(if torque_on {
                                    Color32::WHITE
                                } else {
                                    Color32::from_rgb(176, 190, 210)
                                }),
                        )
                        .fill(if torque_on {
                            Color32::from_rgb(22, 128, 92)
                        } else {
                            Color32::from_rgb(36, 44, 58)
                        })
                        .min_size(Vec2::new(0.0, 24.0))
                        .corner_radius(CornerRadius::same(4));
                        if ui.add(tq_btn).clicked() {
                            self.model.set_joint_torque(j_idx, !torque_on);
                        }

                        if ui.small_button(if cal.inverted { "Inv: ON" } else { "Inv: OFF" }).clicked() {
                            self.model.toggle_invert(j_idx);
                        }
                        if ui.small_button("Zero Here").clicked() {
                            self.model.set_current_as_zero(j_idx);
                        }
                    });
                });

                ui.add_space(4.0);

                // Row 2: Direct visual angle trim slider + nudge
                ui.horizontal(|ui| {
                    let is_drive = self.model.requested_mode == ControlMode::Drive
                        && !self.model.is_estop_active();
                    let mut angle_deg = if is_drive {
                        self.model.target_rad[j_idx].to_degrees()
                    } else {
                        measured_rad.to_degrees()
                    };
                    let min_deg = cal.min_rad.to_degrees();
                    let max_deg = cal.max_rad.to_degrees();
                    let mut changed = false;

                    if ui.small_button("-5°").clicked() {
                        angle_deg = (angle_deg - 5.0).clamp(min_deg, max_deg);
                        changed = true;
                    }
                    ui.scope(|ui| {
                        ui.spacing_mut().slider_width = (bar_w - 205.0).max(90.0);
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut angle_deg, min_deg..=max_deg)
                                    .suffix("°")
                                    .fixed_decimals(1),
                            )
                            .changed();
                    });
                    if ui.small_button("+5°").clicked() {
                        angle_deg = (angle_deg + 5.0).clamp(min_deg, max_deg);
                        changed = true;
                    }
                    if ui.small_button("0°").clicked() {
                        angle_deg = 0.0_f32.clamp(min_deg, max_deg);
                        changed = true;
                    }

                    if changed {
                        let rad = angle_deg.to_radians();
                        self.apply_direct_joint_angle(j_idx, rad);
                    }
                });

                ui.add_space(3.0);

                // Row 3: Visual Soft-Limit Stops (Min / Max + Set-From-Current)
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("Soft Limits:")
                            .size(11.0)
                            .color(Color32::from_rgb(245, 180, 70)),
                    );
                    let mut min_deg = cal.min_rad.to_degrees();
                    let mut max_deg = cal.max_rad.to_degrees();
                    let mut lim_changed = false;

                    lim_changed |= ui
                        .add(
                            egui::DragValue::new(&mut min_deg)
                                .speed(1.0)
                                .range(-180.0..=max_deg)
                                .prefix("Min ")
                                .suffix("°"),
                        )
                        .changed();
                    if ui.small_button("Set Min").clicked() {
                        min_deg = measured_rad.to_degrees().min(max_deg - 1.0);
                        lim_changed = true;
                    }
                    lim_changed |= ui
                        .add(
                            egui::DragValue::new(&mut max_deg)
                                .speed(1.0)
                                .range(min_deg..=180.0)
                                .prefix("Max ")
                                .suffix("°"),
                        )
                        .changed();
                    if ui.small_button("Set Max").clicked() {
                        max_deg = measured_rad.to_degrees().max(min_deg + 1.0);
                        lim_changed = true;
                    }
                    if lim_changed {
                        self.model
                            .update_soft_limits(j_idx, min_deg.to_radians(), max_deg.to_radians());
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("Commit Flash").clicked() {
                            self.model.pending_config.push(ConfigCmd::CommitFlash);
                            self.model.status_message =
                                "Queued calibration commit to MCU flash.".to_string();
                        }
                    });
                });
            });
            self.viewport_hud_rects[1] = bar_resp.response.rect.expand(3.0);
        }

        // 3. On-Demand Right Slide-Over Engineering Drawer (3 Tabs: Balance & Gait, Joints, Setup)
        if self.show_config_drawer {
            let drawer_w = 352.0_f32.min(vp_rect.width() - 24.0);
            let drawer_top = vp_rect.top() + 148.0; // Sits cleanly below the top-right ViewCube
            let drawer_h = (vp_rect.bottom() - drawer_top - pad).max(200.0);
            let drawer_area = Rect::from_min_size(
                pos2(vp_rect.right() - drawer_w - pad, drawer_top),
                vec2(drawer_w, drawer_h),
            );
            let mut drawer_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(drawer_area)
                    .layout(egui::Layout::top_down(egui::Align::LEFT)),
            );
            drawer_ui.set_clip_rect(vp_rect);
            let drawer_resp = egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(14, 20, 31, 240))
                .corner_radius(CornerRadius::same(10))
                .inner_margin(Margin::same(10))
                .stroke(Stroke::new(1.0, Color32::from_rgb(52, 72, 100)))
                .show(&mut drawer_ui, |ui| {
                    ui.set_min_size(vec2(drawer_w - 20.0, drawer_h - 20.0));
                    ui.set_max_size(vec2(drawer_w - 20.0, drawer_h - 20.0));

                    // 4-Tab Header: Balance | Actions | Joints | Setup
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        let tab_w = ((ui.available_width() - 12.0) / 4.0).floor();
                        for tab in [
                            ConfigDrawerTab::BalanceGait,
                            ConfigDrawerTab::Actions,
                            ConfigDrawerTab::Joints,
                            ConfigDrawerTab::Setup,
                        ] {
                            let active = self.config_drawer_tab == tab;
                            let btn = egui::Button::new(
                                RichText::new(tab.label())
                                    .strong()
                                    .size(11.0)
                                    .color(if active {
                                        Color32::WHITE
                                    } else {
                                        Color32::from_rgb(156, 174, 198)
                                    }),
                            )
                            .fill(if active {
                                Color32::from_rgb(14, 116, 144)
                            } else {
                                Color32::from_rgb(22, 30, 44)
                            })
                            .min_size(Vec2::new(tab_w, 28.0))
                            .corner_radius(CornerRadius::same(5));
                            if ui.add(btn).clicked() {
                                self.config_drawer_tab = tab;
                            }
                        }
                    });
                    ui.add_space(6.0);
                    ui.separator();

                    egui::ScrollArea::vertical().show(ui, |ui| match self.config_drawer_tab {
                        ConfigDrawerTab::BalanceGait => self.render_balance_gait_tab(ui),
                        ConfigDrawerTab::Actions => self.render_action_studio_panel(ui, false),
                        ConfigDrawerTab::Joints => self.render_calibration_panel(ui),
                        ConfigDrawerTab::Setup => self.render_setup_tab(ui),
                    });
                });
            self.viewport_hud_rects[2] = drawer_resp.response.rect.expand(3.0);
        }
    }

    fn render_balance_gait_tab(&mut self, ui: &mut egui::Ui) {
        ui.label(
            RichText::new("MCU Geometric UVC Balance (ai2001)")
                .strong()
                .size(13.0)
                .color(Color32::from_rgb(103, 232, 249)),
        );
        ui.label(
            RichText::new("Runs strictly in #![no_std] f32 with 6-DoF analytic leg IK on MCU.")
                .size(11.0)
                .color(Color32::from_rgb(148, 166, 190)),
        );
        ui.add_space(6.0);

        let mut g = self.model.balance_gains;
        let mut changed = false;
        changed |= ui
            .add(egui::Slider::new(&mut g.uvc_gain_pitch, 0.0..=0.80).text("UVC Pitch Gain"))
            .changed();
        changed |= ui
            .add(egui::Slider::new(&mut g.uvc_gain_roll, 0.0..=0.80).text("UVC Roll Gain"))
            .changed();
        let mut deadband_deg = g.tilt_deadband_rad.to_degrees();
        if ui
            .add(
                egui::Slider::new(&mut deadband_deg, 0.5..=6.0)
                    .suffix("°")
                    .text("Deadband"),
            )
            .changed()
        {
            g.tilt_deadband_rad = deadband_deg.to_radians();
            changed = true;
        }
        changed |= ui
            .add(egui::Slider::new(&mut g.gyro_damp_pitch, 0.0..=0.35).text("Gyro Pitch Damp"))
            .changed();
        changed |= ui
            .add(egui::Slider::new(&mut g.gyro_damp_roll, 0.0..=0.35).text("Gyro Roll Damp"))
            .changed();
        changed |= ui
            .add(egui::Slider::new(&mut g.arm_assist, 0.0..=1.5).text("Arm Assist"))
            .changed();
        changed |= ui
            .add(
                egui::Slider::new(&mut g.h_com_m, 0.12..=0.80)
                    .suffix(" m")
                    .text("CoM Height (DCM/ZMP)"),
            )
            .changed();
        let mut cutoff_deg = g.fall_cutoff_rad.to_degrees();
        if ui
            .add(
                egui::Slider::new(&mut cutoff_deg, 20.0..=65.0)
                    .suffix("°")
                    .text("Fall Cutoff"),
            )
            .changed()
        {
            g.fall_cutoff_rad = cutoff_deg.to_radians();
            changed = true;
        }
        if changed {
            self.model.update_balance_gains(g);
        }

        if ui.small_button("Reset Default UVC Gains").clicked() {
            self.model.update_balance_gains(BalanceGains::default());
        }

        ui.add_space(8.0);
        ui.separator();
        ui.label(
            RichText::new("Walking Gait Parameters")
                .strong()
                .size(13.0)
                .color(Color32::from_rgb(245, 180, 70)),
        );
        ui.add(
            egui::Slider::new(&mut self.model.walk_cmd.stance_crouch, 0.0..=1.0)
                .text("Combat Crouch"),
        );
        ui.add(
            egui::Slider::new(&mut self.model.walk_cmd.step_height, 0.1..=1.0).text("Step Height"),
        );
        ui.add(
            egui::Slider::new(&mut self.model.walk_cmd.cadence_hz, 0.6..=2.8)
                .suffix(" Hz")
                .text("Cadence"),
        );

        ui.add_space(6.0);
        ui.collapsing("Runtime UVC Gait & Impact", |ui| {
            let mut ug = self.model.uvc_gait;
            let mut ug_changed = false;
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_foot_lift, 0.005..=0.15).text("Max Foot Lift (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_sway, 0.005..=0.12).text("Max Sway (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_dx, 0.01..=0.25).text("Max dx (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_dy, 0.01..=0.25).text("Max dy (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_swing_dx, 0.01..=0.30).text("Max Swing dx (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.max_swing_dy, 0.01..=0.30).text("Max Swing dy (m)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.step_duration_s, 0.16..=0.45).text("Step Duration (s)"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.toe_push_off_gain, 0.0..=0.60).text("Toe Push Gain"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.waist_balance_gain, 0.0..=1.0).text("Waist Bal Gain"))
                .changed();
            ug_changed |= ui
                .add(egui::Slider::new(&mut ug.arm_swing_gain, 0.0..=2.0).text("Arm Swing Gain"))
                .changed();
            if ug_changed {
                self.model.update_uvc_gait(ug);
            }

            ui.separator();
            let mut ic = self.model.impact_config;
            let mut ic_changed = false;
            ic_changed |= ui
                .add(egui::Slider::new(&mut ic.k_o, 1.0..=200.0).text("Impact K_O"))
                .changed();
            ic_changed |= ui
                .add(egui::Slider::new(&mut ic.threshold_nm, 0.01..=5.0).text("Impact Thresh (Nm)"))
                .changed();
            ic_changed |= ui
                .add(egui::Slider::new(&mut ic.refractory_s, 0.0..=1.0).text("Refractory (s)"))
                .changed();
            ic_changed |= ui
                .add(egui::Slider::new(&mut ic.current_spike_ma, 1.0..=3000.0).text("Spike (mA)"))
                .changed();
            if ic_changed {
                self.model.update_impact_config(ic);
            }
        });

        ui.add_space(8.0);
        ui.separator();
        ui.label(
            RichText::new("Live IMU Pitch / Roll Telemetry (deg)")
                .strong()
                .size(12.0),
        );
        let pitch_pts: PlotPoints<'_> = self
            .history
            .iter()
            .map(|p| [p.t_sec, p.pitch_deg])
            .collect();
        let roll_pts: PlotPoints<'_> = self
            .history
            .iter()
            .map(|p| [p.t_sec, p.roll_deg])
            .collect();

        Plot::new("balance_imu_plot")
            .height(145.0)
            .legend(Legend::default())
            .show_axes(true)
            .show(ui, |plot_ui| {
                plot_ui.line(
                    Line::new("Pitch (deg)", pitch_pts)
                        .color(Color32::from_rgb(56, 208, 248))
                        .width(1.8),
                );
                plot_ui.line(
                    Line::new("Roll (deg)", roll_pts)
                        .color(Color32::from_rgb(245, 158, 11))
                        .width(1.8),
                );
            });

        // Achieved balance/contact signals. NOTE: the WORLD and GRF channels are
        // simulator enrichment (rk-sim publishes the validity bits + plant forces);
        // hardware v1 leaves them clear, so these traces stay flat at zero there.
        ui.add_space(6.0);
        ui.label(
            RichText::new("Achieved CoM — ground frame (m)")
                .strong()
                .size(12.0),
        );
        let com_x_pts = history_points(&self.history, |p| p.com_x_m);
        let com_z_pts = history_points(&self.history, |p| p.com_z_m);
        Plot::new("com_achieved_plot")
            .height(110.0)
            .legend(Legend::default())
            .show_axes(true)
            .show(ui, |plot_ui| {
                plot_ui.line(
                    Line::new("CoM X (m)", com_x_pts)
                        .color(Color32::from_rgb(132, 154, 182))
                        .width(1.6),
                );
                plot_ui.line(
                    Line::new("CoM Z (m)", com_z_pts)
                        .color(Color32::from_rgb(103, 232, 249))
                        .width(1.6),
                );
            });

        ui.add_space(6.0);
        ui.label(
            RichText::new("Achieved GRF (N) & contact mask")
                .strong()
                .size(12.0),
        );
        let grf_l_pts = history_points(&self.history, |p| p.grf_left_n);
        let grf_r_pts = history_points(&self.history, |p| p.grf_right_n);
        let contact_pts = history_points(&self.history, |p| p.contact_mask as f64 * 10.0);
        Plot::new("grf_contact_plot")
            .height(110.0)
            .legend(Legend::default())
            .show_axes(true)
            .show(ui, |plot_ui| {
                plot_ui.line(
                    Line::new("GRF Left (N)", grf_l_pts)
                        .color(Color32::from_rgb(74, 222, 128))
                        .width(1.6),
                );
                plot_ui.line(
                    Line::new("GRF Right (N)", grf_r_pts)
                        .color(Color32::from_rgb(245, 158, 11))
                        .width(1.6),
                );
                // Contact mask plotted at 0/10/20/30 so it stays visually separate
                // from the force traces; 0 = no contact, 10 = right, 20 = left, 30 = both.
                plot_ui.line(
                    Line::new("Contact x10", contact_pts)
                        .color(Color32::from_rgb(190, 132, 255))
                        .width(1.3),
                );
            });
        ui.label(
            RichText::new("Contact mask: 0 none · 10 right · 20 left · 30 both")
                .size(10.0)
                .color(Color32::from_rgb(148, 166, 190)),
        );

        ui.add_space(6.0);
        ui.label(
            RichText::new("Torso tilt rate (deg/s)")
                .strong()
                .size(12.0),
        );
        let tilt_rate_pts = history_points(&self.history, |p| p.tilt_rate_dps);
        Plot::new("tilt_rate_plot")
            .height(100.0)
            .legend(Legend::default())
            .show_axes(true)
            .show(ui, |plot_ui| {
                plot_ui.line(
                    Line::new("Tilt rate (deg/s)", tilt_rate_pts)
                        .color(Color32::from_rgb(251, 191, 36))
                        .width(1.6),
                );
            });
    }

    fn render_setup_tab(&mut self, ui: &mut egui::Ui) {
        self.render_urdf_mcu_panel(ui);
        ui.add_space(10.0);
        ui.separator();

        // Deterministic Copper Replay Verification
        ui.label(
            RichText::new("Deterministic Copper Replay")
                .strong()
                .size(13.0)
                .color(Color32::from_rgb(103, 232, 249)),
        );
        ui.horizontal(|ui| {
            let rec = self.transport.is_recording();
            if ui
                .button(if rec { "Stop Recording" } else { "Record .copper" })
                .clicked()
            {
                if rec {
                    if let Some(log) = self.transport.stop_recording() {
                        self.replay_result = Some(verify_deterministic_replay(&log));
                        // Serialize once into app state so the Export button can
                        // hand it to the platform saver (browser download on WASM,
                        // atomic file write on desktop).
                        self.copper_export_json = serde_json::to_string_pretty(&log).ok();
                    }
                } else {
                    self.replay_result = None;
                    self.copper_export_json = None;
                    self.transport.start_recording();
                }
            }
            if let Some(json) = self.copper_export_json.clone() {
                if ui.button("Export .copper").clicked() {
                    match self.saver.save_file("rocky_replay.copper", "application/json", json.as_bytes()) {
                        Ok(msg) => self.model.status_message = msg,
                        Err(e) => self.model.status_message = format!("Replay export failed: {e}"),
                    }
                }
            }
            if let Some(ver) = self.replay_result {
                ui.label(
                    RichText::new(if ver.exact_match {
                        format!("EXACT {}/{}", ver.matched_ticks, ver.total_ticks)
                    } else {
                        format!("DIFF {}/{}", ver.matched_ticks, ver.total_ticks)
                    })
                    .strong()
                    .color(if ver.exact_match {
                        Color32::from_rgb(74, 222, 128)
                    } else {
                        Color32::from_rgb(248, 113, 113)
                    }),
                );
            }
        });

        ui.add_space(10.0);
        ui.separator();
        self.render_sim_fault_panel(ui);
    }
}

/// Builds an owned `(t_sec, value)` plot series from the telemetry history.
/// The resulting `PlotPoints` owns its samples, so it is independent of the
/// history buffer's borrow and safe to hand straight to `egui_plot`.
fn history_points(
    history: &std::collections::VecDeque<crate::TelemetryPoint>,
    value: impl Fn(&crate::TelemetryPoint) -> f64,
) -> PlotPoints<'static> {
    history.iter().map(|p| [p.t_sec, value(p)]).collect()
}
