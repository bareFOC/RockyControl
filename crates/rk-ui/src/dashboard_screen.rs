//! Screen 1 — Dashboard Home (Connect to Robot -> Live Robot Stats + Control / Config Launchers).

use egui::{Align, Color32, CornerRadius, Layout, Margin, RichText, Stroke, Vec2};
use rk_link::TransportKind;
use rk_model::{AssetBundle, samples};
use rk_proto::{ControlMode, FaultFlags};
use rk_viewport::ViewportOptions;

use crate::{AppScreen, RockyStudioApp};

impl RockyStudioApp {
    pub(crate) fn render_dashboard_screen(&mut self, ui: &mut egui::Ui) {
        if !self.connected {
            self.render_dashboard_disconnected(ui);
        } else {
            self.render_dashboard_connected(ui);
        }
    }

    fn render_dashboard_disconnected(&mut self, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let card_w = avail.x.min(720.0).max(320.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.vertical_centered(|ui| {
                let top_pad = ((avail.y - 520.0) * 0.32).clamp(18.0, 72.0);
                ui.add_space(top_pad);

                egui::Frame::new()
                    .fill(Color32::from_rgb(16, 22, 33))
                    .corner_radius(CornerRadius::same(12))
                    .inner_margin(Margin::same(26))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 74)))
                    .show(ui, |ui| {
                        ui.set_width(card_w - 52.0);
                        ui.with_layout(Layout::top_down(Align::LEFT), |ui| {
                            // Header
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new("ROCKY STUDIO")
                                        .strong()
                                        .size(24.0)
                                        .color(Color32::from_rgb(236, 245, 255)),
                                );
                                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                    egui::Frame::new()
                                        .fill(Color32::from_rgb(26, 36, 52))
                                        .corner_radius(CornerRadius::same(4))
                                        .inner_margin(Margin::symmetric(8, 3))
                                        .show(ui, |ui| {
                                            ui.label(
                                                RichText::new("DISCONNECTED")
                                                    .strong()
                                                    .size(11.0)
                                                    .color(Color32::from_rgb(148, 166, 190)),
                                            );
                                        });
                                });
                            });
                            ui.label(
                                RichText::new(
                                    "Connect to your humanoid controller to view live telemetry, walk & balance controls, and visual 3D calibration.",
                                )
                                .size(13.5)
                                .color(Color32::from_rgb(154, 172, 196)),
                            );

                            ui.add_space(18.0);
                            ui.label(
                                RichText::new("SELECT CONNECTION")
                                    .strong()
                                    .size(11.5)
                                    .color(Color32::from_rgb(110, 200, 232)),
                            );
                            ui.add_space(6.0);

                            // Transport target cards
                            self.transport_option_card(
                                ui,
                                TransportKind::SimKernel,
                                "Simulated Rocky (MCU Firmware Twin)",
                                "Runs the exact #![no_std] RP2350 walk & upper-body balance kernel with 2-axis IMU tilt physics locally.",
                                true,
                            );
                            ui.add_space(8.0);

                            #[cfg(not(target_arch = "wasm32"))]
                            let udp_avail = true;
                            #[cfg(target_arch = "wasm32")]
                            let udp_avail = false;
                            self.transport_option_card(
                                ui,
                                TransportKind::UsbNetworkingUdp,
                                "RP2350 USB-Net UDP (10.42.0.2:7777)",
                                if udp_avail {
                                    "Direct wired CDC-NCM Ethernet-over-USB link to physical RP2350 humanoid firmware."
                                } else {
                                    "Available on Desktop Native builds for direct wired RP2350 hardware."
                                },
                                udp_avail,
                            );
                            ui.add_space(8.0);

                            self.transport_option_card(
                                ui,
                                TransportKind::UsbSerialStage2,
                                "RP2350 USB CDC-ACM Serial",
                                "Hardware serial bridge (Stage 2 bring-up — select Sim Twin or USB-Net UDP to connect now).",
                                false,
                            );

                            ui.add_space(16.0);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new("Robot Model:")
                                        .size(12.5)
                                        .color(Color32::from_rgb(156, 174, 198)),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "{} ({} DoF)",
                                        self.model.rig.robot_name,
                                        self.model.rig.joints.len()
                                    ))
                                    .strong()
                                    .size(12.5)
                                    .color(Color32::from_rgb(226, 236, 248)),
                                );
                                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                    if ui.button("Load URDF / ZIP").clicked() {
                                        if let Err(err) = self.saver.request_open_urdf_file() {
                                            self.model.status_message = err;
                                        }
                                    }
                                    if self.model.rig.robot_name != "rocky_mk1"
                                        && ui.button("Reset Rocky Mk1").clicked()
                                    {
                                        self.apply_loaded_bundle(AssetBundle::bundled_rocky());
                                    }
                                });
                            });

                            ui.add_space(18.0);
                            let connect_btn = egui::Button::new(
                                RichText::new("Connect to Robot")
                                    .strong()
                                    .size(16.0)
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(14, 132, 168))
                            .stroke(Stroke::new(1.2, Color32::from_rgb(56, 208, 248)))
                            .min_size(Vec2::new(ui.available_width(), 46.0))
                            .corner_radius(CornerRadius::same(8));

                            if ui.add(connect_btn).clicked() {
                                self.connect_active_transport();
                            }

                            ui.add_space(14.0);
                            self.render_home_mcp_bar(ui);
                        });
                    });

                ui.add_space(14.0);
                // Optional quick preset switcher row
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("Presets:")
                            .size(11.5)
                            .color(Color32::from_rgb(120, 138, 162)),
                    );
                    if ui.small_button("Rocky Mk1 (21-DoF)").clicked() {
                        self.apply_loaded_bundle(AssetBundle::bundled_rocky());
                    }
                    if ui.small_button("Striker Biped (8-DoF)").clicked() {
                        if let Ok(b) = AssetBundle::from_sample(
                            "Striker Biped Z-Up",
                            samples::STRIKER_BIPED_ZUP_URDF,
                        ) {
                            self.apply_loaded_bundle(b);
                        }
                    }
                });
            });
        });
    }

    fn transport_option_card(
        &mut self,
        ui: &mut egui::Ui,
        kind: TransportKind,
        title: &str,
        desc: &str,
        enabled: bool,
    ) {
        let selected = self.selected_transport_ui == kind;
        let fill = if !enabled {
            Color32::from_rgb(18, 23, 33)
        } else if selected {
            Color32::from_rgb(16, 48, 66)
        } else {
            Color32::from_rgb(21, 28, 41)
        };
        let border = if !enabled {
            Color32::from_rgb(30, 39, 54)
        } else if selected {
            Color32::from_rgb(56, 189, 248)
        } else {
            Color32::from_rgb(42, 55, 76)
        };

        let resp = egui::Frame::new()
            .fill(fill)
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::symmetric(14, 11))
            .stroke(Stroke::new(if selected { 1.5 } else { 1.0 }, border))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    let dot_color = if !enabled {
                        Color32::from_rgb(70, 82, 100)
                    } else if selected {
                        Color32::from_rgb(56, 224, 248)
                    } else {
                        Color32::from_rgb(88, 106, 130)
                    };
                    let (dot_rect, _) =
                        ui.allocate_exact_size(Vec2::splat(14.0), egui::Sense::hover());
                    ui.painter().circle_stroke(
                        dot_rect.center(),
                        6.0,
                        Stroke::new(1.5, dot_color),
                    );
                    if selected {
                        ui.painter().circle_filled(dot_rect.center(), 3.2, dot_color);
                    }
                    ui.vertical(|ui| {
                        ui.label(
                            RichText::new(title)
                                .strong()
                                .size(14.0)
                                .color(if enabled {
                                    Color32::from_rgb(232, 242, 255)
                                } else {
                                    Color32::from_rgb(112, 126, 148)
                                }),
                        );
                        ui.label(
                            RichText::new(desc)
                                .size(12.0)
                                .color(if enabled {
                                    Color32::from_rgb(150, 170, 195)
                                } else {
                                    Color32::from_rgb(92, 106, 126)
                                }),
                        );
                    });
                });
            })
            .response
            .interact(if enabled {
                egui::Sense::click()
            } else {
                egui::Sense::hover()
            });

        if enabled && resp.clicked() {
            self.selected_transport_ui = kind;
        }
    }

    fn render_dashboard_connected(&mut self, ui: &mut egui::Ui) {
        let stats = self.transport.stats();
        let telem = self.model.latest_telemetry;
        let volts = (telem.bus_voltage_mv as f32) / 1000.0;
        let dof = self.model.rig.joints.len();
        let peak_ma = telem.current_ma[..dof.min(rk_proto::MAX_JOINTS)]
            .iter()
            .copied()
            .max()
            .unwrap_or(0);
        let max_temp = telem.temp_c[..dof.min(rk_proto::MAX_JOINTS)]
            .iter()
            .copied()
            .max()
            .unwrap_or(28);
        let pitch_deg = telem.imu.pitch_rad.to_degrees();
        let roll_deg = telem.imu.roll_rad.to_degrees();
        let active_torque_cnt = telem.torque_state_mask.count_ones();
        let uptime_s = (telem.uptime_ms as f32) * 0.001;

        let avail = ui.available_size();
        let max_w = avail.x.min(1180.0).max(340.0);
        let is_narrow = max_w < 760.0;

        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(16.0);
                ui.allocate_ui_with_layout(
                    Vec2::new(max_w - 28.0, 0.0),
                    Layout::top_down(Align::LEFT),
                    |ui| {
                        // Connected Robot Header Banner
                        egui::Frame::new()
                            .fill(Color32::from_rgb(16, 23, 34))
                            .corner_radius(CornerRadius::same(10))
                            .inner_margin(Margin::symmetric(18, 14))
                            .stroke(Stroke::new(1.0, Color32::from_rgb(38, 56, 80)))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.vertical(|ui| {
                                        ui.horizontal(|ui| {
                                            let (dot_r, _) = ui.allocate_exact_size(
                                                Vec2::splat(10.0),
                                                egui::Sense::hover(),
                                            );
                                            let status_col = if self.model.is_estop_active() {
                                                Color32::from_rgb(248, 113, 113)
                                            } else if stats.connected {
                                                Color32::from_rgb(74, 222, 128)
                                            } else {
                                                Color32::from_rgb(251, 191, 36)
                                            };
                                            ui.painter().circle_filled(dot_r.center(), 4.5, status_col);
                                            ui.label(
                                                RichText::new(format!(
                                                    "{}  ·  CONNECTED",
                                                    self.model.rig.robot_name.to_uppercase()
                                                ))
                                                .strong()
                                                .size(17.0)
                                                .color(Color32::from_rgb(236, 245, 255)),
                                            );
                                        });
                                        ui.label(
                                            RichText::new(format!(
                                                "{}  |  {} Actuated DoF  |  {:.0} Hz  ·  {:.1} ms RTT  |  Uptime {:.0}s",
                                                self.selected_transport_ui.label(),
                                                dof,
                                                stats.packet_rate_hz,
                                                stats.rtt_ms,
                                                uptime_s
                                            ))
                                            .size(12.5)
                                            .color(Color32::from_rgb(148, 168, 194)),
                                        );
                                    });

                                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                        let disc_btn = egui::Button::new(
                                            RichText::new("Disconnect")
                                                .strong()
                                                .size(12.5)
                                                .color(Color32::from_rgb(248, 180, 180)),
                                        )
                                        .fill(Color32::from_rgb(56, 24, 30))
                                        .stroke(Stroke::new(1.0, Color32::from_rgb(140, 52, 62)))
                                        .min_size(Vec2::new(104.0, 34.0))
                                        .corner_radius(CornerRadius::same(6));
                                        if ui.add(disc_btn).clicked() {
                                            self.disconnect_transport();
                                        }
                                    });
                                });
                            });

                        ui.add_space(14.0);

                        // Primary Launch Cards: CONTROL and CONFIG & CALIBRATION
                        let launch_h = 104.0;
                        if is_narrow {
                            self.launch_action_card(
                                ui,
                                "CONTROL",
                                "Walk, Balance & Combat",
                                "Live walking pad, WASD/gamepad driving, MCU upper-body vertical balance, combat push tests & quick moves.",
                                Color32::from_rgb(14, 116, 144),
                                Color32::from_rgb(56, 208, 248),
                                Vec2::new(ui.available_width(), launch_h),
                                AppScreen::Control,
                            );
                            ui.add_space(10.0);
                            self.launch_action_card(
                                ui,
                                "CONFIG & CALIBRATION",
                                "Visual 3D Joint Editing & Setup",
                                "Click and drag joints or arms directly on Rocky, adjust soft limits & offsets, tune balance PD gains, and sync URDF/MCU.",
                                Color32::from_rgb(36, 48, 72),
                                Color32::from_rgb(245, 158, 11),
                                Vec2::new(ui.available_width(), launch_h),
                                AppScreen::Config,
                            );
                        } else {
                            let gap = 14.0;
                            let half_w = ((ui.available_width() - gap) * 0.5).floor();
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = gap;
                                self.launch_action_card(
                                    ui,
                                    "CONTROL",
                                    "Walk, Balance & Combat",
                                    "Live walking pad, WASD/gamepad driving, MCU upper-body vertical balance, combat push tests & quick moves.",
                                    Color32::from_rgb(14, 102, 130),
                                    Color32::from_rgb(56, 208, 248),
                                    Vec2::new(half_w, launch_h),
                                    AppScreen::Control,
                                );
                                self.launch_action_card(
                                    ui,
                                    "CONFIG & CALIBRATION",
                                    "Visual 3D Joint Editing & Setup",
                                    "Click and drag joints or arms directly on Rocky, set soft limits & zero offsets, tune balance PD gains & sync URDF/MCU.",
                                    Color32::from_rgb(30, 40, 58),
                                    Color32::from_rgb(245, 158, 11),
                                    Vec2::new(half_w, launch_h),
                                    AppScreen::Config,
                                );
                            });
                        }

                        ui.add_space(14.0);

                        // Robot Stats Grid + Live 3D Preview
                        let preview_h = if is_narrow { 260.0 } else { 320.0 };
                        if is_narrow {
                            self.render_dashboard_stats_column(
                                ui,
                                volts,
                                peak_ma,
                                max_temp,
                                active_torque_cnt,
                                dof,
                                pitch_deg,
                                roll_deg,
                                telem.active_mode,
                                telem.faults,
                            );
                            ui.add_space(12.0);
                            self.render_dashboard_3d_preview(
                                ui,
                                Vec2::new(ui.available_width(), preview_h),
                            );
                        } else {
                            let gap = 14.0;
                            let left_w = ((ui.available_width() - gap) * 0.56).floor();
                            let right_w = (ui.available_width() - gap - left_w).max(260.0);
                            ui.horizontal_top(|ui| {
                                ui.spacing_mut().item_spacing.x = gap;
                                ui.allocate_ui_with_layout(
                                    Vec2::new(left_w, preview_h),
                                    Layout::top_down(Align::LEFT),
                                    |ui| {
                                        self.render_dashboard_stats_column(
                                            ui,
                                            volts,
                                            peak_ma,
                                            max_temp,
                                            active_torque_cnt,
                                            dof,
                                            pitch_deg,
                                            roll_deg,
                                            telem.active_mode,
                                            telem.faults,
                                        );
                                    },
                                );
                                self.render_dashboard_3d_preview(
                                    ui,
                                    Vec2::new(right_w, preview_h),
                                );
                            });
                        }

                        ui.add_space(12.0);
                        self.render_home_mcp_bar(ui);
                    },
                );
                ui.add_space(16.0);
            });
        });
    }

    #[allow(clippy::too_many_arguments)]
    fn launch_action_card(
        &mut self,
        ui: &mut egui::Ui,
        title: &str,
        subtitle: &str,
        desc: &str,
        fill: Color32,
        accent: Color32,
        size: Vec2,
        target: AppScreen,
    ) {
        let resp = egui::Frame::new()
            .fill(fill)
            .corner_radius(CornerRadius::same(10))
            .inner_margin(Margin::symmetric(18, 14))
            .stroke(Stroke::new(1.4, accent))
            .show(ui, |ui| {
                ui.set_min_size(Vec2::new((size.x - 36.0).max(120.0), (size.y - 28.0).max(60.0)));
                ui.set_max_size(Vec2::new((size.x - 36.0).max(120.0), (size.y - 28.0).max(60.0)));
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(title)
                                .strong()
                                .size(17.0)
                                .color(Color32::WHITE),
                        );
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            ui.label(
                                RichText::new("OPEN ->")
                                    .strong()
                                    .size(12.0)
                                    .color(accent),
                            );
                        });
                    });
                    ui.label(
                        RichText::new(subtitle)
                            .strong()
                            .size(12.5)
                            .color(accent),
                    );
                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(desc)
                            .size(12.0)
                            .color(Color32::from_rgb(212, 226, 244)),
                    );
                });
            })
            .response
            .interact(egui::Sense::click());

        if resp.hovered() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        if resp.clicked() {
            self.active_screen = target;
            if target == AppScreen::Control && !self.model.is_estop_active() {
                self.model.status_message =
                    "Control screen active. Switch to Drive to walk or test balance push recovery."
                        .to_string();
            } else if target == AppScreen::Config {
                self.model.status_message =
                    "Config & Calibration: Click/drag robot parts to pose; drag empty space or ViewCube to rotate view."
                        .to_string();
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn render_dashboard_stats_column(
        &mut self,
        ui: &mut egui::Ui,
        volts: f32,
        peak_ma: u16,
        max_temp: u8,
        active_torque_cnt: u32,
        dof: usize,
        pitch_deg: f32,
        roll_deg: f32,
        active_mode: ControlMode,
        faults: FaultFlags,
    ) {
        let gap = 10.0;
        let card_w = ((ui.available_width() - gap) * 0.5).floor().max(130.0);

        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = gap;
            let v_col = if faults.contains(FaultFlags::UNDER_VOLTAGE) {
                Color32::from_rgb(248, 113, 113)
            } else {
                Color32::from_rgb(74, 222, 128)
            };
            stat_card(
                ui,
                card_w,
                "POWER BUS",
                &format!("{volts:.2} V"),
                &format!("Peak {peak_ma} mA  ·  Max {max_temp} C"),
                v_col,
            );
            let mode_str = match active_mode {
                ControlMode::Teach => "TEACH (OFF)",
                ControlMode::Drive => "DRIVE (ON)",
                ControlMode::EStop => "E-STOP",
            };
            let mode_col = match active_mode {
                ControlMode::Teach => Color32::from_rgb(56, 189, 248),
                ControlMode::Drive => Color32::from_rgb(245, 158, 11),
                ControlMode::EStop => Color32::from_rgb(248, 113, 113),
            };
            stat_card(
                ui,
                card_w,
                "ACTUATOR AUTHORITY",
                mode_str,
                &format!("Torque {active_torque_cnt}/{dof} joints  ·  Speed {}%", self.model.speed_limit_pct),
                mode_col,
            );
        });

        ui.add_space(10.0);

        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = gap;
            let imu_avail = self.model.latest_telemetry.imu.available;
            let bal_col = if !imu_avail {
                Color32::from_rgb(148, 166, 190)
            } else if faults.contains(FaultFlags::TILT_FALL_CUTOFF) {
                Color32::from_rgb(248, 113, 113)
            } else if pitch_deg.abs() > 8.0 || roll_deg.abs() > 8.0 {
                Color32::from_rgb(251, 191, 36)
            } else {
                Color32::from_rgb(74, 222, 128)
            };
            let bal_val = if imu_avail {
                format!("P {pitch_deg:+.1}°  R {roll_deg:+.1}°")
            } else {
                "NO IMU".to_string()
            };
            let bal_state = if !imu_avail {
                "ALWAYS UPRIGHT (TOP-UP)"
            } else if faults.contains(FaultFlags::TILT_FALL_CUTOFF) {
                "FALL CUTOFF"
            } else if self.model.walk_cmd.balance_enabled {
                "REAL-WORLD IMU · MCU ON"
            } else {
                "REAL-WORLD IMU · BAL OFF"
            };
            stat_card(
                ui,
                card_w,
                "IMU ATTITUDE & BALANCE",
                &bal_val,
                bal_state,
                bal_col,
            );

            let fault_title = if faults.is_empty() {
                "ALL NOMINAL"
            } else {
                "FAULT ACTIVE"
            };
            let fault_col = if faults.is_empty() {
                Color32::from_rgb(74, 222, 128)
            } else {
                Color32::from_rgb(248, 113, 113)
            };
            let fault_sub = if faults.is_empty() {
                "Watchdog 100 ms OK · Limits Active".to_string()
            } else {
                format!("{:?}", faults)
            };
            stat_card(ui, card_w, "SAFETY & FAULTS", fault_title, &fault_sub, fault_col);
        });

        ui.add_space(10.0);

        // Quick MCU / Model summary bar
        egui::Frame::new()
            .fill(Color32::from_rgb(16, 22, 33))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::symmetric(14, 10))
            .stroke(Stroke::new(1.0, Color32::from_rgb(34, 46, 66)))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("MCU KERNEL:")
                            .strong()
                            .size(11.5)
                            .color(Color32::from_rgb(126, 148, 174)),
                    );
                    ui.label(
                        RichText::new(format!(
                            "UVC {} ({:.2}/{:.2})  ·  Walk {}  ·  Fall Cutoff {:.0}°",
                            rk_proto::uvc_phase::label(self.model.latest_telemetry.balance.uvc_phase),
                            self.model.balance_gains.uvc_gain_pitch,
                            self.model.balance_gains.uvc_gain_roll,
                            if self.model.walk_cmd.walk_enabled {
                                "ON"
                            } else {
                                "STANDBY"
                            },
                            self.model.balance_gains.fall_cutoff_rad.to_degrees(),
                        ))
                        .monospace()
                        .size(12.0)
                        .color(Color32::from_rgb(198, 214, 234)),
                    );
                });
            });
    }

    fn render_dashboard_3d_preview(&mut self, ui: &mut egui::Ui, size: Vec2) {
        let inner_size = Vec2::new((size.x - 12.0).max(160.0), (size.y - 12.0).max(160.0));
        egui::Frame::new()
            .fill(Color32::from_rgb(14, 19, 28))
            .corner_radius(CornerRadius::same(10))
            .inner_margin(Margin::same(6))
            .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 74)))
            .show(ui, |ui| {
                let (rect, _) = ui.allocate_exact_size(inner_size, egui::Sense::hover());
                let mut child = ui.new_child(
                    egui::UiBuilder::new()
                        .max_rect(rect)
                        .layout(Layout::top_down(Align::LEFT)),
                );
                child.set_clip_rect(rect);
                let telem = self.model.latest_telemetry;
                let opts = ViewportOptions {
                    show_ghost: telem.active_mode == ControlMode::Drive,
                    estop_active: self.model.is_estop_active(),
                    hud_blocked: true,
                    show_view_cube: false,
                    show_protractor: false,
                    interactive_drag: false,
                    imu_available: telem.imu.available,
                    imu_pitch_rad: telem.imu.pitch_rad,
                    imu_roll_rad: telem.imu.roll_rad,
                    fall_event_active: telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                        && telem.imu.available,
                };
                let _ = self.viewport.show(
                    &mut child,
                    self.last_dt_sec,
                    &self.model.rig,
                    &self.model.bundle,
                    &telem.measured_rad,
                    &self.model.target_rad,
                    &self.model.calibration,
                    &telem.current_ma,
                    self.model.selected_joint,
                    opts,
                );
            });
    }
}

fn stat_card(
    ui: &mut egui::Ui,
    width: f32,
    label: &str,
    value: &str,
    detail: &str,
    accent: Color32,
) {
    egui::Frame::new()
        .fill(Color32::from_rgb(16, 22, 33))
        .corner_radius(CornerRadius::same(8))
        .inner_margin(Margin::symmetric(14, 12))
        .stroke(Stroke::new(1.0, Color32::from_rgb(36, 49, 70)))
        .show(ui, |ui| {
            ui.set_min_size(Vec2::new((width - 28.0).max(100.0), 82.0));
            ui.set_max_size(Vec2::new((width - 28.0).max(100.0), 88.0));
            ui.vertical(|ui| {
                ui.label(
                    RichText::new(label)
                        .strong()
                        .size(10.5)
                        .color(Color32::from_rgb(130, 150, 176)),
                );
                ui.add_space(4.0);
                ui.label(RichText::new(value).strong().monospace().size(18.0).color(accent));
                ui.add_space(4.0);
                ui.label(
                    RichText::new(detail)
                        .size(11.5)
                        .color(Color32::from_rgb(160, 178, 200)),
                );
            });
        });
}
