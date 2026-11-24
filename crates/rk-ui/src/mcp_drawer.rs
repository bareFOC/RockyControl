//! Home-screen MCP Access toggle bar and slide-over MCP Agent Drawer.

use egui::{Align, Color32, CornerRadius, Layout, Margin, Rect, RichText, Stroke, Vec2, pos2, vec2};
use rk_mcp::McpCallOutcome;
use serde_json::json;

use crate::RockyStudioApp;
use crate::mcp_bridge::McpSnippetTab;

const QUICK_TEST_PRESETS: &[(&str, &str)] = &[
    ("telemetry_snapshot", "Read Telemetry"),
    ("robot_connect", "Connect Sim"),
    ("balance_get_gains", "Get UVC Gains"),
    ("mode_set_drive", "Try Drive Mode"),
    ("sim_push_shove", "Push Shove"),
    ("safety_estop", "Trigger E-Stop"),
];

impl RockyStudioApp {
    /// Compact MCP Agent Access toggle bar displayed on the Dashboard Home screen
    /// (both disconnected and connected views). MCP is enabled by default with zero auth.
    pub(crate) fn render_home_mcp_bar(&mut self, ui: &mut egui::Ui) {
        let enabled = self.mcp.policy.enabled;
        let armed = self.mcp.policy.drive_armed;
        let (status_label, status_col, border_col) = if !enabled {
            (
                "MCP OFF",
                Color32::from_rgb(148, 163, 184),
                Color32::from_rgb(44, 56, 74),
            )
        } else if armed {
            (
                "MCP ARMED",
                Color32::from_rgb(251, 191, 36),
                Color32::from_rgb(180, 120, 24),
            )
        } else {
            (
                "MCP ON",
                Color32::from_rgb(56, 208, 248),
                Color32::from_rgb(32, 96, 124),
            )
        };

        egui::Frame::new()
            .fill(Color32::from_rgb(16, 22, 33))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::symmetric(14, 10))
            .stroke(Stroke::new(1.0, border_col))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 8.0;
                            let (dot_rect, _) =
                                ui.allocate_exact_size(Vec2::splat(10.0), egui::Sense::hover());
                            ui.painter().circle_filled(dot_rect.center(), 4.2, status_col);

                            ui.label(
                                RichText::new("MCP AGENT ACCESS")
                                    .strong()
                                    .size(11.5)
                                    .color(Color32::from_rgb(220, 232, 248)),
                            );

                            egui::Frame::new()
                                .fill(Color32::from_rgb(22, 30, 44))
                                .corner_radius(CornerRadius::same(4))
                                .inner_margin(Margin::symmetric(7, 2))
                                .stroke(Stroke::new(1.0, border_col))
                                .show(ui, |ui| {
                                    ui.label(
                                        RichText::new(status_label)
                                            .strong()
                                            .monospace()
                                            .size(10.5)
                                            .color(status_col),
                                    );
                                });
                        });

                        let sub = if !enabled {
                            "Autonomous agent tools disabled".to_string()
                        } else {
                            format!(
                                "{} · {}",
                                self.mcp.endpoint_status(),
                                if armed {
                                    "Drive ARMED (E-Stop overrides)"
                                } else {
                                    "Safe Teach mode (no auth required)"
                                }
                            )
                        };
                        ui.label(
                            RichText::new(sub)
                                .size(11.5)
                                .color(Color32::from_rgb(142, 162, 188)),
                        );
                    });

                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let panel_btn = egui::Button::new(
                            RichText::new(if self.mcp.drawer_open {
                                "Close Panel"
                            } else {
                                "Open MCP Panel"
                            })
                            .strong()
                            .size(11.5)
                            .color(Color32::from_rgb(212, 232, 250)),
                        )
                        .fill(Color32::from_rgb(24, 34, 50))
                        .stroke(Stroke::new(1.0, Color32::from_rgb(52, 74, 102)))
                        .min_size(Vec2::new(108.0, 30.0))
                        .corner_radius(CornerRadius::same(5));
                        if ui.add(panel_btn).clicked() {
                            self.mcp.drawer_open = !self.mcp.drawer_open;
                        }

                        let toggle_label = if enabled { "Disable MCP" } else { "Enable MCP" };
                        let toggle_btn = egui::Button::new(
                            RichText::new(toggle_label)
                                .strong()
                                .size(11.5)
                                .color(if enabled {
                                    Color32::from_rgb(248, 200, 200)
                                } else {
                                    Color32::WHITE
                                }),
                        )
                        .fill(if enabled {
                            Color32::from_rgb(56, 26, 32)
                        } else {
                            Color32::from_rgb(14, 116, 144)
                        })
                        .stroke(Stroke::new(
                            1.0,
                            if enabled {
                                Color32::from_rgb(138, 56, 66)
                            } else {
                                Color32::from_rgb(56, 208, 248)
                            },
                        ))
                        .min_size(Vec2::new(92.0, 30.0))
                        .corner_radius(CornerRadius::same(5));
                        if ui.add(toggle_btn).clicked() {
                            let next = !enabled;
                            self.mcp.set_enabled(next);
                            self.model.status_message = if next {
                                format!("MCP Agent Access enabled · {}.", self.mcp.endpoint_status())
                            } else {
                                "MCP Agent Access disabled. External agent calls will be rejected."
                                    .to_string()
                            };
                        }
                    });
                });
            });
    }

    /// Slide-over right-hand MCP Agent Drawer (380 px on desktop, responsive on narrow viewports).
    pub(crate) fn render_mcp_drawer(&mut self, ctx: &egui::Context) {
        if !self.mcp.drawer_open {
            self.mcp.drawer_rect = Rect::NOTHING;
            return;
        }

        let screen = ctx.content_rect();
        let top_y = self.ui_metrics.top_bar_height + 2.0;
        let bottom_y = (screen.bottom() - self.ui_metrics.status_bar_height - 2.0).max(top_y + 120.0);
        let drawer_w = 384.0_f32.min((screen.width() - 20.0).max(280.0));
        let drawer_h = (bottom_y - top_y).max(200.0);
        let drawer_rect = Rect::from_min_size(
            pos2(screen.right() - drawer_w, top_y),
            vec2(drawer_w, drawer_h),
        );
        self.mcp.drawer_rect = drawer_rect;

        egui::Area::new(egui::Id::unique("rocky_mcp_slideover_drawer"))
            .order(egui::Order::Foreground)
            .fixed_pos(drawer_rect.min)
            .interactable(true)
            .show(ctx, |ui| {
                egui::Frame::new()
                    .fill(Color32::from_rgb(14, 19, 29))
                    .stroke(Stroke::new(1.2, Color32::from_rgb(44, 62, 88)))
                    .inner_margin(Margin::same(14))
                    .show(ui, |ui| {
                        ui.set_min_size(vec2(drawer_w - 28.0, drawer_h - 28.0));
                        ui.set_max_size(vec2(drawer_w - 28.0, drawer_h - 28.0));

                        // 1. Header
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new("MCP AGENT CONTROL")
                                    .strong()
                                    .size(14.0)
                                    .color(Color32::from_rgb(232, 244, 255)),
                            );
                            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                if ui.small_button("Close").clicked() {
                                    self.mcp.drawer_open = false;
                                }
                                let bridge_tag = if self.mcp.bridge_connected {
                                    ("BRIDGE LIVE", Color32::from_rgb(74, 222, 128))
                                } else {
                                    ("LOCAL", Color32::from_rgb(148, 166, 190))
                                };
                                ui.label(
                                    RichText::new(bridge_tag.0)
                                        .strong()
                                        .monospace()
                                        .size(10.0)
                                        .color(bridge_tag.1),
                                );
                            });
                        });
                        ui.label(
                            RichText::new(format!(
                                "craft-mcp-server (MCP 2025-06-18) · {} · No auth required",
                                self.mcp.endpoint_status()
                            ))
                            .size(11.0)
                            .color(Color32::from_rgb(138, 158, 184)),
                        );

                        ui.add_space(10.0);

                        // Scrollable interior for Controls + Setup Snippets + Quick Test + Live Log
                        egui::ScrollArea::vertical()
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                // 2. Policy Switches
                                egui::Frame::new()
                                    .fill(Color32::from_rgb(19, 26, 38))
                                    .corner_radius(CornerRadius::same(7))
                                    .inner_margin(Margin::same(10))
                                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 74)))
                                    .show(ui, |ui| {
                                        ui.set_width(ui.available_width());
                                        let mut enabled = self.mcp.policy.enabled;
                                        if ui
                                            .checkbox(
                                                &mut enabled,
                                                RichText::new("Enable MCP Agent Access")
                                                    .strong()
                                                    .size(12.5)
                                                    .color(Color32::from_rgb(228, 240, 252)),
                                            )
                                            .changed()
                                        {
                                            self.mcp.set_enabled(enabled);
                                        }
                                        ui.label(
                                            RichText::new(
                                                "Allows agents to inspect telemetry, load URDFs, calibrate joints, tune UVC balance, and pose in Teach mode.",
                                            )
                                            .size(11.0)
                                            .color(Color32::from_rgb(142, 162, 188)),
                                        );

                                        ui.add_space(8.0);
                                        ui.add_enabled_ui(self.mcp.policy.enabled, |ui| {
                                            let mut armed = self.mcp.policy.drive_armed;
                                            let arm_label = if armed {
                                                "Allow Agent Drive (ARMED)"
                                            } else {
                                                "Allow Agent Drive (Disarmed)"
                                            };
                                            let arm_color = if armed {
                                                Color32::from_rgb(251, 191, 36)
                                            } else {
                                                Color32::from_rgb(186, 204, 226)
                                            };
                                            if ui
                                                .checkbox(
                                                    &mut armed,
                                                    RichText::new(arm_label)
                                                        .strong()
                                                        .size(12.5)
                                                        .color(arm_color),
                                                )
                                                .changed()
                                            {
                                                self.mcp.set_drive_armed(armed);
                                            }
                                            ui.label(
                                                RichText::new(
                                                    "Guarded Autonomy: Required for energized Drive mode, walking velocity, and combat moves. E-Stop always cuts torque & disarms.",
                                                )
                                                .size(11.0)
                                                .color(Color32::from_rgb(142, 162, 188)),
                                            );
                                        });
                                    });

                                ui.add_space(10.0);

                                // 3. Client Setup Snippets
                                ui.horizontal(|ui| {
                                    ui.label(
                                        RichText::new("AGENT SETUP SNIPPETS")
                                            .strong()
                                            .size(11.0)
                                            .color(Color32::from_rgb(110, 200, 232)),
                                    );
                                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                        let copied = self.sim_time_sec < self.mcp.copied_notice_until;
                                        let copy_lbl = if copied { "Copied" } else { "Copy" };
                                        if ui.small_button(copy_lbl).clicked() {
                                            let snippet = build_snippet(
                                                self.mcp.snippet_tab,
                                                &self.mcp.http_url(),
                                            );
                                            ui.ctx().copy_text(snippet);
                                            self.mcp.copied_notice_until = self.sim_time_sec + 2.0;
                                            self.model.status_message =
                                                "Copied MCP setup snippet to clipboard.".to_string();
                                        }
                                    });
                                });
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    for tab in [
                                        McpSnippetTab::Http,
                                        McpSnippetTab::ClaudeCursor,
                                        McpSnippetTab::GeminiCurl,
                                    ] {
                                        let active = self.mcp.snippet_tab == tab;
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
                                        .corner_radius(CornerRadius::same(4));
                                        if ui.add(btn).clicked() {
                                            self.mcp.snippet_tab = tab;
                                        }
                                    }
                                });
                                ui.add_space(4.0);
                                let snippet_text =
                                    build_snippet(self.mcp.snippet_tab, &self.mcp.http_url());
                                egui::Frame::new()
                                    .fill(Color32::from_rgb(10, 14, 22))
                                    .corner_radius(CornerRadius::same(6))
                                    .inner_margin(Margin::same(8))
                                    .stroke(Stroke::new(1.0, Color32::from_rgb(32, 44, 62)))
                                    .show(ui, |ui| {
                                        ui.set_width(ui.available_width());
                                        ui.label(
                                            RichText::new(snippet_text)
                                                .monospace()
                                                .size(10.5)
                                                .color(Color32::from_rgb(190, 220, 245)),
                                        );
                                    });

                                ui.add_space(12.0);

                                // 4. Quick Test Tool Runner
                                ui.label(
                                    RichText::new("QUICK TEST CALL")
                                        .strong()
                                        .size(11.0)
                                        .color(Color32::from_rgb(110, 200, 232)),
                                );
                                ui.add_space(4.0);
                                ui.horizontal_wrapped(|ui| {
                                    ui.spacing_mut().item_spacing = Vec2::new(6.0, 6.0);
                                    for (idx, (_key, label)) in QUICK_TEST_PRESETS.iter().enumerate() {
                                        let active = self.mcp.test_preset_idx == idx;
                                        let btn = egui::Button::new(
                                            RichText::new(*label)
                                                .size(11.0)
                                                .color(if active {
                                                    Color32::WHITE
                                                } else {
                                                    Color32::from_rgb(168, 186, 208)
                                                }),
                                        )
                                        .fill(if active {
                                            Color32::from_rgb(20, 96, 124)
                                        } else {
                                            Color32::from_rgb(22, 30, 44)
                                        })
                                        .corner_radius(CornerRadius::same(4));
                                        if ui.add(btn).clicked() {
                                            self.mcp.test_preset_idx = idx;
                                            self.execute_quick_test_preset(idx);
                                        }
                                    }
                                });
                                if !self.mcp.last_test_output.is_empty() {
                                    ui.add_space(6.0);
                                    egui::Frame::new()
                                        .fill(Color32::from_rgb(10, 14, 22))
                                        .corner_radius(CornerRadius::same(6))
                                        .inner_margin(Margin::same(8))
                                        .stroke(Stroke::new(1.0, Color32::from_rgb(32, 44, 62)))
                                        .show(ui, |ui| {
                                            ui.set_width(ui.available_width());
                                            let preview: String = self
                                                .mcp
                                                .last_test_output
                                                .chars()
                                                .take(420)
                                                .collect();
                                            ui.label(
                                                RichText::new(preview)
                                                    .monospace()
                                                    .size(10.0)
                                                    .color(Color32::from_rgb(186, 208, 230)),
                                            );
                                        });
                                }

                                ui.add_space(12.0);

                                // 5. Live Activity Log
                                ui.horizontal(|ui| {
                                    ui.label(
                                        RichText::new(format!(
                                            "LIVE ACTIVITY LOG ({})",
                                            self.mcp.activity.len()
                                        ))
                                        .strong()
                                        .size(11.0)
                                        .color(Color32::from_rgb(110, 200, 232)),
                                    );
                                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                        if !self.mcp.activity.is_empty()
                                            && ui.small_button("Clear").clicked()
                                        {
                                            self.mcp.activity.clear();
                                        }
                                    });
                                });
                                ui.add_space(4.0);
                                if self.mcp.activity.is_empty() {
                                    ui.label(
                                        RichText::new(
                                            "No MCP calls yet. Connect an external agent to /mcp or click a Quick Test Call above.",
                                        )
                                        .size(11.0)
                                        .color(Color32::from_rgb(122, 140, 164)),
                                    );
                                } else {
                                    for entry in self.mcp.activity.iter().rev().take(24) {
                                        let badge_col = match entry.outcome {
                                            McpCallOutcome::Ok => Color32::from_rgb(74, 222, 128),
                                            McpCallOutcome::Guarded => {
                                                Color32::from_rgb(251, 191, 36)
                                            }
                                            McpCallOutcome::Error => {
                                                Color32::from_rgb(248, 113, 113)
                                            }
                                        };
                                        egui::Frame::new()
                                            .fill(Color32::from_rgb(18, 24, 36))
                                            .corner_radius(CornerRadius::same(5))
                                            .inner_margin(Margin::symmetric(8, 5))
                                            .stroke(Stroke::new(
                                                1.0,
                                                Color32::from_rgb(30, 42, 60),
                                            ))
                                            .show(ui, |ui| {
                                                ui.set_width(ui.available_width());
                                                ui.horizontal(|ui| {
                                                    ui.label(
                                                        RichText::new(format!("#{}", entry.seq))
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(Color32::from_rgb(
                                                                118, 136, 160,
                                                            )),
                                                    );
                                                    ui.label(
                                                        RichText::new(entry.outcome.label())
                                                            .strong()
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(badge_col),
                                                    );
                                                    ui.label(
                                                        RichText::new(&entry.method)
                                                            .strong()
                                                            .monospace()
                                                            .size(10.5)
                                                            .color(Color32::from_rgb(
                                                                220, 234, 248,
                                                            )),
                                                    );
                                                });
                                                if !entry.detail.is_empty() {
                                                    ui.label(
                                                        RichText::new(&entry.detail)
                                                            .size(10.5)
                                                            .color(Color32::from_rgb(
                                                                152, 172, 196,
                                                            )),
                                                    );
                                                }
                                            });
                                        ui.add_space(4.0);
                                    }
                                }
                            });
                    });
            });
    }

    fn execute_quick_test_preset(&mut self, idx: usize) {
        match idx {
            0 => self.run_inline_mcp_tool("telemetry_snapshot", json!({})),
            1 => self.run_inline_mcp_tool("robot_connect", json!({ "transport": "sim" })),
            2 => self.run_inline_mcp_tool("balance_get_gains", json!({})),
            3 => self.run_inline_mcp_tool("mode_set", json!({ "mode": "drive" })),
            4 => self.run_inline_mcp_tool(
                "sim_push_shove",
                json!({
                    "pitch_impulse_rad_s": self.push_strength.impulse_rad_s(),
                    "roll_impulse_rad_s": -0.9,
                    "step_ticks": 260,
                }),
            ),
            5 => self.run_inline_mcp_tool("safety_estop", json!({})),
            _ => {}
        }
    }
}

fn build_snippet(tab: McpSnippetTab, http_url: &str) -> String {
    match tab {
        McpSnippetTab::Http => format!(
            "{{\n  \"mcpServers\": {{\n    \"rocky-studio\": {{\n      \"type\": \"streamable-http\",\n      \"url\": \"{http_url}\"\n    }}\n  }}\n}}"
        ),
        McpSnippetTab::ClaudeCursor => format!(
            "// Native stdio (Claude Desktop / Cursor):\n{{\n  \"mcpServers\": {{\n    \"rocky-studio\": {{\n      \"command\": \"rk-mcp\",\n      \"args\": [\"--transport\", \"sim\"]\n    }}\n  }}\n}}\n// Or live browser sync HTTP:\n// url: {http_url}"
        ),
        McpSnippetTab::GeminiCurl => format!(
            "curl -s {http_url} \\\n  -H \"Content-Type: application/json\" \\\n  -d '{{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"tools/list\",\"params\":{{}}}}'"
        ),
    }
}
