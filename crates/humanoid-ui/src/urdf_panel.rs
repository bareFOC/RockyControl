//! URDF Upload, Kinematics Tree Inspector, Paste-XML Modal & MCU Onboard Model Sync UI.

use egui::{Color32, CornerRadius, RichText, Stroke};
use humanoid_model::{AssetBundle, ModelSource, UpAxis, samples};
use humanoid_proto::{MODEL_CHUNK_BYTES, ModelFormat, ModelInfo, ModelRequest, ModelResponse};

use crate::RockyStudioApp;

/// RP2350 Pico 2 reference flash capacity (4 MiB) for footprint budgeting readout.
const RP2350_FLASH_BYTES: f32 = 4.0 * 1024.0 * 1024.0;

#[derive(Clone, Debug, Default)]
pub struct McuModelSyncState {
    pub remote_info: Option<ModelInfo>,
    pub pulling: bool,
    pub pull_target_info: Option<ModelInfo>,
    pub pull_buffer: Vec<u8>,
    pub chunks_received: u32,
    pub last_message: String,
    pub stage_deflated: bool,
    pub queried_once: bool,
}

impl McuModelSyncState {
    pub fn progress(&self) -> f32 {
        let Some(info) = self.pull_target_info else {
            return 0.0;
        };
        if info.total_bytes == 0 {
            return 1.0;
        }
        (self.pull_buffer.len() as f32 / info.total_bytes as f32).clamp(0.0, 1.0)
    }
}

impl RockyStudioApp {
    /// Queries `ModelInfo` from the active transport (Sim MCU or physical RP2350 UDP :7778).
    pub fn query_mcu_model_info(&mut self) {
        self.mcu_sync.queried_once = true;
        match self.transport.exchange_model(ModelRequest::GetInfo) {
            Some(ModelResponse::Info(info)) => {
                self.mcu_sync.remote_info = Some(info);
                self.mcu_sync.last_message = format!(
                    "MCU flash model detected: '{}' ({} bytes, CRC32 {:#010x})",
                    info.name_str(),
                    info.total_bytes,
                    info.crc32
                );
            }
            Some(ModelResponse::NotAvailable) => {
                self.mcu_sync.remote_info = None;
                self.mcu_sync.last_message = "MCU reports no onboard URDF model stored.".to_string();
            }
            _ => {
                self.mcu_sync.last_message =
                    "No response from MCU Model Service (port 7778 / Sim).".to_string();
            }
        }
    }

    /// Starts a chunked CRC32-verified URDF download from the connected MCU.
    pub fn start_pull_mcu_model(&mut self) {
        self.query_mcu_model_info();
        let Some(info) = self.mcu_sync.remote_info else {
            self.model.status_message = self.mcu_sync.last_message.clone();
            return;
        };
        if info.total_bytes == 0 {
            self.model.status_message = "MCU flash model is empty (0 bytes).".to_string();
            return;
        }
        self.mcu_sync.pulling = true;
        self.mcu_sync.pull_target_info = Some(info);
        self.mcu_sync.pull_buffer = Vec::with_capacity(info.total_bytes as usize);
        self.mcu_sync.chunks_received = 0;
        self.mcu_sync.last_message = format!(
            "Pulling '{}' ({} bytes) from MCU flash...",
            info.name_str(),
            info.total_bytes
        );
        self.model.status_message = self.mcu_sync.last_message.clone();
    }

    /// Advances active chunked MCU model download (up to 4 chunks per UI frame for responsive progress).
    pub fn step_mcu_model_pull(&mut self) {
        if !self.mcu_sync.pulling {
            return;
        }
        let Some(info) = self.mcu_sync.pull_target_info else {
            self.mcu_sync.pulling = false;
            return;
        };

        for _ in 0..4 {
            let offset = self.mcu_sync.pull_buffer.len() as u32;
            if offset >= info.total_bytes {
                break;
            }
            let req = ModelRequest::ReadChunk {
                offset,
                max_len: MODEL_CHUNK_BYTES as u16,
            };
            match self.transport.exchange_model(req) {
                Some(ModelResponse::Chunk(chunk)) => {
                    if chunk.offset != offset || chunk.len == 0 {
                        self.mcu_sync.pulling = false;
                        self.mcu_sync.last_message =
                            format!("MCU chunk framing error at offset {offset}.");
                        self.model.status_message = self.mcu_sync.last_message.clone();
                        return;
                    }
                    self.mcu_sync.pull_buffer.extend_from_slice(chunk.payload());
                    self.mcu_sync.chunks_received = self.mcu_sync.chunks_received.saturating_add(1);
                }
                _ => {
                    self.mcu_sync.pulling = false;
                    self.mcu_sync.last_message =
                        format!("MCU model transfer timed out at offset {offset}.");
                    self.model.status_message = self.mcu_sync.last_message.clone();
                    return;
                }
            }
        }

        if self.mcu_sync.pull_buffer.len() >= info.total_bytes as usize {
            self.mcu_sync.pulling = false;
            let bytes = std::mem::take(&mut self.mcu_sync.pull_buffer);
            let chunks = self.mcu_sync.chunks_received;
            match AssetBundle::from_mcu_blob(&info, &bytes) {
                Ok(bundle) => {
                    self.apply_loaded_bundle(bundle);
                    self.mcu_sync.last_message = format!(
                        "Loaded '{}' from MCU flash ({} chunks, CRC32 {:#010x} verified).",
                        info.name_str(),
                        chunks,
                        info.crc32
                    );
                    self.model.status_message = self.mcu_sync.last_message.clone();
                }
                Err(err) => {
                    self.mcu_sync.last_message = err.clone();
                    self.model.status_message = err;
                }
            }
        }
    }

    /// Stages the currently loaded URDF into the simulated MCU flash store (raw XML or zlib deflate).
    pub fn stage_active_urdf_to_sim_mcu(&mut self) {
        let (format, blob) = if self.mcu_sync.stage_deflated {
            match self.model.bundle.to_deflated_urdf_bytes() {
                Ok(bytes) => (ModelFormat::UrdfDeflate, bytes),
                Err(err) => {
                    self.model.status_message = err;
                    return;
                }
            }
        } else {
            (
                ModelFormat::UrdfXml,
                self.model.bundle.urdf_xml.as_bytes().to_vec(),
            )
        };

        let name = self.model.rig.robot_name.clone();
        let joints = self.model.rig.joints.len().min(u8::MAX as usize) as u8;
        let len = blob.len();
        if self
            .transport
            .stage_sim_mcu_model(&name, format, blob.clone(), joints)
        {
            self.staged_sim_model = Some((name.clone(), format, blob, joints));
            self.query_mcu_model_info();
            self.model.status_message = format!(
                "Staged '{}' ({} bytes, {}) into simulated MCU flash.",
                name,
                len,
                format.label()
            );
        } else {
            self.model.status_message =
                "Staging to flash is only available on the Software Sim transport.".to_string();
        }
    }

    /// Docked Panel — URDF Upload, Kinematics Tree Inspector & MCU Onboard Model Storage
    pub fn render_urdf_mcu_panel(&mut self, ui: &mut egui::Ui) {
        let diag = self.model.bundle.diagnostics(&self.model.rig);
        let source_label = self.model.bundle.source.label();
        let dims = [
            self.model.rig.bounds_max[0] - self.model.rig.bounds_min[0],
            self.model.rig.bounds_max[1] - self.model.rig.bounds_min[1],
            self.model.rig.bounds_max[2] - self.model.rig.bounds_min[2],
        ];

        egui::ScrollArea::vertical()
            .id_salt("urdf_mcu_panel_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                // 1. URDF Upload & Sample Presets Card
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("URDF Robot Loader (3D Canvas)")
                                .strong()
                                .size(13.0)
                                .color(Color32::from_rgb(103, 232, 249)),
                        );
                        ui.label(
                            RichText::new(
                                "Upload a .urdf/.xml file or a .zip bundle with STL/OBJ meshes, or drag-and-drop directly onto the 3D canvas.",
                            )
                            .size(11.5)
                            .color(Color32::from_rgb(148, 163, 184)),
                        );
                        ui.add_space(6.0);

                        ui.horizontal_wrapped(|ui| {
                            let upload_btn = egui::Button::new(
                                RichText::new("Upload .urdf / .zip")
                                    .strong()
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(14, 116, 144));
                            if ui.add(upload_btn).clicked() {
                                if let Err(err) = self.saver.request_open_urdf_file() {
                                    self.model.status_message = err;
                                }
                            }
                            if ui.button("Paste URDF XML").clicked() {
                                self.paste_urdf_buf = self.model.bundle.urdf_xml.clone();
                                self.show_paste_urdf_dialog = true;
                            }
                            if ui.button("Export Active .urdf").clicked() {
                                let fname = format!("{}.urdf", self.model.rig.robot_name);
                                match self.saver.save_file(
                                    &fname,
                                    "application/xml",
                                    self.model.bundle.urdf_xml.as_bytes(),
                                ) {
                                    Ok(msg) => self.model.status_message = msg,
                                    Err(err) => self.model.status_message = err,
                                }
                            }
                        });

                        ui.add_space(6.0);
                        ui.label(
                            RichText::new("Quick Sample Robots")
                                .size(11.5)
                                .color(Color32::from_rgb(156, 172, 194)),
                        );
                        let sample_h = if self.ui_metrics.touch_mode { 36.0 } else { 22.0 };
                        let sample_sz = if self.ui_metrics.touch_mode { 12.5 } else { 11.5 };
                        ui.horizontal_wrapped(|ui| {
                            if ui
                                .add(
                                    egui::Button::new(
                                        RichText::new("Rocky Mk1 (17 DoF)").size(sample_sz),
                                    )
                                    .min_size(egui::vec2(0.0, sample_h)),
                                )
                                .clicked()
                            {
                                self.apply_loaded_bundle(AssetBundle::bundled_rocky());
                            }
                            if ui
                                .add(
                                    egui::Button::new(
                                        RichText::new("Striker Biped (8 DoF Z-Up)").size(sample_sz),
                                    )
                                    .min_size(egui::vec2(0.0, sample_h)),
                                )
                                .clicked()
                            {
                                match AssetBundle::from_sample(
                                    "Striker Biped Z-Up",
                                    samples::STRIKER_BIPED_ZUP_URDF,
                                ) {
                                    Ok(b) => self.apply_loaded_bundle(b),
                                    Err(e) => self.model.status_message = e,
                                }
                            }
                            if ui
                                .add(
                                    egui::Button::new(
                                        RichText::new("Combat Arm (6 DoF Prismatic)")
                                            .size(sample_sz),
                                    )
                                    .min_size(egui::vec2(0.0, sample_h)),
                                )
                                .clicked()
                            {
                                match AssetBundle::from_sample(
                                    "Combat Arm 6-DoF",
                                    samples::COMBAT_ARM_6DOF_URDF,
                                ) {
                                    Ok(b) => self.apply_loaded_bundle(b),
                                    Err(e) => self.model.status_message = e,
                                }
                            }
                        });

                        ui.add_space(8.0);
                        ui.separator();
                        ui.add_space(4.0);

                        egui::Grid::new("urdf_meta_grid")
                            .num_columns(2)
                            .spacing([12.0, 3.0])
                            .show(ui, |ui| {
                                ui.label(
                                    RichText::new("Robot")
                                        .size(11.5)
                                        .color(Color32::from_rgb(148, 163, 184)),
                                );
                                ui.label(
                                    RichText::new(&self.model.rig.robot_name)
                                        .monospace()
                                        .strong()
                                        .size(12.0),
                                );
                                ui.end_row();

                                ui.label(
                                    RichText::new("Source")
                                        .size(11.5)
                                        .color(Color32::from_rgb(148, 163, 184)),
                                );
                                ui.label(RichText::new(&source_label).size(11.5));
                                ui.end_row();

                                ui.label(
                                    RichText::new("Topology")
                                        .size(11.5)
                                        .color(Color32::from_rgb(148, 163, 184)),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "{} links · {} movable DoF · {} visuals",
                                        diag.total_links, diag.movable_joints, diag.visual_count
                                    ))
                                    .monospace()
                                    .size(11.5),
                                );
                                ui.end_row();

                                ui.label(
                                    RichText::new("Bounds (m)")
                                        .size(11.5)
                                        .color(Color32::from_rgb(148, 163, 184)),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "W {:.2} × H {:.2} × D {:.2}  ({})",
                                        dims[0],
                                        dims[1],
                                        dims[2],
                                        self.model.rig.up_axis.label()
                                    ))
                                    .monospace()
                                    .size(11.5),
                                );
                                ui.end_row();

                                ui.label(
                                    RichText::new("URDF Size")
                                        .size(11.5)
                                        .color(Color32::from_rgb(148, 163, 184)),
                                );
                                let deflated_est = self
                                    .model
                                    .bundle
                                    .to_deflated_urdf_bytes()
                                    .map(|b| b.len())
                                    .unwrap_or(diag.urdf_bytes);
                                ui.label(
                                    RichText::new(format!(
                                        "{:.1} KB raw · {:.1} KB deflate · CRC32 {:#010x}",
                                        diag.urdf_bytes as f32 / 1024.0,
                                        deflated_est as f32 / 1024.0,
                                        diag.crc32
                                    ))
                                    .monospace()
                                    .size(11.5),
                                );
                                ui.end_row();
                            });

                        if !diag.missing_meshes.is_empty() {
                            ui.add_space(4.0);
                            ui.label(
                                RichText::new(format!(
                                    "Missing CAD meshes ({}): upload a .zip containing these STL/OBJ files:",
                                    diag.missing_meshes.len()
                                ))
                                .size(11.5)
                                .color(Color32::from_rgb(251, 191, 36)),
                            );
                            for m in diag.missing_meshes.iter().take(4) {
                                ui.label(
                                    RichText::new(format!("  - {m}"))
                                        .monospace()
                                        .size(11.0)
                                        .color(Color32::from_rgb(254, 243, 199)),
                                );
                            }
                        } else if !diag.resolved_meshes.is_empty() {
                            ui.add_space(4.0);
                            ui.label(
                                RichText::new(format!(
                                    "Loaded {} CAD mesh file(s) from archive.",
                                    diag.resolved_meshes.len()
                                ))
                                .size(11.5)
                                .color(Color32::from_rgb(134, 239, 172)),
                            );
                        }
                    });

                ui.add_space(8.0);

                // 2. MCU Onboard Kinematics Storage Card
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("MCU Onboard Kinematics Storage (RP2350 / ESP32-P4 Flash)")
                                .strong()
                                .size(13.0)
                                .color(Color32::from_rgb(134, 239, 172)),
                        );
                        ui.label(
                            RichText::new(
                                "Stores the URDF in MCU XIP flash (0 B SRAM) and streams 256 B COBS chunks on UDP :7778 / Sim so any Rocky Studio loads the robot model automatically.",
                            )
                            .size(11.5)
                            .color(Color32::from_rgb(148, 163, 184)),
                        );
                        ui.add_space(6.0);

                        if let Some(info) = self.mcu_sync.remote_info {
                            let flash_pct = (info.total_bytes as f32 / RP2350_FLASH_BYTES) * 100.0;
                            let chunks =
                                (info.total_bytes as usize).div_ceil(MODEL_CHUNK_BYTES);
                            let in_sync = matches!(
                                self.model.bundle.source,
                                ModelSource::McuFlash { crc32, .. } if crc32 == info.crc32
                            ) || diag.crc32 == info.crc32;

                            ui.label(
                                RichText::new(format!(
                                    "Flash Blob: '{}' · {} DoF · {} B ({:.3}% of 4 MB) · {} chunks",
                                    info.name_str(),
                                    info.joint_count,
                                    info.total_bytes,
                                    flash_pct,
                                    chunks
                                ))
                                .monospace()
                                .size(11.5)
                                .color(Color32::from_rgb(212, 224, 240)),
                            );
                            ui.label(
                                RichText::new(format!(
                                    "Format: {} · CRC32: {:#010x} · {}",
                                    info.format.label(),
                                    info.crc32,
                                    if in_sync {
                                        "SYNCED WITH CANVAS"
                                    } else {
                                        "DIFFERS FROM CANVAS"
                                    }
                                ))
                                .monospace()
                                .size(11.0)
                                .color(if in_sync {
                                    Color32::from_rgb(134, 239, 172)
                                } else {
                                    Color32::from_rgb(251, 191, 36)
                                }),
                            );
                        } else {
                            ui.label(
                                RichText::new("MCU flash info not queried yet.")
                                    .size(11.5)
                                    .color(Color32::from_rgb(148, 163, 184)),
                            );
                        }

                        ui.add_space(6.0);
                        ui.horizontal_wrapped(|ui| {
                            let pull_btn = egui::Button::new(
                                RichText::new("Pull Model from MCU")
                                    .strong()
                                    .color(Color32::from_rgb(14, 18, 24)),
                            )
                            .fill(Color32::from_rgb(134, 239, 172));
                            if ui.add_enabled(!self.mcu_sync.pulling, pull_btn).clicked() {
                                self.start_pull_mcu_model();
                            }
                            if ui.button("Query MCU Info").clicked() {
                                self.query_mcu_model_info();
                            }
                        });

                        if self.transport.sim_faults().is_some() {
                            ui.add_space(4.0);
                            ui.horizontal_wrapped(|ui| {
                                if ui.button("Stage Active URDF to Sim MCU").clicked() {
                                    self.stage_active_urdf_to_sim_mcu();
                                }
                                ui.checkbox(
                                    &mut self.mcu_sync.stage_deflated,
                                    "Deflate (~5x smaller)",
                                );
                            });
                        }

                        if self.mcu_sync.pulling {
                            ui.add_space(4.0);
                            ui.add(
                                egui::ProgressBar::new(self.mcu_sync.progress())
                                    .text(format!(
                                        "Downloading chunks ({} / {} B)",
                                        self.mcu_sync.pull_buffer.len(),
                                        self.mcu_sync
                                            .pull_target_info
                                            .map(|i| i.total_bytes)
                                            .unwrap_or(0)
                                    ))
                                    .fill(Color32::from_rgb(56, 189, 248)),
                            );
                        } else if !self.mcu_sync.last_message.is_empty() {
                            ui.add_space(4.0);
                            ui.label(
                                RichText::new(&self.mcu_sync.last_message)
                                    .size(11.0)
                                    .color(Color32::from_rgb(148, 163, 184)),
                            );
                        }
                    });

                ui.add_space(8.0);

                // 3. Parsed Kinematics Tree Inspector
                egui::Frame::new()
                    .fill(Color32::from_rgb(18, 24, 34))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::same(10))
                    .stroke(Stroke::new(1.0, Color32::from_rgb(38, 52, 72)))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(format!(
                                "Kinematics Chain ({} Links · {} Movable Joints)",
                                self.model.rig.links.len(),
                                self.model.rig.joints.len()
                            ))
                            .strong()
                            .size(13.0)
                            .color(Color32::from_rgb(226, 232, 240)),
                        );
                        ui.add_space(4.0);

                        for j in &self.model.rig.joints {
                            let is_sel = self.model.selected_joint == j.index;
                            let unit_range = if j.kind == humanoid_model::JointKind::Prismatic {
                                format!("[{:.2}..{:.2} m]", j.default_min_rad, j.default_max_rad)
                            } else {
                                format!(
                                    "[{:.0}°..{:.0}°]",
                                    j.default_min_rad.to_degrees(),
                                    j.default_max_rad.to_degrees()
                                )
                            };
                            let text = format!(
                                "#{:02} {} ({}) : {} -> {}  {}",
                                j.index,
                                j.name,
                                j.kind.label(),
                                j.parent_link,
                                j.child_link,
                                unit_range
                            );
                            if ui
                                .selectable_label(
                                    is_sel,
                                    RichText::new(text).monospace().size(11.0),
                                )
                                .clicked()
                            {
                                self.model.selected_joint = j.index;
                            }
                        }

                        if self.model.rig.joints.is_empty() {
                            ui.label(
                                RichText::new(
                                    "Static URDF (0 movable joints) — rendering fixed link visuals only.",
                                )
                                .size(11.5)
                                .color(Color32::from_rgb(148, 163, 184)),
                            );
                        }
                    });
            });
    }

    /// Modal window allowing quick paste/edit of URDF XML directly inside Rocky Studio.
    pub fn render_paste_urdf_modal(&mut self, ctx: &egui::Context) {
        if !self.show_paste_urdf_dialog {
            return;
        }
        let mut open = self.show_paste_urdf_dialog;
        let mut load_clicked = false;
        let mut cancel_clicked = false;

        egui::Window::new("Paste / Edit URDF XML")
            .open(&mut open)
            .collapsible(false)
            .default_width(560.0)
            .default_height(420.0)
            .show(ctx, |ui| {
                ui.label(
                    RichText::new(
                        "Paste any valid URDF XML (<robot>...</robot>) to parse and render immediately in the 3D canvas:",
                    )
                    .size(12.0)
                    .color(Color32::from_rgb(168, 184, 205)),
                );
                ui.add_space(4.0);
                egui::ScrollArea::vertical().max_height(310.0).show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(&mut self.paste_urdf_buf)
                            .font(egui::TextStyle::Monospace)
                            .desired_rows(18)
                            .desired_width(f32::INFINITY),
                    );
                });
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                RichText::new("Parse & Render in 3D Canvas")
                                    .strong()
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(14, 116, 144)),
                        )
                        .clicked()
                    {
                        load_clicked = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel_clicked = true;
                    }
                });
            });

        if load_clicked {
            match AssetBundle::from_pasted_xml(&self.paste_urdf_buf) {
                Ok(bundle) => {
                    self.apply_loaded_bundle(bundle);
                    self.show_paste_urdf_dialog = false;
                }
                Err(err) => {
                    self.model.status_message = err;
                }
            }
        }
        if cancel_clicked || !open {
            self.show_paste_urdf_dialog = false;
        }
    }

    /// Toggles the active URDF's up-axis between `Y-Up` and `Z-Up (ROS)` and refreshes the 3D scene.
    pub fn toggle_urdf_up_axis(&mut self) {
        let next = match self.model.rig.up_axis {
            UpAxis::YUp => UpAxis::ZUp,
            UpAxis::ZUp => UpAxis::YUp,
        };
        self.model.rig.set_up_axis(next);
        self.viewport.notify_rig_changed();
        self.viewport.fit_to_rig(&self.model.rig);
        self.model.status_message = format!(
            "Switched '{}' orientation to {} and reframed 3D canvas.",
            self.model.rig.robot_name,
            next.label()
        );
    }
}
