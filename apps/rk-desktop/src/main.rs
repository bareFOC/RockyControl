//! Layer 6 — Native Desktop Runner (`rk-desktop`) for Rocky Studio.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use craft_atomic::{NewFilePermissions, WriteOptions, atomic_write};
use rk_ui::{PlatformSaver, RockyStudioApp};

struct DesktopAtomicSaver {
    opened_queue: Arc<Mutex<Vec<(String, Vec<u8>)>>>,
}

impl DesktopAtomicSaver {
    fn new() -> Self {
        Self {
            opened_queue: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl PlatformSaver for DesktopAtomicSaver {
    fn save_file(&mut self, filename: &str, _mime: &str, bytes: &[u8]) -> Result<String, String> {
        let dir = PathBuf::from("exports");
        let _ = std::fs::create_dir_all(&dir);
        let dest = dir.join(filename);
        let opts = WriteOptions {
            permissions: NewFilePermissions::PlatformDefault,
            ..WriteOptions::default()
        };
        atomic_write(&dest, bytes, &opts)
            .map_err(|e| format!("Atomic save failed for {}: {e}", dest.display()))?;
        Ok(format!("Saved atomically to {}", dest.display()))
    }

    fn request_open_urdf_file(&mut self) -> Result<(), String> {
        let queue = Arc::clone(&self.opened_queue);
        std::thread::spawn(move || {
            let output = std::process::Command::new("zenity")
                .args([
                    "--file-selection",
                    "--title=Select URDF or ZIP Robot Bundle",
                    "--file-filter=URDF / ZIP | *.urdf *.xml *.zip",
                ])
                .output()
                .ok();
            if let Some(out) = output {
                if out.status.success() {
                    let path_str = String::from_utf8_lossy(&out.stdout).trim().to_string();
                    let path = PathBuf::from(&path_str);
                    if let Ok(bytes) = std::fs::read(&path) {
                        let name = path
                            .file_name()
                            .map(|n| n.to_string_lossy().into_owned())
                            .unwrap_or_else(|| "uploaded.urdf".to_string());
                        if let Ok(mut q) = queue.lock() {
                            q.push((name, bytes));
                        }
                    }
                }
            }
        });
        Ok(())
    }

    fn poll_opened_files(&mut self) -> Vec<(String, Vec<u8>)> {
        self.opened_queue
            .lock()
            .map(|mut q| std::mem::take(&mut *q))
            .unwrap_or_default()
    }

    fn is_fullscreen(&self, ctx: &egui::Context) -> bool {
        ctx.input(|i| i.viewport().fullscreen.unwrap_or(false))
    }

    fn toggle_fullscreen(&mut self, ctx: &egui::Context) -> Result<bool, String> {
        let next = !self.is_fullscreen(ctx);
        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(next));
        Ok(next)
    }
}

struct DesktopWrapper {
    inner: RockyStudioApp,
}

impl eframe::App for DesktopWrapper {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        craft_quit::on_close_request(ctx);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.inner.update_frame(ui);
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        viewport: egui::ViewportBuilder::default()
            .with_title("Rocky Studio — Humanoid Control Workstation")
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([960.0, 640.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Rocky Studio",
        options,
        Box::new(|_cc| {
            Ok(Box::new(DesktopWrapper {
                inner: RockyStudioApp::new(Box::new(DesktopAtomicSaver::new())),
            }))
        }),
    )
}
