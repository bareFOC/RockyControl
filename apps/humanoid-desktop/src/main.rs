//! Layer 6 — Native Desktop Runner (`humanoid-desktop`) for Rocky Studio.

use std::path::PathBuf;

use craft_atomic::{NewFilePermissions, WriteOptions, atomic_write};
use humanoid_ui::{PlatformSaver, RockyStudioApp};

struct DesktopAtomicSaver;

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
                inner: RockyStudioApp::new(Box::new(DesktopAtomicSaver)),
            }))
        }),
    )
}
