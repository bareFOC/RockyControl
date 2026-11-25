//! Layer 6 — Android APK runner (`rk-android`) for Rocky Studio.
//!
//! Mirrors `apps/rk-desktop` and `apps/rk-web`: `RockyStudioApp::update_frame`
//! is the single per-frame entry point. No MCP listener, UDP transport,
//! gamepad polling or desktop file utilities are started here.

// On the host, `saver` is compiled only for unit tests (the Android runner is
// target-gated); on Android it is part of the application.
#[cfg(any(target_os = "android", test))]
mod saver;

#[cfg(any(target_os = "android", test))]
use saver::AndroidSaver;

#[cfg(target_os = "android")]
use winit::platform::android::activity::AndroidApp;

#[cfg(target_os = "android")]
#[unsafe(no_mangle)]
fn android_main(app: AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );

    // Capture the app-private files root before moving `app` into eframe.
    let base_dir = app.internal_data_path();
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        android_app: Some(app),
        ..Default::default()
    };

    let result = eframe::run_native(
        "Rocky Studio",
        options,
        Box::new(move |_cc| Ok(Box::new(AndroidWrapper::new(base_dir)))),
    );
    if let Err(err) = result {
        log::error!("eframe exited with an error: {err:?}");
    }
}

#[cfg(target_os = "android")]
struct AndroidWrapper {
    inner: rk_ui::RockyStudioApp,
}

#[cfg(target_os = "android")]
impl AndroidWrapper {
    fn new(base_dir: Option<std::path::PathBuf>) -> Self {
        Self {
            inner: rk_ui::RockyStudioApp::new(Box::new(AndroidSaver::new(base_dir))),
        }
    }
}

#[cfg(target_os = "android")]
impl eframe::App for AndroidWrapper {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Reserve space at the top so the toolbar isn't hidden behind the
        // Android status bar. TODO: use winit safe-area when available
        // (https://github.com/rust-windowing/winit/issues/3910).
        egui::Panel::top("status_bar_space").show(ui, |ui| {
            ui.set_height(32.0);
        });

        self.inner.update_frame(ui);
    }
}
