//! Layer 6 — Browser WASM Runner (`humanoid-web`) for Rocky Studio.

#[cfg(target_arch = "wasm32")]
mod web_app {
    use craft_platform::browser::{DownloadOptions, PreparedDownload};
    use humanoid_ui::{PlatformSaver, RockyStudioApp};
    use wasm_bindgen::JsCast;

    struct BrowserDownloadSaver;

    impl PlatformSaver for BrowserDownloadSaver {
        fn save_file(&mut self, filename: &str, mime: &str, bytes: &[u8]) -> Result<String, String> {
            let dl = PreparedDownload::bytes(filename, mime, bytes)
                .map_err(|e| format!("Prepare download failed: {e:?}"))?;
            dl.click(DownloadOptions::default())
                .map_err(|e| format!("Trigger download failed: {e:?}"))?;
            Ok(format!("Downloaded '{filename}' via browser."))
        }
    }

    struct WebWrapper {
        inner: RockyStudioApp,
    }

    impl eframe::App for WebWrapper {
        fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
            self.inner.update_frame(ui);
        }
    }

    pub fn launch() {
        console_error_panic_hook::set_once();

        let web_options = eframe::WebOptions::default();

        wasm_bindgen_futures::spawn_local(async {
            let document = web_sys::window()
                .expect("browser window")
                .document()
                .expect("browser document");

            let canvas = document
                .get_element_by_id("rocky_canvas")
                .expect("missing #rocky_canvas")
                .dyn_into::<web_sys::HtmlCanvasElement>()
                .expect("#rocky_canvas is not a canvas");

            let start_result = eframe::WebRunner::new()
                .start(
                    canvas,
                    web_options,
                    Box::new(|_cc| {
                        Ok(Box::new(WebWrapper {
                            inner: RockyStudioApp::new(Box::new(BrowserDownloadSaver)),
                        }))
                    }),
                )
                .await;

            if let Err(err) = start_result {
                if let Some(el) = document.get_element_by_id("rocky_loading") {
                    el.set_text_content(Some(&format!(
                        "Rocky Studio WebGL2 startup error: {err:?}"
                    )));
                }
            } else if let Some(el) = document.get_element_by_id("rocky_loading") {
                el.remove();
            }
        });
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    web_app::launch();
}
