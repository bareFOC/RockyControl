//! Layer 6 — Browser WASM Runner (`rk-web`) for Rocky Studio.

#[cfg(target_arch = "wasm32")]
mod web_app {
    use std::cell::RefCell;
    use std::rc::Rc;

    use craft_platform::browser::{DownloadOptions, PreparedDownload};
    use rk_ui::{PlatformSaver, RockyStudioApp};
    use wasm_bindgen::JsCast;
    use wasm_bindgen::closure::Closure;

    struct BrowserDownloadSaver {
        opened_queue: Rc<RefCell<Vec<(String, Vec<u8>)>>>,
        active_input: Option<web_sys::HtmlInputElement>,
        _active_closure: Option<Closure<dyn FnMut(web_sys::Event)>>,
    }

    impl BrowserDownloadSaver {
        fn new() -> Self {
            Self {
                opened_queue: Rc::new(RefCell::new(Vec::new())),
                active_input: None,
                _active_closure: None,
            }
        }
    }

    impl PlatformSaver for BrowserDownloadSaver {
        fn save_file(&mut self, filename: &str, mime: &str, bytes: &[u8]) -> Result<String, String> {
            let dl = PreparedDownload::bytes(filename, mime, bytes)
                .map_err(|e| format!("Prepare download failed: {e:?}"))?;
            dl.click(DownloadOptions::default())
                .map_err(|e| format!("Trigger download failed: {e:?}"))?;
            Ok(format!("Downloaded '{filename}' via browser."))
        }

        fn request_open_urdf_file(&mut self) -> Result<(), String> {
            let document = web_sys::window()
                .and_then(|w| w.document())
                .ok_or_else(|| "Browser document unavailable".to_string())?;
            let input: web_sys::HtmlInputElement = document
                .create_element("input")
                .map_err(|e| format!("Create file input failed: {e:?}"))?
                .dyn_into()
                .map_err(|_| "Input element cast failed".to_string())?;
            input.set_type("file");
            input.set_accept(".urdf,.xml,.zip");
            let _ = input.set_attribute("data-testid", "urdf-file-input");

            let queue = Rc::clone(&self.opened_queue);
            let input_for_cb = input.clone();
            let closure = Closure::wrap(Box::new(move |_event: web_sys::Event| {
                if let Some(files) = input_for_cb.files() {
                    if let Some(file) = files.get(0) {
                        let name = file.name();
                        let queue_inner = Rc::clone(&queue);
                        wasm_bindgen_futures::spawn_local(async move {
                            if let Ok(ab_val) =
                                wasm_bindgen_futures::JsFuture::from(file.array_buffer()).await
                            {
                                let bytes = js_sys::Uint8Array::new(&ab_val).to_vec();
                                queue_inner.borrow_mut().push((name, bytes));
                            }
                        });
                    }
                }
            }) as Box<dyn FnMut(web_sys::Event)>);

            input
                .add_event_listener_with_callback("change", closure.as_ref().unchecked_ref())
                .map_err(|e| format!("Bind change listener failed: {e:?}"))?;
            input.click();

            self.active_input = Some(input);
            self._active_closure = Some(closure);
            Ok(())
        }

        fn poll_opened_files(&mut self) -> Vec<(String, Vec<u8>)> {
            std::mem::take(&mut *self.opened_queue.borrow_mut())
        }

        fn is_fullscreen(&self, _ctx: &egui::Context) -> bool {
            web_sys::window()
                .and_then(|w| w.document())
                .and_then(|d| d.fullscreen_element())
                .is_some()
        }

        fn toggle_fullscreen(&mut self, _ctx: &egui::Context) -> Result<bool, String> {
            let document = web_sys::window()
                .and_then(|w| w.document())
                .ok_or_else(|| "Browser document unavailable".to_string())?;
            if document.fullscreen_element().is_some() {
                document.exit_fullscreen();
                Ok(false)
            } else {
                let root = document
                    .document_element()
                    .ok_or_else(|| "Document root unavailable".to_string())?;
                root.request_fullscreen()
                    .map_err(|e| format!("Fullscreen request blocked: {e:?}"))?;
                Ok(true)
            }
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
                            inner: RockyStudioApp::new(Box::new(BrowserDownloadSaver::new())),
                        }))
                    }),
                )
                .await;

            if let Err(err) = start_result {
                if let Some(el) = document.get_element_by_id("rocky_loading_status") {
                    el.set_text_content(Some(&format!(
                        "WebGL2 startup error: {err:?}"
                    )));
                } else if let Some(el) = document.get_element_by_id("rocky_loading") {
                    el.set_text_content(Some(&format!(
                        "Rocky Studio WebGL2 startup error: {err:?}"
                    )));
                }
            } else if let Some(el) = document.get_element_by_id("rocky_loading") {
                let _ = el.set_attribute("class", "fade-out");
            }
        });
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    web_app::launch();
}
