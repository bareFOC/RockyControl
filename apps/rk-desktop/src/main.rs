//! Layer 6 — Native Desktop Runner (`rk-desktop`) for Rocky Studio.

use std::env;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use craft_atomic::{NewFilePermissions, WriteOptions};
use rk_mcp::{HttpServerHandle, mcp_bridge_channel, spawn_http_bridge};
use rk_ui::{NativeMcpListenerStatus, PlatformSaver, RockyStudioApp};

const DEFAULT_MCP_PORT: u16 = 7788;
const MCP_LOOPBACK_HOST: &str = "127.0.0.1";

/// Resolved native MCP listener configuration (CLI args override the env var).
struct McpListenerConfig {
    /// `None` means `--no-mcp` was passed: do not bind anything.
    addr: Option<String>,
}

impl McpListenerConfig {
    fn resolve() -> Self {
        let mut port = env::var("ROCKY_MCP_PORT")
            .ok()
            .and_then(|raw| parse_port(&raw, "ROCKY_MCP_PORT"))
            .unwrap_or(DEFAULT_MCP_PORT);
        let mut enabled = true;

        let mut args = env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--no-mcp" => enabled = false,
                "--mcp-port" => match args.next() {
                    Some(raw) => {
                        if let Some(parsed) = parse_port(&raw, "--mcp-port") {
                            port = parsed;
                        }
                    }
                    None => eprintln!("[rk-desktop] --mcp-port requires a value; using {port}"),
                },
                _ => {
                    if let Some(raw) = arg.strip_prefix("--mcp-port=") {
                        if let Some(parsed) = parse_port(raw, "--mcp-port") {
                            port = parsed;
                        }
                    }
                }
            }
        }

        // `--no-mcp` is the most explicit opt-out and wins over any port setting.
        let addr = enabled.then(|| format!("{MCP_LOOPBACK_HOST}:{port}"));
        Self { addr }
    }
}

fn parse_port(raw: &str, source: &str) -> Option<u16> {
    match raw.trim().parse::<u16>() {
        Ok(port) if port != 0 => Some(port),
        _ => {
            eprintln!("[rk-desktop] ignoring invalid {source} port {raw:?}");
            None
        }
    }
}

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
        let opts =
            WriteOptions::default().with_new_file_permissions(NewFilePermissions::PlatformDefault);
        opts.atomic_write(&dest, bytes)
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
    /// Owns the background HTTP listener thread; `None` when disabled or on bind failure.
    mcp_listener: Option<HttpServerHandle>,
}

impl DesktopWrapper {
    fn stop_mcp_listener(&mut self) {
        if let Some(handle) = self.mcp_listener.as_mut() {
            handle.stop();
            self.inner.mcp.native_listener_status = NativeMcpListenerStatus::Stopped;
        }
    }
}

impl Drop for DesktopWrapper {
    fn drop(&mut self) {
        // Backup for paths where eframe skips `on_exit` (e.g. a panic-driven teardown).
        self.stop_mcp_listener();
    }
}

impl eframe::App for DesktopWrapper {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // `logic` also runs while the window is hidden, so loopback requests stay
        // serviced when minimized. Run before craft-quit can terminate the process.
        self.inner.pump_mcp_bridge(ctx);
        craft_quit::on_close_request(ctx);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.inner.update_frame(ui);
    }

    fn on_exit(&mut self, _gl: Option<&glow::Context>) {
        self.stop_mcp_listener();
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

    let mcp_config = McpListenerConfig::resolve();

    eframe::run_native(
        "Rocky Studio",
        options,
        Box::new(move |cc| {
            let mut inner = RockyStudioApp::new(Box::new(DesktopAtomicSaver::new()));
            let mut mcp_listener = None;

            if let Some(addr) = mcp_config.addr {
                // The receiver is !Send and must be created on the egui thread. Cloning
                // the Context gives HTTP workers a thread-safe way to request a new
                // `logic` pass so queued requests are drained promptly (even if hidden).
                let ctx = cc.egui_ctx.clone();
                let (tx, rx) = mcp_bridge_channel(move || ctx.request_repaint());
                inner.attach_mcp_bridge(rx);

                match spawn_http_bridge(&addr, tx) {
                    Ok(handle) => {
                        eprintln!("[rk-desktop] native MCP listener on http://{addr}/mcp");
                        inner.mcp.native_listener_status =
                            NativeMcpListenerStatus::Running { addr };
                        mcp_listener = Some(handle);
                    }
                    Err(err) => {
                        // A busy port must never prevent the GUI from launching.
                        eprintln!("[rk-desktop] MCP listener could not bind {addr}: {err}");
                        inner.mcp.native_listener_status = NativeMcpListenerStatus::Failed {
                            message: format!("{err}"),
                        };
                    }
                }
            }

            Ok(Box::new(DesktopWrapper {
                inner,
                mcp_listener,
            }))
        }),
    )
}
