//! Live MCP bridge (`McpHost` implementation for `RockyStudioApp`) and WASM Streamable HTTP poller.

use std::collections::VecDeque;

use rk_link::{FIXED_CONTROL_STEP_SEC, RobotTransport, TransportKind};
use rk_mcp::{
    CameraOrbitRequest, McpActivityEntry, McpHost, McpPolicy, ViewportCameraState,
    dispatch_json_rpc,
};
use rk_model::{AssetBundle, StudioModel};
use rk_proto::{ModelRequest, ModelResponse};
use rk_viewport::CameraPreset;
use serde_json::{Value, json};

use crate::{AppScreen, RockyStudioApp};

const MAX_ACTIVITY_ENTRIES: usize = 64;

/// Selected client setup snippet tab inside the MCP Agent Drawer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum McpSnippetTab {
    Http,
    ClaudeCursor,
    GeminiCurl,
}

impl McpSnippetTab {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Http => "HTTP /mcp",
            Self::ClaudeCursor => "Claude / Cursor",
            Self::GeminiCurl => "Gemini / curl",
        }
    }
}

/// Status of the native loopback HTTP listener started by `rk-desktop`.
///
/// This is reported by the platform wrapper; `rk-ui` never claims a listener is
/// live unless the wrapper has actually bound and started one. Android never
/// starts a listener, so the type is absent there and the drawer reports it as
/// unavailable.
#[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NativeMcpListenerStatus {
    /// No listener was started (e.g. `--no-mcp`).
    NotAvailable,
    /// Listener is bound and serving on this `127.0.0.1:port` address.
    Running { addr: String },
    /// Startup failed; the failure is described by `message`.
    Failed { message: String },
    /// The listener was started and has since been stopped.
    Stopped,
}

#[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
impl NativeMcpListenerStatus {
    /// Short, human-readable endpoint status used in drawer labels.
    pub fn endpoint_status_text(&self) -> String {
        match self {
            Self::NotAvailable => "Endpoint /mcp unavailable".to_string(),
            Self::Running { addr } => format!("Endpoint http://{addr}/mcp live"),
            Self::Failed { message } => format!("Endpoint /mcp unavailable ({message})"),
            Self::Stopped => "Endpoint /mcp stopped".to_string(),
        }
    }
}

/// UI and bridge state for Rocky Studio's built-in MCP server.
pub struct McpUiState {
    pub policy: McpPolicy,
    pub drawer_open: bool,
    pub drawer_rect: egui::Rect,
    pub snippet_tab: McpSnippetTab,
    pub bridge_connected: bool,
    /// Set by `rk-desktop` once the native HTTP listener has started (or failed).
    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
    pub native_listener_status: NativeMcpListenerStatus,
    pub activity: VecDeque<McpActivityEntry>,
    pub mcp_walk_override: Option<(f32, f32, f32, bool)>,
    pub test_preset_idx: usize,
    pub last_test_output: String,
    pub last_activity_sim_time: f64,
    pub copied_notice_until: f64,
    pub(crate) config_dirty: bool,
    pub(crate) next_poll_allowed_at: f64,
    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
    pub(crate) pending_native_screenshots: Vec<(rk_mcp::McpBridgeRequest, web_time::Instant)>,
    #[cfg(target_arch = "wasm32")]
    pub(crate) pending_wasm_screenshots: Vec<(String, Value, web_time::Instant)>,
}

impl Default for McpUiState {
    fn default() -> Self {
        Self {
            policy: McpPolicy::default(), // enabled: true, drive_armed: false
            drawer_open: false,
            drawer_rect: egui::Rect::NOTHING,
            snippet_tab: McpSnippetTab::Http,
            bridge_connected: false,
            #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
            native_listener_status: NativeMcpListenerStatus::NotAvailable,
            activity: VecDeque::with_capacity(MAX_ACTIVITY_ENTRIES),
            mcp_walk_override: None,
            test_preset_idx: 0,
            last_test_output: String::new(),
            last_activity_sim_time: -10.0,
            copied_notice_until: -10.0,
            config_dirty: true,
            next_poll_allowed_at: 0.0,
            #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
            pending_native_screenshots: Vec::new(),
            #[cfg(target_arch = "wasm32")]
            pending_wasm_screenshots: Vec::new(),
        }
    }
}

impl McpUiState {
    /// Honest endpoint status text for the current platform, used by the drawer.
    pub fn endpoint_status(&self) -> String {
        #[cfg(target_arch = "wasm32")]
        {
            if self.bridge_connected {
                "Endpoint /mcp live".to_string()
            } else {
                "Endpoint /mcp disconnected".to_string()
            }
        }
        #[cfg(target_os = "android")]
        {
            "Endpoint /mcp unavailable on Android".to_string()
        }
        #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
        {
            self.native_listener_status.endpoint_status_text()
        }
    }

    /// URL advertised in setup snippets; uses the actual bound address on native.
    pub fn http_url(&self) -> String {
        #[cfg(any(target_arch = "wasm32", target_os = "android"))]
        {
            current_mcp_http_url()
        }
        #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
        {
            match &self.native_listener_status {
                NativeMcpListenerStatus::Running { addr } => format!("http://{addr}/mcp"),
                _ => current_mcp_http_url(),
            }
        }
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        if self.policy.enabled != enabled {
            self.policy.enabled = enabled;
            self.config_dirty = true;
            if !enabled {
                self.policy.drive_armed = false;
                self.mcp_walk_override = None;
            }
        }
    }

    pub fn set_drive_armed(&mut self, armed: bool) {
        let clamped = armed && self.policy.enabled;
        if self.policy.drive_armed != clamped {
            self.policy.drive_armed = clamped;
            self.config_dirty = true;
            if !clamped {
                self.mcp_walk_override = None;
            }
        }
    }

    pub fn on_estop_disarm(&mut self) {
        if self.policy.drive_armed {
            self.policy.drive_armed = false;
            self.config_dirty = true;
        }
        self.mcp_walk_override = None;
    }
}

/// Returns the live Streamable HTTP MCP URL for the current origin (or native default).
/// Android never starts a listener, so it reports the endpoint as unavailable.
pub fn current_mcp_http_url() -> String {
    #[cfg(target_arch = "wasm32")]
    {
        if let Some(win) = web_sys::window() {
            if let Ok(origin) = win.location().origin() {
                if !origin.is_empty() && origin != "null" {
                    return format!("{origin}/mcp");
                }
            }
        }
        "http://127.0.0.1:3000/mcp".to_string()
    }
    #[cfg(target_os = "android")]
    {
        "MCP listener unavailable on Android".to_string()
    }
    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
    {
        "http://127.0.0.1:7788/mcp".to_string()
    }
}

#[cfg(target_arch = "wasm32")]
mod wasm_bridge {
    use std::cell::RefCell;

    use serde_json::{Value, json};
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::{Headers, Request, RequestInit, RequestMode, Response};

    #[derive(Default)]
    pub struct WasmBridgeQueue {
        pub poll_in_flight: bool,
        pub bridge_connected: bool,
        pub last_poll_failed: bool,
        pub incoming: Vec<(String, Value)>,
    }

    thread_local! {
        pub static QUEUE: RefCell<WasmBridgeQueue> = RefCell::new(WasmBridgeQueue::default());
    }

    async fn fetch_json(
        method: &str,
        url: &str,
        body: Option<String>,
    ) -> Result<(u16, Option<String>), String> {
        let win = web_sys::window().ok_or_else(|| "no window".to_string())?;
        let opts = RequestInit::new();
        opts.set_method(method);
        opts.set_mode(RequestMode::SameOrigin);
        if let Some(b) = body {
            opts.set_body(&wasm_bindgen::JsValue::from_str(&b));
            let headers = Headers::new().map_err(|e| format!("{e:?}"))?;
            headers
                .set("Content-Type", "application/json")
                .map_err(|e| format!("{e:?}"))?;
            opts.set_headers(&headers);
        }
        let req = Request::new_with_str_and_init(url, &opts).map_err(|e| format!("{e:?}"))?;
        let resp_val = JsFuture::from(win.fetch_with_request(&req))
            .await
            .map_err(|e| format!("{e:?}"))?;
        let resp: Response = resp_val.dyn_into().map_err(|e| format!("{e:?}"))?;
        let status = resp.status();
        if status == 204 {
            return Ok((status, None));
        }
        let text_prom = resp.text().map_err(|e| format!("{e:?}"))?;
        let text_val = JsFuture::from(text_prom).await.map_err(|e| format!("{e:?}"))?;
        Ok((status, text_val.as_string()))
    }

    pub fn spawn_config_sync(enabled: bool, drive_armed: bool) {
        let body = json!({
            "enabled": enabled,
            "drive_armed": drive_armed,
        })
        .to_string();
        wasm_bindgen_futures::spawn_local(async move {
            let _ = fetch_json("POST", "/api/mcp/bridge/config", Some(body)).await;
        });
    }

    pub fn spawn_reply(req_id: String, response: Option<Value>) {
        let body = json!({
            "id": req_id,
            "response": response,
        })
        .to_string();
        wasm_bindgen_futures::spawn_local(async move {
            let _ = fetch_json("POST", "/api/mcp/bridge/reply", Some(body)).await;
        });
    }

    pub fn spawn_poll(ctx: egui::Context) {
        let started = QUEUE.with(|q| {
            let mut q = q.borrow_mut();
            if q.poll_in_flight {
                return false;
            }
            q.poll_in_flight = true;
            true
        });
        if !started {
            return;
        }
        wasm_bindgen_futures::spawn_local(async move {
            let res = fetch_json("GET", "/api/mcp/bridge/poll?wait_ms=250", None).await;
            QUEUE.with(|q| {
                let mut q = q.borrow_mut();
                q.poll_in_flight = false;
                match res {
                    Ok((200, Some(text))) => {
                        q.bridge_connected = true;
                        q.last_poll_failed = false;
                        if let Ok(val) = serde_json::from_str::<Value>(&text) {
                            if let (Some(id), Some(payload)) = (
                                val.get("id").and_then(Value::as_str),
                                val.get("payload").cloned(),
                            ) {
                                q.incoming.push((id.to_string(), payload));
                            }
                        }
                    }
                    Ok((204, _)) => {
                        q.bridge_connected = true;
                        q.last_poll_failed = false;
                    }
                    _ => {
                        q.bridge_connected = false;
                        q.last_poll_failed = true;
                    }
                }
            });
            ctx.request_repaint();
        });
    }
}

const SCREENSHOT_WAIT_TIMEOUT: web_time::Duration = web_time::Duration::from_secs(3);

fn is_viewport_screenshot_call(msg: &Value) -> bool {
    msg.get("method").and_then(Value::as_str) == Some("tools/call")
        && msg
            .get("params")
            .and_then(|p| p.get("name"))
            .and_then(Value::as_str)
            == Some("viewport_screenshot")
}

impl RockyStudioApp {
    /// Attaches the native bridge receiver. Must be called on the egui thread;
    /// the native platform wrapper drains it from `App::logic` so requests are
    /// also serviced while the window is hidden.
    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
    pub fn attach_mcp_bridge(&mut self, rx: rk_mcp::McpBridgeRx) {
        self.mcp_bridge_rx = Some(rx);
    }

    /// Pumps pending live Streamable HTTP MCP calls into `RockyStudioApp` and syncs policy toggles.
    /// On native this is driven by `DesktopWrapper::logic`, not by `update_frame`.
    pub fn pump_mcp_bridge(&mut self, ctx: &egui::Context) {
        #[cfg(target_arch = "wasm32")]
        {
            if self.mcp.config_dirty {
                self.mcp.config_dirty = false;
                wasm_bridge::spawn_config_sync(self.mcp.policy.enabled, self.mcp.policy.drive_armed);
            }

            let (incoming, connected, failed) = wasm_bridge::QUEUE.with(|q| {
                let mut q = q.borrow_mut();
                let failed = q.last_poll_failed;
                q.last_poll_failed = false;
                (std::mem::take(&mut q.incoming), q.bridge_connected, failed)
            });
            self.mcp.bridge_connected = connected;
            if failed {
                self.mcp.next_poll_allowed_at = self.sim_time_sec + 1.0;
            }

            if !self.mcp.pending_wasm_screenshots.is_empty() {
                let resolved = if let Some(res) = self.viewport.take_screenshot() {
                    Some(res.map(|s| (s.width, s.height, s.png_bytes)))
                } else if self
                    .mcp
                    .pending_wasm_screenshots
                    .first()
                    .is_some_and(|(_, _, t)| t.elapsed() >= SCREENSHOT_WAIT_TIMEOUT)
                {
                    let _ = self.viewport.take_screenshot();
                    Some(Err(
                        "Timed out waiting for 3D viewport to render a frame.".to_string(),
                    ))
                } else {
                    None
                };

                if let Some(shot_res) = resolved {
                    let pending = std::mem::take(&mut self.mcp.pending_wasm_screenshots);
                    for (req_id, payload, _) in pending {
                        self.pending_viewport_screenshot = Some(shot_res.clone());
                        let reply = dispatch_json_rpc(self, payload);
                        self.pending_viewport_screenshot = None;
                        wasm_bridge::spawn_reply(req_id, reply);
                    }
                }
            }

            for (req_id, payload) in incoming {
                if self.mcp.policy.enabled && is_viewport_screenshot_call(&payload) {
                    self.viewport.request_screenshot();
                    ctx.request_repaint();
                    self.mcp
                        .pending_wasm_screenshots
                        .push((req_id, payload, web_time::Instant::now()));
                } else {
                    let reply = dispatch_json_rpc(self, payload);
                    wasm_bridge::spawn_reply(req_id, reply);
                }
            }

            if self.sim_time_sec >= self.mcp.next_poll_allowed_at {
                wasm_bridge::spawn_poll(ctx.clone());
            }
        }
        #[cfg(target_os = "android")]
        {
            let _ = ctx;
            // No native MCP bridge on Android: keep advertised state honest.
            self.mcp.config_dirty = false;
            self.mcp.bridge_connected = false;
        }
        #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
        {
            self.mcp.config_dirty = false;

            if !self.mcp.pending_native_screenshots.is_empty() {
                let resolved = if let Some(res) = self.viewport.take_screenshot() {
                    Some(res.map(|s| (s.width, s.height, s.png_bytes)))
                } else if self
                    .mcp
                    .pending_native_screenshots
                    .first()
                    .is_some_and(|(_, t)| t.elapsed() >= SCREENSHOT_WAIT_TIMEOUT)
                {
                    let _ = self.viewport.take_screenshot();
                    Some(Err(
                        "Timed out waiting for 3D viewport to render a frame (window may be minimized or hidden).".to_string(),
                    ))
                } else {
                    None
                };

                if let Some(shot_res) = resolved {
                    let pending = std::mem::take(&mut self.mcp.pending_native_screenshots);
                    for (req, _) in pending {
                        self.pending_viewport_screenshot = Some(shot_res.clone());
                        let response = dispatch_json_rpc(self, req.message);
                        self.pending_viewport_screenshot = None;
                        let _ = req.reply.send(response);
                    }
                }
            }

            // Collect requests first so the mutable borrow of `mcp_bridge_rx`
            // ends before `dispatch_json_rpc` borrows the whole app.
            let requests = self
                .mcp_bridge_rx
                .as_mut()
                .map(rk_mcp::McpBridgeRx::try_recv_all)
                .unwrap_or_default();
            for req in requests {
                if self.mcp.policy.enabled && is_viewport_screenshot_call(&req.message) {
                    self.viewport.request_screenshot();
                    ctx.request_repaint();
                    self.mcp
                        .pending_native_screenshots
                        .push((req, web_time::Instant::now()));
                } else {
                    let response = dispatch_json_rpc(self, req.message);
                    let _ = req.reply.send(response);
                }
            }
        }
    }

    /// Executes an inline MCP JSON-RPC `tools/call` test directly from the MCP Drawer.
    pub(crate) fn run_inline_mcp_tool(&mut self, name: &str, args: Value) {
        let req = json!({
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": name,
                "arguments": args
            }
        });
        match dispatch_json_rpc(self, req) {
            Some(resp) => {
                if let Some(txt) = resp
                    .get("result")
                    .and_then(|r| r.get("content"))
                    .and_then(Value::as_array)
                    .and_then(|arr| arr.first())
                    .and_then(|c| c.get("text"))
                    .and_then(Value::as_str)
                {
                    self.mcp.last_test_output = txt.to_string();
                } else {
                    self.mcp.last_test_output =
                        serde_json::to_string_pretty(&resp).unwrap_or_else(|_| resp.to_string());
                }
            }
            None => {
                self.mcp.last_test_output = "(no response)".to_string();
            }
        }
    }
}

impl McpHost for RockyStudioApp {
    fn model(&self) -> &StudioModel {
        &self.model
    }

    fn model_mut(&mut self) -> &mut StudioModel {
        &mut self.model
    }

    fn transport(&self) -> &dyn RobotTransport {
        &*self.transport
    }

    fn transport_mut(&mut self) -> &mut dyn RobotTransport {
        &mut *self.transport
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn connect(&mut self, kind: TransportKind) -> Result<String, String> {
        self.selected_transport_ui = kind;
        self.connect_active_transport();
        if self.connected {
            Ok(self.model.status_message.clone())
        } else {
            Err(self.model.status_message.clone())
        }
    }

    fn disconnect(&mut self) {
        self.mcp.mcp_walk_override = None;
        self.disconnect_transport();
    }

    fn load_bundle(&mut self, bundle: AssetBundle) -> Result<String, String> {
        self.apply_loaded_bundle(bundle);
        Ok(self.model.status_message.clone())
    }

    fn sync_mcu_model(&mut self) -> Result<String, String> {
        if !self.connected {
            return Err("Connect to a robot first via robot_connect.".to_string());
        }
        self.query_mcu_model_info();
        match self.transport.exchange_model(ModelRequest::GetInfo) {
            Some(ModelResponse::Info(info)) => Ok(format!(
                "MCU model '{}' ({:?}, {} B, {} DoF, crc32={:#010x})",
                info.name_str(),
                info.format,
                info.total_bytes,
                info.joint_count,
                info.crc32
            )),
            Some(ModelResponse::NotAvailable) => {
                Err("MCU onboard model store reported NotAvailable.".to_string())
            }
            _ => Err("MCU model query did not receive an Info response.".to_string()),
        }
    }

    fn active_screen_name(&self) -> &'static str {
        match self.active_screen {
            AppScreen::Dashboard => "dashboard",
            AppScreen::Control => "control",
            AppScreen::Config => "config",
        }
    }

    fn navigate_screen(&mut self, screen: &str) -> Result<(), String> {
        let target = match screen {
            "dashboard" => AppScreen::Dashboard,
            "control" => AppScreen::Control,
            "config" => AppScreen::Config,
            other => return Err(format!("unknown screen `{other}`")),
        };
        if target != AppScreen::Dashboard && !self.connected {
            return Err(
                "Connect to a robot first via robot_connect before navigating to Control or Config."
                    .to_string(),
            );
        }
        self.active_screen = target;
        Ok(())
    }

    fn set_camera_preset(&mut self, preset: &str) -> Result<(), String> {
        match preset {
            "fit" => {
                self.viewport.fit_to_rig(&self.model.rig);
                Ok(())
            }
            "iso" | "home" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Iso);
                self.viewport.orbit.animate_to_orientation(q);
                self.viewport.fit_to_rig(&self.model.rig);
                Ok(())
            }
            "front" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Front);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            "back" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Back);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            "left" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Left);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            "right" | "side" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Right);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            "top" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Top);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            "bottom" => {
                let q = self.viewport.orbit.preset_orientation(CameraPreset::Bottom);
                self.viewport.orbit.animate_to_orientation(q);
                Ok(())
            }
            other => Err(format!("unknown camera preset `{other}`")),
        }
    }

    fn set_camera_orbit(&mut self, req: &CameraOrbitRequest) -> Result<(), String> {
        if let Some(preset) = req.preset.as_deref() {
            self.set_camera_preset(preset)?;
        }
        if req.target.is_some()
            || req.distance.is_some()
            || req.yaw_deg.is_some()
            || req.pitch_deg.is_some()
        {
            self.viewport
                .orbit
                .set_custom_orbit(req.target, req.distance, req.yaw_deg, req.pitch_deg);
        }
        Ok(())
    }

    fn viewport_camera_state(&self) -> Option<ViewportCameraState> {
        let eye = self.viewport.orbit.eye_position();
        Some(ViewportCameraState {
            eye: [eye.x, eye.y, eye.z],
            target: self.viewport.orbit.target,
            distance: self.viewport.orbit.distance,
            yaw_rad: self.viewport.orbit.yaw_rad,
            pitch_rad: self.viewport.orbit.pitch_rad,
            fov_deg: rk_viewport::CAMERA_FOV_DEG,
        })
    }

    fn apply_joint_angle(&mut self, joint_idx: usize, rad: f32) -> Result<(), String> {
        if joint_idx >= self.model.rig.joints.len() {
            return Err(format!("joint index {joint_idx} out of range"));
        }
        if self.model.is_estop_active() {
            return Err("E-Stop is active. Reset E-Stop before moving joints.".to_string());
        }
        self.apply_direct_joint_angle(joint_idx, rad);
        Ok(())
    }

    fn set_walk_override(&mut self, vx: f32, vy: f32, yaw_rate: f32, enabled: bool) {
        let vx = vx.clamp(-1.0, 1.0);
        let vy = vy.clamp(-1.0, 1.0);
        let yaw_rate = yaw_rate.clamp(-1.0, 1.0);
        self.model.walk_cmd.vx = vx;
        self.model.walk_cmd.vy = vy;
        self.model.walk_cmd.yaw_rate = yaw_rate;
        self.model.walk_cmd.walk_enabled = enabled;
        if enabled || vx.abs() > 0.01 || vy.abs() > 0.01 || yaw_rate.abs() > 0.01 {
            self.mcp.mcp_walk_override = Some((vx, vy, yaw_rate, enabled));
        } else {
            self.mcp.mcp_walk_override = None;
        }
    }

    fn step_ticks(&mut self, ticks: usize, dt_sec: f32) {
        if ticks == 0 {
            return;
        }
        // Single batched step_fixed call: one deterministic harness/executor for
        // all ticks (N separate calls would corrupt the global mock timer queue).
        // Advance playback by the same total time as the old per-tick path.
        let pkt = self.model.build_next_host_packet(dt_sec * ticks as f32);
        if let Some(telem) = self.transport.step_fixed(
            ticks,
            &pkt,
            (ticks as f32) * FIXED_CONTROL_STEP_SEC,
        ) {
            self.model.ingest_telemetry(telem);
        }
    }

    fn policy(&self) -> McpPolicy {
        self.mcp.policy
    }

    fn set_drive_armed(&mut self, armed: bool) {
        self.mcp.set_drive_armed(armed);
    }

    fn trigger_estop_and_disarm(&mut self) {
        self.mcp.on_estop_disarm();
        self.model.trigger_estop();
    }

    fn reset_estop_upright(&mut self) {
        if let Some(f) = self.transport.sim_faults_mut() {
            f.drop_heartbeat = false;
        }
        self.transport.reset_upright();
        self.model.reset_estop();
    }

    fn reset_experiment(&mut self) -> Result<String, String> {
        if self.transport.kind() != TransportKind::SimKernel {
            return Err("Reset experiment is only supported on the Simulation transport.".to_string());
        }
        // Reset in place so the persistent Embassy executor and MCP bridge
        // channel survive; the transport rebuilds kernel, plant, faults, wire
        // state and recorder from its stored construction inputs.
        self.transport.reset_experiment();
        // A fresh experiment must not inherit a latched fall, an armed recovery,
        // saved gains or operator consent from the previous run.
        self.clear_fall_recovery_state();
        // Clear host command sequencing so scripts feed explicit packets rather
        // than inheriting frame-derived seq values, playback or queued config.
        self.model.reset_experiment_state();
        // Drop host-side history/time too so tuning plots start from zero.
        self.history.clear();
        self.sim_time_sec = 0.0;
        self.sim_control_accum_sec = 0.0;
        self.sim_catchup_cap_hit = false;
        Ok("Simulation experiment reset: kernel, plant, faults, wire, recorder, host sequencing, fall/recovery state and UI history are clean.".to_string())
    }

    fn record_activity(&mut self, entry: McpActivityEntry) {
        self.mcp.last_activity_sim_time = self.sim_time_sec;
        self.mcp.activity.push_back(entry);
        while self.mcp.activity.len() > MAX_ACTIVITY_ENTRIES {
            self.mcp.activity.pop_front();
        }
    }

    fn capture_viewport_screenshot(&mut self) -> Result<(u32, u32, Vec<u8>), String> {
        self.pending_viewport_screenshot.take().unwrap_or_else(|| {
            Err("No rendered 3D viewport frame available for screenshot.".to_string())
        })
    }
}
