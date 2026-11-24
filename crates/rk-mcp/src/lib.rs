//! Layer 4.5 — Rocky Studio Model Context Protocol (`rk-mcp`).
//!
//! Built directly on `craft-mcp-server` (`storytold/craft-libs`) following `storytold/craft-contrib`
//! guidelines:
//! - Shared `RockyMcpTools<H: McpHost>` compiles identically to WASM32 (live browser 3D viewport)
//!   and native Rust (standalone `rk-mcp` stdio / Streamable HTTP server).
//! - Exposes complete robot configuration, connection, calibration, UVC balance tuning, FK/IK
//!   posing, walking locomotion, combat actions, shove testing, and deterministic Copper replay
//!   via annotated MCP Tools, Resources, Resource Templates, Prompts, and Completions.
//! - Enforces Guarded Autonomy (`mcp_enabled`, `drive_armed`, and absolute E-Stop override).

#![forbid(unsafe_code)]

#[cfg(not(target_arch = "wasm32"))]
pub mod bridge;
pub mod headless;
pub mod host;
#[cfg(not(target_arch = "wasm32"))]
pub mod http_server;
pub mod schema;
pub mod tools;

pub use craft_mcp_server::{Info, Options, Server};
#[cfg(not(target_arch = "wasm32"))]
pub use bridge::{
    BridgeError, DEFAULT_BRIDGE_REPLY_TIMEOUT, McpBridgeRequest, McpBridgeRx, McpBridgeTx,
    mcp_bridge_channel,
};
pub use headless::{AutomationRoots, HeadlessHost};
pub use host::{McpActivityEntry, McpCallOutcome, McpHost, McpPolicy};
#[cfg(not(target_arch = "wasm32"))]
pub use http_server::{HttpServerHandle, serve_http, spawn_http_bridge};
pub use tools::{RockyMcpTools, dispatch_json_rpc, server_info, server_options};

/// Tests that step the simulator share the process-global mock clock, so they
/// must serialize here. The fixed-step path no longer resets that clock, which
/// would strand the persistent control task's timer alarm.
#[cfg(test)]
pub(crate) mod test_support {
    pub(crate) static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
}
