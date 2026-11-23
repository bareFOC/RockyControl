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

pub mod headless;
pub mod host;
#[cfg(not(target_arch = "wasm32"))]
pub mod http_server;
pub mod schema;
pub mod tools;

pub use craft_mcp_server::{Info, Options, Server};
pub use headless::{AutomationRoots, HeadlessHost};
pub use host::{McpActivityEntry, McpCallOutcome, McpHost, McpPolicy};
pub use tools::{RockyMcpTools, dispatch_json_rpc, server_info, server_options};
