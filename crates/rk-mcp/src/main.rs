//! Standalone native Rocky Studio MCP server binary (`rk-mcp`).
//!
//! Default mode serves MCP over stdio via `craft_mcp_server::Server::run_stdio` for
//! Claude Desktop, Cursor, and Gemini CLI. Pass `--http [addr]` to run standalone Streamable HTTP.

#![forbid(unsafe_code)]

use std::path::PathBuf;

use craft_mcp_server::Server;
use rk_link::TransportKind;
use rk_mcp::{
    AutomationRoots, HeadlessHost, McpHost, McpPolicy, RockyMcpTools, server_info, server_options,
};

fn print_usage() {
    eprintln!(
        "Rocky Studio MCP Server (craft-mcp-server) v{}\n\
         \n\
         USAGE:\n\
           rk-mcp [OPTIONS]\n\
         \n\
         OPTIONS:\n\
           --http [ADDR]                  Serve Streamable HTTP on /mcp (default 127.0.0.1:7788)\n\
           --allow-drive                  Arm Agent Drive authority for headless motion/locomotion\n\
           --transport <sim|usb_udp>      Auto-connect transport on startup\n\
           --automation-read-root <DIR>   Confine URDF file_path reads to DIR (craft-contrib)\n\
           --automation-write-root <DIR>  Confine calibration exports to DIR (craft-contrib)\n\
           --help                         Show this help message",
        env!("CARGO_PKG_VERSION")
    );
}

fn main() {
    let mut http_addr: Option<String> = None;
    let mut allow_drive = false;
    let mut auto_transport: Option<TransportKind> = None;
    let mut roots = AutomationRoots::default();

    let mut args = std::env::args().skip(1).peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                print_usage();
                return;
            }
            "--version" => {
                println!("rk-mcp {}", env!("CARGO_PKG_VERSION"));
                return;
            }
            "--allow-drive" => {
                allow_drive = true;
            }
            "--http" => {
                let next_addr = match args.peek() {
                    Some(v) if !v.starts_with('-') => args.next().unwrap(),
                    _ => "127.0.0.1:7788".to_string(),
                };
                http_addr = Some(next_addr);
            }
            "--transport" => {
                if let Some(val) = args.next() {
                    auto_transport = match val.as_str() {
                        "sim" => Some(TransportKind::SimKernel),
                        "usb_udp" => Some(TransportKind::UsbNetworkingUdp),
                        other => {
                            eprintln!("[rk-mcp] Unknown transport '{other}'; expected sim or usb_udp");
                            std::process::exit(2);
                        }
                    };
                }
            }
            "--automation-read-root" => {
                if let Some(val) = args.next() {
                    roots.read_root = Some(PathBuf::from(val));
                }
            }
            "--automation-write-root" => {
                if let Some(val) = args.next() {
                    roots.write_root = Some(PathBuf::from(val));
                }
            }
            other => {
                eprintln!("[rk-mcp] Unknown option '{other}'");
                print_usage();
                std::process::exit(2);
            }
        }
    }

    let policy = McpPolicy {
        enabled: true,
        drive_armed: allow_drive,
    };
    let mut host = HeadlessHost::new(policy, roots);
    if let Some(kind) = auto_transport {
        if let Err(err) = host.connect(kind) {
            eprintln!("[rk-mcp] Initial connect failed: {err}");
        }
    }

    if let Some(addr) = http_addr {
        if let Err(err) = rk_mcp::http_server::serve_http(&addr, host) {
            eprintln!("[rk-mcp] HTTP server error: {err}");
            std::process::exit(1);
        }
    } else {
        let mut srv = Server::new(server_info(), RockyMcpTools::new(&mut host))
            .with_options(server_options());
        let stdin = std::io::stdin();
        let stdout = std::io::stdout();
        let reader = stdin.lock();
        let writer = stdout.lock();
        if let Err(err) = srv.serve(reader, writer) {
            eprintln!("[rk-mcp] stdio server error: {err}");
            std::process::exit(1);
        }
    }
}
