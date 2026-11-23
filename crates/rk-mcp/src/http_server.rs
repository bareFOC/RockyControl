//! Standalone pure-Rust Streamable HTTP MCP server (`POST /mcp`, `GET /mcp` SSE, `DELETE /mcp`).
//!
//! Uses only `std::net::TcpListener` so the native `rk-mcp` binary requires zero external HTTP crates.

use std::collections::HashSet;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use serde_json::{Value, json};

use crate::headless::HeadlessHost;
use crate::tools::dispatch_json_rpc;

const MAX_HTTP_BODY_BYTES: usize = 2_621_440;

static NEXT_SESSION_ID: AtomicU64 = AtomicU64::new(1);

struct SharedState {
    host: HeadlessHost,
    sessions: HashSet<String>,
}

/// Runs the standalone Streamable HTTP MCP server bound to `addr` (e.g. `127.0.0.1:7788`).
pub fn serve_http(addr: &str, host: HeadlessHost) -> std::io::Result<()> {
    let listener = TcpListener::bind(addr)?;
    eprintln!("[rk-mcp] Streamable HTTP MCP listening on http://{addr}/mcp");

    let state = Arc::new(Mutex::new(SharedState {
        host,
        sessions: HashSet::new(),
    }));

    for stream in listener.incoming() {
        let Ok(stream) = stream else {
            continue;
        };
        let state_clone = Arc::clone(&state);
        thread::spawn(move || {
            let _ = handle_connection(stream, state_clone);
        });
    }
    Ok(())
}

fn handle_connection(mut stream: TcpStream, state: Arc<Mutex<SharedState>>) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut request_line = String::new();
    if reader.read_line(&mut request_line)? == 0 {
        return Ok(());
    }
    let parts: Vec<&str> = request_line.split_whitespace().collect();
    if parts.len() < 2 {
        return write_response(&mut stream, "400 Bad Request", "application/json", None, b"{\"error\":\"bad request\"}");
    }
    let method = parts[0];
    let path = parts[1].split('?').next().unwrap_or(parts[1]);

    let mut content_length: usize = 0;
    let mut session_id: Option<String> = None;
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 || line == "\r\n" || line == "\n" {
            break;
        }
        if let Some((k, v)) = line.split_once(':') {
            let key = k.trim().to_ascii_lowercase();
            let val = v.trim();
            if key == "content-length" {
                content_length = val.parse().unwrap_or(0);
            } else if key == "mcp-session-id" {
                session_id = Some(val.to_string());
            }
        }
    }

    if path == "/health" && method == "GET" {
        return write_response(
            &mut stream,
            "200 OK",
            "application/json",
            None,
            b"{\"ok\":true,\"server\":\"rocky-studio\"}",
        );
    }

    if path != "/mcp" {
        return write_response(
            &mut stream,
            "404 Not Found",
            "application/json",
            None,
            b"{\"error\":\"not found; use /mcp\"}",
        );
    }

    match method {
        "OPTIONS" => {
            let hdr = "HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type, Accept, Mcp-Session-Id, MCP-Protocol-Version\r\nAccess-Control-Expose-Headers: Mcp-Session-Id\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            stream.write_all(hdr.as_bytes())
        }
        "DELETE" => {
            if let Some(sid) = session_id {
                if let Ok(mut guard) = state.lock() {
                    guard.sessions.remove(&sid);
                }
            }
            write_response(&mut stream, "204 No Content", "text/plain", None, b"")
        }
        "GET" => {
            // Streamable HTTP standby SSE notification stream
            let sid = session_id.unwrap_or_else(new_session_id);
            let hdr = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nMcp-Session-Id: {sid}\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: Mcp-Session-Id\r\nConnection: close\r\n\r\n: rocky-studio mcp ready\n\n"
            );
            stream.write_all(hdr.as_bytes())
        }
        "POST" => {
            if content_length > MAX_HTTP_BODY_BYTES {
                let err = json!({"jsonrpc":"2.0","id":null,"error":{"code":-32600,"message":"request too large"}});
                return write_response(
                    &mut stream,
                    "413 Payload Too Large",
                    "application/json",
                    None,
                    err.to_string().as_bytes(),
                );
            }
            let mut body = vec![0u8; content_length];
            reader.read_exact(&mut body)?;

            let parsed: Result<Value, _> = serde_json::from_slice(&body);
            let msg = match parsed {
                Ok(v) => v,
                Err(e) => {
                    let err = json!({"jsonrpc":"2.0","id":null,"error":{"code":-32700,"message":format!("parse error: {e}")}});
                    return write_response(
                        &mut stream,
                        "400 Bad Request",
                        "application/json",
                        None,
                        err.to_string().as_bytes(),
                    );
                }
            };

            let is_init = msg.get("method").and_then(Value::as_str) == Some("initialize");
            let mut guard = match state.lock() {
                Ok(g) => g,
                Err(_) => {
                    return write_response(
                        &mut stream,
                        "500 Internal Server Error",
                        "application/json",
                        None,
                        b"{\"error\":\"lock poisoned\"}",
                    );
                }
            };

            let resp_sid = if is_init || session_id.is_none() {
                let sid = new_session_id();
                guard.sessions.insert(sid.clone());
                Some(sid)
            } else {
                session_id
            };

            let reply = dispatch_json_rpc(&mut guard.host, msg);
            drop(guard);

            match reply {
                Some(val) => {
                    let bytes = val.to_string();
                    write_response(
                        &mut stream,
                        "200 OK",
                        "application/json",
                        resp_sid.as_deref(),
                        bytes.as_bytes(),
                    )
                }
                None => write_response(
                    &mut stream,
                    "202 Accepted",
                    "application/json",
                    resp_sid.as_deref(),
                    b"",
                ),
            }
        }
        _ => write_response(
            &mut stream,
            "405 Method Not Allowed",
            "application/json",
            None,
            b"{\"error\":\"method not allowed\"}",
        ),
    }
}

fn new_session_id() -> String {
    let n = NEXT_SESSION_ID.fetch_add(1, Ordering::Relaxed);
    format!("rk-mcp-{n:08x}")
}

fn write_response(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    session_id: Option<&str>,
    body: &[u8],
) -> std::io::Result<()> {
    let mut header = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: Mcp-Session-Id\r\nConnection: close\r\n",
        body.len()
    );
    if let Some(sid) = session_id {
        header.push_str(&format!("Mcp-Session-Id: {sid}\r\n"));
    }
    header.push_str("\r\n");
    stream.write_all(header.as_bytes())?;
    if !body.is_empty() {
        stream.write_all(body)?;
    }
    stream.flush()
}
