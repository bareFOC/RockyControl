//! Standalone pure-Rust Streamable HTTP MCP server (`POST /mcp`, `GET /mcp` SSE, `DELETE /mcp`).
//!
//! Uses only `std::net::TcpListener` so the native `rk-mcp` binary requires zero external HTTP crates.
//! Requests are either dispatched against a headless host (CLI) or forwarded to a
//! non-`Send` GUI thread through [`McpBridgeTx`] (native desktop app).

use std::collections::HashSet;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use serde_json::{Value, json};

use crate::bridge::{BridgeError, DEFAULT_BRIDGE_REPLY_TIMEOUT, McpBridgeTx};
use crate::headless::HeadlessHost;
use crate::tools::dispatch_json_rpc;

const MAX_HTTP_BODY_BYTES: usize = 2_621_440;
const SOCKET_TIMEOUT: Duration = Duration::from_secs(10);
const LISTENER_POLL_INTERVAL: Duration = Duration::from_millis(20);

static NEXT_SESSION_ID: AtomicU64 = AtomicU64::new(1);

/// Where JSON-RPC messages are executed.
enum Dispatcher {
    /// CLI mode: the host lives on the HTTP worker threads behind its own mutex.
    Headless(Mutex<HeadlessHost>),
    /// Desktop mode: messages are serialized onto the egui thread via the bridge.
    Bridge(McpBridgeTx),
}

struct SharedState {
    /// Kept separate from the dispatcher so no session lock is held while a
    /// request waits for the GUI to dispatch it.
    sessions: Mutex<HashSet<String>>,
    dispatcher: Dispatcher,
}

impl SharedState {
    fn headless(host: HeadlessHost) -> Self {
        Self {
            sessions: Mutex::new(HashSet::new()),
            dispatcher: Dispatcher::Headless(Mutex::new(host)),
        }
    }

    fn bridge(tx: McpBridgeTx) -> Self {
        Self {
            sessions: Mutex::new(HashSet::new()),
            dispatcher: Dispatcher::Bridge(tx),
        }
    }

    /// A single panicking tool must not permanently poison the session table.
    fn sessions(&self) -> MutexGuard<'_, HashSet<String>> {
        self.sessions.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Likewise, recover the headless host after a tool panic.
    fn headless_host(&self) -> Option<MutexGuard<'_, HeadlessHost>> {
        match &self.dispatcher {
            Dispatcher::Headless(host) => Some(host.lock().unwrap_or_else(|e| e.into_inner())),
            Dispatcher::Bridge(_) => None,
        }
    }

    fn dispatch(&self, msg: Value) -> Result<Option<Value>, BridgeError> {
        match &self.dispatcher {
            Dispatcher::Headless(_) => {
                let mut host = self
                    .headless_host()
                    .expect("headless dispatcher always has a host");
                Ok(dispatch_json_rpc(&mut *host, msg))
            }
            Dispatcher::Bridge(tx) => tx.submit(msg, DEFAULT_BRIDGE_REPLY_TIMEOUT),
        }
    }
}

/// Runs the standalone Streamable HTTP MCP server bound to `addr` (e.g. `127.0.0.1:7788`).
/// Blocks forever in the listener loop, preserving the original CLI behavior.
pub fn serve_http(addr: &str, host: HeadlessHost) -> std::io::Result<()> {
    let listener = TcpListener::bind(addr)?;
    eprintln!("[rk-mcp] Streamable HTTP MCP listening on http://{addr}/mcp");
    let state = Arc::new(SharedState::headless(host));
    serve_loop(listener, state, None);
    Ok(())
}

/// Owns a background Streamable HTTP listener bridged to the GUI thread.
///
/// Dropping the handle stops the listener thread. Call [`HttpServerHandle::stop`]
/// explicitly when deterministic shutdown ordering is required.
pub struct HttpServerHandle {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    local_addr: SocketAddr,
}

impl HttpServerHandle {
    /// The address the listener actually bound (useful with port `0`).
    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Signals the listener to stop and joins its thread. Idempotent.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }

    /// Whether the listener thread is still running.
    pub fn is_running(&self) -> bool {
        self.thread.is_some()
    }
}

impl Drop for HttpServerHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Starts a non-blocking Streamable HTTP MCP listener that forwards messages to
/// the GUI through `tx`. Binding happens synchronously so port conflicts are
/// reported to the caller immediately.
pub fn spawn_http_bridge(addr: &str, tx: McpBridgeTx) -> std::io::Result<HttpServerHandle> {
    let listener = TcpListener::bind(addr)?;
    let local_addr = listener.local_addr()?;
    listener.set_nonblocking(true)?;

    let stop = Arc::new(AtomicBool::new(false));
    let stop_loop = Arc::clone(&stop);
    let state = Arc::new(SharedState::bridge(tx));
    let thread = thread::Builder::new()
        .name("rk-mcp-http".to_string())
        .spawn(move || serve_loop(listener, state, Some(stop_loop)))?;

    Ok(HttpServerHandle {
        stop,
        thread: Some(thread),
        local_addr,
    })
}

fn serve_loop(listener: TcpListener, state: Arc<SharedState>, stop: Option<Arc<AtomicBool>>) {
    match stop {
        None => {
            for stream in listener.incoming() {
                let Ok(stream) = stream else {
                    continue;
                };
                let state = Arc::clone(&state);
                thread::spawn(move || {
                    let _ = handle_connection(stream, state);
                });
            }
        }
        Some(stop) => {
            while !stop.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        let state = Arc::clone(&state);
                        thread::spawn(move || {
                            let _ = handle_connection(stream, state);
                        });
                    }
                    Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => {
                        thread::sleep(LISTENER_POLL_INTERVAL);
                    }
                    Err(_) => break,
                }
            }
        }
    }
}

fn handle_connection(mut stream: TcpStream, state: Arc<SharedState>) -> std::io::Result<()> {
    stream.set_read_timeout(Some(SOCKET_TIMEOUT))?;
    stream.set_write_timeout(Some(SOCKET_TIMEOUT))?;

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
            if let Some(sid) = &session_id {
                state.sessions().remove(sid);
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
            // Only touch the session table briefly; never hold it during dispatch,
            // which may block for up to the bridge reply timeout.
            let resp_sid = if is_init || session_id.is_none() {
                let sid = new_session_id();
                state.sessions().insert(sid.clone());
                Some(sid)
            } else {
                session_id.take()
            };

            match state.dispatch(msg) {
                Ok(Some(val)) => write_response(
                    &mut stream,
                    "200 OK",
                    "application/json",
                    resp_sid.as_deref(),
                    val.to_string().as_bytes(),
                ),
                Ok(None) => write_response(
                    &mut stream,
                    "202 Accepted",
                    "application/json",
                    resp_sid.as_deref(),
                    b"",
                ),
                Err(err) => {
                    let (status, code) = match err {
                        BridgeError::Busy => ("503 Service Unavailable", -32001),
                        BridgeError::Timeout => ("504 Gateway Timeout", -32002),
                        BridgeError::GuiClosed => ("503 Service Unavailable", -32003),
                    };
                    let body = json!({
                        "jsonrpc": "2.0",
                        "id": null,
                        "error": {"code": code, "message": err.to_string()}
                    });
                    write_response(
                        &mut stream,
                        status,
                        "application/json",
                        resp_sid.as_deref(),
                        body.to_string().as_bytes(),
                    )
                }
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

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::TcpStream;
    use std::sync::atomic::AtomicUsize;
    use std::thread;
    use std::time::{Duration, Instant};

    use serde_json::{Value, json};

    use crate::AutomationRoots;
    use crate::bridge::{BRIDGE_QUEUE_CAPACITY, mcp_bridge_channel};
    use crate::headless::HeadlessHost;
    use crate::host::McpPolicy;

    use super::*;

    fn raw_request(addr: SocketAddr, method: &str, path: &str, body: &str) -> String {
        let mut stream = TcpStream::connect(addr).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        write!(
            stream,
            "{method} {path} HTTP/1.1\r\nHost: {addr}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        )
        .unwrap();
        let mut response = Vec::new();
        stream.read_to_end(&mut response).unwrap();
        String::from_utf8(response).unwrap()
    }

    /// Drains bridge requests on this (GUI-simulating) thread until `done` is set.
    fn drain_until<F: Fn() -> bool>(rx: &mut crate::bridge::McpBridgeRx, done: F, reply: Value) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline && !done() {
            for req in rx.try_recv_all() {
                let _ = req.reply.send(Some(reply.clone()));
            }
            thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    fn bridge_http_post_initialize_returns_200_json() {
        let (tx, mut rx) = mcp_bridge_channel(|| {});
        let mut handle = spawn_http_bridge("127.0.0.1:0", tx).unwrap();
        let addr = handle.local_addr();
        assert!(handle.is_running());

        let client = thread::spawn(move || {
            raw_request(
                addr,
                "POST",
                "/mcp",
                r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}"#,
            )
        });

        let expected = json!({
            "jsonrpc": "2.0",
            "id": 1,
            "result": {"protocolVersion": "2025-06-18", "serverInfo": {"name": "fake-gui"}}
        });
        let expected_clone = expected.clone();
        drain_until(&mut rx, || client.is_finished(), expected_clone);

        let response = client.join().unwrap();
        assert!(response.starts_with("HTTP/1.1 200 OK"), "{response}");
        assert!(response.to_ascii_lowercase().contains("mcp-session-id:"), "{response}");
        assert!(response.contains("\"protocolVersion\""), "{response}");

        handle.stop();
        assert!(!handle.is_running());
    }

    #[test]
    fn bridge_http_preserves_health_cors_and_session_lifecycle() {
        let (tx, mut rx) = mcp_bridge_channel(|| {});
        let mut handle = spawn_http_bridge("127.0.0.1:0", tx).unwrap();
        let addr = handle.local_addr();

        let health = raw_request(addr, "GET", "/health", "");
        assert!(health.starts_with("HTTP/1.1 200 OK"), "{health}");
        assert!(health.contains("\"ok\":true"), "{health}");

        let options = raw_request(addr, "OPTIONS", "/mcp", "");
        assert!(options.starts_with("HTTP/1.1 204 No Content"), "{options}");
        assert!(options.contains("Access-Control-Allow-Origin: *"), "{options}");

        let client = thread::spawn(move || {
            raw_request(addr, "POST", "/mcp", r#"{"jsonrpc":"2.0","id":2,"method":"ping"}"#)
        });
        drain_until(&mut rx, || client.is_finished(), json!({"jsonrpc":"2.0","id":2,"result":{}}));
        let response = client.join().unwrap();
        assert!(response.starts_with("HTTP/1.1 200 OK"), "{response}");

        let sid = response
            .lines()
            .find_map(|line| line.strip_prefix("Mcp-Session-Id: "))
            .unwrap()
            .trim()
            .to_string();
        assert!(sid.starts_with("rk-mcp-"), "{sid}");

        let delete = raw_request(addr, "DELETE", "/mcp", "");
        assert!(delete.starts_with("HTTP/1.1 204 No Content"), "{delete}");

        handle.stop();
    }

    #[test]
    fn bridge_busy_returns_503() {
        let wakes = Arc::new(AtomicUsize::new(0));
        let wakes_tx = Arc::clone(&wakes);
        let (tx, _rx) = mcp_bridge_channel(move || {
            wakes_tx.fetch_add(1, Ordering::SeqCst);
        });
        let mut handle = spawn_http_bridge("127.0.0.1:0", tx).unwrap();
        let addr = handle.local_addr();

        // No GUI thread drains the queue: saturate it with bounded-capacity requests.
        for i in 0..BRIDGE_QUEUE_CAPACITY {
            thread::spawn(move || {
                raw_request(
                    addr,
                    "POST",
                    "/mcp",
                    &format!(r#"{{"jsonrpc":"2.0","id":{i},"method":"ping"}}"#),
                )
            });
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        while wakes.load(Ordering::SeqCst) < BRIDGE_QUEUE_CAPACITY && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(wakes.load(Ordering::SeqCst), BRIDGE_QUEUE_CAPACITY);

        let response = raw_request(
            addr,
            "POST",
            "/mcp",
            r#"{"jsonrpc":"2.0","id":99,"method":"ping"}"#,
        );
        assert!(response.starts_with("HTTP/1.1 503 Service Unavailable"), "{response}");
        assert!(response.contains("-32001"), "{response}");
        handle.stop();
    }

    #[test]
    fn poisoned_host_mutex_is_recovered() {
        let host = HeadlessHost::new(
            McpPolicy {
                enabled: true,
                drive_armed: false,
            },
            AutomationRoots::default(),
        );
        let state = Arc::new(SharedState::headless(host));
        let poison_state = Arc::clone(&state);
        let _ = thread::spawn(move || {
            let _guard = poison_state.headless_host().unwrap();
            panic!("simulated tool panic");
        })
        .join();
        assert!(state.sessions().is_empty());
        let reply = state.dispatch(json!({"jsonrpc":"2.0","id":9,"method":"initialize"}));
        assert!(matches!(reply, Ok(Some(_))), "dispatch recovered from poison");
    }
}
