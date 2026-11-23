#!/usr/bin/env python3
"""Stdlib HTTP server for Rocky Studio's Rust/WASM dist/ bundle + Streamable HTTP MCP (/mcp)."""

from collections import deque
from html import escape
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse
import uuid

ROOT_DIR = Path(__file__).resolve().parent.parent
DIST_DIR = ROOT_DIR / "dist"
BUILD_LOG = Path("/tmp/rocky-wasm-build.log")
HOST = os.environ.get("ROCKY_HOST", "0.0.0.0")
PORT = int(os.environ.get("ROCKY_PORT", "3000"))

MAX_MCP_BODY_BYTES = 2_621_440
BROWSER_ALIVE_WINDOW_SEC = 4.0
BROWSER_REPLY_TIMEOUT_SEC = 8.0

mimetypes.add_type("application/wasm", ".wasm")
mimetypes.add_type("application/javascript", ".js")
mimetypes.add_type("application/javascript", ".mjs")
mimetypes.add_type("model/stl", ".stl")


def find_rk_mcp_binary() -> Path | None:
    for rel in ("target/release/rk-mcp", "target/debug/rk-mcp"):
        cand = ROOT_DIR / rel
        if cand.is_file() and os.access(cand, os.X_OK):
            return cand
    return None


class HeadlessMcpWorker:
    """Manages a persistent native `rk-mcp` stdio subprocess for headless `/mcp` requests."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._proc: subprocess.Popen[str] | None = None

    def _ensure_started(self) -> subprocess.Popen[str]:
        if self._proc is not None and self._proc.poll() is None:
            return self._proc
        binary = find_rk_mcp_binary()
        if binary is None:
            raise RuntimeError(
                "No open Rocky Studio browser tab and native rk-mcp binary is not built yet. "
                "Open Rocky Studio in the browser or build rk-mcp (`cargo build -p rk-mcp`)."
            )
        self._proc = subprocess.Popen(
            [str(binary)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
            text=True,
            bufsize=1,
            cwd=str(ROOT_DIR),
        )
        return self._proc

    def call(self, payload: dict) -> dict | None:
        is_notification = "id" not in payload
        with self._lock:
            proc = self._ensure_started()
            assert proc.stdin is not None and proc.stdout is not None
            try:
                proc.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
                proc.stdin.flush()
                if is_notification:
                    return None
                line = proc.stdout.readline()
                if not line:
                    self._reset_locked()
                    raise RuntimeError("Headless rk-mcp worker exited unexpectedly")
                return json.loads(line)
            except Exception:
                self._reset_locked()
                raise

    def _reset_locked(self) -> None:
        if self._proc is not None:
            try:
                self._proc.terminate()
            except OSError:
                pass
            self._proc = None

    def shutdown(self) -> None:
        with self._lock:
            self._reset_locked()


class McpBridgeHub:
    """Routes Streamable HTTP `/mcp` requests live to the browser UI tab (with headless fallback)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._poll_cv = threading.Condition(self._lock)
        self.enabled: bool = True
        self.drive_armed: bool = False
        self.last_poll_ts: float = 0.0
        self._queue: deque[tuple[str, dict]] = deque()
        self._waiters: dict[str, tuple[threading.Event, dict]] = {}
        self._sessions: set[str] = set()
        self.headless = HeadlessMcpWorker()

    def is_browser_connected(self) -> bool:
        with self._lock:
            return (time.monotonic() - self.last_poll_ts) <= BROWSER_ALIVE_WINDOW_SEC

    def status_dict(self) -> dict:
        with self._lock:
            browser_alive = (time.monotonic() - self.last_poll_ts) <= BROWSER_ALIVE_WINDOW_SEC
            return {
                "enabled": self.enabled,
                "drive_armed": self.drive_armed,
                "browser_connected": browser_alive,
                "headless_binary": str(find_rk_mcp_binary()) if find_rk_mcp_binary() else None,
                "mcp_endpoint": "/mcp",
            }

    def update_config(self, enabled: bool | None, drive_armed: bool | None) -> dict:
        with self._lock:
            if enabled is not None:
                self.enabled = bool(enabled)
            if drive_armed is not None:
                self.drive_armed = bool(drive_armed)
        return self.status_dict()

    def ensure_session(self, incoming_id: str | None) -> str:
        with self._lock:
            if incoming_id and incoming_id in self._sessions:
                return incoming_id
            sid = incoming_id or f"rocky-{uuid.uuid4().hex[:16]}"
            self._sessions.add(sid)
            return sid

    def close_session(self, sid: str | None) -> None:
        if not sid:
            return
        with self._lock:
            self._sessions.discard(sid)

    def poll_next(self, wait_sec: float) -> tuple[str, dict] | None:
        deadline = time.monotonic() + wait_sec
        with self._poll_cv:
            self.last_poll_ts = time.monotonic()
            while not self._queue:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._poll_cv.wait(timeout=remaining)
            return self._queue.popleft()

    def complete_reply(self, req_id: str, response: dict | None) -> bool:
        with self._lock:
            self.last_poll_ts = time.monotonic()
            waiter = self._waiters.pop(req_id, None)
        if waiter is None:
            return False
        evt, box = waiter
        box["response"] = response
        evt.set()
        return True

    def dispatch_one(self, payload: dict) -> dict | None:
        with self._lock:
            enabled = self.enabled
            browser_alive = (time.monotonic() - self.last_poll_ts) <= BROWSER_ALIVE_WINDOW_SEC

        if not enabled:
            if "id" not in payload:
                return None
            return {
                "jsonrpc": "2.0",
                "id": payload.get("id"),
                "error": {
                    "code": -32000,
                    "message": "MCP access is disabled in Rocky Studio. Enable MCP on the Home screen.",
                },
            }

        if browser_alive:
            req_id = uuid.uuid4().hex
            evt = threading.Event()
            box: dict = {"response": None}
            with self._poll_cv:
                self._waiters[req_id] = (evt, box)
                self._queue.append((req_id, payload))
                self._poll_cv.notify()
            if evt.wait(timeout=BROWSER_REPLY_TIMEOUT_SEC):
                return box["response"]
            with self._lock:
                self._waiters.pop(req_id, None)
            if "id" not in payload:
                return None
            return {
                "jsonrpc": "2.0",
                "id": payload.get("id"),
                "error": {
                    "code": -32001,
                    "message": "Rocky Studio browser tab did not reply in time. Retry or ensure the tab is active.",
                },
            }

        try:
            return self.headless.call(payload)
        except Exception as exc:
            if "id" not in payload:
                return None
            return {
                "jsonrpc": "2.0",
                "id": payload.get("id"),
                "error": {
                    "code": -32603,
                    "message": str(exc),
                },
            }


MCP_HUB = McpBridgeHub()


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class RockyHandler(SimpleHTTPRequestHandler):
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".wasm": "application/wasm",
        ".js": "application/javascript",
        ".mjs": "application/javascript",
        ".css": "text/css; charset=utf-8",
        ".html": "text/html; charset=utf-8",
        ".svg": "image/svg+xml",
        ".json": "application/json",
        ".stl": "model/stl",
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIST_DIR), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, format, *args):
        # Suppress noisy browser long-poll requests from stdout
        path = getattr(self, "path", "")
        if path.startswith("/api/mcp/bridge/poll"):
            return
        sys.stdout.write(f"[rocky-server] {self.address_string()} - {format % args}\n")
        sys.stdout.flush()

    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header(
            "Access-Control-Allow-Headers",
            "Content-Type, Accept, Mcp-Session-Id, MCP-Protocol-Version",
        )
        self.send_header(
            "Access-Control-Expose-Headers",
            "Mcp-Session-Id, MCP-Protocol-Version",
        )

    def _send_json(
        self,
        status: HTTPStatus,
        payload: object,
        extra_headers: dict[str, str] | None = None,
    ) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._send_cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _send_empty(
        self,
        status: HTTPStatus,
        extra_headers: dict[str, str] | None = None,
    ) -> None:
        self.send_response(status)
        self._send_cors_headers()
        self.send_header("Content-Length", "0")
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()

    def _read_json_body(self) -> tuple[object | None, str | None]:
        raw_len = self.headers.get("Content-Length", "0")
        try:
            length = int(raw_len)
        except ValueError:
            return None, "Invalid Content-Length header"
        if length < 0 or length > MAX_MCP_BODY_BYTES:
            return None, f"Request body exceeds {MAX_MCP_BODY_BYTES} byte limit"
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            return json.loads(raw.decode("utf-8")), None
        except Exception as exc:
            return None, f"Invalid JSON payload: {exc}"

    def _serve_building_page(self, head_only: bool = False):
        tail = "Initializing Rust toolchain and Trunk WASM build..."
        if BUILD_LOG.exists():
            try:
                lines = BUILD_LOG.read_text(errors="replace").splitlines()
                if lines:
                    tail = "\n".join(lines[-20:])
            except OSError:
                pass
        body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta http-equiv="refresh" content="3" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Rocky Studio — Building WASM</title>
  <style>
    html, body {{
      margin: 0; padding: 24px; height: 100%; box-sizing: border-box;
      background: #0b0e14; color: #cbd5e1;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    }}
    h1 {{ font-size: 16px; font-weight: 600; margin: 0 0 8px 0; color: #f8fafc; }}
    p {{ font-size: 13px; margin: 0 0 16px 0; color: #94a3b8; }}
    pre {{
      background: #111827; border: 1px solid #1f2937; border-radius: 6px;
      padding: 12px; font-size: 12px; line-height: 1.5; overflow: auto;
      max-height: 70vh; white-space: pre-wrap;
    }}
  </style>
</head>
<body>
  <h1>Rocky Studio (Rust egui + WebGL2 WASM)</h1>
  <p>Rebuilding WASM bundle after container startup. This page refreshes automatically.</p>
  <pre>{escape(tail)}</pre>
</body>
</html>
""".encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def do_OPTIONS(self):
        parsed = urlparse(self.path)
        if parsed.path == "/mcp" or parsed.path.startswith("/api/mcp"):
            self._send_empty(HTTPStatus.NO_CONTENT)
            return
        self._send_empty(HTTPStatus.NO_CONTENT)

    def do_DELETE(self):
        parsed = urlparse(self.path)
        if parsed.path == "/mcp":
            sid = self.headers.get("Mcp-Session-Id")
            MCP_HUB.close_session(sid)
            self._send_empty(HTTPStatus.NO_CONTENT)
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found"})

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/mcp":
            self._handle_mcp_post()
            return
        if parsed.path == "/api/mcp/bridge/config":
            body, err = self._read_json_body()
            if err or not isinstance(body, dict):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": err or "Expected JSON object"})
                return
            status = MCP_HUB.update_config(
                body.get("enabled") if "enabled" in body else None,
                body.get("drive_armed") if "drive_armed" in body else None,
            )
            self._send_json(HTTPStatus.OK, status)
            return
        if parsed.path == "/api/mcp/bridge/reply":
            body, err = self._read_json_body()
            if err or not isinstance(body, dict):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": err or "Expected JSON object"})
                return
            req_id = body.get("id")
            if not isinstance(req_id, str):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "Missing string id"})
                return
            matched = MCP_HUB.complete_reply(req_id, body.get("response"))
            self._send_json(HTTPStatus.OK, {"ok": matched})
            return

        self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found"})

    def _handle_mcp_post(self) -> None:
        body, err = self._read_json_body()
        if err:
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "jsonrpc": "2.0",
                    "id": None,
                    "error": {"code": -32700, "message": err},
                },
            )
            return

        sid = MCP_HUB.ensure_session(self.headers.get("Mcp-Session-Id"))
        extra = {
            "Mcp-Session-Id": sid,
            "MCP-Protocol-Version": "2025-06-18",
        }

        if isinstance(body, list):
            if not body:
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {
                        "jsonrpc": "2.0",
                        "id": None,
                        "error": {"code": -32600, "message": "Empty JSON-RPC batch"},
                    },
                    extra,
                )
                return
            replies = []
            for item in body:
                if isinstance(item, dict):
                    res = MCP_HUB.dispatch_one(item)
                    if res is not None:
                        replies.append(res)
            if not replies:
                self._send_empty(HTTPStatus.ACCEPTED, extra)
            else:
                self._send_json(HTTPStatus.OK, replies, extra)
            return

        if not isinstance(body, dict):
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "jsonrpc": "2.0",
                    "id": None,
                    "error": {"code": -32600, "message": "Expected JSON-RPC object"},
                },
                extra,
            )
            return

        reply = MCP_HUB.dispatch_one(body)
        if reply is None:
            self._send_empty(HTTPStatus.ACCEPTED, extra)
            return
        self._send_json(HTTPStatus.OK, reply, extra)

    def _handle_mcp_sse(self) -> None:
        sid = MCP_HUB.ensure_session(self.headers.get("Mcp-Session-Id"))
        self.send_response(HTTPStatus.OK)
        self._send_cors_headers()
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Connection", "keep-alive")
        self.send_header("Mcp-Session-Id", sid)
        self.send_header("MCP-Protocol-Version", "2025-06-18")
        self.end_headers()
        try:
            self.wfile.write(b": rocky-studio-mcp-sse-ready\n\n")
            self.wfile.flush()
            # Keep the SSE stream open cleanly with periodic comments until client closes
            for _ in range(120):
                time.sleep(15.0)
                self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/mcp":
            accept = self.headers.get("Accept", "")
            if "text/event-stream" in accept:
                self._handle_mcp_sse()
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "name": "rocky-studio",
                    "title": "Rocky Studio Robotics MCP",
                    "protocol": "2025-06-18",
                    "transport": "streamable-http",
                    **MCP_HUB.status_dict(),
                },
            )
            return
        if parsed.path == "/api/mcp/status":
            self._send_json(HTTPStatus.OK, MCP_HUB.status_dict())
            return
        if parsed.path == "/api/mcp/bridge/poll":
            qs = parse_qs(parsed.query)
            try:
                wait_ms = int(qs.get("wait_ms", ["250"])[0])
            except ValueError:
                wait_ms = 250
            wait_sec = max(0.0, min(wait_ms / 1000.0, 1.0))
            item = MCP_HUB.poll_next(wait_sec)
            if item is None:
                self._send_empty(HTTPStatus.NO_CONTENT)
                return
            req_id, payload = item
            self._send_json(HTTPStatus.OK, {"id": req_id, "payload": payload})
            return

        if not (DIST_DIR / "index.html").is_file():
            self._serve_building_page(head_only=False)
            return
        super().do_GET()

    def do_HEAD(self):
        parsed = urlparse(self.path)
        if parsed.path == "/mcp" or parsed.path.startswith("/api/mcp"):
            self._send_empty(HTTPStatus.OK)
            return
        if not (DIST_DIR / "index.html").is_file():
            self._serve_building_page(head_only=True)
            return
        super().do_HEAD()


def main():
    DIST_DIR.mkdir(parents=True, exist_ok=True)
    server = ReusableThreadingHTTPServer((HOST, PORT), RockyHandler)
    sys.stdout.write(
        f"[rocky-server] Listening on http://{HOST}:{PORT} (serving {DIST_DIR}, MCP at /mcp)\n"
    )
    sys.stdout.flush()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        MCP_HUB.headless.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
