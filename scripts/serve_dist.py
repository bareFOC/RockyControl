#!/usr/bin/env python3
"""Minimal stdlib HTTP server for Rocky Studio's Rust/WASM dist/ bundle."""

from html import escape
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import mimetypes
import os
from pathlib import Path
import sys

ROOT_DIR = Path(__file__).resolve().parent.parent
DIST_DIR = ROOT_DIR / "dist"
BUILD_LOG = Path("/tmp/rocky-wasm-build.log")
HOST = os.environ.get("ROCKY_HOST", "0.0.0.0")
PORT = int(os.environ.get("ROCKY_PORT", "3000"))

mimetypes.add_type("application/wasm", ".wasm")
mimetypes.add_type("application/javascript", ".js")
mimetypes.add_type("application/javascript", ".mjs")
mimetypes.add_type("model/stl", ".stl")


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
        sys.stdout.write(f"[rocky-server] {self.address_string()} - {format % args}\n")
        sys.stdout.flush()

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

    def do_GET(self):
        if not (DIST_DIR / "index.html").is_file():
            self._serve_building_page(head_only=False)
            return
        super().do_GET()

    def do_HEAD(self):
        if not (DIST_DIR / "index.html").is_file():
            self._serve_building_page(head_only=True)
            return
        super().do_HEAD()


def main():
    DIST_DIR.mkdir(parents=True, exist_ok=True)
    server = ReusableThreadingHTTPServer((HOST, PORT), RockyHandler)
    sys.stdout.write(f"[rocky-server] Listening on http://{HOST}:{PORT} (serving {DIST_DIR})\n")
    sys.stdout.flush()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
