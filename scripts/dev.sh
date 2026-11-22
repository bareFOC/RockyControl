#!/bin/sh
set -eu

export PATH="$HOME/.cargo/bin:$PATH"

# If dist/index.html is missing (for example after a clean container restart),
# trigger a background Rust/Trunk rebuild while binding port 3000 immediately.
if [ ! -f dist/index.html ]; then
  echo "[rocky-studio] dist/index.html missing; starting background WASM build..."
  (sh scripts/build.sh > /tmp/rocky-wasm-build.log 2>&1) &
fi

exec python3 scripts/serve_dist.py
