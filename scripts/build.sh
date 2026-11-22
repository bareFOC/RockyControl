#!/bin/sh
set -eu

export PATH="$HOME/.cargo/bin:$PATH"

LOCK_DIR="/tmp/rocky-wasm-build.lock"
acquire_lock() {
  while ! mkdir "$LOCK_DIR" 2>/dev/null; do
    if [ -f "$LOCK_DIR/pid" ]; then
      holder_pid="$(cat "$LOCK_DIR/pid" 2>/dev/null || true)"
      if [ -n "$holder_pid" ] && ! kill -0 "$holder_pid" 2>/dev/null; then
        rm -rf "$LOCK_DIR"
        continue
      fi
    fi
    sleep 1
  done
  echo "$$" > "$LOCK_DIR/pid"
}
release_lock() {
  rm -rf "$LOCK_DIR"
}

acquire_lock
trap release_lock EXIT INT TERM

sh scripts/ensure-rust.sh

echo "[rocky-studio] Building Rust egui WASM bundle..."
rm -rf dist.staging
if [ -n "${PUBLIC_URL:-}" ]; then
  CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-2}" trunk build --release --dist dist.staging --public-url "$PUBLIC_URL"
else
  CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-2}" trunk build --release --dist dist.staging
fi

if [ ! -f dist.staging/index.html ]; then
  echo "[rocky-studio] Build finished without dist.staging/index.html" >&2
  exit 1
fi

touch dist.staging/.nojekyll

rm -rf dist.prev
if [ -d dist ]; then
  mv dist dist.prev
fi
mv dist.staging dist
rm -rf dist.prev

echo "[rocky-studio] WASM build ready in dist/"
