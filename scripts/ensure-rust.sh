#!/bin/sh
set -eu

export PATH="$HOME/.cargo/bin:$PATH"

LOCK_DIR="/tmp/rocky-rust-bootstrap.lock"
TRUNK_VERSION="0.21.14"

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

if ! command -v cargo >/dev/null 2>&1 || ! command -v rustc >/dev/null 2>&1; then
  echo "[rocky-studio] Installing Rust toolchain (stable + wasm32-unknown-unknown)..."
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --default-toolchain stable --profile minimal --target wasm32-unknown-unknown
  export PATH="$HOME/.cargo/bin:$PATH"
fi

if command -v rustup >/dev/null 2>&1; then
  if ! rustup target list --installed | grep -q '^wasm32-unknown-unknown$'; then
    echo "[rocky-studio] Adding wasm32-unknown-unknown target..."
    rustup target add wasm32-unknown-unknown
  fi
fi

if ! command -v trunk >/dev/null 2>&1; then
  echo "[rocky-studio] Installing static trunk v${TRUNK_VERSION}..."
  mkdir -p "$HOME/.cargo/bin"
  tmp_dir="$(mktemp -d)"
  arch="$(uname -m)"
  case "$arch" in
    x86_64) trunk_target="x86_64-unknown-linux-musl" ;;
    aarch64|arm64) trunk_target="aarch64-unknown-linux-gnu" ;;
    *)
      echo "[rocky-studio] Unsupported architecture for prebuilt trunk: $arch" >&2
      rm -rf "$tmp_dir"
      exit 1
      ;;
  esac
  trunk_url="https://github.com/trunk-rs/trunk/releases/download/v${TRUNK_VERSION}/trunk-${trunk_target}.tar.gz"
  curl -sSfL "$trunk_url" | tar -xz -C "$tmp_dir"
  mv "$tmp_dir/trunk" "$HOME/.cargo/bin/trunk"
  chmod +x "$HOME/.cargo/bin/trunk"
  rm -rf "$tmp_dir"
fi

echo "[rocky-studio] Toolchain ready: $(rustc --version) | $(trunk --version)"
