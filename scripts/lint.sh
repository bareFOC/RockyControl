#!/bin/sh
set -eu

export PATH="$HOME/.cargo/bin:$PATH"
sh scripts/ensure-rust.sh

cargo check -p rk-proto -p rk-control --no-default-features
