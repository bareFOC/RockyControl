#!/bin/sh
set -eu

export PATH="$HOME/.cargo/bin:$PATH"
sh scripts/ensure-rust.sh

cargo check -p humanoid-proto -p humanoid-control -p humanoid-cu-nodes --no-default-features
