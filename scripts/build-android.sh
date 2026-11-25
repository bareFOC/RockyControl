#!/usr/bin/env bash
# Build (and optionally install) the Rocky Studio Android APK.
#
# Usage:
#   bash scripts/build-android.sh [OPTIONS]
#
# Options:
#   --check     Only cross-check the crates for aarch64-linux-android
#               (plain `cargo check`, no linker and no APK produced).
#   --release   Build the release profile. This is the default; the flag is
#               accepted for explicitness.
#   --debug     Build the dev profile instead of release (faster, still signed
#               with the local debug key).
#   --install   After a successful build, run `adb install -r <apk>`.
#   -h, --help  Show this help and exit.
#
# Environment (all optional; defaults match the verified local toolchain):
#   ANDROID_HOME                          Android SDK root ($HOME/Android/Sdk)
#   ANDROID_NDK_ROOT                      NDK root ($ANDROID_HOME/ndk/27.1.12297006)
#   NDK_TOOLCHAIN                         NDK LLVM prebuilt dir
#   CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER
#                                         clang wrapper used for linking
#   ANDROID_USER_HOME                     dir holding debug.keystore ($HOME/.android)
#   CARGO_APK_RELEASE_KEYSTORE / ..._PASSWORD
#                                         override the release signing key
#   CARGO_TARGET_DIR                      custom cargo target directory
#
# cargo-apk 0.10 always signs the final APK. For release builds with no key
# configured we fall back to the standard Android debug keystore (created with
# `keytool` if missing) so a locally installable APK is still produced. This is
# a dev/debug artifact, not a distributable release signing setup.
#
# Examples:
#   bash scripts/build-android.sh
#   bash scripts/build-android.sh --check
#   bash scripts/build-android.sh --install

set -euo pipefail

TRIPLE="aarch64-linux-android"
PACKAGE="rk-android"
APK_NAME="rk-android"

die() {
  echo "[android] ERROR: $*" >&2
  exit 1
}

usage() {
  sed -n '2,/^set -euo pipefail$/p' "${BASH_SOURCE[0]}" | sed '$d'
}

CHECK_ONLY=0
INSTALL=0
PROFILE="release"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --release) PROFILE="release" ;;
    --debug) PROFILE="dev" ;;
    --install) INSTALL=1 ;;
    -h | --help)
      usage
      exit 0
      ;;
    *) die "unknown option: $1 (try --help)" ;;
  esac
  shift
done

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ---- SDK / NDK environment (overridable) ----------------------------------
export ANDROID_HOME="${ANDROID_HOME:-$HOME/Android/Sdk}"
export ANDROID_NDK_ROOT="${ANDROID_NDK_ROOT:-$ANDROID_HOME/ndk/27.1.12297006}"
export NDK_TOOLCHAIN="${NDK_TOOLCHAIN:-$ANDROID_NDK_ROOT/toolchains/llvm/prebuilt/linux-x86_64}"
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="${CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER:-$NDK_TOOLCHAIN/bin/aarch64-linux-android33-clang}"

[[ -d "$ANDROID_HOME" ]] || die "ANDROID_HOME is not a directory: $ANDROID_HOME"
[[ -d "$ANDROID_NDK_ROOT" ]] || die "ANDROID_NDK_ROOT is not a directory: $ANDROID_NDK_ROOT"
[[ -d "$NDK_TOOLCHAIN" ]] || die "NDK_TOOLCHAIN is not a directory: $NDK_TOOLCHAIN"
[[ -f "$CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER" ]] || die "linker not found: $CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER"
[[ -f "$ANDROID_HOME/platforms/android-35/android.jar" ]] ||
  die "Android platform 35 is missing; install it with: sdkmanager \"platforms;android-35\""

if command -v rustup >/dev/null 2>&1; then
  if ! rustup target list --installed | grep -qx "$TRIPLE"; then
    die "Rust target $TRIPLE is not installed; run: rustup target add $TRIPLE"
  fi
fi

if ! cargo apk build --help >/dev/null 2>&1; then
  die "cargo-apk is not installed; run: cargo install cargo-apk --version 0.10.0"
fi

TARGET_DIR="${CARGO_TARGET_DIR:-target}"
if [[ "$TARGET_DIR" != /* ]]; then
  TARGET_DIR="$REPO_ROOT/$TARGET_DIR"
fi

if [[ "$PROFILE" == "release" ]]; then
  PROFILE_DIR="release"
  PROFILE_ARGS=(--release)
else
  # cargo-apk 0.10 uses the profile's Display name ("dev") for its APK
  # directory while cargo itself uses "debug" for the artifacts.
  PROFILE_DIR="dev"
  PROFILE_ARGS=()
fi

# ---- Cross-check only -----------------------------------------------------
if [[ "$CHECK_ONLY" -eq 1 ]]; then
  echo "[android] Checking $PACKAGE for $TRIPLE (no linking, no APK)..."
  cargo check -p "$PACKAGE" --target "$TRIPLE"
  echo "[android] Check passed."
  exit 0
fi

# ---- Signing key for cargo-apk 0.10 ---------------------------------------
# Release profiles have no automatic key in cargo-apk; use the well-known
# Android debug keystore unless the caller supplied a real one.
if [[ "$PROFILE" == "release" && -z "${CARGO_APK_RELEASE_KEYSTORE:-}" ]]; then
  ANDROID_USER_HOME="${ANDROID_USER_HOME:-$HOME/.android}"
  mkdir -p "$ANDROID_USER_HOME"
  DEBUG_KEYSTORE="$ANDROID_USER_HOME/debug.keystore"
  if [[ ! -f "$DEBUG_KEYSTORE" ]]; then
    echo "[android] Creating local debug keystore at $DEBUG_KEYSTORE"
    keytool -genkey -v \
      -keystore "$DEBUG_KEYSTORE" \
      -storepass android -alias androiddebugkey -keypass android \
      -dname "CN=Android Debug,O=Android,C=US" \
      -keyalg RSA -keysize 2048 -validity 10000 >/dev/null 2>&1
  fi
  export CARGO_APK_RELEASE_KEYSTORE="$DEBUG_KEYSTORE"
  export CARGO_APK_RELEASE_KEYSTORE_PASSWORD="${CARGO_APK_RELEASE_KEYSTORE_PASSWORD:-android}"
fi

# ---- Build ----------------------------------------------------------------
echo "[android] Building $PACKAGE APK for $TRIPLE ($PROFILE)..."
cargo apk build \
  -p "$PACKAGE" \
  --lib \
  --target "$TRIPLE" \
  "${PROFILE_ARGS[@]}"

# ---- Locate the APK -------------------------------------------------------
find_apk() {
  local dir candidate newest=""
  for dir in \
    "$TARGET_DIR/$PROFILE_DIR/apk" \
    "$TARGET_DIR/$TRIPLE/$PROFILE_DIR/apk" \
    "$TARGET_DIR/release/apk" \
    "$TARGET_DIR/debug/apk" \
    "$TARGET_DIR/dev/apk"; do
    for candidate in "$dir/$APK_NAME.apk" "$dir/${APK_NAME//-/_}.apk"; do
      if [[ -f "$candidate" ]]; then
        echo "$candidate"
        return 0
      fi
    done
  done
  # Fallback: newest APK anywhere under the target directory.
  while IFS= read -r candidate; do
    if [[ -z "$newest" || "$candidate" -nt "$newest" ]]; then
      newest="$candidate"
    fi
  done < <(find "$TARGET_DIR" -type f -name '*.apk' 2>/dev/null)
  [[ -n "$newest" ]] || return 1
  echo "$newest"
}

APK_PATH="$(find_apk)" || die "build finished but no .apk was found under $TARGET_DIR"
[[ -f "$APK_PATH" ]] || die "APK path is not a file: $APK_PATH"
echo "[android] APK: $APK_PATH"

# ---- Optional install -----------------------------------------------------
if [[ "$INSTALL" -eq 1 ]]; then
  if [[ -x "$ANDROID_HOME/platform-tools/adb" ]]; then
    ADB="$ANDROID_HOME/platform-tools/adb"
  else
    ADB="adb"
  fi
  command -v "$ADB" >/dev/null 2>&1 || [[ -x "$ADB" ]] || die "adb not found; install platform-tools"
  echo "[android] Installing $APK_PATH with adb..."
  "$ADB" install -r "$APK_PATH"
  echo "[android] Installed."
fi
