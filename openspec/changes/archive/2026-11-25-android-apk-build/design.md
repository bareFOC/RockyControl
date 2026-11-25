# Design

## Context

Shared crates (`rk-ui`, `rk-sim`, `rk-control`, `rk-model`, `rk-viewport`) already compile for `aarch64-linux-android` (verified with `cargo check -p rk-ui --target aarch64-linux-android`). The pinned eframe fork supports `android-native-activity` and handles GLES surface teardown/recreation on suspend. Desktop runner assumptions that must be avoided: `zenity` subprocess, cwd-relative `exports/`, TCP MCP listener, UDP gadget transport.

Environment installed and verified: Android SDK at `$ANDROID_HOME` (`$HOME/Android/Sdk`), platform 35, build-tools 36.0.0, NDK 27.1.12297006, `cargo-apk 0.10.0`, JDK 21, `adb`, `aarch64-linux-android` Rust target. Env exports persisted in `~/.bashrc`.

## Goals / Non-Goals

**Goals:** a runnable APK using unchanged shared UI/control code; safe mobile file I/O; no desktop-only services; reproducible build script + CI check.

**Non-Goals:** signing/Play Store/AAB, USB-OTG hardware connection from Android, gamepad support (gilrs has no Android backend), changing the control kernel, changing desktop behavior.

## Decisions

1. New `apps/rk-android` with `[lib] crate-type = ["cdylib","lib"]`; deps: `rk-ui`, `egui`, `eframe { features = ["glow","default_fonts","android-native-activity"] }`, `glow`, `winit` (resolves to pinned fork), `android_logger`, `log`. No `rk-mcp`, `craft-quit`, `craft-atomic`, `x11`, `wayland`, `accesskit`.
2. `android_main(app)` sets `NativeOptions { renderer: Glow, android_app: Some(app), .. }` and calls `eframe::run_native` with an `AndroidWrapper { inner: RockyStudioApp }` whose `ui`/`logic` delegate exactly like `apps/rk-desktop` and `apps/rk-web`. Add a small status-bar top spacer as in the egui `hello_android` example.
3. `AndroidSaver` implements `PlatformSaver` using `AndroidApp`'s internal files dir; `request_open_urdf_file` uses a JNI SAF picker with bytes delivered through `poll_opened_files`. Initial implementation may fall back to a documented "drag/drop or bundled URDF only" path if a device is unavailable for JNI validation, but must not call `zenity`.
4. Gate native-only paths with `cfg(not(any(target_arch = "wasm32", target_os = "android")))` in `rk-mcp` module declarations/re-exports, `rk-ui` UDP connect + dashboard card + MCP bridge status, `rk-link` UDP transport, and `rk-sim` path-save helper. Keep all tests/desktop builds unchanged.
5. Packaging via `[package.metadata.android]` (`build_targets = ["aarch64-linux-android"]`, min 23 / target 35) and `cargo apk build -p rk-android --lib --target aarch64-linux-android --release`; `scripts/build-android.sh` sets env and runs check → build → optional `adb install`.
6. CI: add a job that installs the Android target and runs `cargo check -p rk-android --target aarch64-linux-android` (and optionally `cargo apk build`) uploading the APK as an artifact. No signing.

## Risks / Trade-offs

- Pinned winit/eframe fork Android path is untested until a real `cargo apk build`/device run; mitigate with an early `cargo check` gate and the fork's first-party `hello_android` template.
- Global Embassy mock clock + leaked task: one runner per process, all stepping on UI thread, reconnect via `reset_experiment`; activity recreation must preserve the app instance.
- GLES context loss: eframe handles surface recreation; verify viewport scene cache (`rk-viewport` thread-local keyed by id) after resume on device.
- `eframe` `persistence` feature may use file storage; if it fails on Android, disable it in the Android crate or supply in-memory storage.
- No gamepad on Android; touch UI + MCP (external) remain the control surfaces. In-app MCP listener explicitly unavailable.
