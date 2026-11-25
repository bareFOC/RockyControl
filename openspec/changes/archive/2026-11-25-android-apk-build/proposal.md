# Proposal

## Why

Rocky Studio already runs the exact same workstation UI and simulation stack on Desktop (`apps/rk-desktop`) and Browser WASM (`apps/rk-web`), and the UI has tablet/touch ergonomics. An Android APK would make the simulator usable on a tablet for offline controller development and demos. The pinned `eframe 0.37` fork has first-party Android support (`android-native-activity`, glow/GLES surface lifecycle) and the shared crates already cross-compile to `aarch64-linux-android`; only a thin runner and a mobile file-saver are missing. The Android SDK, platform 35, build-tools 36, NDK 27, and `cargo-apk` are now installed and verified (`cargo check -p rk-ui --target aarch64-linux-android` passes).

## What Changes

- **New `apps/rk-android` crate**: a `cdylib`+`lib` runner with `#[no_mangle] android_main(app: AndroidApp)` that constructs `RockyStudioApp` with an Android `PlatformSaver` and delegates `eframe::App::ui` to `update_frame`, exactly like the desktop/web runners. Uses `eframe` with `glow` + `android-native-activity`, **no** `x11`/`wayland`/`accesskit`, no `rk-mcp`, no `craft-quit`.
- **Android `PlatformSaver`**: writes/reads under the app-internal files directory obtained from `AndroidApp`; implements URDF open via a SAF/JNI picker (or a documented drag-and-drop fallback initially), with `poll_opened_files` delivery. No `zenity` subprocess and no relative `exports/` path.
- **Platform gating**: change native-only guards from `cfg(not(wasm32))` to `cfg(not(any(target_arch = "wasm32", target_os = "android")))` for the MCP TCP/bridge modules, UDP transport connection and dashboard card, and file-path save. Keep desktop behavior unchanged.
- **Packaging**: `[package.metadata.android]` (min SDK 23, target SDK 35, aarch64) consumed by `cargo apk build`. Add a `scripts/build-android.sh` helper documenting env vars and the build/install commands. Add a CI job that cross-checks and produces an unsigned APK artifact (no signing/Play Store in this change).
- **Docs**: README section for building and installing the APK; note Android limitations (no in-app MCP listener, gamepad inert, single sim session, sim time pauses when suspended, WORLD/GRF channels sim-only).

## Capabilities

### New Capabilities
- `android-app`: Rocky Studio runs as an Android APK using the same shared UI, control kernel, and simulator as desktop and browser.

### Modified Capabilities
- `native-mcp-bridge`: explicitly unavailable on Android; no TCP listener started in the APK.
- `unified-control-application`: gains a third host runner that invokes `RockyStudioApp::update_frame` unchanged.

## Impact

Files: new `apps/rk-android/**`, `scripts/build-android.sh`, root `Cargo.toml` (workspace member), `crates/rk-mcp/src/lib.rs`, `crates/rk-ui/src/{lib.rs,mcp_bridge.rs,dashboard_screen.rs}`, `crates/rk-link/src/lib.rs` (cfg guards only), `crates/rk-sim/src/lib.rs` (cfg guard only), `README.md`, CI workflow, new spec.

Risks: the pinned storytold winit/eframe fork's Android path is not yet compiled to a linked APK (only `cargo check` on shared crates is verified); Android activity recreation must not create a second `CopperSimRunner` against the process-global Embassy mock clock; glow viewport scene cache must survive GLES context loss after suspend/resume.

Deferred: Play Store/AAB signing, USB-OTG connection to real hardware, gamepad support, native file-picker JNI refinement, CI signing.
