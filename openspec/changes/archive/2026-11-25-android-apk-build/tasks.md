# Tasks

> Delegable. Group 1 foundation first; Groups 2/3 in parallel; Group 4 integrates.

- [x] 0.1 Install Android SDK platform 35, build-tools 36.0.0, NDK 27.1.12297006, cmdline-tools; persist ANDROID_HOME/NDK/linker env in ~/.bashrc. Verified `cargo check -p rk-ui --target aarch64-linux-android` passes.

## 1. Android runner crate (OWNER: android agent; FIRST)
- [x] 1.1 Add `apps/rk-android` to workspace; create Cargo.toml with cdylib+lib, eframe glow + android-native-activity, winit, android_logger; `[package.metadata.android]` aarch64/min23/target35. Verify host `cargo check -p rk-android`.
- [x] 1.2 Implement `android_main` + `AndroidWrapper` delegating to `RockyStudioApp::update_frame`; no native services. Verify `cargo check -p rk-android --target aarch64-linux-android`.
- [x] 1.3 Implement `AndroidSaver` against app-internal files dir; URDF open via SAF/JNI or documented fallback; no zenity/exports path. Verify cross-target check.

## 2. Platform gating (OWNER: platform agent; parallel to 1)
- [x] 2.1 Gate MCP TCP/bridge modules in `rk-mcp` and UI MCP status with `not(any(wasm32, android))`; keep desktop unchanged.
- [x] 2.2 Gate UDP transport connect/card and path-save helper for Android; verify `cargo test --workspace` and host checks.
- [x] 2.3 Add a test asserting Android cfg paths compile/behave (e.g. MCP status reports unavailable abstractly).

## 3. Packaging + CI + docs (OWNER: build agent; depends on 1/2)
- [x] 3.1 Add `scripts/build-android.sh` (env setup, `cargo apk build`, optional `adb install`) and README section.
- [x] 3.2 Add CI job for Android target check / APK artifact.
- [x] 3.3 Document Android limitations (MCP unavailable, gamepad, single sim session, suspend behavior).

## 4. Verification (OWNER: integrator; depends on all)
- [x] 4.1 Build a real APK with `cargo apk build -p rk-android --lib --target aarch64-linux-android --release`; verify APK exists and `adb` can list it (device optional). Re-run host tests, firmware-core, thumb, WASM, strict validate. Update status.
