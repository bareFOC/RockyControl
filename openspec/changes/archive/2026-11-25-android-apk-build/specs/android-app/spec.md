# Spec Delta

## Purpose

Rocky Studio runs as an Android APK with the same workstation UI and simulator as desktop and browser, using platform-appropriate file I/O and no desktop-only runtime services.

## ADDED Requirements

### Requirement: Android runner entry point
The system SHALL provide an Android `cdylib` application crate whose `android_main(app)` constructs `RockyStudioApp` and runs it through `eframe` with the glow renderer and the Android native-activity backend. The app SHALL delegate each frame to the same `update_frame` used by desktop and browser, with no duplicate UI or control logic.

#### Scenario: APK launches the workstation UI
- **WHEN** the APK is launched on an Android 23+ device
- **THEN** the Rocky Studio UI renders through glow/GLES and the simulator is available without a host PC

#### Scenario: No duplicate control path
- **WHEN** a frame is rendered on Android
- **THEN** `RockyStudioApp::update_frame` is the single entry point for model, transport, telemetry and UI state

### Requirement: Platform-appropriate file handling
The Android app SHALL NOT spawn desktop utilities or use cwd-relative paths for file I/O. Saved logs and imported URDFs SHALL use an app-internal files directory and an Android document picker, delivered through the existing `PlatformSaver` interface.

#### Scenario: Save a replay log on Android
- **WHEN** the user exports a `.copper` log
- **THEN** the file is written under the app's files directory (or a SAF-selected location) and no `zenity` process is started

#### Scenario: Import a URDF on Android
- **WHEN** the user opens a URDF document
- **THEN** the bytes are delivered to the app through `poll_opened_files` and loaded into the studio model

### Requirement: Desktop-only services disabled on Android
The APK SHALL NOT start the native MCP TCP listener, open UDP transport connections to hardware gadget addresses, or use filesystem-path-only save helpers. These paths SHALL remain available on desktop.

#### Scenario: No listener in APK
- **WHEN** the Android app connects to the simulator
- **THEN** no `TcpListener` is bound and the MCP status reports the listener as unavailable

#### Scenario: Desktop unchanged
- **WHEN** the desktop app is built and run
- **THEN** the MCP listener and UDP transport remain available as before

### Requirement: Single deterministic simulator instance
The Android app SHALL keep one `CopperSimRunner` per process, drive fixed steps on the UI thread, and SHALL NOT reset the process-global Embassy mock clock after the first tick.

#### Scenario: Activity resumes after suspend
- **WHEN** the activity is resumed after a suspend
- **THEN** the existing simulator continues from its prior state and the glow viewport reinitialises its GLES resources

#### Scenario: Reconnect reuses the runner
- **WHEN** the user reconnects the simulator
- **THEN** `reset_experiment` is used rather than allocating a second executor against the global mock driver

### Requirement: APK build and documentation
The repository SHALL provide a documented command to produce an unsigned `aarch64-linux-android` APK with `cargo apk`, consuming Android SDK/NDK environment variables, and CI SHALL verify the Android target builds.

#### Scenario: Build an APK
- **WHEN** a developer runs the documented build script with SDK/NDK installed
- **THEN** an installable unsigned APK is produced for `aarch64-linux-android`

#### Scenario: CI target check
- **WHEN** CI runs the Android job
- **THEN** the app crate and shared crates cross-compile for `aarch64-linux-android` without host-only features
