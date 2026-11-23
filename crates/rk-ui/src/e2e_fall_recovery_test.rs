//! End-to-end, headless verification of the fall -> recover -> guard scenario
//! required by OpenSpec change `fall-recovery-and-visual-alarm` (task 7.3).
//!
//! This test drives the *real* simulation stack — `SimTransport` /
//! `CopperSimRunner` / `SimulatedServoPlant`, the real `DirectServoKernel`
//! inside the Copper runner, and the real `StudioModel` + fall watcher state
//! machine in `RockyStudioApp::update_fall_watcher()` — with no egui context
//! and no GUI. Only the per-frame call order of the real `update_frame()`
//! loop (`build_next_host_packet -> step_exchange -> ingest_telemetry ->
//! update_fall_watcher`) is reproduced by the local `step()` helper below,
//! exactly as documented in `update_frame()`.
//!
//! The recovery is started purely by the operator Get Up control: the watcher
//! queues `ClearFaults` via `reset_estop()` as part of its ordered sequence, so
//! no separate E-Stop Reset button press is needed. Telemetry is never fabricated.

use rk_proto::{ControlMode, FaultFlags, FallOrientation, fall_orientation};

use super::{NoopSaver, RecoveryState, RockyStudioApp};

const DT: f32 = 0.016;

/// Reproduces the control-loop ordering of `RockyStudioApp::update_frame()`
/// without egui: host packet -> transport exchange -> telemetry ingest ->
/// fall watcher. Returns the telemetry frame observed this tick.
fn step(app: &mut RockyStudioApp) -> rk_proto::McuToHost {
    let pkt = app.model.build_next_host_packet(DT);
    let telem = app
        .transport
        .step_exchange(DT, &pkt)
        .expect("sim transport always returns a frame");
    app.model.ingest_telemetry(telem.clone());
    app.update_fall_watcher(DT);
    telem
}

fn arm_drive_with_balance(app: &mut RockyStudioApp) {
    app.connected = true;
    app.model.walk_cmd.balance_enabled = true;
    app.model.walk_cmd.walk_enabled = false;
    app.model.set_authority_mode(ControlMode::Drive);
}

/// The red-alarm flag exposed to both viewports is computed from telemetry:
/// `TILT_FALL_CUTOFF && imu.available` (see `control_screen.rs` /
/// `dashboard_screen.rs`). This mirrors that exact expression.
fn viewport_alarm(telem: &rk_proto::McuToHost) -> bool {
    telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF) && telem.imu.available
}

#[test]
fn e2e_forward_shove_to_prone_get_up_and_back_to_guard() {
    let mut app = RockyStudioApp::new(Box::new(NoopSaver));
    assert_eq!(app.recovery_state, RecoveryState::Idle);
    assert!(!app.fall_event_active);

    // --- 1. Drive + balance, then shove forward (positive pitch impulse) ---
    arm_drive_with_balance(&mut app);
    let cutoff = app.model.balance_gains.fall_cutoff_rad;
    // A strong forward shove: balance catches small shoves (~0.20 rad), so use
    // a violent push (within the plant's +-6.5 rad/s clamp) to exceed cutoff.
    app.transport.apply_push_impulse(6.0, 0.0);

    // Step the plant until a fall latches (FallTuck 40 ms + margin).
    let mut fell = None;
    let mut peak_pitch = 0.0_f32;
    for _ in 0..200 {
        let telem = step(&mut app);
        peak_pitch = peak_pitch.max(telem.imu.pitch_rad.abs());
        if telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF) {
            fell = Some(telem);
            break;
        }
    }
    let fall_telem = fell.expect("forward shove must exceed fall_cutoff_rad and latch a fall");

    // --- 2. Assert the fall latch observable chain ---
    assert!(
        peak_pitch >= cutoff,
        "peak pitch must exceed the fall cutoff (cutoff={cutoff}, peak={peak_pitch})"
    );
    assert!(fall_telem.imu.pitch_rad > 0.0, "forward shove => positive pitch => Prone");
    assert!(fall_telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    assert_eq!(fall_telem.active_mode, ControlMode::EStop);
    assert!(fall_telem.imu.available);
    // Classifier reports Prone for a forward (positive pitch) fall.
    assert_eq!(fall_telem.balance.fall_orientation, fall_orientation::PRONE);
    assert_eq!(
        FallOrientation::from_wire(fall_telem.balance.fall_orientation),
        FallOrientation::Prone
    );
    // Watcher latched the fall and raised the red-alarm flag.
    assert_eq!(app.recovery_state, RecoveryState::FallLatched);
    assert_eq!(app.fall_orientation, FallOrientation::Prone);
    assert!(app.fall_event_active);
    assert!(viewport_alarm(&app.model.latest_telemetry));

    // --- 3. Operator confirms Get Up (no separate E-Stop Reset pressed) ---
    // A fallen torso (>0.60 rad) stays resting in the torque-OFF plant, so use
    // the real sim control `reset_upright()` to stand Rocky back up before the
    // upright dwell is measured. The watcher itself queues ClearFaults via
    // reset_estop() — this is exactly the deadlock the dwell gate fix addresses.
    app.transport.reset_upright();
    app.operator_confirmed = true;
    app.auto_recovery_enabled = true;

    // The upright dwell must elapse before the watcher calls reset_estop() and
    // enters ClearingFault; Drive must not be entered yet. Crucially, the
    // TILT_FALL_CUTOFF bit is STILL latched in telemetry at this point — the
    // watcher must not wait for `!fall_fault` before queueing ClearFaults.
    let mut saw_clearing_fault = false;
    for _ in 0..200 {
        let telem = step(&mut app);
        assert!(
            app.model.requested_mode != ControlMode::Drive,
            "Drive must not be entered before the watcher observes fault clear"
        );
        if app.recovery_state == RecoveryState::ClearingFault {
            saw_clearing_fault = true;
            // Prove the deadlock is gone: ClearFaults is queued while the fault
            // is still latched in the latest telemetry frame.
            assert!(
                telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
                "ClearFaults must be queued from FallLatched while the fall bit is still set"
            );
            break;
        }
    }
    assert!(
        saw_clearing_fault,
        "ordered recovery must pass through ClearingFault without a separate reset; got {:?}",
        app.recovery_state
    );

    // Step until the fault-clear config command has traversed the wire and
    // telemetry no longer reports the fall. The alarm must stay raised.
    let mut fault_cleared = false;
    for _ in 0..200 {
        let telem = step(&mut app);
        assert!(
            app.fall_event_active,
            "alarm must stay raised until stable upright after get-up"
        );
        if !telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
            && telem.active_mode != ControlMode::EStop
        {
            fault_cleared = true;
            break;
        }
    }
    assert!(fault_cleared, "ClearFaults must clear the latched fall fault");
    // Drive may already have been requested now that the fault is clear; the
    // earlier loop asserted it was never entered before the fault cleared.

    // --- 4. Drive re-entered and the Prone get-up move selected ---
    // The watcher advances one transition per frame, so Drive may already have
    // been requested (or playback started) by the time the fault-clear loop
    // exited. Assert the observable end state rather than a transient state.
    let mut started = false;
    for _ in 0..40 {
        if app.model.playback.is_some() {
            started = true;
            break;
        }
        step(&mut app);
    }
    assert!(started, "get-up playback must start");
    assert_eq!(app.model.requested_mode, ControlMode::Drive);
    assert_eq!(app.recovery_state, RecoveryState::PlayingGetUp);
    let pb = app.model.playback.as_ref().expect("playback must be active");
    let move_name = app.model.moves[pb.move_idx].name.as_str();
    assert_eq!(move_name, "Get Up Prone");
    assert!(!app.model.walk_cmd.walk_enabled);
    assert!(app.model.requested_mode == ControlMode::Drive);

    // --- 5. Playback plays to completion across many real control ticks ---
    let mut ticks_playing = 0usize;
    while app.model.playback.is_some() && ticks_playing < 2000 {
        step(&mut app);
        ticks_playing += 1;
    }
    assert!(ticks_playing > 0, "playback must consume control ticks");
    assert!(app.model.playback.is_none(), "get-up playback must complete");
    assert_eq!(app.recovery_state, RecoveryState::Completed);
    assert_eq!(app.model.requested_mode, ControlMode::Teach);
    // Guard stance: torso near upright in telemetry.
    let guard_telem = app.model.latest_telemetry.clone();
    assert!(guard_telem.imu.pitch_rad.abs() < cutoff);
    assert!(guard_telem.imu.roll_rad.abs() < cutoff);

    // Alarm persists until a stable upright dwell is observed.
    assert!(
        app.fall_event_active,
        "alarm must persist until stable upright after get-up"
    );

    // --- 6. Alarm clears after stable upright dwell ---
    for _ in 0..80 {
        step(&mut app);
        if app.recovery_state == RecoveryState::Idle {
            break;
        }
    }
    assert_eq!(app.recovery_state, RecoveryState::Idle);
    assert!(!app.fall_event_active);
    assert_eq!(app.fall_orientation, FallOrientation::Unknown);
    assert!(!viewport_alarm(&app.model.latest_telemetry));
    assert_eq!(app.model.requested_mode, ControlMode::Teach);
    assert!(!app.model.latest_telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
}
