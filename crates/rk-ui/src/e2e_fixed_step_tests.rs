//! Headless end-to-end tests on the interactive `step_fixed` path.
//!
//! These exercise the exact call ordering used by
//! [`RockyStudioApp::update_frame`] — `build_next_host_packet -> step_fixed ->
//! ingest_telemetry -> update_fall_watcher` — with the real `SimTransport`,
//! shared torque-based plant, Embassy control task and `StudioModel`, but no
//! egui context and no wall clock. Each frame advances exactly one 5 ms
//! control tick, so host-side playback/dwell timers and simulator time stay in
//! lockstep and every scenario is deterministic from a clean reset.
//!
//! The Embassy time driver is process-global. All app-level tests that step a
//! harness share this lock so parallel test threads cannot advance the same
//! mock clock underneath each other.

use std::sync::Mutex;

use rk_model::StudioModel;
use rk_proto::{ControlMode, FaultFlags, WalkCmd, estimator_valid, uvc_phase};

use super::{CONTROL_STEP_SEC, NoopSaver, RecoveryState, RockyStudioApp};

/// Serializes every app-level fixed-step harness in this test binary.
pub(super) static E2E_HARNESS_LOCK: Mutex<()> = Mutex::new(());

const TICKS_PER_FRAME: usize = 1;
const FRAME_DT: f32 = CONTROL_STEP_SEC;

/// 800 fixed ticks = 4 s, matching rk-sim's walking reference test.
const WALK_TICKS: usize = 800;
/// Drive-mode warm-up before an action or shove (0.3 s at 5 ms/tick).
const DRIVE_WARMUP_TICKS: usize = 60;
const COMBO_TICKS: usize = 2000; // 10 s — the combo keyframes total ~2.9 s
const SHOVE_RECOVERY_TICKS: usize = 600; // 3 s — UVC catch step + settle

/// One fixed tick per frame: host packet -> transport ticks -> telemetry
/// ingest -> fall watcher. Returns the frame observed after this tick.
fn step(app: &mut RockyStudioApp) -> rk_proto::McuToHost {
    let pkt = app.model.build_next_host_packet(FRAME_DT);
    let telem = app
        .transport
        .step_fixed(TICKS_PER_FRAME, &pkt, FRAME_DT)
        .expect("sim transport always returns a frame");
    app.model.ingest_telemetry(telem.clone());
    app.update_fall_watcher(FRAME_DT);
    telem
}

/// Fresh transport + fresh model state for a reproducible experiment.
fn fresh_app() -> RockyStudioApp {
    let mut app = RockyStudioApp::new(Box::new(NoopSaver));
    app.transport.reset_experiment();
    app.model = StudioModel::new_default();
    app.connected = true;
    app
}

fn arm_drive_with_balance(app: &mut RockyStudioApp) {
    app.model.walk_cmd = WalkCmd {
        walk_enabled: false,
        balance_enabled: true,
        ..Default::default()
    };
    app.model.set_authority_mode(ControlMode::Drive);
}

fn peak_tilt(telem: &rk_proto::McuToHost) -> f32 {
    telem.imu.pitch_rad.abs().max(telem.imu.roll_rad.abs())
}

/// The app must translate the plant forward through `step_fixed`, with the
/// plant-published contact mask alternating between soles (mirroring rk-sim's
/// `end_to_end_walking_translates_with_alternating_contacts`).
#[test]
fn e2e_fixed_step_walking_translates_with_alternating_contacts() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    app.model.walk_cmd = WalkCmd {
        walk_enabled: true,
        balance_enabled: true,
        vx: 0.5,
        cadence_hz: 1.55,
        ..Default::default()
    };
    app.model.set_authority_mode(ControlMode::Drive);

    let mut start_z = None;
    let mut prev_mask: Option<u8> = None;
    let mut swaps = 0_u32;
    let mut contact_ticks = [0_u32; 2];
    let mut last = rk_proto::McuToHost::default();

    for _ in 0..WALK_TICKS {
        let telem = step(&mut app);
        if start_z.is_none() {
            start_z = Some(telem.balance.world_pos_m[2]);
        }
        last = telem;
        assert_eq!(
            app.recovery_state,
            RecoveryState::Idle,
            "walk must not latch a fall"
        );
        assert!(
            !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "walking must stay below the fall cutoff"
        );

        // Contacts are plant-derived and published with the GRF validity bit;
        // never consume them unconditionally (hardware v1 leaves GRF clear).
        if last.estimator.valid_mask & estimator_valid::GRF != 0 {
            let mask = last.estimator.contact_mask & 0b11;
            if mask & 0b01 != 0 {
                contact_ticks[0] += 1;
            }
            if mask & 0b10 != 0 {
                contact_ticks[1] += 1;
            }
            if let Some(prev) = prev_mask {
                if (prev == 0b01 && mask == 0b10) || (prev == 0b10 && mask == 0b01) {
                    swaps += 1;
                }
            }
            prev_mask = Some(mask);
        }
    }

    let start_z = start_z.expect("at least one telemetry frame");
    let displacement = last.balance.world_pos_m[2] - start_z;
    assert_eq!(
        last.active_mode,
        ControlMode::Drive,
        "walking must stay in Drive"
    );
    assert!(
        displacement > 0.10_f32,
        "vx>0 must produce real forward displacement, got {displacement:.3} m"
    );
    assert!(swaps >= 6, "contacts must alternate, got {swaps} swaps");
    assert!(
        contact_ticks[0] > 0 && contact_ticks[1] > 0,
        "both soles must carry ground reaction, got {contact_ticks:?}"
    );
    let cutoff = app.model.balance_gains.fall_cutoff_rad;
    assert!(
        peak_tilt(&last) < cutoff,
        "tilt must stay below fall cutoff: pitch={:.3} roll={:.3}",
        last.imu.pitch_rad,
        last.imu.roll_rad
    );
    assert_ne!(last.estimator.valid_mask & estimator_valid::WORLD, 0);
    assert!(
        last.balance.world_vel_m_s[2] > 0.0,
        "achieved speed must be forward"
    );
}

/// A full strike combo played through the app must complete without a fall or
/// recovery handoff: the guarded keyframe tail keeps the robot upright.
#[test]
fn e2e_fixed_step_combo_playback_stays_upright() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    arm_drive_with_balance(&mut app);
    for _ in 0..DRIVE_WARMUP_TICKS {
        step(&mut app);
    }
    assert_eq!(
        app.model.latest_telemetry.active_mode,
        ControlMode::Drive,
        "warm-up must complete the Drive handshake"
    );

    let move_idx = app
        .model
        .moves
        .iter()
        .position(|m| m.name == "Combo: Jab-Cross-Kick")
        .expect("Jab-Cross-Kick combo exists");
    app.model.play_move(move_idx);
    assert!(app.model.playback.is_some(), "combo playback must start");

    let mut ticks = 0_usize;
    let mut peak_tilt_rad = 0.0_f32;
    while app.model.playback.is_some() && ticks < COMBO_TICKS {
        let telem = step(&mut app);
        ticks += 1;
        peak_tilt_rad = peak_tilt_rad.max(peak_tilt(&telem));
        assert!(
            !telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "combo must not trip the fall cutoff at tick {ticks}"
        );
        assert!(
            !app.fall_event_active && app.recovery_state == RecoveryState::Idle,
            "combo must not trigger recovery at tick {ticks}"
        );
        assert_eq!(
            telem.active_mode,
            ControlMode::Drive,
            "combo must keep torque on"
        );
    }
    assert!(ticks > 0, "playback must consume fixed ticks");
    assert!(app.model.playback.is_none(), "combo playback must complete");
    assert!(
        peak_tilt_rad < app.model.balance_gains.ukemi_tilt_rad,
        "combo must stay upright: peak tilt {peak_tilt_rad:.3} rad"
    );
    assert_eq!(app.model.latest_telemetry.active_mode, ControlMode::Drive);
    assert!(!app.fall_event_active);
    assert_eq!(app.recovery_state, RecoveryState::Idle);
}

/// A bounded shove with balance enabled must trigger a UVC catch step and then
/// settle upright without any fall being reported by the app's watcher.
#[test]
fn e2e_fixed_step_shove_recovers_upright() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    arm_drive_with_balance(&mut app);
    for _ in 0..DRIVE_WARMUP_TICKS {
        step(&mut app);
    }
    assert_eq!(
        app.model.latest_telemetry.active_mode,
        ControlMode::Drive,
        "warm-up must complete the Drive handshake"
    );

    // The rk-sim reference uses 1.6/-1.1 at a 10 ms variable dt. At the real
    // 5 ms fixed tick the same impulse pushes the plant past the fall cutoff,
    // so use a smaller but still visible shove (peak tilt ~0.13 rad) that the
    // UVC catch step recovers without a fall.
    app.transport.apply_push_impulse(1.0, -0.7);

    let mut peak_tilt_rad = 0.0_f32;
    let mut saw_catch_step = false;
    let mut last = rk_proto::McuToHost::default();
    for tick in 0..SHOVE_RECOVERY_TICKS {
        let telem = step(&mut app);
        last = telem;
        peak_tilt_rad = peak_tilt_rad.max(peak_tilt(&telem));
        if telem.balance.uvc_phase == uvc_phase::ACTIVE_CATCH_STEP {
            saw_catch_step = true;
        }
        assert_eq!(
            app.recovery_state,
            RecoveryState::Idle,
            "balanced shove must not latch a fall at tick {tick}"
        );
        assert!(
            !telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "balanced shove must stay below the fall cutoff at tick {tick}"
        );
    }
    assert!(peak_tilt_rad > 0.12, "shove must visibly tilt the torso");
    assert!(saw_catch_step, "UVC must step to catch the shove");
    assert_eq!(last.active_mode, ControlMode::Drive, "must stay standing");
    assert!(
        last.imu.pitch_rad.abs() < 0.06 && last.imu.roll_rad.abs() < 0.06,
        "UVC must recover upright (pitch={:.3}, roll={:.3})",
        last.imu.pitch_rad,
        last.imu.roll_rad
    );
    assert!(!app.fall_event_active);
}
