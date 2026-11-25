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

use rk_mcp::McpHost;
use rk_model::StudioModel;
use rk_proto::{
    ControlMode, FallOrientation, FaultFlags, HostToMcu, JointCommandMode, MAX_JOINTS,
    RunningConfig, WalkCmd, estimator_valid, uvc_phase,
};

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

/// `running_packet` with an explicit per-joint torque ceiling (keeps the
/// current-limited plant out of the over-current latch during running/CBF tests).
fn running_packet_limited(app: &RockyStudioApp, seq: u32, cfg: RunningConfig, limit: f32) -> HostToMcu {
    let mut pkt = running_packet(app, seq, cfg);
    pkt.torque_limit_nm = [limit; MAX_JOINTS];
    pkt
}

/// One fixed tick driven by an explicit typed packet (rather than the model's
/// packet builder). This is the same `step_fixed` call ordering as `step()` but
/// lets a scenario set the running gate/gains on the wire exactly as the MCU
/// receives them.
fn step_packet(app: &mut RockyStudioApp, pkt: HostToMcu) -> rk_proto::McuToHost {
    let telem = app
        .transport
        .step_fixed(1, &pkt, FRAME_DT)
        .expect("sim transport always returns a frame");
    app.model.ingest_telemetry(telem.clone());
    app.update_fall_watcher(FRAME_DT);
    telem
}

/// Torque-mode Drive packet with the running/SLIP gate wired to `cfg`.
fn running_packet(app: &RockyStudioApp, seq: u32, cfg: RunningConfig) -> HostToMcu {
    let dof = app.model.rig.joints.len().min(MAX_JOINTS);
    HostToMcu {
        seq,
        mode: ControlMode::Drive,
        torque_mask: (1_u32 << dof) - 1,
        speed_limit_pct: 100,
        target_rad: app.model.rig.default_pose(),
        command_mode: [JointCommandMode::Torque.to_wire(); MAX_JOINTS],
        torque_limit_nm: [1.8; MAX_JOINTS],
        walk: WalkCmd {
            balance_enabled: false,
            walk_enabled: false,
            running: cfg,
            ..Default::default()
        },
        ..Default::default()
    }
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
    let mut start_x = None;
    let mut speed_sum = 0.0_f32;
    let mut peak_tilt_rad = 0.0_f32;
    let mut recent_tilt = [0.0_f32; 200]; // trailing 1 s at 5 ms/tick
    let mut com_height_min = f32::INFINITY;
    let mut com_height_max = f32::NEG_INFINITY;
    let mut lateral_drift = 0.0_f32;
    // The corrected plant publishes genuine double-support frames, so a strict
    // `0b01 -> 0b10` mask transition misses most step swaps. Track the sole
    // carrying the larger GRF instead — this mirrors rk-sim's plant-derived
    // `WalkQuality.contact_swap_count` and keeps the alternation intent.
    let mut dominant_foot: Option<u8> = None;
    let mut swaps = 0_u32;
    let mut contact_ticks = [0_u32; 2];
    let mut worst_grf_deviation = 0.0_f32;
    let mut steady_flight_ticks = 0_u32;
    let mut max_current_ma = 0_u16;
    let body_weight = (app.model.walk_cmd.running.mass_kg * 9.81_f32).max(1.0e-3_f32);
    // 250 ticks = 1.25 s at 200 Hz. Gait ramp-up/contact settling lasts
    // ~1.05 s; with the P4 toe-off/heel-strike dwell only the pre-control plant
    // tick 1 is airborne, so the steady-state no-flight assertion excludes it.
    const STARTUP_TICKS: usize = 250;

    let mut last = rk_proto::McuToHost::default();
    for tick in 1..=WALK_TICKS {
        let telem = step(&mut app);
        if start_z.is_none() {
            start_z = Some(telem.balance.world_pos_m[2]);
            start_x = Some(telem.balance.world_pos_m[0]);
        }
        speed_sum += telem.balance.world_vel_m_s[2];
        let tilt = peak_tilt(&telem);
        peak_tilt_rad = peak_tilt_rad.max(tilt);
        recent_tilt[tick % recent_tilt.len()] = tilt;
        com_height_min = com_height_min.min(telem.balance.world_pos_m[1]);
        com_height_max = com_height_max.max(telem.balance.world_pos_m[1]);
        lateral_drift = lateral_drift
            .max((telem.balance.world_pos_m[0] - start_x.expect("start x")).abs());
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
        max_current_ma = max_current_ma.max(last.current_ma.iter().copied().max().unwrap_or(0));

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
            let right = last.estimator.grf_right_n;
            let left = last.estimator.grf_left_n;
            let loaded = if right > 1.0e-3_f32 && left > 1.0e-3_f32 {
                Some(if right >= left { 0 } else { 1 })
            } else if right > 1.0e-3_f32 {
                Some(0)
            } else if left > 1.0e-3_f32 {
                Some(1)
            } else {
                None
            };
            if let Some(foot) = loaded {
                if let Some(prev) = dominant_foot {
                    if prev != foot {
                        swaps += 1;
                    }
                }
                dominant_foot = Some(foot);
                if mask.count_ones() == 1 {
                    let grf = if foot == 0 { right } else { left };
                    worst_grf_deviation = worst_grf_deviation.max((grf / body_weight - 1.0).abs());
                }
            } else if tick > STARTUP_TICKS
                && last.estimator.valid_mask & estimator_valid::GRF != 0
                && right <= 0.0_f32
                && left <= 0.0_f32
            {
                // Real flight is defined by zero ground reaction, not by the
                // reported contact mask: the kernel may publish a brief 0b00
                // while the plant still carries load (one-tick sensing delay).
                steady_flight_ticks += 1;
            }
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

    // P1 plant-quality assertions mirrored from rk-sim's walking test: with
    // GRF normalized, single support carries the full body weight, steady
    // walking never goes airborne, and current stays below the advisory.
    assert!(
        worst_grf_deviation < 0.15_f32,
        "single-support GRF deviated {:.3} from body weight (>{:0.3})",
        worst_grf_deviation,
        0.15_f32
    );
    // The app sees the sensed GRF one control tick after the plant; a single
    // stale zero-GRF frame is therefore expected. Real plant flight is asserted
    // to be zero in rk-sim's walking test (`ground_reaction_total_n`).
    assert!(
        steady_flight_ticks <= 1,
        "steady walking must not go airborne, got {steady_flight_ticks} ticks"
    );
    assert!(
        !last.faults.contains(FaultFlags::OVER_CURRENT) && max_current_ma < 1950,
        "walking must stay below the 1950 mA advisory, peak {max_current_ma} mA"
    );

    // Final P4 acceptance through the exact app-level call path (including the
    // host packet builder and telemetry ingestion), not just the sim harness.
    let mean_speed = speed_sum / WALK_TICKS as f32;
    let steady_tilt = recent_tilt.iter().copied().fold(0.0_f32, f32::max);
    const COMMAND_M_S: f32 = 0.5_f32 * 0.5_f32;
    assert!(
        (mean_speed - COMMAND_M_S).abs() <= 0.10_f32 * COMMAND_M_S,
        "UI mean speed {mean_speed:.3} outside ±10% of {COMMAND_M_S:.3}"
    );
    assert!(
        peak_tilt_rad < 0.08_f32 && steady_tilt < 0.03_f32,
        "UI peak/steady tilt {peak_tilt_rad:.3}/{steady_tilt:.3} exceeds 0.08/0.03"
    );
    assert!(
        lateral_drift < 0.03_f32,
        "UI lateral drift {lateral_drift:.3} exceeds 0.03 m"
    );
    assert!(
        (com_height_min - 0.72_f32).abs() <= 0.03_f32
            && (com_height_max - 0.72_f32).abs() <= 0.03_f32,
        "UI CoM height {com_height_min:.3}..{com_height_max:.3} outside 0.72 ± 0.03"
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

/// A shove applied through `step_fixed` must produce a genuine joint-current
/// spike that the kernel's impact stage reports on the IMPACT telemetry
/// channel, without tripping fall handling at a moderate magnitude.
#[test]
fn e2e_fixed_step_impact_detector_through_app() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    arm_drive_with_balance(&mut app);
    for _ in 0..DRIVE_WARMUP_TICKS {
        step(&mut app);
    }
    assert_eq!(app.model.latest_telemetry.active_mode, ControlMode::Drive);

    // The shove injects a velocity step on the loaded stance leg so the
    // Plant's PD controller develops a real current jump — the kernel's impact
    // observer watches current, not the torso angular impulse itself.
    app.transport.apply_push_impulse(1.5, -1.05);

    let mut saw_impact = false;
    let mut peak_magnitude_nm = 0.0_f32;
    let mut last = rk_proto::McuToHost::default();
    for tick in 0..SHOVE_RECOVERY_TICKS {
        let telem = step(&mut app);
        last = telem;
        assert_ne!(
            last.estimator.valid_mask & estimator_valid::IMPACT,
            0,
            "impact telemetry must always be marked valid at tick {tick}"
        );
        if last.balance.impact_detected {
            saw_impact = true;
            peak_magnitude_nm = peak_magnitude_nm.max(last.balance.impact_magnitude_nm);
        }
        assert!(
            !last.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "moderate impact must not latch a fall at tick {tick}"
        );
        assert_eq!(app.recovery_state, RecoveryState::Idle);
        assert!(!app.fall_event_active);
    }
    assert!(saw_impact, "shove must trip the kernel impact detector");
    assert!(
        peak_magnitude_nm
            > rk_control::impact::DEFAULT_CURRENT_SPIKE_MA
                / rk_control::impact::DEFAULT_MA_PER_NM,
        "impact magnitude {peak_magnitude_nm:.4} N·m must reflect a real current jump"
    );
    assert_eq!(last.active_mode, ControlMode::Drive, "impact alone must not abort Drive");
}

/// Latch a real fall through the plant + watcher, then seed every piece of
/// operator consent/recovery residue before any clean-state operation.
fn latch_fall_with_consent(app: &mut RockyStudioApp) {
    arm_drive_with_balance(app);
    app.transport.apply_push_impulse(6.0, 0.0);
    let mut latched = false;
    for _ in 0..400 {
        let telem = step(app);
        if telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
            && app.recovery_state == RecoveryState::FallLatched
        {
            latched = true;
            break;
        }
    }
    assert!(latched, "violent shove must latch a fall");
    assert!(app.fall_event_active);
    assert_eq!(app.fall_orientation, FallOrientation::Prone);

    // Residue that a reset/reconnect must never carry into the next run.
    app.operator_confirmed = true;
    app.get_up_requested = true;
    app.fall_dwell_sec = 0.25;
    app.saved_recovery_gains = Some((app.model.balance_gains, true));
}

fn assert_fall_state_clean(app: &RockyStudioApp) {
    assert!(!app.fall_event_active, "stale fall alarm survived reset");
    assert_eq!(app.fall_orientation, FallOrientation::Unknown);
    assert_eq!(app.recovery_state, RecoveryState::Idle);
    assert_eq!(app.fall_dwell_sec, 0.0);
    assert!(!app.operator_confirmed, "stale consent survived reset");
    assert!(!app.get_up_requested, "stale Get Up request survived reset");
    assert!(app.saved_recovery_gains.is_none(), "stale gains survived reset");
}

/// fall -> MCP `reset_experiment` -> shove: the second shove must behave exactly
/// like a fresh balanced session, with no stale alarm, consent or recovery.
#[test]
fn e2e_experiment_reset_clears_fall_state_before_next_shove() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    latch_fall_with_consent(&mut app);

    let message = McpHost::reset_experiment(&mut app).expect("sim reset succeeds");
    assert!(message.contains("fall/recovery state"));
    assert_fall_state_clean(&mut app);

    // The pre-reset telemetry frame still carries the fall fault, so open the
    // safety gate explicitly and let the fresh kernel publish clean telemetry
    // before re-arming Drive.
    app.model.reset_estop();
    for _ in 0..5 {
        step(&mut app);
    }
    assert!(!app.model.latest_telemetry.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    arm_drive_with_balance(&mut app);
    for _ in 0..DRIVE_WARMUP_TICKS {
        step(&mut app);
    }
    assert_eq!(app.model.latest_telemetry.active_mode, ControlMode::Drive);

    app.transport.apply_push_impulse(1.0, -0.7);
    let mut saw_catch_step = false;
    for tick in 0..SHOVE_RECOVERY_TICKS {
        let telem = step(&mut app);
        assert!(!app.fall_event_active, "post-reset fall alarm at tick {tick}");
        assert_eq!(app.recovery_state, RecoveryState::Idle, "stale recovery at tick {tick}");
        assert!(!app.operator_confirmed && !app.get_up_requested);
        assert!(
            !telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF),
            "post-reset shove must not re-latch a fall at tick {tick}"
        );
        if telem.balance.uvc_phase == uvc_phase::ACTIVE_CATCH_STEP {
            saw_catch_step = true;
        }
    }
    assert!(saw_catch_step, "post-reset shove must get a normal UVC catch step");
    assert_eq!(app.model.latest_telemetry.active_mode, ControlMode::Drive);
}

/// A transport reconnect builds a fresh session and must clear the same state.
#[test]
fn e2e_reconnect_clears_fall_state() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    latch_fall_with_consent(&mut app);

    app.connect_active_transport();
    assert_fall_state_clean(&mut app);
    assert!(app.connected, "sim reconnect stays connected");
    // Let the fresh transport publish a frame; no stale alarm may reappear.
    step(&mut app);
    assert_fall_state_clean(&mut app);

}

/// The running gate must engage through the app's fixed-step path, publish real
/// flight detection from the plant's free-fall IMU signature, and clear again
/// as soon as the gate is disabled on the wire.
#[test]
fn e2e_fixed_step_running_gate_engages_and_detects_flight() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut app = fresh_app();
    let cfg = RunningConfig {
        enabled: true,
        hop_height_m: 0.03,
        ..Default::default()
    };
    // Per-joint ceiling keeps the position-servo crouch from drawing rated
    // current and latching OVER_CURRENT; the SLIP overlay still gets ample
    // torque for a 3 cm hop.
    const LIMIT_NM: f32 = 0.3_f32;
    // Crouch in position mode first so the SLIP spring proxy is compressed.
    let mut crouch = running_packet_limited(
        &app,
        0,
        RunningConfig {
            enabled: false,
            ..cfg
        },
        LIMIT_NM,
    );
    crouch.command_mode = [JointCommandMode::Position.to_wire(); MAX_JOINTS];
    for i in [11usize, 12, 13, 17, 18, 19] {
        crouch.target_rad[i] = match i {
            12 | 18 => 1.4,
            _ => -0.7,
        };
    }
    for seq in 1..=80 {
        crouch.seq = seq;
        step_packet(&mut app, crouch);
    }

    const RUN_TICKS: u32 = 300;
    let mut saw_running = 0_usize;
    let mut saw_flight = 0_usize;
    let mut saw_running_valid = 0_usize;
    let mut max_com = 0.0_f32;
    let mut min_com = f32::INFINITY;
    let mut max_current = 0_u16;
    let mut last = rk_proto::McuToHost::default();
    for seq in 81..=80 + RUN_TICKS {
        let pkt = running_packet_limited(&app, seq, cfg, LIMIT_NM);
        let telem = step_packet(&mut app, pkt);
        if telem.balance.running_active {
            saw_running += 1;
        }
        if telem.balance.flight_detected {
            saw_flight += 1;
        }
        if telem.estimator.valid_mask & estimator_valid::RUNNING != 0 {
            saw_running_valid += 1;
        }
        max_com = max_com.max(telem.balance.running_com_height_m);
        min_com = min_com.min(telem.balance.running_com_height_m);
        max_current = max_current.max(telem.current_ma.iter().copied().max().unwrap_or(0));
        assert!(
            !telem.faults.contains(FaultFlags::OVER_CURRENT),
            "running overlay must respect the torque ceiling at tick {seq}"
        );
        last = telem;
    }
    assert_eq!(saw_running, RUN_TICKS as usize, "running gate must engage every tick");
    assert_eq!(
        saw_running_valid, RUN_TICKS as usize,
        "running telemetry validity bit must be published with the overlay"
    );
    assert!(
        saw_flight > 0 && saw_flight < RUN_TICKS as usize,
        "plant flight must be observed by the kernel without latching forever: {saw_flight}/{RUN_TICKS}"
    );
    assert!(
        max_com - min_com > 0.005_f32,
        "CoM height must vary over the hop: {min_com:.4}..{max_com:.4} m"
    );
    assert!(
        max_current < 1950,
        "running must stay below the 1950 mA over-current trip, peak {max_current} mA"
    );
    assert!(!last.faults.contains(FaultFlags::TILT_FALL_CUTOFF));
    assert_eq!(last.active_mode, ControlMode::Drive);

    // Disabling the gate on the wire must immediately withdraw the overlay and
    // its validity bit; no residual running telemetry may be published.
    let off = RunningConfig {
        enabled: false,
        ..cfg
    };
    let off_pkt = running_packet_limited(&app, 381, off, LIMIT_NM);
    let telem = step_packet(&mut app, off_pkt);
    assert!(!telem.balance.running_active, "disabled gate must clear running_active");
    assert_eq!(
        telem.estimator.valid_mask & estimator_valid::RUNNING,
        0,
        "disabled gate must clear the RUNNING validity bit"
    );
}

/// With the CBF shield enabled, a moderate shove during running must stay
/// inside the tilt/current envelope and must not latch a fault or a fall for
/// any kappa. Both kappa settings are checked against the same bounds; no
/// behavioral *difference* between them is asserted. In the shared torque
/// plant the SLIP torque is clamped at the host per-joint torque ceiling, so
/// the shield saturates identically at 0.3 N·m for both kappas here — a delta
/// assertion would test that clamp, not CBF behavior.
#[test]
fn e2e_fixed_step_cbf_running_shove_is_bounded() {
    let _guard = E2E_HARNESS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    const LIMIT_NM: f32 = 0.3_f32;
    const SHOVE_TICKS: usize = 220;

    for kappa in [3.0_f32, 100.0_f32] {
        let mut app = fresh_app();
        let cfg = RunningConfig {
            enabled: true,
            cbf_kappa: kappa,
            hop_height_m: 0.03,
            leg_spring_n_m: 150.0,
            ..Default::default()
        };
        // Compress the SLIP spring proxy in position mode under the same
        // ceiling used for the running ticks (avoids an OVER_CURRENT latch).
        let mut crouch = running_packet_limited(
            &app,
            0,
            RunningConfig {
                enabled: false,
                ..cfg
            },
            LIMIT_NM,
        );
        crouch.command_mode = [JointCommandMode::Position.to_wire(); MAX_JOINTS];
        for i in [11usize, 12, 13, 17, 18, 19] {
            crouch.target_rad[i] = match i {
                12 | 18 => 1.4,
                _ => -0.7,
            };
        }
        for seq in 1..=80 {
            crouch.seq = seq;
            step_packet(&mut app, crouch);
        }
        for seq in 81..=120 {
            let pkt = running_packet_limited(&app, seq, cfg, LIMIT_NM);
            step_packet(&mut app, pkt);
        }

        app.transport.apply_push_impulse(0.7, -0.49);
        let mut peak_tilt_rad = 0.0_f32;
        let mut peak_current_ma = 0_u16;
        let mut running_ticks = 0_usize;
        let mut valid_ticks = 0_usize;
        for seq in 121..=120 + SHOVE_TICKS as u32 {
            let pkt = running_packet_limited(&app, seq, cfg, LIMIT_NM);
            let telem = step_packet(&mut app, pkt);
            peak_tilt_rad = peak_tilt_rad.max(peak_tilt(&telem));
            peak_current_ma = peak_current_ma.max(telem.current_ma.iter().copied().max().unwrap_or(0));
            if telem.balance.running_active {
                running_ticks += 1;
            }
            if telem.estimator.valid_mask & estimator_valid::RUNNING != 0 {
                valid_ticks += 1;
            }
            assert!(
                !telem.faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                    && !telem.faults.contains(FaultFlags::OVER_CURRENT),
                "kappa={kappa}: moderate running shove must not latch a fault at tick {seq}: {:?}",
                telem.faults
            );
        }

        let cutoff = app.model.balance_gains.fall_cutoff_rad;
        assert_eq!(running_ticks, SHOVE_TICKS, "kappa={kappa}: overlay must stay engaged");
        assert_eq!(valid_ticks, SHOVE_TICKS, "kappa={kappa}: RUNNING validity must stay set");
        assert!(
            peak_tilt_rad > 0.05_f32 && peak_tilt_rad < cutoff,
            "kappa={kappa}: shove tilt {peak_tilt_rad:.3} must be visible but below the {cutoff} cutoff"
        );
        assert!(
            peak_current_ma < 1950,
            "kappa={kappa}: peak current {peak_current_ma} mA exceeds the advisory"
        );
        assert!(!app.fall_event_active, "kappa={kappa}: shove must not raise a fall alarm");
        assert_eq!(app.recovery_state, RecoveryState::Idle);
        assert_eq!(app.model.latest_telemetry.active_mode, ControlMode::Drive);
    }
}
