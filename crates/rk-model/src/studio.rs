//! Bidirectional mirroring authority state machine (`StudioModel`), keyframes, and cubic Hermite playback.

use rk_control::JointCalibration;
use rk_proto::{
    BalanceGains, ConfigCmd, ControlMode, FaultFlags, HostToMcu, MAX_JOINTS, McuToHost, WalkCmd,
};
use serde::{Deserialize, Serialize};

use crate::bundle::AssetBundle;
use crate::kinematics::KinematicRig;

/// Teach & Action Studio keyframe.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Keyframe {
    pub label: String,
    pub duration_ms: u32,
    pub angles_rad: [f32; MAX_JOINTS],
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ActionMove {
    pub name: String,
    pub category: String,
    pub keyframes: Vec<Keyframe>,
}

#[derive(Clone, Debug)]
pub struct PlaybackState {
    pub move_idx: usize,
    pub elapsed_ms: f32,
    pub start_pose: [f32; MAX_JOINTS],
    pub progress: f32,
    pub active_keyframe: usize,
}

/// C1 smoothstep cubic Hermite curve in single-precision `f32`.
#[inline]
pub fn cubic_smoothstep(t: f32) -> f32 {
    let c = t.clamp(0.0_f32, 1.0_f32);
    c * c * (3.0_f32 - 2.0_f32 * c)
}

/// Evaluates an `ActionMove` at `elapsed_ms`, scaling tempo conservatively with `speed_limit_pct`.
pub fn sample_action_move(
    action: &ActionMove,
    start_pose: &[f32; MAX_JOINTS],
    elapsed_ms: f32,
    speed_limit_pct: u8,
) -> ([f32; MAX_JOINTS], bool, f32, usize) {
    if action.keyframes.is_empty() {
        return (*start_pose, true, 1.0, 0);
    }

    let rate_scale = (0.25_f32 + (speed_limit_pct as f32 / 100.0_f32) * 0.75_f32).clamp(0.2, 1.0);
    let scaled_elapsed = elapsed_ms * rate_scale;
    let total_ms: f32 = action
        .keyframes
        .iter()
        .map(|k| k.duration_ms.max(60) as f32)
        .sum();

    if scaled_elapsed >= total_ms {
        let last = action.keyframes.last().expect("non-empty keyframes");
        return (last.angles_rad, true, 1.0, action.keyframes.len() - 1);
    }

    let mut accum = 0.0_f32;
    for (idx, kf) in action.keyframes.iter().enumerate() {
        let dur = kf.duration_ms.max(60) as f32;
        if scaled_elapsed <= accum + dur || idx + 1 == action.keyframes.len() {
            let local_t = (scaled_elapsed - accum) / dur;
            let s = cubic_smoothstep(local_t);
            let prev = if idx == 0 {
                start_pose
            } else {
                &action.keyframes[idx - 1].angles_rad
            };
            let mut out = [0.0_f32; MAX_JOINTS];
            for j in 0..MAX_JOINTS {
                out[j] = prev[j] + (kf.angles_rad[j] - prev[j]) * s;
            }
            return (out, false, (scaled_elapsed / total_ms).clamp(0.0, 1.0), idx);
        }
        accum += dur;
    }

    (*start_pose, true, 1.0, 0)
}

pub fn default_combat_library(rig: &KinematicRig) -> Vec<ActionMove> {
    let pose = |overrides: &[(&str, f32)]| -> [f32; MAX_JOINTS] {
        let mut arr = rig.default_pose();
        for (name, val) in overrides {
            if let Some(j) = rig.joints.iter().find(|jm| jm.name == *name) {
                arr[j.index] = val.clamp(j.default_min_rad, j.default_max_rad);
            }
        }
        arr
    };

    let neutral = pose(&[]);
    let guard = pose(&[
        ("waist_yaw", 0.22),
        ("head_yaw", -0.18),
        ("l_shoulder_pitch", -1.10),
        ("l_shoulder_roll", 0.20),
        ("l_elbow_pitch", -1.72),
        ("r_shoulder_pitch", -1.18),
        ("r_shoulder_roll", -0.20),
        ("r_elbow_pitch", -1.85),
        ("l_hip_pitch", -0.26),
        ("l_knee_pitch", 0.50),
        ("l_ankle_pitch", -0.24),
        ("r_hip_pitch", -0.16),
        ("r_knee_pitch", 0.34),
        ("r_ankle_pitch", -0.18),
    ]);

    let left_jab = pose(&[
        ("waist_yaw", -0.35),
        ("head_yaw", 0.28),
        ("l_shoulder_pitch", -1.56),
        ("l_shoulder_roll", 0.05),
        ("l_elbow_pitch", -0.06),
        ("r_shoulder_pitch", -1.20),
        ("r_shoulder_roll", -0.18),
        ("r_elbow_pitch", -1.92),
        ("l_hip_pitch", -0.32),
        ("l_knee_pitch", 0.60),
        ("l_ankle_pitch", -0.28),
    ]);

    let right_cross_windup = pose(&[
        ("waist_yaw", -0.34),
        ("head_yaw", 0.26),
        ("l_shoulder_pitch", -1.12),
        ("l_elbow_pitch", -1.65),
        ("r_shoulder_pitch", -0.80),
        ("r_shoulder_roll", -0.32),
        ("r_elbow_pitch", -1.75),
    ]);

    let right_cross_impact = pose(&[
        ("waist_yaw", 0.56),
        ("head_yaw", -0.44),
        ("l_shoulder_pitch", -1.22),
        ("l_elbow_pitch", -1.90),
        ("r_shoulder_pitch", -1.58),
        ("r_shoulder_roll", -0.04),
        ("r_elbow_pitch", -0.05),
        ("l_hip_pitch", -0.34),
        ("l_knee_pitch", 0.62),
        ("l_ankle_pitch", -0.28),
    ]);

    let kick_chamber = pose(&[
        ("waist_yaw", -0.14),
        ("l_shoulder_pitch", -1.00),
        ("l_shoulder_roll", 0.35),
        ("l_elbow_pitch", -1.55),
        ("r_shoulder_pitch", -1.00),
        ("r_shoulder_roll", -0.35),
        ("r_elbow_pitch", -1.55),
        ("l_hip_roll", 0.14),
        ("r_hip_pitch", -1.35),
        ("r_knee_pitch", 1.85),
        ("r_ankle_pitch", 0.25),
    ]);

    let kick_snap = pose(&[
        ("waist_yaw", 0.18),
        ("l_shoulder_pitch", -0.92),
        ("l_shoulder_roll", 0.48),
        ("l_elbow_pitch", -1.45),
        ("r_shoulder_pitch", -0.86),
        ("r_shoulder_roll", -0.42),
        ("r_elbow_pitch", -1.45),
        ("l_hip_roll", 0.16),
        ("r_hip_pitch", -1.42),
        ("r_knee_pitch", 0.10),
        ("r_ankle_pitch", 0.42),
    ]);

    vec![
        ActionMove {
            name: "Guard".to_string(),
            category: "Stance".to_string(),
            keyframes: vec![Keyframe {
                label: "High Guard".to_string(),
                duration_ms: 600,
                angles_rad: guard,
            }],
        },
        ActionMove {
            name: "Left Jab".to_string(),
            category: "Strike".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Guard Ready".to_string(),
                    duration_ms: 300,
                    angles_rad: guard,
                },
                Keyframe {
                    label: "Snap Extend".to_string(),
                    duration_ms: 360,
                    angles_rad: left_jab,
                },
                Keyframe {
                    label: "Guard Return".to_string(),
                    duration_ms: 400,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Right Cross".to_string(),
            category: "Strike".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Torque Load".to_string(),
                    duration_ms: 320,
                    angles_rad: right_cross_windup,
                },
                Keyframe {
                    label: "Full Impact".to_string(),
                    duration_ms: 420,
                    angles_rad: right_cross_impact,
                },
                Keyframe {
                    label: "Guard Return".to_string(),
                    duration_ms: 460,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Front Kick".to_string(),
            category: "Kick".to_string(),
            keyframes: vec![
                Keyframe {
                    label: "Chamber Knee".to_string(),
                    duration_ms: 440,
                    angles_rad: kick_chamber,
                },
                Keyframe {
                    label: "Snap Thrust".to_string(),
                    duration_ms: 400,
                    angles_rad: kick_snap,
                },
                Keyframe {
                    label: "Re-Chamber".to_string(),
                    duration_ms: 360,
                    angles_rad: kick_chamber,
                },
                Keyframe {
                    label: "Plant Guard".to_string(),
                    duration_ms: 460,
                    angles_rad: guard,
                },
            ],
        },
        ActionMove {
            name: "Neutral Zero".to_string(),
            category: "Stance".to_string(),
            keyframes: vec![Keyframe {
                label: "Calibration Pose".to_string(),
                duration_ms: 650,
                angles_rad: neutral,
            }],
        },
    ]
}

/// Calibration persistence file format.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct CalibrationFile {
    pub robot_name: String,
    pub joints: Vec<JointCalibration>,
}

/// Bidirectional mirroring domain state machine.
pub struct StudioModel {
    pub bundle: AssetBundle,
    pub rig: KinematicRig,
    pub requested_mode: ControlMode,
    pub speed_limit_pct: u8,
    pub torque_mask: u32,
    pub target_rad: [f32; MAX_JOINTS],
    pub calibration: Vec<JointCalibration>,
    pub latest_telemetry: McuToHost,
    pub selected_joint: usize,
    pub moves: Vec<ActionMove>,
    pub selected_move: usize,
    pub playback: Option<PlaybackState>,
    pub walk_cmd: WalkCmd,
    pub balance_gains: BalanceGains,
    pub pending_config: Vec<ConfigCmd>,
    pub tx_seq: u32,
    pub status_message: String,
}

impl StudioModel {
    pub fn new_default() -> Self {
        let bundle = AssetBundle::bundled_rocky();
        let rig = KinematicRig::from_urdf_xml(&bundle.urdf_xml)
            .expect("bundled rocky_mk1.urdf must be valid");
        let calibration = rig.default_calibration();
        let default_pose = rig.default_pose();
        let moves = default_combat_library(&rig);
        let all_mask = if rig.joints.len() >= 32 {
            u32::MAX
        } else {
            (1u32 << rig.joints.len()) - 1
        };

        let mut latest_telemetry = McuToHost::default();
        latest_telemetry.measured_rad = default_pose;
        latest_telemetry.applied_target_rad = default_pose;

        Self {
            bundle,
            rig,
            requested_mode: ControlMode::Teach,
            speed_limit_pct: 65,
            torque_mask: all_mask,
            target_rad: default_pose,
            calibration,
            latest_telemetry,
            selected_joint: 3,
            moves,
            selected_move: 0,
            playback: None,
            walk_cmd: WalkCmd::default(),
            balance_gains: BalanceGains::default(),
            pending_config: Vec::new(),
            tx_seq: 0,
            status_message: "Ready in Teach mode (Torque OFF). Press Space at any time for E-Stop."
                .to_string(),
        }
    }

    pub fn load_bundle(&mut self, bundle: AssetBundle) -> Result<(), String> {
        let rig = KinematicRig::from_urdf_xml(&bundle.urdf_xml)?;
        let default_pose = rig.default_pose();
        self.requested_mode = ControlMode::Teach;
        self.playback = None;
        self.calibration = rig.default_calibration();
        self.target_rad = default_pose;
        self.latest_telemetry.measured_rad = default_pose;
        self.latest_telemetry.applied_target_rad = default_pose;
        self.moves = default_combat_library(&rig);
        self.selected_joint = 0;
        self.selected_move = 0;
        self.torque_mask = if rig.joints.is_empty() {
            0
        } else if rig.joints.len() >= 32 {
            u32::MAX
        } else {
            (1u32 << rig.joints.len()) - 1
        };
        let diag = bundle.diagnostics(&rig);
        let mesh_note = if !diag.missing_meshes.is_empty() {
            format!(
                " ({} mesh ref(s) missing — drop a .zip with STLs/OBJs)",
                diag.missing_meshes.len()
            )
        } else if !diag.resolved_meshes.is_empty() {
            format!(" ({} CAD mesh(es) loaded)", diag.resolved_meshes.len())
        } else {
            String::new()
        };
        self.status_message = format!(
            "Loaded URDF '{}' ({} DoF, {}, {} links){mesh_note} in Teach mode.",
            rig.robot_name,
            rig.joints.len(),
            rig.up_axis.label(),
            rig.links.len(),
        );
        self.bundle = bundle;
        self.rig = rig;
        Ok(())
    }

    pub fn is_estop_active(&self) -> bool {
        self.requested_mode == ControlMode::EStop
            || self.latest_telemetry.active_mode == ControlMode::EStop
            || self.latest_telemetry.faults.intersects(
                FaultFlags::ESTOP_ACTIVE
                    | FaultFlags::WATCHDOG_TIMEOUT
                    | FaultFlags::TILT_FALL_CUTOFF,
            )
    }

    /// Triggers immediate E-Stop (bound to Spacebar and top red button).
    pub fn trigger_estop(&mut self) {
        self.requested_mode = ControlMode::EStop;
        self.playback = None;
        self.walk_cmd.walk_enabled = false;
        self.walk_cmd.vx = 0.0;
        self.walk_cmd.vy = 0.0;
        self.walk_cmd.yaw_rate = 0.0;
        self.status_message =
            "E-STOP ACTIVE — Torque forced OFF. Click Reset E-Stop to return to Teach.".to_string();
    }

    /// Explicit operator confirmation: clears faults and returns strictly to `Teach` (never directly to `Drive`).
    pub fn reset_estop(&mut self) {
        self.requested_mode = ControlMode::Teach;
        self.playback = None;
        self.walk_cmd.walk_enabled = false;
        self.walk_cmd.vx = 0.0;
        self.walk_cmd.vy = 0.0;
        self.walk_cmd.yaw_rate = 0.0;
        self.target_rad = self.latest_telemetry.measured_rad;
        self.pending_config.push(ConfigCmd::ClearFaults);
        self.status_message = "E-Stop reset -> Teach mode (Torque OFF).".to_string();
    }

    /// Updates MCU upper-body vertical balance PD gains and queues them over the wire protocol.
    pub fn update_balance_gains(&mut self, gains: BalanceGains) {
        self.balance_gains = gains;
        self.pending_config.push(ConfigCmd::SetBalanceGains(gains));
    }

    /// Authority mode toggle enforcing **Mandatory Safety Rule 1 (Bumpless Transfer)**.
    pub fn set_authority_mode(&mut self, mode: ControlMode) {
        if self.is_estop_active() && mode != ControlMode::EStop {
            return;
        }
        if mode == ControlMode::Drive && self.requested_mode != ControlMode::Drive {
            for i in 0..self.rig.joints.len() {
                self.target_rad[i] = self.latest_telemetry.measured_rad[i];
            }
            self.status_message =
                "Switched Teach -> Drive (Bumpless sync: 0.000 rad delta).".to_string();
        } else if mode == ControlMode::Teach {
            self.playback = None;
            self.status_message = "Switched to Teach mode (Robot -> GUI, Torque OFF).".to_string();
        }
        self.requested_mode = mode;
    }

    /// Enables or disables torque on a single joint while enforcing bumpless sync when enabling.
    pub fn set_joint_torque(&mut self, joint: usize, enabled: bool) {
        if joint >= self.rig.joints.len() {
            return;
        }
        let bit = 1u32 << joint;
        let was_enabled = (self.torque_mask & bit) != 0;
        if enabled {
            if !was_enabled {
                self.target_rad[joint] = self.latest_telemetry.measured_rad[joint];
            }
            self.torque_mask |= bit;
        } else {
            self.torque_mask &= !bit;
        }
    }

    /// Captures live `measured_rad` as a new keyframe on the selected combat move (Teach mode workflow).
    pub fn capture_keyframe(&mut self) {
        if let Some(mv) = self.moves.get_mut(self.selected_move) {
            let n = mv.keyframes.len() + 1;
            mv.keyframes.push(Keyframe {
                label: format!("Captured Pose {n}"),
                duration_ms: 450,
                angles_rad: self.latest_telemetry.measured_rad,
            });
            self.status_message = format!("Captured keyframe #{n} into '{}'.", mv.name);
        }
    }

    /// Starts cubic-interpolated playback of a combat move (Drive mode only).
    pub fn play_move(&mut self, move_idx: usize) {
        if self.requested_mode != ControlMode::Drive || self.is_estop_active() {
            self.status_message =
                "Switch to Drive mode before playing combat actions.".to_string();
            return;
        }
        if move_idx < self.moves.len() {
            self.selected_move = move_idx;
            self.playback = Some(PlaybackState {
                move_idx,
                elapsed_ms: 0.0,
                start_pose: self.latest_telemetry.measured_rad,
                progress: 0.0,
                active_keyframe: 0,
            });
            self.status_message = format!("Playing '{}'.", self.moves[move_idx].name);
        }
    }

    pub fn stop_playback(&mut self) {
        if self.playback.take().is_some() {
            self.target_rad = self.latest_telemetry.measured_rad;
            self.status_message = "Playback stopped.".to_string();
        }
    }

    /// Sets the current raw angle of `joint` as zero offset via `ConfigCmd::SetZeroOffset`.
    pub fn set_current_as_zero(&mut self, joint: usize) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            let measured = self.latest_telemetry.measured_rad[joint];
            let sign = if cal.inverted { -1.0_f32 } else { 1.0_f32 };
            let raw = (sign * measured) + cal.zero_offset_rad;
            cal.zero_offset_rad = raw;
            self.target_rad[joint] = 0.0;
            self.pending_config.push(ConfigCmd::SetZeroOffset {
                joint: joint as u8,
                zero_rad: raw,
            });
            self.status_message = format!(
                "Zeroed joint '{}' (offset {:.2} deg).",
                self.rig.joints[joint].label,
                raw.to_degrees()
            );
        }
    }

    pub fn toggle_invert(&mut self, joint: usize) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            cal.inverted = !cal.inverted;
            let inv = cal.inverted;
            self.pending_config.push(ConfigCmd::SetInvert {
                joint: joint as u8,
                inverted: inv,
            });
        }
    }

    pub fn update_soft_limits(&mut self, joint: usize, min_rad: f32, max_rad: f32) {
        if let Some(cal) = self.calibration.get_mut(joint) {
            let (lo, hi) = if min_rad <= max_rad {
                (min_rad, max_rad)
            } else {
                (max_rad, min_rad)
            };
            cal.min_rad = lo;
            cal.max_rad = hi;
            self.pending_config.push(ConfigCmd::SetSoftLimits {
                joint: joint as u8,
                min_rad: lo,
                max_rad: hi,
            });
        }
    }

    pub fn export_calibration_json(&self) -> Result<String, String> {
        let file = CalibrationFile {
            robot_name: self.rig.robot_name.clone(),
            joints: self.calibration.clone(),
        };
        serde_json::to_string_pretty(&file).map_err(|e| format!("Serialize error: {e}"))
    }

    pub fn import_calibration_json(&mut self, json: &str) -> Result<(), String> {
        let file: CalibrationFile =
            serde_json::from_str(json).map_err(|e| format!("Invalid calibration JSON: {e}"))?;
        for (idx, cal) in file.joints.into_iter().enumerate() {
            if idx < self.calibration.len() {
                self.calibration[idx] = cal;
                self.pending_config.push(ConfigCmd::SetZeroOffset {
                    joint: idx as u8,
                    zero_rad: cal.zero_offset_rad,
                });
                self.pending_config.push(ConfigCmd::SetInvert {
                    joint: idx as u8,
                    inverted: cal.inverted,
                });
                self.pending_config.push(ConfigCmd::SetSoftLimits {
                    joint: idx as u8,
                    min_rad: cal.min_rad,
                    max_rad: cal.max_rad,
                });
            }
        }
        self.status_message = "Calibration imported and queued for MCU.".to_string();
        Ok(())
    }

    /// Advances active keyframe playback and constructs the next `HostToMcu` packet.
    pub fn build_next_host_packet(&mut self, dt_sec: f32) -> HostToMcu {
        if self.requested_mode == ControlMode::Drive && !self.is_estop_active() {
            if let Some(pb) = self.playback.as_mut() {
                pb.elapsed_ms += dt_sec * 1000.0;
                if let Some(mv) = self.moves.get(pb.move_idx) {
                    let (pose, done, progress, active_kf) = sample_action_move(
                        mv,
                        &pb.start_pose,
                        pb.elapsed_ms,
                        self.speed_limit_pct,
                    );
                    self.target_rad = pose;
                    pb.progress = progress;
                    pb.active_keyframe = active_kf;
                    if done {
                        self.playback = None;
                    }
                } else {
                    self.playback = None;
                }
            }
        } else if self.requested_mode == ControlMode::Teach {
            self.target_rad = self.latest_telemetry.measured_rad;
        }

        self.tx_seq = self.tx_seq.wrapping_add(1);
        let config = if !self.pending_config.is_empty() {
            self.pending_config.remove(0)
        } else {
            ConfigCmd::None
        };

        let mode = if self.is_estop_active() && config != ConfigCmd::ClearFaults {
            ControlMode::EStop
        } else {
            self.requested_mode
        };

        let torque_mask = if mode == ControlMode::Drive {
            self.torque_mask
        } else {
            0
        };

        HostToMcu {
            seq: self.tx_seq,
            mode,
            torque_mask,
            speed_limit_pct: self.speed_limit_pct.clamp(1, 100),
            target_rad: self.target_rad,
            walk: self.walk_cmd,
            config,
        }
    }

    pub fn ingest_telemetry(&mut self, telem: McuToHost) {
        if telem.active_mode == ControlMode::EStop && self.requested_mode == ControlMode::Drive {
            self.requested_mode = ControlMode::EStop;
            self.playback = None;
        }
        self.latest_telemetry = telem;
    }
}
