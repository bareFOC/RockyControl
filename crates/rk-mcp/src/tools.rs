//! `craft_mcp_server::Tools` implementation for Rocky Studio (`RockyMcpTools`).

use std::sync::atomic::{AtomicU64, Ordering};

use craft_mcp_server::{Contents, Info, Options, Reference, Server, Tools, text_result};
use rk_link::TransportKind;
use rk_model::nalgebra::Point3;
use rk_model::{AssetBundle, samples};
use rk_fw_core::replay::verify_deterministic_replay;
use rk_proto::{BalanceGains, ControlMode, FallOrientation, ModelRequest, ModelResponse, uvc_phase};
use rk_sim::{ExperimentScript, WalkQualityBounds};
use serde_json::{Value, json};

use crate::host::{McpActivityEntry, McpCallOutcome, McpHost};
use crate::schema::{
    EXPERIMENT_FIXTURES, URDF_PRESETS, prompt_definitions, resource_definitions,
    resource_template_definitions, tool_definitions,
};

const MAX_URDF_BYTES: usize = 2 * 1024 * 1024;
const MAX_LINE_BYTES: usize = 2_621_440;
const MAX_STEP_TICKS: usize = 400;
/// Upper bound on a scripted experiment so a malformed script cannot monopolize
/// the headless runner (each tick allocates one frozen `ControlTick` record).
const MAX_SCRIPT_TICKS: usize = 20_000;

static NEXT_ACTIVITY_SEQ: AtomicU64 = AtomicU64::new(1);

/// Compile-time copies of the committed fixtures. Native hosts prefer a confined
/// file read so edited fixtures are honoured; the GUI bridge (no filesystem)
/// uses these exact bytes, keeping replay identical in both paths.
fn embedded_experiment(name: &str) -> Option<&'static str> {
    match name {
        "walk_nominal.json" => Some(include_str!("../../../tests/experiments/walk_nominal.json")),
        "walk_shove.json" => Some(include_str!("../../../tests/experiments/walk_shove.json")),
        "fall_recover.json" => Some(include_str!("../../../tests/experiments/fall_recover.json")),
        _ => None,
    }
}

/// Canonical fixture name for a tool argument; accepts `walk_nominal` and
/// `walk_nominal.json` but never an arbitrary path.
fn resolve_experiment_name(requested: &str) -> Option<&'static str> {
    EXPERIMENT_FIXTURES
        .iter()
        .map(|(name, _)| *name)
        .find(|name| {
            *name == requested
                || name.strip_suffix(".json") == Some(requested)
        })
}

/// Standard `craft-mcp-server` server identity & bounded line length.
pub const fn server_info() -> Info {
    Info {
        name: "rocky-studio",
        title: "Rocky Studio Robotics MCP",
        version: env!("CARGO_PKG_VERSION"),
        max_line: MAX_LINE_BYTES,
    }
}

/// Standard `craft-mcp-server` options (`tool_errors_as_results` so agents always see structured errors).
pub fn server_options() -> Options {
    Options {
        website_url: None,
        tool_errors_as_results: true,
        strict_request_ids: true,
    }
}

/// Dispatches one JSON-RPC 2.0 message against any [`McpHost`] through `craft-mcp-server::Server`.
/// Enforces the master `mcp_enabled` toggle cleanly before invoking any method.
pub fn dispatch_json_rpc<H: McpHost>(host: &mut H, msg: Value) -> Option<Value> {
    if !host.policy().enabled {
        let id = msg.get("id").cloned();
        let method = msg
            .get("method")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        host.record_activity(McpActivityEntry {
            seq: NEXT_ACTIVITY_SEQ.fetch_add(1, Ordering::Relaxed),
            method,
            summary: "blocked".to_string(),
            outcome: McpCallOutcome::Guarded,
            detail: "MCP is disabled in Rocky Studio".to_string(),
        });
        return id.map(|id_val| {
            json!({
                "jsonrpc": "2.0",
                "id": id_val,
                "error": {
                    "code": -32000,
                    "message": "MCP access is disabled in Rocky Studio. Enable MCP on the Home screen or MCP drawer."
                }
            })
        });
    }

    let mut srv = Server::new(server_info(), RockyMcpTools::new(host)).with_options(server_options());
    srv.handle(msg)
}

/// Rocky Studio implementation of `craft_mcp_server::Tools`.
pub struct RockyMcpTools<'a, H: McpHost> {
    host: &'a mut H,
}

impl<'a, H: McpHost> RockyMcpTools<'a, H> {
    pub fn new(host: &'a mut H) -> Self {
        Self { host }
    }

    fn log(&mut self, method: &str, summary: impl Into<String>, outcome: McpCallOutcome, detail: impl Into<String>) {
        self.host.record_activity(McpActivityEntry {
            seq: NEXT_ACTIVITY_SEQ.fetch_add(1, Ordering::Relaxed),
            method: method.to_string(),
            summary: summary.into(),
            outcome,
            detail: detail.into(),
        });
    }

    fn ok_json(&mut self, tool_name: &str, summary: impl Into<String>, payload: Value) -> Value {
        let s = summary.into();
        self.log(tool_name, s.clone(), McpCallOutcome::Ok, s);
        text_result(
            serde_json::to_string_pretty(&payload).unwrap_or_else(|_| payload.to_string()),
            false,
        )
    }

    fn err_result(&mut self, tool_name: &str, summary: impl Into<String>, msg: impl Into<String>) -> Value {
        let m = msg.into();
        self.log(tool_name, summary, McpCallOutcome::Error, m.clone());
        text_result(m, true)
    }

    /// Structured tool error: the `content[0].text` is a JSON object with a
    /// machine-readable `error.code` for agents that need to self-correct.
    fn err_payload(&mut self, tool_name: &str, summary: impl Into<String>, payload: Value) -> Value {
        let message = payload
            .get("error")
            .and_then(|e| e.get("message"))
            .and_then(Value::as_str)
            .unwrap_or("tool error")
            .to_string();
        self.log(tool_name, summary, McpCallOutcome::Error, message);
        text_result(
            serde_json::to_string_pretty(&payload).unwrap_or_else(|_| payload.to_string()),
            true,
        )
    }

    /// Invalid gain writes carry the offending field, its raw value and the
    /// accepted band so an agent can retry without reading prose.
    fn invalid_gain_result(
        &mut self,
        tool_name: &str,
        field: &str,
        value: Value,
        lo: f32,
        hi: f32,
        message: String,
    ) -> Value {
        self.err_payload(
            tool_name,
            format!("invalid `{field}`"),
            json!({
                "error": {
                    "code": "invalid_gain",
                    "field": field,
                    "value": value,
                    "bounds": [lo, hi],
                    "message": message,
                }
            }),
        )
    }

    fn guarded_result(&mut self, tool_name: &str, summary: impl Into<String>, msg: impl Into<String>) -> Value {
        let m = msg.into();
        self.log(tool_name, summary, McpCallOutcome::Guarded, m.clone());
        text_result(m, true)
    }

    fn resolve_joint(&self, val: &Value) -> Result<usize, String> {
        let joints = &self.host.model().rig.joints;
        if let Some(idx) = val.as_u64() {
            let idx = idx as usize;
            if idx < joints.len() {
                return Ok(idx);
            }
            return Err(format!("joint index {idx} out of range (0..{})", joints.len()));
        }
        if let Some(name) = val.as_str() {
            if let Ok(idx) = name.parse::<usize>() {
                if idx < joints.len() {
                    return Ok(idx);
                }
            }
            if let Some(j) = joints
                .iter()
                .find(|j| j.name.eq_ignore_ascii_case(name) || j.label.eq_ignore_ascii_case(name))
            {
                return Ok(j.index);
            }
            return Err(format!("unknown joint `{name}`"));
        }
        Err("joint must be an index integer or joint name string".to_string())
    }

    fn finite_f32(args: &Value, key: &str, min: f32, max: f32) -> Result<Option<f32>, String> {
        let Some(v) = args.get(key) else {
            return Ok(None);
        };
        if v.is_null() {
            return Ok(None);
        }
        let Some(f) = v.as_f64() else {
            return Err(format!("argument `{key}` must be a finite number"));
        };
        let f32_val = f as f32;
        if !f32_val.is_finite() {
            return Err(format!("argument `{key}` must be finite (no NaN/Infinity)"));
        }
        if f32_val < min || f32_val > max {
            return Err(format!("argument `{key}`={f32_val} outside [{min}, {max}]"));
        }
        Ok(Some(f32_val))
    }

    /// Full walk-quality snapshot: the 14 metrics both at the top level and
    /// under `quality` (the same shape used by `run_experiment`), plus the
    /// acceptance `bounds`, per-metric `violations` and `pass` verdict from the
    /// shared `rk-sim` evaluator. Wired transports that cannot measure quality
    /// return `{"quality": null, "pass": false}` with the bounds still present.
    fn quality_json(&self) -> Value {
        let bounds = WalkQualityBounds::default();
        let bounds_json = serde_json::to_value(bounds).unwrap_or(Value::Null);
        let Some(metrics) = self.host.walk_quality() else {
            return json!({
                "quality": Value::Null,
                "bounds": bounds_json,
                "violations": [],
                "pass": false,
            });
        };
        let evaluation = bounds.evaluate(&metrics);
        let Ok(metrics_json) = serde_json::to_value(metrics) else {
            return json!({
                "quality": Value::Null,
                "bounds": bounds_json,
                "violations": [],
                "pass": false,
            });
        };
        let Value::Object(fields) = metrics_json else {
            return json!({
                "quality": Value::Null,
                "bounds": bounds_json,
                "violations": [],
                "pass": false,
            });
        };
        let mut payload = fields.clone();
        payload.insert("quality".to_string(), Value::Object(fields));
        payload.insert("bounds".to_string(), bounds_json);
        payload.insert(
            "violations".to_string(),
            serde_json::to_value(evaluation.violations).unwrap_or(Value::Null),
        );
        payload.insert("pass".to_string(), json!(evaluation.pass));
        Value::Object(payload)
    }

    /// Runs an already-parsed script and builds the shared `run_experiment`
    /// response. Both the inline/file runner and `experiment_run_named` use it
    /// so their verdict and quality payloads cannot diverge.
    fn run_experiment_script(&mut self, name: &str, script: ExperimentScript) -> Value {
        if script.max_ticks == 0 || script.packets.is_empty() {
            return self.err_result(
                name,
                "empty script",
                "A script needs at least one tick (`max_ticks > 0`) and one typed host packet.",
            );
        }
        if script.max_ticks > MAX_SCRIPT_TICKS {
            return self.err_result(
                name,
                "script too large",
                format!("`max_ticks` exceeds the {MAX_SCRIPT_TICKS}-tick safety cap"),
            );
        }

        match self.host.run_experiment(&script) {
            Ok(result) => {
                let check = result.verification;
                let verification = json!({
                    "exact_match": check.exact_match,
                    "matched_ticks": check.matched_ticks,
                    "total_ticks": check.total_ticks,
                });
                let quality = result
                    .quality
                    .and_then(|q| serde_json::to_value(q).ok())
                    .unwrap_or(Value::Null);
                self.ok_json(
                    name,
                    if check.exact_match { "experiment EXACT" } else { "experiment MISMATCH" },
                    json!({
                        // Top-level verdict keys are kept for existing agents.
                        "exact_match": check.exact_match,
                        "matched_ticks": check.matched_ticks,
                        "total_ticks": check.total_ticks,
                        "verification": verification,
                        "quality": quality,
                    }),
                )
            }
            Err(e) => self.err_result(name, "experiment failed", e),
        }
    }

    fn status_json(&self) -> Value {
        let m = self.host.model();
        let pol = self.host.policy();
        let stats = self.host.transport().stats();
        json!({
            "connected": self.host.is_connected(),
            "transport": self.host.transport().kind().label(),
            "active_screen": self.host.active_screen_name(),
            "robot_name": m.rig.robot_name,
            "dof": m.rig.joints.len(),
            "requested_mode": format!("{:?}", m.requested_mode),
            "active_mode": format!("{:?}", m.latest_telemetry.active_mode),
            "estop_active": m.is_estop_active(),
            "speed_limit_pct": m.speed_limit_pct,
            "faults": format!("{:?}", m.latest_telemetry.faults),
            "policy": {
                "mcp_enabled": pol.enabled,
                "agent_drive_armed": pol.drive_armed,
            },
            "link": {
                "packet_rate_hz": stats.packet_rate_hz,
                "rtt_ms": stats.rtt_ms,
                "link_errors": stats.link_errors,
            },
            "status_message": m.status_message,
        })
    }

    fn telemetry_json(&self) -> Value {
        let m = self.host.model();
        let t = &m.latest_telemetry;
        let b = &t.balance;
        let dof = m.rig.joints.len().min(rk_proto::MAX_JOINTS);
        let joints: Vec<Value> = (0..dof)
            .map(|i| {
                let j = &m.rig.joints[i];
                json!({
                    "index": i,
                    "name": j.name,
                    "label": j.label,
                    "measured_rad": t.measured_rad[i],
                    "target_rad": m.target_rad[i],
                    "measured_deg": t.measured_rad[i].to_degrees(),
                    "current_ma": t.current_ma[i],
                    "temp_c": t.temp_c[i],
                })
            })
            .collect();
        let peak_ma = (0..dof).map(|i| t.current_ma[i]).max().unwrap_or(0);
        let max_temp_c = (0..dof).map(|i| t.temp_c[i]).max().unwrap_or(0);
        json!({
            "connected": self.host.is_connected(),
            "active_mode": format!("{:?}", t.active_mode),
            "estop_active": m.is_estop_active(),
            "bus_voltage_v": (t.bus_voltage_mv as f32) / 1000.0,
            "peak_current_ma": peak_ma,
            "max_temp_c": max_temp_c,
            "faults": format!("{:?}", t.faults),
            "imu": {
                "available": t.imu.available,
                "pitch_rad": t.imu.pitch_rad,
                "roll_rad": t.imu.roll_rad,
                "pitch_deg": t.imu.pitch_rad.to_degrees(),
                "roll_deg": t.imu.roll_rad.to_degrees(),
                "pitch_rate_rad_s": t.imu.pitch_rate_rad_s,
                "roll_rate_rad_s": t.imu.roll_rate_rad_s,
            },
            "uvc": {
                "balance_active": b.balance_active,
                "walk_active": b.walk_active,
                "phase": uvc_phase::label(b.uvc_phase),
                "phase_code": b.uvc_phase,
                "support_foot": if b.support_foot == 1 { "left" } else { "right" },
                "foot_dx_m": b.foot_dx_m,
                "foot_dy_m": b.foot_dy_m,
                "hip_height_m": b.hip_height_m,
                "pitch_corr_rad": b.pitch_corr_rad,
                "roll_corr_rad": b.roll_corr_rad,
                "gait_phase_rad": b.gait_phase_rad,
                "fall_orientation": FallOrientation::from_wire(b.fall_orientation).label(),
                "fall_orientation_code": b.fall_orientation,
            },
            "joints": joints,
            "estimator": {
                "com_pos_m": t.estimator.com_pos_m,
                "com_vel_m_s": t.estimator.com_vel_m_s,
                "dcm_xy_m": t.estimator.dcm_xy_m,
                "zmp_xy_m": t.estimator.zmp_xy_m,
                "foot_margin_m": t.estimator.foot_margin_m,
                "contact_mask": t.estimator.contact_mask,
                "ukemi_active": t.estimator.ukemi_active,
                "valid_mask": t.estimator.valid_mask,
            },
            "wire": {
                "protocol_version": t.protocol_version,
                "rx_crc_errors": t.rx_crc_errors,
                "rx_version_mismatches": t.rx_version_mismatches,
                "i3c_timeout_count": t.i3c_timeout_count,
            },
        })
    }

    fn robot_json(&self) -> Value {
        let m = self.host.model();
        let joints: Vec<Value> = m
            .rig
            .joints
            .iter()
            .map(|j| {
                json!({
                    "index": j.index,
                    "name": j.name,
                    "label": j.label,
                    "kind": format!("{:?}", j.kind),
                    "limb_group": format!("{:?}", j.group),
                    "child_link": j.child_link,
                    "default_min_rad": j.default_min_rad,
                    "default_max_rad": j.default_max_rad,
                })
            })
            .collect();
        let presets: Vec<Value> = URDF_PRESETS
            .iter()
            .map(|(id, desc)| json!({"id": id, "description": desc}))
            .collect();
        json!({
            "robot_name": m.rig.robot_name,
            "dof": m.rig.joints.len(),
            "link_count": m.rig.links.len(),
            "up_axis": m.rig.up_axis.label(),
            "joints": joints,
            "presets": presets,
        })
    }

    fn calibration_json(&self) -> Value {
        let m = self.host.model();
        let rows: Vec<Value> = m
            .rig
            .joints
            .iter()
            .enumerate()
            .map(|(i, j)| {
                let cal = m.calibration.get(i).copied().unwrap_or_default();
                json!({
                    "index": i,
                    "name": j.name,
                    "label": j.label,
                    "zero_offset_rad": cal.zero_offset_rad,
                    "min_rad": cal.min_rad,
                    "max_rad": cal.max_rad,
                    "inverted": cal.inverted,
                    "measured_rad": m.latest_telemetry.measured_rad[i],
                })
            })
            .collect();
        json!({
            "robot_name": m.rig.robot_name,
            "joints": rows,
        })
    }

    fn balance_json(&self) -> Value {
        let m = self.host.model();
        let g = m.balance_gains;
        let w = m.walk_cmd;
        json!({
            "gains": {
                "uvc_gain_pitch": g.uvc_gain_pitch,
                "uvc_gain_roll": g.uvc_gain_roll,
                "tilt_deadband_rad": g.tilt_deadband_rad,
                "gyro_damp_pitch": g.gyro_damp_pitch,
                "gyro_damp_roll": g.gyro_damp_roll,
                "arm_assist": g.arm_assist,
                "fall_cutoff_rad": g.fall_cutoff_rad,
                "dcm_step_gain": g.dcm_step_gain,
                "foot_half_length_m": g.foot_half_length_m,
                "foot_margin_m": g.foot_margin_m,
                "h_com_m": g.h_com_m,
                "ukemi_tilt_rad": g.ukemi_tilt_rad,
                "ukemi_torque_scale": g.ukemi_torque_scale,
                "compliance_scale": g.compliance_scale,
            },
            "walk": {
                "balance_enabled": w.balance_enabled,
                "walk_enabled": w.walk_enabled,
                "vx": w.vx,
                "vy": w.vy,
                "yaw_rate": w.yaw_rate,
                "step_height": w.step_height,
                "cadence_hz": w.cadence_hz,
                "stance_crouch": w.stance_crouch,
            }
        })
    }

    fn actions_json(&self) -> Value {
        let m = self.host.model();
        let actions: Vec<Value> = m
            .moves
            .iter()
            .enumerate()
            .map(|(i, mv)| {
                json!({
                    "index": i,
                    "name": mv.name,
                    "category": mv.category,
                    "keyframes": mv.keyframes.len(),
                })
            })
            .collect();
        let playing = m.playback.as_ref().map(|pb| {
            json!({
                "move_idx": pb.move_idx,
                "name": m.moves.get(pb.move_idx).map(|mv| mv.name.as_str()).unwrap_or(""),
                "progress": pb.progress,
                "active_keyframe": pb.active_keyframe,
            })
        });
        json!({
            "actions": actions,
            "playback": playing,
        })
    }

    fn single_joint_json(&self, name_or_idx: &str) -> Result<Value, String> {
        let idx = self.resolve_joint(&Value::String(name_or_idx.to_string()))?;
        let m = self.host.model();
        let j = &m.rig.joints[idx];
        let cal = m.calibration.get(idx).copied().unwrap_or_default();
        let t = &m.latest_telemetry;
        Ok(json!({
            "index": idx,
            "name": j.name,
            "label": j.label,
            "kind": format!("{:?}", j.kind),
            "limb_group": format!("{:?}", j.group),
            "measured_rad": t.measured_rad[idx],
            "target_rad": m.target_rad[idx],
            "measured_deg": t.measured_rad[idx].to_degrees(),
            "current_ma": t.current_ma[idx],
            "temp_c": t.temp_c[idx],
            "calibration": {
                "zero_offset_rad": cal.zero_offset_rad,
                "min_rad": cal.min_rad,
                "max_rad": cal.max_rad,
                "inverted": cal.inverted,
            }
        }))
    }
}

impl<H: McpHost> Tools for RockyMcpTools<'_, H> {
    fn instructions(&self) -> String {
        "Rocky Studio Robotics MCP Server. Use resources (rocky://status, rocky://telemetry, rocky://quality, rocky://robot, rocky://calibration, rocky://balance, rocky://actions) to inspect live robot state. Configuration, URDF loading, calibration, UVC balance tuning, and Teach posing are always permitted when MCP is enabled. Switching to energized Drive mode or commanding active locomotion requires the human operator to arm 'Allow Agent Drive' in Rocky Studio (or --allow-drive on CLI). E-Stop always cuts torque immediately and disarms Drive.".to_string()
    }

    fn list(&mut self) -> Vec<Value> {
        tool_definitions()
    }

    fn resources(&self) -> Vec<Value> {
        resource_definitions()
    }

    fn resource_templates(&self) -> Vec<Value> {
        resource_template_definitions()
    }

    fn read_resource(&mut self, uri: &str) -> Option<Result<Contents, String>> {
        let result = match uri {
            "rocky://status" => Ok(Contents::json(&self.status_json())),
            "rocky://telemetry" => Ok(Contents::json(&self.telemetry_json())),
            "rocky://quality" => Ok(Contents::json(&self.quality_json())),
            "rocky://robot" => Ok(Contents::json(&self.robot_json())),
            "rocky://calibration" => Ok(Contents::json(&self.calibration_json())),
            "rocky://balance" => Ok(Contents::json(&self.balance_json())),
            "rocky://actions" => Ok(Contents::json(&self.actions_json())),
            _ if uri.starts_with("rocky://joints/") => {
                let joint_key = &uri["rocky://joints/".len()..];
                self.single_joint_json(joint_key)
                    .map(|val| Contents::json(&val))
            }
            _ => return None,
        };
        let outcome = if result.is_ok() {
            McpCallOutcome::Ok
        } else {
            McpCallOutcome::Error
        };
        self.log("resources/read", uri, outcome, uri);
        Some(result)
    }

    fn prompts(&self) -> Vec<Value> {
        prompt_definitions()
    }

    fn get_prompt(&mut self, name: &str, args: &Value) -> Result<Value, String> {
        let (desc, text) = match name {
            "calibrate_robot" => {
                let preset = args.get("preset").and_then(Value::as_str).unwrap_or("current");
                (
                    "Guided joint calibration workflow",
                    format!(
                        "Calibrate Rocky Studio robot (preset={preset}): 1) Call robot_connect if disconnected. 2) Read rocky://robot and rocky://calibration. 3) Keep mode in Teach, inspect soft limits and zero offsets, adjust any out-of-range joints with calibration_set_joint, and verify with calibration_export_json."
                    ),
                )
            }
            "tune_uvc_balance" => {
                let shove = args
                    .get("target_shove_rad_s")
                    .and_then(Value::as_str)
                    .unwrap_or("1.6");
                (
                    "Autonomous UVC balance tuning loop",
                    format!(
                        "Tune Rocky's geometric UVC balance against a {shove} rad/s shove: 1) Ensure robot_connect(transport='sim') and mode_set(mode='drive') (if armed). 2) Read rocky://balance and rocky://telemetry. 3) Run sim_push_shove(pitch_impulse_rad_s={shove}, roll_impulse_rad_s=-0.9, step_ticks=260). 4) Adjust balance_set_gains (uvc_gain_pitch, uvc_gain_roll, gyro_damp_pitch, gyro_damp_roll) until residual pitch/roll settles within +-3 deg with clean catch-stepping."
                    ),
                )
            }
            "verify_shove_recovery" => (
                "Shove recovery & protective fall-cutoff check",
                "Verify Rocky safety: 1) With balance enabled in Drive, inject sim_push_shove(1.6, -1.0, step_ticks=280) and confirm upright recovery. 2) Disable balance via balance_set_enabled(false), inject sim_push_shove(2.1, 0.0, step_ticks=200), confirm FallTuck -> EStop latches TILT_FALL_CUTOFF, then call safety_reset_estop.".to_string(),
            ),
            "diagnose_faults" => (
                "Telemetry & fault diagnosis",
                "Read rocky://status and rocky://telemetry. Check bus voltage, peak servo currents, max temperatures, IMU tilt, and latched faults, then explain root causes and safe recovery actions.".to_string(),
            ),
            _ => return Err(format!("unknown prompt `{name}`")),
        };
        self.log("prompts/get", name, McpCallOutcome::Ok, desc);
        Ok(json!({
            "description": desc,
            "messages": [
                {
                    "role": "user",
                    "content": {
                        "type": "text",
                        "text": text
                    }
                }
            ]
        }))
    }

    fn complete(&mut self, reference: Reference<'_>, argument: &str, value: &str) -> Vec<String> {
        let needle = value.to_ascii_lowercase();
        let candidates: Vec<String> = match (reference, argument) {
            (Reference::Resource("rocky://joints/{joint_name}"), "joint_name") => self
                .host
                .model()
                .rig
                .joints
                .iter()
                .map(|j| j.name.clone())
                .collect(),
            (Reference::Prompt("calibrate_robot"), "preset") => URDF_PRESETS
                .iter()
                .map(|(id, _)| (*id).to_string())
                .collect(),
            (Reference::Prompt("tune_uvc_balance"), "target_shove_rad_s") => {
                vec!["1.05".into(), "1.65".into(), "2.20".into()]
            }
            _ => Vec::new(),
        };
        candidates
            .into_iter()
            .filter(|c| c.to_ascii_lowercase().starts_with(&needle))
            .collect()
    }

    fn call(&mut self, name: &str, args: &Value) -> Value {
        match name {
            "robot_connect" => {
                let transport_str = args
                    .get("transport")
                    .and_then(Value::as_str)
                    .unwrap_or("sim");
                let kind = match transport_str {
                    "sim" => TransportKind::SimKernel,
                    "usb_udp" => TransportKind::UsbNetworkingUdp,
                    "usb_serial" => TransportKind::UsbSerialStage2,
                    other => {
                        return self.err_result(name, other, format!("unknown transport `{other}`"));
                    }
                };
                match self.host.connect(kind) {
                    Ok(msg) => self.ok_json(
                        name,
                        format!("connected ({transport_str})"),
                        json!({"message": msg, "status": self.status_json()}),
                    ),
                    Err(err) => self.err_result(name, transport_str, err),
                }
            }
            "robot_disconnect" => {
                self.host.disconnect();
                self.ok_json(
                    name,
                    "disconnected",
                    json!({"message": "Disconnected safely in Teach mode.", "status": self.status_json()}),
                )
            }
            "studio_navigate" => {
                let screen = args.get("screen").and_then(Value::as_str).unwrap_or("dashboard");
                match self.host.navigate_screen(screen) {
                    Ok(()) => self.ok_json(name, screen, json!({"active_screen": self.host.active_screen_name()})),
                    Err(err) => self.err_result(name, screen, err),
                }
            }
            "viewport_camera" => {
                let preset = args.get("preset").and_then(Value::as_str).unwrap_or("iso");
                match self.host.set_camera_preset(preset) {
                    Ok(()) => self.ok_json(name, preset, json!({"camera_preset": preset})),
                    Err(err) => self.err_result(name, preset, err),
                }
            }
            "telemetry_snapshot" => {
                let ticks = args
                    .get("step_ticks")
                    .and_then(Value::as_u64)
                    .unwrap_or(0) as usize;
                if ticks > MAX_STEP_TICKS {
                    return self.err_result(
                        name,
                        format!("ticks={ticks}"),
                        format!("step_ticks must be <= {MAX_STEP_TICKS}"),
                    );
                }
                if ticks > 0 && self.host.is_connected() {
                    self.host.step_ticks(ticks, 0.01);
                }
                self.ok_json(name, format!("ticks={ticks}"), self.telemetry_json())
            }
            "quality_snapshot" => {
                let is_sim = self.host.walk_quality().is_some();
                self.ok_json(
                    name,
                    if is_sim { "walk quality" } else { "quality unavailable" },
                    self.quality_json(),
                )
            }
            "robot_list_presets" => {
                let presets: Vec<Value> = URDF_PRESETS
                    .iter()
                    .map(|(id, desc)| json!({"id": id, "description": desc}))
                    .collect();
                self.ok_json(name, "presets", json!({"presets": presets}))
            }
            "robot_load_preset" => {
                let preset = args.get("preset").and_then(Value::as_str).unwrap_or("");
                let bundle = match preset {
                    "humanoid" => Ok(AssetBundle::bundled_humanoid()),
                    "striker_biped" => {
                        AssetBundle::from_sample("Striker Biped Z-Up", samples::STRIKER_BIPED_ZUP_URDF)
                    }
                    "combat_arm" => {
                        AssetBundle::from_sample("Combat Arm 6-DoF", samples::COMBAT_ARM_6DOF_URDF)
                    }
                    other => Err(format!("unknown preset `{other}`")),
                };
                match bundle.and_then(|b| self.host.load_bundle(b)) {
                    Ok(msg) => self.ok_json(
                        name,
                        preset,
                        json!({"message": msg, "robot": self.robot_json()}),
                    ),
                    Err(err) => self.err_result(name, preset, err),
                }
            }
            "robot_load_urdf" => {
                let label = args
                    .get("name")
                    .and_then(Value::as_str)
                    .unwrap_or("agent_robot.urdf");
                let xml_bytes: Result<Vec<u8>, String> = if let Some(xml) =
                    args.get("urdf_xml").and_then(Value::as_str)
                {
                    if xml.len() > MAX_URDF_BYTES {
                        Err(format!("urdf_xml exceeds {MAX_URDF_BYTES} byte safety cap"))
                    } else {
                        Ok(xml.as_bytes().to_vec())
                    }
                } else if let Some(path) = args.get("file_path").and_then(Value::as_str) {
                    self.host.read_confined_file(path).and_then(|b| {
                        if b.len() > MAX_URDF_BYTES {
                            Err(format!("URDF file exceeds {MAX_URDF_BYTES} byte safety cap"))
                        } else {
                            Ok(b)
                        }
                    })
                } else {
                    Err("provide either `urdf_xml` or `file_path`".to_string())
                };
                match xml_bytes
                    .and_then(|bytes| AssetBundle::from_bytes(label, &bytes))
                    .and_then(|b| self.host.load_bundle(b))
                {
                    Ok(msg) => self.ok_json(
                        name,
                        label,
                        json!({"message": msg, "robot": self.robot_json()}),
                    ),
                    Err(err) => self.err_result(name, label, err),
                }
            }
            "mcu_sync_model" => match self.host.sync_mcu_model() {
                Ok(msg) => self.ok_json(name, "sync", json!({"message": msg})),
                Err(err) => self.err_result(name, "sync", err),
            },
            "calibration_get" => self.ok_json(name, "read", self.calibration_json()),
            "calibration_set_joint" => {
                let idx = match args.get("joint").ok_or_else(|| "missing `joint`".to_string()).and_then(|v| self.resolve_joint(v)) {
                    Ok(i) => i,
                    Err(err) => return self.err_result(name, "invalid joint", err),
                };
                let min_rad = match Self::finite_f32(args, "min_rad", -6.2832, 6.2832) {
                    Ok(v) => v,
                    Err(e) => return self.err_result(name, "min_rad", e),
                };
                let max_rad = match Self::finite_f32(args, "max_rad", -6.2832, 6.2832) {
                    Ok(v) => v,
                    Err(e) => return self.err_result(name, "max_rad", e),
                };
                let zero_off = match Self::finite_f32(args, "zero_offset_rad", -6.2832, 6.2832) {
                    Ok(v) => v,
                    Err(e) => return self.err_result(name, "zero_offset_rad", e),
                };
                let inverted = args.get("inverted").and_then(Value::as_bool);

                let cur = self.host.model().calibration[idx];
                if min_rad.is_some() || max_rad.is_some() {
                    let lo = min_rad.unwrap_or(cur.min_rad);
                    let hi = max_rad.unwrap_or(cur.max_rad);
                    self.host.model_mut().update_soft_limits(idx, lo, hi);
                }
                if let Some(off) = zero_off {
                    let m = self.host.model_mut();
                    m.calibration[idx].zero_offset_rad = off;
                    m.pending_config.push(rk_proto::ConfigCmd::SetZeroOffset {
                        joint: idx as u8,
                        zero_rad: off,
                    });
                }
                if let Some(inv) = inverted {
                    let m = self.host.model_mut();
                    if m.calibration[idx].inverted != inv {
                        m.toggle_invert(idx);
                    }
                }
                if self.host.is_connected() {
                    self.host.step_ticks(3, 0.01);
                }
                let jname = self.host.model().rig.joints[idx].name.clone();
                let payload = self
                    .single_joint_json(&jname)
                    .unwrap_or_else(|_| json!({"index": idx}));
                self.ok_json(name, jname, payload)
            }
            "calibration_zero_here" => {
                let idx = match args.get("joint").ok_or_else(|| "missing `joint`".to_string()).and_then(|v| self.resolve_joint(v)) {
                    Ok(i) => i,
                    Err(err) => return self.err_result(name, "invalid joint", err),
                };
                self.host.model_mut().set_current_as_zero(idx);
                if self.host.is_connected() {
                    self.host.step_ticks(2, 0.01);
                }
                let jname = self.host.model().rig.joints[idx].name.clone();
                let payload = self
                    .single_joint_json(&jname)
                    .unwrap_or_else(|_| json!({"index": idx}));
                self.ok_json(name, format!("zero {jname}"), payload)
            }
            "calibration_export_json" => match self.host.model().export_calibration_json() {
                Ok(json_str) => {
                    let mut written = None;
                    if let Some(path) = args.get("file_path").and_then(Value::as_str) {
                        match self.host.write_confined_file(path, json_str.as_bytes()) {
                            Ok(dest) => written = Some(dest),
                            Err(err) => return self.err_result(name, path, err),
                        }
                    }
                    self.ok_json(
                        name,
                        "exported",
                        json!({"calibration_json": json_str, "written_path": written}),
                    )
                }
                Err(err) => self.err_result(name, "export", err),
            },
            "calibration_import_json" => {
                let json_str = args.get("json").and_then(Value::as_str).unwrap_or("");
                match self.host.model_mut().import_calibration_json(json_str) {
                    Ok(()) => {
                        if self.host.is_connected() {
                            self.host.step_ticks(4, 0.01);
                        }
                        self.ok_json(name, "imported", self.calibration_json())
                    }
                    Err(err) => self.err_result(name, "import", err),
                }
            }
            "balance_get_gains" => self.ok_json(name, "read", self.balance_json()),
            "balance_set_gains" => {
                let mut g = self.host.model().balance_gains;
                let updates: [(&str, f32, f32, &mut f32); 14] = [
                    ("uvc_gain_pitch", 0.0, 1.5, &mut g.uvc_gain_pitch),
                    ("uvc_gain_roll", 0.0, 1.5, &mut g.uvc_gain_roll),
                    ("tilt_deadband_rad", 0.005, 0.15, &mut g.tilt_deadband_rad),
                    ("gyro_damp_pitch", 0.0, 1.0, &mut g.gyro_damp_pitch),
                    ("gyro_damp_roll", 0.0, 1.0, &mut g.gyro_damp_roll),
                    ("arm_assist", 0.0, 1.5, &mut g.arm_assist),
                    ("fall_cutoff_rad", 0.30, 1.35, &mut g.fall_cutoff_rad),
                    ("dcm_step_gain", 0.0, 1.5, &mut g.dcm_step_gain),
                    ("foot_half_length_m", 0.015, 0.12, &mut g.foot_half_length_m),
                    ("foot_margin_m", 0.010, 0.12, &mut g.foot_margin_m),
                    ("h_com_m", 0.12, 0.80, &mut g.h_com_m),
                    ("ukemi_tilt_rad", 0.25, 1.20, &mut g.ukemi_tilt_rad),
                    ("ukemi_torque_scale", 0.05, 0.50, &mut g.ukemi_torque_scale),
                    ("compliance_scale", 0.10, 1.0, &mut g.compliance_scale),
                ];
                for (key, lo, hi, field) in updates {
                    match Self::finite_f32(args, key, lo, hi) {
                        Ok(Some(v)) => *field = v,
                        Ok(None) => {}
                        Err(e) => {
                            let raw = args.get(key).cloned().unwrap_or(Value::Null);
                            return self.invalid_gain_result(name, key, raw, lo, hi, e);
                        }
                    }
                }
                self.host.model_mut().update_balance_gains(g);
                if self.host.is_connected() {
                    self.host.step_ticks(2, 0.01);
                }
                self.ok_json(
                    name,
                    format!("uvc=({:.2},{:.2})", g.uvc_gain_pitch, g.uvc_gain_roll),
                    self.balance_json(),
                )
            }
            "balance_reset_defaults" => {
                let defaults = BalanceGains::default();
                self.host.model_mut().update_balance_gains(defaults);
                if self.host.is_connected() {
                    self.host.step_ticks(2, 0.01);
                }
                self.ok_json(name, "balance gains reset to defaults", self.balance_json())
            }
            "balance_set_enabled" => {
                let enabled = args.get("enabled").and_then(Value::as_bool).unwrap_or(true);
                self.host.model_mut().walk_cmd.balance_enabled = enabled;
                if self.host.is_connected() {
                    self.host.step_ticks(2, 0.01);
                }
                self.ok_json(
                    name,
                    if enabled { "balance ON" } else { "balance OFF" },
                    self.balance_json(),
                )
            }
            "gait_configure" => {
                let mut w = self.host.model().walk_cmd;
                match Self::finite_f32(args, "step_height", 0.1, 1.0) {
                    Ok(Some(v)) => w.step_height = v,
                    Ok(None) => {}
                    Err(e) => return self.err_result(name, "step_height", e),
                }
                match Self::finite_f32(args, "cadence_hz", 0.6, 2.8) {
                    Ok(Some(v)) => w.cadence_hz = v,
                    Ok(None) => {}
                    Err(e) => return self.err_result(name, "cadence_hz", e),
                }
                match Self::finite_f32(args, "stance_crouch", 0.0, 1.0) {
                    Ok(Some(v)) => w.stance_crouch = v,
                    Ok(None) => {}
                    Err(e) => return self.err_result(name, "stance_crouch", e),
                }
                self.host.model_mut().walk_cmd = w;
                self.ok_json(name, format!("cadence={:.2}Hz", w.cadence_hz), self.balance_json())
            }
            "mode_set" => {
                let mode_str = args.get("mode").and_then(Value::as_str).unwrap_or("teach");
                if self.host.model().is_estop_active() {
                    return self.guarded_result(
                        name,
                        mode_str,
                        "E-Stop is latched. Call safety_reset_estop to return to Teach mode first.",
                    );
                }
                match mode_str {
                    "teach" => {
                        self.host.model_mut().set_authority_mode(ControlMode::Teach);
                        self.host.set_walk_override(0.0, 0.0, 0.0, false);
                        if self.host.is_connected() {
                            self.host.step_ticks(2, 0.01);
                        }
                        self.ok_json(name, "Teach", self.status_json())
                    }
                    "drive" => {
                        if !self.host.policy().drive_armed {
                            return self.guarded_result(
                                name,
                                "Drive blocked",
                                "Agent Drive is disarmed. Operator must enable 'Allow Agent Drive' in Rocky Studio (or pass --allow-drive on CLI) before entering Drive mode.",
                            );
                        }
                        if !self.host.is_connected() {
                            return self.err_result(
                                name,
                                "not connected",
                                "Connect to a robot via robot_connect before entering Drive mode.",
                            );
                        }
                        self.host.model_mut().set_authority_mode(ControlMode::Drive);
                        self.host.step_ticks(5, 0.01);
                        self.ok_json(name, "Drive", self.status_json())
                    }
                    other => self.err_result(name, other, format!("unknown mode `{other}`")),
                }
            }
            "speed_limit_set" => {
                let Some(pct) = args.get("speed_limit_pct").and_then(Value::as_u64) else {
                    return self.err_result(name, "missing", "speed_limit_pct must be 10..=100");
                };
                if !(10..=100).contains(&pct) {
                    return self.err_result(name, format!("{pct}%"), "speed_limit_pct must be between 10 and 100");
                }
                self.host.model_mut().speed_limit_pct = pct as u8;
                self.ok_json(name, format!("{pct}%"), json!({"speed_limit_pct": pct}))
            }
            "safety_estop" => {
                self.host.trigger_estop_and_disarm();
                if self.host.is_connected() {
                    self.host.step_ticks(2, 0.01);
                }
                self.ok_json(
                    name,
                    "E-STOP latched & Drive disarmed",
                    self.status_json(),
                )
            }
            "safety_reset_estop" => {
                self.host.reset_estop_upright();
                if self.host.is_connected() {
                    self.host.step_ticks(3, 0.01);
                }
                self.ok_json(name, "Reset -> Teach", self.status_json())
            }
            "joint_set_target" => {
                if self.host.model().is_estop_active() {
                    return self.guarded_result(name, "E-Stop active", "Cannot move joints while E-Stop is latched.");
                }
                if self.host.model().requested_mode == ControlMode::Drive
                    && !self.host.policy().drive_armed
                {
                    return self.guarded_result(
                        name,
                        "Drive disarmed",
                        "Agent Drive is disarmed. Enable 'Allow Agent Drive' to command joints in Drive mode.",
                    );
                }
                let idx = match args.get("joint").ok_or_else(|| "missing `joint`".to_string()).and_then(|v| self.resolve_joint(v)) {
                    Ok(i) => i,
                    Err(err) => return self.err_result(name, "invalid joint", err),
                };
                let rad = match (
                    Self::finite_f32(args, "angle_rad", -6.2832, 6.2832),
                    Self::finite_f32(args, "angle_deg", -360.0, 360.0),
                ) {
                    (Ok(Some(r)), _) => r,
                    (Ok(None), Ok(Some(d))) => d.to_radians(),
                    (Err(e), _) | (_, Err(e)) => return self.err_result(name, "angle", e),
                    (Ok(None), Ok(None)) => {
                        return self.err_result(name, "missing angle", "provide either `angle_rad` or `angle_deg`");
                    }
                };
                if let Err(err) = self.host.apply_joint_angle(idx, rad) {
                    return self.err_result(name, "apply", err);
                }
                if self.host.is_connected() {
                    self.host.step_ticks(4, 0.01);
                }
                let jname = self.host.model().rig.joints[idx].name.clone();
                let payload = self
                    .single_joint_json(&jname)
                    .unwrap_or_else(|_| json!({"index": idx}));
                self.ok_json(name, format!("{jname}={:.2}rad", rad), payload)
            }
            "joints_set_targets" => {
                if self.host.model().is_estop_active() {
                    return self.guarded_result(name, "E-Stop active", "Cannot move joints while E-Stop is latched.");
                }
                if self.host.model().requested_mode == ControlMode::Drive
                    && !self.host.policy().drive_armed
                {
                    return self.guarded_result(
                        name,
                        "Drive disarmed",
                        "Agent Drive is disarmed. Enable 'Allow Agent Drive' to command joints in Drive mode.",
                    );
                }
                let deg_mode = args.get("units").and_then(Value::as_str) == Some("deg");
                let Some(map) = args.get("targets").and_then(Value::as_object) else {
                    return self.err_result(name, "targets", "`targets` must be a JSON object");
                };
                let mut applied = 0usize;
                for (k, v) in map {
                    let idx = match self.resolve_joint(&Value::String(k.clone())) {
                        Ok(i) => i,
                        Err(e) => return self.err_result(name, k.as_str(), e),
                    };
                    let Some(num_val) = v.as_f64() else {
                        return self.err_result(name, k.as_str(), format!("target for `{k}` must be a number"));
                    };
                    let raw = num_val as f32;
                    if !raw.is_finite() {
                        return self.err_result(name, k.as_str(), format!("target for `{k}` must be finite"));
                    }
                    let rad = if deg_mode { raw.to_radians() } else { raw };
                    if let Err(e) = self.host.apply_joint_angle(idx, rad) {
                        return self.err_result(name, k.as_str(), e);
                    }
                    applied += 1;
                }
                if self.host.is_connected() {
                    self.host.step_ticks(4, 0.01);
                }
                self.ok_json(
                    name,
                    format!("{applied} joints"),
                    json!({"applied_count": applied, "telemetry": self.telemetry_json()}),
                )
            }
            "limb_solve_ik" => {
                if self.host.model().is_estop_active() {
                    return self.guarded_result(name, "E-Stop active", "Cannot solve limb IK while E-Stop is latched.");
                }
                if self.host.model().requested_mode == ControlMode::Drive
                    && !self.host.policy().drive_armed
                {
                    return self.guarded_result(
                        name,
                        "Drive disarmed",
                        "Agent Drive is disarmed. Enable 'Allow Agent Drive' before commanding IK in Drive mode.",
                    );
                }
                let link_name = args.get("link_name").and_then(Value::as_str).unwrap_or("");
                let x = match Self::finite_f32(args, "x", -3.0, 3.0) {
                    Ok(Some(v)) => v,
                    Ok(None) => return self.err_result(name, "x", "missing x"),
                    Err(e) => return self.err_result(name, "x", e),
                };
                let y = match Self::finite_f32(args, "y", -1.0, 3.0) {
                    Ok(Some(v)) => v,
                    Ok(None) => return self.err_result(name, "y", "missing y"),
                    Err(e) => return self.err_result(name, "y", e),
                };
                let z = match Self::finite_f32(args, "z", -3.0, 3.0) {
                    Ok(Some(v)) => v,
                    Ok(None) => return self.err_result(name, "z", "missing z"),
                    Err(e) => return self.err_result(name, "z", e),
                };
                let solved = {
                    let m = self.host.model();
                    let (p, r) = if m.latest_telemetry.imu.available {
                        (m.latest_telemetry.imu.pitch_rad, m.latest_telemetry.imu.roll_rad)
                    } else {
                        (0.0, 0.0)
                    };
                    m.rig.solve_limb_ik(
                        link_name,
                        &m.target_rad,
                        Point3::new(x, y, z),
                        p,
                        r,
                        &m.calibration,
                    )
                };
                let Some(pose) = solved else {
                    return self.err_result(
                        name,
                        link_name,
                        format!("link `{link_name}` is not a valid IK end-effector on this robot"),
                    );
                };
                let dof = self.host.model().rig.joints.len().min(rk_proto::MAX_JOINTS);
                for i in 0..dof {
                    let _ = self.host.apply_joint_angle(i, pose[i]);
                }
                if self.host.is_connected() {
                    self.host.step_ticks(4, 0.01);
                }
                self.ok_json(
                    name,
                    format!("{link_name} -> ({x:.2},{y:.2},{z:.2})"),
                    json!({"link_name": link_name, "target": [x, y, z], "telemetry": self.telemetry_json()}),
                )
            }
            "walk_set_velocity" => {
                let vx = match Self::finite_f32(args, "vx", -1.0, 1.0) {
                    Ok(v) => v.unwrap_or(self.host.model().walk_cmd.vx),
                    Err(e) => return self.err_result(name, "vx", e),
                };
                let vy = match Self::finite_f32(args, "vy", -1.0, 1.0) {
                    Ok(v) => v.unwrap_or(self.host.model().walk_cmd.vy),
                    Err(e) => return self.err_result(name, "vy", e),
                };
                let yaw = match Self::finite_f32(args, "yaw_rate", -1.0, 1.0) {
                    Ok(v) => v.unwrap_or(self.host.model().walk_cmd.yaw_rate),
                    Err(e) => return self.err_result(name, "yaw_rate", e),
                };
                let walk_enabled = args
                    .get("walk_enabled")
                    .and_then(Value::as_bool)
                    .unwrap_or(vx.abs() > 0.04 || vy.abs() > 0.04 || yaw.abs() > 0.04);

                if walk_enabled {
                    if self.host.model().is_estop_active() {
                        return self.guarded_result(name, "E-Stop active", "Cannot walk while E-Stop is latched.");
                    }
                    if !self.host.policy().drive_armed {
                        return self.guarded_result(
                            name,
                            "Drive disarmed",
                            "Agent Drive is disarmed. Operator must enable 'Allow Agent Drive' before commanding walking velocity.",
                        );
                    }
                    if self.host.model().requested_mode != ControlMode::Drive {
                        return self.guarded_result(
                            name,
                            "not in Drive",
                            "Switch to Drive mode via mode_set(mode='drive') before commanding walking.",
                        );
                    }
                }

                self.host.set_walk_override(vx, vy, yaw, walk_enabled);
                let ticks = args
                    .get("step_ticks")
                    .and_then(Value::as_u64)
                    .unwrap_or(0)
                    .min(MAX_STEP_TICKS as u64) as usize;
                if ticks > 0 && self.host.is_connected() {
                    self.host.step_ticks(ticks, 0.01);
                }
                self.ok_json(
                    name,
                    format!("vx={vx:+.2} vy={vy:+.2} yaw={yaw:+.2}"),
                    json!({"balance": self.balance_json(), "telemetry": self.telemetry_json()}),
                )
            }
            "action_play" => {
                let req_name = args.get("name").and_then(Value::as_str).unwrap_or("");
                if self.host.model().is_estop_active() {
                    return self.guarded_result(name, req_name, "Cannot play actions while E-Stop is latched.");
                }
                if !self.host.policy().drive_armed {
                    return self.guarded_result(
                        name,
                        req_name,
                        "Agent Drive is disarmed. Enable 'Allow Agent Drive' before playing actions.",
                    );
                }
                if self.host.model().requested_mode != ControlMode::Drive {
                    return self.guarded_result(
                        name,
                        req_name,
                        "Switch to Drive mode via mode_set(mode='drive') before playing actions.",
                    );
                }
                let found = self
                    .host
                    .model()
                    .moves
                    .iter()
                    .position(|mv| mv.name.eq_ignore_ascii_case(req_name));
                let Some(idx) = found else {
                    let avail: Vec<&str> = self
                        .host
                        .model()
                        .moves
                        .iter()
                        .map(|mv| mv.name.as_str())
                        .collect();
                    return self.err_result(
                        name,
                        req_name,
                        format!("unknown action `{req_name}`; available: {}", avail.join(", ")),
                    );
                };
                self.host.model_mut().play_move(idx);
                let ticks = args
                    .get("step_ticks")
                    .and_then(Value::as_u64)
                    .unwrap_or(0)
                    .min(MAX_STEP_TICKS as u64) as usize;
                if ticks > 0 && self.host.is_connected() {
                    self.host.step_ticks(ticks, 0.01);
                }
                self.ok_json(name, req_name, self.actions_json())
            }
            "action_stop" => {
                self.host.model_mut().stop_playback();
                self.ok_json(name, "stopped", self.actions_json())
            }
            "sim_push_shove" => {
                let pitch = match Self::finite_f32(args, "pitch_impulse_rad_s", -5.0, 5.0) {
                    Ok(Some(v)) => v,
                    Ok(None) => return self.err_result(name, "pitch", "missing pitch_impulse_rad_s"),
                    Err(e) => return self.err_result(name, "pitch", e),
                };
                let roll = match Self::finite_f32(args, "roll_impulse_rad_s", -5.0, 5.0) {
                    Ok(v) => v.unwrap_or(0.0),
                    Err(e) => return self.err_result(name, "roll", e),
                };
                self.host.transport_mut().apply_push_impulse(pitch, roll);
                let ticks = args
                    .get("step_ticks")
                    .and_then(Value::as_u64)
                    .unwrap_or(0)
                    .min(MAX_STEP_TICKS as u64) as usize;
                let mut peak_pitch_rad = 0.0_f32;
                let mut peak_roll_rad = 0.0_f32;
                let mut saw_catch_step = false;
                if ticks > 0 && self.host.is_connected() {
                    // Keep each fixed-step run under the 100 ms deadman window;
                    // one packet is repeated for every tick in a run.
                    const SHOVE_BATCH_TICKS: usize = 19;
                    let mut remaining = ticks;
                    while remaining > 0 {
                        let batch = remaining.min(SHOVE_BATCH_TICKS);
                        self.host.step_ticks(batch, 0.01);
                        let t = &self.host.model().latest_telemetry;
                        peak_pitch_rad = peak_pitch_rad.max(t.imu.pitch_rad.abs());
                        peak_roll_rad = peak_roll_rad.max(t.imu.roll_rad.abs());
                        saw_catch_step |= t.balance.uvc_phase == uvc_phase::ACTIVE_CATCH_STEP;
                        remaining -= batch;
                    }
                }
                self.ok_json(
                    name,
                    format!("shove({pitch:+.2},{roll:+.2})"),
                    json!({
                        "pitch_impulse_rad_s": pitch,
                        "roll_impulse_rad_s": roll,
                        "stepped_ticks": ticks,
                        "peak_pitch_deg": peak_pitch_rad.to_degrees(),
                        "peak_roll_deg": peak_roll_rad.to_degrees(),
                        "saw_uvc_catch_step": saw_catch_step,
                        "telemetry": self.telemetry_json(),
                    }),
                )
            }
            "sim_inject_fault" => {
                let stall_idx = match args.get("stall_joint") {
                    Some(v) if !v.is_null() => match self.resolve_joint(v) {
                        Ok(i) => Some(i),
                        Err(e) => return self.err_result(name, "stall_joint", e),
                    },
                    _ => None,
                };
                // One-shot wire corruption: mirror the native UI button, which
                // puzzles a malformed frame into the COBS stream once per click.
                let corrupt_frame_injected =
                    args.get("corrupt_frame").and_then(Value::as_bool) == Some(true);
                if corrupt_frame_injected {
                    self.host.transport_mut().inject_corrupted_frame();
                }
                let lag = match Self::finite_f32(args, "servo_lag_factor", 0.03, 0.35) {
                    Ok(v) => v,
                    Err(e) => return self.err_result(name, "servo_lag_factor", e),
                };
                let Some(faults) = self.host.transport_mut().sim_faults_mut() else {
                    return self.err_result(name, "sim only", "Fault injection is only supported on the Simulation transport.");
                };
                // Every field of `SimFaultInjection` is settable here with the same
                // semantics as the native Sim drawer, so MCP and GUI cannot drift.
                if let Some(v) = args.get("drop_heartbeat").and_then(Value::as_bool) {
                    faults.drop_heartbeat = v;
                }
                if let Some(v) = args.get("voltage_sag").and_then(Value::as_bool) {
                    faults.voltage_sag = v;
                }
                if let Some(v) = args.get("servo_bus_error").and_then(Value::as_bool) {
                    faults.servo_bus_error = v;
                }
                if let Some(v) = args.get("external_hand_wave").and_then(Value::as_bool) {
                    faults.external_hand_wave = v;
                }
                if let Some(v) = args.get("imu_disconnected").and_then(Value::as_bool) {
                    faults.imu_disconnected = v;
                }
                if let Some(v) = args.get("rx_crc_error").and_then(Value::as_bool) {
                    faults.rx_crc_error = v;
                }
                if let Some(v) = args.get("rx_version_mismatch").and_then(Value::as_bool) {
                    faults.rx_version_mismatch = v;
                }
                if let Some(v) = args.get("rx_truncated_frame").and_then(Value::as_bool) {
                    faults.rx_truncated_frame = v;
                }
                if let Some(v) = args.get("rx_fragmented_frame").and_then(Value::as_bool) {
                    faults.rx_fragmented_frame = v;
                }
                if let Some(v) = lag {
                    faults.servo_lag_factor = v;
                }
                if let Some(idx) = stall_idx {
                    let stalled = args.get("stalled").and_then(Value::as_bool).unwrap_or(true);
                    if idx < faults.stalled_joints.len() {
                        faults.stalled_joints[idx] = stalled;
                    }
                }
                let stalled: Vec<usize> = faults
                    .stalled_joints
                    .iter()
                    .enumerate()
                    .filter_map(|(i, s)| s.then_some(i))
                    .collect();
                let summary = json!({
                    "drop_heartbeat": faults.drop_heartbeat,
                    "voltage_sag": faults.voltage_sag,
                    "servo_bus_error": faults.servo_bus_error,
                    "external_hand_wave": faults.external_hand_wave,
                    "imu_disconnected": faults.imu_disconnected,
                    "servo_lag_factor": faults.servo_lag_factor,
                    "stalled_joints": stalled,
                    "corrupt_frame_injected": corrupt_frame_injected,
                    "rx_crc_error": faults.rx_crc_error,
                    "rx_version_mismatch": faults.rx_version_mismatch,
                    "rx_truncated_frame": faults.rx_truncated_frame,
                    "rx_fragmented_frame": faults.rx_fragmented_frame,
                });
                self.ok_json(name, "faults updated", summary)
            }
            "reset_experiment" => {
                match self.host.reset_experiment() {
                    Ok(msg) => self.ok_json(name, "experiment reset", json!({"reset": true, "message": msg})),
                    Err(e) => self.err_result(name, "reset unavailable", e),
                }
            }
            "run_experiment" => {
                // Accept an inline script object/JSON string or a confined file
                // so CI can commit a script and point the CLI at it.
                let script_value: Value = if let Some(raw) = args.get("script") {
                    match raw {
                        Value::Object(_) => raw.clone(),
                        Value::String(text) => match serde_json::from_str::<Value>(text) {
                            Ok(parsed) => parsed,
                            Err(e) => {
                                return self.err_result(name, "invalid script JSON", format!("`script` is not valid JSON: {e}"));
                            }
                        },
                        _ => {
                            return self.err_result(name, "invalid script", "`script` must be an ExperimentScript JSON object or a JSON string");
                        }
                    }
                } else if let Some(path) = args.get("file_path").and_then(Value::as_str) {
                    let bytes = match self.host.read_confined_file(path) {
                        Ok(bytes) => bytes,
                        Err(e) => return self.err_result(name, "script read failed", e),
                    };
                    match serde_json::from_slice::<Value>(&bytes) {
                        Ok(parsed) => parsed,
                        Err(e) => {
                            return self.err_result(name, "invalid script file", format!("`file_path` did not contain valid JSON: {e}"));
                        }
                    }
                } else {
                    return self.err_result(name, "missing script", "Provide `script` (ExperimentScript JSON) or `file_path` (confined JSON file).");
                };

                let script: ExperimentScript = match serde_json::from_value(script_value) {
                    Ok(script) => script,
                    Err(e) => {
                        return self.err_result(name, "invalid script", format!("`script` is not a valid ExperimentScript: {e}"));
                    }
                };
                self.run_experiment_script(name, script)
            }
            "experiment_list" => {
                let experiments: Vec<Value> = EXPERIMENT_FIXTURES
                    .iter()
                    .map(|(name, description)| json!({"name": name, "description": description}))
                    .collect();
                self.ok_json(name, "committed experiments", json!({"experiments": experiments}))
            }
            "experiment_run_named" => {
                let requested = args.get("name").and_then(Value::as_str).unwrap_or("");
                let Some(fixture) = resolve_experiment_name(requested) else {
                    let available: Vec<&str> = EXPERIMENT_FIXTURES.iter().map(|(n, _)| *n).collect();
                    return self.err_result(
                        name,
                        "unknown experiment",
                        format!("unknown experiment `{requested}`; available: {}", available.join(", ")),
                    );
                };
                // Native CLI: honour a file under the confined read root when
                // available. Otherwise use the bytes compiled from that exact
                // committed fixture (GUI bridge has no filesystem access).
                let script_value: Value = if let Ok(bytes) = self.host.read_confined_file(fixture) {
                    match serde_json::from_slice(&bytes) {
                        Ok(parsed) => parsed,
                        Err(e) => {
                            return self.err_result(name, "invalid experiment file", format!("`{fixture}` is not valid JSON: {e}"));
                        }
                    }
                } else if let Some(text) = embedded_experiment(fixture) {
                    match serde_json::from_str(text) {
                        Ok(parsed) => parsed,
                        Err(e) => {
                            return self.err_result(name, "invalid experiment fixture", format!("committed `{fixture}` is not valid JSON: {e}"));
                        }
                    }
                } else {
                    return self.err_result(name, "experiment unavailable", format!("no embedded content for `{fixture}`"));
                };
                let script: ExperimentScript = match serde_json::from_value(script_value) {
                    Ok(script) => script,
                    Err(e) => {
                        return self.err_result(name, "invalid experiment", format!("`{fixture}` is not a valid ExperimentScript: {e}"));
                    }
                };
                self.run_experiment_script(name, script)
            }
            "recording_control" => {
                let action = args.get("action").and_then(Value::as_str).unwrap_or("");
                match action {
                    "start" => {
                        // Replay always starts a fresh ControlTask. Reset first so the
                        // recording begins at kernel index zero (mode transitions such
                        // as Teach -> Drive otherwise leave the live kernel in a state
                        // a fresh task cannot reproduce).
                        if self.host.transport().kind() == TransportKind::SimKernel {
                            if let Err(err) = self.host.reset_experiment() {
                                return self.err_result(name, "reset failed", err);
                            }
                        }
                        self.host.transport_mut().start_recording();
                        self.ok_json(name, "recording started", json!({"recording": true}))
                    }
                    "stop" => {
                        let log = self.host.transport_mut().stop_recording();
                        let count = log.as_ref().map(|l| l.ticks.len()).unwrap_or(0);
                        self.ok_json(
                            name,
                            format!("stopped ({count} ticks)"),
                            json!({"recording": false, "recorded_ticks": count}),
                        )
                    }
                    "verify_replay" => {
                        let log_opt = self.host.transport().recorded_log().cloned();
                        let Some(log) = log_opt else {
                            return self.err_result(
                                name,
                                "no log",
                                "No Copper log recorded yet. Call recording_control(action='start'), run ticks, then verify_replay.",
                            );
                        };
                        let check = verify_deterministic_replay(&log);
                        self.ok_json(
                            name,
                            if check.exact_match { "replay EXACT" } else { "replay MISMATCH" },
                            json!({
                                "exact_match": check.exact_match,
                                "matched_ticks": check.matched_ticks,
                                "total_ticks": check.total_ticks,
                            }),
                        )
                    }
                    other => self.err_result(name, other, format!("unknown recording action `{other}`")),
                }
            }
            _ => self.err_result(name, "unknown", format!("unknown tool `{name}`")),
        }
    }
}

/// Shared helper to query MCU onboard model info over any `RobotTransport`.
pub fn query_mcu_model_summary(transport: &mut dyn rk_link::RobotTransport) -> Result<String, String> {
    match transport.exchange_model(ModelRequest::GetInfo) {
        Some(ModelResponse::Info(info)) => Ok(format!(
            "MCU model '{}' ({} DoF, {} bytes, crc32={:#010x})",
            info.name_str(),
            info.joint_count,
            info.total_bytes,
            info.crc32
        )),
        Some(ModelResponse::NotAvailable) => {
            Err("Connected MCU does not have an onboard URDF model stored.".to_string())
        }
        _ => Err("MCU did not respond to ModelRequest::GetInfo.".to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::headless::{AutomationRoots, HeadlessHost};
    use crate::host::McpPolicy;
    use crate::test_support::TEST_LOCK;

    #[test]
    fn quality_snapshot_reports_all_walk_quality_metrics() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        let init = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}
            }),
        )
        .expect("connect reply");
        assert!(init["result"]["isError"].is_null(), "{init}");
        host.step_ticks(20, 0.01);

        let reply = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": "quality_snapshot", "arguments": {}}
            }),
        )
        .expect("quality reply");
        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let body: Value =
            serde_json::from_str(reply["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        let quality = &body["quality"];
        assert!(!quality.is_null(), "sim transport must report quality: {body}");
        for key in [
            "mean_forward_speed_m_s",
            "peak_tilt_rad",
            "steady_tilt_rad",
            "com_height_min_m",
            "com_height_max_m",
            "lateral_drift_m",
            "max_foot_slip_m",
            "single_support_grf_ratio",
            "flight_ticks",
            "double_support_ticks",
            "max_current_ma",
            "margin_negative_fraction",
            "catch_step_fraction",
            "contact_swap_count",
        ] {
            assert!(quality[key].is_number(), "missing metric `{key}`: {body}");
            assert!(
                body[key].is_number(),
                "metric `{key}` must also be available at the top level: {body}"
            );
        }

        let resource = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 3, "method": "resources/read",
                "params": {"uri": "rocky://quality"}
            }),
        )
        .expect("quality resource reply");
        let text = resource["result"]["contents"][0]["text"]
            .as_str()
            .expect("quality resource text");
        let from_resource: Value = serde_json::from_str(text).unwrap();
        assert_eq!(&from_resource["quality"], quality, "{from_resource}");

        // Acceptance envelope comes from the same shared evaluator as the
        // simulator tests and is mirrored by the resource.
        for envelope in [&body, &from_resource] {
            assert!(envelope["bounds"].is_object(), "missing bounds: {envelope}");
            assert!(envelope["bounds"]["command_forward_speed_m_s"].is_number());
            assert!(envelope["bounds"]["peak_tilt_rad_max"].is_number());
            assert!(envelope["bounds"]["min_contact_swap_count"].is_number());
            assert!(envelope["violations"].is_array(), "missing violations: {envelope}");
            assert!(envelope["pass"].is_boolean(), "missing verdict: {envelope}");
            assert_eq!(
                envelope["pass"],
                json!(envelope["violations"].as_array().unwrap().is_empty()),
                "verdict must match violations: {envelope}"
            );
        }
        assert_eq!(from_resource["bounds"], body["bounds"]);
        assert_eq!(from_resource["violations"], body["violations"]);
        assert_eq!(from_resource["pass"], body["pass"]);
    }

    #[test]
    fn mcp_lifecycle_resources_templates_prompts_and_completions_work() {
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());

        let init = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18"}}),
        )
        .expect("init reply");
        assert_eq!(init["result"]["serverInfo"]["name"], "rocky-studio");
        assert_eq!(init["result"]["protocolVersion"], "2025-06-18");

        let tools = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}),
        )
        .expect("tools reply");
        assert!(tools["result"]["tools"].as_array().unwrap().len() >= 25);

        let res = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 3, "method": "resources/read", "params": {"uri": "rocky://joints/l_hip_pitch"}}),
        )
        .expect("joint template read");
        assert!(res["result"]["contents"][0]["text"]
            .as_str()
            .unwrap()
            .contains("l_hip_pitch"));

        let comp = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0",
                "id": 4,
                "method": "completion/complete",
                "params": {
                    "ref": {"type": "ref/resource", "uri": "rocky://joints/{joint_name}"},
                    "argument": {"name": "joint_name", "value": "l_hip"}
                }
            }),
        )
        .expect("completion reply");
        let vals = comp["result"]["completion"]["values"].as_array().unwrap();
        assert!(vals.iter().any(|v| v == "l_hip_pitch"));
    }

    #[test]
    fn guarded_autonomy_blocks_unarmed_drive_and_estop_disarms_drive() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}}),
        );

        // Unarmed Drive request must be cleanly refused with isError: true
        let blocked = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "mode_set", "arguments": {"mode": "drive"}}}),
        )
        .expect("blocked reply");
        assert_eq!(blocked["result"]["isError"], true);
        assert_eq!(host.model().requested_mode, ControlMode::Teach);

        // Arm Drive -> Drive succeeds, shove catch-step recovers, and E-Stop immediately disarms Drive
        host.set_drive_armed(true);
        let ok_drive = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "mode_set", "arguments": {"mode": "drive"}}}),
        )
        .expect("drive reply");
        assert!(ok_drive["result"]["isError"].is_null());
        assert_eq!(host.model().requested_mode, ControlMode::Drive);

        // Warm up UVC ReadyWatch and inject shove
        for _ in 0..3 {
            host.step_ticks(10, 0.01);
        }
        let shove = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "sim_push_shove", "arguments": {"pitch_impulse_rad_s": 1.5, "roll_impulse_rad_s": -0.9, "step_ticks": 260}}}),
        )
        .expect("shove reply");
        let body: Value = serde_json::from_str(shove["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["saw_uvc_catch_step"], true);

        // E-Stop cuts torque & automatically disarms Agent Drive
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 5, "method": "tools/call", "params": {"name": "safety_estop", "arguments": {}}}),
        );
        assert!(host.model().is_estop_active());
        assert!(!host.policy().drive_armed);
    }

    #[test]
    fn mcp_replay_is_bit_exact_after_fixed_steps() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());

        let connect = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}}),
        )
        .expect("connect reply");
        assert!(connect["result"]["isError"].is_null());

        let start = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "start"}}}),
        )
        .expect("recording start reply");
        assert!(start["result"]["isError"].is_null());

        host.step_ticks(80, 0.005);

        let stop = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "stop"}}}),
        )
        .expect("recording stop reply");
        let stop_body: Value = serde_json::from_str(stop["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(stop_body["recorded_ticks"], 80);

        let verify = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "verify_replay"}}}),
        )
        .expect("verify replay reply");
        let body: Value = serde_json::from_str(verify["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["exact_match"], true, "replay result: {body}");
        assert_eq!(body["matched_ticks"], body["total_ticks"]);
        assert_eq!(body["total_ticks"], 80);
    }

    /// Recording after a Teach -> Drive transition used to capture a kernel
    /// mid-stream while replay always starts a fresh ControlTask, producing
    /// 0/N exact matches. `recording_control(start)` now resets first.
    #[test]
    fn mcp_replay_is_exact_when_recording_starts_after_drive_transition() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        host.set_drive_armed(true);
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}}),
        )
        .expect("connect reply");
        let drive = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "mode_set", "arguments": {"mode": "drive"}}}),
        )
        .expect("mode reply");
        assert!(drive["result"]["isError"].is_null());

        let start = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "start"}}}),
        )
        .expect("recording start reply");
        assert!(start["result"]["isError"].is_null());

        let shove = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "sim_push_shove", "arguments": {"pitch_impulse_rad_s": 0.8, "roll_impulse_rad_s": -0.4, "step_ticks": 40}}}),
        )
        .expect("shove reply");
        assert!(shove["result"]["isError"].is_null());

        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 5, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "stop"}}}),
        )
        .expect("stop reply");
        let verify = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 6, "method": "tools/call", "params": {"name": "recording_control", "arguments": {"action": "verify_replay"}}}),
        )
        .expect("verify reply");
        let body: Value = serde_json::from_str(verify["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["exact_match"], true, "replay result: {body}");
        assert_eq!(body["matched_ticks"], 40);
    }

    /// The native Sim drawer writes the same `SimFaultInjection` fields directly;
    /// this asserts the MCP surface writes every one of them identically.
    #[test]
    fn mcp_fault_injection_matches_native_ui_faults() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}}),
        );

        // Mirrors the native UI (sim_tab.rs): mutate the shared fault struct.
        {
            let f = host.transport_mut().sim_faults_mut().expect("sim faults");
            *f = rk_sim::SimFaultInjection {
                drop_heartbeat: true,
                stalled_joints: {
                    let mut s = [false; rk_proto::MAX_JOINTS];
                    s[0] = true;
                    s[2] = true;
                    s
                },
                voltage_sag: true,
                servo_bus_error: true,
                external_hand_wave: true,
                imu_disconnected: true,
                servo_lag_factor: 0.12,
                rx_crc_error: true,
                rx_version_mismatch: true,
                rx_truncated_frame: true,
                rx_fragmented_frame: true,
                ..Default::default()
            };
        }
        let native = host.transport_mut().sim_faults().expect("sim faults").clone();

        // Reset to defaults, then recreate the exact same state through MCP.
        {
            let f = host.transport_mut().sim_faults_mut().expect("sim faults");
            *f = rk_sim::SimFaultInjection::default();
        }
        let reply = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": "sim_inject_fault", "arguments": {
                    "drop_heartbeat": true, "voltage_sag": true,
                    "servo_bus_error": true, "external_hand_wave": true,
                    "imu_disconnected": true, "servo_lag_factor": 0.12,
                    "rx_crc_error": true, "rx_version_mismatch": true,
                    "rx_truncated_frame": true, "rx_fragmented_frame": true
                }}
            }),
        )
        .expect("inject reply");
        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let mut last_reply = reply.clone();
        for (joint, stalled) in [(0, true), (2, true)] {
            last_reply = dispatch_json_rpc(
                &mut host,
                json!({
                    "jsonrpc": "2.0", "id": 3, "method": "tools/call",
                    "params": {"name": "sim_inject_fault", "arguments": {
                        "stall_joint": joint, "stalled": stalled
                    }}
                }),
            )
            .expect("stall reply");
        }

        let mcp = host.transport_mut().sim_faults().expect("sim faults").clone();
        assert_eq!(mcp.drop_heartbeat, native.drop_heartbeat);
        assert_eq!(mcp.stalled_joints, native.stalled_joints);
        assert_eq!(mcp.voltage_sag, native.voltage_sag);
        assert_eq!(mcp.servo_bus_error, native.servo_bus_error);
        assert_eq!(mcp.external_hand_wave, native.external_hand_wave);
        assert_eq!(mcp.imu_disconnected, native.imu_disconnected);
        assert_eq!(mcp.servo_lag_factor, native.servo_lag_factor);
        assert_eq!(mcp.rx_crc_error, native.rx_crc_error);
        assert_eq!(mcp.rx_version_mismatch, native.rx_version_mismatch);
        assert_eq!(mcp.rx_truncated_frame, native.rx_truncated_frame);
        assert_eq!(mcp.rx_fragmented_frame, native.rx_fragmented_frame);

        let body: Value = serde_json::from_str(reply["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["drop_heartbeat"], true);
        assert_eq!(body["voltage_sag"], true);
        assert_eq!(body["servo_bus_error"], true);
        assert_eq!(body["external_hand_wave"], true);
        assert_eq!(body["imu_disconnected"], true);
        assert!((body["servo_lag_factor"].as_f64().unwrap() - f64::from(0.12_f32)).abs() < 1.0e-6);
        let last_body: Value = serde_json::from_str(last_reply["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(last_body["stalled_joints"], json!([0, 2]));
        assert_eq!(body["rx_crc_error"], true);
        assert_eq!(body["rx_version_mismatch"], true);
        assert_eq!(body["rx_truncated_frame"], true);
        assert_eq!(body["rx_fragmented_frame"], true);
        assert_eq!(body["corrupt_frame_injected"], false);
    }

    /// The reset tool must clear injected faults and keep the tool surface usable.
    #[test]
    fn reset_experiment_tool_clears_injected_faults() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "robot_connect", "arguments": {"transport": "sim"}}}),
        );
        let _ = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "sim_inject_fault", "arguments": {"imu_disconnected": true, "rx_crc_error": true}}}),
        );
        assert!(host.transport().sim_faults().unwrap().imu_disconnected);

        let reply = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "reset_experiment", "arguments": {}}}),
        )
        .expect("reset reply");
        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let faults = host.transport().sim_faults().expect("sim faults");
        assert!(!faults.imu_disconnected);
        assert!(!faults.rx_crc_error);
    }

    #[test]
    fn run_experiment_tool_replays_shove_and_fault_toggle_exactly() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());

        let mut faults = rk_fw_core::replay::SimFaultSnapshot::default();
        faults.voltage_sag = true;
        let script = ExperimentScript {
            seed: 0xDEAD_BEEF,
            events: vec![
                rk_fw_core::replay::TimedSimEvent {
                    tick_index: 40,
                    kind: rk_fw_core::replay::SimEventKind::Shove {
                        pitch_rad_s: 1.6,
                        roll_rad_s: -0.7,
                    },
                },
                rk_fw_core::replay::TimedSimEvent {
                    tick_index: 80,
                    kind: rk_fw_core::replay::SimEventKind::SetFaults(faults),
                },
            ],
            packets: (0..100u32)
                .map(|seq| rk_proto::HostToMcu {
                    seq,
                    mode: ControlMode::Drive,
                    torque_mask: 0b11,
                    ..Default::default()
                })
                .collect(),
            max_ticks: 100,
        };

        let reply = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": "run_experiment", "arguments": {"script": serde_json::to_value(&script).unwrap()}}
            }),
        )
        .expect("run_experiment reply");
        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let body: Value =
            serde_json::from_str(reply["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["exact_match"], true, "{body}");
        assert_eq!(body["total_ticks"], 100, "{body}");
        assert_eq!(body["matched_ticks"], body["total_ticks"], "{body}");
        assert_eq!(body["verification"]["exact_match"], true, "{body}");
        assert_eq!(body["verification"]["total_ticks"], 100, "{body}");
        assert!(!body["quality"].is_null(), "{body}");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn run_experiment_tool_loads_script_from_confined_file() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let base = std::env::temp_dir().join("rk_mcp_experiment_script_test");
        let _ = std::fs::create_dir_all(&base);
        let script = ExperimentScript {
            seed: 7,
            events: Vec::new(),
            packets: (0..12u32)
                .map(|seq| rk_proto::HostToMcu {
                    seq,
                    ..Default::default()
                })
                .collect(),
            max_ticks: 12,
        };
        let path = base.join("script.json");
        std::fs::write(&path, serde_json::to_vec(&script).unwrap()).unwrap();

        let mut host = HeadlessHost::new(
            McpPolicy::default(),
            AutomationRoots {
                read_root: Some(base.clone()),
                write_root: None,
            },
        );
        let reply = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": "run_experiment", "arguments": {"file_path": "script.json"}}
            }),
        )
        .expect("run_experiment reply");
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir_all(&base);

        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let body: Value =
            serde_json::from_str(reply["result"]["content"][0]["text"].as_str().unwrap()).unwrap();
        assert_eq!(body["exact_match"], true, "{body}");
        assert_eq!(body["total_ticks"], 12, "{body}");
        assert_eq!(body["verification"]["exact_match"], true, "{body}");
        assert!(!body["quality"].is_null(), "{body}");
    }

    #[test]
    fn run_experiment_tool_rejects_empty_and_invalid_scripts() {
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        for (id, args) in [
            (2, json!({})),
            (3, json!({"script": {}})),
            (4, json!({"script": {"max_ticks": 5}})),
            (5, json!({"script": {"max_ticks": "oops", "packets": [{}]}})),
            (6, json!({"script": "not json"})),
        ] {
            let reply = dispatch_json_rpc(
                &mut host,
                json!({
                    "jsonrpc": "2.0", "id": id, "method": "tools/call",
                    "params": {"name": "run_experiment", "arguments": args}
                }),
            )
            .expect("run_experiment reply");
            assert_eq!(reply["result"]["isError"], true, "{reply}");
            let text = reply["result"]["content"][0]["text"].as_str().unwrap();
            assert!(!text.is_empty(), "error must carry a readable message");
        }
    }

    #[test]
    fn run_experiment_installs_tuned_gains_without_mutating_the_script() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut host = HeadlessHost::new(McpPolicy::default(), AutomationRoots::default());
        let reply = dispatch_json_rpc(
            &mut host,
            json!({
                "jsonrpc": "2.0", "id": 2, "method": "tools/call",
                "params": {"name": "balance_set_gains", "arguments": {
                    "uvc_gain_pitch": 1.2, "gyro_damp_pitch": 0.5
                }}
            }),
        )
        .expect("set gains reply");
        assert!(reply["result"]["isError"].is_null(), "{reply}");
        let tuned = host.model().balance_gains;
        assert_ne!(tuned.uvc_gain_pitch, rk_proto::BalanceGains::default().uvc_gain_pitch);

        let script = ExperimentScript {
            seed: 7,
            events: Vec::new(),
            packets: (0..160u32).map(rk_sim::scripted_walk_packet).collect(),
            max_ticks: 160,
        };
        let original = script.clone();
        let result = host.run_experiment(&script).expect("run_experiment");
        assert_eq!(script.packets, original.packets, "caller script must stay untouched");
        assert_eq!(script.events, original.events, "caller script must stay untouched");
        assert!(result.verification.exact_match, "{:?}", result.verification);
        let quality = result.quality.expect("sim experiment reports quality");

        fn run_with_gains(
            gains: rk_proto::BalanceGains,
            script: &ExperimentScript,
        ) -> rk_sim::WalkQuality {
            let model = rk_model::StudioModel::new_default();
            let mut runner = rk_sim::CopperSimRunner::new(
                model.rig.joints.len().max(1),
                &model.calibration,
                model.rig.default_pose(),
            );
            let mut script = script.clone();
            if let Some(first) = script.packets.first_mut() {
                first.config = rk_proto::ConfigCmd::SetBalanceGains(gains);
            }
            runner.run_script(&script);
            runner.last_walk_quality()
        }

        let tuned_quality = run_with_gains(tuned, &script);
        let default_quality = run_with_gains(rk_proto::BalanceGains::default(), &script);
        assert_eq!(quality, tuned_quality, "run_experiment must use host gains");
        assert_ne!(quality, default_quality, "tuned gains must affect the scripted run");
    }

    #[test]
    fn disabling_mcp_rejects_requests_cleanly() {
        let mut host = HeadlessHost::new(
            McpPolicy {
                enabled: false,
                drive_armed: false,
            },
            AutomationRoots::default(),
        );
        let resp = dispatch_json_rpc(
            &mut host,
            json!({"jsonrpc": "2.0", "id": 9, "method": "tools/list"}),
        )
        .expect("disabled error");
        assert_eq!(resp["error"]["code"], -32000);
    }

    /// Behavior coverage for the tool surfaces that previously had only schema
    /// coverage (`sim-verification-complete` Phase C). Every case drives a real
    /// `HeadlessHost` and asserts the simulator/model state actually changed.
    mod tool_behavior {
        use super::*;
        use crate::headless::{AutomationRoots, HeadlessHost};
        use crate::host::McpPolicy;
        use crate::test_support::TEST_LOCK;

        fn new_host() -> HeadlessHost {
            HeadlessHost::new(McpPolicy::default(), AutomationRoots::default())
        }

        fn call(host: &mut HeadlessHost, id: u64, name: &str, args: Value) -> Value {
            dispatch_json_rpc(
                host,
                json!({
                    "jsonrpc": "2.0",
                    "id": id,
                    "method": "tools/call",
                    "params": {"name": name, "arguments": args}
                }),
            )
            .unwrap_or_else(|| panic!("{name} produced no reply"))
        }

        /// Unwraps a successful tool result into its JSON payload.
        fn ok_body(reply: &Value) -> Value {
            assert!(reply["result"]["isError"].is_null(), "tool failed: {reply}");
            let text = reply["result"]["content"][0]["text"]
                .as_str()
                .unwrap_or_else(|| panic!("missing tool text: {reply}"));
            serde_json::from_str(text).unwrap_or_else(|e| panic!("tool text is not JSON ({e}): {text}"))
        }

        fn connect(host: &mut HeadlessHost) {
            let reply = call(host, 1, "robot_connect", json!({"transport": "sim"}));
            assert!(reply["result"]["isError"].is_null(), "{reply}");
            assert!(host.is_connected());
        }

        fn arm_drive(host: &mut HeadlessHost) {
            host.set_drive_armed(true);
            let reply = call(host, 2, "mode_set", json!({"mode": "drive"}));
            assert!(reply["result"]["isError"].is_null(), "{reply}");
            assert_eq!(host.model().requested_mode, ControlMode::Drive);
            // One packet per batch: a single packet held for >100 ms latches the
            // deadman watchdog, so warm up in sub-deadman batches.
            for _ in 0..4 {
                host.step_ticks(19, 0.01);
            }
        }

        #[test]
        fn balance_reset_defaults_restores_all_fourteen_gains() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);

            let tuned = ok_body(&call(
                &mut host,
                3,
                "balance_set_gains",
                json!({"uvc_gain_pitch": 1.2, "gyro_damp_pitch": 0.5, "h_com_m": 0.5}),
            ));
            assert!(
                (tuned["gains"]["uvc_gain_pitch"].as_f64().unwrap() - 1.2).abs() < 1e-6,
                "{tuned}"
            );
            assert_ne!(host.model().balance_gains, BalanceGains::default());

            let body = ok_body(&call(&mut host, 4, "balance_reset_defaults", json!({})));
            assert_eq!(
                body["gains"],
                serde_json::to_value(BalanceGains::default()).unwrap(),
                "{body}"
            );
            assert_eq!(host.model().balance_gains, BalanceGains::default());
        }

        #[test]
        fn invalid_gain_write_returns_structured_error() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);

            let reply = call(&mut host, 3, "balance_set_gains", json!({"uvc_gain_pitch": 3.0}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
            let body: Value = serde_json::from_str(
                reply["result"]["content"][0]["text"].as_str().expect("error text"),
            )
            .expect("structured error JSON");
            assert_eq!(body["error"]["code"], "invalid_gain", "{body}");
            assert_eq!(body["error"]["field"], "uvc_gain_pitch", "{body}");
            assert_eq!(body["error"]["value"], 3.0, "{body}");
            assert_eq!(body["error"]["bounds"], json!([0.0, 1.5]), "{body}");
            assert!(
                body["error"]["message"].as_str().unwrap().contains("uvc_gain_pitch"),
                "{body}"
            );
            assert_eq!(
                host.model().balance_gains,
                BalanceGains::default(),
                "an invalid write must not mutate gains"
            );
        }

        #[test]
        fn acceptance_tools_are_advertised_in_tools_list() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            let reply = dispatch_json_rpc(
                &mut host,
                json!({"jsonrpc": "2.0", "id": 1, "method": "tools/list"}),
            )
            .expect("tools list");
            let listed: Vec<&str> = reply["result"]["tools"]
                .as_array()
                .expect("tool array")
                .iter()
                .map(|t| t["name"].as_str().expect("tool name"))
                .collect();
            for name in [
                "quality_snapshot",
                "balance_reset_defaults",
                "experiment_list",
                "experiment_run_named",
            ] {
                assert!(listed.contains(&name), "missing tool schema for `{name}`");
            }
        }

        #[test]
        fn experiment_list_reports_committed_fixtures() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            let body = ok_body(&call(&mut host, 2, "experiment_list", json!({})));
            let experiments = body["experiments"].as_array().expect("experiment array");
            let names: Vec<&str> = experiments
                .iter()
                .map(|e| e["name"].as_str().expect("experiment name"))
                .collect();
            assert_eq!(
                names,
                vec!["walk_nominal.json", "walk_shove.json", "fall_recover.json"]
            );
            for experiment in experiments {
                assert!(
                    experiment["description"].as_str().unwrap_or("").len() > 10,
                    "missing description: {experiment}"
                );
            }
        }

        #[test]
        fn experiment_run_named_replays_exactly_through_confined_read() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let read_root = std::fs::canonicalize(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/experiments"),
            )
            .expect("fixtures directory");
            let mut host = HeadlessHost::new(
                McpPolicy::default(),
                AutomationRoots {
                    read_root: Some(read_root),
                    write_root: None,
                },
            );

            let body = ok_body(&call(
                &mut host,
                2,
                "experiment_run_named",
                json!({"name": "walk_nominal.json"}),
            ));
            assert_eq!(body["exact_match"], true, "{body}");
            assert_eq!(body["verification"]["exact_match"], true, "{body}");
            assert_eq!(body["total_ticks"], 800, "{body}");
            assert_eq!(body["matched_ticks"], 800, "{body}");
            assert!(!body["quality"].is_null());

            // A short name resolves to the same committed fixture.
            let short = ok_body(&call(
                &mut host,
                3,
                "experiment_run_named",
                json!({"name": "walk_nominal"}),
            ));
            assert_eq!(short["quality"], body["quality"]);

            let reply = call(&mut host, 4, "experiment_run_named", json!({"name": "../../evil.json"}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn experiment_run_named_uses_embedded_fixture_without_file_access() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            // No --automation-read-root: the GUI bridge cannot read files, so
            // the embedded fixture bytes must take over.
            let mut host = new_host();
            let body = ok_body(&call(
                &mut host,
                2,
                "experiment_run_named",
                json!({"name": "fall_recover.json"}),
            ));
            assert_eq!(body["exact_match"], true, "{body}");
            assert_eq!(body["total_ticks"], 400, "{body}");
            assert_eq!(body["verification"]["exact_match"], true, "{body}");
            assert!(body["quality"]["peak_tilt_rad"].as_f64().unwrap() > 0.5, "{body}");
        }

        #[test]
        fn action_play_and_stop_control_real_playback() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            arm_drive(&mut host);

            let move_idx = host.model().moves.iter().position(|m| m.name == "Get Up Prone").unwrap();
            let name = host.model().moves[move_idx].name.clone();
            let body = ok_body(&call(
                &mut host,
                3,
                "action_play",
                json!({"name": name, "step_ticks": 20}),
            ));
            assert_eq!(body["playback"]["name"], name, "{body}");
            let pb = host.model().playback.as_ref().expect("playback started");
            assert_eq!(pb.move_idx, move_idx);
            assert_eq!(host.model().selected_move, move_idx);
            assert!(pb.progress > 0.0, "stepped ticks must advance the action");

            let body = ok_body(&call(&mut host, 4, "action_stop", json!({})));
            assert!(body["playback"].is_null(), "{body}");
            assert!(host.model().playback.is_none(), "action_stop must clear playback");
            assert!(host.model().status_message.contains("stopped"));

            // Unknown action must fail without disturbing the running playback.
            let reply = call(&mut host, 5, "action_play", json!({"name": "no_such_action"}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn walk_set_velocity_and_gait_configure_change_locomotion_state() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            arm_drive(&mut host);
            let before_uptime = host.model().latest_telemetry.uptime_ms;

            let body = ok_body(&call(
                &mut host,
                3,
                "walk_set_velocity",
                json!({"vx": 0.3, "vy": -0.1, "yaw_rate": 0.15, "walk_enabled": true, "step_ticks": 20}),
            ));
            let w = host.model().walk_cmd;
            assert!((w.vx - 0.3).abs() < 1e-6 && (w.vy + 0.1).abs() < 1e-6);
            assert!((w.yaw_rate - 0.15).abs() < 1e-6 && w.walk_enabled);
            assert_eq!(body["balance"]["walk"]["walk_enabled"], true, "{body}");
            assert!(
                host.model().latest_telemetry.uptime_ms > before_uptime,
                "step_ticks must drive the simulation forward"
            );

            let body = ok_body(&call(
                &mut host,
                4,
                "gait_configure",
                json!({"step_height": 0.7, "cadence_hz": 1.8, "stance_crouch": 0.4}),
            ));
            let w = host.model().walk_cmd;
            assert!((w.step_height - 0.7).abs() < 1e-6);
            assert!((w.cadence_hz - 1.8).abs() < 1e-6);
            assert!((w.stance_crouch - 0.4).abs() < 1e-6);
            assert!((body["walk"]["cadence_hz"].as_f64().unwrap() - 1.8).abs() < 1e-6, "{body}");

            // Walking is still guarded while Drive is disarmed.
            host.set_drive_armed(false);
            let reply = call(&mut host, 5, "walk_set_velocity", json!({"vx": 0.5, "walk_enabled": true}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn telemetry_snapshot_steps_simulation_and_publishes_state() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            let before = host.model().latest_telemetry.uptime_ms;

            let body = ok_body(&call(&mut host, 2, "telemetry_snapshot", json!({"step_ticks": 19})));
            assert_eq!(body["connected"], true, "{body}");
            assert_eq!(body["active_mode"], "Teach", "{body}");
            assert_eq!(body["joints"].as_array().unwrap().len(), host.model().rig.joints.len());
            assert!(body["imu"]["available"].as_bool().unwrap(), "{body}");
            assert_ne!(body["estimator"]["valid_mask"].as_u64().unwrap(), 0);
            assert!(
                host.model().latest_telemetry.uptime_ms > before,
                "telemetry_snapshot(step_ticks=19) must advance the sim"
            );

            // Over-large batches are rejected instead of silently clamped.
            let reply = call(&mut host, 3, "telemetry_snapshot", json!({"step_ticks": 9999}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn calibration_tools_mutate_and_round_trip_calibration() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);

            let body = ok_body(&call(&mut host, 2, "calibration_get", json!({})));
            let dof = host.model().rig.joints.len();
            assert_eq!(body["joints"].as_array().unwrap().len(), dof);

            let body = ok_body(&call(
                &mut host,
                3,
                "calibration_set_joint",
                json!({"joint": 0, "min_rad": -1.4, "max_rad": 1.4,
                       "zero_offset_rad": 0.25, "inverted": true}),
            ));
            let cal = host.model().calibration[0];
            assert!((cal.min_rad + 1.4).abs() < 1e-6 && (cal.max_rad - 1.4).abs() < 1e-6);
            assert!((cal.zero_offset_rad - 0.25).abs() < 1e-6);
            assert!(cal.inverted);
            assert_eq!(body["index"], 0, "{body}");
            assert_eq!(body["calibration"]["inverted"], true, "{body}");
            assert!((body["calibration"]["zero_offset_rad"].as_f64().unwrap() - 0.25).abs() < 1e-6, "{body}");

            let body = ok_body(&call(&mut host, 4, "calibration_zero_here", json!({"joint": 15})));
            assert_eq!(body["index"], 15, "{body}");
            assert_eq!(host.model().target_rad[15], 0.0, "zero-here must zero the joint target");

            let body = ok_body(&call(&mut host, 5, "calibration_export_json", json!({})));
            let exported = body["calibration_json"].as_str().unwrap();
            let mut file: Value = serde_json::from_str(exported).unwrap();
            assert_eq!(file["robot_name"], host.model().rig.robot_name);
            file["joints"][0]["zero_offset_rad"] = json!(-0.5_f32);

            let body = ok_body(&call(
                &mut host,
                6,
                "calibration_import_json",
                json!({"json": file.to_string()}),
            ));
            assert!((host.model().calibration[0].zero_offset_rad + 0.5).abs() < 1e-6);
            assert_eq!(body["joints"].as_array().unwrap().len(), dof);
        }

        #[test]
        fn robot_presets_urdf_and_mcu_sync_change_model_state() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            let original_name = host.model().rig.robot_name.clone();
            let original_dof = host.model().rig.joints.len();

            let body = ok_body(&call(&mut host, 2, "robot_list_presets", json!({})));
            let presets = body["presets"].as_array().unwrap();
            assert!(presets.iter().any(|p| p["id"] == "humanoid"));
            assert!(presets.iter().any(|p| p["id"] == "striker_biped"));

            let body = ok_body(&call(&mut host, 3, "robot_load_preset", json!({"preset": "striker_biped"})));
            assert_ne!(body["robot"]["robot_name"], original_name, "{body}");
            assert_ne!(host.model().rig.joints.len(), original_dof);

            let bundle = AssetBundle::bundled_humanoid();
            let body = ok_body(&call(
                &mut host,
                4,
                "robot_load_urdf",
                json!({"name": "agent_robot.urdf", "urdf_xml": bundle.urdf_xml}),
            ));
            assert_eq!(host.model().rig.robot_name, original_name, "{body}");
            assert_eq!(host.model().rig.joints.len(), original_dof);

            // Stage a distinct blob into the simulated MCU and prove the tool
            // reports the staged model, not a hardcoded default.
            let blob = b"<robot name=\"staged_for_mcp\"/>".to_vec();
            let len = blob.len();
            assert!(host.transport_mut().stage_sim_mcu_model(
                "staged_for_mcp",
                rk_proto::ModelFormat::UrdfXml,
                blob,
                2,
            ));
            let body = ok_body(&call(&mut host, 5, "mcu_sync_model", json!({})));
            let msg = body["message"].as_str().unwrap();
            assert!(msg.contains("staged_for_mcp"), "{msg}");
            assert!(msg.contains(&format!("{len} bytes")), "{msg}");
            assert!(msg.contains("crc32="), "{msg}");

            // Before connecting, the same tool must fail cleanly.
            let mut fresh = new_host();
            let reply = call(&mut fresh, 6, "mcu_sync_model", json!({}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn safety_estop_and_reset_change_authority_and_faults() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            arm_drive(&mut host);
            let _ = call(
                &mut host,
                3,
                "sim_inject_fault",
                json!({"drop_heartbeat": true}),
            );
            assert!(host.transport().sim_faults().unwrap().drop_heartbeat);

            let body = ok_body(&call(&mut host, 4, "safety_estop", json!({})));
            assert_eq!(body["estop_active"], true, "{body}");
            assert!(host.model().is_estop_active());
            assert!(!host.policy().drive_armed, "E-Stop must disarm Agent Drive");

            let body = ok_body(&call(&mut host, 5, "safety_reset_estop", json!({})));
            assert_eq!(body["estop_active"], false, "{body}");
            assert_eq!(body["requested_mode"], "Teach", "{body}");
            assert!(!host.model().is_estop_active());
            assert_eq!(host.model().requested_mode, ControlMode::Teach);
            assert!(!host.policy().drive_armed, "reset must not re-arm Drive");
            assert!(!host.transport().sim_faults().unwrap().drop_heartbeat);

            let reply = call(&mut host, 6, "mode_set", json!({"mode": "drive"}));
            assert_eq!(reply["result"]["isError"], true, "Drive stays guarded after reset");
        }

        #[test]
        fn limb_solve_ik_reposes_the_selected_limb() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            arm_drive(&mut host);

            // Plant an active action and a different selected joint first: the
            // IK tool applies its solved pose through `apply_joint_angle`, which
            // must abort playback and leave the last applied joint selected.
            let play = ok_body(&call(&mut host, 2, "action_play", json!({"name": "Get Up Prone"})));
            assert!(!play["playback"].is_null(), "{play}");
            host.model_mut().selected_joint = 0;

            let before = host.model().target_rad;
            let foot = host.model().rig.compute_link_transforms(&before)
                .get("Foot_Left_1").expect("Foot_Left_1 link").translation.vector;
            let goal = [foot.x + 0.04, foot.y + 0.04, foot.z + 0.05];

            let body = ok_body(&call(
                &mut host,
                3,
                "limb_solve_ik",
                json!({"link_name": "Foot_Left_1", "x": goal[0], "y": goal[1], "z": goal[2]}),
            ));
            assert_eq!(body["link_name"], "Foot_Left_1", "{body}");
            let target = body["target"].as_array().unwrap();
            for (got, want) in target.iter().zip(goal) {
                assert!((got.as_f64().unwrap() as f32 - want).abs() < 1e-6, "{body}");
            }
            assert!(body["telemetry"]["joints"].as_array().unwrap().len() >= 21, "{body}");
            // Observable proof that the solved pose was applied to every joint.
            assert!(host.model().playback.is_none(), "IK application must abort playback");
            assert_eq!(
                host.model().selected_joint,
                host.model().rig.joints.len() - 1,
                "IK must apply the solved pose to every joint"
            );

            let reply = call(&mut host, 4, "limb_solve_ik", json!({"link_name": "not_a_link", "x": 0.0, "y": 0.0, "z": 0.0}));
            assert_eq!(reply["result"]["isError"], true, "{reply}");
        }

        #[test]
        fn joint_targets_move_the_simulated_servos() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let mut host = new_host();
            connect(&mut host);
            let measured_before = host.model().latest_telemetry.measured_rad[0];

            let body = ok_body(&call(
                &mut host,
                2,
                "joint_set_target",
                json!({"joint": 0, "angle_rad": 0.6}),
            ));
            assert_eq!(body["index"], 0, "{body}");
            // In Teach mode build_next_host_packet mirrors measured angles back
            // into targets, so the durable effect is the moved virtual-hand angle.
            assert_eq!(host.model().selected_joint, 0);
            assert!(
                (host.model().latest_telemetry.measured_rad[0] - measured_before).abs() > 1e-6,
                "step_fixed after the target must move the measured joint"
            );

            let body = ok_body(&call(
                &mut host,
                3,
                "joints_set_targets",
                json!({"units": "deg", "targets": {"l_hip_pitch": 15.0, "1": -8.0}}),
            ));
            assert_eq!(body["applied_count"], 2, "{body}");
            // Give the plant's servo dynamics time to reach the new virtual-hand
            // angles; each batch carries a fresh packet, below the deadman limit.
            for _ in 0..6 {
                host.step_ticks(19, 0.01);
            }
            let measured = host.model().latest_telemetry.measured_rad;
            let l_hip_pitch_idx = host
                .model()
                .rig
                .joints
                .iter()
                .position(|j| j.name == "l_hip_pitch")
                .expect("l_hip_pitch joint");
            assert!((measured[l_hip_pitch_idx] - 15.0_f32.to_radians()).abs() < 0.05, "{measured:?}");
            assert!((measured[1] - (-8.0_f32).to_radians()).abs() < 0.05, "{measured:?}");

            let _ = call(&mut host, 4, "safety_estop", json!({}));
            let reply = call(&mut host, 5, "joint_set_target", json!({"joint": 0, "angle_rad": 0.1}));
            assert_eq!(reply["result"]["isError"], true, "joint motion is blocked by E-Stop");
        }
    }

    /// Acceptance evidence for `sim-verification-complete` (Phase A).
    ///
    /// The committed scripts are loaded from disk through the *same confined-file
    /// `run_experiment` path* the native MCP CLI exposes, and each is re-run
    /// through the shared `CopperSimRunner` so the recorded `WalkQuality` can be
    /// asserted. The golden `.copper` log is replayed through the shared
    /// `ControlTask` independently.
    mod acceptance_experiments {
        use super::*;
        use crate::headless::{AutomationRoots, HeadlessHost};
        use crate::host::McpPolicy;
        use crate::test_support::TEST_LOCK;
        use rk_fw_core::replay::{
            CopperReplayLog, SimEventKind, TimedSimEvent, verify_deterministic_replay,
        };
        use rk_model::StudioModel;
        use rk_proto::{BalanceGains, ConfigCmd, ControlMode, FaultFlags, HostToMcu, WalkCmd};
        use rk_sim::{
            CopperSimRunner, ExperimentScript, SimFaultInjection, WalkQuality, WalkQualityBounds,
            fault_snapshot, scripted_walk_packet,
        };
        use std::path::{Path, PathBuf};

        const NOMINAL_TICKS: usize = 800;
        const SHOVE_TICKS: usize = 800;
        const FALL_TICKS: usize = 400;

        fn fixtures_dir() -> PathBuf {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/experiments")
        }

        fn load_script(name: &str) -> ExperimentScript {
            let path = fixtures_dir().join(name);
            let text = std::fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
            serde_json::from_str(&text)
                .unwrap_or_else(|e| panic!("parse {}: {e}", path.display()))
        }

        /// Mirrors `McpHost::run_experiment`: the same host inputs (including the
        /// tick-0 gain injection), the same shared runner, and therefore the same
        /// `WalkQuality` the MCP tool observed.
        fn run_for_quality(script: &ExperimentScript) -> (WalkQuality, CopperReplayLog) {
            let model = StudioModel::new_default();
            let mut runner = CopperSimRunner::new(
                model.rig.joints.len().max(1),
                &model.calibration,
                model.rig.default_pose(),
            );
            let mut script = script.clone();
            if let Some(first) = script.packets.first_mut() {
                first.config = ConfigCmd::SetBalanceGains(model.balance_gains);
            }
            let log = runner.run_script(&script);
            (runner.last_walk_quality(), log)
        }

        fn call_run_experiment(host: &mut HeadlessHost, id: u64, name: &str) -> serde_json::Value {
            let reply = dispatch_json_rpc(
                host,
                json!({
                    "jsonrpc": "2.0", "id": id, "method": "tools/call",
                    "params": {"name": "run_experiment", "arguments": {"file_path": name}}
                }),
            )
            .expect("run_experiment reply");
            assert!(reply["result"]["isError"].is_null(), "{reply}");
            let body: serde_json::Value = serde_json::from_str(
                reply["result"]["content"][0]["text"].as_str().expect("tool text"),
            )
            .expect("tool JSON");
            body
        }

        fn assert_exact(body: &serde_json::Value, ticks: usize) {
            assert_eq!(body["exact_match"], true, "{body}");
            assert_eq!(body["total_ticks"], ticks, "{body}");
            assert_eq!(body["matched_ticks"], ticks, "{body}");
            assert_eq!(body["verification"]["exact_match"], true, "{body}");
            assert_eq!(body["verification"]["total_ticks"], ticks, "{body}");
            assert_eq!(body["verification"]["matched_ticks"], ticks, "{body}");
            assert!(!body["quality"].is_null(), "{body}");
        }

        /// Fault latching stays a telemetry assertion; every numeric band comes
        /// from the shared `rk-sim` evaluator so the test cannot drift from the
        /// acceptance bounds exposed to agents by `quality_snapshot`.
        fn assert_walking_bounds(
            label: &str,
            quality: &WalkQuality,
            log: &CopperReplayLog,
            bounds: &WalkQualityBounds,
        ) {
            let faults = log.ticks.last().expect("script ticks").tx_telem.faults;
            assert!(
                !faults.contains(FaultFlags::TILT_FALL_CUTOFF)
                    && !faults.contains(FaultFlags::OVER_CURRENT)
                    && !faults.contains(FaultFlags::WATCHDOG_TIMEOUT),
                "{label} latched faults: {faults:?}"
            );
            let evaluation = bounds.evaluate(quality);
            assert!(
                evaluation.pass,
                "{label} walk-quality bounds violated: {:?}",
                evaluation.violations
            );
        }

        /// The shove fixture uses the documented disturbance bands (peak tilt
        /// 0.35, steady 0.05, lateral 0.08) while keeping every other bound
        /// identical to the nominal acceptance matrix.
        fn shove_bounds() -> WalkQualityBounds {
            WalkQualityBounds {
                peak_tilt_rad_max: 0.35,
                steady_tilt_rad_max: 0.05,
                lateral_drift_m_max: 0.08,
                ..WalkQualityBounds::default()
            }
        }

        #[test]
        fn committed_scripts_replay_exactly_and_meet_quality_bounds() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let read_root = std::fs::canonicalize(fixtures_dir()).expect("fixtures directory");
            let mut host = HeadlessHost::new(
                McpPolicy::default(),
                AutomationRoots {
                    read_root: Some(read_root),
                    write_root: None,
                },
            );

            for (id, name, ticks) in [
                (2u64, "walk_nominal.json", NOMINAL_TICKS),
                (3, "walk_shove.json", SHOVE_TICKS),
                (4, "fall_recover.json", FALL_TICKS),
            ] {
                let script = load_script(name);
                assert_eq!(script.max_ticks, ticks, "{name} tick count changed");
                let body = call_run_experiment(&mut host, id, name);
                assert_exact(&body, ticks);
                let tool_quality: WalkQuality = serde_json::from_value(body["quality"].clone())
                    .unwrap_or_else(|e| panic!("{name} tool quality: {e}"));

                let (quality, log) = run_for_quality(&script);
                assert_eq!(tool_quality, quality, "{name} tool quality must match the run");
                assert_eq!(log.ticks.len(), ticks, "{name} recorded tick count");
                assert!(
                    verify_deterministic_replay(&log).exact_match,
                    "{name} replay must be bit-exact"
                );
                eprintln!("{name}: {quality:?}");

                match name {
                    "walk_nominal.json" => {
                        assert_walking_bounds(name, &quality, &log, &WalkQualityBounds::default());
                        assert!(quality.peak_tilt_rad < 0.10, "nominal peak tilt");
                        assert!(quality.steady_tilt_rad < 0.03, "nominal steady tilt");
                        assert!(quality.lateral_drift_m < 0.03, "nominal lateral drift");
                        assert!(quality.max_foot_slip_m < 0.05, "nominal foot slip");
                        assert!(
                            quality.catch_step_fraction < 0.5,
                            "nominal catch-step reporting: {}",
                            quality.catch_step_fraction
                        );
                    }
                    "walk_shove.json" => {
                        assert_walking_bounds(name, &quality, &log, &shove_bounds());
                        assert!(quality.peak_tilt_rad < 0.35, "shoved peak tilt");
                        assert!(quality.steady_tilt_rad < 0.05, "shoved steady tilt");
                        assert!(quality.lateral_drift_m < 0.08, "shoved lateral drift");
                    }
                    "fall_recover.json" => {
                        let last = log.ticks.last().expect("script ticks").tx_telem;
                        assert!(!last.faults.contains(FaultFlags::TILT_FALL_CUTOFF), "{last:?}");
                        assert!(
                            last.imu.pitch_rad.abs() < 0.08 && last.imu.roll_rad.abs() < 0.08,
                            "recovery must end upright: pitch={} roll={}",
                            last.imu.pitch_rad,
                            last.imu.roll_rad
                        );
                        assert!(quality.mean_forward_speed_m_s.is_finite());
                        assert!(quality.peak_tilt_rad > 0.5, "script must actually fall");
                    }
                    _ => unreachable!("fixture list is closed"),
                }
            }
        }

        #[test]
        fn golden_copper_log_verifies_bit_exactly() {
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let path = fixtures_dir().join("walk_nominal.copper");
            let text = std::fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
            let log: CopperReplayLog = serde_json::from_str(&text)
                .unwrap_or_else(|e| panic!("parse {}: {e}", path.display()));
            let check = verify_deterministic_replay(&log);
            assert!(check.exact_match, "golden log must replay exactly: {check:?}");
            assert_eq!(check.total_ticks, NOMINAL_TICKS);
            assert_eq!(check.matched_ticks, NOMINAL_TICKS);
            assert!(log.events.iter().any(|event| {
                event.tick_index == 0
                    && matches!(event.kind, SimEventKind::SetFaults(_))
            }));
        }

        fn fault_event(tick: u64, faults: &SimFaultInjection) -> TimedSimEvent {
            TimedSimEvent {
                tick_index: tick,
                kind: SimEventKind::SetFaults(fault_snapshot(faults)),
            }
        }

        fn standing_packet(seq: u32, mode: ControlMode, config: ConfigCmd) -> HostToMcu {
            HostToMcu {
                seq,
                mode,
                torque_mask: (1u64 << 28) - 1,
                speed_limit_pct: 100,
                walk: WalkCmd {
                    walk_enabled: false,
                    balance_enabled: true,
                    ..Default::default()
                },
                config,
                ..Default::default()
            }
        }

        fn nominal_script() -> ExperimentScript {
            let nominal = SimFaultInjection::default();
            ExperimentScript {
                seed: nominal.seed,
                events: vec![fault_event(0, &nominal)],
                packets: (0..NOMINAL_TICKS as u32).map(scripted_walk_packet).collect(),
                max_ticks: NOMINAL_TICKS,
            }
        }

        fn shove_script() -> ExperimentScript {
            let realistic = SimFaultInjection::realistic_small_humanoid();
            let mut script = nominal_script();
            script.seed = realistic.seed;
            script.events = vec![
                fault_event(0, &realistic),
                TimedSimEvent {
                    tick_index: 300,
                    kind: SimEventKind::Shove {
                        pitch_rad_s: 1.2,
                        roll_rad_s: -0.4,
                    },
                },
            ];
            script.max_ticks = SHOVE_TICKS;
            script.packets = (0..SHOVE_TICKS as u32).map(scripted_walk_packet).collect();
            script
        }

        fn fall_recover_script() -> ExperimentScript {
            let nominal = SimFaultInjection::default();
            let mut events = vec![
                fault_event(0, &nominal),
                TimedSimEvent {
                    tick_index: 30,
                    kind: SimEventKind::Shove {
                        pitch_rad_s: 6.0,
                        roll_rad_s: 0.0,
                    },
                },
            ];
            for tick in (162..322).step_by(8) {
                events.push(TimedSimEvent {
                    tick_index: tick,
                    kind: SimEventKind::Recovery(Some((tick - 162) as f32 / 160.0)),
                });
            }
            events.push(TimedSimEvent {
                tick_index: 322,
                kind: SimEventKind::Recovery(Some(1.0)),
            });
            events.push(TimedSimEvent {
                tick_index: 340,
                kind: SimEventKind::Recovery(None),
            });

            let rec_gains = BalanceGains {
                fall_cutoff_rad: 1.35,
                ..Default::default()
            };
            let packets = (0..FALL_TICKS as u32)
                .map(|seq| match seq {
                    160 => standing_packet(seq, ControlMode::Teach, ConfigCmd::SetBalanceGains(rec_gains)),
                    161 => standing_packet(seq, ControlMode::Teach, ConfigCmd::ClearFaults),
                    323 => standing_packet(seq, ControlMode::Teach, ConfigCmd::SetBalanceGains(BalanceGains::default())),
                    _ => standing_packet(seq, ControlMode::Drive, ConfigCmd::None),
                })
                .collect();
            ExperimentScript {
                seed: nominal.seed,
                events,
                packets,
                max_ticks: FALL_TICKS,
            }
        }

        /// Regeneration aid for the committed fixtures. Run once from the
        /// workspace root with `RK_GENERATE_ACCEPTANCE_FIXTURES=1 cargo test -p
        /// rk-mcp --lib acceptance_experiments::generate_fixtures -- --ignored
        /// --nocapture`; the test is ignored in normal CI runs.
        #[test]
        #[ignore = "regenerates committed acceptance fixtures on demand"]
        fn generate_fixtures() {
            if std::env::var_os("RK_GENERATE_ACCEPTANCE_FIXTURES").is_none() {
                return;
            }
            let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let dir = fixtures_dir();
            std::fs::create_dir_all(&dir).expect("create fixture directory");
            let scripts = [
                ("walk_nominal.json", nominal_script()),
                ("walk_shove.json", shove_script()),
                ("fall_recover.json", fall_recover_script()),
            ];
            for (name, script) in &scripts {
                let path = dir.join(name);
                let bytes = serde_json::to_vec(script).expect("serialize script");
                std::fs::write(&path, &bytes).expect("write script");
                eprintln!("wrote {} ({} bytes)", path.display(), bytes.len());
            }

            let model = StudioModel::new_default();
            let mut runner = CopperSimRunner::new(
                model.rig.joints.len().max(1),
                &model.calibration,
                model.rig.default_pose(),
            );
            let log = runner.run_script(&scripts[0].1);
            let path = dir.join("walk_nominal.copper");
            let bytes = serde_json::to_vec(&log).expect("serialize golden log");
            std::fs::write(&path, &bytes).expect("write golden log");
            eprintln!("wrote {} ({} bytes)", path.display(), bytes.len());

            for (name, script) in &scripts {
                let (quality, log) = run_for_quality(script);
                let last = log.ticks.last().expect("ticks").tx_telem;
                eprintln!(
                    "{name}: quality={quality:?} mode={:?} faults={:?} pitch={} roll={}",
                    last.active_mode, last.faults, last.imu.pitch_rad, last.imu.roll_rad
                );
            }
        }
    }
}
