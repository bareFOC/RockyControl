//! `craft_mcp_server::Tools` implementation for Rocky Studio (`RockyMcpTools`).

use std::sync::atomic::{AtomicU64, Ordering};

use craft_mcp_server::{Contents, Info, Options, Reference, Server, Tools, text_result};
use rk_link::TransportKind;
use rk_model::nalgebra::Point3;
use rk_model::{AssetBundle, samples};
use rk_proto::{ControlMode, FallOrientation, ModelRequest, ModelResponse, uvc_phase};
use rk_sim::verify_deterministic_replay;
use serde_json::{Value, json};

use crate::host::{McpActivityEntry, McpCallOutcome, McpHost};
use crate::schema::{
    URDF_PRESETS, prompt_definitions, resource_definitions, resource_template_definitions,
    tool_definitions,
};

const MAX_URDF_BYTES: usize = 2 * 1024 * 1024;
const MAX_LINE_BYTES: usize = 2_621_440;
const MAX_STEP_TICKS: usize = 400;

static NEXT_ACTIVITY_SEQ: AtomicU64 = AtomicU64::new(1);

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
        "Rocky Studio Robotics MCP Server. Use resources (rocky://status, rocky://telemetry, rocky://robot, rocky://calibration, rocky://balance, rocky://actions) to inspect live robot state. Configuration, URDF loading, calibration, UVC balance tuning, and Teach posing are always permitted when MCP is enabled. Switching to energized Drive mode or commanding active locomotion requires the human operator to arm 'Allow Agent Drive' in Rocky Studio (or --allow-drive on CLI). E-Stop always cuts torque immediately and disarms Drive.".to_string()
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
                    "rocky_mk1" => Ok(AssetBundle::bundled_rocky()),
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
                let updates: [(&str, f32, f32, &mut f32); 7] = [
                    ("uvc_gain_pitch", 0.0, 1.5, &mut g.uvc_gain_pitch),
                    ("uvc_gain_roll", 0.0, 1.5, &mut g.uvc_gain_roll),
                    ("tilt_deadband_rad", 0.005, 0.15, &mut g.tilt_deadband_rad),
                    ("gyro_damp_pitch", 0.0, 1.0, &mut g.gyro_damp_pitch),
                    ("gyro_damp_roll", 0.0, 1.0, &mut g.gyro_damp_roll),
                    ("arm_assist", 0.0, 1.5, &mut g.arm_assist),
                    ("fall_cutoff_rad", 0.30, 1.20, &mut g.fall_cutoff_rad),
                ];
                for (key, lo, hi, field) in updates {
                    match Self::finite_f32(args, key, lo, hi) {
                        Ok(Some(v)) => *field = v,
                        Ok(None) => {}
                        Err(e) => return self.err_result(name, key, e),
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
                    for _ in 0..ticks {
                        self.host.step_ticks(1, 0.01);
                        let t = &self.host.model().latest_telemetry;
                        peak_pitch_rad = peak_pitch_rad.max(t.imu.pitch_rad.abs());
                        peak_roll_rad = peak_roll_rad.max(t.imu.roll_rad.abs());
                        if t.balance.uvc_phase == uvc_phase::ACTIVE_CATCH_STEP {
                            saw_catch_step = true;
                        }
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
                if args.get("corrupt_frame").and_then(Value::as_bool) == Some(true) {
                    self.host.transport_mut().inject_corrupted_frame();
                }
                let Some(faults) = self.host.transport_mut().sim_faults_mut() else {
                    return self.err_result(name, "sim only", "Fault injection is only supported on the Simulation transport.");
                };
                if let Some(drop_hb) = args.get("drop_heartbeat").and_then(Value::as_bool) {
                    faults.drop_heartbeat = drop_hb;
                }
                if let Some(sag) = args.get("voltage_sag").and_then(Value::as_bool) {
                    faults.voltage_sag = sag;
                }
                if let Some(idx) = stall_idx {
                    let stalled = args.get("stalled").and_then(Value::as_bool).unwrap_or(true);
                    if idx < faults.stalled_joints.len() {
                        faults.stalled_joints[idx] = stalled;
                    }
                }
                let summary = json!({
                    "drop_heartbeat": faults.drop_heartbeat,
                    "voltage_sag": faults.voltage_sag,
                });
                self.ok_json(name, "faults updated", summary)
            }
            "recording_control" => {
                let action = args.get("action").and_then(Value::as_str).unwrap_or("");
                match action {
                    "start" => {
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
        host.step_ticks(30, 0.01);
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
}
