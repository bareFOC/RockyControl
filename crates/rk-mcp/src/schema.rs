//! MCP Tool, Resource, Resource Template, and Prompt schemas built on `craft-mcp-server`.

use craft_mcp_server::{Effect, object, string, tool};
use serde_json::{Value, json};

/// Built-in URDF preset identifiers exposed to MCP clients and completions.
pub const URDF_PRESETS: &[(&str, &str)] = &[
    ("rocky_mk1", "Rocky Mk1 (21-DoF Humanoid, 6-DoF UVC Legs)"),
    ("striker_biped", "Striker Biped (8-DoF ROS Z-Up)"),
    ("combat_arm", "Combat Arm (6-DoF Continuous + Prismatic)"),
];

/// Available camera view presets for `viewport_camera`.
pub const CAMERA_PRESETS: &[&str] = &[
    "iso", "front", "back", "left", "right", "top", "bottom", "fit",
];

/// Available transport backend identifiers for `robot_connect`.
pub const TRANSPORT_KINDS: &[&str] = &["sim", "usb_udp", "usb_serial"];

fn num(desc: &str) -> Value {
    json!({"type": "number", "description": desc})
}

fn integer(desc: &str) -> Value {
    json!({"type": "integer", "description": desc})
}

fn boolean(desc: &str) -> Value {
    json!({"type": "boolean", "description": desc})
}

fn enum_str(desc: &str, vals: &[&str]) -> Value {
    json!({"type": "string", "description": desc, "enum": vals})
}

fn joint_ref(desc: &str) -> Value {
    // Gemini-safe union using `anyOf` (per craft-mcp-server::check_arguments convention).
    json!({
        "description": desc,
        "anyOf": [
            {"type": "integer"},
            {"type": "string"}
        ]
    })
}

/// Complete annotated MCP tool catalog served by `tools/list`.
pub fn tool_definitions() -> Vec<Value> {
    vec![
        // --- Connection & Studio ---
        tool(
            "robot_connect",
            "Connect to Robot",
            "Connects Rocky Studio to the simulated MCU twin or wired RP2350 transport in safe Teach mode.",
            object(
                json!({
                    "transport": enum_str("Transport backend: sim (default), usb_udp, or usb_serial", TRANSPORT_KINDS)
                }),
                &[],
            ),
            Effect::Edits,
        ),
        tool(
            "robot_disconnect",
            "Disconnect Robot",
            "Safely drops to Teach mode, zeroes velocity commands, and disconnects the active robot transport.",
            object(json!({}), &[]),
            Effect::Edits,
        ),
        tool(
            "studio_navigate",
            "Navigate Studio Screen",
            "Switches the active Rocky Studio screen between dashboard, control, and config.",
            object(
                json!({
                    "screen": enum_str("Target screen", &["dashboard", "control", "config"])
                }),
                &["screen"],
            ),
            Effect::Edits,
        ),
        tool(
            "viewport_camera",
            "Orient 3D Camera",
            "Snaps the 3D viewport camera to a standard orthographic/isometric preset or refits the active robot.",
            object(
                json!({
                    "preset": enum_str("Camera preset", CAMERA_PRESETS)
                }),
                &["preset"],
            ),
            Effect::Edits,
        ),
        tool(
            "telemetry_snapshot",
            "Read Telemetry Snapshot",
            "Returns live bus voltage, IMU pitch/roll, UVC balance phase and foot offsets, faults, and joint states. Optionally advances simulation ticks first.",
            object(
                json!({
                    "step_ticks": integer("Optional number of simulation ticks (0..=400) to advance before reading")
                }),
                &[],
            ),
            Effect::ReadOnly,
        ),
        // --- Robot Model & Configuration ---
        tool(
            "robot_list_presets",
            "List Built-in Robot Presets",
            "Lists bundled URDF robot models that can be loaded immediately.",
            object(json!({}), &[]),
            Effect::ReadOnly,
        ),
        tool(
            "robot_load_preset",
            "Load Built-in Robot Preset",
            "Loads a bundled robot URDF preset (rocky_mk1, striker_biped, or combat_arm) in safe Teach mode.",
            object(
                json!({
                    "preset": enum_str("Preset identifier", &["rocky_mk1", "striker_biped", "combat_arm"])
                }),
                &["preset"],
            ),
            Effect::Replaces,
        ),
        tool(
            "robot_load_urdf",
            "Load Custom URDF XML",
            "Parses and loads a custom URDF XML string (up to 2 MB) or confined local URDF file path into Rocky Studio in safe Teach mode.",
            object(
                json!({
                    "name": string("Display filename or robot label (e.g. custom_bot.urdf)"),
                    "urdf_xml": string("Raw URDF XML content (provide either urdf_xml or file_path)"),
                    "file_path": string("Path inside --automation-read-root (native CLI only)")
                }),
                &[],
            ),
            Effect::Replaces,
        ),
        tool(
            "mcu_sync_model",
            "Query Onboard MCU Model",
            "Queries the connected MCU flash model store metadata (name, joint count, CRC32, size).",
            object(json!({}), &[]),
            Effect::ReadOnly,
        ),
        // --- Calibration & Joint Setup ---
        tool(
            "calibration_get",
            "Get Joint Calibration Table",
            "Returns per-joint zero offsets, soft limits, inversion flags, and live measured angles.",
            object(json!({}), &[]),
            Effect::ReadOnly,
        ),
        tool(
            "calibration_set_joint",
            "Configure Joint Calibration",
            "Updates soft limits (min_rad, max_rad), zero_offset_rad, and/or inverted flag for a joint and queues ConfigCmd to the MCU.",
            object(
                json!({
                    "joint": joint_ref("Joint index (0..N-1) or exact joint name (e.g. l_hip_pitch)"),
                    "min_rad": num("Lower soft limit in radians"),
                    "max_rad": num("Upper soft limit in radians"),
                    "zero_offset_rad": num("Zero offset in radians"),
                    "inverted": boolean("Whether servo polarity is inverted")
                }),
                &["joint"],
            ),
            Effect::Edits,
        ),
        tool(
            "calibration_zero_here",
            "Zero Joint at Current Angle",
            "Captures the joint's current measured raw position as its calibrated zero offset.",
            object(
                json!({
                    "joint": joint_ref("Joint index or exact joint name")
                }),
                &["joint"],
            ),
            Effect::Edits,
        ),
        tool(
            "calibration_export_json",
            "Export Calibration JSON",
            "Exports the active robot's full calibration JSON (optionally writes to a path inside --automation-write-root on native CLI).",
            object(
                json!({
                    "file_path": string("Optional output path confined to --automation-write-root (native CLI only)")
                }),
                &[],
            ),
            Effect::ReadOnly,
        ),
        tool(
            "calibration_import_json",
            "Import Calibration JSON",
            "Imports a CalibrationFile JSON payload and queues updates to the MCU control kernel.",
            object(
                json!({
                    "json": string("Calibration JSON string")
                }),
                &["json"],
            ),
            Effect::Replaces,
        ),
        // --- UVC Balance & Locomotion Tuning ---
        tool(
            "balance_get_gains",
            "Get UVC Balance & Gait Settings",
            "Returns the active geometric UVC BalanceGains and WalkCmd gait parameters.",
            object(json!({}), &[]),
            Effect::ReadOnly,
        ),
        tool(
            "balance_set_gains",
            "Tune UVC Balance Gains",
            "Updates one or more UVC BalanceGains fields and queues ConfigCmd::SetBalanceGains to the MCU.",
            object(
                json!({
                    "uvc_gain_pitch": num("Sagittal spherical UVC projection gain (0.0..=1.5)"),
                    "uvc_gain_roll": num("Lateral spherical UVC projection gain (0.0..=1.5)"),
                    "tilt_deadband_rad": num("Radial IMU tilt deadband in radians (0.005..=0.15)"),
                    "gyro_damp_pitch": num("Direct sagittal ankle gyro rate damping (0.0..=1.0)"),
                    "gyro_damp_roll": num("Direct lateral ankle gyro rate damping (0.0..=1.0)"),
                    "arm_assist": num("Arm counter-spread balance assist gain (0.0..=1.5)"),
                    "fall_cutoff_rad": num("Protective fall tuck and torque cutoff tilt threshold in radians (0.30..=1.35)"),
                    "dcm_step_gain": num("Kinematic DCM swing-foot landing modulation gain (0.0..=1.5)"),
                    "foot_half_length_m": num("Physical printed foot half-length in meters (default 0.035 m = 7 cm foot)"),
                    "foot_margin_m": num("Usable sagittal ankle/CoP margin before capture step (default 0.028 m = ±2.8 cm)"),
                    "ukemi_tilt_rad": num("Ukemi pre-fall torque relaxation tilt threshold in radians (0.25..=1.20)"),
                    "ukemi_torque_scale": num("Ukemi torque ceiling multiplier (0.05..=0.50)"),
                    "compliance_scale": num("Global software current/torque compliance ceiling scale (0.10..=1.0)")
                }),
                &[],
            ),
            Effect::Edits,
        ),
        tool(
            "balance_set_enabled",
            "Toggle UVC Active Balance",
            "Enables or disables onboard MCU upper-body vertical control (UVC) balance.",
            object(
                json!({
                    "enabled": boolean("True to enable UVC active balance, false to disable")
                }),
                &["enabled"],
            ),
            Effect::Edits,
        ),
        tool(
            "gait_configure",
            "Configure Walking Gait Parameters",
            "Adjusts walking step clearance, cadence, and combat stance crouch.",
            object(
                json!({
                    "step_height": num("Normalized swing foot lift height (0.1..=1.0)"),
                    "cadence_hz": num("Step cadence in Hz (0.6..=2.8)"),
                    "stance_crouch": num("Nominal combat stance crouch (0.0..=1.0)")
                }),
                &[],
            ),
            Effect::Edits,
        ),
        // --- Safety & Authority Control ---
        tool(
            "mode_set",
            "Switch Control Authority Mode",
            "Switches between safe Teach mode (Torque OFF) and energized Drive mode (requires Allow Agent Drive to be armed by the operator).",
            object(
                json!({
                    "mode": enum_str("Target control mode", &["teach", "drive"])
                }),
                &["mode"],
            ),
            Effect::Edits,
        ),
        tool(
            "speed_limit_set",
            "Set Slew Speed Limit",
            "Sets the MCU joint slew-rate speed limit percentage (10..=100).",
            object(
                json!({
                    "speed_limit_pct": integer("Speed limit percent (10..=100)")
                }),
                &["speed_limit_pct"],
            ),
            Effect::Edits,
        ),
        tool(
            "safety_estop",
            "Trigger Emergency Stop",
            "Immediately latches E-Stop, forces servo torque OFF, stops walking/actions, and disarms Allow Agent Drive.",
            object(json!({}), &[]),
            Effect::Replaces,
        ),
        tool(
            "safety_reset_estop",
            "Reset E-Stop to Safe Teach Mode",
            "Clears latched E-Stop/fall faults, resets simulated torso upright, and returns strictly to Teach mode (Torque OFF).",
            object(json!({}), &[]),
            Effect::Edits,
        ),
        // --- Motion, Posing & Testing ---
        tool(
            "joint_set_target",
            "Set Single Joint Angle",
            "Commands one joint angle (radians or degrees). Works in Teach (virtual hand posing) or armed Drive mode.",
            object(
                json!({
                    "joint": joint_ref("Joint index or exact joint name"),
                    "angle_rad": num("Target angle in radians (provide angle_rad or angle_deg)"),
                    "angle_deg": num("Target angle in degrees (converted to radians)")
                }),
                &["joint"],
            ),
            Effect::Edits,
        ),
        tool(
            "joints_set_targets",
            "Set Multiple Joint Angles",
            "Applies a map of joint_name -> angle_rad targets in one call. Works in Teach or armed Drive mode.",
            object(
                json!({
                    "targets": json!({"type": "object", "description": "Map of joint name (or index string) to target angle in radians"}),
                    "units": enum_str("Angle unit: rad (default) or deg", &["rad", "deg"])
                }),
                &["targets"],
            ),
            Effect::Edits,
        ),
        tool(
            "limb_solve_ik",
            "Solve Limb Inverse Kinematics",
            "Solves multi-joint Jacobian IK to move an end-effector link (e.g. l_hand, r_hand, l_foot, r_foot) toward a 3D world position [x, y, z] in meters.",
            object(
                json!({
                    "link_name": string("End-effector link name (e.g. l_hand, r_hand, l_forearm, r_forearm, l_foot, r_foot)"),
                    "x": num("Target world X in meters"),
                    "y": num("Target world Y in meters (height above floor)"),
                    "z": num("Target world Z in meters (forward)")
                }),
                &["link_name", "x", "y", "z"],
            ),
            Effect::Edits,
        ),
        tool(
            "walk_set_velocity",
            "Command Walking Velocity",
            "Commands normalized bipedal walking velocity (vx, vy, yaw_rate in -1.0..=1.0). Enabling walking in Drive mode requires Allow Agent Drive.",
            object(
                json!({
                    "vx": num("Forward/backward speed (-1.0..=1.0)"),
                    "vy": num("Lateral strafe speed (-1.0..=1.0)"),
                    "yaw_rate": num("Turn rate (-1.0..=1.0)"),
                    "walk_enabled": boolean("Whether continuous stepping is active"),
                    "step_ticks": integer("Optional simulation ticks (0..=400) to step immediately")
                }),
                &[],
            ),
            Effect::Edits,
        ),
        tool(
            "action_play",
            "Play Combat / Motion Action",
            "Plays a named keyframe action from the robot's library. Combat actions: Guard, Left Jab, Right Cross, Front Kick, Neutral Zero. Recovery get-ups: Get Up Prone, Get Up Supine, Get Up Left Side, Get Up Right Side. Requires armed Drive mode.",
            object(
                json!({
                    "name": string("Action name (case-insensitive)"),
                    "step_ticks": integer("Optional simulation ticks (0..=400) to step immediately")
                }),
                &["name"],
            ),
            Effect::Edits,
        ),
        tool(
            "action_stop",
            "Stop Action Playback",
            "Stops any active keyframe action playback cleanly at the current measured pose.",
            object(json!({}), &[]),
            Effect::Edits,
        ),
        tool(
            "sim_push_shove",
            "Inject Combat Shove Impulse (Sim)",
            "Injects an angular velocity push impulse into the simulated torso to test UVC catch-stepping and recovery.",
            object(
                json!({
                    "pitch_impulse_rad_s": num("Forward/back pitch impulse in rad/s (-5.0..=5.0)"),
                    "roll_impulse_rad_s": num("Sideways roll impulse in rad/s (-5.0..=5.0)"),
                    "step_ticks": integer("Optional simulation ticks (0..=400) to advance after shove and return recovery telemetry")
                }),
                &["pitch_impulse_rad_s"],
            ),
            Effect::Edits,
        ),
        tool(
            "sim_inject_fault",
            "Inject Simulation Fault (Sim)",
            "Configures simulation fault injection (drop_heartbeat, voltage_sag, stall_joint, corrupt_frame) for safety testing.",
            object(
                json!({
                    "drop_heartbeat": boolean("Drop host heartbeats to test 100 ms watchdog cutoff"),
                    "voltage_sag": boolean("Simulate under-voltage battery sag"),
                    "stall_joint": joint_ref("Optional joint to stall/unstall"),
                    "stalled": boolean("Stall state for stall_joint"),
                    "corrupt_frame": boolean("Inject a malformed COBS wire frame")
                }),
                &[],
            ),
            Effect::Edits,
        ),
        tool(
            "recording_control",
            "Copper Recording & Deterministic Replay",
            "Starts or stops a Copper telemetry recording, or runs bit-exact deterministic replay verification.",
            object(
                json!({
                    "action": enum_str("Recording action", &["start", "stop", "verify_replay"])
                }),
                &["action"],
            ),
            Effect::Edits,
        ),
    ]
}

/// Static MCP resources served by `resources/list`.
pub fn resource_definitions() -> Vec<Value> {
    vec![
        json!({
            "uri": "rocky://status",
            "name": "Studio Status & Safety Policy",
            "description": "Connection status, active transport, control mode, guarded autonomy gates, active screen, and latched faults.",
            "mimeType": "application/json"
        }),
        json!({
            "uri": "rocky://telemetry",
            "name": "Live Telemetry & UVC Balance State",
            "description": "Bus voltage, peak current, max temperature, IMU pitch/roll & rates, UVC phase, support foot, foot offsets, and joint angles.",
            "mimeType": "application/json"
        }),
        json!({
            "uri": "rocky://robot",
            "name": "Active Robot Kinematics & Presets",
            "description": "Active URDF robot metadata, DoF count, up-axis, joint definitions, limb groups, and built-in presets.",
            "mimeType": "application/json"
        }),
        json!({
            "uri": "rocky://calibration",
            "name": "Joint Calibration Table",
            "description": "Per-joint zero offsets, soft limits, and servo inversion flags.",
            "mimeType": "application/json"
        }),
        json!({
            "uri": "rocky://balance",
            "name": "UVC Balance Gains & Gait Command",
            "description": "Active UVC BalanceGains and WalkCmd configuration.",
            "mimeType": "application/json"
        }),
        json!({
            "uri": "rocky://actions",
            "name": "Action & Combat Library",
            "description": "Available keyframe actions and active playback progress.",
            "mimeType": "application/json"
        }),
    ]
}

/// Resource templates served by `resources/templates/list`.
pub fn resource_template_definitions() -> Vec<Value> {
    vec![json!({
        "uriTemplate": "rocky://joints/{joint_name}",
        "name": "Single Joint State & Calibration",
        "description": "Live measured/target angle, current, temperature, soft limits, and zero offset for a specific joint.",
        "mimeType": "application/json"
    })]
}

/// Guided robotics workflow prompts served by `prompts/list`.
pub fn prompt_definitions() -> Vec<Value> {
    vec![
        json!({
            "name": "calibrate_robot",
            "title": "Calibrate Robot Joints & Soft Limits",
            "description": "Guided workflow to inspect joint limits, check zero offsets in Teach mode, and export a verified calibration JSON.",
            "arguments": [
                {"name": "preset", "description": "Optional robot preset to load first (rocky_mk1, striker_biped, combat_arm)", "required": false}
            ]
        }),
        json!({
            "name": "tune_uvc_balance",
            "title": "Tune Geometric UVC Balance Gains",
            "description": "Autonomous loop to tune UVC pitch/roll gains, deadband, and gyro damping against combat shove tests.",
            "arguments": [
                {"name": "target_shove_rad_s", "description": "Target shove impulse in rad/s (default 1.6)", "required": false}
            ]
        }),
        json!({
            "name": "verify_shove_recovery",
            "title": "Verify Shove Catch-Step & Fall Cutoff",
            "description": "Runs both an assisted UVC catch-step shove test and an unassisted fall-cutoff safety verification.",
            "arguments": []
        }),
        json!({
            "name": "diagnose_faults",
            "title": "Diagnose Robot Faults & Telemetry",
            "description": "Inspects live telemetry, voltage, servo currents, temperatures, and latched faults to recommend recovery steps.",
            "arguments": []
        }),
    ]
}
