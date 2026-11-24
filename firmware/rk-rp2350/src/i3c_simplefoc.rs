//! RP2350 SimpleFOC I3C servo-bus adapter (`--features i3c-servo-bus`).
//!
//! The RP2350 has **no native I3C peripheral**. This module deliberately does
//! **not** reimplement a PIO I3C master: that work lives outside this repo.
//! Instead it adapts the external driver to the board-agnostic
//! [`ServoBusHooks`] slot via the [`I3cTransport`] boundary below. The external
//! PIO driver only has to implement `transaction()` and call
//! [`set_i3c_transport`] once at startup; all frame encoding, reply decoding,
//! per-joint timeout accounting and degraded-transition logging live here.
//!
//! Until a transport is registered the adapter uses a safe no-hardware stub
//! that reports a failed transaction for each joint. The control loop keeps
//! running (each joint is isolated, watchdog still fed) — no panic, no hang.
//!
//! # Embassy guideline compliance (critical-section boundary)
//!
//! The write hook is split into three phases so no hardware I/O ever happens
//! with interrupts disabled:
//!
//! 1. **Short critical section** — snapshot the command, clone the tracker and
//!    *take* the transport pointer out of the shared state.
//! 2. **Interrupts enabled** — all 21 `I3cTransport::transaction` calls, frame
//!    encoding, reply decoding, per-joint timeout accounting and the
//!    edge-triggered `degraded` warning run on the local copies.
//! 3. **Short critical section** — put the transport back, commit the tracker
//!    and publish the feedback snapshot.
//!
//! No `embassy_sync` channel/`i3c_io` task is needed: [`I3cTransport`] is a
//! synchronous trait and the hook itself is a plain `fn`, not an async task,
//! so there is no await point to hand off. The external PIO driver can remain
//! synchronous (poll/DMA-complete) as long as it does not disable interrupts;
//! if a future driver requires an await, the channel handoff and task must be
//! wired from `main.rs` (outside this module's ownership) — see the report
//! accompanying the `embassy-guidelines` change.

#![cfg(feature = "i3c-servo-bus")]

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::{info, warn};
use rk_control::{RawServoCommand, RawServoFeedback};
use rk_fw_core::config::DEFAULT_JOINT_COUNT;
use rk_fw_core::i3c_bus::{
    I3C_FB_FRAME_BYTES, I3C_JOINT_BASE_ADDR, I3cBusTracker, unpack_i3c_joint_reply,
};
use rk_fw_core::imu::IMU_SLOT;
use rk_fw_core::servo_bus::{SERVO_BUS_SLOT, ServoBusHooks};

/// Joints actually driven by this adapter (clamped to the wire-protocol maximum).
const JOINT_COUNT: usize = if DEFAULT_JOINT_COUNT < rk_proto::MAX_JOINTS {
    DEFAULT_JOINT_COUNT
} else {
    rk_proto::MAX_JOINTS
};

/// Low-level boundary the external RP2350 PIO I3C master plugs into.
///
/// `addr` is the dynamic I3C address (`I3C_JOINT_BASE_ADDR + joint_id`).
/// `tx` is the 16-byte command frame produced by `I3cBusTracker::encode_joint_cmd`.
/// `rx` must be filled with the 12-byte SimpleFOC reply frame; `Err(())` denotes
/// a timeout or CRC/nack fault for that single joint only.
pub trait I3cTransport: Send {
    fn transaction(&mut self, addr: u8, tx: &[u8], rx: &mut [u8; I3C_FB_FRAME_BYTES])
        -> Result<(), ()>;
}

/// Safe fallback used before the external PIO driver registers itself.
struct NoHardwareTransport;

impl I3cTransport for NoHardwareTransport {
    fn transaction(
        &mut self,
        _addr: u8,
        _tx: &[u8],
        _rx: &mut [u8; I3C_FB_FRAME_BYTES],
    ) -> Result<(), ()> {
        Err(())
    }
}

struct I3cState {
    tracker: I3cBusTracker,
    /// `None` until the external PIO I3C master registers itself.
    transport: Option<&'static mut (dyn I3cTransport + 'static)>,
    last_cmd: RawServoCommand,
    last_feedback: RawServoFeedback,
}

static I3C_STATE: Mutex<RefCell<I3cState>> = Mutex::new(RefCell::new(I3cState {
    tracker: I3cBusTracker::new(DEFAULT_JOINT_COUNT),
    transport: None,
    last_cmd: RawServoCommand {
        torque_mask: 0,
        raw_target_rad: [0.0_f32; rk_proto::MAX_JOINTS],
        command_mode: [rk_proto::JointCommandMode::Position.to_wire(); rk_proto::MAX_JOINTS],
        raw_velocity_rad_s: [0.0_f32; rk_proto::MAX_JOINTS],
        torque_nm: [rk_control::DEFAULT_MAX_TORQUE_NM; rk_proto::MAX_JOINTS],
    },
    last_feedback: RawServoFeedback {
        raw_angle_rad: [0.0_f32; rk_proto::MAX_JOINTS],
        velocity_rad_s: [0.0_f32; rk_proto::MAX_JOINTS],
        current_ma: [0u16; rk_proto::MAX_JOINTS],
        temp_c: [28u8; rk_proto::MAX_JOINTS],
        bus_voltage_mv: 12_200,
        bus_error: false,
        imu: rk_proto::ImuState {
            available: false,
            pitch_rad: 0.0,
            roll_rad: 0.0,
            yaw_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            roll_rate_rad_s: 0.0,
            yaw_rate_rad_s: 0.0,
            gravity: [0.0, -9.81, 0.0],
        },
        i3c_timeout_count: 0,
        imu_source_mask: 0,
    },
}));

/// Registers the external RP2350 PIO I3C master with this adapter.
///
/// Call once from board init before the `copper-rs` runtime starts. The driver
/// must have `static` lifetime (e.g. `static_cell::StaticCell<MyPioI3c>`).
#[allow(dead_code)] // Public board-integration entry point: called by the external PIO I3C driver crate.
pub fn set_i3c_transport(transport: &'static mut (dyn I3cTransport + 'static)) {
    critical_section::with(|cs| {
        I3C_STATE.borrow_ref_mut(cs).transport = Some(transport);
    });
    info!("External PIO I3C transport registered with SimpleFOC servo adapter");
}

/// Hook called by `SERVO_BUS_SLOT` every write phase.
///
/// Encodes each joint command with `I3cBusTracker::encode_joint_cmd`, performs
/// one timeout-bounded transaction **with interrupts enabled**, decodes the
/// reply with `unpack_i3c_joint_reply`, and records a per-joint timeout on any
/// failure. Only the snapshot/restore of shared state uses critical sections.
fn i3c_write_command(cmd: &RawServoCommand) {
    // ----- Phase 1: short critical section — snapshot shared state ---------
    // Clone the tracker (it is `Clone` + `Copy`-cheap arrays) so encoding,
    // sequencing and result recording can run entirely outside the lock. Take
    // the transport out so the exclusive `&mut` can be used without holding
    // `I3C_STATE.borrow_ref_mut(cs)` across any hardware access.
    let (mut tracker, mut taken_transport) = critical_section::with(|cs| {
        let mut st = I3C_STATE.borrow_ref_mut(cs);
        st.last_cmd = *cmd;
        (st.tracker.clone(), st.transport.take())
    });

    // ----- Phase 2: interrupts enabled — all hardware I/O and accounting ---
    let mut stub = NoHardwareTransport;
    let transport: &mut dyn I3cTransport = match taken_transport.as_deref_mut() {
        Some(t) => t,
        None => &mut stub,
    };

    for joint in 0..JOINT_COUNT {
        let seq = tracker.next_seq();
        let frame = tracker.encode_joint_cmd(joint, seq, cmd);
        let addr = I3C_JOINT_BASE_ADDR.saturating_add(joint as u8);
        let mut reply_buf = [0u8; I3C_FB_FRAME_BYTES];

        let was_degraded = tracker.degraded_mask();
        let tx_result = transport.transaction(addr, &frame, &mut reply_buf);
        let decoded = tx_result
            .ok()
            .and_then(|()| unpack_i3c_joint_reply(&reply_buf));

        match decoded {
            Some(decoded) => tracker.record_reply(joint, decoded),
            None => {
                tracker.record_timeout(joint);
                let now_degraded = tracker.degraded_mask();
                // Log only the transition into degraded, not every transient timeout.
                if was_degraded == 0 && now_degraded != 0 {
                    warn!(
                        "I3C servo bus degraded on joint {} (mask=0x{:08x}, timeouts={})",
                        joint,
                        now_degraded,
                        tracker.total_timeout_count()
                    );
                }
            }
        }
    }

    // Merge the async IMU mailbox into the feedback returned to the host.
    // `IMU_SLOT` self-protects with its own critical section, so this can stay
    // outside the `I3C_STATE` lock.
    let mut fb = tracker.snapshot();
    if let Some(imu) = IMU_SLOT.latest() {
        fb.imu = imu;
        fb.imu_source_mask = IMU_SLOT.latest_source_mask();
    }

    // ----- Phase 3: short critical section — commit results ----------------
    critical_section::with(|cs| {
        let mut st = I3C_STATE.borrow_ref_mut(cs);
        // Put the transport back, unless a new one was registered while the
        // transactions were in flight — a fresh registration must win.
        if let Some(t) = taken_transport {
            if st.transport.is_none() {
                st.transport = Some(t);
            }
        }
        st.tracker = tracker;
        st.last_feedback = fb;
    });
}

/// Hook called by `SERVO_BUS_SLOT` every read phase.
fn i3c_read_feedback(_dt_sec: f32) -> RawServoFeedback {
    critical_section::with(|cs| I3C_STATE.borrow_ref(cs).last_feedback)
}

/// Initializes the SimpleFOC I3C servo adapter and registers its board hooks.
///
/// Call from `main()` before starting the control loop. The hooks remain
/// active for the whole lifetime of the firmware; the external PIO driver can
/// be attached at any time via [`set_i3c_transport`].
pub fn init_i3c_servo_bus() {
    SERVO_BUS_SLOT.register_hooks(ServoBusHooks {
        read_feedback: i3c_read_feedback,
        write_command: i3c_write_command,
    });
    info!(
        "SimpleFOC I3C servo bus hooks registered ({} joints, base addr 0x{:02x})",
        DEFAULT_JOINT_COUNT, I3C_JOINT_BASE_ADDR
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hooks_are_registered_and_invoked_without_panicking() {
        // Registered fn pointers must be callable through the global slot.
        SERVO_BUS_SLOT.register_hooks(ServoBusHooks {
            read_feedback: i3c_read_feedback,
            write_command: i3c_write_command,
        });
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.5;
        SERVO_BUS_SLOT.write_command(&cmd);
        let fb = SERVO_BUS_SLOT.read_feedback(0.005);
        // With the no-hardware stub every joint times out, but faults are isolated:
        // the bus_error flag is raised once the first joint degrades and the loop
        // keeps returning feedback without panicking.
        assert!(fb.i3c_timeout_count > 0);
        assert!(fb.bus_error);
    }

    #[test]
    fn decode_round_trip_uses_core_unpacker() {
        let mut buf = [0u8; I3C_FB_FRAME_BYTES];
        buf[0..4].copy_from_slice(&0.25_f32.to_le_bytes());
        buf[4..8].copy_from_slice(&1.5_f32.to_le_bytes());
        buf[8..10].copy_from_slice(&300u16.to_le_bytes());
        buf[10] = 33;
        buf[11] = rk_fw_core::i3c_bus::simple_crc8(&buf[..11]);
        let decoded = unpack_i3c_joint_reply(&buf).unwrap();
        assert!((decoded.angle_rad - 0.25).abs() < 1e-6);
        assert!((decoded.velocity_rad_s - 1.5).abs() < 1e-6);
    }
}
