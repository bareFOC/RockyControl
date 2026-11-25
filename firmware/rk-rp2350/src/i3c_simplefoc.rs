//! RP2350 BareFOC multibus servo adapter (`--features i3c-servo-bus`).
//!
//! The RP2350 has **no native I3C peripheral**. This module deliberately does
//! **not** reimplement a PIO I3C master: that work lives outside this repo.
//! Instead it adapts the external driver to the board-agnostic
//! [`ServoBusHooks`] slot via the [`rk_multibus::PioI3c`] boundary. The external
//! PIO driver only has to implement `transaction()` and call
//! [`set_i3c_transport`] once at startup; register encoding/decoding,
//! per-joint timeout accounting and degraded-transition logging all use the
//! shared `rk-multibus` code path (the same one the simulator exercises).
//!
//! Servo commands are written as BareFOC register-pointer transactions
//! (`0x120` torque enable, `0xE0` operating mode, `0x130/0x134/0x138` goals),
//! one node per joint at `0x20 + joint`. Feedback is a single 14-byte burst
//! read from `0x142` (temperature + position + velocity + current).
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
//! 2. **Interrupts enabled** — all 21 joints' register transactions (command
//!    writes + feedback burst reads), accounting and the edge-triggered
//!    `degraded` warning run on the local copies.
//! 3. **Short critical section** — put the transport back, commit the tracker
//!    and publish the feedback snapshot.
//!
//! No `embassy_sync` channel/`i3c_io` task is needed: [`PioI3c`] is a
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
use rk_fw_core::imu::IMU_SLOT;
use rk_fw_core::servo_bus::{SERVO_BUS_SLOT, ServoBusHooks};
use rk_multibus::{
    I2cTransport, JOINT_NODE_BASE, JointFeedback, MAX_I2C_WRITE_FRAME, MultibusBusTracker,
    PioError, PioI3c, PioI3cBus, TorqueConstants, bind_joint, read_joint_feedback,
    write_joint_command,
};

/// Joints actually driven by this adapter (one BareFOC node per joint).
/// Clamped to the node address space `0x20..=0x34` served by `rk-multibus`.
const JOINT_COUNT: usize = {
    let width = if DEFAULT_JOINT_COUNT < rk_proto::MAX_JOINTS {
        DEFAULT_JOINT_COUNT
    } else {
        rk_proto::MAX_JOINTS
    };
    if width < rk_multibus::MAX_SERVO_JOINTS {
        width
    } else {
        rk_multibus::MAX_SERVO_JOINTS
    }
};

/// Adapts the registered `&mut dyn PioI3c` to the owned-transport form expected
/// by [`PioI3cBus`]. The bus keeps the register-pointer latch alive across the
/// two HAL transactions of a burst read, exactly as on hardware.
struct PioRef<'a, P: PioI3c + ?Sized>(&'a mut P);

impl<P: PioI3c + ?Sized> PioI3c for PioRef<'_, P> {
    fn transaction(&mut self, address: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), PioError> {
        self.0.transaction(address, tx, rx)
    }
}

/// Safe fallback used before the external PIO driver registers itself.
struct NoHardwareTransport;

impl PioI3c for NoHardwareTransport {
    fn transaction(&mut self, _address: u8, _tx: &[u8], _rx: &mut [u8]) -> Result<(), PioError> {
        Err(PioError::NoAcknowledge)
    }
}

struct I3cState {
    tracker: MultibusBusTracker,
    /// `None` until the external PIO I3C master registers itself.
    /// `Send` is required because this state lives in a `critical_section::Mutex`
    /// static (the old board-local trait had the same bound).
    transport: Option<&'static mut (dyn PioI3c + Send + 'static)>,
    last_cmd: RawServoCommand,
    last_feedback: RawServoFeedback,
}

static I3C_STATE: Mutex<RefCell<I3cState>> = Mutex::new(RefCell::new(I3cState {
    tracker: MultibusBusTracker::new(DEFAULT_JOINT_COUNT),
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
/// Call once from board init before the Embassy runtime starts. The driver
/// must have `static` lifetime (e.g. `static_cell::StaticCell<MyPioI3c>`).
#[allow(dead_code)] // Public board-integration entry point: called by the external PIO I3C driver crate.
pub fn set_i3c_transport(transport: &'static mut (dyn PioI3c + Send + 'static)) {
    critical_section::with(|cs| {
        I3C_STATE.borrow_ref_mut(cs).transport = Some(transport);
    });
    info!("External PIO I3C transport registered with BareFOC multibus servo adapter");
}

/// Performs all register transactions for one control tick.
///
/// This is deliberately independent of the global hook state so it can be
/// exercised directly by tests with a loopback PIO driver. Every joint gets
/// its own `ControlTableClient` bound to node `0x20 + joint`; a failed joint is
/// counted and skipped without stalling the others.
fn exchange_joints<P: PioI3c + ?Sized>(
    pio: &mut P,
    cmd: &RawServoCommand,
    tracker: &mut MultibusBusTracker,
    constants: &TorqueConstants,
) {
    let mut bus = PioI3cBus::new(PioRef(pio));

    for joint in 0..JOINT_COUNT {
        let _seq = tracker.next_seq();
        let Some(mut client) =
            bind_joint(joint, I2cTransport::<_, MAX_I2C_WRITE_FRAME>::new(&mut bus))
        else {
            continue;
        };

        // Five register writes (torque, mode, position, velocity, current)
        // through the exact wire encoding used by firmware and simulator.
        let write_result = write_joint_command(&mut client, joint, cmd, constants);

        // `read_joint_feedback` is the single mapper from burst bytes into
        // `RawServoFeedback`. The tracker owns the published snapshot, so copy
        // this joint's decoded values back into a `JointFeedback` for it (the
        // final mA / whole-degree units are unchanged by the round trip).
        let mut decoded = RawServoFeedback::default();
        let read_result = read_joint_feedback(&mut client, joint, &mut decoded);
        // Release the bus borrow before the next joint binds its client.
        drop(client);

        let was_degraded = tracker.degraded_mask();
        if write_result.is_err() || read_result.is_err() {
            tracker.record_timeout(joint);
        } else {
            tracker.record_reply(
                joint,
                JointFeedback {
                    position_rad: decoded.raw_angle_rad[joint],
                    velocity_rad_s: decoded.velocity_rad_s[joint],
                    current_amps: (decoded.current_ma[joint] as f32) * 0.001_f32,
                    temperature_c: decoded.temp_c[joint] as f32,
                },
            );
        }

        // Log only the transition into degraded, not every transient timeout.
        if was_degraded == 0 && tracker.degraded_mask() != 0 {
            warn!(
                "I3C servo bus degraded on joint {} (mask=0x{:08x}, timeouts={})",
                joint,
                tracker.degraded_mask(),
                tracker.total_timeout_count()
            );
        }
    }
}

/// Hook called by `SERVO_BUS_SLOT` every write phase.
fn i3c_write_command(cmd: &RawServoCommand) {
    // ----- Phase 1: short critical section — snapshot shared state ---------
    // Clone the tracker so accounting can run entirely outside the lock. Take
    // the transport out so the exclusive `&mut` is not held across any I/O.
    let (mut tracker, mut taken_transport) = critical_section::with(|cs| {
        let mut st = I3C_STATE.borrow_ref_mut(cs);
        st.last_cmd = *cmd;
        (st.tracker.clone(), st.transport.take())
    });

    // ----- Phase 2: interrupts enabled — all hardware I/O and accounting ---
    let mut stub = NoHardwareTransport;
    let pio: &mut dyn PioI3c = match taken_transport.as_deref_mut() {
        Some(t) => t,
        None => &mut stub,
    };
    exchange_joints(pio, cmd, &mut tracker, &TorqueConstants::default());

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

/// Initializes the BareFOC multibus servo adapter and registers its board hooks.
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
        "BareFOC multibus servo hooks registered ({} joints, node base 0x{:02x})",
        JOINT_COUNT, JOINT_NODE_BASE
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use rk_multibus::{
        CONTROL_TABLE_SIZE, FEEDBACK_BURST_LEN, FEEDBACK_BURST_REG, REG_GOAL_CURRENT,
        REG_GOAL_POSITION, REG_GOAL_VELOCITY, REG_OPERATING_MODE, REG_TORQUE_ENABLE,
        joint_map::operating_mode_for,
    };

    #[test]
    fn hooks_return_isolated_bus_errors_without_panicking() {
        // Registered fn pointers must be callable through the global slot.
        SERVO_BUS_SLOT.register_hooks(ServoBusHooks {
            read_feedback: i3c_read_feedback,
            write_command: i3c_write_command,
        });
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1;
        cmd.raw_target_rad[0] = 0.5;
        SERVO_BUS_SLOT.write_command(&cmd);
        let first = SERVO_BUS_SLOT.read_feedback(0.005);
        // With the no-hardware stub every joint times out, but faults are
        // isolated: a single miss does not yet degrade the bus.
        assert_eq!(first.i3c_timeout_count, JOINT_COUNT as u16);
        assert!(!first.bus_error);

        // Three consecutive misses degrade the joints, but the loop still
        // returns feedback without panicking.
        SERVO_BUS_SLOT.write_command(&cmd);
        SERVO_BUS_SLOT.write_command(&cmd);
        let degraded = SERVO_BUS_SLOT.read_feedback(0.005);
        assert_eq!(degraded.i3c_timeout_count, 3 * JOINT_COUNT as u16);
        assert!(degraded.bus_error);
    }

    #[test]
    fn no_hardware_stub_times_out_every_joint_but_stays_isolated() {
        let mut tracker = MultibusBusTracker::new(JOINT_COUNT);
        let cmd = RawServoCommand::default();
        let constants = TorqueConstants::default();

        exchange_joints(&mut NoHardwareTransport, &cmd, &mut tracker, &constants);
        assert_eq!(tracker.total_timeout_count(), JOINT_COUNT as u16);
        for joint in 0..JOINT_COUNT {
            assert_eq!(tracker.consecutive_timeouts(joint), 1);
        }
        assert_eq!(tracker.degraded_mask(), 0);
        assert_eq!(tracker.snapshot().i3c_timeout_count, JOINT_COUNT as u16);
        assert!(!tracker.snapshot().bus_error);

        for _ in 0..2 {
            exchange_joints(&mut NoHardwareTransport, &cmd, &mut tracker, &constants);
        }
        assert_eq!(tracker.total_timeout_count(), 3 * JOINT_COUNT as u16);
        assert_eq!(tracker.degraded_mask(), (1u32 << JOINT_COUNT) - 1);
        assert!(tracker.snapshot().bus_error);
    }

    /// Minimal in-memory BareFOC register server: one 512-byte control table
    /// per node, pointer latch on writes, burst reply on reads — identical to
    /// the simulator's PIO boundary.
    struct LoopbackPio {
        banks: [[u8; CONTROL_TABLE_SIZE]; JOINT_COUNT],
    }

    impl PioI3c for LoopbackPio {
        fn transaction(&mut self, address: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), PioError> {
            let node = address.checked_sub(JOINT_NODE_BASE).ok_or(PioError::Bus)? as usize;
            let regs = self.banks.get_mut(node).ok_or(PioError::Bus)?;
            if tx.len() < 2 {
                return Err(PioError::Bus);
            }
            let reg = u16::from_be_bytes([tx[0], tx[1]]) as usize;
            if tx.len() > 2 {
                let end = reg.checked_add(tx.len() - 2).ok_or(PioError::Bus)?;
                if end > regs.len() {
                    return Err(PioError::Bus);
                }
                regs[reg..end].copy_from_slice(&tx[2..]);
            } else {
                let end = reg.checked_add(rx.len()).ok_or(PioError::Bus)?;
                if end > regs.len() {
                    return Err(PioError::Bus);
                }
                rx.copy_from_slice(&regs[reg..end]);
            }
            Ok(())
        }
    }

    #[test]
    fn wire_commands_reach_registers_and_burst_feedback_is_mapped() {
        let mut pio = LoopbackPio {
            banks: [[0u8; CONTROL_TABLE_SIZE]; JOINT_COUNT],
        };
        let sample = JointFeedback {
            position_rad: -0.5,
            velocity_rad_s: 1.25,
            current_amps: 0.42,
            temperature_c: 39.0,
        };
        let burst_start = FEEDBACK_BURST_REG as usize;
        pio.banks[0][burst_start..burst_start + FEEDBACK_BURST_LEN]
            .copy_from_slice(&sample.to_burst());

        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1 << 4;
        cmd.command_mode[4] = rk_proto::JointCommandMode::Velocity.to_wire();
        cmd.raw_target_rad[4] = 0.75;
        cmd.raw_velocity_rad_s[4] = 2.5;
        cmd.torque_nm[4] = 1.8; // 2.0 A at the default 0.9 N·m/A constant

        let mut tracker = MultibusBusTracker::new(JOINT_COUNT);
        exchange_joints(&mut pio, &cmd, &mut tracker, &TorqueConstants::default());

        assert_eq!(tracker.total_timeout_count(), 0);
        assert!(!tracker.snapshot().bus_error);
        // Joint 4's node (0x24) received the commanded registers; other nodes
        // keep their own tables, proving node-per-joint addressing on the wire.
        assert_eq!(pio.banks[4][REG_TORQUE_ENABLE as usize], 1);
        assert_eq!(pio.banks[0][REG_TORQUE_ENABLE as usize], 0);
        assert_eq!(
            pio.banks[4][REG_OPERATING_MODE as usize],
            operating_mode_for(rk_proto::JointCommandMode::Velocity) as u8
        );
        assert_eq!(
            &pio.banks[4][REG_GOAL_POSITION as usize..REG_GOAL_POSITION as usize + 4],
            &0.75_f32.to_le_bytes()
        );
        assert_eq!(
            &pio.banks[4][REG_GOAL_VELOCITY as usize..REG_GOAL_VELOCITY as usize + 4],
            &2.5_f32.to_le_bytes()
        );
        assert_eq!(
            &pio.banks[4][REG_GOAL_CURRENT as usize..REG_GOAL_CURRENT as usize + 4],
            &2.0_f32.to_le_bytes()
        );

        let fb = tracker.snapshot();
        assert!((fb.raw_angle_rad[0] + 0.5).abs() < 1e-6);
        assert!((fb.velocity_rad_s[0] - 1.25).abs() < 1e-6);
        assert_eq!(fb.current_ma[0], 420);
        assert_eq!(fb.temp_c[0], 39);
    }
}
