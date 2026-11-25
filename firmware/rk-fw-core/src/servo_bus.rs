//! Board-agnostic servo bus abstraction (`ServoBusPort`) + Day-1 USB bench loopback plant.
//!
//! - **Milestone 2.1 (Wired USB Bench Bring-Up):** Use [`BenchLoopbackBus`] so you can flash
//!   RP2350 or ESP32-P4, plug in a single USB cable (CDC-ACM or USB Ethernet NCM), and verify
//!   real MCU `copper-rs` 200 Hz mirroring, bumpless transfer, E-Stop, and 100 ms watchdog
//!   with zero servos attached.
//! - **Milestone 2.2 (Hardware Servo Bus):** Bind a board UART/PIO half-duplex or CAN/TWAI
//!   servo driver implementing [`ServoBusPort`] into [`SERVO_BUS_SLOT`].

use core::cell::RefCell;

use critical_section::Mutex;
use rk_control::{RawServoCommand, RawServoFeedback};
use rk_proto::{JointCommandMode, MAX_JOINTS};

use crate::config::DEFAULT_JOINT_COUNT;
use crate::joint_plant::{IDLE_CURRENT_MA, JointPlant, DEFAULT_NM_PER_AMP};

/// Hardware-independent servo bus interface called once per `copper-rs` control iteration.
pub trait ServoBusPort: Send {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback;
    fn write_command(&mut self, cmd: &RawServoCommand);
}

/// Pure `no_std` single-precision `f32` bench-test servo plant running directly on the MCU.
///
/// Uses the same [`JointPlant`] model as the simulator, so USB bench tuning transfers to
/// simulation and back. Bus voltage sag, the default (absent) IMU and bus-fault injection
/// remain bench-specific.
pub struct BenchLoopbackBus {
    joint_count: usize,
    plant: JointPlant<MAX_JOINTS>,
    last_cmd: RawServoCommand,
    bus_fault: bool,
}

impl Default for BenchLoopbackBus {
    fn default() -> Self {
        Self::new(DEFAULT_JOINT_COUNT)
    }
}

impl BenchLoopbackBus {
    pub const fn new(joint_count: usize) -> Self {
        Self {
            joint_count: if joint_count < MAX_JOINTS { joint_count } else { MAX_JOINTS },
            plant: JointPlant::new(joint_count, DEFAULT_NM_PER_AMP),
            last_cmd: RawServoCommand {
                torque_mask: 0,
                raw_target_rad: [0.0_f32; MAX_JOINTS],
                command_mode: [JointCommandMode::Position as u8; MAX_JOINTS],
                raw_velocity_rad_s: [0.0_f32; MAX_JOINTS],
                torque_nm: [1.8_f32; MAX_JOINTS],
            },
            bus_fault: false,
        }
    }
}

impl ServoBusPort for BenchLoopbackBus {
    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        let dt = dt_sec.clamp(0.001_f32, 0.05_f32);
        let mut total_ma: u32 = 0;
        let mut raw_angle_rad = [0.0_f32; MAX_JOINTS];
        let mut velocity_rad_s = [0.0_f32; MAX_JOINTS];
        let mut current_ma = [IDLE_CURRENT_MA; MAX_JOINTS];
        let mut temp_c = [28u8; MAX_JOINTS];

        for i in 0..self.joint_count.min(MAX_JOINTS) {
            let energized = (self.last_cmd.torque_mask & (1u32 << i)) != 0;
            let limit = if energized { self.last_cmd.torque_nm[i].max(0.0_f32) } else { 0.0_f32 };
            let (angle, velocity, ma, _) = self.plant.step(
                i,
                dt,
                JointCommandMode::from_wire(self.last_cmd.command_mode[i]),
                self.last_cmd.raw_target_rad[i],
                self.last_cmd.raw_velocity_rad_s[i],
                limit,
                false,
            );
            raw_angle_rad[i] = angle;
            velocity_rad_s[i] = velocity;
            current_ma[i] = ma;
            temp_c[i] = self.plant.temperature_c(i).clamp(0.0_f32, 120.0_f32) as u8;
            total_ma = total_ma.saturating_add(ma as u32);
        }

        // Mild realistic voltage sag under load from 12.20 V nominal 3S LiPo
        let sag_mv = (total_ma / 18).min(900) as u16;
        let bus_voltage_mv = 12_200u16.saturating_sub(sag_mv);

        RawServoFeedback {
            raw_angle_rad,
            velocity_rad_s,
            current_ma,
            temp_c,
            bus_voltage_mv,
            bus_error: self.bus_fault,
            imu: rk_proto::ImuState::default(),
            i3c_timeout_count: 0,
            imu_source_mask: 0,
        }
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        self.last_cmd = *cmd;
    }
}

/// Static servo bus registration slot consumed by `copper-rs` `ServoBusBridge`.
///
/// Boards install either [`BenchLoopbackBus`] or a hardware UART/PIO/TWAI driver before
/// starting the `copper-rs` runtime, keeping `ServoBusBridge` 100% portable across MCUs.
pub struct ServoBusSlot {
    /// Active fallback bench bus (used when no custom hardware callback is registered).
    bench: Mutex<RefCell<BenchLoopbackBus>>,
    /// Optional board hardware hooks `(read_fn, write_fn)`.
    hooks: Mutex<RefCell<Option<ServoBusHooks>>>,
}

#[derive(Clone, Copy)]
pub struct ServoBusHooks {
    pub read_feedback: fn(f32) -> RawServoFeedback,
    pub write_command: fn(&RawServoCommand),
}

impl ServoBusSlot {
    pub const fn new() -> Self {
        Self {
            bench: Mutex::new(RefCell::new(BenchLoopbackBus::new(DEFAULT_JOINT_COUNT))),
            hooks: Mutex::new(RefCell::new(None)),
        }
    }

    /// Registers board-specific hardware UART/PIO/TWAI servo bus hooks.
    pub fn register_hooks(&self, hooks: ServoBusHooks) {
        critical_section::with(|cs| {
            *self.hooks.borrow_ref_mut(cs) = Some(hooks);
        });
    }

    /// Clears custom hardware hooks and reverts to the built-in `BenchLoopbackBus`.
    pub fn use_bench_loopback(&self) {
        critical_section::with(|cs| {
            *self.hooks.borrow_ref_mut(cs) = None;
        });
    }

    pub fn read_feedback(&self, dt_sec: f32) -> RawServoFeedback {
        let maybe_fn = critical_section::with(|cs| self.hooks.borrow_ref(cs).map(|h| h.read_feedback));
        if let Some(read_fn) = maybe_fn {
            read_fn(dt_sec)
        } else {
            critical_section::with(|cs| self.bench.borrow_ref_mut(cs).read_feedback(dt_sec))
        }
    }

    pub fn write_command(&self, cmd: &RawServoCommand) {
        let maybe_fn = critical_section::with(|cs| self.hooks.borrow_ref(cs).map(|h| h.write_command));
        if let Some(write_fn) = maybe_fn {
            write_fn(cmd);
        } else {
            critical_section::with(|cs| self.bench.borrow_ref_mut(cs).write_command(cmd));
        }
    }
}

impl Default for ServoBusSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// Global servo bus bridge slot shared between board BSP initialization and `copper-rs`.
pub static SERVO_BUS_SLOT: ServoBusSlot = ServoBusSlot::new();

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bench_loopback_tracks_when_energized_and_holds_when_off() {
        let mut bus = BenchLoopbackBus::new(17);
        let mut cmd = RawServoCommand::default();
        cmd.torque_mask = 1 << 3;
        cmd.raw_target_rad[3] = 1.0_f32;
        bus.write_command(&cmd);

        for _ in 0..40 {
            let _ = bus.read_feedback(0.005);
        }
        let fb_on = bus.read_feedback(0.005);
        assert!(fb_on.raw_angle_rad[3] > 0.85, "energized joint should converge");
        assert!(fb_on.current_ma[3] >= 120);

        // Cut torque -> angle should freeze and current drop to idle
        cmd.torque_mask = 0;
        cmd.raw_target_rad[3] = -1.0_f32;
        bus.write_command(&cmd);
        let fb_off = bus.read_feedback(0.005);
        assert!((fb_off.raw_angle_rad[3] - fb_on.raw_angle_rad[3]).abs() < 1e-6);
        assert_eq!(fb_off.current_ma[3], 45);
    }
}
