//! Board-agnostic `#![no_std]` non-blocking IMU mailbox (`IMU_SLOT`).
//!
//! Hardware IMU tasks (such as `rk-rp2350/src/imu_bno055.rs` polling BNO055 over I2C0 @ 100 Hz)
//! publish fused Euler pitch/roll + gyro rates into `IMU_SLOT` asynchronously, so the
//! deterministic 200 Hz `copper-rs` `DirectServoCuTask` never blocks waiting on I2C transactions.

use core::cell::RefCell;

use critical_section::Mutex;
use rk_proto::ImuState;

/// Health state of the onboard hardware IMU driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ImuHealth {
    #[default]
    Uninitialized,
    Calibrating,
    Healthy,
    Fault,
}

#[derive(Clone, Copy, Debug, Default)]
struct ImuSnapshot {
    registered: bool,
    state: ImuState,
    health: ImuHealth,
    source_mask: u32,
    sample_count: u32,
    last_update_ms: u32,
}

/// Thread-safe, ISR-safe lockless-style IMU mailbox backed by `critical_section::Mutex`.
pub struct ImuSlot {
    inner: Mutex<RefCell<ImuSnapshot>>,
}

impl ImuSlot {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(ImuSnapshot {
                registered: false,
                state: ImuState {
                    available: false,
                    pitch_rad: 0.0,
                    roll_rad: 0.0,
                    yaw_rad: 0.0,
                    pitch_rate_rad_s: 0.0,
                    roll_rate_rad_s: 0.0,
                    yaw_rate_rad_s: 0.0,
                    gravity: [0.0, -9.81, 0.0],
                },
                health: ImuHealth::Uninitialized,
                source_mask: 0,
                sample_count: 0,
                last_update_ms: 0,
            })),
        }
    }

    /// Marks a hardware IMU driver as attached and updates its latest sample.
    pub fn publish(&self, state: ImuState, health: ImuHealth, now_ms: u32) {
        self.publish_with_source(
            state,
            health,
            rk_proto::estimator_valid::LSM6DSV_OK,
            now_ms,
        );
    }

    /// Publishes an IMU sample tagged with its explicit hardware source bitmask
    /// (`estimator_valid::LSM6DSV_OK` or `estimator_valid::BNO055_OK`).
    pub fn publish_with_source(
        &self,
        state: ImuState,
        health: ImuHealth,
        source_mask: u32,
        now_ms: u32,
    ) {
        critical_section::with(|cs| {
            let mut slot = self.inner.borrow_ref_mut(cs);
            slot.registered = true;
            slot.health = health;
            if state.pitch_rad.is_finite()
                && state.roll_rad.is_finite()
                && state.yaw_rad.is_finite()
                && state.pitch_rate_rad_s.is_finite()
                && state.roll_rate_rad_s.is_finite()
                && state.yaw_rate_rad_s.is_finite()
                && state.gravity.iter().all(|v| v.is_finite())
            {
                slot.state = state;
                slot.source_mask = source_mask;
                slot.sample_count = slot.sample_count.saturating_add(1);
                slot.last_update_ms = now_ms;
            }
        });
    }

    /// Marks the IMU driver faulted without overwriting the last known attitude sample.
    pub fn set_health(&self, health: ImuHealth) {
        critical_section::with(|cs| {
            let mut slot = self.inner.borrow_ref_mut(cs);
            slot.registered = true;
            slot.health = health;
        });
    }

    /// Returns the latest hardware IMU reading if a driver has published samples, or `None`
    /// in zero-hardware bench-loopback builds (allowing `ServoBusBridge` to use bus IMU data).
    pub fn latest(&self) -> Option<ImuState> {
        critical_section::with(|cs| {
            let slot = self.inner.borrow_ref(cs);
            if slot.registered && slot.sample_count > 0 {
                Some(slot.state)
            } else {
                None
            }
        })
    }

    pub fn latest_source_mask(&self) -> u32 {
        critical_section::with(|cs| {
            let slot = self.inner.borrow_ref(cs);
            if slot.registered && slot.sample_count > 0 {
                slot.source_mask
            } else {
                0
            }
        })
    }

    pub fn health(&self) -> ImuHealth {
        critical_section::with(|cs| self.inner.borrow_ref(cs).health)
    }

    pub fn sample_count(&self) -> u32 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).sample_count)
    }

    pub fn last_update_ms(&self) -> u32 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).last_update_ms)
    }
}

/// Global MCU IMU mailbox read by `ServoBusBridge::receive` on every Copper tick.
pub static IMU_SLOT: ImuSlot = ImuSlot::new();

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_slot_publishes_finite_samples_and_rejects_nan() {
        let slot = ImuSlot::new();
        assert_eq!(slot.latest(), None);
        assert_eq!(slot.health(), ImuHealth::Uninitialized);

        let valid = ImuState {
            available: true,
            pitch_rad: 0.12,
            roll_rad: -0.08,
            yaw_rad: 0.25,
            pitch_rate_rad_s: 0.4,
            roll_rate_rad_s: -0.2,
            yaw_rate_rad_s: 0.1,
            gravity: [0.1, -9.8, 0.0],
        };
        slot.publish(valid, ImuHealth::Healthy, 150);
        assert_eq!(slot.latest(), Some(valid));
        assert_eq!(slot.sample_count(), 1);
        assert_eq!(slot.last_update_ms(), 150);

        // NaN sample must not overwrite healthy reading
        slot.publish(
            ImuState {
                pitch_rad: f32::NAN,
                ..valid
            },
            ImuHealth::Fault,
            160,
        );
        assert_eq!(slot.latest(), Some(valid));
        assert_eq!(slot.health(), ImuHealth::Fault);
        assert_eq!(slot.sample_count(), 1);
    }

    #[test]
    fn imu_slot_publish_with_source_tags_bno055_legacy_imu() {
        let slot = ImuSlot::new();
        let state = ImuState {
            available: true,
            pitch_rad: 0.05,
            roll_rad: -0.03,
            yaw_rad: 0.2,
            pitch_rate_rad_s: 0.1,
            roll_rate_rad_s: 0.0,
            yaw_rate_rad_s: 0.0,
            gravity: [0.0, -9.81, 0.0],
        };
        // Default `publish()` must keep tagging LSM6DSV_OK (high-rate IMU).
        slot.publish(state, ImuHealth::Healthy, 10);
        assert_eq!(slot.latest_source_mask(), rk_proto::estimator_valid::LSM6DSV_OK);

        // Legacy BNO055 task must explicitly tag BNO055_OK, never claim LSM6DSV.
        slot.publish_with_source(
            state,
            ImuHealth::Healthy,
            rk_proto::estimator_valid::BNO055_OK,
            20,
        );
        assert_eq!(slot.latest_source_mask(), rk_proto::estimator_valid::BNO055_OK);
        assert_ne!(slot.latest_source_mask(), rk_proto::estimator_valid::LSM6DSV_OK);
    }

    #[test]
    fn imu_slot_rejects_nan_gravity_and_retains_last_good_state() {
        let slot = ImuSlot::new();
        let valid = ImuState {
            available: true,
            pitch_rad: 0.12,
            roll_rad: -0.08,
            yaw_rad: 0.25,
            pitch_rate_rad_s: 0.4,
            roll_rate_rad_s: -0.2,
            yaw_rate_rad_s: 0.1,
            gravity: [0.1, -9.8, 0.0],
        };
        slot.publish(valid, ImuHealth::Healthy, 100);
        assert_eq!(slot.latest(), Some(valid));

        // NaN gravity must not overwrite the healthy reading
        let mut bad_gravity = [0.0_f32; 3];
        bad_gravity[1] = f32::NAN;
        slot.publish(
            ImuState {
                gravity: bad_gravity,
                ..valid
            },
            ImuHealth::Fault,
            110,
        );
        assert_eq!(slot.latest(), Some(valid));
        assert_eq!(slot.sample_count(), 1);
        assert_eq!(slot.last_update_ms(), 100);
    }
}
