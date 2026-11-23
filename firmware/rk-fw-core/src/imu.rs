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
                    pitch_rate_rad_s: 0.0,
                    roll_rate_rad_s: 0.0,
                },
                health: ImuHealth::Uninitialized,
                sample_count: 0,
                last_update_ms: 0,
            })),
        }
    }

    /// Marks a hardware IMU driver as attached and updates its latest sample.
    pub fn publish(&self, state: ImuState, health: ImuHealth, now_ms: u32) {
        critical_section::with(|cs| {
            let mut slot = self.inner.borrow_ref_mut(cs);
            slot.registered = true;
            slot.health = health;
            if state.pitch_rad.is_finite()
                && state.roll_rad.is_finite()
                && state.pitch_rate_rad_s.is_finite()
                && state.roll_rate_rad_s.is_finite()
            {
                slot.state = state;
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
            pitch_rate_rad_s: 0.4,
            roll_rate_rad_s: -0.2,
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
}
