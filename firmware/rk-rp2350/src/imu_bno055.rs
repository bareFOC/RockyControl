//! Async RP2350 Bosch BNO055 9-Axis IMU Driver (`--features imu-bno055`).
//!
//! Hardware wiring (Raspberry Pi Pico 2 / RP2350 `I2C0`):
//! - `GPIO4`: SDA (I2C0)
//! - `GPIO5`: SCL (I2C0)
//! - 400 kHz Fast-Mode I2C, 7-bit device address `0x28` (COM3/ADR low)
//!
//! Register initialization sequence matches ai2001 `main.c`:
//! 1. Verify `CHIP_ID` (`0x00`) == `0xA0`
//! 2. Switch to `CONFIG` mode (`OPR_MODE` `0x3D` = `0x00`)
//! 3. Software reset (`SYS_TRIGGER` `0x3F` = `0x20`) and wait for chip ID ready
//! 4. Normal power mode (`PWR_MODE` `0x3E` = `0x00`), Page 0 (`PAGE_ID` `0x07` = `0x00`)
//! 5. Enable external 32.768 kHz crystal (`SYS_TRIGGER` `0x3F` = `0x80`)
//! 6. Engage NDOF 9-axis fusion (`OPR_MODE` `0x3D` = `0x0C`)
//!
//! Runs as an independent Embassy async task @ 100 Hz (`10 ms`) publishing fused Euler
//! pitch/roll and gyro angular rates into `rk_fw_core::imu::IMU_SLOT` so the deterministic
//! 200 Hz `copper-rs` servo loop never blocks on I2C transactions.

#![cfg(feature = "imu-bno055")]

use core::f32::consts::PI;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::i2c::{Async, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{I2C0, PIN_4, PIN_5};
use embassy_time::{Duration, Instant, Ticker, Timer};
use rk_fw_core::imu::{IMU_SLOT, ImuHealth};
use rk_proto::{ImuState, estimator_valid};

const BNO055_ADDR: u8 = 0x28;
const REG_CHIP_ID: u8 = 0x00;
const REG_PAGE_ID: u8 = 0x07;
const REG_GYR_DATA_X_LSB: u8 = 0x14;
const REG_EUL_HEADING_LSB: u8 = 0x1A;
/// Gravity vector (page 0, 6 bytes, 1 LSB = 0.01 m/s²).
const REG_GRAV_DATA_X_LSB: u8 = 0x2E;
/// Quaternion data fallback (page 0, 8 bytes) when gravity registers are unavailable.
const REG_QUA_DATA_W_LSB: u8 = 0x20;
const REG_OPR_MODE: u8 = 0x3D;
const REG_PWR_MODE: u8 = 0x3E;
const REG_SYS_TRIGGER: u8 = 0x3F;

const BNO055_CHIP_ID: u8 = 0xA0;
const MODE_CONFIG: u8 = 0x00;
const MODE_NDOF: u8 = 0x0C;

/// 16 LSB / deg -> radians per LSB = PI / (180 * 16) = PI / 2880.
const RAD_PER_LSB: f32 = PI / 2880.0_f32;

/// BNO055 gravity registers: 1 LSB = 0.01 m/s².
const GRAVITY_MPS2_PER_LSB: f32 = 0.01_f32;
/// Nominal gravity magnitude (m/s²) used by the fallback derivation.
const GRAVITY_NOMINAL_MPS2: f32 = 9.81_f32;
/// BNO055 quaternion registers use Q14 (1 LSB = 1 / 16384).
const QUAT_PER_LSB: f32 = 1.0_f32 / 16384.0_f32;

/// Converts a little-endian pair of BNO055 gravity register bytes to m/s².
fn decode_gravity_axis(bytes: [u8; 2]) -> f32 {
    (i16::from_le_bytes(bytes) as f32) * GRAVITY_MPS2_PER_LSB
}

/// Derives the gravity vector from a BNO055 quaternion by rotating world-down
/// `(0, -g, 0)` by the inverse of the orientation quaternion.
///
/// Uses the standard quaternion-vector rotation `v' = v + 2 * w * (q_v x v) + 2 * (q_v x (q_v x v))`
/// with the conjugate (inverse) quaternion `(w, -x, -y, -z)` so a level sensor
/// reports approximately `[0, -9.81, 0]`.
fn gravity_from_quaternion(w: f32, x: f32, y: f32, z: f32) -> [f32; 3] {
    // Conjugate (inverse for a unit quaternion): (w, -x, -y, -z).
    // Do NOT negate w as well: q and -q encode the same rotation, so that would
    // leave the rotation unchanged instead of inverting it.
    let (qw, qx, qy, qz) = (w, -x, -y, -z);
    let (vx, vy, vz) = (0.0_f32, -GRAVITY_NOMINAL_MPS2, 0.0_f32);

    // t = 2 * (q_v x v)
    let tx = 2.0_f32 * (qy * vz - qz * vy);
    let ty = 2.0_f32 * (qz * vx - qx * vz);
    let tz = 2.0_f32 * (qx * vy - qy * vx);

    // v' = v + qw * t + (q_v x t)
    [
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    ]
}

/// Wraps an angle into `[-PI, PI]` for the polynomial trig helpers below.
fn wrap_pi(x: f32) -> f32 {
    const TAU: f32 = 2.0_f32 * PI;
    // `floor` without libm: i32 cast truncates toward zero, correct down for negatives.
    let t = x / TAU + 0.5_f32;
    let n = t as i32;
    let n = if (n as f32) > t { n - 1 } else { n };
    x - TAU * (n as f32)
}

/// Deterministic `no_std` sine approximation (degree-7 Taylor on `[-PI/2, PI/2]`).
fn sin_approx(x: f32) -> f32 {
    let a = wrap_pi(x);
    let b = if a > PI / 2.0_f32 {
        PI - a
    } else if a < -PI / 2.0_f32 {
        -PI - a
    } else {
        a
    };
    let b2 = b * b;
    b * (1.0_f32
        - b2 / 6.0_f32 * (1.0_f32 - b2 / 20.0_f32 * (1.0_f32 - b2 / 42.0_f32)))
}

/// Deterministic `no_std` cosine approximation (degree-8 Taylor on `[-PI/2, PI/2]`).
fn cos_approx(x: f32) -> f32 {
    let a = wrap_pi(x);
    let (b, sign) = if a > PI / 2.0_f32 {
        (PI - a, -1.0_f32)
    } else if a < -PI / 2.0_f32 {
        (-PI - a, -1.0_f32)
    } else {
        (a, 1.0_f32)
    };
    let b2 = b * b;
    sign * (1.0_f32
        - b2 / 2.0_f32
            * (1.0_f32
                - b2 / 12.0_f32
                    * (1.0_f32 - b2 / 30.0_f32 * (1.0_f32 - b2 / 56.0_f32))))
}

/// Derives a gravity vector from Euler pitch/roll using the same convention as
/// the kinematics tilt (`pitch` about +X, `-roll` about -Z). This is the last
/// resort when neither the gravity registers nor the quaternion are readable.
///
/// The body-frame gravity is `tilt_rot^-1 * (0, -g, 0)` where
/// `tilt_rot = R_x(pitch) * R_z(-roll)`, hence `R_z(roll) * R_x(-pitch)`.
fn gravity_from_euler(pitch_rad: f32, roll_rad: f32) -> [f32; 3] {
    let sp = sin_approx(-pitch_rad);
    let cp = cos_approx(-pitch_rad);
    // R_x(-pitch) applied to (0, -g, 0).
    let x1 = 0.0_f32;
    let y1 = -GRAVITY_NOMINAL_MPS2 * cp;
    let z1 = -GRAVITY_NOMINAL_MPS2 * sp;
    // R_z(roll) applied to the result.
    let sr = sin_approx(roll_rad);
    let cr = cos_approx(roll_rad);
    [
        x1 * cr - y1 * sr,
        x1 * sr + y1 * cr,
        z1,
    ]
}

/// Spawns the non-blocking 100 Hz BNO055 I2C0 sensor task on the RP2350 Embassy executor.
pub fn spawn_bno055(
    spawner: &Spawner,
    i2c0: Peri<'static, I2C0>,
    scl: Peri<'static, PIN_5>,
    sda: Peri<'static, PIN_4>,
    irqs: impl embassy_rp::interrupt::typelevel::Binding<
        embassy_rp::interrupt::typelevel::I2C0_IRQ,
        I2cInterruptHandler<I2C0>,
    > + 'static,
) {
    let mut cfg = I2cConfig::default();
    cfg.frequency = 400_000;
    let i2c = I2c::new_async(i2c0, scl, sda, irqs, cfg);
    spawner.spawn(bno055_task(i2c).expect("spawn bno055_task"));
    info!("BNO055 I2C0 driver spawned @ 400 kHz (GPIO4 SDA / GPIO5 SCL, addr 0x28)");
}

async fn init_bno055(i2c: &mut I2c<'static, I2C0, Async>) -> bool {
    let mut chip_id = [0u8; 1];
    if i2c
        .write_read_async(BNO055_ADDR, [REG_CHIP_ID], &mut chip_id)
        .await
        .is_err()
        || chip_id[0] != BNO055_CHIP_ID
    {
        return false;
    }

    // Switch to CONFIG mode before reset
    if i2c
        .write_async(BNO055_ADDR, [REG_OPR_MODE, MODE_CONFIG])
        .await
        .is_err()
    {
        return false;
    }
    Timer::after(Duration::from_millis(25)).await;

    // Trigger soft reset and wait for boot
    let _ = i2c.write_async(BNO055_ADDR, [REG_SYS_TRIGGER, 0x20]).await;
    Timer::after(Duration::from_millis(650)).await;

    let mut ready = false;
    for _ in 0..15 {
        if i2c
            .write_read_async(BNO055_ADDR, [REG_CHIP_ID], &mut chip_id)
            .await
            .is_ok()
            && chip_id[0] == BNO055_CHIP_ID
        {
            ready = true;
            break;
        }
        Timer::after(Duration::from_millis(20)).await;
    }
    if !ready {
        return false;
    }

    // Normal power, Page 0, External crystal, NDOF fusion
    if i2c
        .write_async(BNO055_ADDR, [REG_PWR_MODE, 0x00])
        .await
        .is_err()
    {
        return false;
    }
    Timer::after(Duration::from_millis(10)).await;
    if i2c
        .write_async(BNO055_ADDR, [REG_PAGE_ID, 0x00])
        .await
        .is_err()
    {
        return false;
    }
    if i2c
        .write_async(BNO055_ADDR, [REG_SYS_TRIGGER, 0x80])
        .await
        .is_err()
    {
        return false;
    }
    Timer::after(Duration::from_millis(20)).await;
    if i2c
        .write_async(BNO055_ADDR, [REG_OPR_MODE, MODE_NDOF])
        .await
        .is_err()
    {
        return false;
    }
    Timer::after(Duration::from_millis(30)).await;
    true
}

#[embassy_executor::task]
async fn bno055_task(mut i2c: I2c<'static, I2C0, Async>) {
    IMU_SLOT.set_health(ImuHealth::Calibrating);

    // Retry sensor bring-up without ever panicking or stalling the servo loop
    while !init_bno055(&mut i2c).await {
        IMU_SLOT.set_health(ImuHealth::Fault);
        warn!("BNO055 not responding on I2C0 (0x28); retrying in 250 ms");
        Timer::after(Duration::from_millis(250)).await;
    }
    info!("BNO055 NDOF mode active; streaming Euler + Gyro @ 100 Hz");

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut eul_buf = [0u8; 6];
    let mut gyr_buf = [0u8; 6];
    let mut grav_buf = [0u8; 6];
    let mut quat_buf = [0u8; 8];
    let mut err_streak: u8 = 0;
    let mut gravity_fallback_logged = false;

    loop {
        ticker.next().await;

        let eul_ok = i2c
            .write_read_async(BNO055_ADDR, [REG_EUL_HEADING_LSB], &mut eul_buf)
            .await
            .is_ok();
        let gyr_ok = i2c
            .write_read_async(BNO055_ADDR, [REG_GYR_DATA_X_LSB], &mut gyr_buf)
            .await
            .is_ok();

        if eul_ok && gyr_ok {
            err_streak = 0;
            // BNO055 Euler registers (0x1A..0x1F): [Heading, Roll, Pitch] little-endian i16
            let heading_raw = i16::from_le_bytes([eul_buf[0], eul_buf[1]]);
            let roll_raw = i16::from_le_bytes([eul_buf[2], eul_buf[3]]);
            let pitch_raw = i16::from_le_bytes([eul_buf[4], eul_buf[5]]);

            // BNO055 Gyro registers (0x14..0x19): [X=pitch_rate, Y=roll_rate, Z=yaw_rate]
            let gyr_x_raw = i16::from_le_bytes([gyr_buf[0], gyr_buf[1]]);
            let gyr_y_raw = i16::from_le_bytes([gyr_buf[2], gyr_buf[3]]);
            let gyr_z_raw = i16::from_le_bytes([gyr_buf[4], gyr_buf[5]]);

            let pitch_rad = (pitch_raw as f32) * RAD_PER_LSB;
            let roll_rad = (roll_raw as f32) * RAD_PER_LSB;
            let yaw_rad = (heading_raw as f32) * RAD_PER_LSB;

            // Preferred: dedicated gravity vector registers (page 0, 0x2E..0x33).
            let gravity_read = i2c
                .write_read_async(BNO055_ADDR, [REG_GRAV_DATA_X_LSB], &mut grav_buf)
                .await
                .is_ok();
            let gravity = if gravity_read {
                gravity_fallback_logged = false;
                [
                    decode_gravity_axis([grav_buf[0], grav_buf[1]]),
                    decode_gravity_axis([grav_buf[2], grav_buf[3]]),
                    decode_gravity_axis([grav_buf[4], grav_buf[5]]),
                ]
            } else {
                // Risk mitigation: fall back to quaternion (0x20) then Euler.
                let quat_read = i2c
                    .write_read_async(BNO055_ADDR, [REG_QUA_DATA_W_LSB], &mut quat_buf)
                    .await
                    .is_ok();
                if quat_read {
                    let qw = (i16::from_le_bytes([quat_buf[0], quat_buf[1]]) as f32) * QUAT_PER_LSB;
                    let qx = (i16::from_le_bytes([quat_buf[2], quat_buf[3]]) as f32) * QUAT_PER_LSB;
                    let qy = (i16::from_le_bytes([quat_buf[4], quat_buf[5]]) as f32) * QUAT_PER_LSB;
                    let qz = (i16::from_le_bytes([quat_buf[6], quat_buf[7]]) as f32) * QUAT_PER_LSB;
                    gravity_from_quaternion(qw, qx, qy, qz)
                } else {
                    gravity_from_euler(pitch_rad, roll_rad)
                }
            };

            if !gravity_read && !gravity_fallback_logged {
                warn!("BNO055 gravity registers unavailable; deriving gravity from fusion data");
                gravity_fallback_logged = true;
            }

            let state = ImuState {
                available: true,
                pitch_rad,
                roll_rad,
                yaw_rad,
                pitch_rate_rad_s: (gyr_x_raw as f32) * RAD_PER_LSB,
                roll_rate_rad_s: (gyr_y_raw as f32) * RAD_PER_LSB,
                yaw_rate_rad_s: (gyr_z_raw as f32) * RAD_PER_LSB,
                gravity,
            };
            let now_ms = Instant::now().as_millis() as u32;
            IMU_SLOT.publish_with_source(
                state,
                ImuHealth::Healthy,
                now_ms,
                estimator_valid::BNO055_OK,
            );
        } else {
            err_streak = err_streak.saturating_add(1);
            IMU_SLOT.set_health(ImuHealth::Fault);
            if err_streak >= 20 {
                warn!("BNO055 I2C fault streak; re-initializing sensor");
                let _ = init_bno055(&mut i2c).await;
                err_streak = 0;
            }
        }
    }
}
