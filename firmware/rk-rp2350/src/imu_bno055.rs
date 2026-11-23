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
use rk_proto::ImuState;

const BNO055_ADDR: u8 = 0x28;
const REG_CHIP_ID: u8 = 0x00;
const REG_PAGE_ID: u8 = 0x07;
const REG_GYR_DATA_X_LSB: u8 = 0x14;
const REG_EUL_HEADING_LSB: u8 = 0x1A;
const REG_OPR_MODE: u8 = 0x3D;
const REG_PWR_MODE: u8 = 0x3E;
const REG_SYS_TRIGGER: u8 = 0x3F;

const BNO055_CHIP_ID: u8 = 0xA0;
const MODE_CONFIG: u8 = 0x00;
const MODE_NDOF: u8 = 0x0C;

/// 16 LSB / deg -> radians per LSB = PI / (180 * 16) = PI / 2880.
const RAD_PER_LSB: f32 = PI / 2880.0_f32;

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
    let mut err_streak: u8 = 0;

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
            let roll_raw = i16::from_le_bytes([eul_buf[2], eul_buf[3]]);
            let pitch_raw = i16::from_le_bytes([eul_buf[4], eul_buf[5]]);

            // BNO055 Gyro registers (0x14..0x19): [X=pitch_rate, Y=roll_rate, Z=yaw_rate]
            let gyr_x_raw = i16::from_le_bytes([gyr_buf[0], gyr_buf[1]]);
            let gyr_y_raw = i16::from_le_bytes([gyr_buf[2], gyr_buf[3]]);

            let state = ImuState {
                available: true,
                pitch_rad: (pitch_raw as f32) * RAD_PER_LSB,
                roll_rad: (roll_raw as f32) * RAD_PER_LSB,
                pitch_rate_rad_s: (gyr_x_raw as f32) * RAD_PER_LSB,
                roll_rate_rad_s: (gyr_y_raw as f32) * RAD_PER_LSB,
            };
            let now_ms = Instant::now().as_millis() as u32;
            IMU_SLOT.publish(state, ImuHealth::Healthy, now_ms);
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
