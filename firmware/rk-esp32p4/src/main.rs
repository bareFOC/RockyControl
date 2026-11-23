//! Rocky Humanoid ESP32-P4 Board-Support Adapter (`copper-rs` + `esp-hal` + `embassy-rs`).
//!
//! Shares 100% of:
//! - `crates/rk-proto` (`postcard` COBS wire protocol, `f32`)
//! - `crates/rk-control` (`DirectServoKernel`: 100 ms watchdog, soft limits, slew clamp)
//! - `firmware/rk-fw-core` (`DirectServoCuTask`, `HostLinkBridge`, `ServoBusBridge`,
//!   `CobsStreamFramer`, `decode_udp_host_frame`, `BenchLoopbackBus`, `StatusLedMode`)
//!
//! Hardware Bring-Up Checklist for ESP32-P4 (RV32IMAFC 400 MHz + USB-OTG High-Speed 480 Mbps):
//! 1. Install target: `rustup target add riscv32imafc-unknown-none-elf`
//! 2. Install flasher: `cargo install espflash`
//! 3. Bind `esp_hal::otg_fs::Usb` / `esp_hal::usb_serial_jtag` (or High-Speed OTG PHY)
//!    into `embassy-usb` `CdcAcmClass` or `CdcNcmClass` feeding `HOST_LINK_MAILBOX`.
//! 4. Bind ESP32-P4 `UART1` (1 Mbps half-duplex serial bus) or `TWAI0` (CAN-FD) into
//!    `SERVO_BUS_SLOT::register_hooks(...)` once hardware servos are connected.

#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;

use cu29::prelude::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use rk_fw_core::config::{CONTROL_PERIOD_MS, USB_SERIAL_ESP32P4};
use rk_fw_core::copper::current_led_mode;

#[copper_runtime(config = "../copperconfig.ron")]
struct RockyEsp32P4App {}

fn make_embassy_robot_clock() -> RobotClock {
    RobotClock::new_with_rtc(
        || Instant::now().as_micros().saturating_mul(1_000),
        |ns| {
            let wait_us = ns / 1_000;
            let start = Instant::now().as_micros();
            while Instant::now().as_micros().saturating_sub(start) < wait_us {
                core::hint::spin_loop();
            }
        },
    )
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_alloc::heap_allocator!(size: 96 * 1024);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    println!(
        "Rocky Humanoid ESP32-P4 Firmware ({}) booting (copper-rs + esp-hal-embassy)",
        USB_SERIAL_ESP32P4
    );

    // TODO(ESP32-P4 Hardware Bring-Up):
    // 1. Spawn `embassy-usb` CDC-ACM or CDC-NCM High-Speed USB OTG driver pushing decoded
    //    `HostToMcu` frames into `rk_fw_core::transport::HOST_LINK_MAILBOX` and draining
    //    `McuToHost` telemetry frames at 200 Hz.
    // 2. Arm `timg0.wdt` hardware watchdog (250 ms) and feed after each Copper iteration.
    // 3. Drive status LED GPIO from `current_led_mode().is_lit(now_ms)`.

    let clock = make_embassy_robot_clock();
    let noop_logger = Arc::new(Mutex::new(NoopLogger::new()));
    let app = RockyEsp32P4App::builder()
        .with_clock(clock)
        .with_logger::<NoopSectionStorage, NoopLogger>(noop_logger)
        .build()
        .expect("Failed to build RockyEsp32P4App");

    let mut running_app = app.start().expect("Failed to start RockyEsp32P4App");
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));

    loop {
        ticker.next().await;
        let _ = running_app.run_one_iteration();
        let _led_on = current_led_mode().is_lit(Instant::now().as_millis() as u32);
    }
}
