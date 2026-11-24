//! Rocky Humanoid ESP32-P4 Board-Support Adapter (`esp-hal` + `embassy-rs`).
//!
//! Scheduling is handled entirely by Embassy (single scheduler): a 200 Hz `Ticker`
//! drives [`ControlTask`] from `rk-fw-core`, which calls `DirectServoKernel::step()`
//! directly. There is no copper-rs (`cu29`) runtime on the MCU.
//!
//! Shares 100% of:
//! - `crates/rk-proto` (`postcard` COBS wire protocol, `f32`)
//! - `crates/rk-control` (`DirectServoKernel`: 100 ms watchdog, soft limits, slew clamp)
//! - `firmware/rk-fw-core` (`ControlTask`, `CobsStreamFramer`, `decode_udp_host_frame`,
//!   `BenchLoopbackBus`, `StatusLedMode`)
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

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use rk_fw_core::config::{CONTROL_DT_SEC, CONTROL_PERIOD_MS, DEFAULT_JOINT_COUNT, USB_SERIAL_ESP32P4};
use rk_fw_core::copper::{ControlTask, current_led_mode};
use rk_fw_core::servo_bus::SERVO_BUS_SLOT;
use rk_fw_core::transport::HOST_LINK_MAILBOX;
use rk_control::JointCalibration;
use rk_proto::MAX_JOINTS;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    // esp-hal requires a global allocator for its drivers; keep the 96 KiB heap.
    esp_alloc::heap_allocator!(size: 96 * 1024);
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    println!(
        "Rocky Humanoid ESP32-P4 Firmware ({}) booting (esp-hal-embassy)",
        USB_SERIAL_ESP32P4
    );

    // TODO(ESP32-P4 Hardware Bring-Up):
    // 1. Spawn `embassy-usb` CDC-ACM or CDC-NCM High-Speed USB OTG driver pushing decoded
    //    `HostToMcu` frames into `rk_fw_core::transport::HOST_LINK_MAILBOX` and draining
    //    `McuToHost` telemetry frames at 200 Hz.
    // 2. Arm `timg0.wdt` hardware watchdog (250 ms) and feed after each control iteration.
    // 3. Drive status LED GPIO from `current_led_mode().is_lit(now_ms)`.

    let cal = [JointCalibration::default(); MAX_JOINTS];
    let mut control = ControlTask::new(DEFAULT_JOINT_COUNT, &cal);
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));

    loop {
        ticker.next().await;
        let now_ms = Instant::now().as_millis() as u32;

        // Embassy tasks own all I/O; the control step itself is pure and allocation-free.
        // Wire error counters are an explicit input, not a hidden global read.
        let cmd = HOST_LINK_MAILBOX.take_rx_cmd();
        let wire_errors = HOST_LINK_MAILBOX.wire_error_counts();
        let fb = SERVO_BUS_SLOT.read_feedback(CONTROL_DT_SEC);
        let out = control.step(now_ms, CONTROL_DT_SEC, cmd.as_ref(), &fb, wire_errors);

        SERVO_BUS_SLOT.write_command(&out.servo_cmd);
        HOST_LINK_MAILBOX.publish_tx_telem(out.telemetry);

        let _led_on = current_led_mode().is_lit(now_ms);
    }
}
