//! Rocky Humanoid RP2350 Firmware (`copper-rs` + `embassy-rp` + `embassy-usb`).
//!
//! # Architecture
//! - **Control Runtime:** Real `copper-rs` (`#[copper_runtime(config = "copperconfig.ron")]`)
//!   executing `DirectServoCuTask` + `HostLinkBridge` + `ServoBusBridge` at deterministic 200 Hz.
//! - **Async I/O & Scheduling:** `embassy-rp` + `embassy-executor` concurrently servicing:
//!   1. Wired USB transport:
//!      - Default (`--features usb-cdc`): USB CDC-ACM virtual serial (`postcard` COBS frames)
//!      - Wired USB Networking (`--no-default-features --features usb-net,bench-loopback`):
//!        USB CDC-NCM Ethernet (`10.42.0.2/24`, UDP port `7777`)
//!   2. 200 Hz `copper-rs` control loop calling `app.run_one_iteration()`
//!   3. Hardware RP2350 `Watchdog` (250 ms timeout, fed only after each healthy control tick)
//!   4. Status LED (`GPIO25`) reflecting `Teach` (1 Hz blink), `Drive` (solid), `EStop` (8 Hz strobe)

#![no_std]
#![no_main]

extern crate alloc;

mod servo_uart;
mod usb_cdc;
mod usb_net;

use alloc::sync::Arc;
use alloc::vec;
use core::mem::MaybeUninit;

use buddy_system_allocator::LockedHeap;
use cu29::prelude::*;
use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Instant, Ticker};
use humanoid_fw_core::config::{CONTROL_PERIOD_MS, HW_WATCHDOG_TIMEOUT_MS};
use humanoid_fw_core::copper::current_led_mode;
use panic_probe as _;
use spin::Mutex;

// --- Internal SRAM Heap for Copper-rs (cu29) ---
const HEAP_BYTES: usize = 64 * 1024;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_BYTES] = [MaybeUninit::uninit(); HEAP_BYTES];

#[global_allocator]
static ALLOCATOR: LockedHeap<32> = LockedHeap::empty();

fn init_heap() {
    unsafe {
        let start = core::ptr::addr_of_mut!(HEAP_MEM) as usize;
        ALLOCATOR.lock().init(start, HEAP_BYTES);
    }
}

// --- RP2350 Interrupt Bindings ---
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// --- Compile-Time Generated Copper-rs Task Graph ---
#[copper_runtime(config = "copperconfig.ron")]
struct RockyRp2350App {}

#[cfg(all(feature = "usb-cdc", feature = "usb-net"))]
compile_error!(
    "Enable either `usb-cdc` (USB serial) or `usb-net` (USB CDC-NCM networking), not both simultaneously."
);

#[cfg(not(any(feature = "usb-cdc", feature = "usb-net")))]
compile_error!("Enable one wired transport feature: `usb-cdc` or `usb-net`.");

/// Builds a monotonic nanosecond `copper-rs` `RobotClock` backed by `embassy-time`.
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

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    init_heap();
    let p = embassy_rp::init(Default::default());

    info!("Rocky Humanoid RP2350 Firmware booting (copper-rs + embassy-rp)");

    // 1. Status LED (Pico 2 onboard LED on GPIO25)
    let mut status_led = Output::new(p.PIN_25, Level::Low);

    // 2. Bind Servo Bus (Milestone 2.1: BenchLoopbackBus default; Milestone 2.2: UART0 1 Mbps)
    #[cfg(feature = "uart-servo-bus")]
    servo_uart::init_uart_servo_bus(p.UART0, p.PIN_0, p.PIN_1);

    // 3. Bring up wired USB transport (CDC-ACM serial OR CDC-NCM Ethernet-over-USB)
    let usb_driver = UsbDriver::new(p.USB, Irqs);
    #[cfg(feature = "usb-cdc")]
    {
        info!("Transport: Wired USB CDC-ACM (postcard COBS)");
        usb_cdc::spawn_usb_cdc(&spawner, usb_driver);
    }
    #[cfg(all(feature = "usb-net", not(feature = "usb-cdc")))]
    {
        info!("Transport: Wired USB Networking CDC-NCM (10.42.0.2:7777 UDP)");
        usb_net::spawn_usb_net(&spawner, usb_driver);
    }

    // 4. Construct & start the Copper-rs deterministic application graph
    let clock = make_embassy_robot_clock();
    let noop_logger = Arc::new(Mutex::new(NoopLogger::new()));
    let app = RockyRp2350App::builder()
        .with_clock(clock)
        .with_logger::<NoopSectionStorage, NoopLogger>(noop_logger)
        .build()
        .expect("Failed to build RockyRp2350App copper runtime");

    let mut running_app = app.start().expect("Failed to start RockyRp2350App tasks");
    info!("Copper-rs runtime started @ 200 Hz (5 ms period) in Teach mode");

    // 5. Arm RP2350 hardware watchdog (resets MCU if control task stalls > 250 ms)
    let mut hw_watchdog = Watchdog::new(p.WATCHDOG);
    let wd_timeout = Duration::from_millis(HW_WATCHDOG_TIMEOUT_MS);
    hw_watchdog.start(wd_timeout);

    // 6. Deterministic 200 Hz Embassy control loop driving `copper-rs` iterations
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));
    loop {
        ticker.next().await;

        if let Err(err) = running_app.run_one_iteration() {
            error!("Copper iteration error: {:?}", defmt::Debug2Format(&err));
            // Do NOT feed hardware watchdog on fatal runtime error -> forces safe hardware reset
            continue;
        }

        // Update physical safety mode LED
        let now_ms = Instant::now().as_millis() as u32;
        if current_led_mode().is_lit(now_ms) {
            status_led.set_high();
        } else {
            status_led.set_low();
        }

        // Feed hardware watchdog only after a clean control cycle
        hw_watchdog.feed(wd_timeout);
    }
}
