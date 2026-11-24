//! Rocky Humanoid RP2350 Firmware (`embassy-rp` + `embassy-usb`).
//!
//! # Architecture
//! - **Control Runtime:** A dedicated Embassy `control` task drives
//!   `rk_fw_core::copper::ControlTask` (a plain, allocation-free wrapper over
//!   `DirectServoKernel::step()`) at a deterministic 200 Hz (`Ticker`, 5 ms period).
//!   There is no copper-rs (`cu29`) runtime on the MCU: a single scheduler (Embassy)
//!   owns all timing, I/O and task orchestration.
//! - **Async I/O & Scheduling:** `embassy-rp` + `embassy-executor` concurrently servicing:
//!   1. Wired USB transport:
//!      - Default (`--features usb-cdc`): USB CDC-ACM virtual serial (`postcard` COBS frames)
//!      - Wired USB Networking (`--no-default-features --features usb-net,bench-loopback`):
//!        USB CDC-NCM Ethernet (`10.42.0.2/24`, UDP port `7777`)
//!   2. 200 Hz `control` task reading the latest host command from `HOST_LINK_MAILBOX`
//!      (latest-wins), servo feedback from `SERVO_BUS_SLOT` with the hardware IMU
//!      snapshot from `IMU_SLOT` merged in, and publishing servo commands + telemetry
//!      back to the mailbox.
//!   3. Hardware RP2350 `Watchdog` (250 ms timeout, fed only after each healthy control tick)
//!   4. Status LED (`GPIO25`) reflecting `Teach` (1 Hz blink), `Drive` (solid), `EStop` (8 Hz strobe)

#![no_std]
#![no_main]

#[cfg(all(feature = "imu-bno055", not(feature = "imu-lsm6dsv")))]
mod imu_bno055;
#[cfg(feature = "imu-lsm6dsv")]
mod imu_lsm6dsv;
#[cfg(feature = "i3c-servo-bus")]
mod i3c_simplefoc;
mod servo_uart;
mod usb_cdc;
mod usb_net;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::Peri;
use embassy_rp::peripherals::{USB, WATCHDOG};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Instant, Ticker};
use rk_control::{RawServoCommand, RawServoFeedback};
use rk_fw_core::config::{CONTROL_PERIOD_MS, HW_WATCHDOG_TIMEOUT_MS};
use rk_fw_core::control_task::{RobotIo, control_task as shared_control_task};
use rk_fw_core::copper::current_led_mode;
use rk_fw_core::imu::IMU_SLOT;
use rk_fw_core::servo_bus::SERVO_BUS_SLOT;
use rk_fw_core::transport::HOST_LINK_MAILBOX;
use rk_proto::{HostToMcu, McuToHost};
use panic_probe as _;

// The high-rate LSM6DSV (SPI1) and legacy BNO055 (I2C0) IMUs are mutually
// exclusive: motor-current disturbance makes the BNO055 magnetometer untrustworthy
// whenever the LSM6DSV is the active control source.
#[cfg(all(feature = "imu-bno055", feature = "imu-lsm6dsv"))]
compile_error!(
    "Enable either `imu-bno055` or `imu-lsm6dsv`, not both. The LSM6DSV replaces the BNO055 in the control path."
);

// Both servo-bus transports register hooks in SERVO_BUS_SLOT; enabling both
// would let call order silently decide the winner. Reject the conflict instead.
#[cfg(all(feature = "uart-servo-bus", feature = "i3c-servo-bus"))]
compile_error!(
    "Enable either `uart-servo-bus` or `i3c-servo-bus`, not both: they are mutually exclusive servo-bus hooks."
);

// --- RP2350 Interrupt Bindings ---
// A single binding covers every feature combination: USB is always active, and
// I2C0 is only consumed by the optional BNO055 IMU (binding it when unused is
// harmless). This replaces the previously duplicated `Irqs` cfg predicates.
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<embassy_rp::peripherals::I2C0>;
});

#[cfg(all(feature = "usb-cdc", feature = "usb-net"))]
compile_error!(
    "Enable either `usb-cdc` (USB serial) or `usb-net` (USB CDC-NCM networking), not both simultaneously."
);

#[cfg(not(any(feature = "usb-cdc", feature = "usb-net")))]
compile_error!("Enable one wired transport feature: `usb-cdc` or `usb-net`.");

/// Board I/O adapter for the shared, runtime-agnostic control task.
///
/// All timing comes from Embassy (`Ticker` + `Instant`); all data comes from the
/// existing global mailboxes and servo-bus hooks. The shared kernel is constructed
/// by `rk_fw_core::control_task::control_task` with the crate-default 21 joints and
/// identity calibration (the historical board configuration).
struct HardwareRobotIo {
    ticker: Ticker,
    led: Output<'static>,
    watchdog: Watchdog,
    wd_timeout: Duration,
}

impl RobotIo for HardwareRobotIo {
    async fn wait_tick(&mut self) {
        self.ticker.next().await;
    }

    fn now_ms(&self) -> u32 {
        Instant::now().as_millis() as u32
    }

    fn take_cmd(&mut self) -> Option<HostToMcu> {
        // Latest-wins host command; wire errors are read explicitly below and
        // passed into the pure control step (no hidden global mailbox read).
        HOST_LINK_MAILBOX.take_rx_cmd()
    }

    fn wire_error_counts(&self) -> (u16, u16) {
        HOST_LINK_MAILBOX.wire_error_counts()
    }

    fn read_feedback(&mut self, dt_sec: f32) -> RawServoFeedback {
        // Servo feedback from the active bus. Merge the hardware IMU snapshot into
        // the feedback exactly as the old `ServoBusBridge` did (the I3C adapter also
        // does this internally, so re-merging is idempotent for that bus).
        let mut fb = SERVO_BUS_SLOT.read_feedback(dt_sec);
        if let Some(hw_imu) = IMU_SLOT.latest() {
            fb.imu = hw_imu;
            fb.imu_source_mask = IMU_SLOT.latest_source_mask();
        }
        fb
    }

    fn write_command(&mut self, cmd: &RawServoCommand) {
        SERVO_BUS_SLOT.write_command(cmd);
    }

    fn publish_telemetry(&mut self, telem: McuToHost) {
        HOST_LINK_MAILBOX.publish_tx_telem(telem);
    }

    fn post_tick(&mut self, now_ms: u32) {
        // Physical safety mode LED.
        let led_mode = current_led_mode();
        if led_mode.is_lit(now_ms) {
            self.led.set_high();
        } else {
            self.led.set_low();
        }

        // Feed hardware watchdog only after a clean control cycle. A panic or
        // stall before this point leaves the watchdog to force a safe reset.
        self.watchdog.feed(self.wd_timeout);
    }
}

/// Thin board wrapper around the shared 200 Hz control loop.
///
/// The control logic (kernel stepping, dt, tick fan-out) lives exclusively in
/// `rk_fw_core::control_task::control_task`, so firmware and simulation execute
/// the exact same code path. This task only supplies the board I/O adapter and
/// starts the watchdog.
#[embassy_executor::task]
async fn control_task(led: Output<'static>, watchdog: Peri<'static, WATCHDOG>) {
    let wd_timeout = Duration::from_millis(HW_WATCHDOG_TIMEOUT_MS);
    let mut hw_watchdog = Watchdog::new(watchdog);
    hw_watchdog.start(wd_timeout);

    let io = HardwareRobotIo {
        ticker: Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS)),
        led,
        watchdog: hw_watchdog,
        wd_timeout,
    };

    info!("Embassy control runtime started @ 200 Hz (5 ms period) in Teach mode");

    shared_control_task(io).await;
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Rocky Humanoid RP2350 Firmware booting (embassy-rp)");

    // 1. Status LED (Pico 2 onboard LED on GPIO25) owned by the control task.
    let status_led = Output::new(p.PIN_25, Level::Low);

    // 2. Bind Servo Bus (Milestone 2.1: BenchLoopbackBus default; Milestone 2.2: UART0 1 Mbps)
    #[cfg(feature = "uart-servo-bus")]
    servo_uart::init_uart_servo_bus(p.UART0, p.PIN_0, p.PIN_1);

    // 2b. Optional Bosch BNO055 9-Axis IMU over I2C0 (GPIO4 SDA / GPIO5 SCL @ 400 kHz).
    // Suppressed when the high-rate LSM6DSV is enabled (magnetometer disturbed by motor currents).
    #[cfg(all(feature = "imu-bno055", not(feature = "imu-lsm6dsv")))]
    imu_bno055::spawn_bno055(&spawner, p.I2C0, p.PIN_5, p.PIN_4, Irqs);

    // 2c. Optional ST LSM6DSV16X 6-axis IMU over dedicated SPI1 @ 960 Hz FIFO / 200 Hz bursts.
    #[cfg(feature = "imu-lsm6dsv")]
    imu_lsm6dsv::spawn_lsm6dsv(
        &spawner,
        p.SPI1,
        p.PIN_10,
        p.PIN_11,
        p.PIN_8,
        p.PIN_9,
        p.DMA_CH0,
        p.DMA_CH1,
    );

    // 2d. Optional SimpleFOC I3C servo bus adapter (external RP2350 PIO I3C driver plugs
    // into the board hooks registered here; no PIO reimplementation in this module).
    #[cfg(feature = "i3c-servo-bus")]
    i3c_simplefoc::init_i3c_servo_bus();

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

    // 4. Spawn the single deterministic control task. Embassy owns all scheduling.
    spawner.spawn(control_task(status_led, p.WATCHDOG).expect("spawn control_task"));
}
