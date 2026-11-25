//! Milestone 2.2 RP2350 hardware serial servo bus adapter (`--features uart-servo-bus`).
//!
//! Uses RP2350 `UART0` (`GPIO0` TX / `GPIO1` RX @ 1 Mbps) in a non-blocking mailbox pattern
//! so `copper-rs` `ServoBusBridge` runs deterministically without blocking on slow serial reads.
//! Ready to wire Feetech STS3215 / Dynamixel Protocol 2.0 half-duplex smart bus servos.
//!
//! **Legacy / unused:** `rk-multibus` (BareFOC register-pointer transactions over
//! the external PIO I3C driver) is now the single servo wire path for the
//! control application. This UART adapter is kept only as a bring-up placeholder
//! for electrical-continuity checks on GPIO0/GPIO1; it is not part of the
//! production servo bus and is enabled only by `--features uart-servo-bus`.

#![cfg(feature = "uart-servo-bus")]

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::info;
use embassy_rp::Peri;
use embassy_rp::peripherals::{PIN_0, PIN_1, UART0};
use embassy_rp::uart::{Blocking, Config as UartConfig, Uart};
use rk_control::{DEFAULT_MAX_TORQUE_NM, RawServoCommand, RawServoFeedback};
use rk_fw_core::servo_bus::{SERVO_BUS_SLOT, ServoBusHooks};
use rk_proto::{JointCommandMode, MAX_JOINTS};

struct UartBusState {
    uart: Option<Uart<'static, Blocking>>,
    last_feedback: RawServoFeedback,
    last_cmd: RawServoCommand,
}

static UART_BUS: Mutex<RefCell<UartBusState>> = Mutex::new(RefCell::new(UartBusState {
    uart: None,
    last_feedback: RawServoFeedback {
        raw_angle_rad: [0.0_f32; MAX_JOINTS],
        velocity_rad_s: [0.0_f32; MAX_JOINTS],
        current_ma: [0u16; MAX_JOINTS],
        temp_c: [28u8; MAX_JOINTS],
        bus_voltage_mv: 12_000,
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
        imu_source_mask: rk_proto::estimator_valid::LSM6DSV_OK,
        contact_mask: 0,
        grf_n: [0.0; 2],
        contact_valid_mask: 0,
    },
    last_cmd: RawServoCommand {
        torque_mask: 0,
        raw_target_rad: [0.0_f32; MAX_JOINTS],
        command_mode: [JointCommandMode::Position as u8; MAX_JOINTS],
        raw_velocity_rad_s: [0.0_f32; MAX_JOINTS],
        torque_nm: [DEFAULT_MAX_TORQUE_NM; MAX_JOINTS],
    },
}));

fn uart_read_feedback(_dt_sec: f32) -> RawServoFeedback {
    critical_section::with(|cs| UART_BUS.borrow_ref(cs).last_feedback)
}

fn uart_write_command(cmd: &RawServoCommand) {
    critical_section::with(|cs| {
        let mut st = UART_BUS.borrow_ref_mut(cs);
        st.last_cmd = *cmd;
        // Milestone 2.2 wire hook: encode vendor SYNC_WRITE packet (e.g., Feetech STS3215 /
        // Dynamixel Protocol 2.0) and transmit over `st.uart`.
        // Keeping a crisp 8-byte sync header placeholder here verifies UART0 TX electrical
        // continuity on a logic analyzer before plugging in physical actuators.
        if let Some(uart) = st.uart.as_mut() {
            let mask_le = cmd.torque_mask.to_le_bytes();
            let frame = [0xFF, 0xFF, 0xFE, 0x04, mask_le[0], mask_le[1], mask_le[2], mask_le[3]];
            let _ = uart.blocking_write(&frame);
        }
    });
}

/// Initializes RP2350 UART0 at 1,000,000 bps and registers it with `copper-rs` `SERVO_BUS_SLOT`.
pub fn init_uart_servo_bus(
    uart0: Peri<'static, UART0>,
    tx: Peri<'static, PIN_0>,
    rx: Peri<'static, PIN_1>,
) {
    let mut cfg = UartConfig::default();
    cfg.baudrate = 1_000_000;
    let uart = Uart::new_blocking(uart0, tx, rx, cfg);

    critical_section::with(|cs| {
        UART_BUS.borrow_ref_mut(cs).uart = Some(uart);
    });

    SERVO_BUS_SLOT.register_hooks(ServoBusHooks {
        read_feedback: uart_read_feedback,
        write_command: uart_write_command,
    });
    info!("RP2350 UART0 servo bus initialized @ 1 Mbps (GPIO0 TX / GPIO1 RX)");
}
