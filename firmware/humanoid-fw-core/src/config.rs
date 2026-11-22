//! Shared firmware configuration constants for Rocky MCU targets (RP2350 & ESP32-P4).

/// Control loop tick rate in Hertz (200 Hz -> 5 ms deterministic period).
pub const CONTROL_LOOP_HZ: u32 = 200;
/// Control loop period in milliseconds.
pub const CONTROL_PERIOD_MS: u64 = 1000 / CONTROL_LOOP_HZ as u64;
/// Fixed single-precision control step duration in seconds.
pub const CONTROL_DT_SEC: f32 = 1.0_f32 / CONTROL_LOOP_HZ as f32;

/// Hardware watchdog timeout in milliseconds.
/// Fed only after a successful `copper-rs` control iteration.
pub const HW_WATCHDOG_TIMEOUT_MS: u64 = 250;

/// Default number of active humanoid joints for Rocky Mk1.
pub const DEFAULT_JOINT_COUNT: usize = 17;

/// USB device identification (pid.codes open-source test VID/PID range).
pub const USB_VID: u16 = 0x1209;
pub const USB_PID_CDC: u16 = 0x524B; // "RK" — Rocky Studio CDC-ACM serial link
pub const USB_PID_NCM: u16 = 0x524E; // "RN" — Rocky Studio CDC-NCM USB Ethernet link
pub const USB_MANUFACTURER: &str = "Rocky Robotics";
pub const USB_PRODUCT_CDC: &str = "Rocky MCU Link (USB CDC-ACM)";
pub const USB_PRODUCT_NCM: &str = "Rocky MCU Link (USB Ethernet NCM)";
pub const USB_SERIAL_RP2350: &str = "ROCKY-RP2350-001";
pub const USB_SERIAL_ESP32P4: &str = "ROCKY-ESP32P4-001";

/// Wired USB Networking (CDC-NCM) static IPv4 configuration.
///
/// Configure the host PC's USB Ethernet adapter to `10.42.0.1/24`.
/// The MCU listens for `HostToMcu` COBS datagrams on UDP `10.42.0.2:7777`
/// and streams `McuToHost` telemetry datagrams back to the active host peer.
pub const USB_NET_MCU_IPV4: [u8; 4] = [10, 42, 0, 2];
pub const USB_NET_HOST_IPV4: [u8; 4] = [10, 42, 0, 1];
pub const USB_NET_PREFIX_LEN: u8 = 24;
pub const USB_NET_UDP_PORT: u16 = 7777;

/// Locally-administered unicast MAC addresses for USB CDC-NCM.
pub const USB_NET_MCU_MAC: [u8; 6] = [0x02, 0x52, 0x4F, 0x43, 0x4B, 0x02];
pub const USB_NET_HOST_MAC: [u8; 6] = [0x02, 0x52, 0x4F, 0x43, 0x4B, 0x01];

/// Selected wired host transport mode on the MCU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WiredTransportKind {
    /// USB CDC-ACM virtual serial port carrying `postcard` COBS frames.
    UsbCdcAcm,
    /// USB CDC-NCM Ethernet-over-USB carrying UDP datagrams (`10.42.0.2:7777`).
    UsbNetworkingNcm,
}
