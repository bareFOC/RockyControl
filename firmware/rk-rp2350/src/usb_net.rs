//! Wired USB Networking (CDC-NCM Ethernet-over-USB + `embassy-net` UDP) for RP2350.
//!
//! Enabled via `--no-default-features --features usb-net,bench-loopback` (or with `uart-servo-bus`).
//! Enumerates natively on Linux, macOS, and Windows 11 as a USB Ethernet adapter with zero drivers:
//! - MCU static IPv4: `10.42.0.2/24`, UDP port `7777`
//! - Host PC static IPv4: `10.42.0.1/24`
//! - Datagram payload: identical `postcard` COBS-framed `HostToMcu` / `McuToHost` bytes.

#![cfg(all(feature = "usb-net", not(feature = "usb-cdc")))]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::udp::{PacketMetadata, UdpMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_rp::clocks::RoscRng;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Ticker};
use embassy_usb::class::cdc_ncm::embassy_net::{
    Device as NcmDevice, Runner as NcmRunner, State as NetState,
};
use embassy_usb::class::cdc_ncm::{CdcNcmClass, State as NcmState};
use embassy_usb::{Builder, Config, UsbDevice};
use rk_fw_core::config::{
    CONTROL_PERIOD_MS, USB_MANUFACTURER, USB_NET_HOST_MAC, USB_NET_MCU_IPV4, USB_NET_MCU_MAC,
    USB_NET_MODEL_UDP_PORT, USB_NET_PREFIX_LEN, USB_NET_UDP_PORT, USB_PID_NCM, USB_PRODUCT_NCM,
    USB_SERIAL_RP2350, USB_VID,
};
use rk_fw_core::model_store::FlashModelStore;
use rk_fw_core::transport::{
    HOST_LINK_MAILBOX, decode_udp_host_frame, encode_telemetry_frame,
};
use rk_proto::{MAX_COBS_FRAME_BYTES, MAX_MODEL_COBS_FRAME_BYTES};
use static_cell::StaticCell;

type RpUsbDriver = Driver<'static, USB>;
const MTU: usize = 1514;

#[embassy_executor::task]
async fn usb_device_task(mut usb: UsbDevice<'static, RpUsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn usb_ncm_task(runner: NcmRunner<'static, RpUsbDriver, MTU>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_stack_task(mut runner: embassy_net::Runner<'static, NcmDevice<'static, MTU>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn udp_host_link_task(stack: Stack<'static>) -> ! {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 1024];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buf = [0u8; 1024];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket
        .bind(USB_NET_UDP_PORT)
        .expect("bind Rocky UDP control port 7777");

    info!(
        "USB-NCM UDP control link listening on {}.{}.{}.{}:{}",
        USB_NET_MCU_IPV4[0],
        USB_NET_MCU_IPV4[1],
        USB_NET_MCU_IPV4[2],
        USB_NET_MCU_IPV4[3],
        USB_NET_UDP_PORT
    );

    let mut rx_pkt = [0u8; MAX_COBS_FRAME_BYTES];
    let mut wire_buf = [0u8; MAX_COBS_FRAME_BYTES];
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));
    let mut active_peer: Option<UdpMetadata> = None;

    loop {
        match select(socket.recv_from(&mut rx_pkt), ticker.next()).await {
            Either::First(Ok((n, meta))) => match decode_udp_host_frame(&rx_pkt[..n]) {
                Ok(cmd) => {
                    active_peer = Some(meta);
                    HOST_LINK_MAILBOX.push_rx_cmd(cmd);
                }
                Err(_) => {
                    HOST_LINK_MAILBOX.record_rx_error();
                }
            },
            Either::First(Err(e)) => {
                warn!("UDP recv error: {:?}", e);
            }
            Either::Second(()) => {
                let Some(peer) = active_peer else {
                    continue;
                };
                let Some(telem) = HOST_LINK_MAILBOX.take_tx_telem() else {
                    continue;
                };
                if let Ok(encoded) = encode_telemetry_frame(&telem, &mut wire_buf) {
                    if let Err(e) = socket.send_to(encoded, peer).await {
                        warn!("UDP telemetry send error: {:?}", e);
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn udp_model_service_task(stack: Stack<'static>) -> ! {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 512];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buf = [0u8; 512];

    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket
        .bind(USB_NET_MODEL_UDP_PORT)
        .expect("bind Rocky UDP model port 7778");

    let store = FlashModelStore::bundled_rocky();
    let info = store.info();
    info!(
        "MCU Flash Model Service ready on UDP :{} ({} bytes, CRC32={=u32:#010x})",
        USB_NET_MODEL_UDP_PORT, info.total_bytes, info.crc32
    );

    let mut rx_pkt = [0u8; MAX_MODEL_COBS_FRAME_BYTES];
    let mut tx_wire = [0u8; MAX_MODEL_COBS_FRAME_BYTES];

    loop {
        match socket.recv_from(&mut rx_pkt).await {
            Ok((n, peer)) if n > 0 => {
                if let Some(resp_bytes) = store.handle_cobs_datagram(&rx_pkt[..n], &mut tx_wire) {
                    if let Err(e) = socket.send_to(resp_bytes, peer).await {
                        warn!("UDP model send error: {:?}", e);
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                warn!("UDP model recv error: {:?}", e);
            }
        }
    }
}

/// Initializes RP2350 USB CDC-NCM Ethernet device, static IPv4 stack, and UDP control + model tasks.
pub fn spawn_usb_net(spawner: &Spawner, driver: RpUsbDriver) {
    let mut usb_cfg = Config::new(USB_VID, USB_PID_NCM);
    usb_cfg.manufacturer = Some(USB_MANUFACTURER);
    usb_cfg.product = Some(USB_PRODUCT_NCM);
    usb_cfg.serial_number = Some(USB_SERIAL_RP2350);
    usb_cfg.max_power = 250;
    usb_cfg.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    static NCM_STATE: StaticCell<NcmState> = StaticCell::new();
    static NET_STATE: StaticCell<NetState<MTU, 4, 4>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESC.init([0u8; 256]),
        BOS_DESC.init([0u8; 256]),
        &mut [],
        CONTROL_BUF.init([0u8; 128]),
    );

    let class = CdcNcmClass::new(
        &mut builder,
        NCM_STATE.init(NcmState::new()),
        USB_NET_HOST_MAC,
        64,
    );
    let usb = builder.build();
    spawner.spawn(usb_device_task(usb).expect("spawn usb_device_task"));

    let (ncm_runner, device) =
        class.into_embassy_net_device::<MTU, 4, 4>(NET_STATE.init(NetState::new()), USB_NET_MCU_MAC);
    spawner.spawn(usb_ncm_task(ncm_runner).expect("spawn usb_ncm_task"));

    let net_cfg = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(
            Ipv4Address::new(
                USB_NET_MCU_IPV4[0],
                USB_NET_MCU_IPV4[1],
                USB_NET_MCU_IPV4[2],
                USB_NET_MCU_IPV4[3],
            ),
            USB_NET_PREFIX_LEN,
        ),
        gateway: None,
        dns_servers: Default::default(),
    });

    let mut rng = RoscRng;
    let seed = rng.next_u64();
    let (stack, net_runner) = embassy_net::new(
        device,
        net_cfg,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(net_stack_task(net_runner).expect("spawn net_stack_task"));
    spawner.spawn(udp_host_link_task(stack).expect("spawn udp_host_link_task"));
    spawner.spawn(udp_model_service_task(stack).expect("spawn udp_model_service_task"));
}
