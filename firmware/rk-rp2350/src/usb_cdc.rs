//! Wired USB CDC-ACM virtual serial transport for RP2350 (`--features usb-cdc`).
//!
//! Streams `postcard` COBS-framed `HostToMcu` / `McuToHost` packets over Full-Speed USB bulk
//! endpoints and exchanges them with the `copper-rs` `HostLinkBridge` through `HOST_LINK_MAILBOX`.

#![cfg(feature = "usb-cdc")]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::{Builder, Config, UsbDevice};
use rk_fw_core::config::{
    CONTROL_PERIOD_MS, USB_MANUFACTURER, USB_PID_CDC, USB_PRODUCT_CDC, USB_SERIAL_RP2350, USB_VID,
};
use rk_fw_core::transport::{CobsStreamFramer, HOST_LINK_MAILBOX, encode_telemetry_frame};
use rk_proto::MAX_COBS_FRAME_BYTES;
use static_cell::StaticCell;

type RpUsbDriver = Driver<'static, USB>;

#[embassy_executor::task]
async fn usb_device_task(mut usb: UsbDevice<'static, RpUsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn usb_cdc_rx_task(mut rx: Receiver<'static, RpUsbDriver>) -> ! {
    let mut chunk = [0u8; 64];
    let mut framer = CobsStreamFramer::new();

    loop {
        rx.wait_connection().await;
        info!("USB CDC-ACM host connected (RX)");

        loop {
            match rx.read_packet(&mut chunk).await {
                Ok(n) if n > 0 => {
                    framer.push_bytes(&chunk[..n], |pkt| {
                        HOST_LINK_MAILBOX.push_rx_cmd(pkt);
                    });
                }
                Ok(_) => {}
                Err(e) => {
                    warn!("USB CDC-ACM RX disconnected: {:?}", e);
                    break;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_cdc_tx_task(mut tx: Sender<'static, RpUsbDriver>) -> ! {
    let mut wire_buf = [0u8; MAX_COBS_FRAME_BYTES];
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));

    loop {
        tx.wait_connection().await;
        info!("USB CDC-ACM host connected (TX)");

        'connected: loop {
            ticker.next().await;
            let Some(telem) = HOST_LINK_MAILBOX.take_tx_telem() else {
                continue;
            };
            let Ok(encoded) = encode_telemetry_frame(&telem, &mut wire_buf) else {
                continue;
            };

            // Full-Speed USB bulk max packet size is 64 bytes; split frames cleanly.
            for chunk in encoded.chunks(64) {
                if tx.write_packet(chunk).await.is_err() {
                    warn!("USB CDC-ACM TX disconnected");
                    break 'connected;
                }
            }
            // Send Zero-Length Packet (ZLP) if frame length is an exact multiple of 64 bytes.
            if encoded.len() % 64 == 0 && tx.write_packet(&[]).await.is_err() {
                break 'connected;
            }
        }
    }
}

/// Initializes RP2350 USB CDC-ACM device and spawns async RX/TX Embassy tasks.
pub fn spawn_usb_cdc(spawner: &Spawner, driver: RpUsbDriver) {
    let mut usb_cfg = Config::new(USB_VID, USB_PID_CDC);
    usb_cfg.manufacturer = Some(USB_MANUFACTURER);
    usb_cfg.product = Some(USB_PRODUCT_CDC);
    usb_cfg.serial_number = Some(USB_SERIAL_RP2350);
    usb_cfg.max_power = 250;
    usb_cfg.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESC.init([0u8; 256]),
        BOS_DESC.init([0u8; 256]),
        &mut [],
        CONTROL_BUF.init([0u8; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
    let (tx, rx) = class.split();
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb).expect("spawn usb_device_task"));
    spawner.spawn(usb_cdc_rx_task(rx).expect("spawn usb_cdc_rx_task"));
    spawner.spawn(usb_cdc_tx_task(tx).expect("spawn usb_cdc_tx_task"));
}
