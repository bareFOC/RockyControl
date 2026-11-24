//! Async RP2350 ST LSM6DSV16X 6-Axis IMU driver (`--features imu-lsm6dsv`).
//!
//! Hardware wiring (dedicated SPI1 — never shared with the servo bus):
//! - `GPIO10`: SCK  (SPI1 CLK)
//! - `GPIO11`: MOSI (SDI)
//! - `GPIO8`:  MISO (SDO)
//! - `GPIO9`:  CS   (active-low, software-driven)
//! - `INT1` (FIFO threshold / DRDY) is optional — the task polls `FIFO_STATUS1/2`
//!   each 5 ms control tick, so no IRQ binding is required.
//!
//! Register bring-up uses the constants and parser in `rk_fw_core::lsm6dsv`:
//! - `WHO_AM_I` (`0x0F`) must read `0x70`, retried 3 times.
//! - `CTRL3`: `BDU | IF_INC` (auto-increment for burst FIFO reads), then `SW_RESET`.
//! - `CTRL6`: ±1000 dps gyro + LPF1; `CTRL7`: LPF1 enable; `CTRL8`: ±8 g accel (16X).
//! - `FIFO_CTRL3/4`: batch gyro + accel at 960 Hz in continuous mode.
//! - `INT1_CTRL`: route FIFO threshold / gyro DRDY to INT1.
//!
//! The task burst-reads the hardware FIFO at 200 Hz (5 ms), feeds the bytes to
//! `Lsm6dsvFilter::update_from_fifo_burst` and publishes `ImuState` into
//! `IMU_SLOT` tagged with `estimator_valid::LSM6DSV_OK`. A transient SPI fault
//! is handled by `Lsm6dsvFilter::on_read_fault` (20 ms gyro holdout).
//!
//! ## Async transfer path / executor yielding
//!
//! Every SPI byte moves through the RP2350 DMA in `Async` mode: `Spi::new`
//! takes two DMA channels and the task awaits `Spi::write` / `Spi::transfer`.
//! Each `.await` suspends `lsm6dsv_task` and returns the single-thread executor
//! to USB, control and watchdog tasks while the DMA engine clocks the bus.
//! A 2-byte register read is ~16 µs at 1 MHz; a full 224-byte FIFO burst is
//! ~1.8 ms on the wire — previously this was a CPU-blocking stall, now the
//! executor remains responsive for the whole burst. CS is still toggled by
//! software (`Output`) immediately before/after each awaited transfer so the
//! LSM6DSV's active-low frame boundary is preserved across the yield.

#![cfg(feature = "imu-lsm6dsv")]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{
    DMA_CH0, DMA_CH1, PIN_8, PIN_9, PIN_10, PIN_11, SPI1,
};
use embassy_rp::spi::{Async, Config as SpiConfig, Phase, Polarity, Spi};
use embassy_time::{Duration, Instant, Ticker, Timer};
use rk_fw_core::imu::{IMU_SLOT, ImuHealth};
use rk_fw_core::lsm6dsv::{
    CTRL3_BDU, CTRL3_IF_INC, CTRL3_SW_RESET, CTRL6_GYRO_1000DPS_LPF1,
    CTRL7_LPF1_G_EN, CTRL8_ACCEL_8G_16X, FIFO_CTRL3_BATCH_960HZ,
    FIFO_CTRL4_CONTINUOUS, INT1_CTRL_FIFO_TH_DRDY, Lsm6dsvFilter,
    NOMINAL_FIFO_BURST_SAMPLES, ODR_960_HZ, REG_CTRL1, REG_CTRL2, REG_CTRL3,
    REG_CTRL6, REG_CTRL7, REG_CTRL8, REG_FIFO_CTRL3, REG_FIFO_CTRL4,
    REG_FIFO_DATA_OUT_TAG, REG_FIFO_STATUS1, REG_FIFO_STATUS2, REG_INT1_CTRL,
    REG_WHO_AM_I, SPI_READ_BIT, parse_fifo_word_count, verify_lsm6dsv16x_id,
    WHO_AM_I_EXPECTED,
};
use rk_proto::estimator_valid;

/// Maximum 7-byte FIFO records drained per 5 ms tick (clamps the 10-bit FIFO word count).
/// 32 records covers several ticks of backlog without exhausting stack/heap.
const MAX_BURST_RECORDS: usize = 32;
const FIFO_RECORD_BYTES: usize = 7;
const FIFO_BURST_CAP_BYTES: usize = MAX_BURST_RECORDS * FIFO_RECORD_BYTES;
/// Number of `WHO_AM_I` attempts during bring-up (2–3 retries as required).
const ID_RETRIES: u8 = 3;

type LsmSpi = Spi<'static, SPI1, Async>;

// Both SPI DMA channels share DMA_IRQ_0. A single `bind_interrupts!` struct can
// list multiple handlers for that IRQ; we bind DMA_CH0 (TX) and DMA_CH1 (RX)
// here so no change to the shared `Irqs` in `main.rs` is required.
bind_interrupts!(struct LsmIrqs {
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>, DmaInterruptHandler<DMA_CH1>;
});

/// SPI1 mode 0 (CPOL=0, CPHA=0), 1 MHz — within LSM6DSV16X SPI spec.
fn spi_config() -> SpiConfig {
    let mut cfg = SpiConfig::default();
    cfg.frequency = 1_000_000;
    cfg.phase = Phase::CaptureOnFirstTransition;
    cfg.polarity = Polarity::IdleLow;
    cfg
}

/// Asserts / releases the software-driven active-low chip select.
#[inline]
fn cs_low(cs: &mut Output<'static>) {
    cs.set_low();
}

#[inline]
fn cs_high(cs: &mut Output<'static>) {
    cs.set_high();
}

/// Writes a single register over SPI1 using an async DMA transfer.
///
/// The task yields at `.await` while the DMA engine clocks the two bytes out.
async fn write_reg(spi: &mut LsmSpi, cs: &mut Output<'static>, reg: u8, value: u8) {
    cs_low(cs);
    let _ = spi.write(&[reg & !SPI_READ_BIT, value]).await;
    cs_high(cs);
}

/// Reads a single register over SPI1 (dummy byte after the address) using an
/// async DMA transfer. Yields at `.await` for the ~2-byte exchange.
async fn read_reg(spi: &mut LsmSpi, cs: &mut Output<'static>, reg: u8) -> Option<u8> {
    let mut buf = [0u8; 2];
    cs_low(cs);
    let result = spi.transfer(&mut buf, &[reg | SPI_READ_BIT, 0x00]).await;
    cs_high(cs);
    result.ok().map(|_| buf[1])
}

/// Reads a 10-bit FIFO word count from `FIFO_STATUS1/2`.
async fn read_fifo_status(spi: &mut LsmSpi, cs: &mut Output<'static>) -> Option<u16> {
    let s1 = read_reg(spi, cs, REG_FIFO_STATUS1).await?;
    let s2 = read_reg(spi, cs, REG_FIFO_STATUS2).await?;
    Some(parse_fifo_word_count(s1, s2))
}

/// Drains up to [`FIFO_BURST_CAP_BYTES`] of FIFO data starting at
/// `REG_FIFO_DATA_OUT_TAG` using SPI auto-increment (`CTRL3.IF_INC`).
///
/// Up to 224 bytes are clocked by DMA while the task is suspended — this is
/// the ~1.8 ms transfer that previously stalled the executor.
async fn burst_read_fifo(
    spi: &mut LsmSpi,
    cs: &mut Output<'static>,
    words: u16,
    buf: &mut [u8; FIFO_BURST_CAP_BYTES],
) -> Option<usize> {
    let wanted = (words as usize).saturating_mul(FIFO_RECORD_BYTES).min(FIFO_BURST_CAP_BYTES);
    if wanted == 0 {
        return Some(0);
    }
    cs_low(cs);
    let result = spi
        .transfer(&mut buf[..wanted], &[REG_FIFO_DATA_OUT_TAG | SPI_READ_BIT])
        .await;
    cs_high(cs);
    result.ok().map(|_| wanted)
}

/// Bring-up sequence with `WHO_AM_I` verification (3 attempts, no panic on failure).
async fn init_lsm6dsv(spi: &mut LsmSpi, cs: &mut Output<'static>) -> bool {
    IMU_SLOT.set_health(ImuHealth::Calibrating);

    // Software reset first (CTRL3.SW_RESET), with a short settle delay.
    write_reg(spi, cs, REG_CTRL3, CTRL3_SW_RESET).await;
    Timer::after(Duration::from_millis(5)).await;

    for attempt in 0..ID_RETRIES {
        let who = read_reg(spi, cs, REG_WHO_AM_I).await.unwrap_or(0);
        let ctrl8 = read_reg(spi, cs, REG_CTRL8).await.unwrap_or(0);
        if verify_lsm6dsv16x_id(who, ctrl8) {
            info!(
                "LSM6DSV16X WHO_AM_I=0x{:02x} verified (attempt {}); configuring SPI1 FIFO @ 960 Hz",
                who,
                attempt + 1
            );
            break;
        }
        if attempt + 1 == ID_RETRIES {
            warn!(
                "LSM6DSV16X WHO_AM_I mismatch: got 0x{:02x} (expected 0x{:02x}) after {} attempts",
                who, WHO_AM_I_EXPECTED, ID_RETRIES
            );
            IMU_SLOT.set_health(ImuHealth::Fault);
            return false;
        }
        Timer::after(Duration::from_millis(20)).await;
    }

    // BDU + IF_INC (auto-increment enables the 7-byte FIFO burst read).
    write_reg(spi, cs, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC).await;
    // 960 Hz ODR for both accel (CTRL1) and gyro (CTRL2).
    write_reg(spi, cs, REG_CTRL1, ODR_960_HZ).await;
    write_reg(spi, cs, REG_CTRL2, ODR_960_HZ).await;
    // ±1000 dps gyro + LPF1, enable LPF1, ±8 g accel (16X variant).
    write_reg(spi, cs, REG_CTRL6, CTRL6_GYRO_1000DPS_LPF1).await;
    write_reg(spi, cs, REG_CTRL7, CTRL7_LPF1_G_EN).await;
    write_reg(spi, cs, REG_CTRL8, CTRL8_ACCEL_8G_16X).await;
    // Batch gyro + accel at 960 Hz, continuous FIFO mode.
    write_reg(spi, cs, REG_FIFO_CTRL3, FIFO_CTRL3_BATCH_960HZ).await;
    write_reg(spi, cs, REG_FIFO_CTRL4, FIFO_CTRL4_CONTINUOUS).await;
    // Route FIFO threshold + gyro DRDY to INT1 (for optional DRDY wiring).
    write_reg(spi, cs, REG_INT1_CTRL, INT1_CTRL_FIFO_TH_DRDY).await;

    Timer::after(Duration::from_millis(10)).await;
    IMU_SLOT.set_health(ImuHealth::Healthy);
    true
}

/// Spawns the 200 Hz LSM6DSV16X SPI1 FIFO-burst task on the Embassy executor.
///
/// `tx_dma`/`rx_dma` must be distinct DMA channels (SPI1 TX/RX) from the
/// board's `Peripherals`; `main.rs` passes `p.DMA_CH0` and `p.DMA_CH1`.
pub fn spawn_lsm6dsv(
    spawner: &Spawner,
    spi1: Peri<'static, SPI1>,
    sck: Peri<'static, PIN_10>,
    mosi: Peri<'static, PIN_11>,
    miso: Peri<'static, PIN_8>,
    cs: Peri<'static, PIN_9>,
    tx_dma: Peri<'static, DMA_CH0>,
    rx_dma: Peri<'static, DMA_CH1>,
) {
    let spi = Spi::new(spi1, sck, mosi, miso, tx_dma, rx_dma, LsmIrqs, spi_config());
    let cs = Output::new(cs, Level::High);
    spawner.spawn(lsm6dsv_task(spi, cs).expect("spawn lsm6dsv_task"));
    info!("LSM6DSV16X SPI1 DMA FIFO-burst driver spawned @ 200 Hz (GPIO8/9/10/11)");
}

#[embassy_executor::task]
async fn lsm6dsv_task(mut spi: LsmSpi, mut cs: Output<'static>) {
    // Bring up with retries, never panic / stall the control loop.
    while !init_lsm6dsv(&mut spi, &mut cs).await {
        warn!("LSM6DSV16X not responding on SPI1; retrying bring-up in 250 ms");
        Timer::after(Duration::from_millis(250)).await;
    }

    let mut filter = Lsm6dsvFilter::new();
    let mut burst = [0u8; FIFO_BURST_CAP_BYTES];
    let mut ticker = Ticker::every(Duration::from_millis(5));
    let mut fault_streak: u8 = 0;

    loop {
        ticker.next().await;
        let now_ms = Instant::now().as_millis() as u32;

        let words = read_fifo_status(&mut spi, &mut cs).await.unwrap_or(0);
        let read_bytes = burst_read_fifo(&mut spi, &mut cs, words, &mut burst)
            .await
            .unwrap_or(0);

        let state = if read_bytes >= FIFO_RECORD_BYTES * 2 {
            fault_streak = 0;
            filter.update_from_fifo_burst(&burst[..read_bytes], 1.0 / 200.0_f32, now_ms)
        } else {
            // No FIFO data or SPI fault: use holdout/fault path so telemetry never
            // falsely claims a live high-rate sample.
            fault_streak = fault_streak.saturating_add(1);
            filter.on_read_fault(now_ms)
        };

        if fault_streak == 10 {
            warn!("LSM6DSV16X FIFO read fault streak; re-initializing sensor");
            IMU_SLOT.set_health(ImuHealth::Fault);
            let _ = init_lsm6dsv(&mut spi, &mut cs).await;
            filter.reset();
            fault_streak = 0;
        }

        // Always publish with the LSM6DSV source tag — `publish()` already does
        // this, `publish_with_source` makes the invariant explicit.
        IMU_SLOT.publish_with_source(
            state,
            if state.available { ImuHealth::Healthy } else { ImuHealth::Fault },
            estimator_valid::LSM6DSV_OK,
            now_ms,
        );

        // Keep the nominal burst constant referenced so a future DMA read can
        // size its transfer directly from the driver constant.
        let _ = NOMINAL_FIFO_BURST_SAMPLES;
    }
}
