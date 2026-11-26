//! RP2350 QSPI flash persistence for calibrated joint limits, balance gains,
//! UVC gait config, and combat impact config.
//!
//! Uses the last 4 KiB sector of the 2 MiB flash region (`memory.x`: `0x1FF000..0x200000`).
//! Boot reads occur before `control_task` starts; runtime `CommitFlash` writes are
//! queued via a lock-free pending flag and executed asynchronously in `flash_writer_task`
//! outside any user `critical_section`.

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::{info, warn};
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_time::Timer;
use portable_atomic::{AtomicBool, Ordering};
use rk_fw_core::flash_config::{FLASH_SECTOR_BYTES, PersistedConfig};

/// 2 MiB flash size matching `memory.x` (`LENGTH = 2048K`).
pub const FLASH_SIZE_BYTES: usize = 2 * 1024 * 1024;
/// Offset of the last 4 KiB sector in QSPI flash.
pub const CONFIG_SECTOR_OFFSET: u32 = (FLASH_SIZE_BYTES - FLASH_SECTOR_BYTES) as u32;

static PENDING_COMMIT: AtomicBool = AtomicBool::new(false);
static PENDING_DATA: Mutex<RefCell<Option<PersistedConfig>>> = Mutex::new(RefCell::new(None));

/// Loads persisted configuration from the last QSPI flash sector at boot.
/// Returns `None` if the sector is blank or fails magic/version/CRC-32 validation.
pub fn load_boot_config(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE_BYTES>) -> Option<PersistedConfig> {
    let mut buf = [0xFFu8; FLASH_SECTOR_BYTES];
    if let Err(e) = flash.blocking_read(CONFIG_SECTOR_OFFSET, &mut buf) {
        warn!("QSPI flash read failed at boot: {:?}", e);
        return None;
    }
    match PersistedConfig::decode_sector(&buf) {
        Some(cfg) => {
            info!("Loaded persisted config from QSPI flash ({} joints)", cfg.joint_count);
            Some(cfg)
        }
        None => {
            info!("QSPI flash config blank or invalid; using RobotProfile defaults");
            None
        }
    }
}

/// Queues a `PersistedConfig` snapshot for asynchronous flash write.
/// Safe to call from `control_task`: performs only a bounded copy and sets an atomic flag.
pub fn request_commit(cfg: PersistedConfig) {
    critical_section::with(|cs| {
        *PENDING_DATA.borrow_ref_mut(cs) = Some(cfg);
    });
    PENDING_COMMIT.store(true, Ordering::Release);
}

fn take_pending_commit() -> Option<PersistedConfig> {
    if !PENDING_COMMIT.swap(false, Ordering::AcqRel) {
        return None;
    }
    critical_section::with(|cs| PENDING_DATA.borrow_ref_mut(cs).take())
}

/// Background task that persists queued `CommitFlash` snapshots to QSPI flash
/// outside any user critical section.
#[embassy_executor::task]
pub async fn flash_writer_task(mut flash: Flash<'static, FLASH, Blocking, FLASH_SIZE_BYTES>) {
    let mut sector = [0xFFu8; FLASH_SECTOR_BYTES];
    loop {
        Timer::after_millis(25).await;
        if let Some(cfg) = take_pending_commit() {
            cfg.encode_sector(&mut sector);
            if let Err(e) = flash.blocking_erase(
                CONFIG_SECTOR_OFFSET,
                CONFIG_SECTOR_OFFSET + FLASH_SECTOR_BYTES as u32,
            ) {
                warn!("QSPI flash erase failed: {:?}", e);
                continue;
            }
            if let Err(e) = flash.blocking_write(CONFIG_SECTOR_OFFSET, &sector) {
                warn!("QSPI flash write failed: {:?}", e);
                continue;
            }
            info!("Committed calibration & tuning config to QSPI flash");
        }
    }
}
