//! Firmware-only flash access for the config record (sub-project A).
//!
//! Wraps the embassy blocking Flash HAL over the reserved CONFIG sector.
//! `read()` runs once at boot; `write()` runs only while disarmed
//! (erase/write must never happen in the control path). A flash fault
//! degrades to "uncalibrated", never a control hazard.

use embassy_stm32::Peri;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::peripherals::FLASH;

use super::record::{self, Config, RECORD_LEN};

/// Flash-relative offset of the reserved CONFIG sector
/// (0x081E0000 − FLASH_BASE 0x08000000).
pub const CONFIG_OFFSET: u32 = 0x1E_0000;
/// H743 sector size (erase granularity).
pub const CONFIG_SECTOR_LEN: u32 = 128 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistError {
    Erase,
    Write,
}

/// Construct the blocking flash driver. Call once; the caller owns the
/// `FLASH` peripheral.
pub fn driver(flash: Peri<'static, FLASH>) -> Flash<'static, Blocking> {
    Flash::new_blocking(flash)
}

/// Read and validate the stored config. `None` ⇒ use `Config::default()`.
pub fn read(flash: &mut Flash<'static, Blocking>) -> Option<Config> {
    let mut buf = [0u8; RECORD_LEN];
    flash.blocking_read(CONFIG_OFFSET, &mut buf).ok()?;
    record::decode(&buf)
}

/// Erase the sector and write one record. Disarmed-only (caller enforces).
pub fn write(flash: &mut Flash<'static, Blocking>, cfg: &Config) -> Result<(), PersistError> {
    let bytes = record::encode(cfg);
    flash
        .blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + CONFIG_SECTOR_LEN)
        .map_err(|_| PersistError::Erase)?;
    flash
        .blocking_write(CONFIG_OFFSET, &bytes)
        .map_err(|_| PersistError::Write)?;
    Ok(())
}
