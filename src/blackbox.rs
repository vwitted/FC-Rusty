//! Append-only flight log in flash bank 2.
//!
//! # Why flash, and why bank 2
//!
//! The wire is not available in flight. `logger::putc` busy-waits on
//! USART6 TXE inside a `critical_section`, so a sample logged as it is
//! taken stalls the 8 kHz loop for milliseconds -- and there is nobody
//! holding a serial cable anyway.
//!
//! Bank 2 is chosen for two independent reasons, both load-bearing:
//!
//!  - **DFU writes only bank 1.** So a log survives a reflash, and the
//!    whole workflow rests on that: fly a recording build, then flash a
//!    dump build and read back the flight you just did. Without it you
//!    would need a way to get data off the board before reflashing, which
//!    is the hard part of every blackbox.
//!  - **The H7 cannot read a bank while programming it.** Code executes
//!    from bank 1, so writing bank 2 does not stall the fetch. That is
//!    what makes logging possible during flight rather than only when
//!    parked.
//!
//! # Append-only, and why there is no wrap
//!
//! H7 flash programs in 32-byte words, erases in 128 KB sectors, and a
//! word may be written **once** between erases -- writing it twice, even
//! with the same value, corrupts its ECC and the read fails. So the log
//! is strictly append-only and fills forward. When it is full it stops,
//! rather than wrapping: wrapping would need an erase mid-flight, which
//! is a multi-second blocking operation, and losing the beginning of a
//! flight to keep the end is the wrong trade for identifying a plant
//! model -- the interesting transients are at the start.
//!
//! Erased flash reads as 0xFF, and `PlantSample::encode` leaves its
//! reserved bytes at 0x00, so "never written" and "written" are
//! distinguishable with no separate index.
//!
//! Everything here except the `hw` module is pure and host-tested.

// PlantSample is only needed by the hardware half and the tests; a
// host build without `firmware` has neither, so it is imported there.
use crate::plant_log::RECORD_LEN;

/// Flash-relative offset of the log region: bank 2 base, 0x08100000 minus
/// FLASH_BASE 0x08000000. Matches `BLACKBOX` in memory.x.
pub const OFFSET: u32 = 0x0010_0000;

/// Length of the region. Bank 2 up to the config sector, i.e. sectors
/// 8..=14 of 16. Matches `BLACKBOX` in memory.x.
pub const LEN: u32 = 896 * 1024;

/// H743 flash sector size, and therefore the erase granularity.
pub const SECTOR_LEN: u32 = 128 * 1024;

/// Sectors in the region.
pub const SECTORS: u32 = LEN / SECTOR_LEN;

/// Records the log holds when full.
pub const CAPACITY: usize = (LEN as usize) / RECORD_LEN;

/// Default logging rate, Hz.
///
/// The control loop runs at 8 kHz; logging every iteration would fill the
/// region in 3.6 seconds. 200 Hz gives 143 s of flight, and the airframe
/// dynamics being identified -- rotational inertia, drag -- are all well
/// under 20 Hz, so this is oversampled by an order of magnitude already.
pub const DEFAULT_RATE_HZ: u32 = 200;

/// Seconds of log the region holds at a given rate.
pub const fn duration_s(rate_hz: u32) -> u32 {
    CAPACITY as u32 / rate_hz
}

/// True if this record has never been written.
///
/// Erased flash is all-ones. A written record cannot be all-ones because
/// `PlantSample::encode` zeroes its reserved tail, so this needs no
/// separate written-flag and no CRC.
pub fn is_blank(record: &[u8; RECORD_LEN]) -> bool {
    record.iter().all(|&b| b == 0xFF)
}

/// Find the first unwritten record index, given a way to read one.
///
/// Binary search, not a linear scan. The log is append-only, so written
/// records are a strict PREFIX of the region -- which is exactly the
/// property a binary search needs. That turns a full-region scan of
/// 28672 reads into about 15, which matters because this runs at boot
/// before anything else can happen.
///
/// Returns `CAPACITY` when the log is full.
pub fn find_append_index<F>(mut read: F) -> usize
where
    F: FnMut(usize) -> [u8; RECORD_LEN],
{
    // Invariant: everything below `lo` is written, everything at or above
    // `hi` is blank. Start by establishing the ends.
    if is_blank(&read(0)) {
        return 0;
    }
    let mut lo = 0usize; // known written
    let mut hi = CAPACITY; // known blank (or the end)
    if !is_blank(&read(CAPACITY - 1)) {
        return CAPACITY;
    }
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if is_blank(&read(mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// Byte offset of a record index within flash.
pub const fn record_offset(index: usize) -> u32 {
    OFFSET + (index * RECORD_LEN) as u32
}

/// Decimation factor to log at `rate_hz` from a loop running at
/// `loop_hz`. Always at least 1.
pub const fn decimation(loop_hz: u32, rate_hz: u32) -> u32 {
    if rate_hz == 0 || loop_hz <= rate_hz {
        1
    } else {
        loop_hz / rate_hz
    }
}

// ---- Firmware side ----

#[cfg(feature = "firmware")]
pub use hw::Blackbox;

#[cfg(feature = "firmware")]
mod hw {
    use super::*;
    use crate::plant_log::PlantSample;
    use embassy_stm32::flash::{Blocking, Flash};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "firmware", derive(defmt::Format))]
    pub enum BlackboxError {
        Erase,
        Write,
        Read,
        Full,
    }

    pub struct Blackbox {
        next: usize,
    }

    impl Blackbox {
        /// Locate the append point by reading flash.
        ///
        /// Does not erase. A log from a previous flight is left alone --
        /// erasing on boot would destroy the flight you just did the
        /// moment you power-cycled to fetch it, which is the single
        /// easiest way to make a blackbox useless.
        pub fn open(flash: &mut Flash<'static, Blocking>) -> Self {
            let next = find_append_index(|i| {
                let mut buf = [0xFFu8; RECORD_LEN];
                // A failed read reads as blank, which stops the search
                // early and costs log space rather than corrupting a log.
                let _ = flash.blocking_read(record_offset(i), &mut buf);
                buf
            });
            defmt::info!(
                "blackbox: {=usize}/{=usize} records used ({=u32} s at {=u32} Hz remain)",
                next,
                CAPACITY,
                (CAPACITY - next) as u32 / DEFAULT_RATE_HZ,
                DEFAULT_RATE_HZ,
            );
            Self { next }
        }

        pub fn used(&self) -> usize {
            self.next
        }

        pub fn is_full(&self) -> bool {
            self.next >= CAPACITY
        }

        /// Erase the whole region. Seconds of blocking work -- disarmed
        /// only, and the caller enforces that.
        pub fn erase_all(
            &mut self,
            flash: &mut Flash<'static, Blocking>,
        ) -> Result<(), BlackboxError> {
            defmt::info!("blackbox: erasing {=u32} KB, this takes a few seconds", LEN / 1024);
            flash
                .blocking_erase(OFFSET, OFFSET + LEN)
                .map_err(|_| BlackboxError::Erase)?;
            self.next = 0;
            defmt::info!("blackbox: erased");
            Ok(())
        }

        /// Append one record.
        pub fn append(
            &mut self,
            flash: &mut Flash<'static, Blocking>,
            s: &PlantSample,
        ) -> Result<(), BlackboxError> {
            if self.is_full() {
                return Err(BlackboxError::Full);
            }
            let bytes = s.encode();
            flash
                .blocking_write(record_offset(self.next), &bytes)
                .map_err(|_| BlackboxError::Write)?;
            self.next += 1;
            Ok(())
        }

        pub fn read(
            &self,
            flash: &mut Flash<'static, Blocking>,
            index: usize,
        ) -> Result<PlantSample, BlackboxError> {
            let mut buf = [0u8; RECORD_LEN];
            flash
                .blocking_read(record_offset(index), &mut buf)
                .map_err(|_| BlackboxError::Read)?;
            Ok(PlantSample::decode(&buf))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plant_log::PlantSample;

    fn blank() -> [u8; RECORD_LEN] {
        [0xFF; RECORD_LEN]
    }

    fn written(t: u32) -> [u8; RECORD_LEN] {
        PlantSample { t_ms: t, ..PlantSample::default() }.encode()
    }

    #[test]
    fn region_matches_memory_x() {
        // These four numbers are duplicated in memory.x and nothing links
        // them, so if that file changes and this does not, the firmware
        // writes into the config sector or off the end of flash. Pinning
        // them here at least makes the pair a deliberate edit.
        assert_eq!(OFFSET, 0x0010_0000, "bank 2 base, flash-relative");
        assert_eq!(LEN, 896 * 1024);
        assert_eq!(SECTORS, 7);
        // Must stop exactly where the config sector starts.
        assert_eq!(OFFSET + LEN, 0x001E_0000, "would overlap CONFIG");
    }

    #[test]
    fn the_region_is_a_whole_number_of_sectors_and_records() {
        // A part-sector cannot be erased and a part-record cannot be
        // written, so either remainder would be a silent off-by-one at
        // the very end of a flight.
        assert_eq!(LEN % SECTOR_LEN, 0);
        assert_eq!(LEN as usize % RECORD_LEN, 0);
        assert_eq!(CAPACITY, 28_672);
    }

    #[test]
    fn capacity_is_a_useful_flight_length() {
        // If this ever drops below a couple of minutes the rate is wrong,
        // not the region.
        assert!(duration_s(DEFAULT_RATE_HZ) >= 120, "{} s", duration_s(DEFAULT_RATE_HZ));
    }

    #[test]
    fn blank_means_erased_not_zeroed() {
        assert!(is_blank(&blank()));
        // A default-constructed record is all zeros, and MUST NOT read as
        // blank -- otherwise a legitimately logged zero sample would end
        // the log and everything after it would be lost.
        assert!(!is_blank(&[0u8; RECORD_LEN]));
        assert!(!is_blank(&written(0)));
    }

    #[test]
    fn append_index_on_an_empty_log_is_zero() {
        assert_eq!(find_append_index(|_| blank()), 0);
    }

    #[test]
    fn append_index_on_a_full_log_is_capacity() {
        assert_eq!(find_append_index(|i| written(i as u32)), CAPACITY);
    }

    #[test]
    fn append_index_finds_the_boundary_exactly() {
        // Every power of two either side of a boundary, because a binary
        // search that is off by one still looks plausible on most inputs.
        for used in [1usize, 2, 3, 100, 4095, 4096, 4097, CAPACITY - 1] {
            let got = find_append_index(|i| if i < used { written(i as u32) } else { blank() });
            assert_eq!(got, used, "boundary at {used}");
        }
    }

    #[test]
    fn append_index_reads_few_records() {
        // The point of the binary search. A linear scan would be 28672
        // reads at boot; this must be logarithmic.
        let mut reads = 0usize;
        let used = 20_000;
        find_append_index(|i| {
            reads += 1;
            if i < used { written(i as u32) } else { blank() }
        });
        assert!(reads < 40, "{reads} reads -- not a binary search");
    }

    #[test]
    fn record_offsets_are_word_aligned_and_in_region() {
        for i in [0usize, 1, CAPACITY - 1] {
            let off = record_offset(i);
            assert_eq!(off % RECORD_LEN as u32, 0, "record {i} straddles a flash word");
            assert!(off >= OFFSET && off + RECORD_LEN as u32 <= OFFSET + LEN);
        }
    }

    #[test]
    fn decimation_matches_the_rates() {
        assert_eq!(decimation(8000, 200), 40);
        assert_eq!(decimation(8000, 8000), 1);
        // Never zero: a zero would be a divide-by-zero or a modulo that
        // logs nothing, both silent.
        assert_eq!(decimation(200, 8000), 1);
        assert_eq!(decimation(8000, 0), 1);
    }
}
