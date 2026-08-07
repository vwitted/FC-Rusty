// dshot_bb_frame.rs — DShot frame → GPIO BSRR pattern, for bit-banged output.
//
// Transliteration of BF's bbOutputDataInit/bbOutputDataSet/bbOutputDataClear
// (src/platform/STM32/dshot_bitbang.c:140-200).
//
// Each DShot bit is three pacer states:
//
//   state 0  assert the active level      (all pins on the port)
//   state 1  per-pin: deassert iff the bit is 0
//   state 2  deassert                     (all pins on the port)
//
// so a '0' is asserted for 1/3 of the bit and a '1' for 2/3.
//
// BSRR: writing bit n sets pin n HIGH, writing bit n+16 resets it LOW. The
// entire bidirectional inversion is therefore swapping which half we use —
// there is no timer polarity involved, because no timer drives the pin.
//
// The three trailing hold states are NOT padding. BF's comment: a CRC error
// occurs if the line is switched to an input before the ESC has sampled the
// last bit, because on some MCUs the pad momentarily drops low on that
// transition. The hold states buy the ESC that sampling time. Do not remove
// them.

/// Pacer states per DShot symbol (BF: `MOTOR_DSHOT_STATE_PER_SYMBOL`).
pub const STATES_PER_SYMBOL: usize = 3;
/// Bits in a DShot frame (BF: `MOTOR_DSHOT_FRAME_BITS`).
pub const FRAME_BITS: usize = 16;
/// Trailing states that hold the line at idle (BF: `MOTOR_DSHOT_BIT_HOLD_STATES`).
pub const HOLD_STATES: usize = 3;
/// BSRR words per frame (BF: `MOTOR_DSHOT_BUF_LENGTH`) = 51.
pub const BB_BUF_LEN: usize = FRAME_BITS * STATES_PER_SYMBOL + HOLD_STATES;

/// (assert, deassert) BSRR masks for the whole port.
fn masks(port_mask: u16, inverted: bool) -> (u32, u32) {
    let pm = port_mask as u32;
    if inverted {
        (pm << 16, pm) // assert = drive LOW, deassert = drive HIGH
    } else {
        (pm, pm << 16) // assert = drive HIGH, deassert = drive LOW
    }
}

/// Lay down the port-wide skeleton: assert at each symbol start, deassert at
/// each symbol end, plus the trailing hold states. Call once per frame before
/// `output_data_set`.
pub fn output_data_init(buf: &mut [u32; BB_BUF_LEN], port_mask: u16, inverted: bool) {
    let (assert, deassert) = masks(port_mask, inverted);

    for symbol in 0..FRAME_BITS {
        buf[symbol * STATES_PER_SYMBOL] |= assert;
        buf[symbol * STATES_PER_SYMBOL + 1] = 0;
        buf[symbol * STATES_PER_SYMBOL + 2] |= deassert;
    }

    let hold = FRAME_BITS * STATES_PER_SYMBOL;
    buf[hold] |= deassert;
    buf[hold + 1] = 0;
    buf[hold + 2] = 0;
}

/// Reset the per-pin middle states to "no change", leaving the port-wide
/// skeleton intact. Call between frames instead of re-running `init`.
pub fn output_data_clear(buf: &mut [u32; BB_BUF_LEN]) {
    for symbol in 0..FRAME_BITS {
        buf[symbol * STATES_PER_SYMBOL + 1] = 0;
    }
}

/// Write one motor's 16-bit frame into the shared buffer, MSB first. A `0`
/// bit deasserts at 1/3 of the symbol; a `1` bit leaves the middle state
/// untouched so the assert runs to 2/3.
pub fn output_data_set(buf: &mut [u32; BB_BUF_LEN], pin: u8, value: u16, inverted: bool) {
    let middle_bit: u32 = if inverted {
        1 << pin // deassert = drive HIGH = BSRR low half
    } else {
        1 << (pin as u32 + 16) // deassert = drive LOW = BSRR high half
    };

    let mut v = value;
    for symbol in 0..FRAME_BITS {
        if v & 0x8000 == 0 {
            buf[symbol * STATES_PER_SYMBOL + 1] |= middle_bit;
        }
        v <<= 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const M: u16 = 0b1111; // PA0..PA3

    #[test]
    fn init_noninverted_asserts_high_then_low() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, false);
        // Non-inverted: assert = drive HIGH = BSRR low half.
        assert_eq!(buf[0], M as u32, "state 0 of bit 0 should set pins high");
        assert_eq!(buf[1], 0, "middle state starts as no-change");
        assert_eq!(buf[2], (M as u32) << 16, "state 2 should reset pins low");
    }

    #[test]
    fn init_inverted_swaps_bsrr_halves() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        // Inverted: assert = drive LOW = BSRR high half.
        assert_eq!(buf[0], (M as u32) << 16);
        assert_eq!(buf[1], 0);
        assert_eq!(buf[2], M as u32);
    }

    #[test]
    fn init_writes_all_sixteen_symbols() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        for i in 0..16 {
            assert_eq!(buf[i * 3], (M as u32) << 16, "symbol {} state 0", i);
            assert_eq!(buf[i * 3 + 2], M as u32, "symbol {} state 2", i);
        }
    }

    #[test]
    fn init_hold_states_deassert_then_hold() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        // Hold states let the ESC sample the last bit before the pin
        // becomes an input. First deasserts, other two change nothing.
        assert_eq!(buf[48], M as u32, "hold state 0 deasserts");
        assert_eq!(buf[49], 0, "hold state 1 is no-change");
        assert_eq!(buf[50], 0, "hold state 2 is no-change");
    }

    #[test]
    fn set_all_ones_leaves_middles_untouched() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0xFFFF, true);
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 0, "a '1' bit never deasserts early");
        }
    }

    #[test]
    fn set_all_zeros_deasserts_every_middle() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 2, 0x0000, true);
        // Inverted: deassert = drive HIGH = BSRR low half = 1 << pin.
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 2, "a '0' bit deasserts at 1/3");
        }
    }

    #[test]
    fn set_is_msb_first() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 1, 0x8000, true); // only the MSB is 1
        assert_eq!(buf[1], 0, "bit 0 (MSB) is 1 → middle untouched");
        for i in 1..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 1, "bits 1..15 are 0 → deassert");
        }
    }

    #[test]
    fn two_pins_share_one_buffer_without_interfering() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0x0000, true); // all zeros on PA0
        output_data_set(&mut buf, 3, 0xFFFF, true); // all ones  on PA3
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 1 << 0, "only PA0 deasserts early");
        }
    }

    #[test]
    fn clear_resets_only_middles() {
        let mut buf = [0u32; BB_BUF_LEN];
        output_data_init(&mut buf, M, true);
        output_data_set(&mut buf, 0, 0x0000, true);
        output_data_clear(&mut buf);
        for i in 0..16 {
            assert_eq!(buf[i * 3 + 1], 0, "middle cleared");
            assert_eq!(buf[i * 3], (M as u32) << 16, "state 0 preserved");
            assert_eq!(buf[i * 3 + 2], M as u32, "state 2 preserved");
        }
    }
}
