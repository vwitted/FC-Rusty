//! Onboard-LED (PD10) pattern for the magnetometer-cal lifecycle. Pure
//! no_std, host-tested; the renderer (`blink_task`) and publisher
//! (`mekf_task`) live in main.rs.
//! Spec: docs/superpowers/specs/2026-06-23-cal-led-feedback-design.md

/// Cal-feedback LED phase, published by the MEKF task and rendered by the
/// blink task. `Calibrating` carries coverage percent (0..=100).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalLed {
    Idle,
    Calibrating(u8),
    AwaitingLevel,
    Saved,
    Fault,
}

/// Whether the LED should be lit, given the phase and milliseconds since
/// the renderer last saw a *variant* change (progress updates within
/// `Calibrating` keep the same clock, so the blink stays smooth).
pub fn led_on(phase: CalLed, elapsed_ms: u32) -> bool {
    match phase {
        // 1 Hz heartbeat: 100 ms on / 900 ms off.
        CalLed::Idle => elapsed_ms % 1000 < 100,
        // Accelerating: faster + higher duty as coverage fills.
        // period 600→200 ms, duty 40→95 % as p 0→100. Near-solid at 100%.
        CalLed::Calibrating(p) => {
            let p = (p as u32).min(100);
            let period = 600 - 4 * p; // ms
            let duty_pct = 40 + (55 * p) / 100;
            let on_ms = period * duty_pct / 100;
            elapsed_ms % period < on_ms
        }
        // Coverage complete: hard OFF until the craft is held level.
        CalLed::AwaitingLevel => false,
        // Triple-burst (100 ms on/off ×3) then resume heartbeat.
        CalLed::Saved => {
            if elapsed_ms < 600 {
                (elapsed_ms / 100) % 2 == 0
            } else {
                elapsed_ms % 1000 < 100
            }
        }
        // Held fault: 5 s on / 5 s off until the pilot reverts AUX4.
        CalLed::Fault => elapsed_ms % 10000 < 5000,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_is_short_heartbeat() {
        assert!(led_on(CalLed::Idle, 50));
        assert!(!led_on(CalLed::Idle, 500));
    }

    #[test]
    fn calibrating_full_is_near_solid() {
        // p=100 → 200 ms period, 190 ms on.
        assert!(led_on(CalLed::Calibrating(100), 0));
        assert!(led_on(CalLed::Calibrating(100), 180));
        assert!(!led_on(CalLed::Calibrating(100), 195));
    }

    #[test]
    fn calibrating_empty_is_slower() {
        // p=0 → 600 ms period, 240 ms on.
        assert!(led_on(CalLed::Calibrating(0), 0));
        assert!(!led_on(CalLed::Calibrating(0), 300));
    }

    #[test]
    fn awaiting_level_is_off() {
        for t in [0u32, 200, 1000, 5000] {
            assert!(!led_on(CalLed::AwaitingLevel, t), "t={}", t);
        }
    }

    #[test]
    fn saved_bursts_then_heartbeats() {
        assert!(led_on(CalLed::Saved, 0)); // burst on
        assert!(!led_on(CalLed::Saved, 150)); // burst off
        assert!(led_on(CalLed::Saved, 250)); // burst on
        assert!(led_on(CalLed::Saved, 1000)); // heartbeat resumed (on)
    }

    #[test]
    fn fault_is_slow_5s() {
        assert!(led_on(CalLed::Fault, 1000));
        assert!(!led_on(CalLed::Fault, 6000));
    }
}
