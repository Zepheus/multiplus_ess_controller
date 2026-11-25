//! When to stop controlling: hand-back rules for failing reads, slow ticks and failing
//! writes while we own the loop. Pure state with `Instant`/`Duration` inputs, so every
//! rule is testable off-device. While stock owns the loop nothing here applies: stock is
//! in charge, and we simply try again next tick.

use std::time::{Duration, Instant};

/// Core reads may fail this long while we own the loop: well inside the Multi's 60 s
/// setpoint watchdog.
pub const READ_FAIL_LIMIT: Duration = Duration::from_secs(10);
/// A read phase beyond this: a peer answers too slowly to control on.
pub const TICK_STALL: Duration = Duration::from_secs(5);
/// A read phase beyond this counts as slow; `SLOW_TICKS_MAX` in a row hand back.
pub const SLOW_TICK: Duration = Duration::from_millis(2500);
pub const SLOW_TICKS_MAX: u32 = 3;
/// Ticks in a row with a failed write (setpoint or feed-in flag): a command that never
/// lands (a force-charge setpoint, an export cap) must not silently lapse.
pub const WRITE_FAILS_MAX: u32 = 3;

#[derive(Default)]
pub struct Health {
    read_fail_since: Option<Instant>,
    slow_ticks: u32,
    write_fails: u32,
}

impl Health {
    /// After the read phase. `missing` names the core input that could not be read.
    pub fn check_read(&mut self, now: Instant, missing: Option<&str>, owner: bool) -> Result<(), String> {
        let (Some(m), true) = (missing, owner) else {
            self.read_fail_since = None;
            return Ok(());
        };
        let since = *self.read_fail_since.get_or_insert(now);
        if now.duration_since(since) >= READ_FAIL_LIMIT {
            return Err(format!("reads failing for {:.0} s ({m})", READ_FAIL_LIMIT.as_secs_f64()));
        }
        Ok(())
    }

    /// Before the write phase, with the time the tick has taken so far.
    pub fn check_pace(&mut self, work: Duration, owner: bool) -> Result<(), String> {
        if !owner {
            self.slow_ticks = 0;
            return Ok(());
        }
        if work > TICK_STALL {
            return Err(format!("tick took {:.1} s before its writes", work.as_secs_f64()));
        }
        self.slow_ticks = if work > SLOW_TICK { self.slow_ticks + 1 } else { 0 };
        if self.slow_ticks >= SLOW_TICKS_MAX {
            return Err(format!(
                "{} ticks in a row slower than {:.1} s",
                self.slow_ticks,
                SLOW_TICK.as_secs_f64()
            ));
        }
        Ok(())
    }

    /// After the write phase.
    pub fn check_writes(&mut self, failed: bool, owner: bool) -> Result<(), String> {
        self.write_fails = if failed && owner { self.write_fails + 1 } else { 0 };
        if self.write_fails >= WRITE_FAILS_MAX {
            return Err(format!("writes failed {} ticks in a row", self.write_fails));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: fn(u64) -> Duration = Duration::from_secs;
    const MS: fn(u64) -> Duration = Duration::from_millis;

    #[test]
    fn failing_reads_hand_back_after_the_limit_only_while_owning() {
        let t0 = Instant::now();
        let mut h = Health::default();
        assert!(h.check_read(t0, Some("grid"), true).is_ok());
        assert!(h.check_read(t0 + S(9), Some("grid"), true).is_ok());
        assert!(h.check_read(t0 + S(10), Some("grid"), true).is_err());

        let mut h = Health::default();
        for s in [0, 60, 600, 6000] {
            assert!(h.check_read(t0 + S(s), Some("grid"), false).is_ok(), "stock owns");
        }
    }

    #[test]
    fn a_good_read_or_losing_ownership_resets_the_failure_window() {
        let t0 = Instant::now();
        let mut h = Health::default();
        assert!(h.check_read(t0, Some("grid"), true).is_ok());
        assert!(h.check_read(t0 + S(9), None, true).is_ok());
        assert!(h.check_read(t0 + S(15), Some("grid"), true).is_ok());
        assert!(h.check_read(t0 + S(16), Some("grid"), false).is_ok());
        assert!(h.check_read(t0 + S(24), Some("grid"), true).is_ok(), "window restarted");
    }

    #[test]
    fn a_stalled_tick_hands_back_before_writing() {
        let mut h = Health::default();
        assert!(h.check_pace(S(4), true).is_ok());
        assert!(h.check_pace(S(6), true).is_err());
        assert!(h.check_pace(S(60), false).is_ok(), "stock owns");
    }

    #[test]
    fn sustained_slow_ticks_hand_back_single_ones_do_not() {
        let mut h = Health::default();
        assert!(h.check_pace(S(3), true).is_ok());
        assert!(h.check_pace(S(3), true).is_ok());
        assert!(h.check_pace(MS(200), true).is_ok(), "streak broken");
        assert!(h.check_pace(S(3), true).is_ok());
        assert!(h.check_pace(S(3), true).is_ok());
        assert!(h.check_pace(S(3), true).is_err());
    }

    #[test]
    fn repeated_write_failures_hand_back_only_while_owning() {
        let mut h = Health::default();
        for _ in 0..5 {
            assert!(h.check_writes(true, false).is_ok(), "stock owns");
        }
        assert!(h.check_writes(true, true).is_ok());
        assert!(h.check_writes(true, true).is_ok());
        assert!(h.check_writes(false, true).is_ok(), "streak broken");
        assert!(h.check_writes(true, true).is_ok());
        assert!(h.check_writes(true, true).is_ok());
        assert!(h.check_writes(true, true).is_err());
    }
}
