//! Handing the loop back to stock: on a failure while we own it, at startup when a
//! previous run left EXTERNAL standing, and at shutdown. One sequence for all three:
//! stock's feed-in flags go back first, while stock is still disengaged, then the mode
//! write that resumes it. Only a mode that is still EXTERNAL (ours) is touched, so an
//! operator's own mode choice stands.

use crate::dbus::{Bus, P_HUB4MODE, SETTINGS};
use crate::feedin::{self, FlagSnapshot};
use crate::gridmeter::HUB4MODE_EXTERNAL;
use crate::loop_core::LoopState;
use std::time::{Duration, Instant};

/// Total time for putting stock's flags back: the mode write must not wait behind a
/// failing service.
pub const FLAG_RESTORE_BUDGET: Duration = Duration::from_secs(3);

/// What `release` found or did.
#[derive(Debug, PartialEq)]
pub enum Released {
    /// Hub4Mode is back at the stock mode.
    Restored,
    /// Hub4Mode was already something other than EXTERNAL: someone else chose it.
    NotOurs(i32),
    /// The mode write did not land.
    Failed,
}

/// Clear a latched fault so the next call is really sent, logging what it was.
fn clear_fault(bus: &dyn Bus) {
    if let Some(f) = bus.take_fault() {
        eprintln!("hand-back: bus fault: {f}");
    }
}

/// Restore stock's flags, then `orig_mode`, if the mode is still ours (EXTERNAL, or
/// unreadable). A fault on one call never stops the next: each is sent on a clear latch.
pub fn release(bus: &dyn Bus, orig_mode: i32, flags: Option<FlagSnapshot>, attempts: u32) -> Released {
    clear_fault(bus);
    let mode = bus.get_f64(SETTINGS, P_HUB4MODE).map(|m| m as i32);
    clear_fault(bus);
    if let Some(m) = mode.filter(|&m| m != HUB4MODE_EXTERNAL) {
        eprintln!("Hub4Mode is {m}, not ours: left as is");
        return Released::NotOurs(m);
    }
    if let Some(fs) = flags {
        feedin::restore_flags(bus, &fs, Instant::now() + FLAG_RESTORE_BUDGET);
    }
    for attempt in 1..=attempts {
        clear_fault(bus);
        match bus.set_i32(SETTINGS, P_HUB4MODE, orig_mode) {
            Ok(()) => {
                eprintln!("Hub4Mode restored to {orig_mode}");
                return Released::Restored;
            }
            Err(e) => {
                eprintln!("Hub4Mode restore attempt {attempt} failed: {e}");
                if attempt < attempts {
                    std::thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }
    clear_fault(bus);
    Released::Failed
}

/// A failure made this tick unusable. If we own the loop in the takeover stage, hand it
/// back now and count it as a trip, exactly like a sanity hand-back: re-entry then waits
/// the doubled dwell. A mode the operator set meanwhile is a deliberate cede, as decide
/// treats it. A mode write that does not land is retried by the loop's "re-asserting
/// stock mode" path. In the other stages, and while stock owns the loop, the tick is
/// only skipped.
pub fn on_failure(
    bus: &dyn Bus,
    st: &mut LoopState,
    flags: &mut Option<FlagSnapshot>,
    orig_mode: i32,
    takeover: bool,
    t: f64,
    why: &str,
) {
    if !(takeover && st.owner_us) {
        eprintln!("[t={t:.0}] {why}: tick skipped");
        return;
    }
    st.sanity_trips = st.sanity_trips.saturating_add(1);
    eprintln!("[t={t:.0}] SAFETY: {why}: handing back to stock (trip {})", st.sanity_trips);
    if let Released::NotOurs(_) = release(bus, orig_mode, flags.take(), 1) {
        st.ceded_external = true;
        eprintln!("[t={t:.0}] mode changed externally: ceded (sticky)");
    }
    st.owner_us = false;
    st.last_flip_t = t;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::Kind;
    use crate::dbus::{P_DISABLE_FEEDIN, VEBUS};
    use crate::loop_core::testutil;
    use crate::testbus::MockBus;

    fn flags() -> Option<FlagSnapshot> {
        let mut b = MockBus::new();
        b.set(VEBUS, P_DISABLE_FEEDIN, 1.0);
        Some(feedin::snapshot_flags(&b))
    }

    fn written(bus: &MockBus) -> Vec<String> {
        bus.writes.borrow().iter().map(|(_, p, _)| p.clone()).collect()
    }

    #[test]
    fn flags_go_back_before_the_mode_that_resumes_stock() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, f64::from(HUB4MODE_EXTERNAL));
        assert_eq!(release(&bus, 1, flags(), 1), Released::Restored);
        assert_eq!(written(&bus), [P_DISABLE_FEEDIN, P_HUB4MODE]);
    }

    #[test]
    fn a_latched_fault_never_blocks_the_mode_write() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, f64::from(HUB4MODE_EXTERNAL));
        bus.latch_on_write(P_DISABLE_FEEDIN); // the flag restore times out
        assert_eq!(release(&bus, 1, flags(), 1), Released::Restored);
        assert_eq!(written(&bus), [P_HUB4MODE]);
        assert!(bus.fault.borrow().is_none(), "nothing left latched");
    }

    #[test]
    fn an_operator_mode_choice_is_left_alone() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, 2.0);
        assert_eq!(release(&bus, 1, flags(), 5), Released::NotOurs(2));
        assert!(written(&bus).is_empty());
    }

    #[test]
    fn an_unreadable_mode_is_still_handed_back() {
        let bus = MockBus::new();
        assert_eq!(release(&bus, 1, None, 1), Released::Restored);
        assert_eq!(written(&bus), [P_HUB4MODE]);
    }

    #[test]
    fn a_failure_while_owning_hands_back_and_counts_a_trip() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, f64::from(HUB4MODE_EXTERNAL));
        let mut st = testutil::state(Kind::Stock);
        st.owner_us = true;
        let mut fl = flags();
        on_failure(&bus, &mut st, &mut fl, 1, true, 42.0, "bus fault");
        assert!(!st.owner_us && !st.ceded_external);
        assert_eq!((st.sanity_trips, st.last_flip_t), (1, 42.0));
        assert!(fl.is_none(), "flags consumed");
        assert_eq!(written(&bus), [P_DISABLE_FEEDIN, P_HUB4MODE]);
    }

    #[test]
    fn a_failure_while_stock_owns_changes_nothing() {
        let bus = MockBus::new();
        let mut st = testutil::state(Kind::Stock);
        let mut fl = None;
        on_failure(&bus, &mut st, &mut fl, 1, true, 42.0, "bus fault");
        assert_eq!(st.sanity_trips, 0);
        assert!(written(&bus).is_empty());
    }

    #[test]
    fn shadow_and_trim_never_write_on_a_failure() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, f64::from(HUB4MODE_EXTERNAL));
        let mut st = testutil::state(Kind::Stock);
        st.owner_us = true; // shadow's "would own"
        on_failure(&bus, &mut st, &mut None, 1, false, 42.0, "bus fault");
        assert!(written(&bus).is_empty());
        assert_eq!(st.sanity_trips, 0);
    }

    #[test]
    fn an_operator_mode_set_during_a_failure_is_a_sticky_cede() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_HUB4MODE, 2.0);
        let mut st = testutil::state(Kind::Stock);
        st.owner_us = true;
        on_failure(&bus, &mut st, &mut None, 1, true, 42.0, "reads failing");
        assert!(st.ceded_external && !st.owner_us);
        assert!(written(&bus).is_empty(), "the operator's mode stands");
    }
}
