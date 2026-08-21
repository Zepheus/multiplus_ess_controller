use super::*;
use crate::testbus::MockBus;

const CONFIGURED: f64 = 20.0; // the user's static AcPowerSetPoint (the observed idle case)

// ---- override present (finite) swaps the target ----

#[test]
fn finite_override_swaps_the_target() {
    let mut bus = MockBus::new();
    bus.set(HUB4, P_OVR_SETPOINT, -2000.0); // sell up to 2 kW
    let mut d = Dess::new(&bus);
    let out = d.tick(&bus);
    assert_eq!(out.target_override, Some(-2000.0));
    assert_eq!(out.target(CONFIGURED), -2000.0); // configured is ignored
}

#[test]
fn zero_override_is_finite_and_overrides_not_fallback() {
    // §5: `0` is a finite value ⇒ hold exactly 0 W exchange; must NOT be conflated
    // with invalid/None (which would fall back to the configured setpoint).
    let mut bus = MockBus::new();
    bus.set(HUB4, P_OVR_SETPOINT, 0.0);
    let mut d = Dess::new(&bus);
    let out = d.tick(&bus);
    assert_eq!(out.target_override, Some(0.0));
    assert_eq!(out.target(CONFIGURED), 0.0);
}

// ---- override absent / NaN falls back to configured ----

#[test]
fn absent_override_falls_back_to_configured() {
    // DESS OFF: the common path on this install. Empty-array/invalid ⇒ get_f64 None.
    let bus = MockBus::new();
    let mut d = Dess::new(&bus);
    let out = d.tick(&bus);
    assert_eq!(out.target_override, None);
    assert_eq!(out.target(CONFIGURED), CONFIGURED);
    assert_eq!(out.feed_in_excess, 0); // absent ⇒ follow user
}

#[test]
fn nan_override_falls_back_to_configured() {
    // A numeric-but-NaN override must never command garbage; treat as no override.
    let mut bus = MockBus::new();
    bus.set(HUB4, P_OVR_SETPOINT, f64::NAN);
    let mut d = Dess::new(&bus);
    let out = d.tick(&bus);
    assert_eq!(out.target_override, None);
    assert_eq!(out.target(CONFIGURED), CONFIGURED);
}

// ---- FeedInExcess {0,1,2} mapping ----

#[test]
fn feedin_excess_mapping() {
    assert_eq!(FeedInExcess::from_i32(0), FeedInExcess::Follow);
    assert_eq!(FeedInExcess::from_i32(1), FeedInExcess::Disable);
    assert_eq!(FeedInExcess::from_i32(2), FeedInExcess::Enable);
    assert_eq!(FeedInExcess::from_i32(99), FeedInExcess::Follow); // garbage ⇒ safe

    for (raw, expect) in [(0, FeedInExcess::Follow), (1, FeedInExcess::Disable), (2, FeedInExcess::Enable)] {
        let mut bus = MockBus::new();
        bus.set(HUB4, P_OVR_FEEDIN, raw as f64);
        let mut d = Dess::new(&bus);
        let out = d.tick(&bus);
        assert_eq!(out.feed_in_excess, raw);
        assert_eq!(out.feedin(), expect);
    }
}

// ---- watchdog staleness reverts to normal ESS ----

#[test]
fn watchdog_reverts_stale_override_to_normal() {
    let mut bus = MockBus::new();
    bus.set(HUB4, P_OVR_SETPOINT, -3000.0);
    bus.set(HUB4, P_OVR_FEEDIN, 2.0); // enable feed-in
    let mut d = Dess::new(&bus);
    let t0 = Instant::now();

    // Fresh: override applies.
    let fresh = d.tick_at(&bus, t0);
    assert_eq!(fresh.target_override, Some(-3000.0));
    assert_eq!(fresh.feed_in_excess, 2);

    // Still fresh just under the watchdog (value unchanged, producer presumed alive
    // only because nothing has *elapsed* past the window yet).
    let mid = d.tick_at(&bus, t0 + Duration::from_secs(299));
    assert_eq!(mid.target_override, Some(-3000.0));

    // Stale: no change for > 300 s ⇒ fail safe to normal ESS.
    let stale = d.tick_at(&bus, t0 + Duration::from_secs(301));
    assert_eq!(stale.target_override, None);
    assert_eq!(stale.target(CONFIGURED), CONFIGURED);
    assert_eq!(stale.feed_in_excess, 0); // feed-in reverts to follow-user
}

#[test]
fn watchdog_rearms_when_override_changes() {
    // A refreshed (changed) override re-arms the watchdog and keeps DESS active.
    let mut bus = MockBus::new();
    bus.set(HUB4, P_OVR_SETPOINT, -1000.0);
    let mut d = Dess::new(&bus);
    let t0 = Instant::now();
    assert_eq!(d.tick_at(&bus, t0).target_override, Some(-1000.0));

    // Optimiser moves to a new slot value at +290 s → re-arm.
    bus.set(HUB4, P_OVR_SETPOINT, -1500.0);
    assert_eq!(d.tick_at(&bus, t0 + Duration::from_secs(290)).target_override, Some(-1500.0));

    // +290 s again from the re-arm (580 s absolute) is still fresh, not stale.
    let out = d.tick_at(&bus, t0 + Duration::from_secs(580));
    assert_eq!(out.target_override, Some(-1500.0));
}

#[test]
fn watchdog_ignores_absent_override() {
    // With no override ever present, the watchdog stays disarmed and the fallback
    // path is steady regardless of elapsed time (never spuriously "reverts").
    let bus = MockBus::new();
    let mut d = Dess::new(&bus);
    let t0 = Instant::now();
    let a = d.tick_at(&bus, t0);
    let b = d.tick_at(&bus, t0 + Duration::from_secs(10_000));
    assert_eq!(a, b);
    assert_eq!(b.target_override, None);
    assert_eq!(b.feed_in_excess, 0);
}
