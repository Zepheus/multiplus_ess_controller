use super::*;
use crate::testbus::MockBus;

const GRID_SVC: &str = "com.victronenergy.grid.ttyUSB0";

/// A steady grid-mode install with our config (GMR=1, RWGM=0, Hub4Mode=1),
/// AC input connected, and one grid meter reporting `power` W.
fn grid_present(power: f64) -> MockBus {
    let mut b = MockBus::new();
    b.set(SETTINGS, P_HUB4MODE, 1.0);
    b.set(SETTINGS, P_RUN_WITHOUT_METER, 0.0);
    b.set(SETTINGS, P_GRID_METER_REQUIRED, 1.0);
    b.set(SYS, P_SOURCE, 1.0); // grid
    b.set(VEBUS, P_ACTIVE_INPUT, 0.0); // AC connected
    b.set(VEBUS, P_ACTIVEIN, -50.0); // Multi AC-in valid
    b.add_service(GRID_SVC);
    b.set(GRID_SVC, P_METER_POWER, power);
    b
}

/// Same config but with the grid meter removed from the bus.
fn make_absent(b: &mut MockBus) {
    b.services.clear();
    b.clear(GRID_SVC, P_METER_POWER);
    b.clear(SYS, P_GRID); // no system-republished grid power either
}

// ---- presence: present, then absent ----

#[test]
fn meter_present_regulates_on_grid_meter() {
    let bus = grid_present(230.0);
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, false);
    assert!(out.usable);
    assert!(out.should_write_setpoint);
    assert_eq!(out.regulate, Regulation::OnGridMeter);
    assert_eq!(out.presence, Presence::Ok);
    assert!(!out.alarm_no_grid_meter);
    assert_eq!(g.meter_service(), Some(GRID_SVC));
}

#[test]
fn meter_present_then_absent_stops_writing() {
    let mut bus = grid_present(230.0);
    let mut g = GridMeterGuard::new();
    assert!(g.tick(&bus, false).should_write_setpoint);
    make_absent(&mut bus);
    let out = g.tick(&bus, false);
    assert!(!out.usable);
    assert!(!out.should_write_setpoint); // required + absent ⇒ Skip ⇒ passthru
    assert_eq!(out.regulate, Regulation::Skip);
    assert_eq!(out.presence, Presence::Grace); // grace just started, no alarm yet
    assert!(!out.alarm_no_grid_meter);
}

// ---- the 120 s (48-tick) alarm countdown ----

#[test]
fn alarm_fires_after_exactly_48_ticks() {
    let mut bus = grid_present(230.0);
    let mut g = GridMeterGuard::new();
    g.tick(&bus, false); // meter present: grace = 48
    make_absent(&mut bus);
    // 47 absent ticks stay in grace (no alarm)…
    for k in 1..=47 {
        let out = g.tick(&bus, false);
        assert_eq!(out.presence, Presence::Grace, "tick {k}");
        assert!(!out.alarm_no_grid_meter, "tick {k}");
    }
    // …the 48th raises the alarm and holds it.
    let out = g.tick(&bus, false);
    assert_eq!(out.presence, Presence::Missing);
    assert!(out.alarm_no_grid_meter);
    assert!(g.tick(&bus, false).alarm_no_grid_meter, "alarm stays latched while absent");
}

#[test]
fn returning_meter_clears_the_alarm_next_tick() {
    let mut bus = grid_present(230.0);
    let mut g = GridMeterGuard::new();
    g.tick(&bus, false);
    make_absent(&mut bus);
    for _ in 0..48 {
        g.tick(&bus, false);
    }
    assert!(g.tick(&bus, false).alarm_no_grid_meter); // firmly in alarm
    // Meter comes back (fresh value) ⇒ alarm clears immediately, grace reloads.
    let bus = grid_present(180.0);
    let out = g.tick(&bus, false);
    assert!(!out.alarm_no_grid_meter);
    assert_eq!(out.presence, Presence::Ok);
    assert!(out.should_write_setpoint);
}

#[test]
fn no_ac_input_never_alarms() {
    let mut bus = grid_present(230.0);
    make_absent(&mut bus);
    bus.set(VEBUS, P_ACTIVE_INPUT, ACTIVEIN_DISCONNECTED as f64);
    let mut g = GridMeterGuard::new();
    for _ in 0..60 {
        let out = g.tick(&bus, false);
        assert_eq!(out.presence, Presence::NotApplicable);
        assert!(!out.alarm_no_grid_meter);
    }
}

// ---- the frozen-but-present meter (our improvement) ----

#[test]
fn frozen_meter_detected_as_stale_and_stops_writing() {
    // Meter service stays up, value never changes ⇒ must degrade to "absent".
    let bus = grid_present(500.0); // constant 500 W every tick
    let mut g = GridMeterGuard::new();
    // First STALE_TICKS reads still look usable (value could be steady briefly).
    for _ in 0..STALE_TICKS {
        let out = g.tick(&bus, false);
        assert!(out.usable, "not yet declared stale");
        assert!(!out.stale);
    }
    // Once unchanged for STALE_TICKS consecutive ticks it is declared frozen.
    let out = g.tick(&bus, false);
    assert!(out.stale, "frozen meter must be flagged");
    assert!(!out.usable);
    assert!(!out.should_write_setpoint); // required + frozen ⇒ Skip ⇒ passthru
    assert_eq!(out.regulate, Regulation::Skip);
}

#[test]
fn moving_meter_never_flagged_stale() {
    let mut g = GridMeterGuard::new();
    for k in 0..30 {
        let bus = grid_present(200.0 + k as f64); // value advances each tick
        let out = g.tick(&bus, false);
        assert!(out.usable && !out.stale, "tick {k}");
        assert_eq!(out.regulate, Regulation::OnGridMeter);
    }
}

#[test]
fn frozen_guard_disabled_reproduces_stock_blindness() {
    // Parity mode: stock does NOT detect a frozen meter.
    let bus = grid_present(500.0);
    let mut g = GridMeterGuard::new().with_frozen_guard(false);
    for _ in 0..(STALE_TICKS + 20) {
        let out = g.tick(&bus, false);
        assert!(out.usable && !out.stale);
        assert_eq!(out.regulate, Regulation::OnGridMeter);
    }
}

// ---- RunWithoutGridMeter × GridMeterRequired combinations (§3a) ----

#[test]
fn rwgm1_gmr1_absent_runs_on_multi_and_never_alarms() {
    let mut bus = grid_present(230.0);
    make_absent(&mut bus);
    bus.set(SETTINGS, P_RUN_WITHOUT_METER, 1.0);
    let mut g = GridMeterGuard::new();
    for _ in 0..60 {
        let out = g.tick(&bus, false);
        assert_eq!(out.regulate, Regulation::OnMultiInput);
        assert!(out.should_write_setpoint);
        assert_eq!(out.presence, Presence::NotApplicable);
        assert!(!out.alarm_no_grid_meter);
        assert!(out.run_without_meter);
    }
}

#[test]
fn rwgm0_gmr0_absent_runs_on_multi_but_still_alarms() {
    // The subtle row: GridMeterRequired only gates the Skip branch, NOT the
    // alarm. Optional meter ⇒ keep regulating on Multi in, yet alarm after 48.
    let mut bus = grid_present(230.0);
    make_absent(&mut bus);
    bus.set(SETTINGS, P_GRID_METER_REQUIRED, 0.0);
    let mut g = GridMeterGuard::new();
    for _ in 0..48 {
        let out = g.tick(&bus, false);
        assert_eq!(out.regulate, Regulation::OnMultiInput);
        assert!(out.should_write_setpoint);
    }
    let out = g.tick(&bus, false);
    assert!(out.alarm_no_grid_meter, "alarm fires even when meter optional");
    assert_eq!(out.regulate, Regulation::OnMultiInput); // still writing
}

#[test]
fn rwgm0_gmr1_absent_skips_and_alarms() {
    let mut bus = grid_present(230.0);
    make_absent(&mut bus);
    let mut g = GridMeterGuard::new(); // GMR=1, RWGM=0 (default fixture)
    for _ in 0..48 {
        assert_eq!(g.tick(&bus, false).regulate, Regulation::Skip);
    }
    assert!(g.tick(&bus, false).alarm_no_grid_meter);
}

#[test]
fn genset_regulates_without_meter_and_never_alarms() {
    let mut bus = grid_present(230.0);
    make_absent(&mut bus);
    bus.set(SYS, P_SOURCE, SOURCE_GENERATOR as f64);
    let mut g = GridMeterGuard::new();
    for _ in 0..60 {
        let out = g.tick(&bus, false);
        assert_eq!(out.regulate, Regulation::OnMultiInput);
        assert!(out.should_write_setpoint);
        assert!(!out.alarm_no_grid_meter);
    }
}

// ---- fail-safe defaults ----

#[test]
fn hub4mode_external_is_inert() {
    let mut bus = grid_present(230.0);
    bus.set(SETTINGS, P_HUB4MODE, HUB4MODE_EXTERNAL as f64);
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, false);
    assert_eq!(out.regulate, Regulation::Skip);
    assert_eq!(out.presence, Presence::NotApplicable);
    assert!(!out.alarm_no_grid_meter);
}

#[test]
fn hub4mode_external_is_not_inert_when_we_own_mode3() {
    // PROMOTION-BLOCKER regression (found in pre-live review): in the takeover
    // stage WE set Hub4Mode=EXTERNAL, and the guard treating that as "inert"
    // stopped every setpoint write within one presence tick of taking over.
    let mut bus = grid_present(230.0);
    bus.set(SETTINGS, P_HUB4MODE, HUB4MODE_EXTERNAL as f64);
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, true); // mode3_is_ours: takeover stage
    assert!(out.should_write_setpoint, "our own mode-3 must not gate our writes");
    assert_eq!(out.regulate, Regulation::OnGridMeter);
    // And a FOREIGN mode 3 (not ours) still goes inert, matching stock.
    let mut g2 = GridMeterGuard::new();
    assert_eq!(g2.tick(&bus, false).regulate, Regulation::Skip);
}

#[test]
fn unreadable_multi_ac_in_skips() {
    let mut bus = grid_present(230.0);
    bus.clear(VEBUS, P_ACTIVEIN); // Multi AC-in unreadable ⇒ never write
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, false);
    assert!(!out.should_write_setpoint);
    assert_eq!(out.regulate, Regulation::Skip);
}

#[test]
fn unreadable_settings_default_to_safe_skip() {
    // Empty bus except a valid Multi AC-in: no meter, no settings. Defaults
    // (Hub4Mode=3 inert / GMR=true / RWGM=false) must NOT write the setpoint.
    let mut bus = MockBus::new();
    bus.set(VEBUS, P_ACTIVEIN, -50.0);
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, false);
    assert!(!out.should_write_setpoint);
    assert!(!out.usable);
}

#[test]
fn system_grid_fallback_counts_as_present() {
    // No grid.* service, but systemcalc republishes grid power ⇒ present.
    let mut bus = grid_present(230.0);
    bus.services.clear();
    bus.clear(GRID_SVC, P_METER_POWER);
    bus.set(SYS, P_GRID, 210.0);
    let mut g = GridMeterGuard::new();
    let out = g.tick(&bus, false);
    assert!(out.usable);
    assert_eq!(out.regulate, Regulation::OnGridMeter);
    assert_eq!(g.meter_service(), None); // adopted via system fallback
}
