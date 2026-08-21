use super::*;
use std::cell::RefCell;
use std::collections::HashMap;

/// In-memory Bus that ALSO records every write, so we can assert write-if-changed
/// and — critically — that `/Hub4/Sustain` is never written. Self-contained (no
/// dependency on `mod testbus`, which is not wired into the crate).
struct RecBus {
    f: HashMap<(String, String), f64>,
    s: HashMap<(String, String), String>,
    writes: RefCell<Vec<(String, String, f64)>>,
}
impl RecBus {
    fn new() -> Self {
        RecBus { f: HashMap::new(), s: HashMap::new(), writes: RefCell::new(Vec::new()) }
    }
    fn set(&mut self, svc: &str, path: &str, v: f64) {
        self.f.insert((svc.into(), path.into()), v);
    }
    fn clear(&mut self, svc: &str, path: &str) {
        self.f.remove(&(svc.into(), path.into()));
    }
    fn writes_to(&self, svc: &str, path: &str) -> Vec<f64> {
        self.writes
            .borrow()
            .iter()
            .filter(|(s, p, _)| s == svc && p == path)
            .map(|(_, _, v)| *v)
            .collect()
    }
    fn write_count(&self) -> usize {
        self.writes.borrow().len()
    }
}
impl Bus for RecBus {
    fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
        self.f.get(&(svc.into(), path.into())).copied()
    }
    fn get_str(&self, svc: &str, path: &str) -> Option<String> {
        self.s.get(&(svc.into(), path.into())).cloned()
    }
    fn list_services(&self, prefix: &str) -> Vec<String> {
        let _ = prefix;
        Vec::new()
    }
    fn set_i32(&self, svc: &str, path: &str, v: i32) -> Result<(), String> {
        self.writes.borrow_mut().push((svc.into(), path.into(), v as f64));
        Ok(())
    }
    fn set_f64(&self, svc: &str, path: &str, v: f64) -> Result<(), String> {
        self.writes.borrow_mut().push((svc.into(), path.into(), v));
        Ok(())
    }
}

/// Baseline scenario: normal operation, all firmware paths present, DVCC on,
/// battery healthy, not forcing charge. Every test tweaks from here.
fn base() -> RecBus {
    let mut b = RecBus::new();
    b.set(VEBUS, P_MULTI_STATE, 9.0); // valid
    b.set(VEBUS, P_SUSTAIN, 0.0);
    b.set(VEBUS, P_DC_VOLTAGE, 52.0);
    b.set(VEBUS, P_NUM_PHASES, 1.0);
    // firmware-support probes present:
    b.set(VEBUS, P_DNFIO, 0.0);
    b.set(VEBUS, P_TPIMFI, 0.0);
    b.set(VEBUS, &p_maxfeedin(1), 0.0);
    b.set(SYS, P_AC_SOURCE, 1.0); // grid
    b.set(SYS, P_DVCC, 1.0);
    b.set(SYS, P_CHARGE_VOLTAGE, 55.0);
    b.set(SETTINGS, P_HUB4MODE, 3.0);
    b.set(SETTINGS, P_OVERVOLTAGE_FEEDIN, 0.0);
    b.set(SETTINGS, P_ALWAYS_PEAKSHAVE, 0.0);
    b.set(SETTINGS, P_MAXFEEDIN_SETTING, -1.0); // unlimited
    b.set(SETTINGS, P_ACEXPORT_LIMIT, -1.0); // none
    b.set(SETTINGS, P_MAXDISCHARGE, -1.0); // unknown => allowed
    b.set(SETTINGS, P_BL_STATE, 0.0);
    b.set(SETTINGS, P_BL_MINSOC, 10.0);
    b
}

// ---- mode selection & the flags derived from it ----

#[test]
fn normal_operation_tracks_grid_and_opens_feedin() {
    let bus = base();
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.mode, FeedInMode::Normal);
    assert_eq!(o.disable_feedin, 0); // healthy battery, not at floor
    assert_eq!(o.target_power_is_max_feedin, 0);
    // OvervoltageFeedIn off & no override => DC excess blocked, and mode!=TPIMFI:
    assert_eq!(o.do_not_feed_in_overvoltage, 1);
    assert_eq!(o.fix_solar_offset, 1); // DVCC on
}

#[test]
fn flag_snapshot_roundtrip_restores_stock_values() {
    let mut bus = base();
    bus.set(VEBUS, P_DISABLE_FEEDIN, 1.0);
    bus.set(VEBUS, P_TPIMFI, 0.0);
    bus.set(VEBUS, "/Hub4/L1/MaxFeedInPower", 200000.0);
    // L2/L3 unreadable on this single-phase rig: must be skipped by restore.
    let snap = snapshot_flags(&bus);
    restore_flags(&bus, &snap);
    assert_eq!(bus.writes_to(VEBUS, P_DISABLE_FEEDIN), vec![1.0]);
    assert_eq!(bus.writes_to(VEBUS, P_TPIMFI), vec![0.0]);
    assert_eq!(bus.writes_to(VEBUS, "/Hub4/L1/MaxFeedInPower"), vec![200000.0]);
    assert!(bus.writes_to(VEBUS, "/Hub4/L2/MaxFeedInPower").is_empty());
    assert!(bus.writes_to(VEBUS, "/Hub4/L3/MaxFeedInPower").is_empty());
    // DNFIO present (fw probe in base()) => captured and restored; FixSolar
    // unreadable => skipped.
    assert_eq!(bus.writes_to(VEBUS, P_DNFIO), vec![0.0]);
    assert!(bus.writes_to(VEBUS, P_FIX_SOLAR).is_empty());
}

#[test]
fn bms_alarm_discharge_bound_zero_disables_feedin() {
    // A BMS protection alarm (DCL -> 0) reaches feed-in via the live broker bound,
    // NOT the settings item: feed-in must close so DC excess cannot drain the
    // battery around the setpoint clamp. Settings MaxDischargePower stays "unknown".
    let bus = base();
    let mut f = FeedIn::new(&bus);
    f.set_overrides(Overrides { discharge_bound_w: 0.0, ..Overrides::default() });
    let o = f.tick(&bus);
    assert_eq!(o.disable_feedin, 1, "alarm bound must close feed-in");

    // Bound recovers (alarm clears): feed-in reopens.
    f.set_overrides(Overrides { discharge_bound_w: f64::NAN, ..Overrides::default() });
    let o = f.tick(&bus);
    assert_eq!(o.disable_feedin, 0);

    // A finite POSITIVE bound (normal BMS limit) keeps feed-in open.
    f.set_overrides(Overrides { discharge_bound_w: 13214.9, ..Overrides::default() });
    let o = f.tick(&bus);
    assert_eq!(o.disable_feedin, 0);
}

#[test]
fn overvoltage_feedin_setting_allows_dc_excess() {
    let mut bus = base();
    bus.set(SETTINGS, P_OVERVOLTAGE_FEEDIN, 1.0);
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.do_not_feed_in_overvoltage, 0);
}

#[test]
fn keep_charged_forces_tpimfi_and_opens_feedin_path() {
    // BL state 9 (KeepCharged) => mustForceCharge; DVCC + TPIMFI support + excess NOT
    // allowed => mode 0 (TPIMFI).
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0);
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.mode, FeedInMode::Tpimfi);
    assert_eq!(o.target_power_is_max_feedin, 1);
    assert_eq!(o.disable_feedin, 0); // TPIMFI keeps the feed-in path open
    assert_eq!(o.do_not_feed_in_overvoltage, 0); // 0 in TPIMFI mode
}

#[test]
fn force_charge_without_tpimfi_support_is_force_flat() {
    let mut bus = base();
    bus.clear(VEBUS, P_TPIMFI); // old firmware: no TPIMFI
    bus.set(SETTINGS, P_BL_STATE, 8.0); // low-SOC recharge => mustForceCharge
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.mode, FeedInMode::ForceFlat);
    assert_eq!(o.target_power_is_max_feedin, 0);
}

#[test]
fn force_charge_with_excess_allowed_is_force_flat() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0);
    bus.set(SETTINGS, P_OVERVOLTAGE_FEEDIN, 1.0); // excess may be sold
    let mut f = FeedIn::new(&bus);
    assert_eq!(f.tick(&bus).mode, FeedInMode::ForceFlat);
}

#[test]
fn generator_never_backfeeds() {
    let mut bus = base();
    bus.set(SYS, P_AC_SOURCE, 2.0); // generator, DVCC + TPIMFI => TPIMFI cap
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.mode, FeedInMode::Tpimfi);
    // On a genset DoNotFeedInOvervoltage is only 0 via the mode==0 path:
    assert_eq!(o.do_not_feed_in_overvoltage, 0);
}

// ---- DisableFeedIn conditions (§4.2) ----

#[test]
fn sustain_disables_feedin_and_is_mirrored() {
    let mut bus = base();
    bus.set(VEBUS, P_SUSTAIN, 1.0); // firmware sustain -> discharge budget 0
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.disable_feedin, 1); // !can_discharge
    assert!(o.sustain);
}

#[test]
fn floor_state_disables_feedin_unless_peakshaving() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 5.0); // Discharged (floor state), not force-charge
    let mut f = FeedIn::new(&bus);
    assert_eq!(f.tick(&bus).disable_feedin, 1);

    bus.set(SETTINGS, P_ALWAYS_PEAKSHAVE, 1.0);
    let mut f2 = FeedIn::new(&bus);
    assert_eq!(f2.tick(&bus).disable_feedin, 0); // peak shaving keeps it open
}

#[test]
fn zero_discharge_budget_disables_feedin() {
    let mut bus = base();
    bus.set(SETTINGS, P_MAXDISCHARGE, 0.0); // budget 0 => cannot discharge
    let mut f = FeedIn::new(&bus);
    assert_eq!(f.tick(&bus).disable_feedin, 1);
}

#[test]
fn unknown_discharge_budget_allows_feedin() {
    // §6.8: NaN/unknown budget => can_discharge = true (matches the reference implementation).
    let bus = base(); // MAXDISCHARGE = -1 => NaN
    let mut f = FeedIn::new(&bus);
    assert_eq!(f.tick(&bus).disable_feedin, 0);
}

// ---- MaxFeedInPower: sentinel + slew (§4.6) ----

#[test]
fn maxfeedin_unlimited_writes_exact_sentinel() {
    let mut bus = base();
    bus.set(SETTINGS, P_OVERVOLTAGE_FEEDIN, 1.0); // excess_ok, both limits unlimited
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.max_feedin_power[0], MAXFEEDIN_UNLIMITED); // exact, no slew
}

#[test]
fn maxfeedin_finite_slews_up_only() {
    let mut bus = base();
    bus.set(SETTINGS, P_MAXFEEDIN_SETTING, 3000.0); // finite, single phase
    let mut f = FeedIn::new(&bus);
    // tick 1: 0 -> 3000 is an up-step => 0.6*3000 + 0.4*0 = 1800
    let o1 = f.tick(&bus);
    assert!((o1.max_feedin_power[0] - 1800.0).abs() < 1e-6);
    // tick 2: 1800 -> 3000 up-step => 0.6*3000 + 0.4*1800 = 2520
    let o2 = f.tick(&bus);
    assert!((o2.max_feedin_power[0] - 2520.0).abs() < 1e-6);
}

#[test]
fn maxfeedin_down_step_is_immediate() {
    let mut bus = base();
    bus.set(SETTINGS, P_MAXFEEDIN_SETTING, 3000.0);
    let mut f = FeedIn::new(&bus);
    let _ = f.tick(&bus); // 1800
    let _ = f.tick(&bus); // 2520
    bus.set(SETTINGS, P_MAXFEEDIN_SETTING, 500.0); // drop below current => immediate
    let o = f.tick(&bus);
    assert!((o.max_feedin_power[0] - 500.0).abs() < 1e-6);
}

#[test]
fn maxfeedin_skipped_on_old_firmware() {
    let mut bus = base();
    bus.clear(VEBUS, &p_maxfeedin(1)); // path absent
    bus.set(SETTINGS, P_MAXFEEDIN_SETTING, 3000.0);
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    o.write(&bus).unwrap();
    assert!(bus.writes_to(VEBUS, &p_maxfeedin(1)).is_empty());
}

// ---- DC-overvoltage integrator (§4.11) ----

#[test]
fn dc_overvoltage_integrator_accumulates_and_trips() {
    // Vdc above 1.01*ChargeVoltage integrates up; below resets.
    let mut bus = base();
    bus.set(SYS, P_CHARGE_VOLTAGE, 55.0); // 1.01*55 = 55.55
    bus.set(VEBUS, P_DC_VOLTAGE, 56.55); // excess = 1.0 each tick => +1.0
    let mut f = FeedIn::new(&bus);
    let o1 = f.tick(&bus);
    assert!((o1.ov_accum - 1.0).abs() < 1e-6);
    assert!(!o1.block_charge()); // 1.0 is not > 1.0
    let o2 = f.tick(&bus);
    assert!((o2.ov_accum - 2.0).abs() < 1e-6);
    assert!(o2.block_charge()); // now > trip => block charging
}

#[test]
fn dc_overvoltage_integrator_resets_below_threshold() {
    let mut bus = base();
    bus.set(SYS, P_CHARGE_VOLTAGE, 55.0);
    bus.set(VEBUS, P_DC_VOLTAGE, 56.55);
    let mut f = FeedIn::new(&bus);
    let _ = f.tick(&bus);
    let _ = f.tick(&bus); // accum = 2.0
    bus.set(VEBUS, P_DC_VOLTAGE, 54.0); // below threshold => reset
    let o = f.tick(&bus);
    assert_eq!(o.ov_accum, 0.0);
    assert!(!o.block_charge());
}

#[test]
fn dc_overvoltage_integrator_is_nan_safe() {
    let mut bus = base();
    bus.clear(SYS, P_CHARGE_VOLTAGE); // => NaN => excess NaN => reset, never accumulate NaN
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    assert_eq!(o.ov_accum, 0.0);
}

// ---- Sustain is NEVER written (the load-bearing safety invariant) ----

#[test]
fn sustain_is_never_written() {
    let mut bus = base();
    bus.set(VEBUS, P_SUSTAIN, 1.0);
    let mut f = FeedIn::new(&bus);
    // Several ticks across mode changes; write every time.
    for st in [0.0, 9.0, 8.0, 5.0, 0.0] {
        bus.set(SETTINGS, P_BL_STATE, st);
        let o = f.tick(&bus);
        o.write(&bus).unwrap();
    }
    assert!(bus.writes_to(VEBUS, P_SUSTAIN).is_empty(), "must never write /Hub4/Sustain");
    // And no write ever targeted the Sustain path on any service.
    assert!(bus.writes.borrow().iter().all(|(_, p, _)| p != P_SUSTAIN));
}

// ---- write-if-changed dedup ----

#[test]
fn writes_only_on_change() {
    // Enable excess feed-in so MaxFeedInPower takes the exact-sentinel path and is
    // stable across ticks (the finite/unlimited slew would keep changing).
    let mut bus = base();
    bus.set(SETTINGS, P_OVERVOLTAGE_FEEDIN, 1.0);
    let mut f = FeedIn::new(&bus);
    let o1 = f.tick(&bus);
    o1.write(&bus).unwrap();
    let first = bus.write_count();
    assert!(first > 0); // initial writes happen
    // Second identical tick: nothing changed => no new writes.
    let o2 = f.tick(&bus);
    o2.write(&bus).unwrap();
    assert_eq!(bus.write_count(), first, "unchanged flags must not re-write");
}

#[test]
fn changed_flag_rewrites() {
    let mut bus = base();
    let mut f = FeedIn::new(&bus);
    f.tick(&bus).write(&bus).unwrap();
    let before = bus.writes_to(VEBUS, P_DISABLE_FEEDIN).len();
    bus.set(VEBUS, P_SUSTAIN, 1.0); // flips DisableFeedIn 0 -> 1
    f.tick(&bus).write(&bus).unwrap();
    let after = bus.writes_to(VEBUS, P_DISABLE_FEEDIN);
    assert_eq!(after.len(), before + 1);
    assert_eq!(*after.last().unwrap(), 1.0);
}

// ---- gating ----

#[test]
fn invalid_multi_state_writes_nothing() {
    let mut bus = base();
    bus.clear(VEBUS, P_MULTI_STATE); // invalid => gate
    let mut f = FeedIn::new(&bus);
    let o = f.tick(&bus);
    o.write(&bus).unwrap();
    assert_eq!(bus.write_count(), 0);
}

#[test]
fn dnfio_and_tpimfi_skipped_on_old_firmware() {
    let mut bus = base();
    bus.clear(VEBUS, P_DNFIO);
    bus.clear(VEBUS, P_TPIMFI);
    let mut f = FeedIn::new(&bus);
    f.tick(&bus).write(&bus).unwrap();
    assert!(bus.writes_to(VEBUS, P_DNFIO).is_empty());
    assert!(bus.writes_to(VEBUS, P_TPIMFI).is_empty());
    // DisableFeedIn / FixSolarOffset are ungated and still written.
    assert!(!bus.writes_to(VEBUS, P_DISABLE_FEEDIN).is_empty());
    assert!(!bus.writes_to(VEBUS, P_FIX_SOLAR).is_empty());
}
