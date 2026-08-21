use super::*;
use crate::testbus::MockBus;
use std::cell::RefCell;
use std::collections::HashMap;

/// A Bus that records every `set_f64` so live-mode writes can be asserted exactly
/// (the shared `MockBus` treats writes as no-ops).
struct RecordingBus {
    f: HashMap<(String, String), f64>,
    writes: RefCell<Vec<(String, String, f64)>>,
}
impl RecordingBus {
    fn new() -> Self {
        RecordingBus { f: HashMap::new(), writes: RefCell::new(Vec::new()) }
    }
    fn set(&mut self, svc: &str, path: &str, v: f64) {
        self.f.insert((svc.into(), path.into()), v);
    }
    fn last(&self, svc: &str, path: &str) -> Option<f64> {
        self.writes
            .borrow()
            .iter()
            .rev()
            .find(|(s, p, _)| s == svc && p == path)
            .map(|(_, _, v)| *v)
    }
    fn write_count(&self) -> usize {
        self.writes.borrow().len()
    }
}
impl Bus for RecordingBus {
    fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
        self.f.get(&(svc.into(), path.into())).copied()
    }
    fn get_str(&self, _: &str, _: &str) -> Option<String> {
        None
    }
    fn list_services(&self, _: &str) -> Vec<String> {
        Vec::new()
    }
    fn set_i32(&self, _: &str, _: &str, _: i32) -> Result<(), String> {
        Ok(())
    }
    fn set_f64(&self, svc: &str, path: &str, v: f64) -> Result<(), String> {
        self.writes.borrow_mut().push((svc.into(), path.into(), v));
        Ok(())
    }
}

// ---- DORMANT: AcInputLimit = -1 -> no-op, path untouched (spec §5) ----

#[test]
fn dormant_minus_one_setting_reads_as_none() {
    let mut bus = MockBus::new();
    bus.set(SETTINGS, P_AC_INPUT_LIMIT, -1.0);
    assert_eq!(read_ac_input_limit(&bus), None);
}

#[test]
fn dormant_leaves_every_phase_untouched() {
    let mut sh = ShoreLimiter::new(1);
    // None (unset/-1/NaN) -> dormant regardless of currents.
    let out = sh.tick(None, false, &[Some(10.0)], &[Some(10.0)], false);
    assert!(!out.active);
    assert_eq!(out.writes, vec![None]);
    assert!(out.overruled_shore_limit_a.is_nan());
}

#[test]
fn dormant_write_touches_nothing() {
    let bus = RecordingBus::new();
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(None, false, &[Some(10.0)], &[Some(10.0)], false);
    sh.write(&bus, &out).unwrap();
    assert_eq!(bus.write_count(), 0); // never clears OverruledShoreLimit
}

#[test]
fn nan_limit_is_dormant_not_a_zero_clamp() {
    // SAFETY: a NaN AcInputLimit must NOT impose (which could clamp AC-in near 0).
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(f64::NAN), false, &[Some(30.0)], &[Some(30.0)], false);
    assert!(!out.active);
    assert_eq!(out.writes, vec![None]);
}

// ---- ACTIVE: IIR converges toward the configured Amps ----

#[test]
fn active_iir_converges_to_configured_amps() {
    // meter_I == multi_I (same physical current, two sensors) => error == limit,
    // so state converges to the configured limit (in Amps).
    let limit = 16.0;
    let load = 10.0;
    let mut sh = ShoreLimiter::new(1);
    let mut last = 0.0;
    for _ in 0..40 {
        let out = sh.tick(Some(limit), false, &[Some(load)], &[Some(load)], false);
        assert!(out.active);
        match out.writes[0] {
            Some(ShoreWrite::Impose(a)) => last = a,
            other => panic!("expected Impose, got {other:?}"),
        }
    }
    assert!((last - limit).abs() < 0.05, "converged to {last}, want {limit}");
    // exposed scalar mirrors the L1 impose value
    let out = sh.tick(Some(limit), false, &[Some(load)], &[Some(load)], false);
    assert!((out.overruled_shore_limit_a - limit).abs() < 0.05);
}

/// Extract the imposed Amps from a phase decision, panicking on anything else.
fn imposed(w: Option<ShoreWrite>) -> f64 {
    match w {
        Some(ShoreWrite::Impose(a)) => a,
        other => panic!("expected Impose, got {other:?}"),
    }
}

#[test]
fn active_first_step_is_the_expected_iir_fraction() {
    // From a cold (unseeded=0) state: state1 = 0.2 * ((16 - 10) + 10) = 0.2*16 = 3.2
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
    assert!((imposed(out.writes[0]) - 3.2).abs() < 1e-9);
}

#[test]
fn state_persists_across_ticks() {
    // Two identical ticks must NOT give the same output — the filter accumulates.
    let mut sh = ShoreLimiter::new(1);
    let a = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
    let b = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
    assert_ne!(a.writes[0], b.writes[0]);
}

#[test]
fn missing_phase_data_leaves_that_phase_untouched() {
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(16.0), false, &[None], &[Some(10.0)], false);
    assert!(out.active); // subsystem is active...
    assert_eq!(out.writes, vec![None]); // ...but this phase had no data
}

#[test]
fn active_writes_amps_to_vebus_l1() {
    let mut bus = RecordingBus::new();
    bus.set(VEBUS, &p_active_in_i(1), 10.0);
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick_from_bus(&bus, Some(16.0), false, false, true, false);
    sh.write(&bus, &out).unwrap();
    // first step from cold: 3.2 A written to /Hub4/L1/OverruledShoreLimit
    let written = bus.last(VEBUS, &p_overruled(1)).expect("L1 written");
    assert!((written - 3.2).abs() < 1e-9, "wrote {written}");
}

// ---- meter_present gate (spec §4.4) ----

#[test]
fn external_meter_present_skips_write_loop() {
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(16.0), true, &[Some(10.0)], &[Some(10.0)], false);
    assert!(!out.active);
    assert_eq!(out.writes, vec![None]);
}

// ---- release conditions: sentinel 3270 ----

#[test]
fn release_writes_the_sentinel() {
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], true);
    assert_eq!(out.writes[0], Some(ShoreWrite::Release));
    assert_eq!(out.writes[0].unwrap().value(), SHORE_UNLIMITED);
    assert_eq!(out.overruled_shore_limit_a, SHORE_UNLIMITED);
}

#[test]
fn release_still_advances_the_filter() {
    // Release chooses the sentinel to WRITE, but the IIR state must keep tracking,
    // so a later impose resumes from the right place (matches the stock ESS loop: the
    // filter update precedes the impose/release choice).
    let mut sh = ShoreLimiter::new(1);
    let _ = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], true);
    let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
    // second step from state=3.2: 0.8*3.2 + 0.2*16 = 5.76
    assert!((imposed(out.writes[0]) - 5.76).abs() < 1e-9);
}

#[test]
fn force_charge_releases() {
    assert!(should_release(false, true, true, false));
    assert!(should_release(true, true, true, false)); // even with always-peakshave
}

#[test]
fn below_min_soc_non_always_releases_but_always_holds() {
    assert!(should_release(false, false, false, false)); // below SOC, not "always"
    assert!(!should_release(true, false, false, false)); // "always" keeps shaving
}

#[test]
fn above_min_soc_imposes() {
    assert!(!should_release(false, true, false, false));
}

#[test]
fn negative_state_forces_release() {
    assert!(should_release(true, true, false, true));
    assert!(!should_release(true, true, false, false));
}

#[test]
fn any_phase_negative_tracks_filter_state() {
    let mut sh = ShoreLimiter::new(1);
    // Drive the state negative: limit=0, big positive meter current, zero multi.
    // error = (0 - 50) + 0 = -50 -> state goes negative.
    let _ = sh.tick(Some(0.0), false, &[Some(50.0)], &[Some(0.0)], false);
    assert!(sh.any_phase_negative());
}

#[test]
fn impose_value_never_negative() {
    // Even a negative filter state imposes max(0, state) = 0, never a negative A.
    let mut sh = ShoreLimiter::new(1);
    let out = sh.tick(Some(0.0), false, &[Some(50.0)], &[Some(0.0)], false);
    match out.writes[0] {
        Some(ShoreWrite::Impose(a)) => assert!(a >= 0.0, "imposed {a}"),
        other => panic!("expected Impose, got {other:?}"),
    }
}

// ---- multiphase faithfulness ----

#[test]
fn three_phase_writes_all_three() {
    let mut sh = ShoreLimiter::new(3);
    let out = sh.tick(
        Some(16.0),
        false,
        &[Some(10.0), Some(11.0), Some(12.0)],
        &[Some(10.0), Some(11.0), Some(12.0)],
        false,
    );
    assert_eq!(out.writes.len(), 3);
    assert!(out.writes.iter().all(|w| matches!(w, Some(ShoreWrite::Impose(_)))));
}
