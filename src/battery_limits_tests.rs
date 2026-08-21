use super::*;
use crate::control::clamp;
use std::collections::HashMap;

const TEST_BMS: &str = "com.victronenergy.battery.test";

/// In-memory Bus for off-host regression / unit tests. No zbus, no device.
struct MockBus {
    f: HashMap<(String, String), f64>,
    s: HashMap<(String, String), String>,
    services: Vec<String>,
}
impl MockBus {
    fn new() -> Self {
        MockBus { f: HashMap::new(), s: HashMap::new(), services: vec![TEST_BMS.into()] }
    }
    fn set(&mut self, svc: &str, path: &str, v: f64) {
        self.f.insert((svc.into(), path.into()), v);
    }
    fn set_str(&mut self, svc: &str, path: &str, v: &str) {
        self.s.insert((svc.into(), path.into()), v.into());
    }
    fn clear(&mut self, svc: &str, path: &str) {
        self.f.remove(&(svc.into(), path.into()));
    }
}
impl Bus for MockBus {
    fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
        self.f.get(&(svc.into(), path.into())).copied()
    }
    fn get_str(&self, svc: &str, path: &str) -> Option<String> {
        self.s.get(&(svc.into(), path.into())).cloned()
    }
    fn list_services(&self, prefix: &str) -> Vec<String> {
        self.services.iter().filter(|s| s.starts_with(prefix)).cloned().collect()
    }
    fn set_i32(&self, _: &str, _: &str, _: i32) -> Result<(), String> { Ok(()) }
    fn set_f64(&self, _: &str, _: &str, _: f64) -> Result<(), String> { Ok(()) }
}

/// A regular-mode (DVCC + fw>0x414) scenario with all broker inputs populated.
fn regular(dcl: f64, v: f64, disch_pct: f64) -> MockBus {
    let mut b = MockBus::new();
    b.set(SYS, P_DVCC, 1.0);
    b.set(VEBUS, P_FIRMWARE, 1376.0); // 0x560 > 0x414 => regular
    b.set(VEBUS, P_DC_VOLTAGE, v);
    b.set(VEBUS, P_SUSTAIN, 0.0);
    b.set(VEBUS, P_MULTI_MAXCHG_I, 140.0);
    b.set_str(SYS, P_ACTIVE_BMS, TEST_BMS);
    b.set(SYS, P_PV_CURRENT, 0.0);
    b.set(TEST_BMS, P_BMS_CCL, 280.0);
    b.set(TEST_BMS, P_BMS_DCL, dcl);
    b.set(SETTINGS, P_MAXCHARGE, -1.0);
    b.set(SETTINGS, P_MAXDISCHARGE, -1.0);
    b.set(SETTINGS, P_MAXCHARGE_PCT, 100.0);
    b.set(SETTINGS, P_MAXDISCHARGE_PCT, disch_pct);
    b.set(SETTINGS, P_BL_STATE, 0.0);
    b.set(SETTINGS, P_BL_MINSOC, 10.0);
    b
}

// ---- broker math (faithfulness to the RE + live validation) ----

#[test]
fn regular_discharge_matches_live_to_the_watt() {
    let bus = regular(280.0, 52.44, 100.0);
    let mut br = BatteryBroker::new(&bus);
    let tr = br.tick(&bus);
    // 280 A * 52.44 V * 0.9 = 13214.9 W (the live-validated value)
    assert!((tr.published.max_discharge_power - 13214.9).abs() < 1.0,
            "discharge = {}", tr.published.max_discharge_power);
    // regular mode leaves charge to DVCC -> invalid (NaN, published as [])
    assert!(tr.published.max_charge_power.is_nan());
}

#[test]
fn legacy_discharge_applies_7a_offset() {
    let mut bus = regular(100.0, 50.0, 100.0);
    bus.set(SYS, P_DVCC, 0.0); // DVCC off => legacy
    let mut br = BatteryBroker::new(&bus);
    let tr = br.tick(&bus);
    // legacy: (100 - 7) * 50 * 0.9 = 4185
    assert!((tr.published.max_discharge_power - 4185.0).abs() < 1.0,
            "discharge = {}", tr.published.max_discharge_power);
}

#[test]
fn sustain_zeroes_discharge() {
    let mut bus = regular(280.0, 52.44, 100.0);
    bus.set(VEBUS, P_SUSTAIN, 1.0);
    let mut br = BatteryBroker::new(&bus);
    assert_eq!(br.tick(&bus).published.max_discharge_power, 0.0);
}

#[test]
fn discharged_state_zeroes_enforced_but_not_published() {
    let mut bus = regular(280.0, 52.44, 100.0);
    bus.set(SETTINGS, P_BL_STATE, 8.0); // force-charge state
    let mut br = BatteryBroker::new(&bus);
    let tr = br.tick(&bus);
    assert_eq!(tr.enforced.max_discharge_power, 0.0);
    assert!(tr.published.max_discharge_power > 10000.0); // published stays at BMS value
}

#[test]
fn dcl_drop_is_tracked_down() {
    // The safety fix: if the BMS lowers its limit, the clamp follows it down.
    let bus = regular(20.0, 52.44, 100.0);
    let mut br = BatteryBroker::new(&bus);
    let tr = br.tick(&bus);
    // 20 A * 52.44 * 0.9 = 943.9 W
    assert!((tr.published.max_discharge_power - 943.9).abs() < 2.0,
            "discharge = {}", tr.published.max_discharge_power);
}

#[test]
fn bms_alarm_dcl_zero_blocks_discharge_while_present() {
    // A BMS asserting a protection alarm typically keeps publishing but drops
    // DCL to 0 — that must hard-block discharge (distinct from disappearing).
    let mut bus = regular(280.0, 52.44, 100.0);
    let mut br = BatteryBroker::new(&bus);
    let _ = br.tick(&bus); // normal first
    bus.set(TEST_BMS, P_BMS_DCL, 0.0);
    let tr = br.tick(&bus);
    assert_eq!(tr.published.max_discharge_power, 0.0);
    assert_eq!(tr.enforced.max_discharge_power, 0.0);
    let (lo, hi) = clamp_bounds(tr.enforced, -7030.0, 7030.0);
    assert_eq!(lo, 0.0, "no discharge may be commanded during the alarm");
    assert!(hi > 0.0, "charge direction stays available");
}

#[test]
fn bms_disappearing_hard_blocks_discharge_after_latch() {
    let mut bus = regular(280.0, 52.44, 100.0);
    let mut br = BatteryBroker::new(&bus);
    let _ = br.tick(&bus); // latch DCL finite
    bus.clear(TEST_BMS, P_BMS_DCL); // BMS drops off
    let tr = br.tick(&bus);
    assert_eq!(tr.published.max_discharge_power, 0.0); // latched + None => hard block
}

// ---- SAFETY MARGIN: never over-generate design capacity ----

#[test]
fn envelope_caps_unlimited_discharge() {
    // Even if the broker returns unlimited (NaN), the design-capacity envelope
    // must cap the command -> no over-generation.
    let cap = crate::HARD_CLAMP_W;
    let enforced = Limits { max_charge_power: f64::NAN, max_discharge_power: f64::NAN };
    let (lo, hi) = clamp_bounds(enforced, -cap, cap);
    assert_eq!(lo, -cap);
    assert_eq!(hi, cap);
    assert!(lo.is_finite() && hi.is_finite());
}

#[test]
fn envelope_caps_huge_bms_limit() {
    let cap = crate::HARD_CLAMP_W;
    let enforced = Limits { max_charge_power: 20000.0, max_discharge_power: 13214.9 };
    let (lo, hi) = clamp_bounds(enforced, -cap, cap);
    assert_eq!(lo, -cap); // BMS 13 kW clamped to design capacity
    assert_eq!(hi, cap);
}

#[test]
fn command_never_exceeds_capacity_property() {
    // Sweep enforced limits (incl. NaN/negatives) and assert |lo|,|hi| <= capacity.
    let cap = crate::HARD_CLAMP_W;
    for &d in &[0.0, 500.0, 7030.0, 13214.9, 1e9, f64::NAN, -100.0] {
        for &c in &[0.0, 500.0, 20000.0, f64::NAN, -100.0] {
            let (lo, hi) = clamp_bounds(
                Limits { max_charge_power: c, max_discharge_power: d },
                -cap, cap,
            );
            assert!(lo >= -cap - 1e-9 && lo <= 0.0 + 1e-9, "lo={lo} d={d} c={c}");
            assert!(hi <= cap + 1e-9 && hi >= 0.0 - 1e-9, "hi={hi} d={d} c={c}");
        }
    }
}

// ---- GOLDEN REPLAY: real captured snapshots vs the stock ESS loop's actual output ----

#[test]
fn golden_replay_matches_hub4_maxdischarge() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/golden-sample.csv");
    let data = match std::fs::read_to_string(path) {
        Ok(d) => d,
        Err(_) => return, // fixture optional; skip if absent
    };
    let mut lines = data.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let ix = |n: &str| header.iter().position(|h| *h == n).expect(n);
    let (i_dvcc, i_fw, i_vdc, i_sus, i_mchg, i_ccl, i_dcl, i_pv, i_smc, i_smd, i_cpct,
         i_dpct, i_bls, i_blm, i_h4d) = (
        ix("dvcc"), ix("fw"), ix("vdc"), ix("sustain"), ix("multi_maxchg_i"),
        ix("bms_ccl"), ix("bms_dcl"), ix("pv_current"), ix("set_maxchg"),
        ix("set_maxdis"), ix("set_maxchg_pct"), ix("set_maxdis_pct"),
        ix("bl_state"), ix("bl_minsoc"), ix("h4_maxdis"),
    );
    let mut broker: Option<BatteryBroker> = None;
    let mut checked = 0;
    let mut worst = 0.0f64;
    for line in lines {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < header.len() {
            continue;
        }
        let num = |i: usize| f[i].parse::<f64>().ok();
        let truthy = |i: usize| f[i] == "true" || f[i] == "1";
        let mut bus = MockBus::new();
        bus.set(SYS, P_DVCC, if truthy(i_dvcc) { 1.0 } else { 0.0 });
        if let Some(x) = num(i_fw) { bus.set(VEBUS, P_FIRMWARE, x); }
        if let Some(x) = num(i_vdc) { bus.set(VEBUS, P_DC_VOLTAGE, x); }
        bus.set(VEBUS, P_SUSTAIN, if truthy(i_sus) { 1.0 } else { 0.0 });
        if let Some(x) = num(i_mchg) { bus.set(VEBUS, P_MULTI_MAXCHG_I, x); }
        bus.set_str(SYS, P_ACTIVE_BMS, TEST_BMS);
        if let Some(x) = num(i_ccl) { bus.set(TEST_BMS, P_BMS_CCL, x); }
        if let Some(x) = num(i_dcl) { bus.set(TEST_BMS, P_BMS_DCL, x); }
        bus.set(SYS, P_PV_CURRENT, num(i_pv).unwrap_or(0.0));
        bus.set(SETTINGS, P_MAXCHARGE, num(i_smc).unwrap_or(-1.0));
        bus.set(SETTINGS, P_MAXDISCHARGE, num(i_smd).unwrap_or(-1.0));
        bus.set(SETTINGS, P_MAXCHARGE_PCT, num(i_cpct).unwrap_or(100.0));
        bus.set(SETTINGS, P_MAXDISCHARGE_PCT, num(i_dpct).unwrap_or(100.0));
        bus.set(SETTINGS, P_BL_STATE, num(i_bls).unwrap_or(0.0));
        bus.set(SETTINGS, P_BL_MINSOC, num(i_blm).unwrap_or(0.0));

        if broker.is_none() {
            broker = Some(BatteryBroker::new(&bus));
        }
        let tr = broker.as_mut().unwrap().tick(&bus);
        if let Some(golden) = num(i_h4d) {
            let ours = tr.published.max_discharge_power;
            let err = (ours - golden).abs();
            worst = worst.max(err);
            // 50 W tolerance: Vdc drifts between our read instant and hub4's compute.
            assert!(err < 50.0, "replay mismatch: ours={ours:.1} hub4={golden:.1} (Δ{err:.1})");
            checked += 1;
        }
    }
    assert!(checked > 0, "no golden rows had a hub4 /MaxDischargePower to check");
    eprintln!("golden replay: {checked} rows matched hub4 (worst Δ = {worst:.1} W)");
}

// ---- adversarial-review regression tests ----

#[test]
fn clamp_is_nan_safe_and_non_inverting() {
    // Finding 2: a non-finite command must become idle, never be written as garbage.
    assert_eq!(clamp(f64::NAN, -7030.0, 7030.0), 0.0);
    assert_eq!(clamp(f64::INFINITY, -7030.0, 7030.0), 0.0);
    assert_eq!(clamp(f64::NEG_INFINITY, -7030.0, 7030.0), 0.0);
    // Finding 1 defense: inverted bounds collapse safely (no panic, stays finite).
    let r = clamp(1000.0, -100.0, -500.0);
    assert!(r.is_finite() && r == -100.0);
}

#[test]
fn negative_charge_limit_is_sanitized_to_zero() {
    // Finding 1 root: a negative BMS CCL must NOT become a negative charge bound.
    let mut bus = regular(280.0, 52.44, 100.0);
    bus.set(SYS, P_DVCC, 0.0); // legacy path computes charge from CCL
    bus.set(TEST_BMS, P_BMS_CCL, -100.0);
    let mut br = BatteryBroker::new(&bus);
    let tr = br.tick(&bus);
    assert_eq!(tr.published.max_charge_power, 0.0);
    let (_, hi) = clamp_bounds(tr.enforced, -7030.0, 7030.0);
    assert!(hi >= 0.0);
}

#[test]
fn managed_install_blocks_discharge_before_bms_latches() {
    // Finding 3: regular mode (BMS expected) with no DCL yet -> block, not unlimited.
    let mut bus = regular(280.0, 52.44, 100.0);
    bus.clear(TEST_BMS, P_BMS_DCL);
    let mut br = BatteryBroker::new(&bus);
    assert_eq!(br.tick(&bus).published.max_discharge_power, 0.0);
}

#[test]
fn get_bool_treats_nan_as_false() {
    // Finding 5.
    let mut bus = MockBus::new();
    bus.set("svc", "/p", f64::NAN);
    assert_eq!(bus.get_bool("svc", "/p"), Some(false));
    bus.set("svc", "/q", 1.0);
    assert_eq!(bus.get_bool("svc", "/q"), Some(true));
}

#[test]
fn mode_is_sticky_on_read_failure() {
    // Finding 6: a dropped DVCC/fw read must not flip the mode for a tick.
    let bus_ok = regular(280.0, 52.44, 100.0); // regular
    let mut br = BatteryBroker::new(&bus_ok);
    let _ = br.tick(&bus_ok);
    let mut bus_bad = regular(280.0, 52.44, 100.0);
    bus_bad.clear(SYS, P_DVCC);
    bus_bad.clear(VEBUS, P_FIRMWARE);
    // Still regular -> charge stays NaN (defers to DVCC), not a legacy CCL value.
    assert!(br.tick(&bus_bad).published.max_charge_power.is_nan());
}
