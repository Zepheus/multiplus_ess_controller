use super::*;
use crate::states::bl;
use crate::testbus::MockBus;

const TEST_BMS: &str = "com.victronenergy.battery.test";

/// Baseline: state 0 (inert), BMS present, no charge request, finite Vdc, TPIMF
/// firmware present, no configured charge ceiling.
fn base() -> MockBus {
    let mut b = MockBus::new();
    b.add_service(TEST_BMS);
    b.set_str(SYS, P_ACTIVE_BMS, TEST_BMS);
    b.set(SETTINGS, P_BL_STATE, 0.0);
    b.set(SETTINGS, P_BL_MINSOC, 10.0);
    b.set(TEST_BMS, P_CHARGE_REQUEST, 0.0);
    b.set(VEBUS, P_DC_VOLTAGE, 52.0);
    b.set(VEBUS, P_TPIMF, 1.0); // item present => firmware supports it
    b
}

// ---- discharged-set mask fidelity ----

#[test]
fn discharged_set_is_exactly_5_6_8_11_12() {
    for s in -3..=20 {
        let want = matches!(s, 5 | 6 | 8 | 11 | 12);
        assert_eq!(is_discharged_set(s), want, "state {s}");
    }
}

#[test]
fn discharge_inhibited_only_in_the_set() {
    for s in 0..=13 {
        let mut bus = base();
        bus.set(SETTINGS, P_BL_STATE, s as f64);
        let out = SocGuard::new(&bus).tick(&bus);
        if matches!(s, 5 | 6 | 8 | 11 | 12) {
            assert_eq!(out.max_discharge_w, 0.0, "state {s} should inhibit");
        } else {
            assert_eq!(out.max_discharge_w, f64::INFINITY, "state {s} free");
        }
    }
}

// ---- /Overrides/MaxDischargePower (schedule post-target hold / DESS cap) ----

#[test]
fn discharge_override_caps_max_discharge() {
    // schedule.py post-target hold: max(1, 0.8·pvpower) => 1 W with no PV overnight.
    let mut bus = base();
    bus.set(HUB4, P_OVR_MAXDISCHARGE, 1.0);
    assert_eq!(SocGuard::new(&bus).tick(&bus).max_discharge_w, 1.0);
    // Daytime tail with PV: pass through ~80-95 % of PV instead.
    bus.set(HUB4, P_OVR_MAXDISCHARGE, 800.0);
    assert_eq!(SocGuard::new(&bus).tick(&bus).max_discharge_w, 800.0);
}

#[test]
fn discharge_override_negative_or_unset_means_unlimited() {
    // systemcalc writes -1 to clear (window exit / AllowDischarge windows).
    let mut bus = base();
    bus.set(HUB4, P_OVR_MAXDISCHARGE, -1.0);
    assert_eq!(
        SocGuard::new(&bus).tick(&bus).max_discharge_w,
        f64::INFINITY
    );
    bus.clear(HUB4, P_OVR_MAXDISCHARGE); // item invalid/absent
    assert_eq!(
        SocGuard::new(&bus).tick(&bus).max_discharge_w,
        f64::INFINITY
    );
}

#[test]
fn discharge_override_combines_with_state_inhibit_most_restrictive() {
    // BL state 11 inhibits (0 W) regardless of a looser 500 W override...
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 11.0);
    bus.set(HUB4, P_OVR_MAXDISCHARGE, 500.0);
    assert_eq!(SocGuard::new(&bus).tick(&bus).max_discharge_w, 0.0);
    // ...and a 0 W override inhibits even in a free state.
    bus.set(SETTINGS, P_BL_STATE, 10.0);
    bus.set(HUB4, P_OVR_MAXDISCHARGE, 0.0);
    assert_eq!(SocGuard::new(&bus).tick(&bus).max_discharge_w, 0.0);
}

// ---- each force-charge trigger (§4a-3) ----

#[test]
fn no_force_charge_in_baseline() {
    let out = SocGuard::new(&base()).tick(&base());
    assert!(!out.force_charge);
    assert_eq!(out.cmd_override, None);
    assert!(!out.tpimf);
}

#[test]
fn force_charge_via_charge_request() {
    let mut bus = base();
    bus.set(TEST_BMS, P_CHARGE_REQUEST, 1.0);
    let out = SocGuard::new(&bus).tick(&bus);
    assert!(out.force_charge);
    assert!(out.cmd_override.is_some());
}

#[test]
fn force_charge_via_state_8_and_12() {
    for s in [8.0, 12.0] {
        let mut bus = base();
        bus.set(SETTINGS, P_BL_STATE, s);
        let out = SocGuard::new(&bus).tick(&bus);
        assert!(out.force_charge, "state {s} should force-charge");
        // both are also in the discharged set → discharge inhibited
        assert_eq!(out.max_discharge_w, 0.0);
    }
}

#[test]
fn force_charge_via_keepcharged_state_9() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0);
    let out = SocGuard::new(&bus).tick(&bus);
    assert!(out.force_charge);
    // KeepCharged is NOT in the discharged set → discharge not inhibited here.
    assert_eq!(out.max_discharge_w, f64::INFINITY);
}

#[test]
fn force_charge_via_minsoc_above_99() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_MINSOC, 100.0);
    assert!(SocGuard::new(&bus).tick(&bus).force_charge);
    // boundary: exactly 99.0 is NOT keep-charged
    bus.set(SETTINGS, P_BL_MINSOC, 99.0);
    assert!(!SocGuard::new(&bus).tick(&bus).force_charge);
}

// ---- force-charge command target (§4a-4, mode 1/0) ----

#[test]
fn cmd_override_uses_configured_ceiling_when_finite() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0);
    bus.set(SETTINGS, P_MAXCHARGE, 4700.0);
    assert_eq!(SocGuard::new(&bus).tick(&bus).cmd_override, Some(4700.0));
}

#[test]
fn cmd_override_falls_back_to_sentinel_when_unlimited() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0);
    // no MaxChargePower set (or the -1 "off" sentinel) => unlimited => 414 000 W
    bus.set(SETTINGS, P_MAXCHARGE, -1.0);
    assert_eq!(
        SocGuard::new(&bus).tick(&bus).cmd_override,
        Some(FORCE_CHARGE_SENTINEL_W)
    );
}

// ---- TPIMF capability probe ----

#[test]
fn tpimf_only_when_forced_and_firmware_supports_it() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 9.0); // force
    assert!(SocGuard::new(&bus).tick(&bus).tpimf);
    // firmware lacks the item => no modern path (legacy force-charge)
    bus.clear(VEBUS, P_TPIMF);
    assert!(!SocGuard::new(&bus).tick(&bus).tpimf);
    assert!(SocGuard::new(&bus).tick(&bus).force_charge); // still forces, just legacy
}

// ---- state-6 slow-charge floor (§4a-4) ----

#[test]
fn state_6_raises_charge_floor_to_5a_times_vdc() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 6.0);
    bus.set(VEBUS, P_DC_VOLTAGE, 52.0);
    let out = SocGuard::new(&bus).tick(&bus);
    assert_eq!(out.charge_floor_w, 5.0 * 52.0); // 260 W
    // state 6 is in the discharged set (inhibit) but is NOT a force-charge trigger
    assert_eq!(out.max_discharge_w, 0.0);
    assert!(!out.force_charge);
    assert_eq!(out.cmd_override, None);
}

#[test]
fn no_charge_floor_outside_state_6() {
    let out = SocGuard::new(&base()).tick(&base());
    assert_eq!(out.charge_floor_w, 0.0);
}

// ---- NaN / missing-read edge cases (block, never over-command) ----

#[test]
fn state_6_floor_is_zero_when_voltage_unread() {
    let mut bus = base();
    bus.set(SETTINGS, P_BL_STATE, 6.0);
    bus.clear(VEBUS, P_DC_VOLTAGE); // Vdc unread => NaN
    // floor must degrade to 0, never NaN/negative into the command clamp
    assert_eq!(SocGuard::new(&bus).tick(&bus).charge_floor_w, 0.0);
}

#[test]
fn missing_state_read_is_inert() {
    let mut bus = base();
    bus.clear(SETTINGS, P_BL_STATE); // unread => default 0 (BLDisabled)
    let out = SocGuard::new(&bus).tick(&bus);
    assert!(!out.force_charge);
    assert_eq!(out.max_discharge_w, f64::INFINITY);
    assert_eq!(out.charge_floor_w, 0.0);
    assert_eq!(out.cmd_override, None);
}

#[test]
fn no_bms_means_no_charge_request() {
    let mut bus = MockBus::new(); // no BMS service, no ActiveBmsService
    bus.set(SETTINGS, P_BL_STATE, 0.0);
    bus.set(SETTINGS, P_BL_MINSOC, 10.0);
    let out = SocGuard::new(&bus).tick(&bus);
    assert!(!out.force_charge);
}

#[test]
fn nan_min_soc_does_not_force_charge() {
    // A NaN min-SOC must not satisfy `> 99.0` (NaN comparisons are false).
    let inp = Inputs {
        bl_state: bl::DISABLED,
        min_soc: f64::NAN,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: false,
        ovr_max_discharge_w: f64::NAN,
    };
    assert!(!enact(&inp).force_charge);
}

#[test]
fn nan_eff_charge_yields_sentinel_not_nan_command() {
    let inp = Inputs {
        bl_state: KEEP_CHARGED,
        min_soc: 10.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: false,
        override_force_charge: false,
        ovr_max_discharge_w: f64::NAN,
    };
    assert_eq!(enact(&inp).cmd_override, Some(FORCE_CHARGE_SENTINEL_W));
}

/// Dispatch scenario: a Home-Assistant automation RAISES `MinimumSocLimit` when Octopus
/// grants a cheap car-charge window, so the ESS charges instead of discharging. Verified
/// on-device (2026-08-15): the stock ESS loop subscribes to NO battery-SOC dbus path —
/// `updateBatteryLimits()` fires only on state/minSoc/dcVoltage/max{Charge,Discharge}Power/
/// sustain — so it CANNOT compare SOC to MinSOC itself. That decision is made entirely by
/// `dbus-systemcalc-py` `batterylife.py` (which we do NOT replace) and delivered via
/// `BatteryLife/State`:
///   SocGuardDefault(10) --SOC≤MinSOC--> SocGuardDischarged(11)
///                       --SOC≤MinSOC-5--> SocGuardLowSocCharge(12)
///                       --SOC≥MinSOC--> back to (11)/(10).
/// Our job is faithful ENACTMENT of whatever state systemcalc emits. This replays that exact
/// sequence and asserts we inhibit at 11 and force-charge at 12 — i.e. the HA automation
/// keeps working byte-for-byte under the rewrite.
#[test]
fn dispatch_minsoc_raise_charges_via_state_machine() {
    let sg = |state: i32| {
        enact(&Inputs {
            bl_state: state,
            min_soc: 80.0, // HA raised it; the stock/our path only cares about >99, inert here
            charge_request: false,
            dc_voltage: 52.0,
            eff_max_charge_w: 4700.0,
            multi_supports_tpimf: true,
            override_force_charge: false,
            ovr_max_discharge_w: f64::NAN,
        })
    };
    // Before dispatch: SocGuardDefault, SOC above min → discharge free, no charge.
    let d = sg(10);
    assert!(!d.force_charge);
    assert_eq!(d.max_discharge_w, f64::INFINITY);

    // Dispatch raises MinSOC above SOC → systemcalc: SocGuardDischarged(11).
    // We must STOP discharging (hold) but not yet actively charge.
    let hold = sg(11);
    assert!(!hold.force_charge, "state 11 holds, does not force-charge");
    assert_eq!(hold.max_discharge_w, 0.0, "state 11 must inhibit discharge");

    // SOC ≥5% below the raised min → systemcalc: SocGuardLowSocCharge(12).
    // We must force-charge from grid AND keep discharge inhibited.
    let chg = sg(12);
    assert!(chg.force_charge, "state 12 must force-charge (the dispatch charge)");
    assert_eq!(chg.max_discharge_w, 0.0);
    assert_eq!(chg.cmd_override, Some(4700.0), "charge at the configured ceiling");
    assert!(chg.tpimf, "modern firmware path drives TPIMF force-charge");

    // MinSOC=100 edge (HA sets 100%): our own >99 KeepCharged path also charges,
    // independent of systemcalc — matches the stock loop's only internal SOC-ish rule.
    let kc = enact(&Inputs { min_soc: 100.0, bl_state: bl::SOCG_DEFAULT, ..dispatch_inp() });
    assert!(kc.force_charge, "MinSOC>99 is KeepCharged even at state 10");
}

fn dispatch_inp() -> Inputs {
    Inputs {
        bl_state: bl::SOCG_DEFAULT,
        min_soc: 80.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: 4700.0,
        multi_supports_tpimf: true,
        override_force_charge: false,
        ovr_max_discharge_w: f64::NAN,
    }
}

// Regression against REAL captured data: during scheduled charge (state 259) hub4
// force-charges via /Overrides/ForceCharge with NO BatteryLife/ChargeRequest signal
// (bl_state stays 10). Before the fix our socguard triggered on 0 of those rows.
#[test]
fn golden_scheduled_charge_is_enacted() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/golden-sample.csv");
    let data = match std::fs::read_to_string(path) {
        Ok(d) => d,
        Err(_) => return, // fixture optional
    };
    let mut lines = data.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let ix = |n: &str| header.iter().position(|h| *h == n).expect(n);
    let (i_fc, i_bls, i_blm) = (ix("h4_forcecharge"), ix("bl_state"), ix("bl_minsoc"));
    let mut rows = 0;
    let mut fc_rows = 0;
    for line in lines {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < header.len() {
            continue;
        }
        let hub4_fc = f[i_fc].trim() == "1";
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_BL_STATE, f[i_bls].parse().unwrap_or(0.0));
        bus.set(SETTINGS, P_BL_MINSOC, f[i_blm].parse().unwrap_or(0.0));
        bus.set(HUB4, P_OVR_FORCECHARGE, if hub4_fc { 1.0 } else { 0.0 });
        bus.set(VEBUS, P_DC_VOLTAGE, 52.0);
        let out = SocGuard::new(&bus).tick(&bus);
        assert_eq!(
            out.force_charge, hub4_fc,
            "force-charge mismatch vs hub4 (bl_state={}): ours={}",
            f[i_bls], out.force_charge
        );
        if hub4_fc {
            fc_rows += 1;
        }
        rows += 1;
    }
    assert!(fc_rows > 0, "fixture had no force-charge rows to validate the fix");
    eprintln!("scheduled-charge replay: {rows} rows, {fc_rows} force-charge, all matched hub4");
}
