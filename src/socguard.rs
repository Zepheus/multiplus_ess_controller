//! SocGuard / BatteryLife enactment (matches the stock ESS behavior).
//!
//! The *decision* half stays in Python (`dbus-systemcalc-py` `batterylife.py`): it
//! watches SOC and writes a single integer into `Settings/CGwacs/BatteryLife/State`
//! plus `MinimumSocLimit`. This module is the *actuator* — it never looks at SOC. It
//! reads that state integer, the min-SOC limit and the managed BMS `/Info/ChargeRequest`,
//! and turns them into the force-charge / discharge-inhibit enactment the main loop
//! applies to the control-loop command (per-tick order step 5, after the control law).
//!
//!   - force-charge predicate: state∈{8,12} ∨ ChargeRequest ∨ KeepCharged
//!     (state 9 ∨ MinSOC > 99.0)
//!   - discharge-inhibit set {5,6,8,11,12} via the mask (s-5<8) && (0xCB>>(s-5))&1
//!   - force-charge power = Σ effective max-charge, or the 414 000 W sentinel when
//!     unlimited ("charge as hard as the Multi/DVCC allows")
//!   - BL state 6 (24 h slow charge): battery-power lower clamp raised to +5 A × Vdc
//!   - TargetPowerIsMaxFeedIn "modern" path when the firmware advertises the item
//!
//! Scope: enactment only. The grid-meter presence/fallback half and the
//! `/Hub4/DisableFeedIn` composition live in a separate module; this one only
//! produces `force_charge`, the discharge-inhibit bound, the `tpimf` flag, the
//! legacy force-charge command and the state-6 charge floor.
//!
//! Safety discipline (matches `battery_limits.rs`): every read is defended, a
//! non-finite input degrades to the *safe* direction (no force-charge, no
//! over-command, block rather than drain), and the module never emits an
//! over-capacity number itself — the main loop's final `control::clamp(cmd, lo, hi)`
//! against design capacity is the last line of defence and is relied on for the
//! 414 000 W sentinel.

use crate::dbus::*;

/// KeepCharged BatteryLife state (min SOC held at 100 %).
pub const KEEP_CHARGED: i32 = crate::states::bl::SOCG_KEEP_CHARGED;
/// BLForceCharge — the 24 h "slow charge": raise the lower clamp to +5 A × Vdc.
pub const BL_FORCE_CHARGE: i32 = crate::states::bl::FORCE_CHARGE;
/// "Charge as hard as allowed" sentinel emitted when no finite charge limit is
/// configured; the Multi/DVCC then decides. Kept bit-compatible for the firmware.
pub const FORCE_CHARGE_SENTINEL_W: f64 = 414_000.0;
/// MinimumSocLimit strictly above this is treated as KeepCharged (min SOC 100 %).
const MINSOC_KEEPCHARGED: f64 = 99.0;
/// BL state-6 slow-charge current floor (A); × Dc/0/Voltage ≈ 260 W at 52 V.
const FORCE_CHARGE_FLOOR_A: f64 = 5.0;

// D-Bus paths (P_CHARGE_REQUEST on the active BMS service, P_TPIMF on VEBUS, and the
// HUB4 /Overrides/* surface) are defined and documented in dbus.rs. In shadow we read
// stock's published override values; in takeover, our own served ones.

/// Discharge-inhibit set {5,6,8,11,12} = {BLDischarged, BLForceCharge, BLLowSocCharge,
/// SocGuardDischarged, SocGuardLowSocCharge}. the stock daemon tests it with the literal
/// mask `(state-5 < 8) && ((0xCB >> (state-5)) & 1)`; reproduced here for fidelity.
/// The set it encodes: {DISCHARGED, FORCE_CHARGE, LOW_SOC_CHARGE, SOCG_DISCHARGED,
/// SOCG_LOW_SOC_CHARGE} (see `crate::states::bl`).
fn is_discharged_set(s: i32) -> bool {
    let d = s.wrapping_sub(5);
    (0..8).contains(&d) && (0xCB >> d) & 1 != 0
}

/// Non-finite or negative → 0 (a floor must never push the command negative or NaN).
fn sanitize_floor(x: f64) -> f64 {
    if x.is_finite() && x > 0.0 {
        x
    } else {
        0.0
    }
}

/// Live inputs sampled once per tick (all via `Bus`), pre-defended so `enact` is a
/// pure, total function over finite-or-flagged values.
#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    /// `Settings/CGwacs/BatteryLife/State`. Defaults to 0 (BLDisabled = inert) on a
    /// failed read, so a dropped read never fabricates a force-charge or an inhibit.
    pub bl_state: i32,
    /// `Settings/CGwacs/BatteryLife/MinimumSocLimit`; only compared against 99.0.
    pub min_soc: f64,
    /// Managed BMS `/Info/ChargeRequest` (false when no BMS / unread).
    pub charge_request: bool,
    /// VEBUS `/Dc/0/Voltage`; NaN when unread → state-6 floor degrades to 0.
    pub dc_voltage: f64,
    /// Σ effective max-charge (finite W) or NaN = "unlimited" → 414 000 W sentinel.
    pub eff_max_charge_w: f64,
    /// VEBUS `/Hub4/TargetPowerIsMaxFeedIn` item present (firmware capability probe).
    pub multi_supports_tpimf: bool,
    /// `com.victronenergy.hub4 /Overrides/ForceCharge` (int→bool): external force-charge
    /// trigger from systemcalc's schedule/DESS delegate (e.g. scheduled off-peak
    /// charging). Verified against captured state-259 data where it was the ONLY signal.
    pub override_force_charge: bool,
    /// `com.victronenergy.hub4 /Overrides/MaxDischargePower` (W): external discharge cap,
    /// NaN (invalid/unread) or < 0 = none. Set by the schedule delegate to ~1 W during a
    /// window's post-target hold — without honoring it we would discharge the freshly
    /// charged battery back into the house for the rest of the window.
    pub ovr_max_discharge_w: f64,
}

/// Enactment the main loop applies to the control-loop command, in this order:
///   1. discharge inhibit: `cmd = cmd.max(-max_discharge_w)` (no-op when +∞)
///   2. force charge: if `cmd_override` is `Some(fc)` —
///      tpimf → `set_i32(VEBUS, P_TPIMF, 1); cmd = cmd.min(fc)` (normal law passes
///      through as the firmware's feed-in cap; may be negative);
///      legacy → `set_i32(VEBUS, P_TPIMF, 0); cmd = fc`;
///      else `set_i32(VEBUS, P_TPIMF, 0)`
///   3. state-6 slow-charge floor: `cmd = cmd.max(charge_floor_w)`
///   4. final safety clamp (existing): `cmd = control::clamp(cmd, lo, hi)`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SocGuardOut {
    /// Force-charge predicate result.
    pub force_charge: bool,
    /// Discharge inhibit: max discharge magnitude allowed. `0.0` in the discharged
    /// set (command clamped ≥ 0), `f64::INFINITY` otherwise (defer to the envelope).
    pub max_discharge_w: f64,
    /// Legacy / capped force-charge command target: `Some(Σ max-charge | 414 000 W)`
    /// when `force_charge`, else `None`. `tpimf` selects how the main loop applies it.
    pub cmd_override: Option<f64>,
    /// Write `VEBUS /Hub4/TargetPowerIsMaxFeedIn` = this (modern force-charge path).
    pub tpimf: bool,
    /// BL state-6 charge floor (+5 A × Vdc, a *lower* bound on a charge command), else 0.
    pub charge_floor_w: f64,
}

/// Pure enactment — no I/O, total over any `Inputs` (incl. NaN). Matches the stock behavior and
/// unit-testable in isolation; `SocGuard::tick` samples `Inputs` and calls this.
pub fn enact(inp: &Inputs) -> SocGuardOut {
    // Force-charge predicate, INCLUDING the external Overrides/ForceCharge
    // trigger (schedule / DESS) so scheduled off-peak charging is actually enacted.
    let keep_charged = inp.bl_state == KEEP_CHARGED || inp.min_soc > MINSOC_KEEPCHARGED;
    let force_charge = inp.override_force_charge
        || inp.charge_request
        || keep_charged
        || matches!(
            inp.bl_state,
            crate::states::bl::LOW_SOC_CHARGE | crate::states::bl::SOCG_LOW_SOC_CHARGE
        );

    // Discharge inhibit: effective MaxDischargePower forced to 0 in the set,
    // AND capped by the external /Overrides/MaxDischargePower (ephemeral override —
    // schedule post-target hold / DESS). Most restrictive wins.
    let state_inhibit = if is_discharged_set(inp.bl_state) {
        0.0
    } else {
        f64::INFINITY
    };
    let ovr_cap = if inp.ovr_max_discharge_w.is_finite() && inp.ovr_max_discharge_w >= 0.0 {
        inp.ovr_max_discharge_w
    } else {
        f64::INFINITY
    };
    let max_discharge_w = state_inhibit.min(ovr_cap);

    // (state-6 special case) +5 A × Vdc slow-charge floor. NaN Vdc → 0 (safe).
    let charge_floor_w = if inp.bl_state == BL_FORCE_CHARGE {
        sanitize_floor(FORCE_CHARGE_FLOOR_A * inp.dc_voltage)
    } else {
        0.0
    };

    // Modern TPIMF path only when the firmware advertises the item. The
    // feed-in-force / generator refinements are [M]; the safe subset is used here.
    let tpimf = force_charge && inp.multi_supports_tpimf;

    // (mode 1/0) Force-charge command target: Σ effective max-charge, or the
    // 414 000 W sentinel when unlimited. Only meaningful while force_charge.
    let cmd_override = if force_charge {
        Some(if inp.eff_max_charge_w.is_finite() && inp.eff_max_charge_w > 0.0 {
            inp.eff_max_charge_w
        } else {
            FORCE_CHARGE_SENTINEL_W
        })
    } else {
        None
    };

    SocGuardOut {
        force_charge,
        max_discharge_w,
        cmd_override,
        tpimf,
        charge_floor_w,
    }
}

/// Resolve the active BMS service (mirrors `battery_limits::resolve_bms`): prefer
/// `com.victronenergy.system /ActiveBmsService`, else the first battery service.
fn resolve_bms(bus: &dyn Bus) -> Option<String> {
    if let Some(s) = bus.get_str(SYS, P_ACTIVE_BMS) {
        if !s.is_empty() {
            return Some(s);
        }
    }
    bus.list_services(BATTERY_PREFIX).into_iter().next()
}

/// SocGuard/BatteryLife enactor. Holds only the resolved BMS service name; the
/// enactment itself is a near-pure function of the per-tick reads.
pub struct SocGuard {
    bms_service: Option<String>,
}

impl SocGuard {
    pub fn new(bus: &dyn Bus) -> Self {
        let bms_service = resolve_bms(bus);
        eprintln!("socguard: bms={bms_service:?}");
        SocGuard { bms_service }
    }

    fn read(&mut self, bus: &dyn Bus) -> Inputs {
        // Late-appearing BMS (GX boot race): keep trying to resolve until found.
        if self.bms_service.is_none() {
            self.bms_service = resolve_bms(bus);
        }
        let charge_request = self
            .bms_service
            .as_deref()
            .and_then(|b| bus.get_bool(b, P_CHARGE_REQUEST))
            .unwrap_or(false);

        // Σ effective max-charge: the settings ceiling if configured (> 0), else NaN
        // to trip the 414 000 W sentinel — same rule as `pick_limit`. The
        // caller may instead feed the broker's computed Σ via `enact` directly.
        let eff_max_charge_w = match bus.get_f64(SETTINGS, P_MAXCHARGE) {
            Some(x) if x > 0.0 => x,
            _ => f64::NAN,
        };

        Inputs {
            // Unread state → 0 (BLDisabled): no force-charge, no inhibit. Safe default.
            bl_state: bus.get_i32(SETTINGS, P_BL_STATE).unwrap_or(0),
            min_soc: bus.get_f64(SETTINGS, P_BL_MINSOC).unwrap_or(0.0),
            charge_request,
            dc_voltage: bus.get_f64(VEBUS, P_DC_VOLTAGE).unwrap_or(f64::NAN),
            eff_max_charge_w,
            multi_supports_tpimf: bus.get_f64(VEBUS, P_TPIMF).is_some(),
            override_force_charge: bus.get_i32(HUB4, P_OVR_FORCECHARGE).unwrap_or(0) != 0,
            ovr_max_discharge_w: bus.get_f64(HUB4, P_OVR_MAXDISCHARGE).unwrap_or(f64::NAN),
        }
    }

    /// Sample live inputs and compute the enactment. Cheap (a handful of reads + a
    /// pure fn); call once per main-loop tick, not on the control hot path.
    pub fn tick(&mut self, bus: &dyn Bus) -> SocGuardOut {
        let inp = self.read(bus);
        enact(&inp)
    }
}

#[cfg(test)]
mod tests {
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

    // ---- each force-charge trigger ----

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

    // ---- force-charge command target (mode 1/0) ----

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

    // ---- state-6 slow-charge floor ----

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

    /// Dispatch scenario: a Home-Assistant automation RAISES `MinimumSocLimit` when
    /// Octopus grants a cheap car-charge window, so the ESS charges instead of
    /// discharging. The stock loop subscribes to NO
    /// battery-SOC dbus path — `updateBatteryLimits()` fires only on
    /// state/minSoc/dcVoltage/max{Charge,Discharge}Power/sustain — so it CANNOT compare
    /// SOC to MinSOC itself. That decision is made entirely by `dbus-systemcalc-py`
    /// `batterylife.py` (which we do NOT replace) and delivered via `BatteryLife/State`:
    ///   SocGuardDefault(10) --SOC≤MinSOC--> SocGuardDischarged(11)
    ///                       --SOC≤MinSOC-5--> SocGuardLowSocCharge(12)
    ///                       --SOC≥MinSOC--> back to (11)/(10).
    /// Our job is faithful ENACTMENT of whatever state systemcalc emits. This replays
    /// that exact sequence and asserts we inhibit at 11 and force-charge at 12 — i.e.
    /// the HA automation keeps working byte-for-byte under the rewrite.
    #[test]
    fn dispatch_minsoc_raise_charges_via_state_machine() {
        let sg = |state: i32| {
            enact(&Inputs {
                bl_state: state,
                min_soc: 80.0, // HA raised it; hub4/our path only cares about >99, so inert here
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
        // independent of systemcalc — matches stock hub4's only internal SOC-ish rule.
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
}
