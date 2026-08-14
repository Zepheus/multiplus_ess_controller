//! SocGuard / BatteryLife enactment (the stock ESS loop §4a, spec §6).
//!
//! The *decision* half stays in Python (`dbus-systemcalc-py` `batterylife.py`): it
//! watches SOC and writes a single integer into `Settings/CGwacs/BatteryLife/State`
//! plus `MinimumSocLimit`. This module is the *actuator* — it never looks at SOC. It
//! reads that state integer, the min-SOC limit and the managed BMS `/Info/ChargeRequest`,
//! and turns them into the force-charge / discharge-inhibit enactment the main loop
//! applies to the ControlLoop command (per-tick order step 5, after the control law).
//!
//! Recovered constants used faithfully (RE confidence [H] unless noted, see
//! the design notes §4a/§5):
//!   - force-charge predicate: state∈{8,12} ∨ ChargeRequest ∨ KeepCharged
//!     (state 9 ∨ MinSOC > 99.0)
//!   - discharge-inhibit set {5,6,8,11,12} via the mask (s-5<8) && (0xCB>>(s-5))&1
//!   - force-charge power = Σ effective max-charge, or the 414 000 W sentinel when
//!     unlimited ("charge as hard as the Multi/DVCC allows")
//!   - BL state 6 (24 h slow charge): battery-power lower clamp raised to +5 A × Vdc
//!   - TargetPowerIsMaxFeedIn "modern" path when the firmware advertises the item
//!
//! Scope: enactment only. The grid-meter presence/fallback half (§4b) and the
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
pub const KEEP_CHARGED: i32 = 9;
/// BLForceCharge — the 24 h "slow charge": raise the lower clamp to +5 A × Vdc.
pub const BL_FORCE_CHARGE: i32 = 6;
/// "Charge as hard as allowed" sentinel emitted when no finite charge limit is
/// configured; the Multi/DVCC then decides. Kept bit-compatible for the firmware.
pub const FORCE_CHARGE_SENTINEL_W: f64 = 414_000.0;
/// MinimumSocLimit strictly above this is treated as KeepCharged (min SOC 100 %).
const MINSOC_KEEPCHARGED: f64 = 99.0;
/// BL state-6 slow-charge current floor (A); × Dc/0/Voltage ≈ 260 W at 52 V.
const FORCE_CHARGE_FLOOR_A: f64 = 5.0;

/// New BusItem paths this module reads (not already in `dbus.rs`).
/// On the runtime-resolved active BMS service.
pub const P_CHARGE_REQUEST: &str = "/Info/ChargeRequest"; // BMS, int→bool
/// On the vebus service; item *validity* = firmware supports the modern path.
pub const P_TPIMF: &str = "/Hub4/TargetPowerIsMaxFeedIn"; // VEBUS, bool/int

/// Discharge-inhibit set {5,6,8,11,12} = {BLDischarged, BLForceCharge, BLLowSocCharge,
/// SocGuardDischarged, SocGuardLowSocCharge}. the stock ESS loop tests it with the literal
/// mask `(state-5 < 8) && ((0xCB >> (state-5)) & 1)`; reproduced here for fidelity.
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
}

/// Enactment the main loop applies to the ControlLoop command, in this order:
///   1. discharge inhibit: `cmd = cmd.max(-max_discharge_w)` (no-op when +∞)
///   2. force charge: if `cmd_override` is `Some(fc)` —
///        tpimf  → `set_i32(VEBUS, P_TPIMF, 1); cmd = cmd.max(0.0).min(fc)`
///        legacy → `set_i32(VEBUS, P_TPIMF, 0); cmd = fc`
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

/// Pure enactment — no I/O, total over any `Inputs` (incl. NaN). Faithful to §4a and
/// unit-testable in isolation; `SocGuard::tick` samples `Inputs` and calls this.
pub fn enact(inp: &Inputs) -> SocGuardOut {
    // (§4a-3) Force-charge predicate. Overrides/ForceCharge is the separate Tier-B
    // external-control surface and is intentionally NOT part of this predicate.
    let keep_charged = inp.bl_state == KEEP_CHARGED || inp.min_soc > MINSOC_KEEPCHARGED;
    let force_charge = inp.charge_request || keep_charged || matches!(inp.bl_state, 8 | 12);

    // (§4a-2) Discharge inhibit: effective MaxDischargePower forced to 0 in the set.
    let max_discharge_w = if is_discharged_set(inp.bl_state) {
        0.0
    } else {
        f64::INFINITY
    };

    // (§4a-4, state-6 special case) +5 A × Vdc slow-charge floor. NaN Vdc → 0 (safe).
    let charge_floor_w = if inp.bl_state == BL_FORCE_CHARGE {
        sanitize_floor(FORCE_CHARGE_FLOOR_A * inp.dc_voltage)
    } else {
        0.0
    };

    // (§4a-4) Modern TPIMF path only when the firmware advertises the item. The
    // feed-in-force / generator refinements are [M]; the safe subset is used here.
    let tpimf = force_charge && inp.multi_supports_tpimf;

    // (§4a-4, mode 1/0) Force-charge command target: Σ effective max-charge, or the
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
        // to trip the 414 000 W sentinel — same rule as `pick_limit`/§6.4. The
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
            bl_state: 0,
            min_soc: f64::NAN,
            charge_request: false,
            dc_voltage: 52.0,
            eff_max_charge_w: f64::NAN,
            multi_supports_tpimf: true,
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
        };
        assert_eq!(enact(&inp).cmd_override, Some(FORCE_CHARGE_SENTINEL_W));
    }
}
