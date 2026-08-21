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
//! ../../re/socguard-gridmeter.md §4a/§5):
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

/// New BusItem paths this module reads (not already in `dbus.rs`).
/// On the runtime-resolved active BMS service.
pub const P_CHARGE_REQUEST: &str = "/Info/ChargeRequest"; // BMS, int→bool
/// On the vebus service; item *validity* = firmware supports the modern path.
pub const P_TPIMF: &str = "/Hub4/TargetPowerIsMaxFeedIn"; // VEBUS, bool/int
/// The external force-charge trigger. systemcalc's schedule/DESS delegate SetValues
/// this (e.g. the scheduled off-peak charge window) — the same override surface DESS
/// uses. In shadow we read stock hub4's published value; in Tier-B, our own served one.
pub const HUB4: &str = "com.victronenergy.hub4";
pub const P_OVR_FORCECHARGE: &str = "/Overrides/ForceCharge"; // hub4, int→bool
/// The DESS/schedule setpoint-override surface. Writing it (mode 1) lets the stock ESS loop
/// keep full authority while we bias only the grid target — the Stage-1 "trim" path, guarded
/// by the verified 300 s override watchdog. (Read/written by main's Trim stage.)
pub const P_OVR_SETPOINT: &str = "/Overrides/Setpoint"; // hub4, f64 (invalid = no override)
/// External discharge-power cap. Written by systemcalc's schedule delegate as the
/// post-target hold: once a scheduled-charge window reaches its target SOC (< 100 %),
/// it sets this to `max(1, 0.8..0.95 × DC-PV power)` — with no PV that is **1 W**, i.e.
/// "hold the charge, don't discharge for the rest of the window". (DESS reuses the same
/// surface as a dynamic cap.) < 0 or invalid = no cap. Live-verified 2026-08-16: during
/// the window tail the stock daemon held ~+48 W against a 1.9 kW house load while the
/// unclamped normal law wanted ≈ −1.86 kW.
pub const P_OVR_MAXDISCHARGE: &str = "/Overrides/MaxDischargePower"; // hub4, f64

/// Discharge-inhibit set {5,6,8,11,12} = {BLDischarged, BLForceCharge, BLLowSocCharge,
/// SocGuardDischarged, SocGuardLowSocCharge}. the stock ESS loop tests it with the literal
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

/// Enactment the main loop applies to the ControlLoop command, in this order:
///   1. discharge inhibit: `cmd = cmd.max(-max_discharge_w)` (no-op when +∞)
///   2. force charge: if `cmd_override` is `Some(fc)` —
///        tpimf  → `set_i32(VEBUS, P_TPIMF, 1); cmd = cmd.min(fc)` (normal law passes
///                 through as the firmware's feed-in cap; may be negative)
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
    // (§4a-3) Force-charge predicate, INCLUDING the external Overrides/ForceCharge
    // trigger (schedule / DESS) so scheduled off-peak charging is actually enacted.
    let keep_charged = inp.bl_state == KEEP_CHARGED || inp.min_soc > MINSOC_KEEPCHARGED;
    let force_charge = inp.override_force_charge
        || inp.charge_request
        || keep_charged
        || matches!(
            inp.bl_state,
            crate::states::bl::LOW_SOC_CHARGE | crate::states::bl::SOCG_LOW_SOC_CHARGE
        );

    // (§4a-2) Discharge inhibit: effective MaxDischargePower forced to 0 in the set,
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
#[path = "socguard_tests.rs"]
mod tests;
