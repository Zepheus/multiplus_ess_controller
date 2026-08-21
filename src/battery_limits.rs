//! Battery charge/discharge limit broker (Tier A).
//!
//! Converts the live BMS/DVCC current limits into the AC-side (lo, hi) power clamp
//! the ControlLoop uses, so we never command past what the battery allows. This is
//! the safety-critical piece the earlier build was missing: it clamped discharge to
//! a *static* startup value and never tracked the live BMS discharge-current limit,
//! so if the BMS lowers its limit (cold / low SOC / cell imbalance) we would not
//! follow it down.
//!
//! empirically modelled and validated live (280 A × 52.44 V × 0.9 = 13214.9 W matched
//! the published /MaxDischargePower to the watt). See the design notes.
//!
//! Tier A scope: the read-only broker + clamp only. No hub4 D-Bus server and no
//! machine-written ephemeral overrides yet (those are Tier B).

use crate::dbus::*;

const CHG_EFF: f64 = 0.97;
const DIS_EFF: f64 = 0.90; // validated live
const DIS_EFF_LOW: f64 = 0.60; // regular discharge below the 14 A knee
const DIS_OFFSET: f64 = 7.0; // legacy discharge reserve (A) above the knee
const DIS_KNEE: f64 = 14.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    Legacy,
    Regular,
}

/// AC-side power bounds. NaN = unlimited/invalid.
#[derive(Clone, Copy)]
pub struct Limits {
    pub max_charge_power: f64,
    pub max_discharge_power: f64,
}

impl Limits {
    /// Charge ceiling for the clamp `hi` (NaN -> +inf, i.e. defer to the outer envelope).
    pub fn charge_bound(&self) -> f64 {
        if self.max_charge_power.is_finite() {
            self.max_charge_power
        } else {
            f64::INFINITY
        }
    }
    /// Discharge magnitude for the clamp `lo = -this` (NaN -> +inf).
    pub fn discharge_bound(&self) -> f64 {
        if self.max_discharge_power.is_finite() {
            self.max_discharge_power
        } else {
            f64::INFINITY
        }
    }
}

struct Inputs {
    dc_voltage: f64,
    bms_ccl: Option<f64>,
    bms_dcl: Option<f64>,
    pv_current: f64,
    multi_max_charge_current: f64,
    sustain: bool,
    s_max_charge_power: f64,    // NaN if the -1 "off" sentinel
    s_max_discharge_power: f64, // NaN if -1
    s_charge_pct: f64,
    s_discharge_pct: f64,
    bl_state: i64,
    bl_min_soc: f64,
}

pub struct BatteryBroker {
    mode: Mode,
    latched_ccl: bool, // once BMS CCL has ever been finite
    latched_dcl: bool, // once BMS DCL has ever been finite (persisted -> never un-blocks)
    bms_service: Option<String>,
}

fn keep_charged(i: &Inputs) -> bool {
    i.bl_state == 9 || i.bl_min_soc > 99.0
}
fn is_discharged_set(state: i64) -> bool {
    matches!(state, 5 | 6 | 8 | 11 | 12)
}
fn sanitize(x: f64) -> f64 {
    if x.is_finite() && x < 0.0 {
        0.0
    } else {
        x
    }
}

fn detect_mode(bus: &dyn Bus) -> (Mode, u32, bool) {
    let dvcc = bus.get_bool(SYS, P_DVCC).unwrap_or(false);
    let fw = bus.get_f64(VEBUS, P_FIRMWARE).unwrap_or(0.0) as u32;
    let mode = if dvcc && fw > 0x414 {
        Mode::Regular
    } else {
        Mode::Legacy
    };
    (mode, fw, dvcc)
}

/// Mode detection that returns None if EITHER input read fails, so a transient dropped
/// read never flips the mode (and hence the discharge formula) for a tick. Sticky:
/// callers keep the current mode on None. (Adversarial review Finding 6.)
fn detect_mode_opt(bus: &dyn Bus) -> Option<Mode> {
    let dvcc = bus.get_bool(SYS, P_DVCC)?;
    let fw = bus.get_f64(VEBUS, P_FIRMWARE)? as u32;
    Some(if dvcc && fw > 0x414 {
        Mode::Regular
    } else {
        Mode::Legacy
    })
}

fn resolve_bms(bus: &dyn Bus) -> Option<String> {
    if let Some(s) = bus.get_str(SYS, P_ACTIVE_BMS) {
        if !s.is_empty() {
            return Some(s);
        }
    }
    bus.list_services(BATTERY_PREFIX).into_iter().next()
}

impl BatteryBroker {
    pub fn new(bus: &dyn Bus) -> Self {
        let (mode, fw, dvcc) = detect_mode(bus);
        let bms_service = resolve_bms(bus);
        eprintln!(
            "battery broker: mode={mode:?} fw={fw:#x} dvcc={dvcc} bms={bms_service:?}"
        );
        BatteryBroker {
            mode,
            latched_ccl: false,
            latched_dcl: false,
            bms_service,
        }
    }

    fn read(&mut self, bus: &dyn Bus) -> Inputs {
        // Sticky: only switch mode when BOTH DVCC and firmware read cleanly.
        if let Some(mode) = detect_mode_opt(bus) {
            if mode != self.mode {
                eprintln!("battery broker: mode {:?} -> {mode:?}", self.mode);
                self.mode = mode;
            }
        }
        if self.bms_service.is_none() {
            self.bms_service = resolve_bms(bus);
        }
        let bms = self.bms_service.as_deref();
        let from_bms = |p: &str| bms.and_then(|b| bus.get_f64(b, p));
        // -1 is Victron's "off/unset" sentinel for the power settings -> NaN.
        let neg1_nan = |v: Option<f64>| match v {
            Some(x) if x >= 0.0 => x,
            _ => f64::NAN,
        };
        Inputs {
            dc_voltage: bus.get_f64(VEBUS, P_DC_VOLTAGE).unwrap_or(f64::NAN),
            bms_ccl: from_bms(P_BMS_CCL),
            bms_dcl: from_bms(P_BMS_DCL),
            pv_current: bus.get_f64(SYS, P_PV_CURRENT).unwrap_or(0.0),
            multi_max_charge_current: bus.get_f64(VEBUS, P_MULTI_MAXCHG_I).unwrap_or(f64::NAN),
            sustain: bus.get_bool(VEBUS, P_SUSTAIN).unwrap_or(false),
            s_max_charge_power: neg1_nan(bus.get_f64(SETTINGS, P_MAXCHARGE)),
            s_max_discharge_power: neg1_nan(bus.get_f64(SETTINGS, P_MAXDISCHARGE)),
            s_charge_pct: bus.get_f64(SETTINGS, P_MAXCHARGE_PCT).unwrap_or(100.0),
            s_discharge_pct: bus.get_f64(SETTINGS, P_MAXDISCHARGE_PCT).unwrap_or(100.0),
            bl_state: bus.get_i32(SETTINGS, P_BL_STATE).unwrap_or(0) as i64,
            bl_min_soc: bus.get_f64(SETTINGS, P_BL_MINSOC).unwrap_or(0.0),
        }
    }

    fn recompute(&mut self, i: &Inputs) -> Limits {
        let v = i.dc_voltage;
        let mut ccl = i.bms_ccl.unwrap_or(f64::NAN);
        let mut dcl = i.bms_dcl.unwrap_or(f64::NAN);
        if ccl.is_finite() {
            self.latched_ccl = true;
        } else {
            ccl = 0.0;
        }
        if dcl.is_finite() {
            self.latched_dcl = true;
        } else {
            dcl = 0.0;
        }

        // ---- charge ----
        let charge = match self.mode {
            // Regular: charge is DVCC's job; we publish invalid ([]). Matches live.
            Mode::Regular => f64::NAN,
            Mode::Legacy => {
                let mut cp = if self.latched_ccl {
                    ccl * v / CHG_EFF
                } else if i.s_charge_pct <= 99.0 {
                    i.multi_max_charge_current * v
                } else {
                    f64::NAN
                };
                if i.s_charge_pct <= 100.0 && cp.is_finite() {
                    cp *= i.s_charge_pct.clamp(0.0, 100.0) / 100.0;
                }
                cp.min(i.s_max_charge_power)
            }
        };

        // ---- discharge ----
        let scaled = i.s_discharge_pct.clamp(0.0, 100.0) / 100.0;
        let mut dp = if self.latched_dcl {
            if dcl > 0.0 {
                let dcl_h = dcl + i.pv_current; // MPPT amps add discharge headroom
                let amps = match self.mode {
                    Mode::Regular => dcl_h,
                    Mode::Legacy => {
                        if dcl_h > DIS_KNEE {
                            dcl_h - DIS_OFFSET
                        } else {
                            dcl_h * 0.5
                        }
                    }
                };
                let eff = match self.mode {
                    Mode::Regular => {
                        if dcl_h > DIS_KNEE {
                            DIS_EFF
                        } else {
                            DIS_EFF_LOW
                        }
                    }
                    Mode::Legacy => DIS_EFF,
                };
                scaled * (amps * v * eff).max(0.0)
            } else {
                0.0 // DCL latched then went to 0 => hard block
            }
        } else if self.bms_service.is_some() || self.mode == Mode::Regular {
            // Managed install (a BMS is expected) but its DCL has not been read finite
            // yet — e.g. a GX boot race where our process starts before the battery
            // service. BLOCK discharge until the real limit latches; never discharge at
            // the full envelope while the battery's limit is unknown. (Adversarial
            // review Finding 3.)
            0.0
        } else {
            // Truly unmanaged (no BMS present/expected): the percentage acts as an
            // on/off switch, matching stock behaviour.
            if i.s_discharge_pct < 50.0 {
                0.0
            } else {
                f64::NAN
            }
        };
        if !keep_charged(i) {
            dp = dp.min(i.s_max_discharge_power); // f64::min ignores NaN
        }
        if i.sustain || i.bl_state == crate::states::bl::SUSTAIN as i64 {
            dp = 0.0;
        }

        // Sanitize: a finite-NEGATIVE limit is nonsensical (a sensor/sign glitch, e.g. a
        // negative BMS current or voltage) — treat it as 0 (block), never a negative
        // bound that could invert the command clamp. (Adversarial review Finding 1.)
        Limits {
            max_charge_power: sanitize(charge),
            max_discharge_power: sanitize(dp),
        }
    }

    /// Read live inputs and recompute limits. Returns both the PUBLISHED limits
    /// (what the stock ESS loop puts on `/MaxChargePower` / `/MaxDischargePower`, pre-
    /// tightening — used for golden regression) and the ENFORCED clamp for the
    /// ControlLoop (discharge additionally zeroed in BatteryLife discharged states
    /// {5,6,8,11,12}). Reads are cheap; call once per tick, not the control hot path.
    pub fn tick(&mut self, bus: &dyn Bus) -> TickResult {
        let i = self.read(bus);
        let published = self.recompute(&i);
        let mut enforced = published;
        // Zero discharge in BatteryLife discharged/force-charge states. Intentionally
        // UNCONDITIONAL (stock does this "unless keep-charged"); blocking discharge when
        // the battery wants to stay charged is a deliberate safe-direction divergence.
        // (Adversarial review Finding 4.) Note: this affects only the enforced clamp,
        // not the published value the golden test compares.
        if is_discharged_set(i.bl_state) {
            enforced.max_discharge_power = 0.0;
        }
        TickResult { published, enforced }
    }
}

pub struct TickResult {
    pub published: Limits,
    pub enforced: Limits,
}

/// Compose the broker's enforced limits with the outer envelope into the final
/// (lo, hi) command clamp. Because `lo >= env_lo` and `hi <= env_hi` always, passing
/// `env = +/- design-capacity` guarantees the command can never exceed the inverter
/// design capacity in either direction (no over-generation). Pure -> unit-testable.
pub fn clamp_bounds(enforced: Limits, env_lo: f64, env_hi: f64) -> (f64, f64) {
    // `.min(0.0)`/`.max(0.0)` are defensive: idle (command 0) is ALWAYS safe, so the
    // range must include 0 and can never invert, even if a limit came through negative
    // or NaN. Combined with env = +/- design-capacity this guarantees the command is
    // always within [-capacity, +capacity] and 0 is reachable.
    let lo = (-enforced.discharge_bound()).max(env_lo).min(0.0);
    let hi = enforced.charge_bound().min(env_hi).max(0.0);
    (lo, hi)
}

#[cfg(test)]
#[path = "battery_limits_tests.rs"]
mod tests;
