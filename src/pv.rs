//! AC-coupled PV power limiter (the stock ESS loop subsystem #8 replacement).
//!
//! Curtails `com.victronenergy.pvinverter.*` inverters when their production can be
//! neither absorbed by the battery (charge headroom -> 0) nor exported (feed-in
//! forbidden / capped), and produces the two hub4 status flags the GUI/Multi read:
//! `/PvPowerLimiterActive` and `/Pv/Disable`, plus the `PvPowerSetpointOffset`
//! feed-forward the grid-setpoint loop pre-compensates with.
//!
//! empirically modelled from `com.victronenergy.hub4` (`PVPowerLimiter` +
//! `PVInverter`). See the design notes; the constants below are read exact from
//! the reference implementation `.rodata`.
//!
//! THIS INSTALL HAS NO PV and runs ESS External Control (Hub4Mode 3), where the
//! limiter is doubly dead (no inverters + mode gate). The behaviourally-exact output
//! is then the constant-0 stub: `/PvPowerLimiterActive = 0`, `/Pv/Disable = 0`, PV
//! offset `0`. `tick()` returns exactly that whenever no `pvinverter` service is
//! present (the default here), touching nothing else. The full curtailment path
//! below only engages once real AC PV inverters appear.
//!
//! Safety: we never curtail/disable something that isn't there (no inverters -> the
//! constant-0 stub, no writes), never write `Ac/PowerLimit` to a non-`pvinverter`
//! service, and every written limit is clamped to `[0, MaxPower]` or the RELEASE
//! sentinel — never NaN.

use crate::control::clamp;
use crate::dbus::*;

// --- dbus paths this subsystem owns (defined locally; not in dbus.rs) ---
/// Prefix enumerated to discover controllable AC PV inverters.
pub const PVINV_PREFIX: &str = "com.victronenergy.pvinverter.";
const P_AC_POWER: &str = "/Ac/Power"; // f64 W, actual output now
const P_AC_POWERLIMIT: &str = "/Ac/PowerLimit"; // f64 W, writable actuator
const P_AC_MAXPOWER: &str = "/Ac/MaxPower"; // f64 W, nameplate ceiling
const P_POSITION: &str = "/Position"; // int: 0=AC-in, 1=AC-out, 2=AC-in-2
const P_STATUSCODE: &str = "/StatusCode"; // int: run state
/// Settings export cap read for the proportional "allowed total" (NaN if unset).
const P_MAXFEEDIN: &str = "/Settings/CGwacs/MaxFeedInPower";

// --- extracted constants (exact, from the stock ESS loop .rodata; see re/pv-limiter.md §4.1) ---
const RELEASE_W: f64 = 200_000.0; // DAT_00051730 "no limit / release" sentinel
const CURTAIL_W: f64 = 0.0; // DAT_00051738 curtail-to-zero
const RAMP_FRAC: f64 = 0.01; // DAT_00050350 ramp step = max(1%·MaxPower, 50 W)
const RAMP_MIN_W: f64 = 50.0; // DAT_00050358
const DEADBAND_FRAC: f64 = 0.02; // DAT_00050218 min meaningful step: 2%·MaxPower
const FAST_THROTTLE: f64 = 0.015; // DAT_00051e60 aggressive cut when battery-full
const OFF_EMA_NEW: f64 = 0.6; // DAT_00051210 feed-forward EMA new-sample weight
const OFF_EMA_OLD: f64 = 0.4; // DAT_00051218 feed-forward EMA history weight
/// Hub4Mode value that gates the limiter fully off (External Control).
const HUB4MODE_EXTERNAL: i32 = 3;

/// Per-tick result — what the hub4 service publishes and the control loop feeds forward.
/// `pv_disable` -> `/Pv/Disable`, `pv_limiter_active` -> `/PvPowerLimiterActive`
/// (both int 0/1), `pv_offset_w` is the smoothed feed-forward added into the grid
/// setpoint bias. With no PV this is the constant `{0, 0, 0.0}` stub.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PvOut {
    pub pv_disable: i32,
    pub pv_limiter_active: i32,
    pub pv_offset_w: f64,
}

impl PvOut {
    /// The behaviourally-exact no-PV / mode-3 stub: never limiting, never disabling,
    /// zero feed-forward.
    pub const STUB: PvOut = PvOut { pv_disable: 0, pv_limiter_active: 0, pv_offset_w: 0.0 };
}

/// Shared, battery-derived context the limiter needs but does not own. Produced once
/// per tick by the battery-limit subsystem (#2/#3) and the setpoint loop, and passed
/// in so both read one consistent snapshot (re/pv-limiter.md §6.3). The no-PV stub
/// path ignores it entirely, so `Default` is safe for the stub caller.
#[derive(Clone, Copy, Debug)]
pub struct PvContext {
    /// Watts the battery can still absorb (). The absorb sink size.
    pub charge_headroom_w: f64,
    /// Battery cannot absorb meaningfully more (charge state "full").
    pub battery_full: bool,
    /// Feed-in forbidden (DisableFeedIn/preventFeedback) and not force-charging.
    pub export_blocked: bool,
    /// Global gate (bridge ): false => nothing to limit, release everyone.
    pub limiter_enabled: bool,
}

impl Default for PvContext {
    fn default() -> Self {
        PvContext {
            charge_headroom_w: f64::INFINITY,
            battery_full: false,
            export_blocked: false,
            limiter_enabled: true,
        }
    }
}

/// One AC PV inverter's per-tick snapshot.
struct PvInverter {
    service: String,
    ac_power: Option<f64>,
    power_limit: Option<f64>,
    max_power: Option<f64>,
    position: Option<i32>,
    status: Option<i32>,
}

impl PvInverter {
    fn read(bus: &dyn Bus, service: String) -> Self {
        PvInverter {
            ac_power: bus.get_f64(&service, P_AC_POWER),
            power_limit: bus.get_f64(&service, P_AC_POWERLIMIT),
            max_power: bus.get_f64(&service, P_AC_MAXPOWER),
            position: bus.get_i32(&service, P_POSITION),
            status: bus.get_i32(&service, P_STATUSCODE),
            service,
        }
    }
    /// "producing / online" (): running StatusCode {7,11,12} OR Ac/Power > 0.
    fn producing(&self) -> bool {
        matches!(self.status, Some(7 | 11 | 12)) || self.ac_power.map_or(false, |p| p > 0.0)
    }
    /// Finite nameplate ceiling, if the inverter reported one (>0). None => uncontrollable.
    fn nameplate(&self) -> Option<f64> {
        self.max_power.filter(|m| m.is_finite() && *m > 0.0)
    }
}

pub struct PvLimiter {
    offset_ema: f64,          // per-phase feed-forward state (+0x38); NaN until first sample
    active: bool,             // -> /PvPowerLimiterActive
    disabled: bool,           // -> /Pv/Disable
    writes: Vec<(String, f64)>, // Ac/PowerLimit writes issued this tick (for inspection/tests)
}

impl PvLimiter {
    pub fn new(bus: &dyn Bus) -> Self {
        let n = bus.list_services(PVINV_PREFIX).len();
        eprintln!("pv limiter: {n} AC PV inverter(s) present -> {}",
            if n == 0 { "constant-0 stub" } else { "active curtailment path" });
        PvLimiter { offset_ema: f64::NAN, active: false, disabled: false, writes: Vec::new() }
    }

    /// Ac/PowerLimit writes issued on the most recent `tick` (service, watts).
    pub fn last_writes(&self) -> &[(String, f64)] {
        &self.writes
    }

    /// Read live PV, recompute limits, write `Ac/PowerLimit`, and return the hub4
    /// flags + feed-forward. Call once per main-loop tick, AFTER the battery limits
    /// (so `ctx.charge_headroom_w` is fresh — re/pv-limiter.md §6.4).
    pub fn tick(&mut self, bus: &dyn Bus, ctx: PvContext) -> PvOut {
        self.writes.clear();

        // --- no-PV stub: the behaviourally-exact path for THIS install ---
        // No inverters => nothing to curtail or disable. Reset all state and return the
        // constant-0 stub without touching the bus further.
        let services = bus.list_services(PVINV_PREFIX);
        if services.is_empty() {
            self.offset_ema = f64::NAN;
            self.active = false;
            self.disabled = false;
            return PvOut::STUB;
        }

        // --- full path: snapshot every inverter ---
        let inverters: Vec<PvInverter> =
            services.into_iter().map(|s| PvInverter::read(bus, s)).collect();

        // Feed-forward runs on its own timer in the firmware (not mode-gated); compute
        // it every tick from measured production.
        let offset = self.update_power(&inverters);

        // Hub4Mode == 3 (External Control) short-circuits the limiter to inactive.
        let hub4_mode = bus.get_i32(SETTINGS, P_HUB4MODE).unwrap_or(HUB4MODE_EXTERNAL);
        if hub4_mode == HUB4MODE_EXTERNAL {
            self.active = false;
            self.disabled = false;
            return PvOut { pv_disable: 0, pv_limiter_active: 0, pv_offset_w: offset };
        }

        self.update_limits(bus, &inverters, &ctx);
        PvOut {
            pv_disable: self.disabled as i32,
            pv_limiter_active: self.active as i32,
            pv_offset_w: offset,
        }
    }

    /// `updateLimits()` port (re/pv-limiter.md §4.2). Decides the global posture, writes
    /// each inverter's `Ac/PowerLimit`, and sets `active`/`disabled`.
    fn update_limits(&mut self, bus: &dyn Bus, inverters: &[PvInverter], ctx: &PvContext) {
        // 1. Nothing to limit -> release everyone to the sentinel.
        if !ctx.limiter_enabled {
            for inv in inverters {
                self.write_limit(bus, inv, RELEASE_W);
            }
            self.active = false;
            self.disabled = false;
            return;
        }

        // 2. No sink at all (battery full AND export blocked) -> hard curtail via
        //    frequency-shift: /Pv/Disable=1, producing inverters to 0, idle inverters
        //    released to MaxPower so they can restart when a sink returns.
        if ctx.battery_full && ctx.export_blocked {
            for inv in inverters.iter().filter(controllable) {
                if inv.producing() {
                    self.write_limit(bus, inv, CURTAIL_W);
                } else if let Some(mp) = inv.nameplate() {
                    self.write_limit(bus, inv, mp);
                }
            }
            self.disabled = true;
            self.active = true;
            return;
        }

        // 3. A partial sink exists. `allowed` = absorb headroom + export cap.
        let feedin = bus.get_f64(SETTINGS, P_MAXFEEDIN);
        let feedin = feedin.filter(|f| f.is_finite() && *f >= 0.0).unwrap_or(0.0);
        let headroom = if ctx.charge_headroom_w.is_finite() {
            ctx.charge_headroom_w.max(0.0)
        } else {
            0.0
        };
        let allowed = headroom + feedin;

        // Producing, controllable, non-AC-output inverters share the allowance and ramp
        // toward it; AC-output (Position==1) inverters are released to MaxPower.
        let ramping: Vec<&PvInverter> = inverters
            .iter()
            .filter(controllable)
            .filter(|inv| inv.producing() && inv.position != Some(1))
            .collect();
        let share = if ramping.is_empty() { allowed } else { allowed / ramping.len() as f64 };

        let mut any_capped = false;
        for inv in inverters.iter().filter(controllable) {
            let mp = match inv.nameplate() {
                Some(mp) => mp,
                // Uncontrollable (no nameplate) -> don't curtail blind; release it.
                None => {
                    self.write_limit(bus, inv, RELEASE_W);
                    continue;
                }
            };
            if !inv.producing() || inv.position == Some(1) {
                // Idle or AC-output -> open to nameplate.
                self.write_limit(bus, inv, mp);
                continue;
            }
            // Current effective limit, treating the release sentinel / over-range as fully open.
            let cur = match inv.power_limit {
                Some(l) if l.is_finite() && l < mp => l,
                _ => mp,
            };
            let deadband = DEADBAND_FRAC * mp;
            let new = if ctx.battery_full {
                // Battery full but export still a sink: aggressive step-down toward the
                // export share (fast-throttle factor), floored at the allowed share.
                (cur * FAST_THROTTLE).max(share.min(mp))
            } else {
                let step = (RAMP_FRAC * mp).max(RAMP_MIN_W);
                ramp(cur, share.min(mp), step, deadband)
            };
            let new = clamp(new, 0.0, mp);
            self.write_limit(bus, inv, new);
            if new < mp - 0.5 {
                any_capped = true;
            }
        }
        self.active = any_capped;
        self.disabled = false;
    }

    /// `updatePower()` feed-forward port (re/pv-limiter.md §4.3), collapsed to one phase.
    /// EMA (0.6·new + 0.4·prev) of the PV currently being produced; the caller adds this
    /// into the grid-setpoint bias so the Multi anticipates the PV inverter's slow report.
    fn update_power(&mut self, inverters: &[PvInverter]) -> f64 {
        let raw: f64 = inverters
            .iter()
            .filter(|inv| inv.producing())
            .filter_map(|inv| inv.ac_power)
            .filter(|p| p.is_finite())
            .map(|p| p.max(0.0))
            .sum();
        let filtered = if self.offset_ema.is_finite() {
            OFF_EMA_NEW * raw + OFF_EMA_OLD * self.offset_ema
        } else {
            raw // seed on first sample
        };
        self.offset_ema = filtered;
        filtered.max(0.0)
    }

    fn write_limit(&mut self, bus: &dyn Bus, inv: &PvInverter, watts: f64) {
        // Only ever a pvinverter service, only ever finite. Record then write.
        self.writes.push((inv.service.clone(), watts));
        let _ = bus.set_f64(&inv.service, P_AC_POWERLIMIT, watts);
    }
}

/// Only inverters that currently report a finite `Ac/PowerLimit` participate in limiting.
fn controllable(inv: &&PvInverter) -> bool {
    inv.power_limit.map_or(false, f64::is_finite)
}

/// Ramp `cur` toward `target` by at most `step`, snapping when within `deadband`.
fn ramp(cur: f64, target: f64, step: f64, deadband: f64) -> f64 {
    if (cur - target).abs() <= deadband.max(step) {
        target
    } else if cur < target {
        cur + step
    } else {
        cur - step
    }
}

#[cfg(test)]
#[path = "pv_tests.rs"]
mod tests;
