//! Grid-meter presence / failure-mode guard (Tier A safety gate).
//!
//! empirically modelled from `the stock ESS loop` (ARM32, closed-source). Decides,
//! once per presence tick, whether the ESS setpoint write should proceed and
//! whether the `/Alarms/NoGridMeter` warning must be raised — exactly the two
//! independent code paths the reference implementation runs off its `ControlLoop`:
//!
//!   * setpoint gate  — `` early-returns (stop writing → the Multi's
//!     own 60 s watchdog reverts it to Passthru: battery idle, loads on grid).
//!   * presence / alarm FSM — `onTimer` case 5, a 48-tick grace counter that
//!     raises `/Alarms/NoGridMeter` after 48 × 2.5 s = 120 s.
//!
//! Both paths are faithful to the reference implementation (grace `+0x48` reload `0x30`=48, timers
//! `0x9c4`=2500 ms, `ActiveInput==240` disconnect sentinel, `Source==2` genset,
//! `Hub4Mode==3` inert). See the design notes §1, §3, §6.
//!
//! DELIBERATE DIVERGENCE (safety improvement, §4): stock keys presence on the
//! meter *pointer* and generic item *validity* only — it has **no staleness /
//! value-age check**, so a frozen-but-present meter (D-Bus service up, values
//! stuck) is silently regulated on stale data, with NO passthru and NO alarm.
//! We add a freshness guard: a meter whose value has not moved for `STALE_TICKS`
//! consecutive ticks (~15 s) is treated as **absent**, degrading to the handled
//! "meter absent" rows (clean passthru + alarm) instead of silent mis-regulation.
//! Gated by `frozen_guard` so stock parity remains auditable.
//!
//! Cadence: `tick()` is expected to be driven once per presence tick (2.5 s) —
//! e.g. main.rs accumulates `dt` and steps once per `PRESENCE_TICK`. Grace and
//! staleness are counted in ticks so the core stays clock-free and deterministic.
//!
//! Safety-first: when anything is unreadable or in doubt, we do **not** write the
//! setpoint (`Regulation::Skip` → passthru). Reads default to the safe direction
//! (meter required = true, run-without = false, inert Hub4Mode).

use crate::dbus::*;
use std::time::Duration;

// ---- faithful constants (verified against the reference implementation) --------------

/// Grace counter reload (`ControlLoop +0x48`, `mov …,#0x30`). [RE][H]
pub const GRACE_TICKS: u32 = 48;
/// Presence-tick interval (both `QTimer`s `movw r1,#0x9c4`). [RE][H]
pub const PRESENCE_TICK: Duration = Duration::from_millis(2500);
/// Absent-to-alarm delay: 48 × 2.5 s (rate-independent form). [RE][H]
pub const GRACE_TIMEOUT: Duration = Duration::from_secs(120);
/// `/Ac/ActiveIn/ActiveInput == 240` (0xf0) ⇒ no AC input connected. [RE][H]
pub const ACTIVEIN_DISCONNECTED: i32 = 240;
/// `com.victronenergy.system /Ac/ActiveIn/Source == 2` ⇒ generator. [RE][H]
pub const SOURCE_GENERATOR: i32 = 2;
/// `Hub4Mode == 3` ⇒ external control, `the stock ESS loop` inert. [RE][H]
pub const HUB4MODE_EXTERNAL: i32 = 3;

// ---- staleness guard (our improvement, non-stock) -------------------------

/// Consecutive unchanged-value ticks after which a present meter is declared
/// frozen/stale and treated as absent. 6 ticks ≈ 15 s at the 2.5 s cadence —
/// generous versus a healthy ET112 (~2 s refresh, value practically always
/// moves) yet well under the Multi's 60 s watchdog, so we passthru deliberately
/// rather than regulate on stale data. See §4 / §6.3.
pub const STALE_TICKS: u32 = 6;
/// Documentation form of `STALE_TICKS` at the presence cadence (~15 s).
pub const STALE_TIMEOUT: Duration = Duration::from_secs(15);

// ---- D-Bus paths this module needs (new; others reused from dbus.rs) ------

/// Grid-meter services to enumerate/adopt (`ListNames`, first in list order).
pub const GRID_PREFIX: &str = "com.victronenergy.grid.";
/// Total grid-meter power on an adopted `com.victronenergy.grid.*` service.
pub const P_METER_POWER: &str = "/Ac/Power";
/// Single-phase fallback when a meter has no aggregate `/Ac/Power`.
pub const P_METER_L1_POWER: &str = "/Ac/L1/Power";
/// `com.victronenergy.settings` — run the loop with no grid meter (bool).
const P_RUN_WITHOUT_METER: &str = "/Settings/CGwacs/RunWithoutGridMeter";
/// `com.victronenergy.settings` — grid meter required (bool, default 1/true).
const P_GRID_METER_REQUIRED: &str = "/Settings/CGwacs/GridMeterRequired";
/// `com.victronenergy.system` — active-input source (1 grid / 2 gen / 3 shore).
const P_SOURCE: &str = "/Ac/ActiveIn/Source";
/// vebus — active input index (240 = disconnected).
const P_ACTIVE_INPUT: &str = "/Ac/ActiveIn/ActiveInput";
// Reused from dbus.rs: SYS, VEBUS, SETTINGS, P_GRID (system-republished grid
// power), P_HUB4MODE (/Settings/CGwacs/Hub4Mode), P_ACTIVEIN (vebus AC-in P).

/// Presence state emitted by the FSM (mirrors the reference implementation's `gridMeterPresence`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Presence {
    /// Meter present and usable. (`presence = 0`)
    Ok,
    /// Meter absent but still within the 120 s grace window. (`presence = 2`)
    Grace,
    /// Meter absent past grace ⇒ raise `/Alarms/NoGridMeter`. (`presence = 1`)
    Missing,
    /// Alarm inapplicable: inert / no AC-in / run-without / genset. (`presence = 2`)
    NotApplicable,
}

/// What the setpoint path should regulate against this tick (mirrors the
/// `` guards). `Skip` = write nothing → Multi watchdog → passthru.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Regulation {
    /// Do NOT write `/Hub4/L1/AcPowerSetpoint` — let the Multi fail safe.
    Skip,
    /// Normal ESS law: process variable = external grid-meter power.
    OnGridMeter,
    /// No usable meter: process variable = the Multi's own AC-in power.
    OnMultiInput,
}

/// Everything the FSM reads each tick, gathered from the bus (safe defaults).
#[derive(Clone, Copy, Debug)]
struct Inputs {
    hub4mode: i32,           // /Settings/CGwacs/Hub4Mode, default 3 (inert) if unreadable
    multi_valid: bool,       // vebus /Ac/ActiveIn/L1/P is a valid finite number
    run_without_meter: bool, // /Settings/CGwacs/RunWithoutGridMeter, default false
    grid_meter_required: bool, // /Settings/CGwacs/GridMeterRequired, default TRUE
    source: i32,             // system /Ac/ActiveIn/Source (0 if unreadable = grid mode)
    active_input: i32,       // vebus /Ac/ActiveIn/ActiveInput (240 = no AC in)
}

/// The gate main.rs acts on. `regulate`/`presence` carry the full detail; the
/// four flat flags are the common decisions the daemon loop needs.
#[derive(Clone, Copy, Debug)]
pub struct GridMeterGuardOut {
    /// A grid meter is present AND fresh (passes the staleness guard).
    pub usable: bool,
    /// Write the setpoint this tick? `false` ⇒ let the Multi revert to passthru.
    pub should_write_setpoint: bool,
    /// Raise `com.victronenergy.hub4 /Alarms/NoGridMeter` (grace expired).
    pub alarm_no_grid_meter: bool,
    /// The effective RunWithoutGridMeter setting (for main.rs / logging).
    pub run_without_meter: bool,
    /// Which process variable to regulate against (Skip / grid meter / Multi in).
    pub regulate: Regulation,
    /// Full presence state (Ok / Grace / Missing / NotApplicable).
    pub presence: Presence,
    /// A present meter was detected FROZEN this tick (our non-stock guard tripped).
    pub stale: bool,
}

/// Grid-meter presence / failure-mode guard. Holds the grace counter and the
/// staleness tracker across ticks.
pub struct GridMeterGuard {
    /// Absent-meter countdown (ticks). Reloaded to `GRACE_TICKS` whenever the
    /// meter is present or the alarm is not applicable.
    grace: u32,
    /// Last meter value seen (for the freshness comparison).
    last_meter_value: Option<f64>,
    /// Consecutive ticks the meter value has been unchanged.
    unchanged_ticks: u32,
    /// Adopted grid service (first in list order; promoted on removal). Observability.
    meter_service: Option<String>,
    /// Enable the non-stock frozen-meter staleness guard (default on).
    frozen_guard: bool,
}

impl Default for GridMeterGuard {
    fn default() -> Self {
        Self::new()
    }
}

impl GridMeterGuard {
    /// New guard with the staleness improvement ENABLED (safety default).
    pub fn new() -> Self {
        GridMeterGuard {
            grace: GRACE_TICKS,
            last_meter_value: None,
            unchanged_ticks: 0,
            meter_service: None,
            frozen_guard: true,
        }
    }

    /// Builder: toggle the non-stock frozen-meter guard. `false` reproduces
    /// stock behaviour (present-but-frozen meter is treated as usable) so parity
    /// against the reference implementation stays auditable.
    pub fn with_frozen_guard(mut self, on: bool) -> Self {
        self.frozen_guard = on;
        self
    }

    /// The grid service currently adopted (if any) — diagnostics only.
    pub fn meter_service(&self) -> Option<&str> {
        self.meter_service.as_deref()
    }

    /// Adopt the first grid meter in list order and read its power. Falls back to
    /// the system-republished grid power, which also goes invalid exactly when
    /// the last grid meter leaves the bus. Returns a finite value or `None`.
    fn read_meter(&mut self, bus: &dyn Bus) -> Option<f64> {
        for svc in bus.list_services(GRID_PREFIX) {
            let v = bus
                .get_f64(&svc, P_METER_POWER)
                .or_else(|| bus.get_f64(&svc, P_METER_L1_POWER));
            if let Some(v) = v {
                if v.is_finite() {
                    self.meter_service = Some(svc);
                    return Some(v);
                }
            }
        }
        self.meter_service = None;
        bus.get_f64(SYS, P_GRID).filter(|v| v.is_finite())
    }

    /// Update the freshness tracker and return `(valid, fresh)` for this tick.
    fn update_freshness(&mut self, raw: Option<f64>) -> (bool, bool) {
        match raw {
            Some(v) => {
                match self.last_meter_value {
                    // Exact repeat ⇒ meter value has not advanced this tick.
                    Some(prev) if prev == v => {
                        self.unchanged_ticks = self.unchanged_ticks.saturating_add(1);
                    }
                    _ => self.unchanged_ticks = 0,
                }
                self.last_meter_value = Some(v);
                let fresh = !self.frozen_guard || self.unchanged_ticks < STALE_TICKS;
                (true, fresh)
            }
            None => {
                self.last_meter_value = None;
                self.unchanged_ticks = 0;
                (false, false)
            }
        }
    }

    fn read_inputs(bus: &dyn Bus) -> Inputs {
        Inputs {
            // Default 3 (inert → Skip) if the setting can't be read — safe.
            hub4mode: bus.get_i32(SETTINGS, P_HUB4MODE).unwrap_or(HUB4MODE_EXTERNAL),
            multi_valid: bus
                .get_f64(VEBUS, P_ACTIVEIN)
                .map_or(false, |v| v.is_finite()),
            run_without_meter: bus.get_bool(SETTINGS, P_RUN_WITHOUT_METER).unwrap_or(false),
            // Default TRUE (meter required) — the reference implementation's default and the safe one.
            grid_meter_required: bus.get_bool(SETTINGS, P_GRID_METER_REQUIRED).unwrap_or(true),
            // Default 0 = grid mode (meter matters); never assume genset.
            source: bus.get_i32(SYS, P_SOURCE).unwrap_or(0),
            // Default 0 = "AC connected" so a missing meter is NOT silently excused
            // from the alarm when this read fails (alarm is orthogonal to the write).
            active_input: bus.get_i32(VEBUS, P_ACTIVE_INPUT).unwrap_or(0),
        }
    }

    /// Run one presence tick: read the bus, apply the staleness guard, step the
    /// setpoint gate and the presence/alarm FSM, and return the decision.
    ///
    /// Call once per `PRESENCE_TICK` (2.5 s).
    pub fn tick(&mut self, bus: &dyn Bus) -> GridMeterGuardOut {
        let i = Self::read_inputs(bus);
        let raw = self.read_meter(bus);
        let (valid, fresh) = self.update_freshness(raw);
        let meter_present = valid && fresh; // effective presence (with the guard)
        let stale = valid && !fresh; // present-but-frozen detected this tick

        let genset = i.source == SOURCE_GENERATOR;
        let inert = i.hub4mode == HUB4MODE_EXTERNAL || !i.multi_valid;

        // ---- setpoint gate (mirrors  early-returns) ----
        let regulate = if inert {
            // External mode or Multi/AC-in unreadable ⇒ write nothing (fail safe).
            Regulation::Skip
        } else if i.run_without_meter || genset {
            // RWGM or genset: meter term degenerates to 0 ⇒ regulate on Multi AC-in.
            Regulation::OnMultiInput
        } else if meter_present {
            Regulation::OnGridMeter
        } else if !i.grid_meter_required {
            // Meter optional: fall back to the Multi's own AC-in instead of stopping.
            Regulation::OnMultiInput
        } else {
            // Meter required, absent (or frozen), grid mode, not RWGM ⇒ NO WRITE.
            Regulation::Skip
        };

        // ---- presence / alarm FSM (mirrors onTimer case 5) ----
        // Independent of the gate: the alarm fires after grace even when the meter
        // is *optional* (GridMeterRequired only gates the Skip branch). [RE][H]
        let presence = if inert || i.run_without_meter || genset {
            self.grace = GRACE_TICKS;
            Presence::NotApplicable
        } else if meter_present {
            self.grace = GRACE_TICKS;
            Presence::Ok
        } else if i.active_input == ACTIVEIN_DISCONNECTED {
            // No AC input at all ⇒ nothing to regulate against, not an alarm.
            self.grace = GRACE_TICKS;
            Presence::NotApplicable
        } else if self.grace > 1 {
            self.grace -= 1;
            Presence::Grace
        } else {
            self.grace = 0;
            Presence::Missing
        };

        GridMeterGuardOut {
            usable: meter_present,
            should_write_setpoint: regulate != Regulation::Skip,
            alarm_no_grid_meter: presence == Presence::Missing,
            run_without_meter: i.run_without_meter,
            regulate,
            presence,
            stale,
        }
    }
}

#[cfg(test)]
mod tests {
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
        let out = g.tick(&bus);
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
        assert!(g.tick(&bus).should_write_setpoint);
        make_absent(&mut bus);
        let out = g.tick(&bus);
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
        g.tick(&bus); // meter present: grace = 48
        make_absent(&mut bus);
        // 47 absent ticks stay in grace (no alarm)…
        for k in 1..=47 {
            let out = g.tick(&bus);
            assert_eq!(out.presence, Presence::Grace, "tick {k}");
            assert!(!out.alarm_no_grid_meter, "tick {k}");
        }
        // …the 48th raises the alarm and holds it.
        let out = g.tick(&bus);
        assert_eq!(out.presence, Presence::Missing);
        assert!(out.alarm_no_grid_meter);
        assert!(g.tick(&bus).alarm_no_grid_meter, "alarm stays latched while absent");
    }

    #[test]
    fn returning_meter_clears_the_alarm_next_tick() {
        let mut bus = grid_present(230.0);
        let mut g = GridMeterGuard::new();
        g.tick(&bus);
        make_absent(&mut bus);
        for _ in 0..48 {
            g.tick(&bus);
        }
        assert!(g.tick(&bus).alarm_no_grid_meter); // firmly in alarm
        // Meter comes back (fresh value) ⇒ alarm clears immediately, grace reloads.
        let bus = grid_present(180.0);
        let out = g.tick(&bus);
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
            let out = g.tick(&bus);
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
            let out = g.tick(&bus);
            assert!(out.usable, "not yet declared stale");
            assert!(!out.stale);
        }
        // Once unchanged for STALE_TICKS consecutive ticks it is declared frozen.
        let out = g.tick(&bus);
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
            let out = g.tick(&bus);
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
            let out = g.tick(&bus);
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
            let out = g.tick(&bus);
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
            let out = g.tick(&bus);
            assert_eq!(out.regulate, Regulation::OnMultiInput);
            assert!(out.should_write_setpoint);
        }
        let out = g.tick(&bus);
        assert!(out.alarm_no_grid_meter, "alarm fires even when meter optional");
        assert_eq!(out.regulate, Regulation::OnMultiInput); // still writing
    }

    #[test]
    fn rwgm0_gmr1_absent_skips_and_alarms() {
        let mut bus = grid_present(230.0);
        make_absent(&mut bus);
        let mut g = GridMeterGuard::new(); // GMR=1, RWGM=0 (default fixture)
        for _ in 0..48 {
            assert_eq!(g.tick(&bus).regulate, Regulation::Skip);
        }
        assert!(g.tick(&bus).alarm_no_grid_meter);
    }

    #[test]
    fn genset_regulates_without_meter_and_never_alarms() {
        let mut bus = grid_present(230.0);
        make_absent(&mut bus);
        bus.set(SYS, P_SOURCE, SOURCE_GENERATOR as f64);
        let mut g = GridMeterGuard::new();
        for _ in 0..60 {
            let out = g.tick(&bus);
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
        let out = g.tick(&bus);
        assert_eq!(out.regulate, Regulation::Skip);
        assert_eq!(out.presence, Presence::NotApplicable);
        assert!(!out.alarm_no_grid_meter);
    }

    #[test]
    fn unreadable_multi_ac_in_skips() {
        let mut bus = grid_present(230.0);
        bus.clear(VEBUS, P_ACTIVEIN); // Multi AC-in unreadable ⇒ never write
        let mut g = GridMeterGuard::new();
        let out = g.tick(&bus);
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
        let out = g.tick(&bus);
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
        let out = g.tick(&bus);
        assert!(out.usable);
        assert_eq!(out.regulate, Regulation::OnGridMeter);
        assert_eq!(g.meter_service(), None); // adopted via system fallback
    }
}
