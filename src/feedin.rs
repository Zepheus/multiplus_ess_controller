//! Feed-in / export management + Sustain enactment (mode-3-replacement half).
//!
//! This is the subsystem the setpoint loop (`control.rs` + the `main.rs` envelope)
//! does NOT cover: it owns the VE.Bus feed-in / export *flags* and mirrors the
//! firmware's Sustain state into our discharge policy. empirically modelled from
//! the stock ESS loop (Venus OS v3.75); see the design notes §4 and §6.
//!
//! Everything here is *policy above the VE.Bus firmware*. Five outputs, all written
//! to the Multi's vebus service (`VEBUS`), never to our own hub4 service:
//!   * `/Hub4/DisableFeedIn`          (§4.2)
//!   * `/Hub4/DoNotFeedInOvervoltage` (§4.3, gated on firmware support)
//!   * `/Hub4/TargetPowerIsMaxFeedIn` (§4.4, gated on firmware support)
//!   * `/Hub4/FixSolarOffsetTo100mV`  (§4.5)
//!   * `/Hub4/L{n}/MaxFeedInPower`    (§4.6, gated on firmware support)
//! plus the DC-overvoltage charge-stop integrator (§4.11), whose `ov_accum` the
//! setpoint law consumes to zero any *positive* (charging) command.
//!
//! TWO correctness invariants that shape the whole module (spec §6 preamble):
//!   1. **Sustain is FIRMWARE-owned.** We only READ `/Hub4/Sustain` and mirror it
//!      into our discharge budget (-> DisableFeedIn=1, discharge clamp 0). We NEVER
//!      write it, and there is no sustain-voltage/hysteresis logic to port — the
//!      ESS assistant runs that even in mode 3.
//!   2. **"Overvoltage" here is the DC bus, not AC grid voltage.**
//!      `DoNotFeedInOvervoltage` gates *DC-coupled PV excess* feed-in; the §4.11
//!      integrator watches `Vdc` vs the BMS charge voltage. We deliberately DO NOT
//!      add an AC grid-code over-voltage trip — that lives in the Multi's grid-code
//!      firmware, and faking it here would double-act and be unsafe.
//!
//! Scope note: this is a DC-coupled, PV-less, single-Multi install. Per §6.9 the
//! AC-PV-inverter curtailment and the `OverruledShoreLimit` peak-shave path are
//! out of scope (they need grid-meter per-phase current/voltage wiring we do not
//! do here). Their absence is faithful for this site: with no shore-limit IIR we
//! never report input-limit saturation, so the mode selector takes the same branch
//! it would when the input current limit is simply not being hit.

use crate::dbus::*;

// --- wire-contract constants (lifted verbatim; the GUI, modbus registers and Multi
// firmware all expect these exact values — spec §6.4 / §6.8). ---
const MAXFEEDIN_UNLIMITED: f64 = 200_000.0; // sentinel: "no feed-in limit"
const SLEW_UP_NEW: f64 = 0.6; // upward IIR on MaxFeedInPower: new = 0.6*new + 0.4*prev
const SLEW_UP_PREV: f64 = 0.4; // downward steps are immediate
const OV_CHARGE_FACTOR: f64 = 1.01; // excess = Vdc - 1.01*ChargeVoltage
const OV_TRIP: f64 = 1.0; // ov_accum > 1.0 => block positive (charging) setpoint
const KEEPCHARGED_SOC: f64 = 99.0; // MinimumSocLimit > 99 => keepCharged

const AC_SOURCE_GENERATOR: i32 = 2; // /Ac/ActiveIn/Source == 2 => genset (never back-feed)
const BL_SUSTAIN_STATE: i32 = crate::states::bl::SUSTAIN; // firmware Sustain
const MAX_PHASES: usize = 4; // single vebus service: totals + L1..L3

/// BatteryLife states at/below the SOC floor: {5,6,8,11,12} (the `(0xCB>>(s-5))&1`
/// bitmask from the reference implementation). In these states feed-in is disabled unless peak shaving
/// must stay available.
fn is_floor_state(state: i32) -> bool {
    matches!(state, 5 | 6 | 8 | 11 | 12)
}

// --- local dbus path consts (spec §2 / §6.6). Reused from `dbus.rs`: SETTINGS, SYS,
// VEBUS, P_SUSTAIN, P_DC_VOLTAGE, P_DVCC, P_HUB4MODE, P_MAXDISCHARGE, P_BL_STATE,
// P_BL_MINSOC. ---
const P_OVERVOLTAGE_FEEDIN: &str = "/Settings/CGwacs/OvervoltageFeedIn"; // SETTINGS, bool
const P_MAXFEEDIN_SETTING: &str = "/Settings/CGwacs/MaxFeedInPower"; // SETTINGS, W (NaN/<0 = unlimited)
const P_ACEXPORT_LIMIT: &str = "/Settings/CGwacs/AcExportLimit"; // SETTINGS, A
const P_ALWAYS_PEAKSHAVE: &str = "/Settings/CGwacs/AlwaysPeakShave"; // SETTINGS, bool
const P_AC_SOURCE: &str = "/Ac/ActiveIn/Source"; // SYS, int (2 = generator)
const P_CHARGE_VOLTAGE: &str = "/Dc/Battery/ChargeVoltage"; // SYS, V (DC-overvoltage ref)
const P_NUM_PHASES: &str = "/Ac/NumberOfPhases"; // VEBUS, int
const P_MULTI_STATE: &str = "/State"; // VEBUS, validity gate

// Output paths on VEBUS.
const P_DISABLE_FEEDIN: &str = "/Hub4/DisableFeedIn"; // int 0/1
const P_DNFIO: &str = "/Hub4/DoNotFeedInOvervoltage"; // int 0/1 (fw-gated)
const P_TPIMFI: &str = "/Hub4/TargetPowerIsMaxFeedIn"; // int 0/1 (fw-gated)
const P_FIX_SOLAR: &str = "/Hub4/FixSolarOffsetTo100mV"; // int 0/1
fn p_maxfeedin(n: usize) -> String {
    format!("/Hub4/L{n}/MaxFeedInPower") // f64, per phase (fw-gated)
}

/// Snapshot of every feed-in flag this module can write, captured at take-over so a
/// hand-back can put back exactly what the stock daemon had written. Stock's write
/// side is write-if-changed against its own in-process cache: if we leave a flag
/// different from that cache while its DESIRED value equals the cache, it is never
/// rewritten — the divergence would persist indefinitely. Restoring the captured
/// values makes actual == stock's cache again.
#[derive(Clone, Debug, Default)]
pub struct FlagSnapshot {
    disable_feedin: Option<i32>,
    dnfio: Option<i32>,
    tpimfi: Option<i32>,
    fix_solar: Option<i32>,
    maxfeedin: [Option<f64>; 3],
}

pub fn snapshot_flags(bus: &dyn Bus) -> FlagSnapshot {
    FlagSnapshot {
        disable_feedin: bus.get_i32(VEBUS, P_DISABLE_FEEDIN),
        dnfio: bus.get_i32(VEBUS, P_DNFIO),
        tpimfi: bus.get_i32(VEBUS, P_TPIMFI),
        fix_solar: bus.get_i32(VEBUS, P_FIX_SOLAR),
        maxfeedin: [
            bus.get_f64(VEBUS, &p_maxfeedin(1)),
            bus.get_f64(VEBUS, &p_maxfeedin(2)),
            bus.get_f64(VEBUS, &p_maxfeedin(3)),
        ],
    }
}

/// Best-effort restore (unreadable-at-capture items are skipped; write errors are
/// reported but do not abort the remaining restores).
pub fn restore_flags(bus: &dyn Bus, snap: &FlagSnapshot) {
    let put_i32 = |path: &str, v: Option<i32>| {
        if let Some(v) = v {
            if let Err(e) = bus.set_i32(VEBUS, path, v) {
                eprintln!("flag restore {path}={v} failed: {e}");
            }
        }
    };
    put_i32(P_DISABLE_FEEDIN, snap.disable_feedin);
    put_i32(P_DNFIO, snap.dnfio);
    put_i32(P_TPIMFI, snap.tpimfi);
    put_i32(P_FIX_SOLAR, snap.fix_solar);
    for (n, v) in snap.maxfeedin.iter().enumerate() {
        if let Some(v) = *v {
            let path = p_maxfeedin(n + 1);
            if let Err(e) = bus.set_f64(VEBUS, &path, v) {
                eprintln!("flag restore {path}={v} failed: {e}");
            }
        }
    }
}

/// The feed-in tri-state (spec §4.1). The exact numeric values are not written to the
/// bus (we write the *derived* flags), but the three-way choice drives every flag.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FeedInMode {
    /// mode 0: force-charge, `AcPowerSetpoint` reinterpreted as MAX FEED-IN; the Multi
    /// self-charges from all sources. Needs DVCC + firmware TPIMFI support.
    Tpimfi,
    /// mode 1: force-charge flat-out (setpoint slammed positive; the clamp caps it).
    ForceFlat,
    /// mode 2: normal operation — track the configured grid target.
    Normal,
}

/// External `/Overrides/*` (served by the hub4 D-Bus server, spec §6.2). Until that
/// server exists, defaults apply: no override. All fields are volatile per the reference implementation's
/// invalidate-overrides watchdog.
#[derive(Clone, Copy)]
pub struct Overrides {
    /// Live merged discharge bound from the battery broker ∧ SocGuard (W; NaN = unknown/
    /// unlimited). Feeds canDischarge (§4.2) so a BMS alarm (DCL → 0), an ephemeral
    /// override, or a BL no-discharge state disables feed-in exactly as stock does —
    /// without this, feed-in excess could drain the battery around the setpoint clamp.
    pub discharge_bound_w: f64,
    /// `/Overrides/FeedInExcess`: 0 = no override, 1 = force NO feed-in, 2 = force feed-in.
    pub feed_in_excess: i32,
    /// `/Overrides/ForceCharge`.
    pub force_charge: bool,
}

impl Default for Overrides {
    fn default() -> Self {
        Overrides { feed_in_excess: 0, force_charge: false, discharge_bound_w: f64::NAN }
    }
}

/// Raw per-tick reads (spec §6.6). `None` / NaN sentinels are handled by the caller.
struct Inputs {
    multi_state: Option<f64>, // /State — None => invalid => gate the whole tick
    hub4mode: i32,
    sustain: bool,
    dc_voltage: f64,
    charge_voltage: f64,
    phases: usize,
    ac_source: i32,
    dvcc: bool,
    overvoltage_feedin: bool,
    always_peakshave: bool,
    maxfeedin_setting: f64, // NaN => unlimited
    acexport_limit: f64,    // NaN => none
    max_discharge: f64,     // merged budget approximation; NaN => unknown
    bl_state: i32,
    bl_min_soc: f64,
}

/// `-1` (and any negative) is Victron's "off/unset" sentinel -> NaN (= unknown).
fn neg1_nan(v: Option<f64>) -> f64 {
    match v {
        Some(x) if x >= 0.0 => x,
        _ => f64::NAN,
    }
}

/// Write-if-changed cache. We mirror the reference implementation's write-only-on-change setters so the
/// MK3 link is never spammed. `None` = never written yet (or path absent on this fw).
#[derive(Clone, Copy, Default)]
struct LastWritten {
    disable_feedin: Option<i32>,
    dnfio: Option<i32>,
    tpimfi: Option<i32>,
    fix_solar: Option<i32>,
    maxfeedin: [Option<f64>; MAX_PHASES],
}

pub struct FeedIn {
    mode: FeedInMode,
    prev_maxfeedin: [f64; MAX_PHASES], // per-phase upward-slew memory (§4.6), init 0
    ov_accum: f64,                     // DC-overvoltage integrator (§4.11), init 0
    last: LastWritten,                 // write-if-changed cache
    overrides: Overrides,              // filled by the hub4 server (§6.2); default = none
    // Firmware-support gates, probed once at startup: writes to paths an old Multi does
    // not publish must be skipped forever (spec §6.6 / §6.8 item 8).
    has_dnfio: bool,
    has_tpimfi: bool,
    has_maxfeedin: bool,
}

impl FeedIn {
    pub fn new(bus: &dyn Bus) -> Self {
        // A path present on the bus answers GetValue; a missing item errors -> None.
        let has_dnfio = bus.get_f64(VEBUS, P_DNFIO).is_some();
        let has_tpimfi = bus.get_f64(VEBUS, P_TPIMFI).is_some();
        let has_maxfeedin = bus.get_f64(VEBUS, &p_maxfeedin(1)).is_some();
        eprintln!(
            "feedin: fw-gates dnfio={has_dnfio} tpimfi={has_tpimfi} maxfeedin={has_maxfeedin}"
        );
        FeedIn {
            mode: FeedInMode::Normal,
            prev_maxfeedin: [0.0; MAX_PHASES],
            ov_accum: 0.0,
            last: LastWritten::default(),
            overrides: Overrides::default(),
            has_dnfio,
            has_tpimfi,
            has_maxfeedin,
        }
    }

    /// Update the cached `/Overrides/*` (called by the hub4 server, spec §6.2). Until
    /// that server exists the defaults from `new` stand.
    pub fn set_overrides(&mut self, ov: Overrides) {
        self.overrides = ov;
    }

    fn read(&self, bus: &dyn Bus) -> Inputs {
        let phases = bus
            .get_i32(VEBUS, P_NUM_PHASES)
            .map(|p| (p.max(1) as usize).min(MAX_PHASES - 1))
            .unwrap_or(1);
        Inputs {
            multi_state: bus.get_f64(VEBUS, P_MULTI_STATE),
            hub4mode: bus.get_i32(SETTINGS, P_HUB4MODE).unwrap_or(3), // unknown => 3 (do nothing)
            sustain: bus.get_bool(VEBUS, P_SUSTAIN).unwrap_or(false),
            dc_voltage: bus.get_f64(VEBUS, P_DC_VOLTAGE).unwrap_or(f64::NAN),
            charge_voltage: bus.get_f64(SYS, P_CHARGE_VOLTAGE).unwrap_or(f64::NAN),
            phases,
            ac_source: bus.get_i32(SYS, P_AC_SOURCE).unwrap_or(0),
            dvcc: bus.get_bool(SYS, P_DVCC).unwrap_or(false),
            overvoltage_feedin: bus.get_bool(SETTINGS, P_OVERVOLTAGE_FEEDIN).unwrap_or(false),
            always_peakshave: bus.get_bool(SETTINGS, P_ALWAYS_PEAKSHAVE).unwrap_or(false),
            maxfeedin_setting: neg1_nan(bus.get_f64(SETTINGS, P_MAXFEEDIN_SETTING)),
            acexport_limit: neg1_nan(bus.get_f64(SETTINGS, P_ACEXPORT_LIMIT)),
            max_discharge: neg1_nan(bus.get_f64(SETTINGS, P_MAXDISCHARGE)),
            bl_state: bus.get_i32(SETTINGS, P_BL_STATE).unwrap_or(0),
            bl_min_soc: bus.get_f64(SETTINGS, P_BL_MINSOC).unwrap_or(0.0),
        }
    }

    /// keepCharged (§4.1): BL state 9 or a "keep charged" minimum-SOC setting.
    fn keep_charged(&self, i: &Inputs) -> bool {
        i.bl_state == 9 || i.bl_min_soc > KEEPCHARGED_SOC
    }

    /// mustForceCharge (§4.1 / §6.6). `batteryChargeRequested` (BMS `Info/ChargeRequest`)
    /// belongs to the battery subsystem and is not read here; defaulting it to false is
    /// safe (it can only *raise* force-charge, never lower it).
    fn must_force_charge(&self, i: &Inputs) -> bool {
        self.overrides.force_charge
            || self.keep_charged(i)
            || matches!(i.bl_state, 8 | 12)
    }

    /// feedInExcessAllowed (§4.1): the override wins if set, else the settings flag.
    fn feed_in_excess_allowed(&self, i: &Inputs) -> bool {
        if self.overrides.feed_in_excess != 0 {
            self.overrides.feed_in_excess == 2
        } else {
            i.overvoltage_feedin
        }
    }

    /// Feed-in tri-state (§4.1). `fw_tpimfi` = DVCC AND firmware TPIMFI support. The
    /// "any shore limit saturated => mode 1" branch is faithfully a no-op here: we do
    /// not compute the OverruledShoreLimit peak-shave IIR (§6.9), so the input current
    /// limit is never reported saturated and the branch is not taken.
    fn compute_mode(&self, i: &Inputs) -> FeedInMode {
        let fw_tpimfi = i.dvcc && self.has_tpimfi;
        if i.ac_source == AC_SOURCE_GENERATOR {
            // Never back-feed a genset: TPIMFI cap if supported, else charge-flat.
            if fw_tpimfi {
                FeedInMode::Tpimfi
            } else {
                FeedInMode::ForceFlat
            }
        } else if !self.must_force_charge(i) {
            FeedInMode::Normal
        } else if fw_tpimfi {
            if self.feed_in_excess_allowed(i) {
                FeedInMode::ForceFlat // force-charge, excess may be sold
            } else {
                // (else-if any shore saturated => ForceFlat: not reached, see §6.9)
                FeedInMode::Tpimfi // force-charge, feed-in capped by the setpoint
            }
        } else {
            FeedInMode::ForceFlat // force-charge, no TPIMFI available
        }
    }

    /// canDischarge (§4.2): the merged discharge budget is `> 0`, OR unknown (NaN =>
    /// allowed, matching the binary — see §6.8). The budget is the settings
    /// `MaxDischargePower` merged with the live broker/SocGuard bound handed in via
    /// `Overrides.discharge_bound_w` (BMS DCL, ephemeral override, BL no-discharge
    /// states), and forced to 0 while Sustain is active or BL state == 7.
    fn can_discharge(&self, i: &Inputs) -> bool {
        let mut max_dis = i.max_discharge; // NaN-ignoring merge: f64::min skips NaN
        let ovr = self.overrides.discharge_bound_w;
        if ovr.is_finite() {
            max_dis = if max_dis.is_finite() { max_dis.min(ovr) } else { ovr };
        }
        if i.sustain || i.bl_state == BL_SUSTAIN_STATE {
            max_dis = 0.0;
        }
        !max_dis.is_finite() || max_dis > 0.0
    }

    /// Per-phase `/Hub4/L{n}/MaxFeedInPower` targets (§4.6, reduced per §6.9 to
    /// `total / phases` with upward-only 0.6/0.4 slew — no AC-PV reserve term here).
    fn compute_maxfeedin(&mut self, i: &Inputs, excess_ok: bool) -> [f64; MAX_PHASES] {
        let maxfeedin_finite = i.maxfeedin_setting.is_finite();
        let acexport_finite = i.acexport_limit.is_finite();
        let mut out = [f64::NAN; MAX_PHASES];

        if excess_ok && !maxfeedin_finite && !acexport_finite {
            // "Unlimited": write the exact sentinel (no slew), per §4.6's dedicated
            // branch and §6.8 ("keep the exact sentinels"). Reset slew memory so a later
            // finite limit steps down immediately rather than from 200 kW.
            for n in 0..i.phases {
                out[n] = MAXFEEDIN_UNLIMITED;
                self.prev_maxfeedin[n] = MAXFEEDIN_UNLIMITED;
            }
            return out;
        }

        // Finite cap. `MaxFeedInPower` unlimited/NaN => 200 kW; AcExportLimit->W
        // conversion needs grid-meter per-phase voltage we do not wire here, so it is
        // treated as unlimited (documented deviation, §6.6). The min() therefore reduces
        // to the configured MaxFeedInPower when set.
        let total = if maxfeedin_finite {
            i.maxfeedin_setting
        } else {
            MAXFEEDIN_UNLIMITED
        };
        let per_phase = total / i.phases as f64;
        for n in 0..i.phases {
            let mut new = per_phase;
            if new > self.prev_maxfeedin[n] {
                new = SLEW_UP_NEW * new + SLEW_UP_PREV * self.prev_maxfeedin[n]; // slew up only
            }
            self.prev_maxfeedin[n] = new;
            out[n] = new;
        }
        out
    }

    /// One slow-tick update (spec §6.5). Reads live inputs, computes the flags + the DC-
    /// overvoltage integrator, updates the write-if-changed cache, and returns the
    /// `Output` (flag snapshot + a `write` method). Pure of side effects on the bus:
    /// the caller decides whether to `Output::write` (live) or discard it (shadow).
    pub fn tick(&mut self, bus: &dyn Bus) -> Output {
        let i = self.read(bus);

        // Gate: Multi /State unreadable => invalid => write nothing, hold last flags,
        // do not advance the integrator (spec §4.12 / §6.5 step 1). NOTE: unlike the
        // stock binary we do NOT bail on Hub4Mode==3 — in this full replacement WE are
        // the mode-3 owner of these vebus flags (spec §6.5/§6.7). `hub4mode` is read only
        // to fail safe (unknown => 3) for a future minimal-integration handback.
        if i.multi_state.is_none() {
            return Output::gated(self.mode, self.ov_accum, i.sustain);
        }
        let _ = i.hub4mode;

        let mode = self.compute_mode(&i);
        let excess_ok = self.feed_in_excess_allowed(&i);
        let can_discharge = self.can_discharge(&i);

        // §4.2 /Hub4/DisableFeedIn
        let disable_feedin = if mode == FeedInMode::Tpimfi {
            0 // TPIMFI needs the feed-in path open
        } else if !can_discharge {
            1 // battery empty / BMS-blocked / sustain
        } else if is_floor_state(i.bl_state) {
            i32::from(!i.always_peakshave)
        } else {
            0
        };

        // §4.3 /Hub4/DoNotFeedInOvervoltage (DC-coupled excess; 0 = allowed)
        let allow_dc_excess = i.ac_source != AC_SOURCE_GENERATOR
            && (self.overrides.feed_in_excess == 2
                || (self.overrides.feed_in_excess == 0 && i.overvoltage_feedin));
        let dnfio = if allow_dc_excess {
            0
        } else {
            i32::from(mode != FeedInMode::Tpimfi) // also 0 in TPIMFI mode
        };

        // §4.4 /Hub4/TargetPowerIsMaxFeedIn
        let tpimfi = i32::from(mode == FeedInMode::Tpimfi);

        // §4.5 /Hub4/FixSolarOffsetTo100mV = DVCC
        let fix_solar = i32::from(i.dvcc);

        // §4.6 /Hub4/L{n}/MaxFeedInPower
        let maxfeedin = self.compute_maxfeedin(&i, excess_ok);

        // §4.11 DC-overvoltage charge-stop integrator. NaN-safe: a NaN excess makes the
        // `> 0` test false, resetting the accumulator (never accumulate NaN).
        let excess = i.dc_voltage - OV_CHARGE_FACTOR * i.charge_voltage;
        self.ov_accum = if excess > 0.0 { self.ov_accum + excess * excess } else { 0.0 };

        self.mode = mode;

        // Build the write-plan (diff vs last-written), honouring firmware gates. Update
        // the cache optimistically: a value re-emits when it next changes, so a dropped
        // write costs at most one stale tick and never MK3 spam.
        let plan_i32 = |last: &mut Option<i32>, gated_present: bool, val: i32| -> Option<i32> {
            if !gated_present {
                return None; // path not on this firmware => never write
            }
            if *last == Some(val) {
                None
            } else {
                *last = Some(val);
                Some(val)
            }
        };
        let w_disable_feedin = plan_i32(&mut self.last.disable_feedin, true, disable_feedin);
        let w_dnfio = plan_i32(&mut self.last.dnfio, self.has_dnfio, dnfio);
        let w_tpimfi = plan_i32(&mut self.last.tpimfi, self.has_tpimfi, tpimfi);
        let w_fix_solar = plan_i32(&mut self.last.fix_solar, true, fix_solar);

        let mut w_maxfeedin: [Option<f64>; MAX_PHASES] = [None; MAX_PHASES];
        if self.has_maxfeedin {
            for n in 0..i.phases {
                let val = maxfeedin[n];
                if self.last.maxfeedin[n] != Some(val) {
                    self.last.maxfeedin[n] = Some(val);
                    w_maxfeedin[n] = Some(val);
                }
            }
        }

        Output {
            mode,
            ov_accum: self.ov_accum,
            sustain: i.sustain,
            disable_feedin,
            do_not_feed_in_overvoltage: dnfio,
            target_power_is_max_feedin: tpimfi,
            fix_solar_offset: fix_solar,
            max_feedin_power: maxfeedin,
            w_disable_feedin,
            w_dnfio,
            w_tpimfi,
            w_fix_solar,
            w_maxfeedin,
        }
    }
}

/// Result of one `FeedIn::tick`: the computed flag values (for telemetry / the setpoint
/// law) plus an internal write-plan. `write` emits only the changed, firmware-present
/// paths to the vebus service. In shadow mode the caller simply drops this.
pub struct Output {
    pub mode: FeedInMode,
    /// DC-overvoltage integrator (§4.11). The setpoint law forces any *positive*
    /// (charging) command to 0 while this exceeds the trip.
    pub ov_accum: f64,
    /// Firmware Sustain (read-only). Mirrors into the discharge budget: while set the
    /// setpoint law's lower clamp is 0 (no discharge).
    pub sustain: bool,

    pub disable_feedin: i32,
    pub do_not_feed_in_overvoltage: i32,
    pub target_power_is_max_feedin: i32,
    pub fix_solar_offset: i32,
    pub max_feedin_power: [f64; MAX_PHASES],

    // write-plan: Some => changed since last write AND path present => emit.
    w_disable_feedin: Option<i32>,
    w_dnfio: Option<i32>,
    w_tpimfi: Option<i32>,
    w_fix_solar: Option<i32>,
    w_maxfeedin: [Option<f64>; MAX_PHASES],
}

impl Output {
    /// A gated (no-write) result when the Multi is invalid: carries the last mode /
    /// integrator so the setpoint law still fails safe, but plans no writes.
    fn gated(mode: FeedInMode, ov_accum: f64, sustain: bool) -> Self {
        Output {
            mode,
            ov_accum,
            sustain,
            disable_feedin: 0,
            do_not_feed_in_overvoltage: 0,
            target_power_is_max_feedin: 0,
            fix_solar_offset: 0,
            max_feedin_power: [f64::NAN; MAX_PHASES],
            w_disable_feedin: None,
            w_dnfio: None,
            w_tpimfi: None,
            w_fix_solar: None,
            w_maxfeedin: [None; MAX_PHASES],
        }
    }

    /// Does the DC-overvoltage integrator currently block charging? The setpoint law
    /// forces any positive command to 0 when true (§4.11).
    pub fn block_charge(&self) -> bool {
        self.ov_accum > OV_TRIP
    }

    /// mode 1: the setpoint law should command force-charge flat-out (§4.9).
    pub fn force_flat(&self) -> bool {
        self.mode == FeedInMode::ForceFlat
    }

    /// Write the changed flags to the vebus service (live mode only). Only paths that
    /// changed since the previous write AND exist on this firmware are emitted, so the
    /// MK3 link is never spammed. IMPORTANT: this only ever touches `/Hub4/*` *flag*
    /// paths — it NEVER writes `/Hub4/Sustain` (firmware-owned, read-only).
    pub fn write(&self, bus: &dyn Bus) -> Result<(), String> {
        if let Some(v) = self.w_disable_feedin {
            bus.set_i32(VEBUS, P_DISABLE_FEEDIN, v)?;
        }
        if let Some(v) = self.w_dnfio {
            bus.set_i32(VEBUS, P_DNFIO, v)?;
        }
        if let Some(v) = self.w_tpimfi {
            bus.set_i32(VEBUS, P_TPIMFI, v)?;
        }
        if let Some(v) = self.w_fix_solar {
            bus.set_i32(VEBUS, P_FIX_SOLAR, v)?;
        }
        // w_maxfeedin is indexed 0-based by phase; the dbus paths are 1-based (L1..L3).
        for (n, slot) in self.w_maxfeedin.iter().enumerate() {
            if let Some(v) = slot {
                bus.set_f64(VEBUS, &p_maxfeedin(n + 1), *v)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "feedin_tests.rs"]
mod tests;
