//! Pure decision core — the `decide()` phase of the sample/decide/actuate split
//! (re/ROLLOUT-AND-FAILSAFE.md §3.1).
//!
//!   sample()  (main.rs)  — does EVERY bus read, packs a `Snapshot`. The only reader.
//!   decide()  (here)     — PURE, total: `Snapshot × LoopState → Decision`. No I/O, so the
//!                          whole composed control law is unit-testable + replayable off-host.
//!   actuate() (main.rs)  — does EVERY write from the `Decision`. The single place the final
//!                          command is emitted, and therefore where write-phase safety checks
//!                          belong.
//!
//! Keeping `decide()` free of I/O is the whole point: shadow-vs-live becomes purely "does the
//! caller run actuate()", and any composition regression is caught by a host unit test.

use crate::control::Controller;
use crate::gridmeter::HUB4MODE_EXTERNAL;
use crate::shore::ShoreOut;
use crate::socguard::SocGuardOut;
use crate::Stage;

// --- Design anchor -----------------------------------------------------------------------
// The safety layer is parameterised by ONE hardware number — the inverter pair's continuous
// AC power — so the slew / sanity / clamp thresholds scale with the actual machine instead of
// being independently hand-tuned. The battery power envelope is already inferred live from the
// BMS (battery_limits: DCL/CCL x Vdc); this anchors the inverter side.

/// Inverter pair continuous AC power (W): the single hardware fact for this install
/// (2x MultiPlus-II 48/5000, ~3700 W continuous each @ 40 C). Overridable at runtime by the
/// --max-charge/--max-discharge envelope, so this is the only place the hardware is named.
pub const INVERTER_CONT_PAIR_W: f64 = 7400.0;
/// Never command past this fraction of continuous rating.
pub const CAPACITY_MARGIN: f64 = 0.95;
/// Absolute design-capacity command clamp (W), both directions.
pub const HARD_CLAMP_W: f64 = INVERTER_CONT_PAIR_W * CAPACITY_MARGIN;

/// Glitch-guard slew allows at most a full design-range move in this many seconds — so it only
/// blunts a gross single-tick jump, never normal control (the Multi already ramps ~400 W/s).
pub const SLEW_TRAVERSE_S: f64 = 2.0;
/// "Gross" grid divergence for the sanity supervisor = this fraction of design power.
pub const SANITY_FRACTION: f64 = 0.15;
/// Sanity dwell before hand-back / neutralise (s).
pub const SANITY_SECS: f64 = 30.0;
/// dt-clamp floor (s); the runtime uses max(this, 5 x control interval).
pub const MIN_DT_CLAMP_S: f64 = 5.0;

/// Safety parameters DERIVED from the design power + control interval (each overridable).
/// Constructed once at startup; there are no independent magic constants to tune.
#[derive(Clone, Copy, Debug)]
pub struct SafetyLimits {
    pub design_w: f64,
    pub slew_w_per_s: f64,
    pub sanity_band_w: f64,
    pub sanity_secs: f64,
    pub max_dt_s: f64,
}

impl SafetyLimits {
    /// `design_w` is the inverter envelope (the --max-* override, else `HARD_CLAMP_W`).
    /// A `None` override falls back to the value derived from `design_w`.
    pub fn derive(
        design_w: f64,
        interval_s: f64,
        slew_override: Option<f64>,
        band_override: Option<f64>,
    ) -> Self {
        SafetyLimits {
            design_w,
            slew_w_per_s: slew_override.unwrap_or(design_w / SLEW_TRAVERSE_S),
            sanity_band_w: band_override.unwrap_or(design_w * SANITY_FRACTION),
            sanity_secs: SANITY_SECS,
            max_dt_s: (interval_s * 5.0).max(MIN_DT_CLAMP_S),
        }
    }
}

/// Which write-phase safety interventions fired this tick. Surfaced so a trip is easy to
/// detect: it maps to a Victron-style alarm level (for the Tier-B `/Alarms/EssSafety` signal
/// the HA/VRM integration reads natively) and a stable reason string for logs + telemetry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Safety {
    /// dt was clamped (a scheduler stall / long tick was detected).
    pub dt_clamped: bool,
    /// The slew guard limited this tick's commanded step (a glitchy jump was blunted).
    pub slew_clamped: bool,
    /// The sanity supervisor tripped: grid diverged grossly while we drove -> handing back.
    pub sanity_tripped: bool,
}

impl Safety {
    pub fn any(self) -> bool {
        self.dt_clamped || self.slew_clamped || self.sanity_tripped
    }
    /// Victron alarm convention (0 = ok, 1 = warning, 2 = alarm), for `/Alarms/EssSafety`.
    /// Sanity handback is an alarm; slew/dt clamps are warnings.
    pub fn alarm_level(self) -> i32 {
        if self.sanity_tripped {
            2
        } else if self.slew_clamped || self.dt_clamped {
            1
        } else {
            0
        }
    }
    /// Stable, greppable reason for the `SAFETY:` log line + the telemetry reason column.
    pub fn reason(self) -> &'static str {
        if self.sanity_tripped {
            "sanity-handback"
        } else if self.slew_clamped {
            "slew-clamped"
        } else if self.dt_clamped {
            "dt-clamped"
        } else {
            ""
        }
    }
}

/// Glitch guard: limit how fast the written value may move (W/s). Pure.
fn slew_limit(prev: Option<f64>, desired: f64, max_rate_w_s: f64, dt: f64) -> f64 {
    match prev {
        Some(p) if p.is_finite() => {
            let step = (max_rate_w_s * dt).max(0.0);
            p + (desired - p).clamp(-step, step)
        }
        _ => desired,
    }
}

/// Stock's setpoint write law (matched to the stock daemon's per-phase writer exactly): an EMA toward the
/// command, `new = prev + gain·(cmd − prev)`, with
///   gain = 0.5                                   for |Δ| < 10 W, or with `adaptive` off;
///   gain = clamp(0.25·ln(|Δ|/10), 0.5, 1.1)      otherwise — a DELIBERATE overshoot up
///          to 1.1× on large steps (plausibly countering the Multi's ~4 % under-tracking).
/// Stock enables the adaptive variant only with a fast grid meter (< 250 ms updates) AND
/// Multi firmware > 0x1C1FF; on this install the live traces (monotone hold convergence
/// −160 → −80 → −38 → +30, no overshoot) show the flat-0.5 branch. First write (no prev)
/// passes through. The result is rounded to the integer watt, half away from zero — stock
/// writes integers (every live-captured setpoint is one). Pure.
/// Identified plant + meter chain (system identification 2026-08-22 from live
/// actuation traces; identified 2026-08-22 from live actuation traces).
/// The Multi's AC power follows the written setpoint through a 1 s transport
/// delay and a first-order response (per-second alpha 0.8, tau ~1.25 s); the
/// ET112 grid reading reaches the loop ~1 s stale (750 ms register + RS485).
/// POSTDICTION-VALIDATED: reproduces the captured microwave event (export
/// 3.15 vs 3.02 Wh real) and a captured stock step-off (peak -1944 vs -1925).
pub mod plant {
    /// rep = A + B * w_eff  (static tracking fit, 32k steady rows)
    pub const A: f64 = 4.7;
    pub const B: f64 = 0.9562;
    /// w_eff(t) = w_eff(t-1) + ALPHA * (write(t-1-DELAY_S) - w_eff(t-1))
    pub const ALPHA: f64 = 0.8;
    /// Transport delay (write to response onset). The production Smith state and
    /// the harness both realize it via the one-tick `pending` write buffer.
    #[allow(dead_code)]
    pub const DELAY_S: usize = 1;
    /// Meter-chain staleness of the grid reading as seen by the loop.
    pub const METER_LAG_S: usize = 1;
}

/// Write pacing, SYMMETRIC at 0.5/s — fast pacing is safe BECAUSE of the Smith
/// meter-lag compensation (the 2026-08-22 event study, 233 + 331 out-of-sample
/// events): without compensation, fast pacing chases the ~1 s-stale meter and
/// over-discharges into step-offs (the 2026-08-21 live exports); with it, the
/// smith+0.5 combination beats stock on export, import AND export duration.
/// History: stock-equivalent 0.242 was tried live and exported MORE than stock
/// (slow recovery, +11% on the event library); an asymmetric variant was
/// rejected (blocks the adaptive learn-gate's steadiness detector). If Smith is
/// disabled (--no-smith), consider that pacing trade-off before going live.
pub const EMA_GAIN_UP: f64 = 0.5;
pub const EMA_GAIN_DOWN: f64 = 0.5;

pub fn stock_write_ema(
    prev: Option<f64>,
    cmd: f64,
    adaptive: bool,
    gain_up: f64,
    gain_down: f64,
) -> f64 {
    let v = match prev {
        Some(p) if p.is_finite() => {
            let delta = cmd - p;
            let gain = if adaptive && delta.abs() >= 10.0 {
                (0.25 * (delta.abs() / 10.0).ln()).clamp(0.5, 1.1)
            } else if delta < 0.0 {
                gain_down
            } else {
                gain_up
            };
            p + gain * delta
        }
        _ => cmd,
    };
    v.round()
}

/// Live sensor scalars (grid/reported/target in W, soc in %, state = systemcalc code).
#[derive(Clone, Copy, Debug)]
pub struct Sensors {
    pub grid: f64,
    pub reported: f64,
    pub target: f64,
    pub soc: f64,
    pub state: i64,
}

/// Everything `decide()` needs, gathered by `sample()` in one read pass. Plain data only —
/// no bus handle — so `decide()` structurally cannot perform I/O.
pub struct Snapshot {
    pub s: Sensors,
    pub dt: f64,
    /// The raw dt exceeded the clamp and was clamped (set by the caller) — a stall marker.
    pub dt_clamped: bool,
    /// LIVE `/Settings/CGwacs/BatteryLife/MinimumSocLimit` (%), read each tick — the ownership
    /// floor. It is UI/VRM/API-controlled and is raised by the user's dispatch automation, so
    /// it must never be a static value. Falls back to `--min-soc` only if the read fails.
    pub min_soc: f64,
    /// Design-capacity-bounded command clamp from the battery broker (W).
    pub lo: f64,
    pub hi: f64,
    /// Effective control target = DESS override applied to the setpoint + PV feed-forward (W).
    pub effective_target: f64,
    /// SocGuard/BatteryLife enactment (discharge inhibit, force-charge, floor).
    pub sg: SocGuardOut,
    /// Feed-in composition results (booleans extracted from the feed-in module output).
    pub feed_block_charge: bool,
    pub feed_force_flat: bool,
    /// Generator export inhibit.
    pub gen_disable_export: bool,
    /// Grid-meter guard decision (cached at the 2.5 s presence cadence).
    pub gm_should_write: bool,
    /// Shore/peak-shave limiter output (written verbatim in takeover).
    pub shore_out: ShoreOut,
    /// Stock the stock daemon's current setpoint, for the shadow-fidelity delta (read-only).
    pub hub4_actual: Option<f64>,
    /// Measured Multi AC-out power (W, vebus `/Ac/Out/L1/P`; NaN = unread, treated as 0).
    /// Feedforward term for battery-frame clamps — see `composed_command`.
    pub ac_out_w: f64,
    /// Live `/Settings/CGwacs/Hub4Mode` read this tick (None = unreadable). Takeover uses
    /// it to (a) cede immediately when someone external flips the mode away from
    /// EXTERNAL while we own, and (b) keep re-asserting the stock mode after a
    /// hand-back until the restore actually sticks.
    pub hub4mode_live: Option<i32>,
}

/// State carried across ticks; `decide()` mutates it (integrator, ownership, trim state).
pub struct LoopState {
    pub ctrl: Controller,
    pub owner_us: bool,
    /// `t` (seconds since start) of the last ownership flip — anti-thrash dwell reference.
    pub last_flip_t: f64,
    /// Stage-1 outer-integral state: Some(override_w) while trimming, None otherwise.
    pub trim_override: Option<f64>,
    /// Last actuated value (W) for the slew guard; None => start fresh (not currently driving).
    pub last_out: Option<f64>,
    /// `t` when grid first went out of the sanity band while driving; None while in-band.
    pub oob_since: Option<f64>,
    /// Previous tick's "grid legitimately diverges" flag: force-charge OR an external
    /// discharge cap (`/Overrides/MaxDischargePower` — schedule post-target hold — or a
    /// BL discharged state). The sanity check is suppressed while it was set; using last
    /// tick avoids an ordering dependency.
    pub prev_force_charge: bool,
    /// Smith-style meter-lag compensation state (see `decide`): a running model
    /// of the Multi's effective drive (`plant::ALPHA` first-order over our own
    /// write history) plus a one-tick history so the stale grid reading can be
    /// corrected by the battery-side change since it was sampled. NaN = unseeded
    /// (reseeded from the measured Multi power on the next actuating tick).
    pub smith_weff: f64,
    /// w_eff METER_LAG_S ticks ago (the value the stale meter reading "saw").
    pub smith_weff_lag: f64,
    /// Our write from the previous tick (enters the plant after DELAY_S).
    pub smith_prev_write: f64,
    /// Latched when an EXTERNAL actor flipped Hub4Mode away while we owned (user, VRM,
    /// another tool): we cede and never re-take until the process is restarted — a
    /// deliberate operator action must not be fought by an auto re-take.
    pub ceded_external: bool,
    /// Sanity-trip latch: how many times the sanity supervisor has forced a hand-back this
    /// run. Each trip DOUBLES the dwell required before re-takeover (capped), so a
    /// persistently-wrong controller escalates to hour-scale retries instead of flip-flopping
    /// mode 3<->1 every `min_dwell_s`. Never reset within a run (a restart starts clean).
    pub sanity_trips: u32,
}

/// Immutable per-run config for `decide()`.
pub struct DecideCfg {
    pub stage: Stage,
    /// SOC hysteresis (%) above the LIVE floor (`Snapshot.min_soc`) before re-taking control.
    pub soc_hyst: f64,
    pub min_dwell_s: f64,
    pub trim_ki: f64,
    pub orig_mode: i32,
    pub state_recharge: i64,
    /// Glitch-guard slew (W/s) applied to the actuated value.
    pub slew_w_per_s: f64,
    /// Sanity supervisor band (W) and dwell (s).
    pub sanity_band_w: f64,
    pub sanity_secs: f64,
    /// Adaptive EMA gain in the write law (see `stock_write_ema`). Stock enables it only
    /// with a fast grid meter (< 250 ms) + Multi fw > 0x1C1FF; false = flat 0.5 (this
    /// install's live-observed behavior).
    pub ema_adaptive: bool,
    /// Directional write-pacing gains (see `EMA_GAIN_UP`/`EMA_GAIN_DOWN`).
    pub ema_gain_up: f64,
    pub ema_gain_down: f64,
    /// Smith-style meter-lag compensation (see `decide`). Event-library study
    /// (233 events, 2026-08-22): with compensation + 0.5 pacing the loop beats
    /// stock on export (-11%), import (-30%) AND export duration (-15%);
    /// without it, slow pacing alone exported MORE than stock.
    pub smith: bool,
}

/// What `actuate()` should write this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Write {
    Nothing,
    /// VEBUS /Hub4/L1/AcPowerSetpoint (takeover).
    Setpoint(f64),
    /// hub4 /Overrides/Setpoint (trim).
    Override(f64),
}

/// The output of `decide()`, consumed by `actuate()` + the loggers.
#[derive(Clone, Debug)]
pub struct Decision {
    /// Final composed command (design-clamped), for display + the shadow delta.
    pub command: f64,
    pub write: Write,
    /// Some(m) => write Hub4Mode this tick (takeover ownership flip).
    pub hub4mode: Option<i32>,
    /// Also write feed-in + shore flags (takeover, owning, meter usable).
    pub feed_shore_write: bool,
    pub owner_us: bool,
    /// Value to show/telemeter (the override in trim, else the command).
    pub display_cmd: f64,
    pub owner_str: &'static str,
    pub note: &'static str,
    /// Ownership changed this tick (caller emits the stderr transition event).
    pub flipped: bool,
    /// Which write-phase safety interventions fired this tick (for logs/telemetry/alarms).
    pub safety: Safety,
}

/// The composed control law: integral fix → SocGuard → feed-in → generator → design clamp.
/// Pure; factored out so both `decide()` and its unit tests exercise the exact same math.
pub fn composed_command(
    snap: &Snapshot,
    ctrl: &mut Controller,
    actuating: bool,
    meter_corr_w: f64,
) -> f64 {
    let s = &snap.s;
    let mut cmd = ctrl.update(
        s.grid + meter_corr_w,
        s.reported,
        snap.effective_target,
        snap.dt,
        snap.lo,
        snap.hi,
        actuating,
    );
    // SocGuard discharge inhibit / external cap, MIMICKING STOCK EXACTLY (live-verified
    // 2026-08-17, actuation tick + distributor + writer): stock builds its total command in
    // the acIn−acOut frame — total = clamp(target + acIn − grid − acOut, −maxdis, maxchg) —
    // and the per-phase write adds measured `Ac/Out/L{n}/P` back on top. In normal operation
    // acOut cancels exactly (the write IS the raw law); when a clamp binds, the written
    // setpoint floats at clamp + acOut so the BATTERY, not the setpoint, respects the cap
    // (live: hold writes of +30 W / +48 W = acOut on those mornings, cap 1 W). A bare
    // `max(-maxdis)` would instead trickle-discharge the battery by the Multis' output
    // load + losses for the whole hold. NaN acOut (unread) degrades to the bare clamp.
    // (−INF + acOut is still −INF, so this is a no-op without a cap.)
    let ac_out = if snap.ac_out_w.is_finite() { snap.ac_out_w } else { 0.0 };
    cmd = cmd.max(-snap.sg.max_discharge_w + ac_out);
    if let Some(fc) = snap.sg.cmd_override {
        // Force charge. TPIMF (mode 0): keep writing the normal-law value capped at
        // Σ max-charge — the firmware charges maximally on its own and reinterprets the
        // setpoint as a feed-in cap, so the value may legitimately go negative (§4a-4;
        // live-verified 2026-08-16 across a full 6 h window: stock writes the raw normal
        // law, median |Δ| 2.5 W). Legacy (mode 1): the setpoint IS the charge command.
        cmd = if snap.sg.tpimf { cmd.min(fc) } else { fc };
    }
    if snap.sg.charge_floor_w > 0.0 {
        // minimum-charge floor ONLY when set (BL state 6); 0 must not zero discharge.
        // Same frame as the discharge cap: stock's dcV·5 floor sits in the acIn−acOut
        // total, so the write floor is dcV·5 + acOut (charge 5 A AND serve the output).
        cmd = cmd.max(snap.sg.charge_floor_w + ac_out);
    }
    // STOCK PARITY (stock-behavior analysis + 48.8 h of shadow data with ZERO
    // positive stock writes in normal regimes): outside force-charge and the BL
    // minimum-charge floor, stock ZEROES any non-negative battery-frame total
    // before writing — the final AC command never exceeds measured AC-out, so the
    // battery cannot charge from the grid on transient meter skew (appliance
    // step-off) or any other normal-regime condition. Charging happens ONLY in
    // charge regimes (ForceCharge / ChargeRequest / KeepCharged / BL recharge).
    if !snap.sg.force_charge && snap.sg.charge_floor_w <= 0.0 {
        cmd = cmd.min(ac_out);
    }
    if snap.feed_block_charge {
        cmd = cmd.min(0.0);
    }
    if snap.gen_disable_export || snap.feed_force_flat {
        // Never export toward a genset, nor in feed-in force-flat mode.
        cmd = cmd.max(0.0);
    }
    // Final safety clamp (NaN-safe, design capacity).
    crate::control::clamp(cmd, snap.lo, snap.hi)
}

/// The pure per-tick decision. `t` is seconds-since-start (used only for the dwell timer, so
/// the function stays free of `Instant` and is fully deterministic in tests).
pub fn decide(snap: &Snapshot, st: &mut LoopState, cfg: &DecideCfg, t: f64) -> Decision {
    let s = snap.s;

    // Sanity supervisor (before ownership): while WE actually drive, the meter is usable, and
    // we're NOT in a (previous-tick) force-charge where grid legitimately diverges, require the
    // grid to stay within `sanity_band_w` of the setpoint. Beyond it for `sanity_secs` => trip,
    // which forces hand-back below. Catches a gross bug, not the small ~4 % leak.
    // A NON-FINITE grid error also counts as out-of-band: if we cannot verify sanity while
    // driving, hand back after `sanity_secs` and let stock + its meter guard deal with it
    // (a NaN must not silently RESET the out-of-band timer).
    let gerr = s.grid - s.target;
    let oob = st.owner_us
        && cfg.stage.writes()
        && snap.gm_should_write
        && !st.prev_force_charge
        && (!gerr.is_finite() || gerr.abs() > cfg.sanity_band_w);
    st.oob_since = if oob { Some(st.oob_since.unwrap_or(t)) } else { None };
    let sanity_tripped = st.oob_since.is_some_and(|t0| t - t0 >= cfg.sanity_secs);

    // 1. Ownership envelope — may own iff SOC is comfortably above the floor and we're not in
    //    a recharge state. Hysteresis + dwell prevent thrash; hand-BACK is never throttled.
    //    A sanity trip forces exit (hand back / neutralise) regardless of dwell, and each trip
    //    doubles the re-entry dwell (latch/backoff): a persistently-wrong controller escalates
    //    to hour-scale retries instead of oscillating mode 3<->1 forever.
    // 1b. External-mode verification (takeover only; read done in sample()). Runs
    // BEFORE the want/dwell computation so a cede takes effect this tick.
    let mut ext_note = "";
    let mut ext_mode_write: Option<i32> = None;
    if matches!(cfg.stage, Stage::Takeover) {
        match snap.hub4mode_live {
            // Someone else (user / VRM / another tool) flipped the mode away while we
            // owned: the stock loop is running again. Cede IMMEDIATELY, do not fight
            // the change, and never auto re-take — a deliberate operator action stays
            // ceded until this process is restarted. (The sanity supervisor cannot
            // catch this: both laws track the same target, so grid stays in-band.)
            Some(m) if st.owner_us && m != HUB4MODE_EXTERNAL => {
                st.owner_us = false;
                st.last_flip_t = t;
                st.ceded_external = true;
                ext_note = "mode changed externally: ceded (sticky)";
            }
            // Handed back but the mode still reads EXTERNAL: our restore write never
            // landed (transient dbus error) or a previous run died mid-takeover. Stock
            // gates its whole loop on this setting, so keep re-asserting until it
            // sticks — otherwise nobody runs ESS (Multi sits in watchdog passthru).
            Some(m) if !st.owner_us && m == HUB4MODE_EXTERNAL => {
                ext_mode_write = Some(cfg.orig_mode);
                ext_note = "re-asserting stock mode";
            }
            _ => {}
        }
    }

    let enter_ok = s.soc >= snap.min_soc + cfg.soc_hyst
        && s.state != cfg.state_recharge
        && !st.ceded_external;
    let exit_now = s.soc <= snap.min_soc || s.state == cfg.state_recharge || sanity_tripped;
    let want_us = if st.owner_us { !exit_now } else { enter_ok };
    let backoff_s = cfg.min_dwell_s * f64::from(1u32 << st.sanity_trips.min(7)); // 30s .. ~64min
    let dwell_ok = t - st.last_flip_t >= backoff_s;

    let mut note = ext_note;
    let mut hub4mode = ext_mode_write;
    let mut flipped = !ext_note.is_empty() && ext_mode_write.is_none();
    let mut transition_write = Write::Nothing;
    if want_us != st.owner_us && (dwell_ok || !want_us) {
        st.owner_us = want_us;
        st.last_flip_t = t;
        flipped = true;
        if !want_us && sanity_tripped {
            // Trip-forced hand-back: latch it (doubles the re-entry dwell, see above).
            st.sanity_trips = st.sanity_trips.saturating_add(1);
        }
        match cfg.stage {
            // Takeover flips Hub4Mode stock<->EXTERNAL so stock relinquishes / resumes
            // the whole loop.
            Stage::Takeover => {
                hub4mode = Some(if st.owner_us { HUB4MODE_EXTERNAL } else { cfg.orig_mode });
                note = if st.owner_us { "TOOK OVER (mode3)" } else { "handed back (mode1)" };
                if st.owner_us {
                    // Seed the write-EMA/slew from stock's LAST written setpoint so the
                    // first write converges from where the Multi already is, instead of
                    // jumping to the raw law (which can be a multi-kW transient if the
                    // take-over lands on a meter-lag tick).
                    st.last_out = snap.hub4_actual.filter(|v| v.is_finite());
                }
            }
            // Trim never touches Hub4Mode: seed the integral on activation; on deactivation
            // emit one neutralising override write (configured setpoint) so stock resumes.
            Stage::Trim => {
                if st.owner_us {
                    st.trim_override = Some(s.target);
                    note = "trim ON (override)";
                } else {
                    transition_write = Write::Override(s.target);
                    st.trim_override = None;
                    note = "trim OFF (stock)";
                }
            }
            Stage::Shadow => {
                note = if st.owner_us { "would take over" } else { "would hand back" };
            }
        }
        if !st.owner_us {
            st.ctrl.reset(); // avoid integral windup while the stock daemon drives
        }
    }

    // 2. Composed command — always computed (used for display, and to write in takeover).
    // The adaptive controller may only LEARN when its command actually drives the plant:
    // takeover stage, we own the loop, and the grid meter is usable/writable. In shadow
    // and trim our command is not actuated, so learning would train on a phantom regime.
    let actuating = st.owner_us && matches!(cfg.stage, Stage::Takeover) && snap.gm_should_write;

    // Smith-style meter-lag compensation: the grid reading is METER_LAG_S stale,
    // so during OUR OWN ramps the law chases a reading that predates the battery's
    // latest moves (the mechanism behind the deep step-off exports). We know our
    // write history and the identified plant, so the predictable part of that
    // staleness is corrected: grid_est = grid_meas + B*(w_eff_now - w_eff_lagged).
    // Only the battery-side term is corrected — load changes remain as measured.
    // Active only while actuating (shadow simulates it identically for telemetry).
    let meter_corr = if cfg.smith && (actuating || matches!(cfg.stage, Stage::Shadow) && st.owner_us) {
        if !st.smith_weff.is_finite() {
            // Seed the drive model from the measured Multi power.
            let w0 = (snap.s.reported - plant::A) / plant::B;
            st.smith_weff = w0;
            st.smith_weff_lag = w0;
            st.smith_prev_write = st.last_out.unwrap_or(w0);
        }
        let lagged = st.smith_weff_lag;
        st.smith_weff_lag = st.smith_weff; // METER_LAG_S == 1 tick history
        st.smith_weff += plant::ALPHA * (st.smith_prev_write - st.smith_weff);
        plant::B * (st.smith_weff - lagged)
    } else {
        st.smith_weff = f64::NAN;
        0.0
    };
    let cmd = composed_command(snap, &mut st.ctrl, actuating, meter_corr);

    // 3. Per-stage RAW write target (pre-slew).
    let mut display_cmd = cmd;
    let mut safety = Safety { dt_clamped: snap.dt_clamped, slew_clamped: false, sanity_tripped };
    let owner_str: &'static str = if !st.owner_us {
        "hub4"
    } else {
        match cfg.stage {
            Stage::Takeover => "US",
            Stage::Trim => "US-trim",
            Stage::Shadow => "US?",
        }
    };
    let write = if st.owner_us {
        let raw = match cfg.stage {
            Stage::Takeover => {
                if snap.gm_should_write { Write::Setpoint(cmd) } else { Write::Nothing }
            }
            Stage::Trim => {
                // Outer integral on /Overrides/Setpoint; stock owns everything else. Suspend
                // (hold neutral) while stock force-charges, an external discharge cap is
                // active (schedule post-target hold: stock deliberately lets grid diverge,
                // trimming would only wind up and bite when the cap clears), or the meter
                // is unusable.
                let o = st.trim_override.get_or_insert(s.target);
                if snap.sg.force_charge
                    || snap.sg.max_discharge_w.is_finite()
                    || !snap.gm_should_write
                {
                    *o = s.target;
                } else {
                    *o = crate::control::clamp(
                        *o + cfg.trim_ki * (s.target - s.grid) * snap.dt,
                        snap.lo,
                        snap.hi,
                    );
                }
                Write::Override(*o)
            }
            // Shadow never writes, but runs the FULL write path below (EMA + guards) so
            // its displayed/telemetered value is directly comparable to stock's writes.
            Stage::Shadow => Write::Setpoint(cmd),
        };
        // 4. The write law, mimicking stock exactly: EMA toward the command (see
        // `stock_write_ema`), then our W/s slew guard as a safety backstop (stock has only
        // a ±138 kW sanity clamp there — with the EMA damping, the guard binds only on
        // pathological steps), then the hard design envelope.
        match raw {
            Write::Setpoint(v) => {
                let ema = stock_write_ema(st.last_out, v, cfg.ema_adaptive, cfg.ema_gain_up, cfg.ema_gain_down);
                let sv = slew_limit(st.last_out, ema, cfg.slew_w_per_s, snap.dt);
                let sv = crate::control::clamp(sv, snap.lo, snap.hi);
                safety.slew_clamped = (sv - ema).abs() > 1e-6;
                st.last_out = Some(sv);
                st.smith_prev_write = sv; // enters the plant model next tick
                display_cmd = sv;
                if matches!(cfg.stage, Stage::Shadow) { Write::Nothing } else { Write::Setpoint(sv) }
            }
            Write::Override(v) => {
                // NO EMA here: the trim override is stock's INPUT target — its own write
                // EMA runs downstream of it. Smoothing both sides would double-filter.
                let sv = slew_limit(st.last_out, v, cfg.slew_w_per_s, snap.dt);
                safety.slew_clamped = (sv - v).abs() > 1e-6;
                st.last_out = Some(sv);
                display_cmd = sv;
                Write::Override(sv)
            }
            Write::Nothing => {
                // Keep the EMA/slew memory across a skipped write (meter-guard gap):
                // resetting here made every post-gap write a raw, unsmoothed step at
                // exactly the moment the law input is least trustworthy. Ownership
                // loss (the else branch below) still clears it.
                Write::Nothing
            }
        }
    } else {
        st.ctrl.reset();
        st.last_out = None; // start the slew fresh next time we drive
        // A trim-deactivation neutralise (from the ownership block) goes out directly (no slew,
        // so hand-back is clean).
        transition_write
    };

    // Feed-in + shore flags are written only in full takeover, while owning + meter usable.
    let feed_shore_write =
        st.owner_us && matches!(cfg.stage, Stage::Takeover) && snap.gm_should_write;

    // Remember force-charge / external discharge hold for the next tick's sanity
    // suppression (both make the grid legitimately diverge from the setpoint).
    st.prev_force_charge = snap.sg.force_charge || snap.sg.max_discharge_w.is_finite();

    Decision {
        command: cmd,
        write,
        hub4mode,
        feed_shore_write,
        owner_us: st.owner_us,
        display_cmd,
        owner_str,
        note,
        flipped,
        safety,
    }
}

/// Shared test scaffolding (helpers used by the inline unit tests AND the
/// cross-module golden tests in `src/tests/`). Compiled only under cfg(test).
#[cfg(test)]
pub mod testutil {
    use super::*;
    use crate::control::Kind;
    use crate::states::sysstate;

    pub fn sg_free() -> SocGuardOut {
        SocGuardOut {
            force_charge: false,
            max_discharge_w: f64::INFINITY,
            cmd_override: None,
            tpimf: false,
            charge_floor_w: 0.0,
        }
    }

    pub fn snap(stage_gm: bool) -> Snapshot {
        Snapshot {
            s: Sensors { grid: 100.0, reported: 0.0, target: 20.0, soc: 60.0, state: sysstate::DISCHARGING },
            dt: 1.0,
            dt_clamped: false,
            min_soc: 15.0,
            lo: -HARD_CLAMP_W,
            hi: HARD_CLAMP_W,
            effective_target: 20.0,
            sg: sg_free(),
            feed_block_charge: false,
            feed_force_flat: false,
            gen_disable_export: false,
            gm_should_write: stage_gm,
            shore_out: ShoreOut::default(),
            hub4_actual: None,
                ac_out_w: f64::NAN,
                hub4mode_live: None,
        }
    }

    pub fn state(stage_kind: Kind) -> LoopState {
        LoopState {
            ctrl: Controller::new(stage_kind),
            owner_us: false,
            last_flip_t: -1000.0,
            trim_override: None,
            last_out: None,
            oob_since: None,
            prev_force_charge: false,
            smith_weff: f64::NAN,
            smith_weff_lag: f64::NAN,
            smith_prev_write: f64::NAN,
            ceded_external: false,
            sanity_trips: 0,
        }
    }

    pub fn cfg(stage: Stage) -> DecideCfg {
        // Safety params derived from the design anchor, exactly as the runtime does.
        let sl = SafetyLimits::derive(HARD_CLAMP_W, 1.0, None, None);
        DecideCfg {
            stage,
            soc_hyst: 3.0,
            min_dwell_s: 30.0,
            trim_ki: 0.02,
            orig_mode: 1,
            state_recharge: sysstate::RECHARGE,
            slew_w_per_s: sl.slew_w_per_s,
            sanity_band_w: sl.sanity_band_w,
            sanity_secs: sl.sanity_secs,
            ema_adaptive: false,
            // Goldens were captured under symmetric 0.5 pacing; pin it here so the
            // tick-exact replays stay exact. Production defaults are asymmetric.
            ema_gain_up: 0.5,
            ema_gain_down: 0.5,
            // Captures predate the Smith compensation - replay without it.
            smith: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::*;
    use super::*;
    use crate::control::Kind;
    use crate::states::sysstate;





    #[test]
    fn takeover_takes_over_above_floor_and_writes_setpoint() {
        let mut st = state(Kind::Stock);
        let d = decide(&snap(true), &mut st, &cfg(Stage::Takeover), 0.0);
        assert!(d.owner_us && d.flipped);
        assert_eq!(d.hub4mode, Some(HUB4MODE_EXTERNAL));
        assert!(matches!(d.write, Write::Setpoint(_)));
    }

    #[test]
    fn takeover_holds_write_when_meter_unusable() {
        let mut st = state(Kind::Stock);
        st.owner_us = true; // already owning, no flip
        let d = decide(&snap(false), &mut st, &cfg(Stage::Takeover), 100.0);
        assert!(d.owner_us);
        assert_eq!(d.write, Write::Nothing, "no setpoint write when meter unusable");
    }

    #[test]
    fn below_floor_hands_back_immediately() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let mut sp = snap(true);
        sp.s.soc = 10.0; // <= min_soc
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 1.0); // dwell not elapsed, but handback is never throttled
        assert!(!d.owner_us && d.flipped);
        assert_eq!(d.hub4mode, Some(1));
    }

    #[test]
    fn takeover_entry_is_dwell_throttled() {
        let mut st = state(Kind::Stock);
        st.last_flip_t = 0.0; // just flipped
        let d = decide(&snap(true), &mut st, &cfg(Stage::Takeover), 5.0); // 5s < 30s dwell
        assert!(!d.owner_us, "take-over throttled until dwell elapses");
    }

    #[test]
    fn trim_integrates_toward_setpoint() {
        let mut st = state(Kind::Stock);
        let mut sp = snap(true);
        sp.s.grid = 500.0; // grid above the 20 W target => positive error pushes override up
        sp.s.soc = 60.0;
        let c = cfg(Stage::Trim);
        // first call takes over + seeds override at target
        let d1 = decide(&sp, &mut st, &c, 0.0);
        assert!(d1.owner_us);
        // subsequent calls integrate: override moves away from the seed toward compensating.
        let d2 = decide(&sp, &mut st, &c, 1.0);
        let d3 = decide(&sp, &mut st, &c, 2.0);
        match (d2.write, d3.write) {
            (Write::Override(o2), Write::Override(o3)) => {
                assert!(o2 < sp.s.target && o3 < o2, "integral drives override down: {o2} then {o3}");
            }
            _ => panic!("trim must write override"),
        }
    }

    #[test]
    fn trim_suspends_during_force_charge() {
        let mut st = state(Kind::Stock);
        let mut sp = snap(true);
        sp.sg.force_charge = true;
        sp.s.grid = 500.0;
        let c = cfg(Stage::Trim);
        decide(&sp, &mut st, &c, 0.0); // take over
        let d = decide(&sp, &mut st, &c, 1.0);
        assert_eq!(d.write, Write::Override(sp.s.target), "hold neutral, don't fight stock");
    }


    /// The window-tail hold (live 2026-08-16): ForceCharge already 0, but
    /// `/Overrides/MaxDischargePower` = 1 W pins the battery while a 1.9 kW load runs.
    /// Trim must hold neutral — integrating against a deliberately-held battery only
    /// winds the override up and bites when the hold clears at window end.
    #[test]
    fn trim_suspends_during_window_tail_hold() {
        let mut st = state(Kind::Stock);
        let mut sp = snap(true);
        sp.sg.max_discharge_w = 1.0; // external hold, NOT force-charge
        sp.s.grid = 1908.9;
        let c = cfg(Stage::Trim);
        decide(&sp, &mut st, &c, 0.0); // take over
        let d = decide(&sp, &mut st, &c, 1.0);
        assert_eq!(d.write, Write::Override(sp.s.target), "hold neutral during the tail hold");
    }

    #[test]
    fn sanity_is_suppressed_during_window_tail_hold() {
        // Same scenario in takeover: grid legitimately 1.9 kW off-target while the hold
        // is active — must not accumulate toward a sanity trip / spurious hand-back.
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        st.prev_force_charge = true; // set by the previous tick's hold (see decide())
        let mut sp = snap(true);
        sp.sg.max_discharge_w = 1.0;
        sp.s.grid = 1908.9;
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
        assert!(st.oob_since.is_none(), "hold must suppress the out-of-band timer");
        assert!(d.owner_us, "no hand-back during the tail hold");
        // And the flag re-arms for the next tick from the cap itself.
        assert!(st.prev_force_charge);
    }

    #[test]
    fn shadow_never_writes() {
        let mut st = state(Kind::Stock);
        let d = decide(&snap(true), &mut st, &cfg(Stage::Shadow), 0.0);
        assert_eq!(d.write, Write::Nothing);
        assert_eq!(d.hub4mode, None);
    }

    #[test]
    fn force_charge_command_overrides_discharge() {
        // SocGuard force-charge with a finite ceiling: composed command must be the charge cmd.
        let mut sp = snap(true);
        sp.sg = SocGuardOut {
            force_charge: true,
            max_discharge_w: 0.0,
            cmd_override: Some(4700.0),
            tpimf: false,
            charge_floor_w: 0.0,
        };
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        assert_eq!(cmd, 4700.0);
    }

    /// Live capture 2026-08-16 ~04:15, scheduled-charge window MID (SystemState 259,
    /// `/Overrides/ForceCharge`=1, TPIMF firmware): stock the stock daemon keeps writing the
    /// raw normal-law value — observed `AcPowerSetpoint` = −230 W at grid 2 771.7 W /
    /// reported +2 513 W / target 5 W (the firmware charges maximally on its own; the
    /// setpoint only caps feed-in). Our first implementation clamped this to ≥ 0 and
    /// deviated by ~house-loads (~+200 W median) for the whole 6 h window.
    #[test]
    fn golden_tpimf_force_charge_passes_normal_law_through() {
        let mut sp = snap(true);
        sp.s = Sensors { grid: 2771.7, reported: 2513.0, target: 5.0, soc: 73.0, state: sysstate::SCHEDULED_CHARGE };
        sp.effective_target = 5.0;
        sp.sg = SocGuardOut {
            force_charge: true,
            max_discharge_w: f64::INFINITY, // schedule sets NO discharge cap while charging
            cmd_override: Some(crate::socguard::FORCE_CHARGE_SENTINEL_W),
            tpimf: true,
            charge_floor_w: 0.0,
        };
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        // Normal law: reported + (target − grid) = 2513 + 5 − 2771.7 = −253.7 W.
        // the stock daemon wrote −230 W on this row (Δ = meter/tick noise).
        assert!((cmd - (-253.7)).abs() < 1.0, "normal law must pass through, got {cmd}");
        assert!((cmd - (-230.0)).abs() < 100.0, "must track the stock daemon's live write, got {cmd}");
    }

    /// Live capture 2026-08-16 ~04:30, scheduled-charge window TAIL: target SOC (90 %)
    /// reached with the window still open. systemcalc drops `/Overrides/ForceCharge` to 0
    /// and sets `/Overrides/MaxDischargePower` = max(1, 0.8·pv) = 1 W (no PV at night) —
    /// "hold the charge". Stock held ~+48 W against a 1.9 kW morning load while the raw
    /// normal law wanted −1 868 W. Without honoring the override we would have discharged
    /// the freshly charged battery into the house for the remaining ~37 min of the window.
    #[test]
    fn golden_window_tail_discharge_hold_is_honored() {
        let mut sp = snap(true);
        sp.s = Sensors { grid: 1908.9, reported: 35.0, target: 5.0, soc: 90.0, state: sysstate::SCHEDULED_CHARGE };
        sp.effective_target = 5.0;
        sp.sg = sg_free();
        sp.sg.max_discharge_w = 1.0; // enact() of /Overrides/MaxDischargePower = 1
        sp.ac_out_w = 49.0; // measured Multi AC-out that morning
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        // Stock's hold write = clamp(−1 W) + acOut = +48 W — stock-behavior analysis-verified frame
        // (see composed_command) and the EXACT value the stock daemon wrote on this live row.
        assert!((cmd - 48.0).abs() < 1e-9, "hold must float at −cap + acOut, got {cmd}");

        // acOut unread (NaN): degrade to the bare cap — still no discharge.
        sp.ac_out_w = f64::NAN;
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        assert!((cmd - (-1.0)).abs() < 1e-9, "NaN acOut degrades to the bare cap, got {cmd}");

        // Same row with the override cleared (window exit, or an AllowDischarge schedule
        // >1 % over target): normal discharge resumes; acOut must NOT leak into the law.
        sp.sg = sg_free();
        sp.ac_out_w = 49.0;
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        assert!(cmd < -1500.0, "cleared override must discharge normally, got {cmd}");
    }

    /// The AC-out feedforward composes with every battery-frame floor, exactly as stock's
    /// single clamp does: BL-state discharge inhibit (floor 0 → write = acOut) and the
    /// state-6 minimum-charge floor (write = dcV·5 + acOut).
    #[test]
    fn ac_out_feedforward_lifts_state_floors() {
        let mut sp = snap(true);
        sp.s.grid = 1500.0; // big load: raw law is strongly negative
        sp.sg = sg_free();
        sp.sg.max_discharge_w = 0.0; // BL no-discharge state
        sp.ac_out_w = 15.0;
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        assert!((cmd - 15.0).abs() < 1e-9, "inhibit floor lifts to acOut, got {cmd}");

        sp.sg.charge_floor_w = 260.0; // state 6: dcV·5A
        let mut ctrl = Controller::new(Kind::Stock);
        let cmd = composed_command(&sp, &mut ctrl, false, 0.0);
        assert!((cmd - 275.0).abs() < 1e-9, "charge floor lifts to dcV·5 + acOut, got {cmd}");
    }





    /// Persistently NON-FINITE grid while driving counts as out-of-band: we cannot verify
    /// sanity, so after sanity_secs we must trip and hand back (a NaN must never silently
    /// reset the out-of-band timer).
    #[test]
    fn sanity_nan_grid_counts_out_of_band() {
        let mut st = state(Kind::Pi { ki: 0.05, i_max: 300.0 });
        st.owner_us = true;
        let c = cfg(Stage::Takeover);
        let mut trip_t = None;
        for i in 0..100 {
            let mut sp = snap(true);
            sp.s.grid = f64::NAN;
            let d = decide(&sp, &mut st, &c, i as f64);
            if d.safety.sanity_tripped {
                trip_t = Some(i);
                // At the trip tick the supervisor must have forced the hand-back.
                assert!(!st.owner_us, "trip must hand back immediately");
                assert_eq!(st.sanity_trips, 1);
                break;
            }
        }
        let trip_t = trip_t.expect("persistent NaN grid must trip the sanity supervisor");
        assert!(
            (trip_t as f64) >= c.sanity_secs,
            "trip only after the sanity dwell, got t={trip_t}"
        );
    }

    /// Sanity-trip latch: each trip doubles the re-entry dwell, so a persistently-wrong
    /// controller escalates instead of flip-flopping mode 3<->1 every min_dwell.
    #[test]
    fn sanity_trip_backoff_doubles_reentry_dwell() {
        let mut st = state(Kind::Pi { ki: 0.05, i_max: 300.0 });
        st.owner_us = true;
        let c = cfg(Stage::Takeover);
        let mut t = 0.0;
        // Grossly out-of-band until the supervisor trips and hands back.
        while st.owner_us {
            let mut sp = snap(true);
            sp.s.grid = sp.s.target + 5000.0;
            decide(&sp, &mut st, &c, t);
            t += 1.0;
            assert!(t < 100.0, "should have tripped within sanity_secs");
        }
        assert_eq!(st.sanity_trips, 1);
        let handback_t = t;
        // Healthy again: re-entry must now wait 2x min_dwell, not 1x.
        while !st.owner_us {
            let mut sp = snap(true);
            sp.s.grid = sp.s.target; // in band
            decide(&sp, &mut st, &c, t);
            if st.owner_us {
                break;
            }
            t += 1.0;
            assert!(t < handback_t + 500.0, "never re-entered");
        }
        let waited = t - handback_t;
        assert!(
            waited >= 2.0 * c.min_dwell_s - 1.0,
            "after one trip re-entry must wait 2x dwell ({}s), waited {waited}s",
            2.0 * c.min_dwell_s
        );
    }

    // ---- write-phase safety features (each is observable via `Decision.safety`) ----

    #[test]
    fn slew_guard_limits_a_huge_jump_and_flags_it() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        st.last_out = Some(0.0);
        let mut c = cfg(Stage::Takeover);
        c.slew_w_per_s = 100.0; // tight for the test
        let mut sp = snap(true);
        // Discharge direction: a positive command outside charge regimes is now
        // structurally clamped (stock parity), so the guard is exercised downward.
        sp.s.grid = 5020.0; // law => ~ -5000 in one step
        sp.s.reported = 0.0;
        let d = decide(&sp, &mut st, &c, 100.0);
        match d.write {
            Write::Setpoint(v) => assert!((v + 100.0).abs() < 1e-6, "slewed 0->-100, got {v}"),
            other => panic!("expected slewed setpoint, got {other:?}"),
        }
        assert!(d.safety.slew_clamped);
        assert_eq!(d.safety.alarm_level(), 1);
        assert_eq!(d.safety.reason(), "slew-clamped");
    }

    /// Smith compensation: during our OWN discharge ramp the stale meter makes the
    /// raw law over-discharge; the corrected law must command LESS discharge than
    /// the uncompensated one (correction = B*(w_eff step), battery-side only).
    #[test]
    fn smith_compensation_softens_own_ramp_chasing() {
        let mk = |smith: bool| {
            let mut c = cfg(Stage::Takeover);
            c.smith = smith;
            c
        };
        let run = |c: &DecideCfg| {
            let mut st = state(Kind::Stock);
            st.owner_us = true;
            let mut sp = snap(true);
            sp.s.grid = 2020.0; // big load lands: law ~ -2000
            let d1 = decide(&sp, &mut st, c, 0.0);
            let d2 = decide(&sp, &mut st, c, 1.0); // meter still stale at 2020
            (d1.command, d2.command)
        };
        let (a1, a2) = run(&mk(true));
        let (b1, b2) = run(&mk(false));
        assert_eq!(a1, b1, "first tick: model just seeded, no correction yet");
        assert!(
            a2 > b2 + 50.0,
            "compensated law must chase the stale reading less: {a2} vs {b2}"
        );
    }

    #[test]
    fn smith_state_resets_when_not_actuating() {
        let mut c = cfg(Stage::Takeover);
        c.smith = true;
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let mut sp = snap(true);
        sp.s.grid = 1500.0;
        decide(&sp, &mut st, &c, 0.0);
        assert!(st.smith_weff.is_finite(), "seeded while actuating");
        sp.gm_should_write = false; // meter gap: not actuating
        decide(&sp, &mut st, &c, 1.0);
        assert!(st.smith_weff.is_nan(), "must reset across non-actuating ticks");
    }

    #[test]
    fn stock_write_ema_gain_schedule() {
        // First write: passthrough (rounded).
        assert_eq!(stock_write_ema(None, -286.9, false, 0.5, 0.5), -287.0);
        // Flat mode: 0.5 regardless of step size.
        assert_eq!(stock_write_ema(Some(0.0), 5000.0, false, 0.5, 0.5), 2500.0);
        // Small step (< 10 W): 0.5 even in adaptive mode.
        assert_eq!(stock_write_ema(Some(0.0), 8.0, true, 0.5, 0.5), 4.0);
        // Adaptive: Δ=100 → gain 0.25·ln(10) = 0.5756…
        let v = stock_write_ema(Some(0.0), 100.0, true, 0.5, 0.5);
        assert_eq!(v, (0.25f64 * 10.0f64.ln() * 100.0).round());
        // Adaptive: Δ=50 → 0.25·ln(5) = 0.402 → clamped UP to 0.5.
        assert_eq!(stock_write_ema(Some(0.0), 50.0, true, 0.5, 0.5), 25.0);
        // Adaptive: Δ=1000 → 0.25·ln(100) = 1.151 → clamped to 1.1 (overshoot!).
        assert_eq!(stock_write_ema(Some(0.0), 1000.0, true, 0.5, 0.5), 1100.0);
        // Fixed point: integer target reached and held exactly.
        assert_eq!(stock_write_ema(Some(30.0), 30.0, false, 0.5, 0.5), 30.0);
    }

    /// The live hold-entry convergence (2026-08-17, stock wrote −160 → −80 → −38 → +30 at
    /// its 2.5 s cadence): flat-0.5 EMA from the same start toward the same target must
    /// converge monotonically with no overshoot and settle ON the integer target.
    #[test]
    fn stock_write_ema_converges_monotone_flat() {
        let mut v = -160.0;
        let mut prev = v;
        for _ in 0..12 {
            v = stock_write_ema(Some(v), 30.0, false, 0.5, 0.5);
            assert!(v >= prev && v <= 30.0, "monotone, no overshoot: {prev} -> {v}");
            prev = v;
        }
        assert_eq!(v, 30.0, "must settle exactly on the target");
    }

    /// EMA through decide(): first write after take-over is direct; subsequent writes
    /// halve toward the command and are integer watts.
    #[test]
    fn takeover_write_is_ema_smoothed_and_integer() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let c = cfg(Stage::Takeover);
        let mut sp = snap(true);
        sp.s.grid = 100.0; // law: 0 + 20 − 100 = −80
        let d1 = decide(&sp, &mut st, &c, 0.0);
        let Write::Setpoint(v1) = d1.write else { panic!("expected setpoint") };
        assert_eq!(v1, -80.0, "first write is direct (stock passthrough)");
        sp.s.grid = 900.0; // law: −880
        let d2 = decide(&sp, &mut st, &c, 1.0);
        let Write::Setpoint(v2) = d2.write else { panic!("expected setpoint") };
        assert_eq!(v2, -480.0, "second write halves toward the command");
        assert_eq!(v2.fract(), 0.0, "writes are integer watts");
    }

    /// Shadow simulates the identical write path (for stock-comparable telemetry) but
    /// still never writes.
    #[test]
    fn shadow_simulates_write_path_without_writing() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let c = cfg(Stage::Shadow);
        let mut sp = snap(true);
        sp.s.grid = 100.0;
        let d1 = decide(&sp, &mut st, &c, 0.0);
        assert_eq!(d1.write, Write::Nothing);
        assert_eq!(d1.display_cmd, -80.0);
        sp.s.grid = 900.0;
        let d2 = decide(&sp, &mut st, &c, 1.0);
        assert_eq!(d2.write, Write::Nothing, "shadow must never write");
        assert_eq!(d2.hub4mode, None);
        assert_eq!(d2.display_cmd, -480.0, "displayed value follows the real write law");
    }

    #[test]
    fn external_mode_flip_cedes_ownership_immediately() {
        // User/VRM/another tool flips Hub4Mode away while we own: stock is running
        // again — cede this tick, write no mode (never fight the external change).
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        st.last_flip_t = -1000.0;
        let mut sp = snap(true);
        sp.hub4mode_live = Some(1);
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
        assert!(!d.owner_us && d.flipped, "must cede when the mode is taken from us");
        assert_eq!(d.hub4mode, None, "must not write the mode back");
        assert_eq!(d.write, Write::Nothing);
        // Sticky: hours later, with take-over conditions perfect, we still defer to
        // the operator's choice — only a process restart re-arms the stage.
        sp.hub4mode_live = Some(1);
        let d2 = decide(&sp, &mut st, &cfg(Stage::Takeover), 50_000.0);
        assert!(!d2.owner_us && d2.hub4mode.is_none(), "cede must be sticky");
    }

    #[test]
    fn failed_handback_mode_is_reasserted_until_it_sticks() {
        // Handed back (e.g. after a crash-restart) but the persistent setting still
        // reads EXTERNAL: stock gates its whole loop on it, so keep restoring.
        let mut st = state(Kind::Stock);
        st.owner_us = false;
        let mut sp = snap(true);
        sp.s.soc = 10.0; // below floor: no take-over wanted
        sp.hub4mode_live = Some(HUB4MODE_EXTERNAL);
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
        assert_eq!(d.hub4mode, Some(1), "must re-assert the stock mode");
        assert!(!d.owner_us);
    }

    #[test]
    fn takeover_first_write_is_seeded_from_stock_setpoint() {
        // Take-over landing on a meter-lag tick must not write the raw transient:
        // the EMA seeds from stock's last written setpoint.
        let mut st = state(Kind::Stock);
        let mut sp = snap(true);
        sp.s.grid = 100.0; // law: 0 + 20 − 100 = −80
        sp.hub4_actual = Some(-500.0); // stock's last write
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 0.0);
        assert!(d.owner_us && d.flipped);
        let Write::Setpoint(v) = d.write else { panic!("expected setpoint") };
        assert_eq!(v, -290.0, "EMA from stock's −500 toward law −80, got {v}");
    }

    #[test]
    fn write_gap_preserves_ema_memory() {
        // A meter-guard write gap must not reset the EMA: the post-gap write
        // continues from the last actuated value instead of stepping raw.
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let c = cfg(Stage::Takeover);
        let mut sp = snap(true);
        sp.s.grid = 100.0; // law −80
        let d1 = decide(&sp, &mut st, &c, 0.0);
        let Write::Setpoint(v1) = d1.write else { panic!() };
        assert_eq!(v1, -80.0);
        sp.gm_should_write = false; // guard gap
        let d2 = decide(&sp, &mut st, &c, 1.0);
        assert_eq!(d2.write, Write::Nothing);
        sp.gm_should_write = true;
        sp.s.grid = 900.0; // law −880
        let d3 = decide(&sp, &mut st, &c, 2.0);
        let Write::Setpoint(v3) = d3.write else { panic!() };
        assert_eq!(v3, -480.0, "post-gap write must be EMA'd from −80, got {v3}");
    }

    #[test]
    fn sanity_supervisor_hands_back_after_sustained_divergence() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let c = cfg(Stage::Takeover);
        let mut sp = snap(true);
        sp.s.grid = 3000.0; // 3000 vs setpoint 20 => far outside the 1000 W band

        assert!(decide(&sp, &mut st, &c, 0.0).owner_us, "not tripped instantly");
        assert!(decide(&sp, &mut st, &c, 10.0).owner_us, "still within the dwell");
        let d = decide(&sp, &mut st, &c, 31.0); // past the 30 s sanity dwell
        assert!(!d.owner_us && d.flipped, "sanity forces hand-back");
        assert!(d.safety.sanity_tripped);
        assert_eq!(d.safety.alarm_level(), 2);
        assert_eq!(d.hub4mode, Some(1), "handed the setpoint loop back to stock");
    }

    #[test]
    fn sanity_is_suppressed_during_force_charge() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        st.prev_force_charge = true; // stock force-charging last tick => grid legitimately diverges
        let c = cfg(Stage::Takeover);
        let mut sp = snap(true);
        sp.s.grid = 3000.0;
        let d = decide(&sp, &mut st, &c, 31.0);
        assert!(d.owner_us, "must not trip while force-charging");
        assert!(!d.safety.sanity_tripped);
    }

    #[test]
    fn safety_params_derive_from_the_design_anchor() {
        // No overrides: slew = full range / 2 s, sanity band = 15 % of design, dt-clamp floor.
        let sl = SafetyLimits::derive(HARD_CLAMP_W, 1.0, None, None);
        assert!((sl.slew_w_per_s - HARD_CLAMP_W / SLEW_TRAVERSE_S).abs() < 1e-6);
        assert!((sl.sanity_band_w - HARD_CLAMP_W * SANITY_FRACTION).abs() < 1e-6);
        assert_eq!(sl.max_dt_s, MIN_DT_CLAMP_S); // max(5*1, 5)
        // A larger interval widens the dt-clamp; overrides win over the derived value.
        let sl2 = SafetyLimits::derive(4000.0, 3.0, Some(999.0), Some(500.0));
        assert_eq!(sl2.slew_w_per_s, 999.0);
        assert_eq!(sl2.sanity_band_w, 500.0);
        assert_eq!(sl2.max_dt_s, 15.0); // 5 * 3 s
    }

    #[test]
    fn dt_clamp_flag_propagates_to_safety() {
        let mut st = state(Kind::Stock);
        st.owner_us = true;
        let mut sp = snap(true);
        sp.dt_clamped = true;
        let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
        assert!(d.safety.dt_clamped && d.safety.any());
    }

    // Full-loop replay (§3.2): drive the WHOLE composed decision over the real captured trace
    // (both discharge 256 + scheduled-charge 259 regimes). Crucially the per-row command bounds
    // and force-charge decision come from the PRODUCTION code (battery_limits::BatteryBroker +
    // socguard::SocGuard fed by the row's own dbus values) — not hand-rolled constants — so this
    // is a genuine replay: every invariant is checked against the limits the real system derived.
}
