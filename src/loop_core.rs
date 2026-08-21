//! Pure decision core — the `decide()` phase of the sample/decide/actuate split
//! (the rollout/failsafe design notes §3.1).
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
pub fn stock_write_ema(prev: Option<f64>, cmd: f64, adaptive: bool) -> f64 {
    let v = match prev {
        Some(p) if p.is_finite() => {
            let delta = cmd - p;
            let gain = if !adaptive || delta.abs() < 10.0 {
                0.5
            } else {
                (0.25 * (delta.abs() / 10.0).ln()).clamp(0.5, 1.1)
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
    /// The stock ESS loop's current setpoint, for the shadow-fidelity delta (read-only).
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
    /// with a fast grid meter (< 250 ms) + recent Multi firmware; false = flat 0.5 (this
    /// install's live-observed behavior).
    pub ema_adaptive: bool,
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
pub fn composed_command(snap: &Snapshot, ctrl: &mut Controller, actuating: bool) -> f64 {
    let s = &snap.s;
    let mut cmd = ctrl.update(
        s.grid,
        s.reported,
        snap.effective_target,
        snap.dt,
        snap.lo,
        snap.hi,
        actuating,
    );
    // SocGuard discharge inhibit / external cap, MIMICKING THE STOCK DAEMON EXACTLY
    // (live-verified 2026-08-17): the stock loop builds its command in the acIn−acOut
    // frame — total = clamp(target + acIn − grid − acOut, −maxdis, maxchg) — and the
    // per-phase write adds measured `Ac/Out/L{n}/P` back on top. In normal operation
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
        // setpoint as a feed-in cap, so the value may legitimately go negative (live-
        // verified 2026-08-16 across a full 6 h window: the stock daemon writes the raw
        // normal law, median |Δ| 2.5 W). Legacy (mode 1): the setpoint IS the charge command.
        cmd = if snap.sg.tpimf { cmd.min(fc) } else { fc };
    }
    if snap.sg.charge_floor_w > 0.0 {
        // minimum-charge floor ONLY when set (BL state 6); 0 must not zero discharge.
        // Same frame as the discharge cap: the stock dcV·5 floor sits in the acIn−acOut
        // total, so the write floor is dcV·5 + acOut (charge 5 A AND serve the output).
        cmd = cmd.max(snap.sg.charge_floor_w + ac_out);
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
            st.ctrl.reset(); // avoid integral windup while the stock ESS loop drives
        }
    }

    // 2. Composed command — always computed (used for display, and to write in takeover).
    // The adaptive controller may only LEARN when its command actually drives the plant:
    // takeover stage, we own the loop, and the grid meter is usable/writable. In shadow
    // and trim our command is not actuated, so learning would train on a phantom regime.
    let actuating = st.owner_us && matches!(cfg.stage, Stage::Takeover) && snap.gm_should_write;
    let cmd = composed_command(snap, &mut st.ctrl, actuating);

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
                let ema = stock_write_ema(st.last_out, v, cfg.ema_adaptive);
                let sv = slew_limit(st.last_out, ema, cfg.slew_w_per_s, snap.dt);
                let sv = crate::control::clamp(sv, snap.lo, snap.hi);
                safety.slew_clamped = (sv - ema).abs() > 1e-6;
                st.last_out = Some(sv);
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

#[cfg(test)]
#[path = "loop_core_tests.rs"]
mod tests;
