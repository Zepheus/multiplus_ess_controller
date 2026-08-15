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
    /// Previous tick's force-charge flag — the sanity check is suppressed during force-charge
    /// (grid legitimately diverges then); using last tick avoids an ordering dependency.
    pub prev_force_charge: bool,
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
pub fn composed_command(snap: &Snapshot, ctrl: &mut Controller) -> f64 {
    let s = &snap.s;
    let mut cmd = ctrl.update(s.grid, s.reported, snap.effective_target, snap.dt, snap.lo, snap.hi);
    // SocGuard: discharge inhibit (INF => no-op; 0 => no discharge).
    cmd = cmd.max(-snap.sg.max_discharge_w);
    if let Some(fc) = snap.sg.cmd_override {
        // force charge
        cmd = if snap.sg.tpimf { cmd.max(0.0).min(fc) } else { fc };
    }
    if snap.sg.charge_floor_w > 0.0 {
        // minimum-charge floor ONLY when set (BL state 6); 0 must not zero discharge.
        cmd = cmd.max(snap.sg.charge_floor_w);
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
    let oob = st.owner_us
        && cfg.stage.writes()
        && snap.gm_should_write
        && !st.prev_force_charge
        && (s.grid - s.target).abs() > cfg.sanity_band_w;
    st.oob_since = if oob { Some(st.oob_since.unwrap_or(t)) } else { None };
    let sanity_tripped = st.oob_since.is_some_and(|t0| t - t0 >= cfg.sanity_secs);

    // 1. Ownership envelope — may own iff SOC is comfortably above the floor and we're not in
    //    a recharge state. Hysteresis + dwell prevent thrash; hand-BACK is never throttled.
    //    A sanity trip forces exit (hand back / neutralise) regardless of dwell.
    let enter_ok = s.soc >= snap.min_soc + cfg.soc_hyst && s.state != cfg.state_recharge;
    let exit_now = s.soc <= snap.min_soc || s.state == cfg.state_recharge || sanity_tripped;
    let want_us = if st.owner_us { !exit_now } else { enter_ok };
    let dwell_ok = t - st.last_flip_t >= cfg.min_dwell_s;

    let mut note = "";
    let mut hub4mode = None;
    let mut flipped = false;
    let mut transition_write = Write::Nothing;
    if want_us != st.owner_us && (dwell_ok || !want_us) {
        st.owner_us = want_us;
        st.last_flip_t = t;
        flipped = true;
        match cfg.stage {
            // Takeover flips Hub4Mode 1<->3 so stock relinquishes / resumes the whole loop.
            Stage::Takeover => {
                hub4mode = Some(if st.owner_us { 3 } else { cfg.orig_mode });
                note = if st.owner_us { "TOOK OVER (mode3)" } else { "handed back (mode1)" };
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
    let cmd = composed_command(snap, &mut st.ctrl);

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
                // (hold neutral) while stock force-charges or the meter is unusable.
                let o = st.trim_override.get_or_insert(s.target);
                if snap.sg.force_charge || !snap.gm_should_write {
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
            // Shadow never writes even when it "owns" hypothetically.
            Stage::Shadow => Write::Nothing,
        };
        // 4. Glitch-guard slew on the live actuated value (records a trip in `safety`).
        match raw {
            Write::Setpoint(v) => {
                let sv = slew_limit(st.last_out, v, cfg.slew_w_per_s, snap.dt);
                safety.slew_clamped = (sv - v).abs() > 1e-6;
                st.last_out = Some(sv);
                display_cmd = sv;
                Write::Setpoint(sv)
            }
            Write::Override(v) => {
                let sv = slew_limit(st.last_out, v, cfg.slew_w_per_s, snap.dt);
                safety.slew_clamped = (sv - v).abs() > 1e-6;
                st.last_out = Some(sv);
                display_cmd = sv;
                Write::Override(sv)
            }
            Write::Nothing => {
                st.last_out = None;
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

    // Remember force-charge for the next tick's sanity suppression.
    st.prev_force_charge = snap.sg.force_charge;

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
mod tests {
    use super::*;
    use crate::control::Kind;

    fn sg_free() -> SocGuardOut {
        SocGuardOut {
            force_charge: false,
            max_discharge_w: f64::INFINITY,
            cmd_override: None,
            tpimf: false,
            charge_floor_w: 0.0,
        }
    }

    fn snap(stage_gm: bool) -> Snapshot {
        Snapshot {
            s: Sensors { grid: 100.0, reported: 0.0, target: 20.0, soc: 60.0, state: 256 },
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
        }
    }

    fn state(stage_kind: Kind) -> LoopState {
        LoopState {
            ctrl: Controller::new(stage_kind),
            owner_us: false,
            last_flip_t: -1000.0,
            trim_override: None,
            last_out: None,
            oob_since: None,
            prev_force_charge: false,
        }
    }

    fn cfg(stage: Stage) -> DecideCfg {
        // Safety params derived from the design anchor, exactly as the runtime does.
        let sl = SafetyLimits::derive(HARD_CLAMP_W, 1.0, None, None);
        DecideCfg {
            stage,
            soc_hyst: 3.0,
            min_dwell_s: 30.0,
            trim_ki: 0.02,
            orig_mode: 1,
            state_recharge: 258,
            slew_w_per_s: sl.slew_w_per_s,
            sanity_band_w: sl.sanity_band_w,
            sanity_secs: sl.sanity_secs,
        }
    }

    #[test]
    fn takeover_takes_over_above_floor_and_writes_setpoint() {
        let mut st = state(Kind::Stock);
        let d = decide(&snap(true), &mut st, &cfg(Stage::Takeover), 0.0);
        assert!(d.owner_us && d.flipped);
        assert_eq!(d.hub4mode, Some(3));
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
        let cmd = composed_command(&sp, &mut ctrl);
        assert_eq!(cmd, 4700.0);
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
        sp.effective_target = 5000.0; // Stock law => command ~ +5000 in one step
        sp.s.grid = 0.0;
        sp.s.reported = 0.0;
        let d = decide(&sp, &mut st, &c, 100.0);
        match d.write {
            Write::Setpoint(v) => assert!((v - 100.0).abs() < 1e-6, "slewed 0->100, got {v}"),
            other => panic!("expected slewed setpoint, got {other:?}"),
        }
        assert!(d.safety.slew_clamped);
        assert_eq!(d.safety.alarm_level(), 1);
        assert_eq!(d.safety.reason(), "slew-clamped");
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
    #[test]
    fn golden_full_loop_replay_invariants() {
        use crate::dbus::*;
        use crate::testbus::MockBus;
        use crate::{battery_limits, socguard};

        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/golden-sample.csv");
        let data = match std::fs::read_to_string(path) {
            Ok(d) => d,
            Err(_) => return, // fixture optional
        };
        let mut lines = data.lines();
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        let ix = |n: &str| header.iter().position(|h| *h == n).unwrap_or_else(|| panic!("col {n}"));
        let col = [
            "grid", "soc", "state", "dvcc", "acin_p", "vdc", "fw", "multi_maxchg_i", "bms_ccl",
            "bms_dcl", "bms_chgreq", "set_setpoint", "bl_state", "bl_minsoc", "h4_forcecharge",
        ]
        .map(&ix);
        let [c_grid, c_soc, c_state, c_dvcc, c_acinp, c_vdc, c_fw, c_mmci, c_ccl, c_dcl, c_chgreq, c_setp, c_bls, c_blm, c_fc] =
            col;
        let getf = |f: &[&str], i: usize| f[i].trim().parse::<f64>().unwrap_or(f64::NAN);
        let or = |v: f64, d: f64| if v.is_nan() { d } else { v };

        const BMS: &str = "com.victronenergy.battery.golden";
        // Persistent broker + socguard (they hold latch/sticky state across the trace, exactly
        // as the running daemon does). Seeded so the BMS service resolves.
        let mut seed = MockBus::new();
        seed.add_service(BMS);
        seed.set_str(SYS, P_ACTIVE_BMS, BMS);
        let mut broker = battery_limits::BatteryBroker::new(&seed);
        let mut socguard = socguard::SocGuard::new(&seed);

        // The design-capacity envelope handed to the production clamp — named, not a literal.
        let (env_lo, env_hi) = (-HARD_CLAMP_W, HARD_CLAMP_W);
        let mut st = state(Kind::Stock);
        st.owner_us = true; // exercise the full owning path
        let c = cfg(Stage::Takeover);
        let (mut rows, mut fc_rows) = (0u32, 0u32);

        for (n, line) in lines.enumerate() {
            let f: Vec<&str> = line.split(',').collect();
            if f.len() < header.len() {
                continue;
            }
            let (grid, soc, target) = (getf(&f, c_grid), getf(&f, c_soc), getf(&f, c_setp));
            if grid.is_nan() || soc.is_nan() || target.is_nan() {
                continue;
            }
            let hub4_fc = f[c_fc].trim() == "1";

            // Rebuild the row's dbus view and derive bounds + SocGuard via the REAL code.
            let mut bus = MockBus::new();
            bus.add_service(BMS);
            bus.set_str(SYS, P_ACTIVE_BMS, BMS);
            bus.set(SYS, P_DVCC, or(getf(&f, c_dvcc), 1.0));
            bus.set(SYS, P_PV_CURRENT, 0.0);
            bus.set(VEBUS, P_FIRMWARE, or(getf(&f, c_fw), 1376.0));
            bus.set(VEBUS, P_DC_VOLTAGE, or(getf(&f, c_vdc), 52.0));
            bus.set(VEBUS, P_SUSTAIN, 0.0);
            bus.set(VEBUS, P_MULTI_MAXCHG_I, or(getf(&f, c_mmci), 140.0));
            bus.set(VEBUS, socguard::P_TPIMF, 1.0);
            bus.set(BMS, P_BMS_CCL, or(getf(&f, c_ccl), 280.0));
            bus.set(BMS, P_BMS_DCL, or(getf(&f, c_dcl), 280.0));
            bus.set(BMS, socguard::P_CHARGE_REQUEST, or(getf(&f, c_chgreq), 0.0));
            bus.set(SETTINGS, P_MAXCHARGE, -1.0);
            bus.set(SETTINGS, P_MAXDISCHARGE, -1.0);
            bus.set(SETTINGS, P_MAXCHARGE_PCT, 100.0);
            bus.set(SETTINGS, P_MAXDISCHARGE_PCT, 100.0);
            bus.set(SETTINGS, P_BL_STATE, or(getf(&f, c_bls), 0.0));
            bus.set(SETTINGS, P_BL_MINSOC, or(getf(&f, c_blm), 10.0));
            bus.set(socguard::HUB4, socguard::P_OVR_FORCECHARGE, if hub4_fc { 1.0 } else { 0.0 });

            let tr = broker.tick(&bus);
            let (lo, hi) = battery_limits::clamp_bounds(tr.enforced, env_lo, env_hi);
            let sg = socguard.tick(&bus);

            // Regime equality end-to-end: our force-charge decision == hub4's on real data.
            assert_eq!(sg.force_charge, hub4_fc, "row {n}: force-charge regime mismatch");

            let snap = Snapshot {
                s: Sensors {
                    grid,
                    reported: or(getf(&f, c_acinp), 0.0),
                    target,
                    soc,
                    state: getf(&f, c_state) as i64,
                },
                dt: 20.0,
                dt_clamped: false,
                min_soc: or(getf(&f, c_blm), 10.0),
                lo,
                hi,
                effective_target: target,
                sg,
                feed_block_charge: false,
                feed_force_flat: false,
                gen_disable_export: false,
                gm_should_write: true,
                shore_out: ShoreOut::default(),
                hub4_actual: None,
            };
            let d = decide(&snap, &mut st, &c, n as f64 * 20.0);

            // Invariants, all checked against the bounds the PRODUCTION broker derived this row:
            assert!(d.command.is_finite(), "row {n}: non-finite command");
            assert!(
                d.command >= lo - 1e-6 && d.command <= hi + 1e-6,
                "row {n}: command {} outside the BMS-derived envelope [{lo},{hi}]",
                d.command
            );
            assert!(
                d.command.abs() <= HARD_CLAMP_W + 1e-6,
                "row {n}: command {} exceeds design capacity",
                d.command
            );
            if sg.force_charge {
                assert!(d.command >= -1e-6, "row {n}: force-charge but command {} discharges", d.command);
            }
            if hub4_fc {
                fc_rows += 1;
            }
            rows += 1;
        }
        assert!(rows > 100, "expected the full golden trace, got {rows} rows");
        assert!(fc_rows > 0, "fixture should include force-charge rows");
        eprintln!("golden full-loop replay: {rows} rows ({fc_rows} force-charge), invariants held vs live-derived bounds");
    }
}
