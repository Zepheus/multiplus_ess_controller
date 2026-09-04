//! venus-ess-controller — run our own grid-setpoint loop on a Victron Venus GX.
//!
//! Modes
//!   shadow (default): READ-ONLY. Reads live dbus, computes what it WOULD command,
//!                     prints it. Never writes, never changes Hub4Mode.
//!   live            : Sets Hub4Mode=3 (takes the setpoint loop from the stock daemon),
//!                     writes /Hub4/L1/AcPowerSetpoint each tick, restores Hub4Mode
//!                     on exit. DOUBLE-GATED: requires `--mode live` AND the env var
//!                     VENUS_ESS_LIVE=I_UNDERSTAND, or it refuses to run.
//!
//! ## Where the logic lives (module map)
//!
//! One tick = `sample()` (every bus read) -> `loop_core::decide()` (every decision)
//! -> `actuate()` (every write). Everything else is either a per-subsystem LAW
//! (a pure function you can read top-to-bottom) or machinery.
//!
//! CALCULATIONS / BUSINESS LOGIC — the four places power numbers are made:
//!   control.rs         the control law: stock reference, PI integral, adaptive
//!                      feed-forward (RLS leak model) — `Controller::update`
//!   loop_core.rs       command composition (`composed_command`: law -> SocGuard
//!                      floors + AC-out feedforward -> feed-in/generator gates ->
//!                      envelope clamp), ownership envelope, sanity supervisor,
//!                      the stock write-EMA (`stock_write_ema`)
//!   socguard.rs        BatteryLife enactment (`enact`: force-charge predicate,
//!                      discharge-inhibit set, /Overrides caps, charge floor)
//!   battery_limits.rs  BMS-derived charge/discharge bounds (`recompute`) and the
//!                      final (lo,hi) clamp (`clamp_bounds`)
//!
//! SUPPORTING SUBSYSTEM LAWS (same shape: read Inputs -> pure tick -> Out):
//!   feedin.rs      feed-in mode/flag laws + MaxFeedInPower distribution
//!   gridmeter.rs   meter presence/staleness guard: may we write this tick?
//!   shore.rs       AC-input current limit (peak shave) -> OverruledShoreLimit
//!   generator.rs   genset detection: never export toward a generator
//!   dess.rs        Dynamic-ESS overrides (target swap; inert when DESS is off)
//!   pv.rs          AC-PV power limiter (constant stub on installs without AC PV)
//!
//! MACHINERY (no power math):
//!   main.rs        this file: the tick loop, sample/actuate glue, take-over /
//!                  hand-back sequencing, shutdown restore
//!   cli.rs         argument parsing + help text
//!   telemetry.rs   bounded tmpfs CSV writer + eMMC path guard
//!   dbus.rs        the Bus trait + zbus transport; states.rs: named enum mirror
//!   src/tests/     cross-module golden + replay suites (unit tests live
//!                  inline at the bottom of the module they test)
//! Safety envelope (live):
//!   * We only ever discharge/idle within limits. We NEVER force-charge.
//!   * At/below the SOC floor we HOLD (no further discharge) but keep the loop;
//!     SocGuard/sustain/scheduling are enacted from systemcalc's state like stock.
//!     Control is handed back to the stock daemon (Hub4Mode=1) only on a sanity trip,
//!     a hold breach, or an external mode change.
//!   * Commands are clamped to the configured charge/discharge power limits and a
//!     hard sanity bound.
//!   * If the process dies for any reason, it stops writing; the Multi reverts to
//!     passthru within 60 s (safe: house runs on grid, battery idle). On clean exit
//!     or SIGINT/SIGTERM we additionally restore Hub4Mode=1.

mod battery_limits;
mod control;
mod dbus;
mod dess;
mod feedin;
mod generator;
mod gridmeter;
mod loop_core;
mod pv;
mod shore;
mod socguard;
mod states;
mod telemetry;
mod cli;
#[cfg(test)]
mod testbus;
#[cfg(test)]
#[path = "tests/goldens.rs"]
mod goldens_tests;
#[cfg(test)]
#[path = "tests/replay.rs"]
mod replay_tests;

use control::Controller;
use cli::parse_args;
use dbus::*;
use telemetry::Telemetry;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};


static SHUTDOWN: AtomicBool = AtomicBool::new(false);

extern "C" {
    fn signal(signum: i32, handler: usize) -> usize;
}
extern "C" fn on_signal(_sig: i32) {
    SHUTDOWN.store(true, Ordering::SeqCst);
}

// The design-capacity clamp + all derived safety params live in loop_core, anchored on the
// one hardware number (`loop_core::HARD_CLAMP_W` = inverter pair continuous x margin).
use loop_core::HARD_CLAMP_W;

/// The rollout ladder, selected with `--stage`. Each rung
/// is strictly larger-authority than the previous; `writes()` marks the ones that touch the
/// live battery system and therefore require the double-gate.
///   Shadow (0)   read-only: compute what we WOULD do, compare to stock, never write.
///   Trim   (1)   mode 1 + /Overrides/Setpoint outer-integral trim. Stock keeps ALL safety
///                (SocGuard, clamps, feed-in, meter failsafe); we only bias the grid target.
///                Fail-safe = the 300 s override watchdog reverts to stock's own setpoint.
///   Takeover (2) mode 1->3: WE own the whole loop (setpoint + every subsystem). Fail-safe =
///                the 60 s VE.Bus watchdog reverts the Multi to passthru.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Stage {
    Shadow,
    Trim,
    Takeover,
}

impl Stage {
    fn parse(s: &str) -> Option<Stage> {
        match s {
            "shadow" | "0" => Some(Stage::Shadow),
            "trim" | "1" => Some(Stage::Trim),
            // `live` kept as a back-compat alias for the old --mode live (== full takeover).
            "takeover" | "2" | "live" => Some(Stage::Takeover),
            _ => None,
        }
    }
    /// True for stages that write to the live system (require the double-gate).
    fn writes(self) -> bool {
        !matches!(self, Stage::Shadow)
    }
    fn name(self) -> &'static str {
        match self {
            Stage::Shadow => "shadow(0)",
            Stage::Trim => "trim(1)",
            Stage::Takeover => "takeover(2)",
        }
    }
}


use loop_core::Sensors;

fn read_sensors(bus: &dyn Bus) -> Option<Sensors> {
    Some(Sensors {
        grid: bus.get_f64(SYS, P_GRID)?,
        reported: bus.get_f64(VEBUS, P_ACTIVEIN)?,
        target: bus.get_f64(SETTINGS, P_SETPOINT_SETTING)?,
        soc: bus.get_f64(SYS, P_SOC)?,
        // Telemetry/log only (nothing decides on it): never abort a tick over it.
        state: bus.get_f64(SYS, P_STATE).map_or(-1, |v| v as i64),
    })
}

/// All stateful subsystem instances + the grid-meter presence cache, held together so the
/// read phase (`sample`) can drive them and the write phase (`actuate`) can reach the two
/// modules that own their own write side (`feedin`, `shore`).
struct Subsystems {
    broker: battery_limits::BatteryBroker,
    dess: dess::Dess,
    feedin: feedin::FeedIn,
    generator: generator::Generator,
    gridmeter: gridmeter::GridMeterGuard,
    shore: shore::ShoreLimiter,
    pv: pv::PvLimiter,
    socguard: socguard::SocGuard,
    /// Grid-meter guard decision, refreshed at the 2.5 s presence cadence and cached between.
    gm_cached: gridmeter::GridMeterGuardOut,
    gm_next: Instant,
    logged_clamp: bool,
    /// Most recent feed-in tick output — held so `actuate` can call its `.write()` (the
    /// feed-in flags live on the tick output, not the `FeedIn` instance).
    last_feed: Option<feedin::Output>,
}

/// The READ phase: every bus read for this tick, packed into a `Snapshot`. The only reader.
#[allow(clippy::too_many_arguments)]
fn sample(
    bus: &dyn Bus,
    sub: &mut Subsystems,
    stage: Stage,
    owner_us: bool,
    env_lo: f64,
    env_hi: f64,
    min_soc_fallback: f64,
    now: Instant,
    dt: f64,
    dt_clamped: bool,
) -> Option<loop_core::Snapshot> {
    // The takeover stage IS the legitimate mode-3 (external-control) owner: the
    // grid-meter guard must not treat our own mode write as "someone else owns it".
    let mode3_is_ours = matches!(stage, Stage::Takeover);
    let s = read_sensors(bus)?;

    // LIVE ownership floor: the UI/VRM/API-controlled MinimumSocLimit (raised by the user's
    // dispatch automation), read every tick. --min-soc is only the fallback if the read fails.
    let min_soc = bus
        .get_f64(SETTINGS, P_BL_MINSOC)
        .filter(|v| v.is_finite())
        .unwrap_or(min_soc_fallback);

    // Battery limits -> the per-tick command clamp (design-capacity bounded).
    let tr = sub.broker.tick(bus);
    let (lo, hi) = battery_limits::clamp_bounds(tr.enforced, env_lo, env_hi);
    if !sub.logged_clamp {
        let hub4_live = bus.get_f64("com.victronenergy.hub4", "/MaxDischargePower");
        eprintln!(
            "BMS clamp: lo={lo:.0}W hi={hi:.0}W  our_published_maxdis={:.0}W hub4_live={hub4_live:?}  (env [{env_lo:.0},{env_hi:.0}])",
            tr.published.max_discharge_power
        );
        sub.logged_clamp = true;
    }

    // Effective target = DESS scheduled-setpoint override + PV feed-forward. PV curtailment
    // can WRITE to real PV inverters, so it runs only in full takeover while we own control
    // (STUB — read-only — otherwise); this is the one subsystem that self-actuates.
    let dess_out = sub.dess.tick(bus);
    let pv_out = if matches!(stage, Stage::Takeover) && owner_us {
        sub.pv.tick(bus, pv::PvContext::default())
    } else {
        pv::PvOut::STUB
    };
    let effective_target = dess_out.target(s.target) + pv_out.pv_offset_w;

    // SocGuard first (its force_charge feeds the feed-in override).
    let sg = sub.socguard.tick(bus);
    // Merged live discharge bound for feed-in's canDischarge: broker (BMS DCL, sustain,
    // BL discharged states) ∧ SocGuard (state inhibit ∧ /Overrides/MaxDischargePower).
    let mut dis_bound = tr.enforced.max_discharge_power;
    if sg.max_discharge_w.is_finite() {
        dis_bound =
            if dis_bound.is_finite() { dis_bound.min(sg.max_discharge_w) } else { sg.max_discharge_w };
    }
    sub.feedin.set_overrides(feedin::Overrides {
        feed_in_excess: dess_out.feed_in_excess,
        force_charge: sg.force_charge,
        discharge_bound_w: dis_bound,
    });
    let feed = sub.feedin.tick(bus);
    let feed_block_charge = feed.block_charge();
    let feed_force_flat = feed.force_flat();
    sub.last_feed = Some(feed); // held for actuate's write side
    let gen = sub.generator.tick(bus, generator::ForceChargeCtx::default());

    // Grid-meter guard at its native 2.5 s cadence; reuse the cached decision between.
    if now >= sub.gm_next {
        sub.gm_cached = sub.gridmeter.tick(bus, mode3_is_ours);
        sub.gm_next = now + gridmeter::PRESENCE_TICK;
    }

    let shore_out = sub.shore.tick_from_bus(
        bus,
        shore::read_ac_input_limit(bus),
        sub.gm_cached.usable,
        shore::read_always_peak_shave(bus),
        s.soc > min_soc,
        sg.force_charge,
    );

    // Keep the diagnostic-only module outputs "used".
    let _ = (pv_out.pv_disable, sub.gm_cached.alarm_no_grid_meter);

    Some(loop_core::Snapshot {
        s,
        dt,
        dt_clamped,
        min_soc,
        lo,
        hi,
        effective_target,
        sg,
        feed_block_charge,
        feed_force_flat,
        gen_disable_export: gen.disable_export,
        gm_should_write: sub.gm_cached.should_write_setpoint,
        shore_out,
        hub4_actual: bus.get_f64(VEBUS, P_HUB4_SETPOINT),
        dc_batt_w: bus.get_f64(SYS, P_DC_BATTERY_POWER).unwrap_or(f64::NAN),
        ac_out_w: bus.get_f64(VEBUS, P_ACOUT).unwrap_or(f64::NAN),
        hub4mode_live: bus.get_i32(SETTINGS, P_HUB4MODE),
    })
}

/// The WRITE phase: the single place every actuation happens. Write-phase safety checks
/// belong here. `st` is `&mut` only so a Hub4Mode write-failure can fail safe to not-owner.
fn actuate(
    bus: &dyn Bus,
    sub: &Subsystems,
    snap: &loop_core::Snapshot,
    dec: &loop_core::Decision,
    st: &mut loop_core::LoopState,
) {
    let mut mode_ok = true;
    if let Some(m) = dec.hub4mode {
        if let Err(e) = bus.set_i32(SETTINGS, P_HUB4MODE, m) {
            eprintln!("Hub4Mode set failed: {e}");
            st.owner_us = false; // fail safe
            mode_ok = false;
        }
    }
    match dec.write {
        // Skip the setpoint when this tick's mode flip failed: stock is still in its
        // own mode and writing — never race it with a second writer, even for one tick.
        loop_core::Write::Setpoint(v) if mode_ok && v.is_finite() => {
            if let Err(e) = bus.set_f64(VEBUS, P_HUB4_SETPOINT, v) {
                eprintln!("setpoint write failed: {e} (Multi will passthru if persistent)");
            }
        }
        loop_core::Write::Setpoint(v) => {
            eprintln!("skipping setpoint write ({v}): mode flip failed or non-finite");
        }
        loop_core::Write::Override(v) if v.is_finite() => {
            if let Err(e) = bus.set_f64(dbus::HUB4, dbus::P_OVR_SETPOINT, v) {
                eprintln!("override write failed: {e} (stock setpoint resumes after 300 s)");
            }
        }
        loop_core::Write::Override(v) => {
            eprintln!("skipping non-finite override write ({v})");
        }
        loop_core::Write::Nothing => {}
    }
    if dec.feed_shore_write {
        if let Some(f) = &sub.last_feed {
            let _ = f.write(bus);
        }
        let _ = sub.shore.write(bus, &snap.shore_out);
    }
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };

    // Double-gate any stage that writes to the live system (trim + takeover).
    if args.stage.writes() {
        let env_ok = std::env::var("VENUS_ESS_LIVE").as_deref() == Ok("I_UNDERSTAND");
        if !env_ok || !args.confirm {
            eprintln!(
                "REFUSING {}: needs BOTH `--confirm` AND env VENUS_ESS_LIVE=I_UNDERSTAND.\n\
                 This writes to the live battery system.",
                args.stage.name()
            );
            std::process::exit(3);
        }
    }

    unsafe {
        signal(1, on_signal as *const () as usize); // SIGHUP (dropped SSH session)
        signal(2, on_signal as *const () as usize); // SIGINT
        signal(15, on_signal as *const () as usize); // SIGTERM
    }

    let bus = match SystemBus::connect() {
        Ok(b) => b,
        Err(e) => {
            eprintln!("dbus connect failed: {e}");
            std::process::exit(5);
        }
    };
    // Optional RAM-backed telemetry (bounded, never flash). Cap per file; with one
    // rotation total footprint is at most 2x this. Default 4 MB/file (~48 h @ 1 Hz).
    let telemetry_cap: u64 = args.telemetry_max_mb * 1024 * 1024;
    let mut telemetry = match &args.telemetry {
        Some(p) => match Telemetry::open(p, telemetry_cap) {
            Ok(t) => {
                eprintln!("telemetry -> {p} (bounded {}MB x2, tmpfs)", telemetry_cap / 1_048_576);
                Some(t)
            }
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(4);
            }
        },
        None => None,
    };

    // Outer envelope: CLI --max-* if given, else the +-138 kW absurdity ceiling the
    // stock writer applies. The command bound itself follows stock: the battery
    // broker's live BMS/user/override discharge limit (verified in the stock binary —
    // there is no inverter-capacity term; the inverters police their own capacity).
    let env_lo = -pick_limit(args.max_discharge, None, loop_core::ABSURD_CLAMP_W)
        .min(loop_core::ABSURD_CLAMP_W);
    let env_hi = pick_limit(args.max_charge, None, loop_core::ABSURD_CLAMP_W)
        .min(loop_core::ABSURD_CLAMP_W);

    // Design-power anchor for the safety layer: the explicit --max-* envelope if given, else the
    // inverter design capacity. slew / sanity-band / dt-clamp all DERIVE from this (each still
    // overridable via --slew-w-per-s / --sanity-band-w) — no independently hand-tuned constants.
    let design_w = {
        let m = args.max_charge.max(args.max_discharge);
        if m > 0.0 { m.min(HARD_CLAMP_W) } else { HARD_CLAMP_W }
    };
    let safety = loop_core::SafetyLimits::derive(
        design_w,
        args.interval,
        args.slew_w_per_s,
        args.sanity_band_w,
    );
    eprintln!(
        "safety (derived from design {:.0}W): slew={:.0}W/s{} sanity>{:.0}W for {:.0}s dt-clamp {:.1}s",
        safety.design_w,
        safety.slew_w_per_s,
        if args.slew_w_per_s.is_some() { " (override)" } else { "" },
        safety.sanity_band_w,
        safety.sanity_secs,
        safety.max_dt_s,
    );

    // Record original mode so we can always restore it. If the mode is ALREADY
    // EXTERNAL at startup, a previous takeover run died uncleanly (the setting is
    // persistent) — "restoring" it on hand-back would keep stock disengaged forever,
    // so treat the original as the stock default instead.
    let orig_mode = {
        let m = bus.get_f64(SETTINGS, P_HUB4MODE).unwrap_or(1.0) as i32;
        if m == gridmeter::HUB4MODE_EXTERNAL {
            eprintln!(
                "WARNING: Hub4Mode was already EXTERNAL({m}) at startup — previous run \
                 likely died mid-takeover. Using 1 as the restore target."
            );
            1
        } else {
            m
        }
    };

    // Subsystem instances (driven by `sample`; `feedin`/`shore` also written by `actuate`).
    let mut gridmeter = gridmeter::GridMeterGuard::new();
    let gm0 = gridmeter.tick(&bus, matches!(args.stage, Stage::Takeover)); // seed the presence cache
    let mut sub = Subsystems {
        broker: battery_limits::BatteryBroker::new(&bus),
        dess: dess::Dess::new(&bus),
        feedin: feedin::FeedIn::new(&bus),
        generator: generator::Generator::new(&bus),
        gridmeter,
        shore: shore::ShoreLimiter::new(1),
        pv: pv::PvLimiter::new(&bus),
        socguard: socguard::SocGuard::new(&bus),
        gm_cached: gm0,
        gm_next: Instant::now() + gridmeter::PRESENCE_TICK,
        logged_clamp: false,
        last_feed: None,
    };

    // Carried decision state + immutable per-run config for the pure `decide()`.
    let mut st = loop_core::LoopState {
        ctrl: Controller::new(args.kind),
        owner_us: false,
        last_flip_t: -60.0, // dwell already elapsed at boot
        trim_override: None,
        last_out: None,
        smith_weff: f64::NAN,
        smith_weff_lag: f64::NAN,
        smith_prev_write: f64::NAN,
        oob_since: None,
        hold_oob_since: None,
        prev_force_charge: false,        ceded_external: false,
        cyc_edges: [f64::NEG_INFINITY; loop_core::CYC_EDGES_MAX],
        cyc_prev_load: f64::NAN,
        cycling: false,
        prev_saturated: 0,
        sanity_trips: 0,
    };
    // Saturation reference: the pair's advertised nominal power. Unknown => INFINITY
    // (saturation then only detectable at the clamp bounds).
    let inverter_nominal_w = bus
        .get_f64(VEBUS, P_NOMINAL_INVERTER)
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(f64::INFINITY);
    eprintln!(
        "inverter nominal {inverter_nominal_w:.0} W (saturation beyond {:.0} W); bounds follow stock: BMS/user/override, +-{:.0} W ceiling",
        inverter_nominal_w * loop_core::CAPACITY_MARGIN,
        loop_core::ABSURD_CLAMP_W
    );
    // The hold supervisor's witness. NaN/None here means it stays inert for the whole run
    // (no evidence, no trip) — say so up front rather than let a silent gap pass as "fine".
    let dc0 = bus.get_f64(SYS, P_DC_BATTERY_POWER);
    eprintln!(
        "hold supervisor: /Dc/Battery/Power={} (discharge past the active inhibit by >{:.0} W for {:.0} s trips; None => supervisor INERT); err-gain pacing: dead={} W slope={} W cap={}",
        dc0.map_or("None".to_string(), |v| format!("{v:.0}W")),
        safety.sanity_band_w,
        safety.sanity_secs,
        args.errgain_dead_w,
        args.errgain_slope_w,
        args.errgain_cap
    );
    let cfg = loop_core::DecideCfg {
        stage: args.stage,
        min_dwell_s: 30.0, // anti-thrash on ownership flips
        trim_ki: args.trim_ki,
        orig_mode,
        slew_w_per_s: safety.slew_w_per_s,
        sanity_band_w: safety.sanity_band_w,
        sanity_secs: safety.sanity_secs,
        // Flat 0.5 write-EMA gain: stock's adaptive variant needs a fast grid meter
        // (< 250 ms updates) + recent Multi firmware, and this install's live traces
        // show the flat branch. See loop_core::stock_write_ema.
        ema_adaptive: false,
        ema_gain_up: loop_core::EMA_GAIN_UP,
        ema_gain_down: loop_core::EMA_GAIN_DOWN,
        errgain_dead_w: args.errgain_dead_w,
        errgain_slope_w: args.errgain_slope_w,
        errgain_cap: args.errgain_cap,
        inverter_nominal_w,
        smith: args.smith,
    };
    // Edge-triggered safety logging state (only log a trip when it newly fires).
    let mut last_safety = loop_core::Safety::default();

    eprintln!(
        "stage={} controller={} interval={}s min_soc_fallback={}% (live from BatteryLife/MinimumSocLimit) envelope=[{:.0}..{:.0}]W orig_hub4mode={}",
        args.stage.name(),
        st.ctrl.name(),
        args.interval,
        args.min_soc,
        env_lo,
        env_hi,
        orig_mode
    );
    match args.stage {
        Stage::Shadow => eprintln!("shadow: READ-ONLY, no writes, no mode change."),
        Stage::Trim => eprintln!(
            "TRIM: Hub4Mode stays {orig_mode}; writing /Overrides/Setpoint (trim_ki={}) — stock keeps \
             all safety; 300 s override watchdog reverts us.",
            args.trim_ki
        ),
        Stage::Takeover => eprintln!(
            "TAKEOVER: will set Hub4Mode=3 inside the envelope and restore {orig_mode} on exit."
        ),
    }
    if !args.quiet {
        println!(
            "{:>6} {:>8} {:>9} {:>7} {:>5} {:>6} {:>9} {:>7} {:>8}",
            "t", "grid", "reported", "target", "soc", "state", "command", "owner", "note"
        );
    }

    let start = Instant::now();
    // Seed `prev` one interval back so the FIRST tick's dt is a full interval, not
    // the ~0 that `Instant::now()` here would give (which collapses the slew budget
    // to a few watts and false-alarms slew/dt-clamp on the take-over write).
    let mut prev = start
        .checked_sub(Duration::from_secs_f64(args.interval))
        .unwrap_or(start);
    let mut tick: u64 = 0;
    // Stock's feed-in flag values captured at take-over, restored at hand-back/exit
    // (stock's write-if-changed cache can otherwise leave OUR last flag values stuck).
    let mut flag_snap: Option<feedin::FlagSnapshot> = None;

    loop {
        if SHUTDOWN.load(Ordering::SeqCst) {
            break;
        }
        let now = Instant::now();
        // Failsafe: clamp dt so a scheduler stall can't lurch the integrator on the catch-up
        // tick. The clamp is itself a (warning-level) safety event.
        let raw_dt = (now - prev).as_secs_f64();
        let dt = raw_dt.clamp(0.001, safety.max_dt_s);
        let dt_clamped = raw_dt > safety.max_dt_s;
        prev = now;
        let t = (now - start).as_secs_f64();

        // --- sample: every bus read for this tick -------------------------------
        let snap = match sample(
            &bus, &mut sub, args.stage, st.owner_us, env_lo, env_hi, args.min_soc, now, dt,
            dt_clamped,
        ) {
            Some(snap) => snap,
            None => {
                println!("{t:6.0} {:>8} read-fail: skipping tick (no write)", "");
                sleep_or_break(args.interval);
                continue;
            }
        };

        // --- decide: pure composed law + ownership envelope + trim integral -----
        let dec = loop_core::decide(&snap, &mut st, &cfg, t);

        // --- actuate: the single write phase (writing stages only) --------------
        if args.stage.writes() {
            // Capture stock's feed-in flags BEFORE our first write of them (take-over
            // tick); put them back the moment we hand back.
            if matches!(args.stage, Stage::Takeover) && dec.flipped && dec.owner_us {
                flag_snap = Some(feedin::snapshot_flags(&bus));
            }
            actuate(&bus, &sub, &snap, &dec, &mut st);
            if matches!(args.stage, Stage::Takeover) && dec.flipped && !dec.owner_us {
                if let Some(fs) = flag_snap.take() {
                    feedin::restore_flags(&bus, &fs);
                    eprintln!("feed-in flags restored to stock's captured values");
                }
            }
        }

        // Ownership transitions are rare events -> stderr, so a --quiet service records them.
        if dec.flipped {
            eprintln!(
                "[t={t:.0}] {} (soc={:.0}% min-soc={:.0}% state={} grid={:.0}W)",
                dec.note, snap.s.soc, snap.min_soc, snap.s.state, snap.s.grid
            );
        }

        // Safety trips: stderr on the RISING edge with a stable `SAFETY:` prefix (so a
        // log/command-line HA sensor or VRM can detect it), plus the telemetry reason column
        // below. The trip's *effect* is also visible through signals HA already reads: in
        // takeover a sanity handback reverts Hub4Mode to 1; in trim it invalidates
        // /Overrides/Setpoint. (Tier-B additionally publishes /Alarms/EssSafety.)
        if dec.safety.any() && dec.safety != last_safety {
            eprintln!(
                "[t={t:.0}] SAFETY: {} (grid={:.0}W target={:.0}W soc={:.0}% min-soc={:.0}% dc={:.0}W maxdis={:.0}W fc={} alarm={})",
                dec.safety.reason(),
                snap.s.grid,
                snap.s.target,
                snap.s.soc,
                snap.min_soc,
                snap.dc_batt_w,
                snap.sg.max_discharge_w,
                snap.sg.force_charge as u8,
                dec.safety.alarm_level()
            );
        }
        last_safety = dec.safety;

        // Adaptive model: log the learned coefficients + current authority periodically so
        // convergence is observable (essential in --ff-mode observe, where nothing else
        // reveals what the model would do). Cheap: ~once/minute at a 1 s interval.
        if let Some(ff) = st.ctrl.ff_snapshot(snap.s.reported) {
            if tick == 0 || tick.is_multiple_of(60) {
                eprintln!(
                    "[t={t:.0}] MODEL: a={:.1}W b={:.4} ({:.1}% of load) conf={:.2} @load={:.0}W",
                    ff.a,
                    ff.b,
                    ff.b * 100.0,
                    ff.conf,
                    snap.s.reported.abs()
                );
            }
        }
        tick += 1;

        // Shadow fidelity: how close is our proposal to the stock daemon's actual command?
        let note_buf = if matches!(args.stage, Stage::Shadow) {
            snap.hub4_actual
                .map(|actual| format!("actual={actual:.0} Δ={:.0}", actual - dec.command))
        } else {
            None
        };
        let note: &str = note_buf.as_deref().unwrap_or(dec.note);

        // Per-tick human table -> stdout (suppressed with --quiet for a persistent service).
        if !args.quiet {
            let cmd_str = if dec.owner_us {
                format!("{:9.1}", dec.display_cmd)
            } else {
                format!("({:7.0})", dec.command)
            };
            println!(
                "{:6.0} {:8.1} {:9.1} {:7.1} {:5.0} {:6} {:>9} {:>7} {:12}",
                t,
                snap.s.grid,
                snap.s.reported,
                snap.s.target,
                snap.s.soc,
                snap.s.state,
                cmd_str,
                dec.owner_str,
                note
            );
        }

        // Per-tick machine telemetry -> bounded tmpfs file (never flash).
        if let Some(tel) = telemetry.as_mut() {
            let actual = note_buf
                .as_ref()
                .and_then(|nb| nb.split("actual=").nth(1))
                .and_then(|s| s.split_whitespace().next())
                .unwrap_or("");
            let reason = dec.safety.reason();
            // maxdis: effective discharge cap (state inhibit ∧ /Overrides/MaxDischargePower);
            // empty = unlimited. fc: the force-charge regime. Both exist so a charge-window
            // capture is replayable as a golden fixture without a separate capture tool.
            // acout: measured Multi AC-out (the clamp feedforward); empty = unread.
            // dcbatt: battery DC power (+ = charging), the hold supervisor's witness; empty = unread.
            let dcbatt = if snap.dc_batt_w.is_finite() {
                format!("{:.0}", snap.dc_batt_w)
            } else {
                String::new()
            };
            let acout = if snap.ac_out_w.is_finite() {
                format!("{:.0}", snap.ac_out_w)
            } else {
                String::new()
            };
            let maxdis = if snap.sg.max_discharge_w.is_finite() {
                format!("{:.0}", snap.sg.max_discharge_w)
            } else {
                String::new()
            };
            let _ = tel.write_line(&format!(
                "{t:.0},{:.1},{:.1},{:.1},{:.0},{},{:.1},{},{actual},{},{maxdis},{acout},{reason},{:.0},{dcbatt}",
                snap.s.grid,
                snap.s.reported,
                snap.s.target,
                snap.s.soc,
                snap.s.state,
                dec.display_cmd,
                dec.owner_str,
                snap.sg.force_charge as u8,
                snap.min_soc
            ));
        }

        if args.seconds != 0 && t >= args.seconds as f64 {
            break;
        }
        sleep_or_break(args.interval);
    }

    // --- clean shutdown: hand the system back to stock -------------------------
    match args.stage {
        // Takeover: restore the original Hub4Mode (we flipped it to EXTERNAL) and
        // stock's captured feed-in flags.
        Stage::Takeover => {
            if let Some(fs) = flag_snap.take() {
                feedin::restore_flags(&bus, &fs);
                eprintln!("feed-in flags restored to stock's captured values");
            }
            eprintln!("restoring Hub4Mode={orig_mode} ...");
            for attempt in 1..=5 {
                match bus.set_i32(SETTINGS, P_HUB4MODE, orig_mode) {
                    Ok(()) => {
                        eprintln!("Hub4Mode restored to {orig_mode}.");
                        break;
                    }
                    Err(e) => {
                        eprintln!("restore attempt {attempt} failed: {e}");
                        std::thread::sleep(Duration::from_millis(500));
                    }
                }
            }
        }
        // Trim: neutralise our override so stock resumes immediately; the 300 s override
        // watchdog is the backstop if this final write doesn't land.
        Stage::Trim => {
            let sp = bus.get_f64(SETTINGS, P_SETPOINT_SETTING).unwrap_or(0.0);
            match bus.set_f64(dbus::HUB4, dbus::P_OVR_SETPOINT, sp) {
                Ok(()) => eprintln!("trim override neutralised to {sp:.0}W (stock resumes)."),
                Err(e) => eprintln!("override neutralise failed: {e} (reverts via 300 s watchdog)."),
            }
        }
        Stage::Shadow => {}
    }
    eprintln!("stopped.");
}

fn pick_limit(arg: f64, setting: Option<f64>, default: f64) -> f64 {
    if arg > 0.0 {
        return arg;
    }
    match setting {
        Some(v) if v > 0.0 => v,
        _ => default,
    }
}

fn sleep_or_break(interval: f64) {
    // Sleep in small slices so a signal is noticed within ~100 ms.
    let mut left = interval;
    while left > 0.0 && !SHUTDOWN.load(Ordering::SeqCst) {
        let slice = left.min(0.1);
        std::thread::sleep(Duration::from_secs_f64(slice));
        left -= slice;
    }
}
