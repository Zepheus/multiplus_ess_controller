//! multiplus_ess_controller — run our own grid-setpoint loop on a Victron Venus GX.
//!
//! Modes
//!   shadow (default): READ-ONLY. Reads live dbus, computes what it WOULD command,
//!                     prints it. Never writes, never changes Hub4Mode.
//!   live            : Sets Hub4Mode=3 (takes the setpoint loop from the stock ESS loop),
//!                     writes /Hub4/L1/AcPowerSetpoint each tick, restores Hub4Mode
//!                     on exit. DOUBLE-GATED: requires `--mode live` AND the env var
//!                     VENUS_ESS_LIVE=I_UNDERSTAND, or it refuses to run.
//!
//! Safety envelope (live):
//!   * We only ever discharge/idle within limits. We NEVER force-charge.
//!   * Below the SOC floor, or on an explicit recharge state, we hand control back
//!     to the stock ESS loop (Hub4Mode=1) so SocGuard/sustain/scheduling still work.
//!   * Commands are clamped to the configured charge/discharge power limits and a
//!     hard sanity bound.
//!   * If the process dies for any reason, it stops writing; the Multi reverts to
//!     passthru within 60 s (safe: house runs on grid, battery idle). On clean exit
//!     or SIGINT/SIGTERM we additionally restore Hub4Mode=1.

mod control;
mod dbus;

use control::{Controller, Kind};
use dbus::*;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Size-capped, self-rotating CSV writer for RAM-backed (tmpfs) telemetry.
/// Total on-disk footprint is bounded to 2*cap, so it can never grow into an OOM
/// even if left running for months. Refuses to touch flash-backed paths.
struct Telemetry {
    path: String,
    cap: u64,
    written: u64,
    file: std::fs::File,
}

impl Telemetry {
    /// cap_bytes is per-file; with one rotation the total is at most 2*cap_bytes.
    fn open(path: &str, cap_bytes: u64) -> Result<Self, String> {
        if is_flash_path(path) {
            return Err(format!(
                "refusing telemetry to '{path}': resolves onto eMMC flash. \
                 Use a tmpfs path like /run/… or /tmp/… instead."
            ));
        }
        if let Some(parent) = std::path::Path::new(path).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)
            .map_err(|e| format!("cannot open telemetry '{path}': {e}"))?;
        let mut t = Telemetry { path: path.to_string(), cap: cap_bytes, written: 0, file };
        let _ = t.write_line(
            "t,grid,reported,target,soc,state,command,owner,actual,delta",
        );
        Ok(t)
    }

    fn write_line(&mut self, line: &str) -> std::io::Result<()> {
        let bytes = line.len() as u64 + 1;
        if self.written + bytes > self.cap {
            self.rotate();
        }
        writeln!(self.file, "{line}")?;
        self.written += bytes;
        Ok(())
    }

    fn rotate(&mut self) {
        // Keep exactly one previous generation: path -> path.1 (overwrite), reopen fresh.
        let _ = std::fs::rename(&self.path, format!("{}.1", self.path));
        if let Ok(f) = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&self.path)
        {
            self.file = f;
            self.written = 0;
        }
    }
}

/// True if the path lives on the eMMC (directly under /data, or via the
/// /var/log -> /data/log symlink). Checks the canonicalised parent directory.
fn is_flash_path(path: &str) -> bool {
    let p = std::path::Path::new(path);
    let probe = p.parent().filter(|x| !x.as_os_str().is_empty()).unwrap_or(p);
    let resolved = std::fs::canonicalize(probe)
        .map(|r| r.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    resolved == "/data" || resolved.starts_with("/data/")
}

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

extern "C" {
    fn signal(signum: i32, handler: usize) -> usize;
}
extern "C" fn on_signal(_sig: i32) {
    SHUTDOWN.store(true, Ordering::SeqCst);
}

// Victron SystemState code we must react to: SocGuard force-charge -> hand back.
const STATE_RECHARGE: i64 = 258;

// Hard sanity bound on any command (W), regardless of settings. Two MultiPlus-II
// 48/5000 in parallel ~ 8 kW continuous; leave margin.
const HARD_CLAMP_W: f64 = 8000.0;

struct Args {
    live: bool,
    kind: Kind,
    interval: f64,
    min_soc: f64,
    soc_hyst: f64,
    max_discharge: f64,
    max_charge: f64,
    seconds: u64,
    confirm: bool,
    quiet: bool,
    telemetry: Option<String>,
    telemetry_max_mb: u64,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        live: false,
        kind: Kind::Pi { ki: 0.05, i_max: 300.0 },
        interval: 1.0,
        min_soc: 15.0,
        soc_hyst: 3.0,
        max_discharge: 0.0, // 0 => read from settings / default
        max_charge: 0.0,
        seconds: 0,
        confirm: false,
        quiet: false,
        telemetry: None,
        telemetry_max_mb: 4,
    };
    let mut ki: Option<f64> = None;
    let mut i_max = 300.0;
    let mut ctrl = String::from("pi");
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = || it.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--mode" => a.live = matches!(val()?.as_str(), "live"),
            "--controller" => ctrl = val()?,
            "--ki" => ki = Some(val()?.parse().map_err(|_| "bad --ki")?),
            "--i-max" => i_max = val()?.parse().map_err(|_| "bad --i-max")?,
            "--interval" => a.interval = val()?.parse().map_err(|_| "bad --interval")?,
            "--min-soc" => a.min_soc = val()?.parse().map_err(|_| "bad --min-soc")?,
            "--max-discharge" => a.max_discharge = val()?.parse().map_err(|_| "bad")?,
            "--max-charge" => a.max_charge = val()?.parse().map_err(|_| "bad")?,
            "--seconds" => a.seconds = val()?.parse().map_err(|_| "bad --seconds")?,
            "--confirm" => a.confirm = true,
            "--quiet" => a.quiet = true,
            "--telemetry" => a.telemetry = Some(val()?),
            "--telemetry-max-mb" => {
                a.telemetry_max_mb = val()?.parse().map_err(|_| "bad --telemetry-max-mb")?
            }
            "-h" | "--help" => return Err(help()),
            other => return Err(format!("unknown arg {other}\n{}", help())),
        }
    }
    a.kind = match ctrl.as_str() {
        "stock" => Kind::Stock,
        "true-integral" => Kind::TrueIntegral { ki: ki.unwrap_or(1.0) },
        "pi" => Kind::Pi { ki: ki.unwrap_or(0.05), i_max },
        o => return Err(format!("unknown controller {o}")),
    };
    Ok(a)
}

fn help() -> String {
    "\
multiplus_ess_controller [--mode shadow|live] [--controller stock|pi|true-integral]
  [--ki F] [--i-max F] [--interval S] [--min-soc %] [--max-discharge W]
  [--max-charge W] [--seconds N] [--confirm] [--quiet] [--telemetry PATH]

Logging (flash-safe):
  Per-tick output goes to STDOUT (terminal) and, if --telemetry is set, to a
  bounded RAM file. --quiet suppresses per-tick stdout for a persistent service.
  --telemetry PATH must be tmpfs (e.g. /run/venus-ess.csv or /tmp/…); it is
  size-capped with rotation (<=8 MB total) and REFUSES flash paths (/data,
  /var/log). Rare events (takeover/handback/errors) always go to stderr.

Default is shadow (read-only). Live also requires env VENUS_ESS_LIVE=I_UNDERSTAND."
        .to_string()
}

struct Sensors {
    grid: f64,
    reported: f64,
    target: f64,
    soc: f64,
    state: i64,
}

fn read_sensors(bus: &dyn Bus) -> Option<Sensors> {
    Some(Sensors {
        grid: bus.get_f64(SYS, P_GRID)?,
        reported: bus.get_f64(VEBUS, P_ACTIVEIN)?,
        target: bus.get_f64(SETTINGS, P_SETPOINT_SETTING)?,
        soc: bus.get_f64(SYS, P_SOC)?,
        state: bus.get_f64(SYS, P_STATE)? as i64,
    })
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };

    // Double-gate the live path.
    if args.live {
        let env_ok = std::env::var("VENUS_ESS_LIVE").as_deref() == Ok("I_UNDERSTAND");
        if !env_ok || !args.confirm {
            eprintln!(
                "REFUSING live: needs BOTH `--confirm` AND env VENUS_ESS_LIVE=I_UNDERSTAND.\n\
                 This flips Hub4Mode 1->3 and writes setpoints to the Multis."
            );
            std::process::exit(3);
        }
    }

    unsafe {
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
    let mut ctrl = Controller::new(args.kind);

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

    // Resolve power limits.
    let max_dis = pick_limit(args.max_discharge, bus.get_f64(SETTINGS, P_MAXDISCHARGE), 5000.0);
    let max_chg = pick_limit(args.max_charge, bus.get_f64(SETTINGS, P_MAXCHARGE), 5000.0);
    let lo = -max_dis; // most-negative command = full discharge
    let hi = max_chg.min(HARD_CLAMP_W);
    let lo = lo.max(-HARD_CLAMP_W);

    // Record original mode so we can always restore it.
    let orig_mode = bus.get_f64(SETTINGS, P_HUB4MODE).unwrap_or(1.0) as i32;

    eprintln!(
        "mode={} controller={} interval={}s min_soc={}% clamp=[{:.0}..{:.0}]W orig_hub4mode={}",
        if args.live { "LIVE" } else { "shadow" },
        ctrl.name(),
        args.interval,
        args.min_soc,
        lo,
        hi,
        orig_mode
    );
    if args.live {
        eprintln!("LIVE: will set Hub4Mode=3 inside the envelope and restore {orig_mode} on exit.");
    } else {
        eprintln!("shadow: READ-ONLY, no writes, no mode change.");
    }
    if !args.quiet {
        println!(
            "{:>6} {:>8} {:>9} {:>7} {:>5} {:>6} {:>9} {:>7} {:>8}",
            "t", "grid", "reported", "target", "soc", "state", "command", "owner", "note"
        );
    }

    let start = Instant::now();
    let mut prev = Instant::now();
    let mut owner_us = false; // do WE currently own control (Hub4Mode=3)?
    let mut last_flip = Instant::now()
        .checked_sub(Duration::from_secs(60))
        .unwrap_or_else(Instant::now);
    let min_dwell = Duration::from_secs(30); // anti-thrash on mode flips

    loop {
        if SHUTDOWN.load(Ordering::SeqCst) {
            break;
        }
        let now = Instant::now();
        let dt = (now - prev).as_secs_f64().max(0.001);
        prev = now;
        let t = (now - start).as_secs_f64();

        let s = match read_sensors(&bus) {
            Some(s) => s,
            None => {
                println!("{t:6.0} {:>8} read-fail: skipping tick (no write)", "");
                sleep_or_break(args.interval);
                continue;
            }
        };

        // --- envelope decision -------------------------------------------------
        // We may own control iff SOC is comfortably above the floor and we're not
        // being asked to force-charge. Hysteresis + dwell prevent mode thrash.
        let enter_ok = s.soc >= args.min_soc + args.soc_hyst && s.state != STATE_RECHARGE;
        let exit_now = s.soc <= args.min_soc || s.state == STATE_RECHARGE;
        let want_us = if owner_us { !exit_now } else { enter_ok };

        let dwell_ok = now - last_flip >= min_dwell;
        let mut note = "";
        if want_us != owner_us && (dwell_ok || !want_us) {
            // Always allow immediate hand-BACK (safety); throttle only take-OVER.
            owner_us = want_us;
            last_flip = now;
            if args.live {
                let target_mode = if owner_us { 3 } else { orig_mode };
                match bus.set_i32(SETTINGS, P_HUB4MODE, target_mode) {
                    Ok(()) => note = if owner_us { "TOOK OVER (mode3)" } else { "handed back (mode1)" },
                    Err(e) => {
                        eprintln!("Hub4Mode set failed: {e}");
                        owner_us = false; // fail safe
                    }
                }
            } else {
                note = if owner_us { "would take over" } else { "would hand back" };
            }
            // Rare event -> stderr, so a --quiet service still records transitions.
            eprintln!(
                "[t={t:.0}] {note} (soc={:.0}% state={} grid={:.0}W)",
                s.soc, s.state, s.grid
            );
            if !owner_us {
                ctrl.reset(); // avoid integral windup while the stock ESS loop drives
            }
        }

        // --- control -----------------------------------------------------------
        let cmd = ctrl.update(s.grid, s.reported, s.target, dt, lo, hi);
        let (owner_str, cmd_str) = if owner_us {
            if args.live {
                if let Err(e) = bus.set_f64(VEBUS, P_HUB4_SETPOINT, cmd) {
                    eprintln!("setpoint write failed: {e} (Multi will passthru if persistent)");
                }
            }
            ("US", format!("{cmd:9.1}"))
        } else {
            ctrl.reset(); // don't wind up the integral while the stock ESS loop drives
            ("hub4", format!("({cmd:7.0})"))
        };

        // Shadow fidelity: how close is our proposal to the stock ESS loop's actual command?
        let note_buf = if !args.live {
            bus.get_f64(VEBUS, P_HUB4_SETPOINT)
                .map(|actual| format!("actual={actual:.0} Δ={:.0}", actual - cmd))
        } else {
            None
        };
        if let Some(ref nb) = note_buf {
            note = nb;
        }

        // Per-tick human table -> stdout (terminal in interactive runs; suppressed
        // with --quiet so a persistent service emits nothing per-tick).
        if !args.quiet {
            println!(
                "{:6.0} {:8.1} {:9.1} {:7.1} {:5.0} {:6} {:>9} {:>7} {:12}",
                t, s.grid, s.reported, s.target, s.soc, s.state, cmd_str, owner_str, note
            );
        }

        // Per-tick machine telemetry -> bounded tmpfs file (never flash).
        if let Some(tel) = telemetry.as_mut() {
            let actual = note_buf
                .as_ref()
                .and_then(|nb| nb.split("actual=").nth(1))
                .and_then(|s| s.split_whitespace().next())
                .unwrap_or("");
            let _ = tel.write_line(&format!(
                "{t:.0},{:.1},{:.1},{:.1},{:.0},{},{:.1},{},{actual},",
                s.grid, s.reported, s.target, s.soc, s.state, cmd, owner_str
            ));
        }

        if args.seconds != 0 && t >= args.seconds as f64 {
            break;
        }
        sleep_or_break(args.interval);
    }

    // --- clean shutdown: restore original mode ---------------------------------
    if args.live {
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
