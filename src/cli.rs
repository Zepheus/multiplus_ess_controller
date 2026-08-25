//! CLI: argument parsing and the --help text. Pure machinery — the flags' meanings
//! are enforced in main/loop_core, not here.

use crate::control::{self, Kind};
use crate::Stage;

pub struct Args {
    pub stage: Stage,
    pub kind: Kind,
    pub interval: f64,
    pub min_soc: f64,
    pub soc_hyst: f64,
    pub max_discharge: f64,
    pub max_charge: f64,
    pub seconds: u64,
    pub confirm: bool,
    pub quiet: bool,
    pub telemetry: Option<String>,
    pub telemetry_max_mb: u64,
    /// Smith-style meter-lag compensation (default on; --no-smith to disable).
    pub smith: bool,
    pub trim_ki: f64,
    /// Export-conditional pacing (defaults from the Pareto sweep over the live
    /// event library — see tools/pace_sweep.py; deadband INFINITY disables).
    pub export_fast_gain: f64,
    pub export_fast_dead_w: f64,
    /// Safety overrides — None => derive from the design power (SafetyLimits::derive).
    pub slew_w_per_s: Option<f64>,
    pub sanity_band_w: Option<f64>,
}

pub fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        stage: Stage::Shadow,
        kind: Kind::Pi { ki: 0.05, i_max: 300.0 },
        interval: 1.0,
        min_soc: 15.0,
        soc_hyst: 3.0,
        max_discharge: 0.0, // 0 => read from settings / default
        export_fast_gain: 0.9,
        export_fast_dead_w: 100.0,
        max_charge: 0.0,
        seconds: 0,
        confirm: false,
        quiet: false,
        telemetry: None,
        telemetry_max_mb: 4,
        smith: true,
        // Outer-integral gain for Stage-1 trim: override_delta = trim_ki * grid_error_W * dt_s.
        // Small + slow — it only trims steady-state bias on top of stock's fast inner loop.
        trim_ki: 0.02,
        slew_w_per_s: None,
        sanity_band_w: None,
    };
    let mut ki: Option<f64> = None;
    let mut i_max: f64 = 300.0;
    let mut ctrl = String::from("pi");
    // Adaptive feed-forward knobs (only consulted for --controller adaptive).
    let mut ff_a: Option<f64> = None;
    let mut ff_b: Option<f64> = None;
    let mut ff_mode = String::from("apply");
    let mut ff_lambda: Option<f64> = None;
    let mut ff_conf_scale: Option<f64> = None;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = || it.next().ok_or_else(|| format!("{arg} needs a value"));
        match arg.as_str() {
            "--stage" => {
                let v = val()?;
                a.stage = Stage::parse(&v).ok_or_else(|| format!("bad --stage {v} (want shadow|trim|takeover)"))?;
            }
            // Deprecated alias: --mode shadow|live maps onto --stage shadow|takeover.
            "--mode" => {
                let v = val()?;
                a.stage = Stage::parse(&v)
                    .ok_or_else(|| format!("bad --mode {v} (use --stage shadow|trim|takeover)"))?;
            }
            "--trim-ki" => a.trim_ki = val()?.parse().map_err(|_| "bad --trim-ki")?,
            "--slew-w-per-s" => {
                a.slew_w_per_s = Some(val()?.parse().map_err(|_| "bad --slew-w-per-s")?)
            }
            "--sanity-band-w" => {
                a.sanity_band_w = Some(val()?.parse().map_err(|_| "bad --sanity-band-w")?)
            }
            "--controller" => ctrl = val()?,
            "--ki" => ki = Some(val()?.parse().map_err(|_| "bad --ki")?),
            "--i-max" => i_max = val()?.parse().map_err(|_| "bad --i-max")?,
            "--ff-a" => ff_a = Some(val()?.parse().map_err(|_| "bad --ff-a")?),
            "--ff-b" => ff_b = Some(val()?.parse().map_err(|_| "bad --ff-b")?),
            "--ff-mode" => ff_mode = val()?,
            // Convenience alias for --ff-mode observe (learn but don't actuate).
            "--ff-learn-only" => ff_mode = String::from("observe"),
            "--ff-lambda" => ff_lambda = Some(val()?.parse().map_err(|_| "bad --ff-lambda")?),
            "--ff-conf-scale-w" => {
                ff_conf_scale = Some(val()?.parse().map_err(|_| "bad --ff-conf-scale-w")?)
            }
            "--interval" => a.interval = val()?.parse().map_err(|_| "bad --interval")?,
            "--min-soc" => a.min_soc = val()?.parse().map_err(|_| "bad --min-soc")?,
            "--max-discharge" => a.max_discharge = val()?.parse().map_err(|_| "bad")?,
            "--max-charge" => a.max_charge = val()?.parse().map_err(|_| "bad")?,
            "--seconds" => a.seconds = val()?.parse().map_err(|_| "bad --seconds")?,
            "--confirm" => a.confirm = true,
            "--quiet" => a.quiet = true,
            "--no-smith" => a.smith = false,
            "--export-fast-gain" => {
                a.export_fast_gain = val()?.parse().map_err(|_| "bad --export-fast-gain")?
            }
            "--export-fast-dead-w" => {
                a.export_fast_dead_w = val()?.parse().map_err(|_| "bad --export-fast-dead-w")?
            }
            "--no-export-fast" => a.export_fast_dead_w = f64::INFINITY,
            "--telemetry" => a.telemetry = Some(val()?),
            "--telemetry-max-mb" => {
                a.telemetry_max_mb = val()?.parse().map_err(|_| "bad --telemetry-max-mb")?
            }
            "-h" | "--help" => return Err(help()),
            other => return Err(format!("unknown arg {other}\n{}", help())),
        }
    }
    // Numeric-flag validation: a bad value must error LOUDLY at startup, never silently
    // disable or invert a control/safety mechanism (e.g. NaN ki pins the integral at 0 ==
    // silently reverting to the leaky stock law; negative i-max inverts the clamp range and
    // pins the integral at +|i_max| == a permanent phantom-charge bias; interval <= 0
    // busy-loops the shared GX; slew <= 0 freezes the actuated setpoint).
    if let Some(k) = ki {
        if !(k.is_finite() && k > 0.0) {
            return Err("--ki must be finite and > 0".into());
        }
    }
    if !(i_max.is_finite() && i_max > 0.0) {
        return Err("--i-max must be finite and > 0".into());
    }
    if !(a.interval.is_finite() && (0.1..=3600.0).contains(&a.interval)) {
        return Err("--interval must be within [0.1, 3600] seconds".into());
    }
    if !(a.trim_ki.is_finite() && a.trim_ki > 0.0) {
        return Err("--trim-ki must be finite and > 0".into());
    }
    if !(a.min_soc.is_finite() && (0.0..=100.0).contains(&a.min_soc)) {
        return Err("--min-soc must be within [0, 100]".into());
    }
    if let Some(s) = a.slew_w_per_s {
        if !(s.is_finite() && s > 0.0) {
            return Err("--slew-w-per-s must be finite and > 0".into());
        }
    }
    if let Some(b) = a.sanity_band_w {
        if !(b.is_finite() && b > 0.0) {
            return Err("--sanity-band-w must be finite and > 0".into());
        }
    }
    a.kind = match ctrl.as_str() {
        "stock" => Kind::Stock,
        "true-integral" => Kind::TrueIntegral { ki: ki.unwrap_or(1.0) },
        "pi" => Kind::Pi { ki: ki.unwrap_or(0.05), i_max },
        "adaptive" => {
            let mode = match ff_mode.as_str() {
                "apply" => control::FfMode::Apply,
                "observe" | "learn-only" => control::FfMode::Observe,
                "frozen" => control::FfMode::Frozen,
                o => return Err(format!("bad --ff-mode {o} (want apply|observe|frozen)")),
            };
            let mut c = control::AdaptiveCfg::tuned();
            c.ki = ki.unwrap_or(c.ki);
            c.i_max = i_max;
            c.mode = mode;
            // A supplied seed (either coefficient) tightens the prior so it is trusted.
            c.seeded = ff_a.is_some() || ff_b.is_some();
            c.a0 = ff_a.unwrap_or(0.0);
            c.b0 = ff_b.unwrap_or(0.0);
            if !c.a0.is_finite() || !c.b0.is_finite() {
                return Err("--ff-a / --ff-b must be finite".into());
            }
            if let Some(l) = ff_lambda {
                // λ ∈ (0,1]; NaN/≤0 would make the RLS never learn (denom never > 0).
                if !(l.is_finite() && l > 0.0 && l <= 1.0) {
                    return Err("--ff-lambda must be in (0, 1]".into());
                }
                c.lambda = l;
            }
            if let Some(s) = ff_conf_scale {
                // Must be > 0: conf = scale/(scale+σ) only stays in (0,1] for positive scale.
                if !(s.is_finite() && s > 0.0) {
                    return Err("--ff-conf-scale-w must be > 0".into());
                }
                c.conf_scale_w = s;
            }
            Kind::Adaptive(c)
        }
        o => return Err(format!("unknown controller {o}")),
    };
    Ok(a)
}

pub fn help() -> String {
    "\
venus-ess-controller [--stage shadow|trim|takeover] [--controller stock|pi|true-integral|adaptive]
  [--ki F] [--i-max F] [--trim-ki F] [--slew-w-per-s F] [--sanity-band-w F] [--interval S]
  [--min-soc %] [--max-discharge W] [--max-charge W] [--seconds N] [--confirm] [--quiet]
  [--telemetry PATH]

adaptive: PI + a continuously-learned feed-forward leak(throughput) ~= a + b*|reported|
  (RLS, in-memory, no persistence). The model is taught by the integral and earns actuation
  authority only as its confidence grows, so a cold start behaves like pi and a low-data run
  never over-trusts. Options:
  [--ff-a W] [--ff-b FRAC] seed initial coefficients (tightens the prior so they are used
    from tick one); [--ff-mode apply|observe|frozen] (observe == --ff-learn-only: keep
    learning but actuate integral-only; frozen: apply the seed, never learn). NOTE:
    learning requires ACTUATION (takeover + owning + usable meter) — in shadow/trim the
    model deliberately stays null (MODEL: a=0 b=0), because a non-actuated loop would
    train it on a phantom regime. So observe/learn-only is a TAKEOVER mode: it drives
    with the plain integral while the model learns on the side;
  [--ff-lambda 0<L<=1] RLS forgetting (memory ~= 1/(1-L)); [--ff-conf-scale-w W] prediction-
    sigma at which the model gets 50% authority. The learned model is logged periodically as
    `MODEL: a=.. b=.. conf=..`; hard safety (write phase) is independent of anything learned.

Live ESS settings (read every tick, UI/VRM/API-controlled — never hardcoded): the ownership
floor is BatteryLife/MinimumSocLimit (--min-soc is only a fallback if that read fails); the
charge/discharge power limits are MaxChargePower/MaxDischargePower, enforced per-tick by the
battery broker alongside the live BMS DCL/CCL. --max-* set the fixed inverter design envelope.

Safety (write stages) is DERIVED from the design power (the --max-* envelope, else the inverter
design capacity) — no separate tuning: slew glitch-guard = full range / 2 s, sanity band = 15 %
of design power (hand back if grid stays beyond it for 30 s), dt-clamp = 5x --interval. Override
with --slew-w-per-s / --sanity-band-w. Every trip logs a stable `SAFETY:` prefix + a telemetry
`reason` column; the hand-back itself is visible via Hub4Mode / Overrides (Tier-B also publishes
/Alarms/EssSafety, the same /Alarms/* surface the HA/VRM Victron integration already reads).

Rollout stages (--stage, see):
  shadow (0)   READ-ONLY. Compute what we would command, compare to stock, never write.
  trim   (1)   Mode 1 + /Overrides/Setpoint outer-integral trim. Stock keeps ALL safety;
               we only bias the grid target. Reverts via the 300 s override watchdog.
  takeover (2) Mode 1->3. WE own the whole loop. Reverts to passthru via the 60 s watchdog.
  (--mode shadow|live is a deprecated alias: live == takeover.)

Logging (flash-safe):
  Per-tick output goes to STDOUT (terminal) and, if --telemetry is set, to a
  bounded RAM file. --quiet suppresses per-tick stdout for a persistent service.
  --telemetry PATH must be tmpfs (e.g. /run/venus-ess.csv or /tmp/…); it is
  size-capped with rotation (<=8 MB total) and REFUSES flash paths (/data,
  /var/log). Rare events (takeover/handback/errors) always go to stderr.

Default is shadow (read-only). trim and takeover also require --confirm AND env
VENUS_ESS_LIVE=I_UNDERSTAND (they write to the live battery system)."
        .to_string()
}
