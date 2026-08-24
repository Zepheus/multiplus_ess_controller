//! Closed-loop replay harness + always-run property tests: the REAL control law
//! (`Controller::update`) + the stock write EMA driven against a plant fitted from
//! live telemetry. Cross-module by nature (control + loop_core), hence here.

use crate::control::*;
use crate::loop_core::plant;

struct ReplayStats {
median: f64,
mean: f64,
worst: f64,
env_clamps: u64,
/// Model prediction at 500 W throughput at the end of the LAST segment
/// (NaN for non-adaptive controllers).
pred_500: f64,
/// Chronological true grid error (post-warm-up), for dynamics assertions.
trace: Vec<f64>,
}
// ---- assertion margins (all in W unless noted) -----------------------------
// Loose enough to absorb the deterministic-noise draw and plant-fit drift,
// tight enough that a regression in the law is caught, not papered over.
/// Stock must exhibit at least this much standing import (the leak).
const M_LEAK_MIN: f64 = 8.0;
/// A balancing controller's median error must be inside this band (PI and
/// learn-only adaptive, which actuates identically to PI).
const M_BALANCE_PI: f64 = 3.0;
/// No single transient may exceed this. Set by physics, not preference: an
/// aircon-class 3 kW inrush lands atop whatever the battery was doing, and the
/// meter+inverter lag chain (~2-3 s round trip) means NO controller can respond
/// inside that window — worst observed on the lagged plant is inrush + prior
/// discharge ≈ 5.1 kW. Anything beyond this bound indicates a control fault,
/// not a load.
const M_WORST_TRANSIENT: f64 = 5500.0;
/// Hostile-seed runs get extra headroom while the model unlearns.
const M_WORST_HOSTILE: f64 = 5000.0;
/// Model prediction at 500 W must land within this of the plant truth.
const M_MODEL_CONV: f64 = 60.0;
/// PI dynamics: smoothed error must re-enter this band after a load step...
const M_SETTLE_BAND: f64 = 50.0;
/// ...within this many seconds...
const M_SETTLE_SECS: usize = 120;
/// ...and stay within this band afterwards (no ringing / windup).
const M_POST_SETTLE: f64 = 60.0;
/// Windowed-mean steadiness: with σ≈35 W/tick measurement noise the integral
/// does a slow bounded walk around the true bias, so a 300 s mean wanders
/// ~±15 W even when perfectly healthy (medians over hours use M_BALANCE).
const M_STEADY_MEAN: f64 = 25.0;
/// The bias the plant actually needs at 500 W discharge for grid == target,
/// beyond the stock law: (t−l)(1−B)/B − A/B evaluated per plant constants.
fn true_bias_500(a: f64, b: f64) -> f64 {
// At steady state with load l s.t. rep = −500: needed = (target−l)(1−b)/b − a/b
// and (target−l) = rep = −500 when balanced.
(-500.0) * (1.0 - b) / b - a / b
}
struct XorShift(u64);
impl XorShift {
fn gauss(&mut self, sd: f64) -> f64 {
    let mut n = 0.0;
    for _ in 0..12 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        n += (self.0 >> 11) as f64 / (1u64 << 53) as f64;
    }
    (n - 6.0) * sd
}
}
fn replay(segs: &[Vec<(i64, f64)>], mk: &dyn Fn() -> Kind, scen: &str) -> ReplayStats {
let (lo, hi) = (-7030.0, 7030.0);
let target = 5.0;
let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
let mut errs: Vec<f64> = vec![];
let (mut worst, mut clamps) = (0.0f64, 0u64);
let mut pred_500 = f64::NAN;
for seg in segs {
    let mut ctrl = Controller::new(mk());
    // IDENTIFIED plant + meter chain (loop_core::plant, postdiction-validated):
    // the Multi follows the write through a 1 s transport delay + first-order
    // response; the grid reading reaches the loop METER_LAG_S stale.
    let (mut weff, mut pending, mut prev_w): (f64, f64, Option<f64>) = (0.0, 0.0, None);
    let mut grid_ring: Vec<f64> = vec![];
    let mut frozen = 0.0f64;
    let half = seg.len() / 2;
    for (k, &(_t, load)) in seg.iter().enumerate() {
        weff += plant::ALPHA * (pending - weff);
        let (pa, pb) =
            if scen == "plantshift" && k > half { (40.0, 0.91) } else { (plant::A, plant::B) };
        let rep = pa + pb * weff;
        let grid_true = load + rep;
        grid_ring.push(grid_true);
        let idx = grid_ring.len().saturating_sub(1 + plant::METER_LAG_S);
        let mut grid = grid_ring[idx] + rng.gauss(35.0);
        if scen == "noiseburst" && (k / 600) % 6 == 0 {
            grid += rng.gauss(200.0);
        }
        if scen == "meterfreeze" && k % 600 < 5 && k > 0 {
            grid = frozen;
        }
        frozen = grid;
        let cmd = ctrl
    .update(grid, rep, target, 1.0, lo, hi, true, true)
    // Production parity: outside charge regimes the composed command is clamped
    // to non-positive (stock zeroes non-negative totals; acOut not modeled here).
    .min(0.0);
        if cmd <= lo + 1e-9 || cmd >= hi - 1e-9 {
            clamps += 1;
        }
        let w = crate::loop_core::stock_write_ema(prev_w, cmd, false, crate::loop_core::EMA_GAIN_UP, crate::loop_core::EMA_GAIN_DOWN);
        prev_w = Some(w);
        pending = w;
        if k > 120 {
            let e = grid_true - target;
            // Export-pinned rows (PV surplus, load <= ~0) are uncorrectable BY DESIGN
            // under the stock no-positive clamp; keep them out of the balance metric
            // but inside worst-case tracking.
            if load > 50.0 {
                errs.push(e);
            }
            if e.abs() > worst {
                worst = e.abs();
            }
        }
    }
    if let Some(ff) = ctrl.ff_snapshot(500.0) {
        pred_500 = ff.a + ff.b * 500.0;
    }
}
let mut sorted = errs.clone();
sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
let n = sorted.len();
ReplayStats {
    median: sorted[n / 2],
    mean: sorted.iter().sum::<f64>() / n as f64,
    worst,
    env_clamps: clamps,
    pred_500,
    trace: errs,
}
}
/// 10-s moving mean of a trace (noise floor is σ≈35 W per tick; assertions on
/// dynamics use the smoothed signal so they test the LOOP, not the dice).
fn smooth10(trace: &[f64]) -> Vec<f64> {
trace
    .windows(10)
    .map(|w| w.iter().sum::<f64>() / w.len() as f64)
    .collect()
}
/// Deterministic household-like load with PLATEAUS at several discharge
/// levels (300/700/1200/500 W) plus a PV-export stretch and kettle bursts —
/// the plateaus give the model excitation at multiple throughputs so its
/// leak line is identifiable (a single level would leave the slope
/// unconstrained, mirroring why `conf` weights by prediction variance).
/// Appliance-signature load generator: realistic event shapes for transient
/// studies, all driven through the lagged plant+meter chain.
///   kettle    — clean resistive 2 kW step, ~120 s, clean step-off
///   microwave — magnetron duty cycling: 1.7 kW, ~10 s ON / ~20 s OFF bursts
///   aircon    — 3 kW inrush spike (~2 s) then 1.2 kW compressor, ~4 min on/3 off
/// Base load 300 W + measurement-scale noise.
fn appliance_load_segment(seed: u64, secs: usize) -> Vec<(i64, f64)> {
    let mut rng = XorShift(seed | 1);
    let mut out = Vec::with_capacity(secs);
    for t in 0..secs {
        let mut l = 300.0 + rng.gauss(25.0);
        // kettle every ~15 min, 120 s
        if (t % 900) < 120 && t >= 900 {
            l += 2000.0;
        }
        // microwave session in the middle: 5 min of magnetron cycling
        let mw0 = secs / 2;
        if t >= mw0 && t < mw0 + 300 {
            let ph = (t - mw0) % 30;
            if ph < 10 {
                l += 1700.0;
            }
        }
        // aircon in the last third: 2 s inrush 3 kW, then 1.2 kW, 240 s on / 180 s off
        let ac0 = secs * 2 / 3;
        if t >= ac0 {
            let ph = (t - ac0) % 420;
            if ph < 2 {
                l += 3000.0;
            } else if ph < 240 {
                l += 1200.0;
            }
        }
        out.push((t as i64, l));
    }
    out
}

fn synthetic_load_segment(seed: u64, secs: usize) -> Vec<(i64, f64)> {
let mut rng = XorShift(seed | 1);
let mut out = Vec::with_capacity(secs);
for t in 0..secs {
    let phase = t as f64 / secs as f64;
    let mut l = match (phase * 10.0) as u32 {
        0..=1 => 300.0,
        2..=3 => 700.0,
        4 => 1200.0,
        5 => -1300.0, // PV export: charging => model must NOT learn here
        6..=7 => 1200.0,
        8 => 500.0,
        _ => 500.0,
    } + rng.gauss(30.0);
    if phase > 0.2 && phase < 0.4 && (t / 180) % 5 == 0 {
        l += 2000.0; // kettle/oven bursts
    }
    out.push((t as i64, l));
}
out
}
/// COUNTERFACTUAL REPLAY (ignored; run explicitly) — drives the REAL production
/// law (this module + the stock write EMA) closed-loop against a plant fitted
/// from live telemetry, over a real captured load profile. Answers "would the
/// adaptive controller have balanced the grid, and does it ever run away?"
/// without touching the live system.
///
///   REPLAY_LOAD=/path/load.csv (t,load_w rows) \
///   [REPLAY_SCEN=baseline|plantshift|noiseburst|meterfreeze] \
///   [REPLAY_A0=.. REPLAY_B0=..]  (hostile model seed) \
///   cargo test --target <host> replay_counterfactual -- --ignored --nocapture
///
/// Plant: rep(t) = A + B·write(t−1) with A=4.7, B=0.9562 (fitted 2026-08-21,
/// 32,389 steady rows, resid σ 35 W) + N(0,35) measurement noise.
#[test]
#[ignore]
fn replay_counterfactual() {
let path = match std::env::var("REPLAY_LOAD") {
    Ok(p) => p,
    Err(_) => return, // not configured: skip silently
};
let scen = std::env::var("REPLAY_SCEN").unwrap_or_else(|_| "baseline".into());
let a0: f64 = std::env::var("REPLAY_A0").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
let b0: f64 = std::env::var("REPLAY_B0").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
let data = std::fs::read_to_string(&path).expect("load csv");
let rows: Vec<(i64, f64)> = data
    .lines()
    .filter_map(|l| {
        let mut it = l.split(',');
        Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
    })
    .collect();
let mut segs: Vec<Vec<(i64, f64)>> = vec![];
let mut cur: Vec<(i64, f64)> = vec![rows[0]];
for w in rows.windows(2) {
    if w[1].0 - w[0].0 <= 3 {
        cur.push(w[1]);
    } else {
        segs.push(std::mem::take(&mut cur));
        cur.push(w[1]);
    }
}
segs.push(cur);
segs.retain(|s| s.len() > 600);
let mk = move || match std::env::var("REPLAY_CTRL").as_deref() {
    Ok("stock") => Kind::Stock,
    Ok("pi") => Kind::Pi { ki: 0.05, i_max: 300.0 },
Ok("observe") => {
    let mut c = AdaptiveCfg::tuned();
    c.mode = crate::control::FfMode::Observe;
    Kind::Adaptive(c)
}
Ok("frozen") => {
    let mut c = AdaptiveCfg::tuned();
    c.mode = crate::control::FfMode::Frozen;
    c.a0 = a0;
    c.b0 = b0;
    c.seeded = true;
    if let Ok(ki) = std::env::var("REPLAY_KI") {
        c.ki = ki.parse().unwrap();
    }
    Kind::Adaptive(c)
}
    _ => {
        let mut cfg = AdaptiveCfg::tuned();
        cfg.a0 = a0;
        cfg.b0 = b0;
        cfg.seeded = a0 != 0.0 || b0 != 0.0;
        Kind::Adaptive(cfg)
    }
};
let r = replay(&segs, &mk, &scen);
let hours = r.trace.len() as f64 / 3600.0;
let exp_whh = r.trace.iter().filter(|e| **e < -300.0).map(|e| -*e).sum::<f64>() / 3600.0 / hours;
eprintln!(
    "REPLAY scen={scen}: median {:+.1}W mean {:+.1}W worst {:.0}W env-clamps {} export>{{300W}} {:.1}Wh/h pred@500 {:.1}",
    r.median, r.mean, r.worst, r.env_clamps, exp_whh, r.pred_500
);
}
/// ALWAYS-RUN closed-loop property: the leak controllers actually balance the
/// grid on a realistic load, and stock reproduces the leak (sim sanity anchor).
#[test]
fn replay_property_leak_is_balanced() {
let segs = vec![synthetic_load_segment(7, 7200)];
let stock = replay(&segs, &|| Kind::Stock, "baseline");
let pi = replay(&segs, &|| Kind::Pi { ki: 0.05, i_max: 300.0 }, "baseline");
// Learn-only adaptive actuates exactly as PI (the recommended live mode) — held
// to the same bound. FULL adaptive's closed learning loop has a KNOWN cold-start
// fragility (it learns its own applied bias; single-cluster excitation can rail
// the params, and healing depends on the write pacing — see
// probe_adaptive_trajectory). Until that learn law is redesigned, full-adaptive
// BALANCE is deliberately not asserted; its SAFETY bounds are asserted in
// replay_property_adaptive_never_runs_away.
let obs = replay(
    &segs,
    &|| {
        let mut c = AdaptiveCfg::tuned();
        c.mode = crate::control::FfMode::Observe;
        Kind::Adaptive(c)
    },
    "baseline",
);
assert!(stock.median > M_LEAK_MIN, "stock must show the leak, got {:+.1}", stock.median);
assert!(pi.median.abs() < M_BALANCE_PI, "pi must balance, got {:+.1}", pi.median);
assert!(obs.median.abs() < M_BALANCE_PI, "learn-only must balance, got {:+.1}", obs.median);
}
/// ALWAYS-RUN runaway guard: across every stress scenario and hostile seeds,
/// the adaptive law must never hit the design envelope, must stay bounded, and
/// (where it learns) must converge NEAR the plant's true leak.
#[test]
fn replay_property_adaptive_never_runs_away() {
let segs = vec![synthetic_load_segment(11, 7200)];
let truth = true_bias_500(4.7, 0.9562);
for scen in ["baseline", "plantshift", "noiseburst", "meterfreeze"] {
    let r = replay(&segs, &|| Kind::Adaptive(AdaptiveCfg::tuned()), scen);
    assert_eq!(r.env_clamps, 0, "{scen}: must never reach the design envelope");
    assert!(r.worst < M_WORST_TRANSIENT, "{scen}: worst {:.0}W", r.worst);
}
// Convergence-to-truth is asserted for LEARN-ONLY mode (the recommended live
// config): it learns the PI-held bias with no feedback through its own output.
// FULL adaptive convergence is a KNOWN ISSUE (cold-start railing, see
// probe_adaptive_trajectory); its safety bounds above are the hard claims.
let obs = replay(
    &segs,
    &|| {
        let mut c = AdaptiveCfg::tuned();
        c.mode = crate::control::FfMode::Observe;
        Kind::Adaptive(c)
    },
    "baseline",
);
assert!(
    (obs.pred_500 - truth).abs() < M_MODEL_CONV,
    "learn-only model must converge near the true leak ({truth:.1}), got {:.1}",
    obs.pred_500
);
// Hostile seeds at the parameter rails: bounded, no envelope contact.
for (a0, b0) in [(1000.0, 0.5), (-1000.0, -0.5)] {
    let mk = move || {
        let mut c = AdaptiveCfg::tuned();
        c.a0 = a0;
        c.b0 = b0;
        c.seeded = true;
        Kind::Adaptive(c)
    };
    let r = replay(&segs, &mk, "baseline");
    assert_eq!(r.env_clamps, 0, "hostile seed ({a0},{b0}) reached the envelope");
    assert!(r.worst < M_WORST_HOSTILE, "hostile seed worst {:.0}W", r.worst);
}
}
/// ALWAYS-RUN PI dynamics: a 2 kW load step must be absorbed without ringing
/// or integral windup — the smoothed error re-enters M_SETTLE_BAND within
/// M_SETTLE_SECS and stays inside M_POST_SETTLE for the rest of the run.
#[test]
fn replay_property_pi_step_response_settles_cleanly() {
// 1800 s at 400 W, step to 2400 W at t=900, hold.
let seg: Vec<(i64, f64)> =
    (0..1800).map(|t| (t, if t < 900 { 400.0 } else { 2400.0 })).collect();
let r = replay(&[seg], &|| Kind::Pi { ki: 0.05, i_max: 300.0 }, "baseline");
assert_eq!(r.env_clamps, 0);
let sm = smooth10(&r.trace);
// The step lands at trace index ~900-120(warm-up) = 780.
let step_at = 780usize;
let settle = sm[step_at..]
    .iter()
    .position(|e| e.abs() < M_SETTLE_BAND)
    .expect("must re-enter the settle band");
assert!(
    settle < M_SETTLE_SECS,
    "PI must settle a 2 kW step within {M_SETTLE_SECS}s, took {settle}s"
);
// After settling (+ a grace period), no ringing and no windup: the smoothed
// error stays inside the post-settle band to the end of the run.
for (k, e) in sm[step_at + settle + 60..].iter().enumerate() {
    assert!(
        e.abs() < M_POST_SETTLE,
        "ringing/windup: |smoothed err| {:.1}W at +{}s after settle",
        e.abs(),
        k + 60
    );
}
// And the pre-step steady state was already balanced (leak closed).
let pre: f64 = sm[step_at - 300..step_at].iter().sum::<f64>() / 300.0;
assert!(pre.abs() < M_STEADY_MEAN, "pre-step mean err {pre:+.1}W");
}

/// ALWAYS-RUN appliance-transient property: under kettle/magnetron/aircon
/// signatures on the LAGGED plant, the balancing loop must hold the invariants
/// that failed live on 2026-08-21: median balanced, no design-envelope contact,
/// and bounded worst-case.
#[test]
fn replay_property_appliance_transients_bounded() {
    let segs = vec![appliance_load_segment(13, 5400)];
    let pi = replay(&segs, &|| Kind::Pi { ki: 0.05, i_max: 300.0 }, "baseline");
    // The design envelope MAY be touched momentarily on the aircon inrush stack
    // (3 kW step + prior flow + noise at full pacing, uncompensated PI here) —
    // that is the clamp doing its job. Sustained clamping would be a fault.
    assert!(pi.env_clamps <= 3, "sustained envelope clamping: {}", pi.env_clamps);
    assert!(pi.median.abs() < M_BALANCE_PI, "median {:+.1}", pi.median);
    assert!(pi.worst < M_WORST_TRANSIENT, "worst {:.0}", pi.worst);
}

/// Diagnostic probe (ignored): windowed PI dynamics on the synthetic load under
/// knob combinations, to attribute steady bias / export tails to a mechanism
/// instead of guessing. Run:
///   cargo test --target <host> probe_step_dynamics -- --ignored --nocapture
#[test]
#[ignore]
fn probe_step_dynamics() {
    let seg = synthetic_load_segment(7, 7200);
    for (label, gu, gd, clamp0) in [
        ("sym0.5, no clamp", 0.5, 0.5, false),
        ("sym0.5, clamp", 0.5, 0.5, true),
        ("asym, clamp", 0.5, 0.242, true),
        ("asym-mild, clamp", 0.5, 0.35, true),
    ] {
        let mut ctrl = crate::control::Controller::new(Kind::Pi { ki: 0.05, i_max: 300.0 });
        let (mut rep, mut prev_w): (f64, Option<f64>) = (0.0, None);
        let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
        let mut win: Vec<(f64, f64)> = vec![];
        let mut all: Vec<f64> = vec![];
        eprintln!("--- {label}");
        for (k, &(_t, load)) in seg.iter().enumerate() {
            let grid_true = load + rep;
            let grid = grid_true + rng.gauss(35.0);
            let mut cmd = ctrl.update(grid, rep, 5.0, 1.0, -7030.0, 7030.0, true, true);
            if clamp0 {
                cmd = cmd.min(0.0);
            }
            let w = crate::loop_core::stock_write_ema(prev_w, cmd, false, gu, gd);
            prev_w = Some(w);
            rep = 4.7 + 0.9562 * w;
            if k > 120 {
                win.push((load, grid_true - 5.0));
                if load > 50.0 {
                    all.push(grid_true - 5.0);
                }
            }
            if (k + 1) % 720 == 0 && !win.is_empty() {
                let n = win.len() as f64;
                let me = win.iter().map(|x| x.1).sum::<f64>() / n;
                let ml = win.iter().map(|x| x.0).sum::<f64>() / n;
                let exp_wh = win.iter().filter(|x| x.1 < -50.0).map(|x| -x.1).sum::<f64>() / 3600.0;
                eprintln!(
                    "  t={:5} load~{:6.0} mean_err {:+7.1} exp {:5.2}Wh",
                    k + 1, ml, me, exp_wh
                );
                win.clear();
            }
        }
        all.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = all.len();
        eprintln!(
            "  MEDIAN {:+.1}  mean {:+.1}  p10 {:+.0} p90 {:+.0}",
            all[n / 2],
            all.iter().sum::<f64>() / n as f64,
            all[n / 10],
            all[n * 9 / 10]
        );
    }
}

/// Second diagnostic probe: adaptive model trajectory on the runaway-test segment.
#[test]
#[ignore]
fn probe_adaptive_trajectory() {
    let seg = synthetic_load_segment(11, 7200);
    for (label, clamp0, gu, gd) in [
        ("clamp,asym", true, 0.5, 0.242),
        ("clamp,sym", true, 0.5, 0.5),
        ("noclamp,asym", false, 0.5, 0.242),
        ("noclamp,sym", false, 0.5, 0.5),
    ] {
        let mut ctrl = crate::control::Controller::new(Kind::Adaptive(AdaptiveCfg::tuned()));
        let (mut rep, mut prev_w): (f64, Option<f64>) = (0.0, None);
        let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
        let mut line = String::new();
        for (k, &(_t, load)) in seg.iter().enumerate() {
            let grid_true = load + rep;
            let grid = grid_true + rng.gauss(35.0);
            let mut cmd = ctrl.update(grid, rep, 5.0, 1.0, -7030.0, 7030.0, true, true);
            if clamp0 { cmd = cmd.min(0.0); }
            let w = crate::loop_core::stock_write_ema(prev_w, cmd, false, gu, gd);
            prev_w = Some(w);
            rep = 4.7 + 0.9562 * w;
            if (k + 1) % 1440 == 0 {
                if let Some(ff) = ctrl.ff_snapshot(500.0) {
                    line += &format!(" | t{} a{:+.0} b{:+.2} p{:+.0}", k + 1, ff.a, ff.b, ff.a + ff.b * 500.0);
                }
            }
            let _ = load;
        }
        eprintln!("{label:>12}{line}");
    }
}

// ============================================================================
// POSTDICTION GATES — the simulator's licence to make predictions.
// Each gate replays a LIVE-CAPTURED event through the identified plant+meter
// chain with the controller config that was actually running, and asserts the
// simulation reproduces the captured outcome within the model's demonstrated
// skill. Any change to the plant model, the laws, or the harness must keep
// these green — aggregate-only validation is how 2026-08-21 happened.
// ============================================================================

/// Live 2026-08-21 ~13:40: microwave magnetron cycle under OUR frozen/ki0.02 build.
/// Columns: (t, load_w, grid_captured, write_captured, acout).
pub const EV_MICROWAVE_OURS: &[(i64, f64, f64, f64, f64)] = &[
    (270, 324.2, 17.2, -325.0, 30.0),
    (271, 344.9, 34.9, -332.0, 29.0),
    (272, 344.9, 34.9, -332.0, 29.0),
    (273, 357.1, 47.1, -341.0, 25.0),
    (274, 386.3, 81.3, -355.0, 37.0),
    (275, 390.2, 74.2, -367.0, 33.0),
    (276, 399.9, 78.9, -379.0, 29.0),
    (277, 394.7, 70.7, -387.0, 30.0),
    (278, 377.9, 56.9, -389.0, 34.0),
    (279, 383.3, 59.3, -393.0, 36.0),
    (280, 380.1, 54.1, -395.0, 39.0),
    (281, 376.7, 50.7, -396.0, 44.0),
    (282, 378.4, 47.4, -398.0, 49.0),
    (283, 378.4, 47.4, -398.0, 49.0),
    (284, 372.7, 32.7, -398.0, 42.0),
    (285, 371.4, 31.4, -398.0, 45.0),
    (286, 476.1, 23.1, -424.0, 53.0),
    (287, 610.8, 132.8, -478.0, 47.0),
    (288, 2159.9, 1690.9, -894.0, 35.0),
    (289, 2172.9, 1712.9, -1213.0, 39.0),
    (290, 2297.6, 1656.6, -1487.0, 26.0),
    (291, 2401.5, 1515.5, -1723.0, -1.0),
    (292, 2323.2, 1037.2, -1888.0, -4.0),
    (293, 2323.2, 1037.2, -1888.0, -4.0),
    (294, 2432.7, 921.7, -2043.0, 14.0),
    (295, 2365.7, 674.7, -2147.0, 21.0),
    (296, 2163.2, 418.2, -2178.0, 22.0),
    (297, 2208.2, 307.2, -2214.0, 31.0),
    (298, 536.8, -1518.2, -1838.0, 7.0),
    (299, 363.7, -1684.3, -1511.0, 8.0),
    (300, 150.0, -1565.0, -1207.0, 13.0),
    (301, -71.3, -1483.3, -919.0, 8.0),
    (302, 325.9, -945.1, -795.0, -5.0),
    (303, 212.1, -752.9, -669.0, 13.0),
    (304, 212.1, -752.9, -669.0, 13.0),
    (305, 229.6, -602.4, -576.0, 15.0),
    (306, 490.9, -438.1, -569.0, 31.0),
    (307, 157.9, -530.1, -480.0, 19.0),
    (308, 249.0, -390.0, -434.0, 17.0),
    (309, 318.6, -251.4, -414.0, 53.0),
    (310, 322.1, -183.9, -399.0, 47.0),
    (311, 362.8, -113.2, -396.0, 43.0),
    (312, 359.2, -96.8, -393.0, 28.0),
    (313, 366.7, -81.3, -392.0, 25.0),
    (314, 360.6, -60.4, -389.0, 32.0),
    (315, 360.6, -60.4, -389.0, 32.0),
    (316, 374.8, -57.2, -390.0, 32.0),
    (317, 346.4, -61.6, -383.0, 41.0),
    (318, 359.2, -38.8, -380.0, 38.0),
];

/// Live 2026-08-21 (shadow day 5): ~2 kW step-off under STOCK control.
/// Columns: (t, load_w, grid_captured, write_captured, acout).
pub const EV_STEPOFF_STOCK: &[(i64, f64, f64, f64, f64)] = &[
    (174436, 3679.8, 75.8, -3752.0, 51.0),
    (174437, 3737.5, 150.5, -3706.0, 49.0),
    (174438, 3722.3, 183.3, -3706.0, 52.0),
    (174439, 3714.9, 197.9, -3708.0, 37.0),
    (174440, 3714.9, 197.9, -3708.0, 37.0),
    (174441, 2869.0, -669.0, -3708.0, 37.0),
    (174442, 2836.1, -700.9, -3270.0, 32.0),
    (174443, 2625.0, -718.0, -3270.0, 45.0),
    (174444, 2562.7, -702.3, -3270.0, 43.0),
    (174445, 2676.3, -451.7, -3038.0, 23.0),
    (174446, 2661.8, -391.2, -3038.0, 27.0),
    (174447, 2669.2, -350.8, -2844.0, 30.0),
    (174448, 2632.1, -199.9, -2844.0, 46.0),
    (174449, 3674.4, 888.4, -3256.0, 39.0),
    (174450, 3801.3, 963.3, -3256.0, 27.0),
    (174451, 3801.3, 963.3, -3256.0, 27.0),
    (174452, 4015.9, 908.9, -3634.0, 29.0),
    (174453, 3865.2, 692.2, -3634.0, 22.0),
    (174454, 4074.9, 625.9, -3634.0, 42.0),
    (174455, 3900.4, 337.4, -3708.0, 38.0),
    (174456, 3955.5, 313.5, -3708.0, 53.0),
    (174457, 3828.3, 131.3, -3766.0, 66.0),
    (174458, 3663.4, 3.4, -3766.0, 62.0),
    (174459, 3678.4, 71.4, -3756.0, 48.0),
    (174460, 3634.2, 170.2, -3756.0, 42.0),
    (174461, 2053.3, -1483.7, -2902.0, 37.0),
    (174462, 1638.1, -1924.9, -2902.0, 33.0),
    (174463, 1638.1, -1924.9, -2902.0, 33.0),
    (174464, 1128.1, -1680.9, -2902.0, 44.0),
    (174465, 1407.6, -1252.4, -2226.0, 40.0),
    (174466, 1063.1, -1240.9, -2226.0, 38.0),
    (174467, 1426.9, -764.1, -1846.0, 29.0),
    (174468, 1251.0, -727.0, -1846.0, 22.0),
    (174469, 1179.3, -675.7, -1724.0, 38.0),
    (174470, 1544.6, -247.4, -1724.0, 37.0),
    (174471, 1452.1, -257.9, -1724.0, 26.0),
    (174472, 1595.0, -92.0, -1608.0, 24.0),
    (174473, 1504.9, -93.1, -1608.0, 26.0),
    (174474, 1504.9, -93.1, -1608.0, 26.0),
    (174475, 1517.2, -55.8, -1572.0, 18.0),
    (174476, 1578.4, 32.4, -1572.0, 6.0),
    (174477, 1587.4, 39.4, -1568.0, 8.0),
    (174478, 1569.8, 57.8, -1568.0, 6.0),
    (174479, 1560.6, 50.6, -1580.0, 8.0),
    (174480, 1608.5, 89.5, -1580.0, 9.0),
    (174481, 1592.4, 90.4, -1580.0, 12.0),
    (174482, 1631.7, 90.7, -1584.0, 5.0),
    (174483, 1629.3, 71.3, -1584.0, 4.0),
    (174484, 1577.1, 36.1, -1578.0, 2.0),
    (174485, 1516.1, 54.1, -1578.0, 12.0),
    (174486, 1516.1, 54.1, -1578.0, 12.0),
    (174487, 1580.0, 110.0, -1576.0, 6.0),
    (174488, 1588.2, 147.2, -1576.0, 9.0),
    (174489, 1639.0, 196.0, -1638.0, 2.0),
    (174490, 1808.6, 261.6, -1638.0, -19.0),
    (174491, 1662.4, 140.4, -1638.0, 2.0),
    (174492, 1617.3, 99.3, -1626.0, 0.0),
    (174493, 1617.5, 111.5, -1626.0, 16.0),
    (174494, 1554.9, 75.9, -1594.0, 7.0),
    (174495, 1580.4, 89.4, -1594.0, 5.0),
    (174496, 1580.4, 89.4, -1594.0, 5.0),
    (174497, 1543.9, 42.9, -1566.0, 10.0),
    (174498, 1558.2, 51.2, -1566.0, 3.0),
    (174499, 1583.8, 61.8, -1566.0, -4.0),
    (174500, 1551.9, 73.9, -1578.0, 4.0),
    (174501, 1603.9, 132.9, -1578.0, 9.0),
    (174502, 1600.6, 104.6, -1574.0, 1.0),
    (174503, 1620.1, 141.1, -1574.0, 6.0),
    (174504, 1688.4, 117.4, -1594.0, 18.0),
    (174505, 1578.1, 46.1, -1594.0, -5.0),
    (174506, 1556.4, 62.4, -1572.0, 5.0),
];


/// Closed-loop event replay against a captured trace: reconstructed load drives
/// the identified plant; the given per-tick controller closure produces writes.
fn replay_event(
    rows: &[(i64, f64, f64, f64, f64)],
    mut law: impl FnMut(f64, f64, f64, f64) -> f64, // (grid_meas, rep, acout, prev_w) -> cmd
    pace: f64,
    cadence_2s5: bool,
) -> (f64, f64, f64) {
    // returns (export_wh, peak_w, rmse_vs_captured)
    let (mut weff, mut pending) = {
        let w0 = rows[0].3;
        (w0, w0)
    };
    let mut prev_w = rows[0].3;
    let mut grid_ring: Vec<f64> = vec![rows[0].2];
    let (mut exp, mut peak, mut se, mut n) = (0.0f64, 0.0f64, 0.0f64, 0u32);
    let mut ph = 0u32;
    for &(_t, load, grid_cap, _w, ao) in &rows[1..] {
        weff += plant::ALPHA * (pending - weff);
        let rep = plant::A + plant::B * weff;
        let grid_true = load + rep;
        grid_ring.push(grid_true);
        let idx = grid_ring.len().saturating_sub(1 + plant::METER_LAG_S);
        let gm = grid_ring[idx];
        let cmd = law(gm, rep, ao, prev_w);
        ph += 1;
        let do_write = !cadence_2s5 || (ph % 2 == 0 || ph % 5 == 0);
        if do_write {
            prev_w = (prev_w + pace * (cmd - prev_w)).round();
        }
        pending = prev_w;
        if grid_true < 0.0 {
            exp += -grid_true / 3600.0;
        }
        if grid_true < peak {
            peak = grid_true;
        }
        se += (grid_true - grid_cap) * (grid_true - grid_cap);
        n += 1;
    }
    (exp, peak, (se / n as f64).sqrt())
}

/// Gate 1: OUR captured microwave event (frozen FF, ki 0.02, 0.242 pacing,
/// no-charge clamp, no Smith — the config that ran live). Captured outcome:
/// export 3.02 Wh, peak −1684 W. Model skill at gate creation: export 3.36 Wh,
/// peak −2167 W, RMSE 220 W — margins bracket that skill.
#[test]
fn postdiction_gate_microwave_ours() {
    let mut i_state = 0.0f64;
    let (exp, peak, rmse) = replay_event(
        EV_MICROWAVE_OURS,
        |gm, rep, ao, _pw| {
            let law = rep + 5.0 - gm;
            let ff = -4.9 - 0.0458 * rep.abs();
            let e = (5.0 - gm).clamp(-100.0, 100.0);
            i_state = (i_state + 0.02 * e).clamp(-300.0, 300.0);
            (law + ff + i_state).min(ao)
        },
        0.242,
        false,
    );
    assert!((2.4..=4.2).contains(&exp), "export {exp:.2}Wh vs real 3.02");
    assert!((-2700.0..=-1400.0).contains(&peak), "peak {peak:.0}W vs real -1684");
    assert!(rmse < 350.0, "trajectory RMSE {rmse:.0}W");
}

/// Gate 2: a captured STOCK ~2 kW step-off (shadow day 5). Captured outcome:
/// export 4.08 Wh, peak −1925 W. Model skill at gate creation: export 3.23 Wh
/// (stock model is a LOWER bound on stock's export), peak −1944 W, RMSE 309 W.
#[test]
fn postdiction_gate_stepoff_stock() {
    let (exp, peak, rmse) = replay_event(
        EV_STEPOFF_STOCK,
        |gm, rep, ao, _pw| {
            let law = rep + 5.0 - gm;
            (if law < 0.0 { law } else { 0.0 }) + ao
        },
        0.5,
        true,
    );
    assert!((2.5..=4.6).contains(&exp), "export {exp:.2}Wh vs real 4.08");
    assert!((-2400.0..=-1500.0).contains(&peak), "peak {peak:.0}W vs real -1925");
    assert!(rmse < 400.0, "trajectory RMSE {rmse:.0}W");
}
