//! Control laws for the mode-3 AC-in power loop.
//!
//! Ported from the validated `ess_lab` Python sandbox.  Each controller outputs
//! the Multi command directly (mode 3): negative = discharge, positive = charge.
//!
//!   grid     : measured grid power (W, +import)
//!   reported : Multi's reported AC-in power (W, neg=discharge)
//!   target   : grid setpoint (W)
//!   dt       : seconds since previous tick
//!
//! The `Adaptive` controller adds a continuously-learned feed-forward model
//! `leak(throughput) ≈ a + b·|reported|` on top of the PI residual integral. The
//! model is taught BY the integral (it learns what steady bias each load needs) and
//! earns actuation authority only as its statistical confidence grows, so it can
//! never do harm while uncertain and degrades to plain PI when it has no data. All
//! of it lives in RAM (no persistence); the hard safety envelope is downstream in
//! the write phase and is unaffected by anything learned here.

pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    // Fail-safe: a non-finite command (NaN/inf from a bad sensor read) becomes idle
    // (0) — never written to the inverter as garbage.
    if !x.is_finite() {
        return 0.0;
    }
    // Defensive: if bounds ever invert (lo > hi), collapse to the safe tighter bound
    // so we never return a value outside an intended range.
    let hi = hi.max(lo);
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// How the adaptive feed-forward model is used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FfMode {
    /// Learn the model AND apply it (confidence-weighted). The continuous default.
    Apply,
    /// Learn-only: keep updating the model but actuate integral-only, so you can watch
    /// it converge (via the periodic `MODEL:` log) before trusting it. Zero feed-forward
    /// output effect. NOTE: learning still requires ACTUATION (takeover + owning) — the
    /// integral must really be driving the plant for its value to mean anything. In
    /// shadow/trim the model stays null by design (see the `learn` gate in `update`).
    Observe,
    /// Do not learn: apply the seeded `(a, b)` as a fixed feed-forward. For pinning a
    /// known-good model or reproducible A/B runs.
    Frozen,
}

/// Static configuration for the `Adaptive` controller. All `Copy` scalars so `Kind`
/// stays `Copy`; the mutable learning state lives in `Controller`.
#[derive(Clone, Copy, Debug)]
pub struct AdaptiveCfg {
    pub ki: f64,
    pub i_max: f64,
    /// Seed offset (W) and slope (W per W of throughput). `(0, 0)` => cold start.
    pub a0: f64,
    pub b0: f64,
    /// True when the caller supplied a seed => start with tighter covariance (more
    /// initial trust) around `(a0, b0)`.
    pub seeded: bool,
    /// RLS forgetting factor (0 < λ ≤ 1). Effective memory ≈ `1/(1-λ)` clean samples.
    pub lambda: f64,
    /// Prediction-σ (W) at which the model is given 50 % authority. Smaller => stricter.
    pub conf_scale_w: f64,
    /// Learn only when |grid error| and the tick-to-tick throughput change are both
    /// within this band (W) — excludes load transients, which would poison the fit.
    pub steady_band_w: f64,
    pub mode: FfMode,
}

impl AdaptiveCfg {
    /// Sensible defaults; the integral part mirrors `Pi`.
    pub fn tuned() -> Self {
        AdaptiveCfg {
            ki: 0.05,
            i_max: 300.0,
            a0: 0.0,
            b0: 0.0,
            seeded: false,
            lambda: 0.999, // ~1000-sample memory
            conf_scale_w: 20.0,
            steady_band_w: 25.0,
            mode: FfMode::Apply,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Kind {
    /// Exact reproduction of the stock ESS loop: `command = target - grid + reported`.
    /// No integral -> leaks (1-k)*load.  Used to prove shadow fidelity.
    Stock,
    /// One-line fix: integrate on our OWN previous command, not `reported`,
    /// so the firmware deficit is inside the loop.  Drives grid -> target.
    TrueIntegral { ki: f64 },
    /// Feed-forward stock law + slow integral trim on the residual.
    /// Best steady + transient behaviour.
    Pi { ki: f64, i_max: f64 },
    /// PI plus a continuously-learned load-indexed feed-forward (see module docs).
    Adaptive(AdaptiveCfg),
}

/// In-memory recursive-least-squares fit of `y ≈ a + b·x` with a forgetting factor.
/// Carries the 2×2 parameter covariance so callers can read a confidence for the
/// current prediction. Everything is kept finite and bounded defensively.
#[derive(Clone, Copy, Debug)]
struct RlsModel {
    a: f64,
    b: f64,
    // Symmetric covariance [[p00, p01], [p01, p11]].
    p00: f64,
    p01: f64,
    p11: f64,
    // Upper cap on p00/p11 (= prior variances): covariance never inflates past ignorance.
    p00_cap: f64,
    p11_cap: f64,
    lambda: f64,
}

// Defensive bounds so a pathological data run can never push the model somewhere the
// downstream clamps would then have to absorb. Generous vs any real ESS leak.
const A_ABS_MAX: f64 = 1000.0; // offset within ±1 kW
const B_ABS_MAX: f64 = 0.5; // slope within ±50 % of throughput

// Prior parameter std-dev before any data. A WIDE prior (unseeded) makes early
// predictions low-confidence, so the model contributes ~nothing until it has learned;
// a SEED starts tighter so the supplied numbers are trusted from tick one. Their sum
// also serves as the covariance TRACE cap: total covariance magnitude is bounded at
// the prior trace (an individual element may transiently exceed its own prior within
// that total), which bounds the RLS gain and prevents low-excitation overflow.
const PRIOR_A_SD_WIDE: f64 = 500.0; // W
const PRIOR_B_SD_WIDE: f64 = 0.5; // fraction of throughput
const PRIOR_A_SD_SEED: f64 = 50.0;
const PRIOR_B_SD_SEED: f64 = 0.05;

impl RlsModel {
    fn new(a0: f64, b0: f64, seeded: bool, lambda: f64) -> Self {
        let (sa, sb) = if seeded {
            (PRIOR_A_SD_SEED, PRIOR_B_SD_SEED)
        } else {
            (PRIOR_A_SD_WIDE, PRIOR_B_SD_WIDE)
        };
        let (va, vb) = (sa * sa, sb * sb);
        RlsModel {
            a: a0,
            b: b0,
            p00: va,
            p01: 0.0,
            p11: vb,
            p00_cap: va,
            p11_cap: vb,
            lambda: lambda.clamp(0.90, 1.0),
        }
    }

    fn predict(&self, x: f64) -> f64 {
        self.a + self.b * x
    }

    /// Variance of the mean prediction at `x`: φᵀ P φ with φ = [1, x]. Parameter
    /// uncertainty only (not measurement noise) — i.e. how well-pinned the model is.
    /// A non-finite `x` is "unknown" => infinite variance => zero confidence.
    fn pred_var(&self, x: f64) -> f64 {
        if !x.is_finite() {
            return f64::INFINITY;
        }
        let pphi0 = self.p00 + self.p01 * x;
        let pphi1 = self.p01 + self.p11 * x;
        (pphi0 + pphi1 * x).max(0.0)
    }

    /// One RLS step toward target `y` at regressor `x`. No-op on non-finite inputs.
    fn update(&mut self, x: f64, y: f64) {
        if !x.is_finite() || !y.is_finite() {
            return;
        }
        let pphi0 = self.p00 + self.p01 * x;
        let pphi1 = self.p01 + self.p11 * x;
        let denom = self.lambda + (pphi0 + pphi1 * x);
        if !(denom > 0.0) || !denom.is_finite() {
            return; // numerically degenerate — skip rather than blow up
        }
        let k0 = pphi0 / denom;
        let k1 = pphi1 / denom;
        let err = y - self.predict(x);
        self.a += k0 * err;
        self.b += k1 * err;
        // P = (P - k φᵀ P) / λ, keeping symmetry (k0*pphi1 == k1*pphi0 by construction).
        self.p00 = (self.p00 - k0 * pphi0) / self.lambda;
        self.p01 = (self.p01 - k0 * pphi1) / self.lambda;
        self.p11 = (self.p11 - k1 * pphi1) / self.lambda;
        // Sanitize params: `f64::clamp` returns NaN for a NaN input, so a divergent step
        // must be reset explicitly, THEN rail-clamped.
        if !self.a.is_finite() {
            self.a = 0.0;
        }
        if !self.b.is_finite() {
            self.b = 0.0;
        }
        self.a = self.a.clamp(-A_ABS_MAX, A_ABS_MAX);
        self.b = self.b.clamp(-B_ABS_MAX, B_ABS_MAX);
        // Covariance hygiene: reset a blown-up diagonal element to the prior (zeroing the
        // off-diagonal so the reset stays PSD).
        if !self.p00.is_finite() || self.p00 < 0.0 {
            self.p00 = self.p00_cap;
            self.p01 = 0.0;
        }
        if !self.p11.is_finite() || self.p11 < 0.0 {
            self.p11 = self.p11_cap;
            self.p01 = 0.0;
        }
        if !self.p01.is_finite() {
            self.p01 = 0.0;
        }
        // Anti-windup: bound the TOTAL covariance (trace) at the prior trace by scaling the
        // WHOLE matrix (preserves PSD and the eigen-directions — unlike per-element caps,
        // which broke PSD). Without this, persistent low excitation inflates the null-space
        // element by 1/λ per tick until it overflows and snaps the model back to ignorance.
        // Known trade-off while the cap is active (long single-load dwell): the identified
        // direction's variance keeps shrinking, so `conf` reads optimistically and re-learning
        // a drift AT THE SAME load is slow — the residual integral covers the difference, and
        // a load CHANGE still adapts fast (null-direction variance stays high).
        let trace = self.p00 + self.p11;
        let trace_cap = self.p00_cap + self.p11_cap;
        if trace > trace_cap && trace.is_finite() {
            let s = trace_cap / trace;
            self.p00 *= s;
            self.p01 *= s;
            self.p11 *= s;
        }
    }
}

pub struct Controller {
    kind: Kind,
    cmd: Option<f64>, // TrueIntegral state
    i: f64,           // Pi / Adaptive residual integral
    ff: RlsModel,     // Adaptive learned model (unused by other kinds)
    prev_x: Option<f64>, // Adaptive: previous throughput, for the steady-state gate
}

/// Snapshot of the learned model for logging/telemetry: `(a, b, confidence)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FfSnapshot {
    pub a: f64,
    pub b: f64,
    pub conf: f64,
}

impl Controller {
    pub fn new(kind: Kind) -> Self {
        let ff = match kind {
            Kind::Adaptive(c) => RlsModel::new(c.a0, c.b0, c.seeded, c.lambda),
            _ => RlsModel::new(0.0, 0.0, false, 0.999),
        };
        Controller { kind, cmd: None, i: 0.0, ff, prev_x: None }
    }

    pub fn name(&self) -> &'static str {
        match self.kind {
            Kind::Stock => "stock",
            Kind::TrueIntegral { .. } => "true-integral",
            Kind::Pi { .. } => "pi",
            Kind::Adaptive(_) => "adaptive",
        }
    }

    /// Reset the transient loop state on hand-back to avoid integral windup while the
    /// stock firmware drives. The LEARNED model is deliberately preserved — it is
    /// long-lived knowledge about the plant, not per-episode state.
    pub fn reset(&mut self) {
        self.cmd = None;
        self.i = 0.0;
        self.prev_x = None;
    }

    /// Current learned model, if this is an `Adaptive` controller. Confidence maps the
    /// prediction-σ at the given throughput onto `(0, 1]` via `s/(s+σ)` (a non-finite
    /// throughput, or infinite σ, reads as 0). This is the model's *statistical*
    /// confidence — what `Apply` mode actually actuates; `Observe`/`Frozen` deliberately
    /// override the applied weight, so for those this is the "would-be" confidence.
    pub fn ff_snapshot(&self, throughput: f64) -> Option<FfSnapshot> {
        match self.kind {
            Kind::Adaptive(c) => {
                let sigma = self.ff.pred_var(throughput.abs()).sqrt();
                let conf = if sigma.is_finite() {
                    c.conf_scale_w / (c.conf_scale_w + sigma)
                } else {
                    0.0
                };
                Some(FfSnapshot { a: self.ff.a, b: self.ff.b, conf })
            }
            _ => None,
        }
    }

    /// `learn` gates adaptive learning: it must be true only when THIS controller's
    /// command actually drives the plant (takeover + owning + writing). In shadow/trim
    /// our command is not actuated, so learning from the resulting grid would train the
    /// model on a regime it will never drive. Ignored by the non-adaptive kinds.
    pub fn update(
        &mut self,
        grid: f64,
        reported: f64,
        target: f64,
        dt: f64,
        lo: f64,
        hi: f64,
        learn: bool,
    ) -> f64 {
        match self.kind {
            Kind::Stock => clamp(target - grid + reported, lo, hi),
            Kind::TrueIntegral { ki } => {
                // Hold the accumulated command through a non-finite reading (emit the safe
                // idle command, resume from held state on recovery) — never wipe it to 0.
                if !grid.is_finite() {
                    return clamp(f64::NAN, lo, hi); // -> 0 (idle)
                }
                let c = self.cmd.unwrap_or(target - grid + reported);
                let c = clamp(c - ki * (grid - target), lo, hi);
                self.cmd = Some(c);
                c
            }
            Kind::Pi { ki, i_max } => {
                let err = target - grid;
                // Hold the integral on a non-finite reading rather than letting `clamp`
                // silently zero it (same policy as the adaptive path below); the command
                // itself still degrades to the safe idle 0 via the final clamp.
                if err.is_finite() && dt > 0.0 {
                    self.i = clamp(self.i + ki * err * dt, -i_max, i_max);
                }
                clamp((target - grid + reported) + self.i, lo, hi)
            }
            Kind::Adaptive(c) => {
                self.update_adaptive(c, grid, reported, target, dt, lo, hi, learn)
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn update_adaptive(
        &mut self,
        c: AdaptiveCfg,
        grid: f64,
        reported: f64,
        target: f64,
        dt: f64,
        lo: f64,
        hi: f64,
        learn: bool,
    ) -> f64 {
        let x = reported.abs(); // throughput regressor
        let err = target - grid;
        // Residual integral drives the grid onto target (correctness backstop). Hold it
        // on a non-finite reading rather than letting `clamp` silently zero it.
        if err.is_finite() && dt > 0.0 {
            self.i = clamp(self.i + c.ki * err * dt, -c.i_max, c.i_max);
        }

        // Model prediction and its actuation authority for this tick. A non-finite
        // throughput contributes nothing (and never poisons `i` with a NaN).
        let m = self.ff.predict(x);
        let (conf, model_term) = if x.is_finite() {
            let conf = match c.mode {
                // Confidence-weighted: earns authority as prediction-σ shrinks below scale.
                FfMode::Apply => {
                    let sigma = self.ff.pred_var(x).sqrt();
                    c.conf_scale_w / (c.conf_scale_w + sigma)
                }
                FfMode::Observe => 0.0, // learn but don't actuate the feed-forward
                FfMode::Frozen => 1.0,  // trust the seed fully
            };
            (conf, conf * m)
        } else {
            (0.0, 0.0)
        };
        // Bias actually applied beyond the stock feed-forward.
        let applied_bias = model_term + self.i;

        // Learn (unless frozen) only when WE actuate and from clean steady-state
        // DISCHARGE/idle samples: settled grid error AND settled throughput. Load
        // transients are excluded (they'd poison the fit), and so are charging samples
        // (`reported > 0`, e.g. a force-charge/charge-floor episode) — the model is a
        // DISCHARGE leak curve and charge efficiency differs.
        let steady = learn
            && err.abs() <= c.steady_band_w
            && self
                .prev_x
                .map(|px| (x - px).abs() <= c.steady_band_w)
                .unwrap_or(false)
            && dt > 0.0
            && grid.is_finite()
            && reported.is_finite()
            && reported <= 0.0;
        if c.mode != FfMode::Frozen && steady {
            // Teach the model the bias the loop is ACTUALLY holding at this load, so it
            // converges to the true leak(x) regardless of how the split is weighted.
            self.ff.update(x, applied_bias);
            if c.mode == FfMode::Apply {
                // Transfer authority integral -> model without double-counting: subtract
                // the model's increase from the integral so `conf*m + i` is invariant at
                // the transfer instant. Over time i -> 0 and the model carries the load.
                let m_new = self.ff.predict(x);
                self.i = clamp(self.i - conf * (m_new - m), -c.i_max, c.i_max);
            }
        }
        // A bad read must not seed the steady gate with a NaN throughput next tick.
        self.prev_x = if x.is_finite() { Some(x) } else { None };

        clamp((target - grid + reported) + applied_bias, lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn adaptive(cfg: AdaptiveCfg) -> Controller {
        Controller::new(Kind::Adaptive(cfg))
    }

    /// A tiny closed-loop plant. At throughput `x` the loop needs a compensating bias
    /// `comp(x)` beyond the stock feed-forward to sit on target; the grid moves toward
    /// target as the applied bias approaches `comp(x)`. Sign matches the controller's
    /// convention (negative feedback), so `grid = target - comp(x) + prev_bias`.
    /// Returns the applied bias for the next tick.
    fn plant_tick(
        ctrl: &mut Controller,
        x: f64,
        target: f64,
        prev_bias: f64,
        comp: impl Fn(f64) -> f64,
    ) -> f64 {
        let reported = -x; // discharge
        let grid = target - comp(x) + prev_bias;
        let cmd = ctrl.update(grid, reported, target, 2.5, -9000.0, 9000.0, true);
        // bias = command - stock feed-forward
        cmd - (target - grid + reported)
    }

    #[test]
    fn adaptive_learns_affine_leak_and_generalizes() {
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        let (a0, b0) = (25.0, 0.04);
        let target = 20.0;
        let leak = |x: f64| a0 + b0 * x;
        // Alternate two well-separated load clusters -> spread for slope identifiability.
        let mut bias = 0.0;
        for i in 0..4000 {
            // dwell in blocks so the steady-state gate accepts samples (Δx==0 within block)
            let x = if (i / 50) % 2 == 0 { 300.0 } else { 1500.0 };
            bias = plant_tick(&mut ctrl, x, target, bias, leak);
        }
        let snap = ctrl.ff_snapshot(900.0).unwrap();
        assert!((snap.a - a0).abs() < 4.0, "a={} want ~{a0}", snap.a);
        assert!((snap.b - b0).abs() < 0.01, "b={} want ~{b0}", snap.b);
        assert!(snap.conf > 0.5, "confidence should be high, got {}", snap.conf);
        // Extrapolation: a load never dwelt at is predicted from the affine fit.
        let pred_3k = snap.a + snap.b * 3000.0;
        assert!((pred_3k - leak(3000.0)).abs() < 20.0, "extrapolated {pred_3k}");
    }

    #[test]
    fn adaptive_cold_start_behaves_like_pi() {
        // With no data and a wide prior, first ticks give the model ~no authority, so
        // the command tracks a plain PI to within a small tolerance.
        let mut adapt = adaptive(AdaptiveCfg::tuned());
        let mut pi = Controller::new(Kind::Pi { ki: 0.05, i_max: 300.0 });
        for _ in 0..3 {
            let ca = adapt.update(300.0, -1000.0, 20.0, 2.5, -9000.0, 9000.0, true);
            let cp = pi.update(300.0, -1000.0, 20.0, 2.5, -9000.0, 9000.0, true);
            assert!((ca - cp).abs() < 5.0, "cold-start adaptive {ca} vs pi {cp}");
        }
    }

    #[test]
    fn adaptive_observe_learns_but_does_not_actuate() {
        let mut cfg = AdaptiveCfg::tuned();
        cfg.mode = FfMode::Observe;
        let mut obs = adaptive(cfg);
        let comp = |x: f64| 25.0 + 0.04 * x;
        let mut bias = 0.0;
        for i in 0..1000 {
            let x = if (i / 50) % 2 == 0 { 300.0 } else { 1500.0 };
            bias = plant_tick(&mut obs, x, 20.0, bias, comp);
        }
        // Model moved (it learned the slope) ...
        let snap = obs.ff_snapshot(900.0).unwrap();
        assert!(snap.b > 0.02, "observe should still learn slope, got {}", snap.b);
        // ... but observe never applies the feed-forward. With the integral reset (model
        // preserved), the command must equal a fresh PI's for identical inputs — i.e. the
        // learned model contributes nothing to the output.
        obs.reset();
        let mut p2 = Controller::new(Kind::Pi { ki: cfg.ki, i_max: cfg.i_max });
        let cmd_o = obs.update(30.0, -800.0, 20.0, 2.5, -9000.0, 9000.0, true);
        let cmd_p = p2.update(30.0, -800.0, 20.0, 2.5, -9000.0, 9000.0, true);
        assert!((cmd_o - cmd_p).abs() < 1e-6, "observe actuated FF: {cmd_o} vs {cmd_p}");
    }

    #[test]
    fn adaptive_frozen_applies_seed_without_learning() {
        let mut cfg = AdaptiveCfg::tuned();
        cfg.mode = FfMode::Frozen;
        cfg.a0 = 30.0;
        cfg.b0 = 0.05;
        cfg.seeded = true;
        let mut ctrl = adaptive(cfg);
        // Frozen model must apply a+b*x on top of stock immediately.
        let cmd = ctrl.update(20.0, -1000.0, 20.0, 2.5, -9000.0, 9000.0, true);
        let stock = 20.0 - 20.0 + (-1000.0);
        // grid==target so integral stays 0; applied bias should be the seed model.
        assert!((cmd - (stock + 30.0 + 0.05 * 1000.0)).abs() < 1e-6, "frozen cmd {cmd}");
        // And it never learns: params unchanged after many steps.
        for _ in 0..500 {
            let _ = plant_tick(&mut ctrl, 800.0, 20.0, 0.0, |x| 25.0 + 0.04 * x);
        }
        let snap = ctrl.ff_snapshot(0.0).unwrap();
        assert_eq!(snap.a, 30.0);
        assert_eq!(snap.b, 0.05);
    }

    #[test]
    fn adaptive_low_confidence_without_spread() {
        // Train at ONE operating point only: the slope stays unidentified, so authority
        // at a far, unseen load must remain low even after many samples.
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        for _ in 0..3000 {
            let _ = plant_tick(&mut ctrl, 500.0, 20.0, 0.0, |x| 25.0 + 0.04 * x);
        }
        let near = ctrl.ff_snapshot(500.0).unwrap().conf;
        let far = ctrl.ff_snapshot(3000.0).unwrap().conf;
        assert!(far < 0.4, "far-load authority should stay low, got {far}");
        assert!(far < near, "confidence should fall off away from sampled load");
    }

    #[test]
    fn adaptive_reset_preserves_learned_model() {
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        for i in 0..1000 {
            let x = if (i / 50) % 2 == 0 { 300.0 } else { 1500.0 };
            let _ = plant_tick(&mut ctrl, x, 20.0, 0.0, |x| 25.0 + 0.04 * x);
        }
        let before = ctrl.ff_snapshot(900.0).unwrap();
        ctrl.reset();
        let after = ctrl.ff_snapshot(900.0).unwrap();
        assert_eq!(before.a, after.a, "reset must not wipe learned offset");
        assert_eq!(before.b, after.b, "reset must not wipe learned slope");
        assert_eq!(ctrl.i, 0.0, "reset must clear the integral");
    }

    // ---- closed-loop replay machinery (shared by the always-run property tests
    // below and the ignored real-capture replay) --------------------------------
    // Plant: rep(t) = A + B·write(t−1) — the live-fitted Multi tracking model
    // (2026-08-21: A=4.7, B=0.9562, i.e. the 4.4 % under-tracking leak) + N(0,35 W)
    // measurement noise from a deterministic xorshift (reproducible, no crates).

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
    /// A balancing controller's median error must be inside this band.
    const M_BALANCE: f64 = 5.0;
    /// No single transient may exceed this (physical load steps included).
    const M_WORST_TRANSIENT: f64 = 4000.0;
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
            let (mut rep, mut prev_w): (f64, Option<f64>) = (0.0, None);
            let mut frozen = 0.0f64;
            let half = seg.len() / 2;
            for (k, &(_t, load)) in seg.iter().enumerate() {
                let (pa, pb) =
                    if scen == "plantshift" && k > half { (40.0, 0.91) } else { (4.7, 0.9562) };
                let grid_true = load + rep;
                let mut grid = grid_true + rng.gauss(35.0);
                if scen == "noiseburst" && (k / 600) % 6 == 0 {
                    grid += rng.gauss(200.0);
                }
                if scen == "meterfreeze" && k % 600 < 5 && k > 0 {
                    grid = frozen;
                }
                frozen = grid;
                let cmd = ctrl.update(grid, rep, target, 1.0, lo, hi, true);
                if cmd <= lo + 1e-9 || cmd >= hi - 1e-9 {
                    clamps += 1;
                }
                let w = crate::loop_core::stock_write_ema(prev_w, cmd, false);
                prev_w = Some(w);
                rep = pa + pb * w;
                if k > 120 {
                    let e = grid_true - target;
                    errs.push(e);
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
            _ => {
                let mut cfg = AdaptiveCfg::tuned();
                cfg.a0 = a0;
                cfg.b0 = b0;
                cfg.seeded = a0 != 0.0 || b0 != 0.0;
                Kind::Adaptive(cfg)
            }
        };
        let r = replay(&segs, &mk, &scen);
        eprintln!(
            "REPLAY scen={scen}: median {:+.1}W mean {:+.1}W worst {:.0}W env-clamps {} pred@500 {:.1}",
            r.median, r.mean, r.worst, r.env_clamps, r.pred_500
        );
    }

    /// ALWAYS-RUN closed-loop property: the leak controllers actually balance the
    /// grid on a realistic load, and stock reproduces the leak (sim sanity anchor).
    #[test]
    fn replay_property_leak_is_balanced() {
        let segs = vec![synthetic_load_segment(7, 7200)];
        let stock = replay(&segs, &|| Kind::Stock, "baseline");
        let pi = replay(&segs, &|| Kind::Pi { ki: 0.05, i_max: 300.0 }, "baseline");
        let ad = replay(&segs, &|| Kind::Adaptive(AdaptiveCfg::tuned()), "baseline");
        assert!(stock.median > M_LEAK_MIN, "stock must show the leak, got {:+.1}", stock.median);
        assert!(pi.median.abs() < M_BALANCE, "pi must balance, got {:+.1}", pi.median);
        assert!(ad.median.abs() < M_BALANCE, "adaptive must balance, got {:+.1}", ad.median);
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
        let base = replay(&segs, &|| Kind::Adaptive(AdaptiveCfg::tuned()), "baseline");
        assert!(
            (base.pred_500 - truth).abs() < M_MODEL_CONV,
            "model must converge near the true leak ({truth:.1}), got {:.1}",
            base.pred_500
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

    #[test]
    fn adaptive_does_not_learn_when_not_actuating() {
        // learn=false (shadow/trim: our command isn't applied) must freeze learning even
        // on perfectly steady data, so the live-carried model isn't trained on a phantom
        // regime.
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        for _ in 0..2000 {
            // steady discharge at a fixed load, but not actuating
            let _ = ctrl.update(20.0, -800.0, 20.0, 2.5, -9000.0, 9000.0, false);
        }
        let snap = ctrl.ff_snapshot(800.0).unwrap();
        assert_eq!(snap.a, 0.0, "must not learn offset when not actuating");
        assert_eq!(snap.b, 0.0, "must not learn slope when not actuating");
    }

    #[test]
    fn adaptive_covariance_bounded_under_long_low_excitation() {
        // A day-plus of near-constant load must not inflate the covariance to overflow
        // (which would periodically snap the model back to ignorance). Dwell at a single
        // load far longer than a real day and assert the model stays finite and confident.
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        let comp = |x: f64| 25.0 + 0.04 * x;
        let mut bias = 0.0;
        for _ in 0..200_000 {
            bias = plant_tick(&mut ctrl, 800.0, 20.0, bias, comp);
        }
        let snap = ctrl.ff_snapshot(800.0).unwrap();
        assert!(snap.a.is_finite() && snap.b.is_finite(), "params blew up");
        assert!(snap.conf > 0.5, "confidence collapsed (covariance overflow?): {}", snap.conf);
        assert!((bias - comp(800.0)).abs() < 8.0, "lost the correction: {bias}");
    }

    #[test]
    fn adaptive_nan_input_is_safe() {
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        // warm the model up first
        for _ in 0..100 {
            let _ = plant_tick(&mut ctrl, 800.0, 20.0, 0.0, |x| 25.0 + 0.04 * x);
        }
        let good = ctrl.ff_snapshot(800.0).unwrap();
        let cmd = ctrl.update(f64::NAN, -800.0, 20.0, 2.5, -9000.0, 9000.0, true);
        assert!(cmd.is_finite(), "NaN grid must yield a finite command");
        let after = ctrl.ff_snapshot(800.0).unwrap();
        assert!(after.a.is_finite() && after.b.is_finite());
        // A NaN error must not have corrupted the model.
        assert!((after.a - good.a).abs() < 50.0 && (after.b - good.b).abs() < 0.1);
    }

    #[test]
    fn adaptive_model_takes_over_from_integral_no_windup() {
        // The anti-double-count invariant is at STEADY STATE: as the model learns the
        // needed bias, the integral must RELAX toward zero (the model carries the load),
        // rather than the two stacking to twice the correction (windup). Dwell at a fixed
        // load and check the model ends up carrying it while the integral is near zero.
        let mut ctrl = adaptive(AdaptiveCfg::tuned());
        let comp = |x: f64| 25.0 + 0.04 * x; // needed bias at load x
        let x = 800.0;
        let target = 20.0;
        let mut bias = 0.0;
        for _ in 0..4000 {
            bias = plant_tick(&mut ctrl, x, target, bias, comp);
        }
        let want = comp(x); // 57 W
        // Total applied bias converged to the single correction, not double it.
        assert!((bias - want).abs() < 6.0, "applied bias {bias} want ~{want}");
        // The model now carries it (high-confidence prediction ~= the needed bias) ...
        let snap = ctrl.ff_snapshot(x).unwrap();
        let model_term = snap.conf * (snap.a + snap.b * x);
        assert!((model_term - want).abs() < 12.0, "model term {model_term} want ~{want}");
        // ... and the integral has relaxed toward zero (no windup / no double count).
        assert!(ctrl.i.abs() < 15.0, "integral should relax, got {}", ctrl.i);
    }
}
