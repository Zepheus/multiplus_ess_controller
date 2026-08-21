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
#[path = "control_tests.rs"]
mod tests;
