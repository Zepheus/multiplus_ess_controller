//! Control laws for the mode-3 AC-in power loop.
//!
//! Ported from the validated `ess_lab` Python sandbox.  Each controller outputs
//! the Multi command directly (mode 3): negative = discharge, positive = charge.
//!
//!   grid     : measured grid power (W, +import)
//!   reported : Multi's reported AC-in power (W, neg=discharge)
//!   target   : grid setpoint (W)
//!   dt       : seconds since previous tick

pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
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
}

pub struct Controller {
    kind: Kind,
    cmd: Option<f64>, // TrueIntegral state
    i: f64,           // Pi integral state
}

impl Controller {
    pub fn new(kind: Kind) -> Self {
        Controller { kind, cmd: None, i: 0.0 }
    }

    pub fn name(&self) -> &'static str {
        match self.kind {
            Kind::Stock => "stock",
            Kind::TrueIntegral { .. } => "true-integral",
            Kind::Pi { .. } => "pi",
        }
    }

    pub fn reset(&mut self) {
        self.cmd = None;
        self.i = 0.0;
    }

    pub fn update(
        &mut self,
        grid: f64,
        reported: f64,
        target: f64,
        dt: f64,
        lo: f64,
        hi: f64,
    ) -> f64 {
        match self.kind {
            Kind::Stock => clamp(target - grid + reported, lo, hi),
            Kind::TrueIntegral { ki } => {
                let c = self.cmd.unwrap_or(target - grid + reported);
                let c = clamp(c - ki * (grid - target), lo, hi);
                self.cmd = Some(c);
                c
            }
            Kind::Pi { ki, i_max } => {
                let err = target - grid;
                self.i = clamp(self.i + ki * err * dt, -i_max, i_max);
                clamp((target - grid + reported) + self.i, lo, hi)
            }
        }
    }
}
