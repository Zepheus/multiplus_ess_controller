//! Generator / genset gating for the ESS control loop.
//!
//! A generator (genset) is a fundamentally different AC source from the grid:
//! a synchronous genset **cannot absorb reverse power** — pushing current into it
//! drives the engine as a motor / overspeeds the alternator and trips or damages it.
//! So when the Multi's active input is a genset, ESS must suppress the whole
//! "sell surplus to AC-in" machinery it runs on grid.
//!
//! The mechanism (faithful to the stock ESS loop) is NOT a blunt `DisableFeedIn=1`; it is a
//! *reinterpretation* of the AC-in setpoint via `TargetPowerIsMaxFeedIn`: the Multi
//! charges the battery from everything available but treats `AcPowerSetpoint` as the
//! **maximum** it may push toward AC-in, so the genset is only ever loaded, never
//! back-fed. A genset can therefore only ever be in feed mode 0 (TPIMFI) or mode 1
//! (charge-hard, feed-in shut) — **never** the normal grid-tracking/export mode.
//!
//! Only `Ac/ActiveIn/Source == 2` (genset) changes policy. Grid (1), shore (3),
//! island (240) and unknown/missing (0/[]) all behave grid-like. On THIS install the
//! source reads 1 (grid); the genset branches are cold code here and kept behind
//! `is_genset()` guards so a future input reconfiguration can never silently back-feed.
//!
//! empirically modelled against the stock ESS loop (Venus OS v3.75). See
//! the design notes (its §6 is the drop-in spec this module implements).
//!
//! Safety posture:
//!   * Unknown / missing / unmapped source read  => grid-like (safe default: never
//!     forces an export cap, never loses grid export on a misconfigured input).
//!   * DVCC unknown  => off  => a genset degrades to charge-hard (safe: no feed-in).
//!   * `/Hub4/TargetPowerIsMaxFeedIn` absent (old firmware) => never written; the
//!     genset then relies on `DisableFeedIn` / `DoNotFeedInOvervoltage=1` to block
//!     back-feed.
//!   * Sticky-genset (safe-direction divergence from the reference implementation): once source==2 has
//!     been observed, a *dropped* read (None) holds Genset rather than flipping to
//!     grid-like for a tick — a transient read glitch must never re-enable export
//!     toward a physical generator. An *explicit* non-genset value (incl. 0/240) is
//!     always honoured, so a real source change still takes effect.

use crate::dbus::*;

// --- new dbus path consts (local to this subsystem) ---
// `P_DVCC` ("/Control/Dvcc", on SYS) already exists in dbus.rs and is reused.
/// AC-input type enum (SYS). 0=n/a, 1=grid, 2=genset, 3=shore, 240=island.
pub const P_ACIN_SOURCE: &str = "/Ac/ActiveIn/Source";
/// DC-coupled overvoltage feed-in permission setting (SETTINGS, bool).
/// Multi: setpoint-becomes-max-feed-in-cap flag (VEBUS, int 0/1). Its *validity*
/// (does firmware expose it) is the "Multi supports TPIMFI" gate.
pub const P_TPIMFI: &str = "/Hub4/TargetPowerIsMaxFeedIn";

/// The AC-input type (`Ac/ActiveIn/Source`). Only `Genset` changes feed-in policy;
/// every other value is grid-like.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AcSource {
    /// 0 / invalid `[]` / unmapped — config error or read failure. Grid-like (safe).
    Unknown,
    /// 1 — grid. Normal ESS: track setpoint, export allowed.
    Grid,
    /// 2 — genset / generator. Charge-only, no back-feed (this module's whole point).
    Genset,
    /// 3 — shore power. Handled identically to grid.
    Shore,
    /// 240 (0xF0) — inverting / island (no AC input connected). Grid-like here; the
    /// grid-meter presence state machine owns this case (a different subsystem).
    Island,
}

impl AcSource {
    /// Map a raw `Ac/ActiveIn/Source` read. `None` (invalid `[]` / read fail) and any
    /// unmapped integer fall to `Unknown` => grid-like (matches the reference implementation's non-genset
    /// defaults of 0 on the flag path / 3 on the tick).
    pub fn from_dbus(v: Option<i32>) -> Self {
        match v {
            Some(1) => AcSource::Grid,
            Some(2) => AcSource::Genset,
            Some(3) => AcSource::Shore,
            Some(240) => AcSource::Island,
            _ => AcSource::Unknown, // 0, other, or missing/invalid
        }
    }
    /// The ONLY predicate that changes feed-in policy in the stock ESS loop.
    pub fn is_genset(self) -> bool {
        self == AcSource::Genset
    }
    /// Grid, Shore, Island and Unknown are all handled identically (grid-like).
    pub fn is_grid_like(self) -> bool {
        !self.is_genset()
    }
}

/// Feed-in mode tri-state (the stock ESS loop ``, stored at ControlLoop+0x28).
/// **2** = normal grid tracking (export allowed); **1** = charge as hard as possible,
/// no feed-in; **0** = TPIMFI (setpoint reinterpreted as a max-feed-in cap, Multi
/// self-charges). A genset can only ever be `Tpimfi` or `ChargeHard`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FeedMode {
    Tpimfi = 0,
    ChargeHard = 1,
    Normal = 2,
}

/// Force-charge context from the (not-yet-built) force-charge / feed-in subsystem
/// (see the feed-in/sustain design notes). All-false `Default` yields grid `Normal`, which is the
/// correct grid-only stub for THIS install. The genset branch ignores every field.
#[derive(Clone, Copy, Default)]
pub struct ForceChargeCtx {
    pub must_force_charge: bool,
    pub feed_in_excess_allowed: bool,
    pub shore_limit_saturated: bool,
}


/// The gating decision main.rs folds into the feed-in flags / setpoint interpretation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct GenOut {
    /// Source is a genset (source == 2). The one predicate that changes policy.
    pub is_genset: bool,
    /// Force `TargetPowerIsMaxFeedIn = 1`: the AC-in setpoint becomes a *max feed-in
    /// cap* so the source is only ever loaded, never back-fed. True iff mode == Tpimfi.
    pub force_tpimfi: bool,
    /// The controller must NOT run normal export tracking (regulate-to-setpoint with
    /// export). True whenever mode != Normal — i.e. always on a genset, and on grid
    /// only while force-charging. main.rs clamps the command's export (negative) side.
    pub disable_export: bool,
    /// The computed feed mode (drives the flag writers). Extra to the contract fields.
    pub mode: FeedMode,
}

/// Pure port of the stock ESS loop's feed-in mode tri-state (``, §4.1).
fn feed_mode(
    source: AcSource,
    dvcc: bool,
    tpimfi_supported: bool,
    fc: ForceChargeCtx,
) -> FeedMode {
    if source.is_genset() {
        // GENSET: never Normal. Clean TPIMFI iff DVCC + firmware, else charge-hard.
        // Deliberately does NOT consult must_force_charge / excess / shore-limit —
        // a genset is always in the "don't export" regime regardless of battery state.
        if dvcc && tpimfi_supported {
            FeedMode::Tpimfi
        } else {
            FeedMode::ChargeHard
        }
    } else if !fc.must_force_charge {
        FeedMode::Normal
    } else if dvcc && tpimfi_supported {
        if fc.feed_in_excess_allowed || fc.shore_limit_saturated {
            FeedMode::ChargeHard
        } else {
            FeedMode::Tpimfi
        }
    } else {
        FeedMode::ChargeHard
    }
}

/// Generator / genset gating subsystem. Holds the last-read source/DVCC/firmware-support
/// plus the write-if-changed caches for the three vebus flags.
pub struct Generator {
    source: AcSource,
    dvcc: bool,
    tpimfi_supported: bool,
}

impl Generator {
    /// Read the initial source / DVCC / TPIMFI-support and log. Reads are cheap.
    pub fn new(bus: &dyn Bus) -> Self {
        let mut g = Generator {
            source: AcSource::Unknown,
            dvcc: false,
            tpimfi_supported: false,
        };
        g.refresh(bus);
        eprintln!(
            "generator gate: source={:?} dvcc={} tpimfi_supported={}",
            g.source, g.dvcc, g.tpimfi_supported
        );
        g
    }

    /// Refresh the cached source / DVCC / firmware-support from the bus.
    ///
    /// Sticky-genset: a *dropped* source read (None) holds a previously observed
    /// `Genset` rather than flipping to grid-like — a transient glitch must never
    /// re-enable export toward a generator. An explicit value (incl. 0/240) is always
    /// honoured, so a genuine source change still takes effect.
    fn refresh(&mut self, bus: &dyn Bus) {
        self.source = match bus.get_i32(SYS, P_ACIN_SOURCE) {
            Some(v) => AcSource::from_dbus(Some(v)),
            None if self.source == AcSource::Genset => AcSource::Genset,
            None => AcSource::Unknown,
        };
        // DVCC unknown => off (safe: a genset then degrades to charge-hard).
        self.dvcc = bus.get_bool(SYS, P_DVCC).unwrap_or(false);
        // Firmware exposes the path iff the item is valid on the bus.
        self.tpimfi_supported = bus.get_i32(VEBUS, P_TPIMFI).is_some();
    }

    pub fn source(&self) -> AcSource {
        self.source
    }

    /// Per-tick: refresh inputs, compute the feed mode, and return the gating decision.
    /// Call once per loop iteration BEFORE the setpoint clamp and the flag writers, both
    /// of which consume the result. `fc` all-false (Default) is the grid-only stub.
    pub fn tick(&mut self, bus: &dyn Bus, fc: ForceChargeCtx) -> GenOut {
        self.refresh(bus);
        let mode = feed_mode(self.source, self.dvcc, self.tpimfi_supported, fc);
        GenOut {
            is_genset: self.source.is_genset(),
            force_tpimfi: mode == FeedMode::Tpimfi,
            disable_export: mode != FeedMode::Normal,
            mode,
        }
    }

}

#[cfg(test)]
#[path = "generator_tests.rs"]
mod tests;
