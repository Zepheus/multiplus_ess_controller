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
pub const P_OVFEEDIN: &str = "/Settings/CGwacs/OvervoltageFeedIn";
/// Multi: setpoint-becomes-max-feed-in-cap flag (VEBUS, int 0/1). Its *validity*
/// (does firmware expose it) is the "Multi supports TPIMFI" gate.
pub const P_TPIMFI: &str = "/Hub4/TargetPowerIsMaxFeedIn";
/// Multi: block DC-PV overvoltage feed-in (VEBUS, int 0/1; 0=allowed, 1=blocked).
pub const P_DONOTFEEDIN: &str = "/Hub4/DoNotFeedInOvervoltage";
/// Multi: disable AC-in feed-in entirely (VEBUS, int 0/1).
pub const P_DISABLEFEEDIN: &str = "/Hub4/DisableFeedIn";

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

impl FeedMode {
    pub fn as_i32(self) -> i32 {
        self as i32
    }
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

/// `/Overrides/FeedInExcess` (server-side state written into our own hub4 service by
/// DESS/systemcalc). In the grid-only stub it is absent => `UseSetting`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum FeedInExcess {
    #[default]
    UseSetting, // 0 — consult /Settings/CGwacs/OvervoltageFeedIn
    ForceNo,    // 1 — force NO excess feed-in
    Force,      // 2 — force feed-in
}

/// Inputs to the flag writers that live in OTHER subsystems (battery-limit / sustain).
/// Grid-only stub: `Default` is safe (no excess override, no overvoltage feed-in,
/// discharge blocked). Only used by [`Generator::write_feedin_flags`].
#[derive(Clone, Copy, Default)]
pub struct FlagInputs {
    pub excess: FeedInExcess,
    /// `/Settings/CGwacs/OvervoltageFeedIn` (DC-PV overvoltage feed-in permission).
    pub overvoltage_feedin: bool,
    pub can_discharge: bool,
    pub battery_at_floor: bool,
    pub always_peak_shave: bool,
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
    // write-if-changed caches (never spam the MK3): compare before every write.
    last_tpimfi: Option<i32>,
    last_do_not_feedin: Option<i32>,
    last_disable_feedin: Option<i32>,
}

impl Generator {
    /// Read the initial source / DVCC / TPIMFI-support and log. Reads are cheap.
    pub fn new(bus: &dyn Bus) -> Self {
        let mut g = Generator {
            source: AcSource::Unknown,
            dvcc: false,
            tpimfi_supported: false,
            last_tpimfi: None,
            last_do_not_feedin: None,
            last_disable_feedin: None,
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
    pub fn dvcc(&self) -> bool {
        self.dvcc
    }
    pub fn tpimfi_supported(&self) -> bool {
        self.tpimfi_supported
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

    /// Full genset path: write-if-changed the three vebus feed-in flags for `out.mode`.
    /// (main.rs may instead fold `GenOut` into its own writers; this is the complete
    /// self-contained writer for callers that want it.) No-op writes are suppressed.
    pub fn write_feedin_flags(&mut self, bus: &dyn Bus, out: &GenOut, fi: &FlagInputs) {
        // --- TargetPowerIsMaxFeedIn = (mode == Tpimfi); only if firmware exposes it ---
        if self.tpimfi_supported {
            let v = (out.mode == FeedMode::Tpimfi) as i32;
            if self.last_tpimfi != Some(v) {
                if bus.set_i32(VEBUS, P_TPIMFI, v).is_ok() {
                    self.last_tpimfi = Some(v);
                }
            }
        }

        // --- DoNotFeedInOvervoltage: the permissive branch is GATED on !genset (§4.3) ---
        let excess_allows_ov = match fi.excess {
            FeedInExcess::Force => true,
            FeedInExcess::UseSetting => fi.overvoltage_feedin,
            FeedInExcess::ForceNo => false,
        };
        let permissive = !out.is_genset && excess_allows_ov;
        let dnf = if permissive {
            0
        } else {
            (out.mode != FeedMode::Tpimfi) as i32
        };
        if self.last_do_not_feedin != Some(dnf) {
            if bus.set_i32(VEBUS, P_DONOTFEEDIN, dnf).is_ok() {
                self.last_do_not_feedin = Some(dnf);
            }
        }

        // --- DisableFeedIn: not source-gated directly, but a genset never reaches
        //     Normal, so mode + these battery gates structurally prevent back-feed. ---
        let disable = if out.mode == FeedMode::Tpimfi {
            0 // mode 0 needs the feed path open (TPIMFI caps it instead)
        } else if !fi.can_discharge {
            1
        } else if fi.battery_at_floor {
            (!fi.always_peak_shave) as i32
        } else {
            0
        };
        if self.last_disable_feedin != Some(disable) {
            if bus.set_i32(VEBUS, P_DISABLEFEEDIN, disable).is_ok() {
                self.last_disable_feedin = Some(disable);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testbus::MockBus;
    use std::cell::RefCell;
    use std::collections::HashMap;

    // ---- enum mapping ----------------------------------------------------------

    #[test]
    fn source_enum_maps_every_value() {
        assert_eq!(AcSource::from_dbus(Some(0)), AcSource::Unknown);
        assert_eq!(AcSource::from_dbus(Some(1)), AcSource::Grid);
        assert_eq!(AcSource::from_dbus(Some(2)), AcSource::Genset);
        assert_eq!(AcSource::from_dbus(Some(3)), AcSource::Shore);
        assert_eq!(AcSource::from_dbus(Some(240)), AcSource::Island);
        assert_eq!(AcSource::from_dbus(Some(99)), AcSource::Unknown); // unmapped
        assert_eq!(AcSource::from_dbus(None), AcSource::Unknown); // invalid []
        // Only genset is special; everything else is grid-like.
        for s in [AcSource::Unknown, AcSource::Grid, AcSource::Shore, AcSource::Island] {
            assert!(s.is_grid_like() && !s.is_genset());
        }
        assert!(AcSource::Genset.is_genset() && !AcSource::Genset.is_grid_like());
    }

    // ---- gating per source (the required matrix), via tick() + MockBus ----------

    /// Bus with a source value and DVCC + TPIMFI-firmware present (the "clean" case).
    fn bus_with(source: Option<i32>, dvcc: bool, tpimfi_fw: bool) -> MockBus {
        let mut b = MockBus::new();
        if let Some(v) = source {
            b.set(SYS, P_ACIN_SOURCE, v as f64);
        }
        b.set(SYS, P_DVCC, if dvcc { 1.0 } else { 0.0 });
        if tpimfi_fw {
            b.set(VEBUS, P_TPIMFI, 0.0); // present-and-valid => firmware supports it
        }
        b
    }

    #[test]
    fn each_source_gates_correctly_no_force_charge() {
        let fc = ForceChargeCtx::default(); // must_force_charge = false
        // Grid / Shore / Island / Unknown => grid-like Normal: no cap, export allowed.
        for src in [Some(1), Some(3), Some(240), Some(0), None] {
            let bus = bus_with(src, true, true);
            let out = Generator::new(&bus).tick(&bus, fc);
            assert!(!out.is_genset, "src={src:?}");
            assert!(!out.force_tpimfi, "src={src:?}");
            assert!(!out.disable_export, "src={src:?}");
            assert_eq!(out.mode, FeedMode::Normal, "src={src:?}");
        }
        // Genset (DVCC + fw) => clean TPIMFI: force cap on, export disabled.
        let bus = bus_with(Some(2), true, true);
        let out = Generator::new(&bus).tick(&bus, fc);
        assert!(out.is_genset);
        assert!(out.force_tpimfi);
        assert!(out.disable_export);
        assert_eq!(out.mode, FeedMode::Tpimfi);
    }

    #[test]
    fn missing_source_read_is_safe_grid_like_default() {
        // No /Ac/ActiveIn/Source key at all => Unknown => grid-like, never a forced cap.
        let bus = bus_with(None, true, true);
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default());
        assert_eq!(g.source(), AcSource::Unknown);
        assert!(!out.is_genset && !out.force_tpimfi && !out.disable_export);
    }

    #[test]
    fn genset_forces_tpimfi_and_disables_export() {
        let bus = bus_with(Some(2), true, true);
        let out = Generator::new(&bus).tick(&bus, ForceChargeCtx::default());
        assert!(out.force_tpimfi, "genset with DVCC+fw must force TPIMFI");
        assert!(out.disable_export, "genset must never allow export tracking");
    }

    #[test]
    fn genset_never_enters_normal_even_when_not_force_charging() {
        // must_force_charge = false would give grid a Normal export mode; a genset must
        // still refuse Normal. Both DVCC/fw combinations checked.
        for (dvcc, fw) in [(true, true), (false, true), (true, false), (false, false)] {
            let bus = bus_with(Some(2), dvcc, fw);
            let out = Generator::new(&bus).tick(&bus, ForceChargeCtx::default());
            assert_ne!(out.mode, FeedMode::Normal, "dvcc={dvcc} fw={fw}");
            assert!(out.disable_export, "dvcc={dvcc} fw={fw}");
        }
    }

    #[test]
    fn genset_without_dvcc_or_fw_degrades_to_charge_hard() {
        // No DVCC, or no firmware support => ChargeHard: no TPIMFI cap, but export still
        // structurally disabled (safe: pure hard charge, feed path shut).
        for (dvcc, fw) in [(false, true), (true, false), (false, false)] {
            let bus = bus_with(Some(2), dvcc, fw);
            let out = Generator::new(&bus).tick(&bus, ForceChargeCtx::default());
            assert_eq!(out.mode, FeedMode::ChargeHard, "dvcc={dvcc} fw={fw}");
            assert!(!out.force_tpimfi, "dvcc={dvcc} fw={fw}");
            assert!(out.disable_export, "dvcc={dvcc} fw={fw}");
        }
    }

    #[test]
    fn grid_force_charge_can_reach_tpimfi_but_is_not_genset() {
        // Grid + force-charge + DVCC/fw + no excess => Tpimfi, yet is_genset stays false.
        let bus = bus_with(Some(1), true, true);
        let fc = ForceChargeCtx {
            must_force_charge: true,
            feed_in_excess_allowed: false,
            shore_limit_saturated: false,
        };
        let out = Generator::new(&bus).tick(&bus, fc);
        assert!(!out.is_genset);
        assert_eq!(out.mode, FeedMode::Tpimfi);
        assert!(out.force_tpimfi && out.disable_export);
    }

    #[test]
    fn grid_force_charge_with_excess_is_charge_hard() {
        let bus = bus_with(Some(1), true, true);
        let fc = ForceChargeCtx {
            must_force_charge: true,
            feed_in_excess_allowed: true, // sellable excess => ChargeHard, not TPIMFI
            shore_limit_saturated: false,
        };
        let out = Generator::new(&bus).tick(&bus, fc);
        assert_eq!(out.mode, FeedMode::ChargeHard);
        assert!(!out.force_tpimfi);
    }

    #[test]
    fn sticky_genset_holds_through_a_dropped_read_but_yields_to_explicit_change() {
        let mut b = bus_with(Some(2), true, true);
        let mut g = Generator::new(&b);
        assert_eq!(g.source(), AcSource::Genset);
        // Dropped read (None): must HOLD Genset, never flip to grid-like for a tick.
        b.clear(SYS, P_ACIN_SOURCE);
        let out = g.tick(&b, ForceChargeCtx::default());
        assert_eq!(g.source(), AcSource::Genset);
        assert!(out.is_genset && out.disable_export);
        // Explicit change to grid (1) IS honoured — a real source change takes effect.
        b.set(SYS, P_ACIN_SOURCE, 1.0);
        let out = g.tick(&b, ForceChargeCtx::default());
        assert_eq!(g.source(), AcSource::Grid);
        assert!(!out.is_genset);
    }

    // ---- flag writes (write-if-changed), via a local recording bus -------------

    /// Records set_i32 writes; reads back through an inner MockBus.
    struct RecBus {
        inner: MockBus,
        writes: RefCell<HashMap<(String, String), i32>>,
    }
    impl RecBus {
        fn new(inner: MockBus) -> Self {
            RecBus { inner, writes: RefCell::new(HashMap::new()) }
        }
        fn last(&self, svc: &str, path: &str) -> Option<i32> {
            self.writes.borrow().get(&(svc.into(), path.into())).copied()
        }
        fn count(&self) -> usize {
            self.writes.borrow().len()
        }
    }
    impl Bus for RecBus {
        fn get_f64(&self, s: &str, p: &str) -> Option<f64> {
            self.inner.get_f64(s, p)
        }
        fn get_str(&self, s: &str, p: &str) -> Option<String> {
            self.inner.get_str(s, p)
        }
        fn list_services(&self, prefix: &str) -> Vec<String> {
            self.inner.list_services(prefix)
        }
        fn set_i32(&self, s: &str, p: &str, v: i32) -> Result<(), String> {
            self.writes.borrow_mut().insert((s.into(), p.into()), v);
            Ok(())
        }
        fn set_f64(&self, _: &str, _: &str, _: f64) -> Result<(), String> {
            Ok(())
        }
    }

    #[test]
    fn genset_flags_written_tpimfi_on_donotfeedin_off() {
        let bus = RecBus::new(bus_with(Some(2), true, true));
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default());
        g.write_feedin_flags(&bus, &out, &FlagInputs::default());
        // Clean genset (Tpimfi): cap ON, overvoltage feed-in ALLOWED (it's the bounded
        // transport for DC-PV excess in mode 0), feed path open.
        assert_eq!(bus.last(VEBUS, P_TPIMFI), Some(1));
        assert_eq!(bus.last(VEBUS, P_DONOTFEEDIN), Some(0));
        assert_eq!(bus.last(VEBUS, P_DISABLEFEEDIN), Some(0));
    }

    #[test]
    fn genset_charge_hard_blocks_overvoltage_and_feedin() {
        // Genset without DVCC => ChargeHard: DC-PV excess must NOT reach the generator.
        let bus = RecBus::new(bus_with(Some(2), false, true));
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default());
        // can_discharge stays false (default) => DisableFeedIn = 1.
        g.write_feedin_flags(&bus, &out, &FlagInputs::default());
        assert_eq!(bus.last(VEBUS, P_TPIMFI), Some(0)); // fw present => written 0
        assert_eq!(bus.last(VEBUS, P_DONOTFEEDIN), Some(1)); // blocked (mode != Tpimfi)
        assert_eq!(bus.last(VEBUS, P_DISABLEFEEDIN), Some(1)); // !can_discharge
    }

    #[test]
    fn genset_permissive_overvoltage_branch_is_gated_off() {
        // Even with OvervoltageFeedIn setting on, a genset can NEVER take the permissive
        // DoNotFeedInOvervoltage=0 branch except via mode 0. Here ChargeHard => blocked.
        let bus = RecBus::new(bus_with(Some(2), false, true));
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default());
        let fi = FlagInputs { overvoltage_feedin: true, excess: FeedInExcess::Force, ..Default::default() };
        g.write_feedin_flags(&bus, &out, &fi);
        assert_eq!(bus.last(VEBUS, P_DONOTFEEDIN), Some(1)); // still blocked on genset
    }

    #[test]
    fn grid_normal_overvoltage_follows_setting() {
        let bus = RecBus::new(bus_with(Some(1), true, true));
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default()); // Normal
        // Grid + OvervoltageFeedIn on + UseSetting => permissive => DoNotFeedIn = 0.
        let fi = FlagInputs { overvoltage_feedin: true, ..Default::default() };
        g.write_feedin_flags(&bus, &out, &fi);
        assert_eq!(bus.last(VEBUS, P_TPIMFI), Some(0));
        assert_eq!(bus.last(VEBUS, P_DONOTFEEDIN), Some(0));
    }

    #[test]
    fn tpimfi_not_written_when_firmware_absent() {
        // No /Hub4/TargetPowerIsMaxFeedIn on the bus => never write it (genset then
        // relies on DisableFeedIn/DoNotFeedIn=1 to block back-feed).
        let bus = RecBus::new(bus_with(Some(2), true, false)); // fw absent
        let mut g = Generator::new(&bus);
        assert!(!g.tpimfi_supported());
        let out = g.tick(&bus, ForceChargeCtx::default());
        assert_eq!(out.mode, FeedMode::ChargeHard); // no fw => degrades
        g.write_feedin_flags(&bus, &out, &FlagInputs::default());
        assert_eq!(bus.last(VEBUS, P_TPIMFI), None); // never written
        assert_eq!(bus.last(VEBUS, P_DONOTFEEDIN), Some(1));
    }

    #[test]
    fn writes_are_suppressed_when_unchanged() {
        let bus = RecBus::new(bus_with(Some(2), true, true));
        let mut g = Generator::new(&bus);
        let out = g.tick(&bus, ForceChargeCtx::default());
        g.write_feedin_flags(&bus, &out, &FlagInputs::default());
        let n = bus.count();
        assert!(n > 0);
        // Second identical pass must not re-write anything (write-if-changed).
        let before = bus.writes.borrow().clone();
        g.write_feedin_flags(&bus, &out, &FlagInputs::default());
        assert_eq!(*bus.writes.borrow(), before, "no-op writes must be suppressed");
    }
}
