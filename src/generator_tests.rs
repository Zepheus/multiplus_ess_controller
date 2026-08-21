use super::*;
use crate::testbus::MockBus;

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
