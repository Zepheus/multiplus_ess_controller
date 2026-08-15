//! Shore / genset AC-input current limiter (peak-shaving), subsystem #7.
//!
//! Ports the stock ESS loop's `` shore/peak-shave tick. the stock ESS loop never
//! shaves in software: it dynamically *overrides* the Multi's VE.Bus firmware
//! AC-input current limit, per phase, by writing `/Hub4/L{n}/OverruledShoreLimit`
//! (Amps) on the vebus service. The firmware then PowerAssists from the battery to
//! keep the current under that limit. We decide the limit; the firmware does the
//! shaving. See the design notes (its §6 is the drop-in spec).
//!
//! Two behaviours, both implemented here:
//!   * DORMANT (this install): `Settings/CGwacs/AcInputLimit = -1` -> the getter is
//!     NaN, the per-phase write loop is skipped, and `OverruledShoreLimit` is NEVER
//!     touched. A no-op is behaviourally exact (proved in spec §5). `tick` returns
//!     `active=false` with every phase `None` (leave the path alone), matching
//!     the stock ESS loop which does not clear it.
//!   * ACTIVE (a breaker/inlet limit is configured): the first-order IIR
//!         state[n] = 0.8*old + 0.2*((AcInputLimit - meter_I[n]) + multi_I[n])
//!     converges the written limit toward the configured Amps, gently slewed so the
//!     firmware limit never steps. We write `max(0, state)` to impose, or the
//!     `3270.0` "unlimited" sentinel to release (charging / force-charge /
//!     below-min-SOC / non-physical negative state).
//!
//! Constants (0.8/0.2 IIR, 3270 sentinel) are read straight from the reference implementation
//! `.rodata` (spec §4.1). `AcInputLimit` is in AMPS (−1 = unlimited).
//!
//! SAFETY: when `AcInputLimit` is unset / −1 / NaN we do NOT impose a limit and we
//! do NOT clamp AC-in to 0 — we leave `OverruledShoreLimit` untouched, exactly as
//! the stock ESS loop does. The impose value is `max(0, state)` so a written limit is never
//! negative; a state that goes negative triggers a *release* (sentinel), never a
//! non-physical negative current limit reaching the firmware.

use crate::dbus::*;

// --- dbus paths (NEW, local to this subsystem) ---
/// AC-input (shore/genset) current limit. AMPS; −1 = unlimited (getter -> NaN).
pub const P_AC_INPUT_LIMIT: &str = "/Settings/CGwacs/AcInputLimit";
/// AC-export current limit. AMPS (−1 = unlimited); NOT consumed by this filter — its
/// getter is structurally identical to AcInputLimit's (default −1, NaN-if-negative),
/// so it is almost certainly the export-direction AC-in *current* limit in Amps. Kept
/// here only so no caller mistakes it for `CGwacs/MaxFeedInPower` (Watts, subsystem #4).
pub const P_AC_EXPORT_LIMIT: &str = "/Settings/CGwacs/AcExportLimit";
/// Always-peak-shave flag (int bool 0/1). Selects the firmware "always vs above-min-SOC"
/// policy; here it feeds `should_release`.
pub const P_ALWAYS_PEAKSHAVE: &str = "/Settings/CGwacs/AlwaysPeakShave";

// --- IIR constants (spec §4.1, read from .rodata) ---
const IIR_RETAIN: f64 = 0.8; // DAT_00046988 — retain coefficient (old value)
const IIR_UPDATE: f64 = 0.2; // DAT_00046978 — update coefficient (new error)
/// DAT_00046980 — the "unlimited" sentinel written on release. Opaque magic constant;
/// treated as effectively-infinite, never computed.
pub const SHORE_UNLIMITED: f64 = 3270.0;

const MAX_PHASES: usize = 3;

/// `/Hub4/L{phase}/OverruledShoreLimit` (double, Amps) on the vebus service.
pub fn p_overruled(phase: u8) -> String {
    format!("/Hub4/L{phase}/OverruledShoreLimit")
}
/// `/Ac/ActiveIn/L{phase}/I` (double, Amps) — the Multi's reported AC-in current.
pub fn p_active_in_i(phase: u8) -> String {
    format!("/Ac/ActiveIn/L{phase}/I")
}

/// The per-phase decision for one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShoreWrite {
    /// Impose the filtered limit (Amps, already `max(0, state)` — never negative).
    Impose(f64),
    /// Release: write the `3270.0` "unlimited" sentinel.
    Release,
}

impl ShoreWrite {
    /// The f64 actually written to `OverruledShoreLimit`: the Amps for `Impose`, or
    /// the sentinel for `Release`.
    pub fn value(&self) -> f64 {
        match self {
            ShoreWrite::Impose(a) => *a,
            ShoreWrite::Release => SHORE_UNLIMITED,
        }
    }
}

/// Output of one shore tick.
///
/// `active` distinguishes the two behaviours: `true` while the subsystem is actually
/// managing the limit (a finite `AcInputLimit` is configured AND we are the metering
/// source); `false` when dormant (no limit / external meter) — in which case every
/// `writes` entry is `None` and no path is touched.
#[derive(Default)]
pub struct ShoreOut {
    /// Resolved value for L1 in Amps — the filtered limit when imposing, or the
    /// `3270.0` sentinel when releasing. NaN means "L1 left untouched" (dormant, or no
    /// L1 data this tick). Provided for the single-phase target (only L1 exists);
    /// multiphase callers use `writes`.
    pub overruled_shore_limit_a: f64,
    /// True while the subsystem actively manages the limit; false = dormant/untouched.
    pub active: bool,
    /// Full per-phase decision, index 0 = L1. `None` = leave that dbus path untouched
    /// (matches the stock ESS loop, which never clears `OverruledShoreLimit`).
    pub writes: Vec<Option<ShoreWrite>>,
}

/// The shore/peak-shave limiter. Holds the per-phase IIR filter memory across ticks
/// (the state a faithful reimplementation MUST persist — like the battery broker holds
/// its latches).
pub struct ShoreLimiter {
    /// Per-phase IIR state (Amps), L1..L3. `None` = not yet seeded (treated as 0.0,
    /// spec's `DAT_00046970` filter seed). Persistent across ticks and across
    /// dormant/active transitions, exactly like the stock ESS loop's QVector.
    filt: [Option<f64>; MAX_PHASES],
    /// Active phase count (1 for this single-phase install; only L1 exists).
    phases: u8,
}

impl ShoreLimiter {
    pub fn new(phases: u8) -> Self {
        let phases = phases.clamp(1, MAX_PHASES as u8);
        ShoreLimiter { filt: [None; MAX_PHASES], phases }
    }

    /// True if any seeded per-phase state has gone negative — a non-physical current
    /// limit, which forces a release (spec §4.3).
    pub fn any_phase_negative(&self) -> bool {
        self.filt.iter().flatten().any(|v| *v < 0.0)
    }

    /// One control tick (call once per ~1 Hz loop, same cadence as the grid setpoint).
    ///
    /// * `ac_input_limit_a` — the `CGwacs/AcInputLimit` setting in Amps, or `None` if
    ///   unset / −1 / NaN (see [`read_ac_input_limit`]). `None` -> dormant.
    /// * `meter_present` — is an external grid meter metering AC-in? If so the loop is
    ///   skipped (spec §4.4, Medium confidence — verify live before trusting it).
    /// * `meter_i[n]` — per-phase measured AC-in current (A, signed): the grid meter,
    ///   or the Multi self-meter when there is no external meter. `None` = unavailable.
    /// * `multi_i[n]` — the Multi's `/Ac/ActiveIn/Ln/I` (A). `None` = unavailable.
    /// * `release` — the impose-vs-release decision (see [`should_release`]): `true`
    ///   writes the sentinel, `false` imposes the filtered limit.
    ///
    /// Returns per-phase writes; `None` for a phase means "leave the path untouched".
    pub fn tick(
        &mut self,
        ac_input_limit_a: Option<f64>,
        meter_present: bool,
        meter_i: &[Option<f64>],
        multi_i: &[Option<f64>],
        release: bool,
    ) -> ShoreOut {
        let n_phases = self.phases as usize;
        let writes: Vec<Option<ShoreWrite>> = vec![None; n_phases];

        // Gate 1 (SAFETY): no limit configured (unset / −1 / NaN) -> DORMANT. The write
        // loop never runs; `OverruledShoreLimit` is left untouched. We must NOT fall
        // through to imposing anything here, or we could clamp AC-in by mistake.
        let limit = match ac_input_limit_a {
            Some(l) if l.is_finite() && l >= 0.0 => l,
            _ => return ShoreOut { overruled_shore_limit_a: f64::NAN, active: false, writes },
        };

        // Gate 2: an external grid meter is metering AC-in -> loop skipped, untouched.
        if meter_present {
            return ShoreOut { overruled_shore_limit_a: f64::NAN, active: false, writes };
        }

        let mut writes = writes;
        for n in 0..n_phases {
            // Missing/non-finite data this tick -> leave THIS phase untouched, keep its
            // filter memory intact (do not seed it with garbage).
            let mi = meter_i.get(n).copied().flatten();
            let vi = multi_i.get(n).copied().flatten();
            let (mi, vi) = match (mi, vi) {
                (Some(mi), Some(vi)) if mi.is_finite() && vi.is_finite() => (mi, vi),
                _ => continue,
            };

            let old = self.filt[n].filter(|v| v.is_finite()).unwrap_or(0.0);
            // first-order IIR: meter_I and multi_I are the same physical
            // current from two sensors, so error ≈ limit + tracking gap; state slews
            // toward the configured Amps.
            let error = (limit - mi) + vi;
            let next = IIR_RETAIN * old + IIR_UPDATE * error;
            self.filt[n] = Some(next);

            writes[n] = Some(if release {
                ShoreWrite::Release
            } else {
                // Impose: never negative (spec §3 "Value semantics").
                ShoreWrite::Impose(next.max(0.0))
            });
        }

        let l1 = writes
            .first()
            .and_then(|w| w.as_ref())
            .map(ShoreWrite::value)
            .unwrap_or(f64::NAN);
        ShoreOut { overruled_shore_limit_a: l1, active: true, writes }
    }

    /// Live-mode write: apply a tick's decision to the vebus service. Only phases with
    /// `Some(..)` are written; `None` phases are left untouched. The setter writes
    /// unconditionally (no change-suppression), matching the stock ESS loop's ``.
    pub fn write(&self, bus: &dyn Bus, out: &ShoreOut) -> Result<(), String> {
        for (i, w) in out.writes.iter().enumerate() {
            if let Some(w) = w {
                let phase = (i + 1) as u8;
                bus.set_f64(VEBUS, &p_overruled(phase), w.value())?;
            }
        }
        Ok(())
    }

    /// Convenience for `main.rs`: read the meter/Multi currents off the bus, then tick.
    /// For a single Multi with self-metering the meter current *is* `/Ac/ActiveIn/Ln/I`
    /// (spec §6.8.3), so `meter_present=false` and both current vectors come from vebus.
    /// The caller supplies the release decision from the SocGuard/BatteryLife state it
    /// already tracks (subsystem #6).
    pub fn tick_from_bus(
        &mut self,
        bus: &dyn Bus,
        ac_input_limit_a: Option<f64>,
        meter_present: bool,
        always_peak_shave: bool,
        above_min_soc: bool,
        force_charging: bool,
    ) -> ShoreOut {
        let n_phases = self.phases as usize;
        let multi_i: Vec<Option<f64>> = (0..n_phases)
            .map(|n| bus.get_f64(VEBUS, &p_active_in_i(n as u8 + 1)))
            .collect();
        // Single Multi self-metering: the meter current is the same reading as multi_i.
        let meter_i = multi_i.clone();
        let release = should_release(
            always_peak_shave,
            above_min_soc,
            force_charging,
            self.any_phase_negative(),
        );
        self.tick(ac_input_limit_a, meter_present, &meter_i, &multi_i, release)
    }
}

/// Port of ``'s impose-vs-release predicate (spec §6.5). Exact predicate
/// is Medium confidence; this is the safe, behaviour-matching approximation.
/// `true` -> release (write the sentinel); `false` -> impose the filtered limit.
pub fn should_release(
    always_peak_shave: bool,
    above_min_soc: bool,
    force_charging: bool,
    any_phase_negative: bool,
) -> bool {
    if force_charging {
        return true; // charging from grid: don't constrain AC-in
    }
    if !always_peak_shave && !above_min_soc {
        return true; // below min-SOC in the non-"always" policy: release
    }
    any_phase_negative // computed limit non-physical -> release
}

/// Read `CGwacs/AcInputLimit` as Amps, mapping Victron's −1 "unlimited" / any negative
/// / missing / non-finite value to `None` (dormant). The getter (``)
/// returns NaN for a negative setting; we return `None` for the same cases so the
/// caller never imposes a limit when none is configured.
pub fn read_ac_input_limit(bus: &dyn Bus) -> Option<f64> {
    match bus.get_f64(SETTINGS, P_AC_INPUT_LIMIT) {
        Some(l) if l.is_finite() && l >= 0.0 => Some(l),
        _ => None,
    }
}

/// Read the `AlwaysPeakShave` flag (int bool). Missing -> false.
pub fn read_always_peak_shave(bus: &dyn Bus) -> bool {
    bus.get_bool(SETTINGS, P_ALWAYS_PEAKSHAVE).unwrap_or(false)
}

/// Boot-time self-check so the dormant path can't silently diverge if someone sets an
/// AC-input limit. Logs to stderr; does not write.
pub fn warn_if_limit_set(bus: &dyn Bus) {
    match read_ac_input_limit(bus) {
        Some(l) => eprintln!(
            "shore: AcInputLimit={l} A is CONFIGURED -> peak-shave limiter is ACTIVE. \
             Verify the meter_present gate and release policy against a real breaker \
             limit before trusting live OverruledShoreLimit writes."
        ),
        None => eprintln!(
            "shore: AcInputLimit unset/-1 -> dormant; OverruledShoreLimit left untouched \
             (behaviourally exact vs the stock ESS loop)."
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testbus::MockBus;
    use std::cell::RefCell;
    use std::collections::HashMap;

    /// A Bus that records every `set_f64` so live-mode writes can be asserted exactly
    /// (the shared `MockBus` treats writes as no-ops).
    struct RecordingBus {
        f: HashMap<(String, String), f64>,
        writes: RefCell<Vec<(String, String, f64)>>,
    }
    impl RecordingBus {
        fn new() -> Self {
            RecordingBus { f: HashMap::new(), writes: RefCell::new(Vec::new()) }
        }
        fn set(&mut self, svc: &str, path: &str, v: f64) {
            self.f.insert((svc.into(), path.into()), v);
        }
        fn last(&self, svc: &str, path: &str) -> Option<f64> {
            self.writes
                .borrow()
                .iter()
                .rev()
                .find(|(s, p, _)| s == svc && p == path)
                .map(|(_, _, v)| *v)
        }
        fn write_count(&self) -> usize {
            self.writes.borrow().len()
        }
    }
    impl Bus for RecordingBus {
        fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
            self.f.get(&(svc.into(), path.into())).copied()
        }
        fn get_str(&self, _: &str, _: &str) -> Option<String> {
            None
        }
        fn list_services(&self, _: &str) -> Vec<String> {
            Vec::new()
        }
        fn set_i32(&self, _: &str, _: &str, _: i32) -> Result<(), String> {
            Ok(())
        }
        fn set_f64(&self, svc: &str, path: &str, v: f64) -> Result<(), String> {
            self.writes.borrow_mut().push((svc.into(), path.into(), v));
            Ok(())
        }
    }

    // ---- DORMANT: AcInputLimit = -1 -> no-op, path untouched (spec §5) ----

    #[test]
    fn dormant_minus_one_setting_reads_as_none() {
        let mut bus = MockBus::new();
        bus.set(SETTINGS, P_AC_INPUT_LIMIT, -1.0);
        assert_eq!(read_ac_input_limit(&bus), None);
    }

    #[test]
    fn dormant_leaves_every_phase_untouched() {
        let mut sh = ShoreLimiter::new(1);
        // None (unset/-1/NaN) -> dormant regardless of currents.
        let out = sh.tick(None, false, &[Some(10.0)], &[Some(10.0)], false);
        assert!(!out.active);
        assert_eq!(out.writes, vec![None]);
        assert!(out.overruled_shore_limit_a.is_nan());
    }

    #[test]
    fn dormant_write_touches_nothing() {
        let bus = RecordingBus::new();
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(None, false, &[Some(10.0)], &[Some(10.0)], false);
        sh.write(&bus, &out).unwrap();
        assert_eq!(bus.write_count(), 0); // never clears OverruledShoreLimit
    }

    #[test]
    fn nan_limit_is_dormant_not_a_zero_clamp() {
        // SAFETY: a NaN AcInputLimit must NOT impose (which could clamp AC-in near 0).
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(f64::NAN), false, &[Some(30.0)], &[Some(30.0)], false);
        assert!(!out.active);
        assert_eq!(out.writes, vec![None]);
    }

    // ---- ACTIVE: IIR converges toward the configured Amps ----

    #[test]
    fn active_iir_converges_to_configured_amps() {
        // meter_I == multi_I (same physical current, two sensors) => error == limit,
        // so state converges to the configured limit (in Amps).
        let limit = 16.0;
        let load = 10.0;
        let mut sh = ShoreLimiter::new(1);
        let mut last = 0.0;
        for _ in 0..40 {
            let out = sh.tick(Some(limit), false, &[Some(load)], &[Some(load)], false);
            assert!(out.active);
            match out.writes[0] {
                Some(ShoreWrite::Impose(a)) => last = a,
                other => panic!("expected Impose, got {other:?}"),
            }
        }
        assert!((last - limit).abs() < 0.05, "converged to {last}, want {limit}");
        // exposed scalar mirrors the L1 impose value
        let out = sh.tick(Some(limit), false, &[Some(load)], &[Some(load)], false);
        assert!((out.overruled_shore_limit_a - limit).abs() < 0.05);
    }

    /// Extract the imposed Amps from a phase decision, panicking on anything else.
    fn imposed(w: Option<ShoreWrite>) -> f64 {
        match w {
            Some(ShoreWrite::Impose(a)) => a,
            other => panic!("expected Impose, got {other:?}"),
        }
    }

    #[test]
    fn active_first_step_is_the_expected_iir_fraction() {
        // From a cold (unseeded=0) state: state1 = 0.2 * ((16 - 10) + 10) = 0.2*16 = 3.2
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
        assert!((imposed(out.writes[0]) - 3.2).abs() < 1e-9);
    }

    #[test]
    fn state_persists_across_ticks() {
        // Two identical ticks must NOT give the same output — the filter accumulates.
        let mut sh = ShoreLimiter::new(1);
        let a = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
        let b = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
        assert_ne!(a.writes[0], b.writes[0]);
    }

    #[test]
    fn missing_phase_data_leaves_that_phase_untouched() {
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(16.0), false, &[None], &[Some(10.0)], false);
        assert!(out.active); // subsystem is active...
        assert_eq!(out.writes, vec![None]); // ...but this phase had no data
    }

    #[test]
    fn active_writes_amps_to_vebus_l1() {
        let mut bus = RecordingBus::new();
        bus.set(VEBUS, &p_active_in_i(1), 10.0);
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick_from_bus(&bus, Some(16.0), false, false, true, false);
        sh.write(&bus, &out).unwrap();
        // first step from cold: 3.2 A written to /Hub4/L1/OverruledShoreLimit
        let written = bus.last(VEBUS, &p_overruled(1)).expect("L1 written");
        assert!((written - 3.2).abs() < 1e-9, "wrote {written}");
    }

    // ---- meter_present gate (spec §4.4) ----

    #[test]
    fn external_meter_present_skips_write_loop() {
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(16.0), true, &[Some(10.0)], &[Some(10.0)], false);
        assert!(!out.active);
        assert_eq!(out.writes, vec![None]);
    }

    // ---- release conditions: sentinel 3270 ----

    #[test]
    fn release_writes_the_sentinel() {
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], true);
        assert_eq!(out.writes[0], Some(ShoreWrite::Release));
        assert_eq!(out.writes[0].unwrap().value(), SHORE_UNLIMITED);
        assert_eq!(out.overruled_shore_limit_a, SHORE_UNLIMITED);
    }

    #[test]
    fn release_still_advances_the_filter() {
        // Release chooses the sentinel to WRITE, but the IIR state must keep tracking,
        // so a later impose resumes from the right place (matches the stock ESS loop: the
        // filter update precedes the impose/release choice).
        let mut sh = ShoreLimiter::new(1);
        let _ = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], true);
        let out = sh.tick(Some(16.0), false, &[Some(10.0)], &[Some(10.0)], false);
        // second step from state=3.2: 0.8*3.2 + 0.2*16 = 5.76
        assert!((imposed(out.writes[0]) - 5.76).abs() < 1e-9);
    }

    #[test]
    fn force_charge_releases() {
        assert!(should_release(false, true, true, false));
        assert!(should_release(true, true, true, false)); // even with always-peakshave
    }

    #[test]
    fn below_min_soc_non_always_releases_but_always_holds() {
        assert!(should_release(false, false, false, false)); // below SOC, not "always"
        assert!(!should_release(true, false, false, false)); // "always" keeps shaving
    }

    #[test]
    fn above_min_soc_imposes() {
        assert!(!should_release(false, true, false, false));
    }

    #[test]
    fn negative_state_forces_release() {
        assert!(should_release(true, true, false, true));
        assert!(!should_release(true, true, false, false));
    }

    #[test]
    fn any_phase_negative_tracks_filter_state() {
        let mut sh = ShoreLimiter::new(1);
        // Drive the state negative: limit=0, big positive meter current, zero multi.
        // error = (0 - 50) + 0 = -50 -> state goes negative.
        let _ = sh.tick(Some(0.0), false, &[Some(50.0)], &[Some(0.0)], false);
        assert!(sh.any_phase_negative());
    }

    #[test]
    fn impose_value_never_negative() {
        // Even a negative filter state imposes max(0, state) = 0, never a negative A.
        let mut sh = ShoreLimiter::new(1);
        let out = sh.tick(Some(0.0), false, &[Some(50.0)], &[Some(0.0)], false);
        match out.writes[0] {
            Some(ShoreWrite::Impose(a)) => assert!(a >= 0.0, "imposed {a}"),
            other => panic!("expected Impose, got {other:?}"),
        }
    }

    // ---- multiphase faithfulness ----

    #[test]
    fn three_phase_writes_all_three() {
        let mut sh = ShoreLimiter::new(3);
        let out = sh.tick(
            Some(16.0),
            false,
            &[Some(10.0), Some(11.0), Some(12.0)],
            &[Some(10.0), Some(11.0), Some(12.0)],
            false,
        );
        assert_eq!(out.writes.len(), 3);
        assert!(out.writes.iter().all(|w| matches!(w, Some(ShoreWrite::Impose(_)))));
    }
}
