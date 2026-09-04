//! Cross-module golden tests: live-captured sequences and end-to-end regimes
//! replayed through the REAL `socguard::enact` + `loop_core::decide` pipeline.
//! Unit tests live inline at the bottom of each module; this directory holds
//! everything that spans modules.

use crate::control::{AdaptiveCfg, Kind};
use crate::loop_core::testutil::*;
use crate::socguard;
use crate::loop_core::*;
use crate::shore::ShoreOut;
use crate::states::{bl, sysstate};
use crate::Stage;

/// Live capture 2026-08-16 ~08:07 — an Octopus Intelligent Go dispatch: an HA
/// automation raised `BatteryLife/MinimumSocLimit` 10 % → 90 % for one minute.
// (t, grid, reported, soc, sysstate, bl_state, min_soc) — from the live capture.
#[rustfmt::skip]
fn dispatch_rows() -> &'static [(f64, f64, f64, f64, i64, i32, f64)] {
    &[
    (19456.0,   49.9, -1867.0, 78.0, 256, 10, 10.0),
    (19457.0,   32.1, -1835.0, 78.0, 256, 10, 10.0),
    (19458.0,   82.0, -1851.0, 78.0, 256, 10, 10.0),
    (19459.0,   59.6, -1843.0, 78.0, 256, 10, 10.0),
    (19460.0,   49.4, -1838.0, 77.0, 256, 10, 90.0), // raise lands -> our floor hold binds NOW
    (19461.0,   74.4, -1848.0, 77.0, 256, 12, 90.0), // BL 12 leads SystemState
    (19462.0,   54.0, -1820.0, 77.0, 258, 12, 90.0),
    (19465.0, 3178.3,  4648.0, 77.0, 258, 12, 90.0), // charge ramp
    (19470.0, 7015.0,  5097.0, 77.0, 258, 12, 90.0), // ~5 kW charge + 1.9 kW load
    (19500.0, 6881.0,  4871.0, 78.0, 258, 12, 90.0),
    (19520.0, 5423.1,  5090.0, 78.0, 258, 12, 90.0),
    (19530.0, 5311.9,  4984.0, 78.0, 258, 12, 90.0),
    (19531.0, 5293.3,  4987.0, 78.0, 258, 10, 10.0), // BL reverts + floor drops: law resumes
    (19532.0, 5290.9,  4999.0, 78.0, 252, 10, 10.0), // (stock re-took here; we never left)
    (19534.0, 5310.4,  1252.0, 78.0, 252, 10, 10.0), // meter-lag transient
    (19535.0,  786.4,   239.0, 78.0, 252, 10, 10.0),
    (19536.0,  430.6,   -49.0, 78.0, 252, 10, 10.0),
    (19538.0,  191.8,  -200.0, 78.0, 252, 10, 10.0),
    (19540.0,   60.6,  -211.0, 78.0, 256, 10, 10.0),
]
}

/// SYNTHETIC replay through the REAL socguard::enact + decide(): the rows are real
/// telemetry, but they were captured while STOCK owned the loop (from t=19460 to 19532).
/// A raised floor is a dispatch — normal operation inside the margins, not a hand-back —
/// so the rows are fed to a loop that keeps ownership throughout and the test pins what
/// OUR loop writes for them. Two caveats keep this honest: under our ownership systemcalc
/// would report SystemState 252 (not 258/259 — `decide` does not read the state at all),
/// and the 5 kW TPIMF ramp timing at 19462..19470 is the firmware's under stock's writes;
/// ours would differ by a tick or two. The pins:
///   t=19460  the raise lands (BL state still 10): our own floor hold binds THAT tick —
///            soc 77 ≤ 90 — before batterylife/systemcalc publish state 12 / 258. The
///            composed command is the hold (0 = acOut, unread here); the WRITE decays
///            onto it through the stock write EMA: −1903 → −952 → −476 → −238.
///   t=19461  BL state 12 arrives (force-charge + discharge inhibit). The firmware
///            charges at ~5 kW on its own (TPIMF: our write is a feed-in cap, capped at
///            the Σ max-charge which is unread here, so the raw law passes): +618 on
///            the ramp tick where grid lags the charge step, then a positive decay
///            309 → 155 → 78 → 39 with the composed command pinned at 0 throughout.
///   t=19531  BL reverts to 10 and the floor drops: the normal law (−301) resumes on
///            that very tick despite SystemState 258, write −131; no mode flap anywhere.
///   t=19534  one-tick meter lag (reported collapses 4999 → 1252 while grid is still
///            5.3 kW): raw law −4053. The stock write EMA absorbs it — prev −209 →
///            write −2131 — and the W/s slew guard is not needed.
#[test]
fn golden_octopus_dispatch_minsoc_raise_sequence() {
let rows = dispatch_rows();
// Device-derived safety numbers (envelope 7030 W -> slew 3515 W/s), as live.
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Takeover,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: f64::INFINITY, // captured pre-feature: err-gain pacing off
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    // Captures predate the Smith compensation - replay without it.
    smith: false,
};
let mut st = state(Kind::Stock);
st.owner_us = true; // had owned for hours before the dispatch
st.last_flip_t = 19000.0;
let mut last_setpoint: Option<f64> = None;

for &(t, grid, reported, soc, sysstate, bl, min_soc) in rows {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl,
        min_soc,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: false, // stayed 0 the whole event (watcher-verified)
        ovr_max_discharge_w: f64::NAN, // idem
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate },
        dt: 1.0,
        dt_clamped: false,
        min_soc,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: f64::NAN,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);

    assert!(d.owner_us, "t={t}: a dispatch never costs ownership");
    assert!(!d.flipped, "t={t}: no ownership flip anywhere in the sequence");
    assert_eq!(d.hub4mode, None, "t={t}: no mode flapping");
    assert!(matches!(d.write, Write::Setpoint(_)), "t={t}: we keep driving throughout");
    assert!(!d.safety.sanity_tripped, "t={t}: a dispatch is not a fault");
    let written = match d.write {
        Write::Setpoint(v) => v,
        w => panic!("t={t}: expected a setpoint write, got {w:?}"),
    };
    let pin = |want: f64| assert!((written - want).abs() < 1.0, "t={t}: write {written}, want {want}");
    match t as i64 {
        19459 => pin(-1903.0),
        19460 => {
            assert!(!sg.force_charge, "t={t}: BL still 10 — the floor alone binds");
            assert!(d.command.abs() < 1e-9, "t={t}: our floor hold, cmd {}", d.command);
            pin(-952.0); // −1903 + 0.5·(0 − −1903)
        }
        19461 => pin(-476.0),
        19462 => pin(-238.0),
        19465 => {
            assert!((d.command - 1474.7).abs() < 0.1, "t={t}: ramp-tick law {}", d.command);
            pin(618.0); // −238 + 0.5·(1474.7 − −238)
        }
        19470..=19530 => {
            assert!(sg.force_charge && sg.tpimf, "t={t}: BL-12 regime");
            assert_eq!(sg.max_discharge_w, 0.0, "t={t}: BL-12 inhibits discharge");
            // Steady charge: law is negative, discharge-inhibited to 0 (matches the
            // captured telemetry cmd = -0.0 on every one of these rows); the write
            // halves toward it each row: 309, 155, 78, 39.
            assert!(d.command.abs() < 1e-9, "t={t}: cmd {}", d.command);
            let prev = last_setpoint.expect("writing throughout");
            assert!(written >= 0.0 && written <= prev, "t={t}: {prev} -> {written} must decay toward the hold");
            if t == 19530.0 {
                pin(39.0);
            }
        }
        19531 => {
            assert!(!sg.force_charge, "t={t}: BL already back to 10");
            assert!((d.command - (-301.3)).abs() < 0.1, "t={t}: law resumes, cmd {}", d.command);
            pin(-131.0); // 39 + 0.5·(−301.3 − 39)
        }
        19532 => pin(-209.0),
        19534 => {
            // Meter-lag tick: raw law = 1252 + 5 − 5310.4 = −4053.4. The stock write
            // EMA absorbs it (−209 → −2131), exactly how stock rides out one-tick meter
            // glitches; the W/s slew guard stays as a backstop and must NOT fire here.
            assert!((d.command - (-4053.4)).abs() < 0.1, "t={t}: raw law {}", d.command);
            pin(-2131.0);
            assert!(!d.safety.slew_clamped,
                "t={t}: the EMA damping must keep the transient under the slew guard");
        }
        _ => {}
    }
    if let Write::Setpoint(v) = d.write {
        last_setpoint = Some(v);
    }
}
// Grid was back near target by the last rows: no sanity trip may be pending.
assert!(st.oob_since.is_none(), "recovered grid must clear the out-of-band timer");
assert!(st.owner_us, "we own throughout the dispatch");
assert_eq!(st.sanity_trips, 0, "the dispatch must not count as a sanity trip");
}

/// The same rows under the LIVE pacing config (`--errgain-dead-w 300`, slope 2000, cap 1.0):
/// multi-kW errors give the write EMA full authority, so the dispatch edges are STEPS, not
/// decays, and the meter-lag tick at 19534 IS caught by the W/s slew guard (−287 → −3802,
/// 3515 W/s × dt 1 s), exactly the `slew-clamped` line the service log shows on such ticks.
/// Both pictures are pinned so nobody mistakes one config's behaviour for the other's.
#[test]
fn golden_octopus_dispatch_minsoc_raise_sequence_live_pacing() {
let rows = dispatch_rows();
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Takeover,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: 300.0,
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    smith: false,
};
let mut st = state(Kind::Stock);
st.owner_us = true;
st.last_flip_t = 19000.0;
for &(t, grid, reported, soc, sysstate, bl, min_soc) in rows {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl,
        min_soc,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: false,
        ovr_max_discharge_w: f64::NAN,
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate },
        dt: 1.0,
        dt_clamped: false,
        min_soc,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: f64::NAN,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);
    assert!(d.owner_us && !d.flipped && d.hub4mode.is_none(), "t={t}: ownership kept, no flap");
    assert!(!d.safety.sanity_tripped, "t={t}: a dispatch is not a fault");
    let written = match d.write {
        Write::Setpoint(v) => v,
        w => panic!("t={t}: expected a setpoint write, got {w:?}"),
    };
    let pin = |want: f64| assert!((written - want).abs() < 1.0, "t={t}: write {written}, want {want}");
    match t as i64 {
        19460 => {
            // err 44 W: inside the deadband, base 0.5 gain — the hold still decays here.
            assert!(d.command.abs() < 1e-9);
            pin(-952.0);
            assert!(!d.safety.slew_clamped);
        }
        19465 => {
            // err 3173 W: full authority — the ramp-tick law is written as is.
            pin(1475.0);
            assert!(!d.safety.slew_clamped);
        }
        19470..=19530 => {
            assert!(d.command.abs() < 1e-9);
            pin(0.0); // full authority: parked exactly on the hold
        }
        19531 => pin(-301.0),
        19532 => pin(-287.0),
        19534 => {
            // err 5305 W: full authority asks for the raw −4053 from −287 — a 3766 W step,
            // beyond the 3515 W/s guard, which binds: this is the live `slew-clamped`.
            pin(-287.0 - sl.slew_w_per_s);
            assert!(d.safety.slew_clamped, "t={t}: the slew guard is what bounds this tick live");
        }
        _ => {}
    }
}
assert!(st.owner_us && st.sanity_trips == 0);
}
/// Live capture 2026-08-17 — the full scheduled-charge window edge sequence (SystemState
/// 259), replayed row-for-row through the REAL socguard::enact + decide(). Two boundary
/// behaviors of the systemcalc scheduler that pure steady-state tests can't see:
///
///   ENTRY (t=35577): the delegate briefly writes BOTH `/Overrides/ForceCharge`=1 AND
///   `/Overrides/MaxDischargePower`=1 for ~5 s before clearing the cap. The merge must
///   be most-restrictive on discharge (law −462 W → −1 W) while positive/charge values
///   pass untouched (+367 W two ticks later).
///
///   TARGET-SOC HOLD TRANSITION (t=54389..54394): ForceCharge drops to 0 the tick the
///   target SOC is reached, but MaxDischargePower=1 lands only ~6 s LATER — an
///   unguarded gap in which the raw law transiently commands −1.6/−2.1 kW (the grid
///   meter still shows the collapsing 2.3 kW charge). Stock the stock daemon coasts through
///   it on its own write smoothing (−192 W held). We pass the raw law through — a
///   KNOWN, bounded divergence (≈2 ticks, ≲1 Wh, well inside the slew envelope) that
///   this test pins so any future "fix" is a deliberate decision, not drift.
#[test]
fn golden_schedule_window_edges_dual_override_and_hold_gap() {
// (t, grid, reported, soc, sysstate, ovr_forcecharge, ovr_maxdischarge) — live rows.
#[rustfmt::skip]
let rows: &[(f64, f64, f64, f64, i64, bool, f64)] = &[
    (35575.0,   83.0,  -372.0, 58.0, 256, false, f64::NAN), // pre-window, normal ESS
    (35577.0,   80.0,  -387.0, 58.0, 256, true,  1.0),      // BOTH overrides land
    (35578.0,   75.0,  -400.0, 58.0, 259, true,  1.0),
    (35580.0,   57.0,   419.0, 58.0, 259, true,  1.0),      // law swings positive
    (35582.0, 1240.0,  2712.0, 58.0, 259, true,  f64::NAN), // cap cleared; charging
    (54388.0, 2352.0,  2129.0, 90.0, 259, true,  f64::NAN), // last force-charge tick
    (54389.0, 2340.0,  2131.0, 90.0, 259, false, f64::NAN), // fc drops — gap begins
    (54390.0, 2329.0,  2133.0, 90.0, 259, false, f64::NAN),
    (54391.0, 2346.0,   770.0, 90.0, 259, false, f64::NAN), // law −1571 W (transient)
    (54393.0, 2352.0,   240.0, 90.0, 259, false, f64::NAN), // law −2107 W (transient)
    (54394.0,  263.0,    70.0, 90.0, 259, false, f64::NAN), // meter settled: law −188
    (54395.0,  212.0,   -25.0, 90.0, 259, false, 1.0),      // hold lands — gap over
    (54396.0,  198.0,   -37.0, 90.0, 259, false, 1.0),
    (54398.0,  185.0,   -45.0, 90.0, 259, false, 1.0),
];
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Takeover,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: f64::INFINITY, // captured pre-feature: err-gain pacing off
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    // Captures predate the Smith compensation - replay without it.
    smith: false,
};
let mut st = state(Kind::Stock);
st.owner_us = true;
st.last_flip_t = 35000.0;

for &(t, grid, reported, soc, sysstate, ovr_fc, ovr_maxdis) in rows {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl::SOCG_DEFAULT, // BatteryLife stays in its normal regime; this path is pure overrides
        min_soc: 10.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: ovr_fc,
        ovr_max_discharge_w: ovr_maxdis,
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate },
        dt: 1.0,
        dt_clamped: false,
        min_soc: 10.0,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: f64::NAN,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);

    assert!(d.owner_us && !d.flipped, "t={t}: the window must not disturb ownership");
    assert_eq!(d.hub4mode, None, "t={t}: no mode writes mid-window");
    assert!(matches!(d.write, Write::Setpoint(_)), "t={t}: takeover keeps writing");

    match t as i64 {
        35575 => {
            assert!(!sg.force_charge, "t={t}: pre-window");
            assert!((d.command - (-450.0)).abs() < 3.0, "t={t}: normal law, got {}", d.command);
        }
        35577 | 35578 => {
            // Dual override: charge forced AND discharge capped at 1 W, same ticks.
            assert!(sg.force_charge && sg.tpimf, "t={t}: force-charge regime");
            assert_eq!(sg.max_discharge_w, 1.0, "t={t}: entry cap present");
            assert!((d.command - (-1.0)).abs() < 1e-9,
                "t={t}: law −462 must clamp to the 1 W cap, got {}", d.command);
        }
        35580 => {
            assert!((d.command - 367.0).abs() < 3.0,
                "t={t}: positive law passes the discharge cap, got {}", d.command);
        }
        35582 => {
            assert!(sg.max_discharge_w.is_infinite(), "t={t}: entry cap cleared");
            assert!((d.command - 1477.0).abs() < 3.0,
                "t={t}: TPIMF raw-law passthrough, got {}", d.command);
        }
        54388 => {
            assert!(sg.force_charge, "t={t}: still force-charging at target SOC");
            assert!((d.command - (-218.0)).abs() < 3.0, "t={t}: got {}", d.command);
        }
        54391 => {
            // The gap: no force-charge, no cap yet, meter mid-collapse.
            assert!(!sg.force_charge && sg.max_discharge_w.is_infinite(), "t={t}");
            assert!((d.command - (-1571.0)).abs() < 3.0,
                "t={t}: pinned known transient, got {}", d.command);
        }
        54393 => {
            assert!((d.command - (-2107.0)).abs() < 3.0,
                "t={t}: pinned known transient, got {}", d.command);
            // Bounded by the slew guard's envelope even at its worst.
            if let Write::Setpoint(v) = d.write {
                assert!(v >= -7030.0, "t={t}: inside envelope");
            }
        }
        54395..=54398 => {
            assert_eq!(sg.max_discharge_w, 1.0, "t={t}: hold active");
            // acOut was not captured in this telemetry (NaN → bare cap). With it,
            // the hold floats at −1 + acOut ≈ +30 — the value stock wrote here.
            assert!((d.command - (-1.0)).abs() < 1e-9,
                "t={t}: hold pins the command at −1 W (bare cap), got {}", d.command);
        }
        _ => {}
    }
}
assert_eq!(st.sanity_trips, 0, "window edges must not trip sanity");
assert!(st.owner_us, "still owning after the window");
}
/// Live capture 2026-08-18 — the window-tail transition replayed TICK-EXACTLY through
/// the full write path (feedforward + EMA + guards): force-charge end, the ~4 s
/// override gap, the hold landing, and convergence onto the −1 + acOut floor. Since
/// the shadow telemetry `command` column IS the simulated write, every row must
/// reproduce to the watt — this pins the whole composed pipeline (raw law → acOut
/// floor lift → flat-0.5 EMA → half-away-from-zero integer rounding) in one shot.
/// Includes the subtle edges seen live: rounding at exactly .5 (−75.5 → −76,
/// 2.5 → 3, 31.5 → 32) and NEGATIVE measured acOut (−16/−20 W) pushing the floor
/// slightly below −1.
/// 2026-08-25 overnight: the FIRST full 5.26 h scheduled-charge window under the
/// integral-freeze fix, replayed tick-exact across the window EXIT — the exact
/// scenario of the fixed window-integral bug. Pins that (1) the frozen-FF + Smith
/// live config reproduces the recorded writes, and (2) the exit is a bounded
/// ~10 s plant-lag transient that settles to the MaxDischargePower=1 hold value,
/// with NO sustained export tail (the bug railed ~-300 W for minutes).
#[test]
fn golden_charge_window_exit_overnight_2026_08_25() {
// (t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, recorded_write)
type Row = (f64, f64, f64, f64, bool, f64, f64, f64);
#[rustfmt::skip]
let rows: &[Row] = &[
    (55922.0, 2407.5, 2172.0, 89.0, true,  f64::NAN, 10.0, -338.0),
    (55923.0, 2387.6, 2170.0, 89.0, true,  f64::NAN, 12.0, -331.0),
    (55924.0, 2374.8, 2168.0, 89.0, true,  f64::NAN, 17.0, -324.0),
    (55925.0, 2389.0, 2155.0, 89.0, true,  f64::NAN, 27.0, -340.0),
    (55926.0, 2370.6, 2143.0, 89.0, true,  f64::NAN, 33.0, -337.0),
    (55927.0, 2357.9, 2159.0, 89.0, true,  f64::NAN, 23.0, -319.0),
    (55928.0, 2379.3, 2160.0, 89.0, true,  f64::NAN, 26.0, -333.0),
    (55929.0, 2379.8, 2165.0, 90.0, false, f64::NAN, 23.0, -335.0), // window exit
    (55930.0, 2361.0, 1537.0, 90.0, false, f64::NAN, 33.0, -618.0),
    (55931.0, 1641.6,  262.0, 90.0, false, f64::NAN, 42.0, -905.0),
    (55933.0,  435.6, -311.0, 90.0, false, f64::NAN, 29.0, -603.0),
    (55934.0,  113.8, -1090.0, 90.0, false, f64::NAN, 37.0, -896.0),
    (55935.0, -1210.4, -784.0, 90.0, false, 1.0,     16.0, -440.0), // hold engages
    (55936.0, -586.0, -672.0, 90.0, false, 1.0,      35.0, -203.0),
    (55937.0, -506.7, -559.0, 90.0, false, 1.0,      29.0,  -88.0),
    (55938.0, -420.6, -434.0, 90.0, false, 1.0,       8.0,  -41.0),
    (55939.0, -260.7, -234.0, 90.0, false, 1.0,      16.0,  -13.0),
    (55940.0,  -68.3, -199.0, 90.0, false, 1.0,     -17.0,  -16.0),
    (55941.0,   42.0, -101.0, 90.0, false, 1.0,       9.0,   -4.0),
    (55942.0,   91.7,  -66.0, 90.0, false, 1.0,      25.0,   10.0),
    (55944.0,  118.3,  -53.0, 90.0, false, 1.0,      27.0,   18.0),
    (55945.0,  138.4,  -21.0, 90.0, false, 1.0,      47.0,   32.0),
    (55946.0,  142.9,   -7.0, 90.0, false, 1.0,      57.0,   44.0),
    (55947.0,  148.6,  -10.0, 90.0, false, 1.0,      53.0,   48.0),
    (55948.0,  153.0,   11.0, 90.0, false, 1.0,      60.0,   54.0),
    (55949.0,  160.0,   14.0, 90.0, false, 1.0,      59.0,   56.0),
    (55950.0,  166.3,   19.0, 90.0, false, 1.0,      60.0,   58.0),
];
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Shadow, // real write path, exactly as captured
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: f64::INFINITY, // captured pre-feature: err-gain pacing off
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    smith: true, // live config: Smith on
};
// Live controller: frozen FF at the identified leak curve, ki 0.02.
let mut acfg = crate::control::AdaptiveCfg::tuned();
acfg.mode = crate::control::FfMode::Frozen;
acfg.a0 = -4.9;
acfg.b0 = -0.0458;
acfg.seeded = true;
acfg.ki = 0.02;
let mut st = state(Kind::Adaptive(acfg));
st.owner_us = true;
st.last_flip_t = 0.0;
st.last_out = Some(-338.0); // the write at the seed row (55922)
// Steady mid-window Smith state: w_eff has long converged onto the written cap.
st.smith_weff = -338.0;
st.smith_weff_lag = -338.0;
st.smith_prev_write = -338.0;
// The residual integral as live had it: frozen through the 5.26 h window at the
// pre-window value (solved from the 15 exact hold-regime rows; see diff pattern).
st.ctrl.seed_integral(-17.0);

let mut prev_t = 55921.0;
let mut worst: f64 = 0.0; // over the exit rows; the in-window rows warm up the
                          // reconstructed seeds (EMA/Smith/integral) unasserted
for &(t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, expect) in &rows[1..] {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl::SOCG_DEFAULT,
        min_soc: 10.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: ovr_fc,
        ovr_max_discharge_w: ovr_maxdis,
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate::SCHEDULED_CHARGE },
        dt: t - prev_t,
        dt_clamped: false,
        min_soc: 10.0,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: acout,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);
    if t >= 55929.0 {
        worst = worst.max((d.display_cmd - expect).abs());
    }
    prev_t = t;
}
assert!(worst <= 2.0, "window-exit replay drifted from the live capture: worst {worst:.1} W");
}

/// Drive the FULL decide() write path (EMA + err-gain + cycling detector +
/// Smith) over a synthetic load on the identified plant. Returns the grid
/// trace and whether the cycling regime ever engaged.
fn sim_decide_plant(load: &[f64], errgain: bool) -> (Vec<f64>, bool) {
    use crate::loop_core::plant;
    let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
    let c = DecideCfg {
        stage: Stage::Takeover,
            min_dwell_s: 30.0,
        trim_ki: 0.02,
        orig_mode: 1,
            slew_w_per_s: sl.slew_w_per_s,
        sanity_band_w: sl.sanity_band_w,
        sanity_secs: sl.sanity_secs,
        ema_adaptive: false,
        ema_gain_up: 0.5,
        ema_gain_down: 0.5,
        errgain_dead_w: if errgain { 300.0 } else { f64::INFINITY },
        errgain_slope_w: 2000.0,
        errgain_cap: 1.0,
        inverter_nominal_w: f64::INFINITY,
        smith: true,
    };
    let mut st = state(Kind::Pi { ki: 0.02, i_max: 300.0 });
    st.owner_us = true;
    st.last_flip_t = 0.0;
    let (mut weff, mut pending) = (0.0f64, 0.0f64);
    let mut grid_ring: Vec<f64> = vec![];
    let mut trace = vec![];
    let mut saw_cycling = false;
    for (k, &ld) in load.iter().enumerate() {
        weff += plant::ALPHA * (pending - weff);
        let rep = plant::A + plant::B * weff;
        let grid_true = ld + rep;
        grid_ring.push(grid_true);
        let lag = plant::METER_LAG_S;
        let gm = if grid_ring.len() > lag { grid_ring[grid_ring.len() - 1 - lag] } else { grid_true };
        let sg = socguard::enact(&socguard::Inputs {
            bl_state: bl::SOCG_DEFAULT,
            min_soc: 10.0,
            charge_request: false,
            dc_voltage: 52.0,
            eff_max_charge_w: f64::NAN,
            multi_supports_tpimf: true,
            override_force_charge: false,
            ovr_max_discharge_w: f64::NAN,
        });
        let snap = Snapshot {
            s: Sensors { grid: gm, reported: rep, target: 5.0, soc: 70.0, state: sysstate::DISCHARGING },
            dt: 1.0,
            dt_clamped: false,
            min_soc: 10.0,
            lo: -7030.0,
            hi: 7030.0,
            effective_target: 5.0,
            sg,
            feed_block_charge: false,
            feed_force_flat: false,
            gen_disable_export: false,
            gm_should_write: true,
            shore_out: ShoreOut::default(),
            hub4_actual: None,
            ac_out_w: 20.0,
            hub4mode_live: None,
            dc_batt_w: f64::NAN,
        };
        let d = decide(&snap, &mut st, &c, k as f64);
        saw_cycling |= st.cycling;
        if let Write::Setpoint(v) = d.write {
            pending = v - 20.0; // acOut feed-forward frame back to battery-side
        }
        trace.push(grid_true);
    }
    (trace, saw_cycling)
}

/// Err-gain regime behavior on the identified plant: (a) a modulating (hob-like)
/// load flips the cycling regime and the feature goes NEUTRAL — same exchange as
/// base pacing within a few percent; (b) an ISOLATED step-off arrests strictly
/// faster with the feature on. This pins the whole design contract.
#[test]
fn errgain_hob_neutral_isolated_step_faster() {
    // (a) hob: 300 s of 3 kW square wave, 8 s per phase, after a quiet warm-up.
    let mut hob = vec![400.0f64; 60];
    for k in 0..300 {
        hob.push(if (k / 8) % 2 == 0 { 3400.0 } else { 400.0 });
    }
    let (t_on, cyc_on) = sim_decide_plant(&hob, true);
    let (t_off, _) = sim_decide_plant(&hob, false);
    assert!(cyc_on, "hob square wave must engage the cycling regime");
    let xchg = |tr: &[f64]| -> f64 {
        tr[60..].iter().map(|g| (g - 5.0).abs()).sum::<f64>() / 3600.0
    };
    let (a, b) = (xchg(&t_on), (xchg(&t_off)));
    assert!(
        (a - b).abs() / b < 0.05,
        "err-gain must be neutral on the hob: on {a:.2} Wh vs off {b:.2} Wh"
    );
    // (b) isolated 1.5 kW step-off out of quiet: strictly faster arrest.
    let mut step = vec![1900.0f64; 120];
    step.extend(vec![400.0f64; 60]);
    let (t_on, _) = sim_decide_plant(&step, true);
    let (t_off, _) = sim_decide_plant(&step, false);
    let exp = |tr: &[f64]| -> (f64, usize) {
        let e: f64 = tr[120..].iter().filter(|&&g| g < 0.0).map(|g| -g / 3600.0).sum();
        let d = tr[120..].iter().filter(|&&g| g < -100.0).count();
        (e, d)
    };
    let (e_on, d_on) = exp(&t_on);
    let (e_off, d_off) = exp(&t_off);
    assert!(
        e_on < e_off && d_on <= d_off,
        "err-gain must arrest an isolated step-off faster: on {e_on:.2}Wh/{d_on}s vs off {e_off:.2}Wh/{d_off}s"
    );
}

/// 2026-08-26 live: a ~1.3 kW microwave step-off onto a discharging battery —
/// the isolated-appliance export-arrest case, replayed tick-exact under the live
/// config (symmetric 0.5, Smith, frozen FF, ki 0.02). Pins today's recovery shape
/// (the err-gain feature replays OFF here: the capture predates it).
#[test]
fn golden_microwave_stepoff_2026_08_26() {
// (t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, recorded_write)
type Row = (f64, f64, f64, f64, bool, f64, f64, f64);
#[rustfmt::skip]
let rows: &[Row] = &[
    (45590.0, -54.6, -1728.0, 85.0, false, f64::NAN, 25.0, -1811.0),
    (45591.0, 4.6, -1720.0, 85.0, false, f64::NAN, 27.0, -1827.0),
    (45592.0, 39.4, -1652.0, 85.0, false, f64::NAN, 5.0, -1816.0),
    (45593.0, 98.1, -1694.0, 85.0, false, f64::NAN, 11.0, -1858.0),
    (45595.0, 34.0, -1706.0, 85.0, false, f64::NAN, 13.0, -1843.0),
    (45596.0, 19.1, -1567.0, 85.0, false, f64::NAN, 8.0, -1755.0),
    (45597.0, -1034.6, -1682.0, 85.0, false, f64::NAN, 1.0, -1292.0),
    (45598.0, -1261.8, -1587.0, 85.0, false, f64::NAN, 8.0, -1078.0),
    (45599.0, -1003.1, -1368.0, 85.0, false, f64::NAN, 15.0, -1069.0),
    (45600.0, -749.8, -1164.0, 85.0, false, f64::NAN, -17.0, -927.0),
    (45601.0, -712.6, -1107.0, 85.0, false, f64::NAN, -1.0, -784.0),
    (45602.0, -656.5, -997.0, 85.0, false, f64::NAN, 21.0, -722.0),
    (45603.0, -566.6, -831.0, 85.0, false, f64::NAN, 14.0, -625.0),
    (45605.0, -401.4, -758.0, 85.0, false, f64::NAN, 9.0, -598.0),
    (45606.0, -314.5, -672.0, 85.0, false, f64::NAN, 5.0, -564.0),
    (45607.0, -277.6, -647.0, 85.0, false, f64::NAN, 9.0, -524.0),
    (45608.0, -271.8, -667.0, 85.0, false, f64::NAN, 16.0, -515.0),
    (45609.0, -227.7, -575.0, 85.0, false, f64::NAN, 21.0, -474.0),
    (45610.0, -179.0, -550.0, 85.0, false, f64::NAN, 30.0, -462.0),
    (45611.0, -156.3, -505.0, 85.0, false, f64::NAN, 45.0, -444.0),
    (45612.0, -135.0, -494.0, 85.0, false, f64::NAN, 51.0, -430.0),
    (45613.0, -118.5, -487.0, 85.0, false, f64::NAN, 50.0, -425.0),
    (45614.0, -100.7, -439.0, 85.0, false, f64::NAN, 61.0, -401.0),
    (45616.0, -96.2, -413.0, 85.0, false, f64::NAN, 64.0, -379.0),
    (45617.0, -56.5, -410.0, 85.0, false, f64::NAN, 56.0, -393.0),
    (45618.0, -44.1, -402.0, 85.0, false, f64::NAN, 60.0, -389.0),
    (45619.0, -49.6, -413.0, 85.0, false, f64::NAN, 48.0, -380.0),
    (45620.0, -45.7, -407.0, 85.0, false, f64::NAN, 49.0, -380.0),
    (45621.0, -62.0, -422.0, 85.0, false, f64::NAN, 49.0, -379.0),
    (45622.0, -64.7, -418.0, 85.0, false, f64::NAN, 47.0, -371.0),
    (45623.0, -45.0, -396.0, 85.0, false, f64::NAN, 52.0, -368.0),
    (45624.0, -43.1, -394.0, 85.0, false, f64::NAN, 45.0, -367.0),
    (45626.0, -44.5, -375.0, 85.0, false, f64::NAN, 51.0, -353.0),
    (45627.0, -30.4, -389.0, 85.0, false, f64::NAN, 46.0, -364.0),
    (45628.0, -37.1, -377.0, 85.0, false, f64::NAN, 48.0, -355.0),
    (45629.0, -18.9, -357.0, 85.0, false, f64::NAN, 51.0, -347.0),
    (45630.0, -14.7, -334.0, 85.0, false, f64::NAN, 60.0, -339.0),
    (45631.0, -6.0, -337.0, 85.0, false, f64::NAN, 56.0, -342.0),
    (45632.0, -6.5, -340.0, 85.0, false, f64::NAN, 57.0, -341.0),
    (45633.0, -9.0, -336.0, 85.0, false, f64::NAN, 63.0, -334.0),
    (45634.0, -5.6, -329.0, 85.0, false, f64::NAN, 59.0, -331.0),
    (45635.0, -10.2, -327.0, 85.0, false, f64::NAN, 50.0, -328.0),
    (45636.0, -11.7, -326.0, 85.0, false, f64::NAN, 58.0, -323.0),
    (45638.0, -5.3, -321.0, 85.0, false, f64::NAN, 55.0, -322.0),
    (45639.0, -11.3, -317.0, 85.0, false, f64::NAN, 46.0, -316.0),
    (45640.0, 15.0, -316.0, 85.0, false, f64::NAN, 47.0, -326.0),
];
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Takeover, // the capture is a LIVE takeover: the integral runs
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: f64::INFINITY, // capture predates the feature
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    smith: true,
};
let mut acfg = crate::control::AdaptiveCfg::tuned();
acfg.mode = crate::control::FfMode::Frozen;
acfg.a0 = -4.9;
acfg.b0 = -0.0458;
acfg.seeded = true;
acfg.ki = 0.02;
let mut st = state(Kind::Adaptive(acfg));
st.owner_us = true;
st.last_flip_t = 0.0;
st.last_out = Some(-1811.0);
st.smith_weff = -1811.0;
st.smith_weff_lag = -1811.0;
st.smith_prev_write = -1811.0;
st.ctrl.seed_integral(-32.0); // steady-state residual solved from the pre-event rows

let mut prev_t = 45589.0;
let mut worst: f64 = 0.0; // asserted from the step-off onward; earlier rows warm up seeds
for &(t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, expect) in &rows[1..] {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl::SOCG_DEFAULT,
        min_soc: 10.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: ovr_fc,
        ovr_max_discharge_w: ovr_maxdis,
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate::DISCHARGING },
        dt: t - prev_t,
        dt_clamped: false,
        min_soc: 10.0,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: acout,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);
    if t >= 45597.0 {
        worst = worst.max((d.display_cmd - expect).abs());
    }
    prev_t = t;
}
assert!(worst <= 4.0, "microwave replay drifted from the live capture: worst {worst:.1} W");
}

#[test]
fn golden_window_tail_ema_replay_is_tick_exact() {
// (t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, expected_write) — live rows;
// stock's own writes tracked the same path within its 2.5 s cadence.
type TailRow = (f64, f64, f64, f64, bool, f64, f64, f64);
#[rustfmt::skip]
let rows: &[TailRow] = &[
    (71647.0, 1569.7,  925.0, 89.0, true,  f64::NAN, 79.0, -624.0),
    (71648.0, 1566.4,  926.0, 89.0, true,  f64::NAN, 72.0, -630.0),
    (71649.0, 1569.9,  927.0, 89.0, true,  f64::NAN, 61.0, -634.0),
    (71650.0, 1572.3,  929.0, 90.0, false, f64::NAN, 63.0, -636.0), // fc drops
    (71651.0, 1574.3,  929.0, 90.0, false, f64::NAN, 66.0, -638.0),
    (71652.0, 1573.4,  406.0, 90.0, false, f64::NAN, 22.0, -900.0), // gap: law −1162, EMA halves
    (71653.0,  257.5, -349.0, 90.0, false, f64::NAN, 31.0, -751.0),
    (71654.0, -976.8, -996.0, 90.0, false, 1.0,      13.0, -370.0), // hold lands
    (71655.0, -657.2, -864.0, 90.0, false, 1.0,       1.0, -185.0),
    (71657.0, -616.7, -657.0, 90.0, false, 1.0,      35.0,  -76.0), // −75.5 rounds away
    (71658.0, -348.0, -500.0, 90.0, false, 1.0,      35.0,  -21.0),
    (71659.0, -285.4, -422.0, 90.0, false, 1.0,      27.0,    3.0), // 2.5 rounds away
    (71660.0, -261.2, -335.0, 90.0, false, 1.0,      37.0,   20.0),
    (71661.0,  -70.9, -298.0, 90.0, false, 1.0,      11.0,   15.0),
    (71662.0,  -88.2, -196.0, 90.0, false, 1.0,       9.0,   12.0),
    (71663.0,   19.8, -188.0, 90.0, false, 1.0,       8.0,   10.0),
    (71664.0,  513.1, -179.0, 90.0, false, 1.0,     -16.0,   -4.0), // negative acOut floor
    (71665.0,  546.0, -134.0, 90.0, false, 1.0,     -20.0,  -13.0),
    (71666.0,  583.9,  -89.0, 90.0, false, 1.0,       6.0,   -4.0),
    (71667.0,  592.4,  -60.0, 90.0, false, 1.0,      27.0,   11.0),
    (71669.0,  593.9,  -49.0, 90.0, false, 1.0,      32.0,   21.0),
    (71670.0,  606.2,  -23.0, 90.0, false, 1.0,      43.0,   32.0),
    (71671.0,  613.4,  -50.0, 90.0, false, 1.0,      32.0,   32.0), // 31.5 rounds away
    (71672.0,  610.5,  -22.0, 90.0, false, 1.0,      43.0,   37.0),
];
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Shadow, // exactly as captured; shadow runs the real write path
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
    errgain_dead_w: f64::INFINITY, // captured pre-feature: err-gain pacing off
    errgain_slope_w: 2000.0,
    errgain_cap: 1.0,
    inverter_nominal_w: f64::INFINITY,
    // Captures predate the Smith compensation - replay without it.
    smith: false,
};
let mut st = state(Kind::Stock);
st.owner_us = true;
st.last_flip_t = 71000.0;
st.last_out = Some(-608.0); // the write at t=71646, where the replay seeds

for &(t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, expect) in rows {
    let sg = socguard::enact(&socguard::Inputs {
        bl_state: bl::SOCG_DEFAULT,
        min_soc: 10.0,
        charge_request: false,
        dc_voltage: 52.0,
        eff_max_charge_w: f64::NAN,
        multi_supports_tpimf: true,
        override_force_charge: ovr_fc,
        ovr_max_discharge_w: ovr_maxdis,
    });
    let snap = Snapshot {
        s: Sensors { grid, reported, target: 5.0, soc, state: sysstate::SCHEDULED_CHARGE },
        dt: 1.0,
        dt_clamped: false,
        min_soc: 10.0,
        lo: -7030.0,
        hi: 7030.0,
        effective_target: 5.0,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: acout,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, t);
    assert_eq!(d.write, Write::Nothing, "t={t}: shadow never writes");
    assert!(
        (d.display_cmd - expect).abs() < 0.6,
        "t={t}: replay must be tick-exact, expected {expect}, got {}",
        d.display_cmd
    );
}
assert_eq!(st.sanity_trips, 0);
}
#[test]
fn golden_full_loop_replay_invariants() {
use crate::dbus::*;
use crate::testbus::MockBus;
use crate::{battery_limits, socguard};

let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/golden-sample.csv");
let data = match std::fs::read_to_string(path) {
    Ok(d) => d,
    Err(_) => return, // fixture optional
};
let mut lines = data.lines();
let header: Vec<&str> = lines.next().unwrap().split(',').collect();
let ix = |n: &str| header.iter().position(|h| *h == n).unwrap_or_else(|| panic!("col {n}"));
let col = [
    "grid", "soc", "state", "dvcc", "acin_p", "vdc", "fw", "multi_maxchg_i", "bms_ccl",
    "bms_dcl", "bms_chgreq", "set_setpoint", "bl_state", "bl_minsoc", "h4_forcecharge",
]
.map(&ix);
let [c_grid, c_soc, c_state, c_dvcc, c_acinp, c_vdc, c_fw, c_mmci, c_ccl, c_dcl, c_chgreq, c_setp, c_bls, c_blm, c_fc] =
    col;
let getf = |f: &[&str], i: usize| f[i].trim().parse::<f64>().unwrap_or(f64::NAN);
let or = |v: f64, d: f64| if v.is_nan() { d } else { v };

const BMS: &str = "com.victronenergy.battery.golden";
// Persistent broker + socguard (they hold latch/sticky state across the trace, exactly
// as the running daemon does). Seeded so the BMS service resolves.
let mut seed = MockBus::new();
seed.add_service(BMS);
seed.set_str(SYS, P_ACTIVE_BMS, BMS);
let mut broker = battery_limits::BatteryBroker::new(&seed);
let mut socguard = socguard::SocGuard::new(&seed);

// The design-capacity envelope handed to the production clamp — named, not a literal.
let (env_lo, env_hi) = (-HARD_CLAMP_W, HARD_CLAMP_W);
let mut st = state(Kind::Stock);
st.owner_us = true; // exercise the full owning path
let c = cfg(Stage::Takeover);
let (mut rows, mut fc_rows) = (0u32, 0u32);

for (n, line) in lines.enumerate() {
    let f: Vec<&str> = line.split(',').collect();
    if f.len() < header.len() {
        continue;
    }
    let (grid, soc, target) = (getf(&f, c_grid), getf(&f, c_soc), getf(&f, c_setp));
    if grid.is_nan() || soc.is_nan() || target.is_nan() {
        continue;
    }
    let hub4_fc = f[c_fc].trim() == "1";

    // Rebuild the row's dbus view and derive bounds + SocGuard via the REAL code.
    let mut bus = MockBus::new();
    bus.add_service(BMS);
    bus.set_str(SYS, P_ACTIVE_BMS, BMS);
    bus.set(SYS, P_DVCC, or(getf(&f, c_dvcc), 1.0));
    bus.set(SYS, P_PV_CURRENT, 0.0);
    bus.set(VEBUS, P_FIRMWARE, or(getf(&f, c_fw), 1376.0));
    bus.set(VEBUS, P_DC_VOLTAGE, or(getf(&f, c_vdc), 52.0));
    bus.set(VEBUS, P_SUSTAIN, 0.0);
    bus.set(VEBUS, P_MULTI_MAXCHG_I, or(getf(&f, c_mmci), 140.0));
    bus.set(VEBUS, crate::dbus::P_TPIMF, 1.0);
    bus.set(BMS, P_BMS_CCL, or(getf(&f, c_ccl), 280.0));
    bus.set(BMS, P_BMS_DCL, or(getf(&f, c_dcl), 280.0));
    bus.set(BMS, crate::dbus::P_CHARGE_REQUEST, or(getf(&f, c_chgreq), 0.0));
    bus.set(SETTINGS, P_MAXCHARGE, -1.0);
    bus.set(SETTINGS, P_MAXDISCHARGE, -1.0);
    bus.set(SETTINGS, P_MAXCHARGE_PCT, 100.0);
    bus.set(SETTINGS, P_MAXDISCHARGE_PCT, 100.0);
    bus.set(SETTINGS, P_BL_STATE, or(getf(&f, c_bls), 0.0));
    bus.set(SETTINGS, P_BL_MINSOC, or(getf(&f, c_blm), 10.0));
    bus.set(crate::dbus::HUB4, crate::dbus::P_OVR_FORCECHARGE, if hub4_fc { 1.0 } else { 0.0 });

    let tr = broker.tick(&bus);
    let (lo, hi) = battery_limits::clamp_bounds(tr.enforced, env_lo, env_hi);
    let sg = socguard.tick(&bus);

    // Regime equality end-to-end: our force-charge decision == hub4's on real data.
    assert_eq!(sg.force_charge, hub4_fc, "row {n}: force-charge regime mismatch");

    let snap = Snapshot {
        s: Sensors {
            grid,
            reported: or(getf(&f, c_acinp), 0.0),
            target,
            soc,
            state: getf(&f, c_state) as i64,
        },
        dt: 20.0,
        dt_clamped: false,
        min_soc: or(getf(&f, c_blm), 10.0),
        lo,
        hi,
        effective_target: target,
        sg,
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: true,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
        ac_out_w: f64::NAN,
        hub4mode_live: None,
        dc_batt_w: f64::NAN,
    };
    let d = decide(&snap, &mut st, &c, n as f64 * 20.0);

    // Invariants, all checked against the bounds the PRODUCTION broker derived this row:
    assert!(d.command.is_finite(), "row {n}: non-finite command");
    assert!(
        d.command >= lo - 1e-6 && d.command <= hi + 1e-6,
        "row {n}: command {} outside the BMS-derived envelope [{lo},{hi}]",
        d.command
    );
    assert!(
        d.command.abs() <= HARD_CLAMP_W + 1e-6,
        "row {n}: command {} exceeds design capacity",
        d.command
    );
    if sg.force_charge {
        // TPIMF: the setpoint is a feed-in cap while firmware charges maximally —
        // the normal-law value passes through (may be negative), capped at Σ max-
        // charge. Legacy: the setpoint IS the charge command and must not discharge.
        let fc = sg.cmd_override.expect("force_charge implies cmd_override");
        if sg.tpimf {
            assert!(d.command <= fc + 1e-6, "row {n}: TPIMF command {} above Σ max-charge {fc}", d.command);
        } else {
            assert!(d.command >= -1e-6, "row {n}: legacy force-charge but command {} discharges", d.command);
        }
    }
    if hub4_fc {
        fc_rows += 1;
    }
    rows += 1;
}
assert!(rows > 100, "expected the full golden trace, got {rows} rows");
assert!(fc_rows > 0, "fixture should include force-charge rows");
eprintln!("golden full-loop replay: {rows} rows ({fc_rows} force-charge), invariants held vs live-derived bounds");
}
/// End-to-end learn-gate test THROUGH decide(): identical steady discharge inputs must
/// leave the adaptive model untouched in shadow and trim (our command isn't actuated
/// there — learning would train on a phantom regime)...
#[test]
fn adaptive_gate_shadow_and_trim_never_learn_e2e() {
for stage in [Stage::Shadow, Stage::Trim] {
    let mut st = state(Kind::Adaptive(AdaptiveCfg::tuned()));
    st.owner_us = true; // even while "owning" (trim active / shadow hypothetical)
    let c = cfg(stage);
    for i in 0..500 {
        let mut sp = snap(true);
        sp.s.grid = sp.s.target - 10.0; // steady small error => integral winds
        sp.s.reported = -800.0; // steady discharge throughput
        decide(&sp, &mut st, &c, i as f64);
    }
    let ff = st.ctrl.ff_snapshot(800.0).unwrap();
    assert_eq!((ff.a, ff.b), (0.0, 0.0), "{} must not learn", stage.name());
}
}
/// ...and the SAME inputs in takeover (owning + meter usable) MUST learn. Together with
/// the test above this pins the `actuating` expression in decide() in both directions.
#[test]
fn adaptive_gate_takeover_learns_e2e() {
let mut st = state(Kind::Adaptive(AdaptiveCfg::tuned()));
st.owner_us = true;
let c = cfg(Stage::Takeover);
for i in 0..500 {
    let mut sp = snap(true);
    sp.s.grid = sp.s.target - 10.0;
    sp.s.reported = -800.0;
    decide(&sp, &mut st, &c, i as f64);
}
let ff = st.ctrl.ff_snapshot(800.0).unwrap();
assert!(
    ff.a != 0.0 || ff.b != 0.0,
    "takeover + owning + usable meter must learn, got a={} b={}",
    ff.a,
    ff.b
);
}
