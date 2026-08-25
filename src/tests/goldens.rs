//! Cross-module golden tests: live-captured sequences and end-to-end regimes
//! replayed through the REAL `socguard::enact` + `loop_core::decide` pipeline.
//! Unit tests live inline at the bottom of each module; this directory holds
//! everything that spans modules.

use crate::control::{AdaptiveCfg, Kind};
use crate::loop_core::testutil::*;
use crate::socguard;
use crate::gridmeter::HUB4MODE_EXTERNAL;
use crate::loop_core::*;
use crate::shore::ShoreOut;
use crate::states::{bl, sysstate};
use crate::Stage;

/// Live capture 2026-08-16 ~08:07 — an Octopus Intelligent Go dispatch: an HA
/// automation raised `BatteryLife/MinimumSocLimit` 10 % → 90 % for one minute.
/// Replayed row-for-row (real telemetry values) through the REAL socguard::enact +
/// decide(). The captured sequence, which this test pins tick-exactly:
///   t=19460  the min-SOC raise lands (BL state still 10, SystemState still 256):
///            hand-back fires THAT tick — min-SOC is sampled live, soc 77 ≤ 90 —
///            before batterylife/systemcalc publish state 12 / 258 at all.
///   t=19461  BL state 12 arrives (force-charge + discharge inhibit); state still 256.
///   t=..19530 SystemState 258; firmware TPIMF-charges at ~5 kW (grid peaks 7.1 kW,
///            both hub4 override items stay inert — this path is pure BL-state);
///            we must stay handed-back throughout (soc 77 < 90+hyst, state 258).
///   t=19531  BL reverts to 10 (fc=0) but SystemState is STILL 258 — the state-258
///            gate blocks re-entry for exactly this one tick.
///   t=19532  SystemState 252: re-take (72 s since hand-back > 30 s dwell).
///   t=19534  one-tick meter lag (reported collapses 4999 → 1252 while grid is still
///            5.3 kW): raw law −4053 W — the slew guard caps the actuated step.
#[test]
fn golden_octopus_dispatch_minsoc_raise_sequence() {
// (t, grid, reported, soc, sysstate, bl_state, min_soc) — from the live capture.
#[rustfmt::skip]
let rows: &[(f64, f64, f64, f64, i64, i32, f64)] = &[
    (19456.0,   49.9, -1867.0, 78.0, 256, 10, 10.0),
    (19457.0,   32.1, -1835.0, 78.0, 256, 10, 10.0),
    (19458.0,   82.0, -1851.0, 78.0, 256, 10, 10.0),
    (19459.0,   59.6, -1843.0, 78.0, 256, 10, 10.0),
    (19460.0,   49.4, -1838.0, 77.0, 256, 10, 90.0), // raise lands -> hand back NOW
    (19461.0,   74.4, -1848.0, 77.0, 256, 12, 90.0), // BL 12 leads SystemState
    (19462.0,   54.0, -1820.0, 77.0, 258, 12, 90.0),
    (19465.0, 3178.3,  4648.0, 77.0, 258, 12, 90.0), // charge ramp
    (19470.0, 7015.0,  5097.0, 77.0, 258, 12, 90.0), // ~5 kW charge + 1.9 kW load
    (19500.0, 6881.0,  4871.0, 78.0, 258, 12, 90.0),
    (19520.0, 5423.1,  5090.0, 78.0, 258, 12, 90.0),
    (19530.0, 5311.9,  4984.0, 78.0, 258, 12, 90.0),
    (19531.0, 5293.3,  4987.0, 78.0, 258, 10, 10.0), // BL reverts first; 258 blocks
    (19532.0, 5290.9,  4999.0, 78.0, 252, 10, 10.0), // re-take
    (19534.0, 5310.4,  1252.0, 78.0, 252, 10, 10.0), // meter-lag transient
    (19535.0,  786.4,   239.0, 78.0, 252, 10, 10.0),
    (19536.0,  430.6,   -49.0, 78.0, 252, 10, 10.0),
    (19538.0,  191.8,  -200.0, 78.0, 252, 10, 10.0),
    (19540.0,   60.6,  -211.0, 78.0, 256, 10, 10.0),
];
// Device-derived safety numbers (envelope 7030 W -> slew 3515 W/s), as live.
let sl = SafetyLimits::derive(7030.0, 1.0, None, None);
let c = DecideCfg {
    stage: Stage::Takeover,
    soc_hyst: 3.0,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    state_recharge: sysstate::RECHARGE,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
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
    };
    let d = decide(&snap, &mut st, &c, t);

    match t as i64 {
        19456..=19459 => {
            assert!(d.owner_us, "t={t}: normal ESS, we own");
            assert!(matches!(d.write, Write::Setpoint(_)));
        }
        19460 => {
            assert!(d.flipped && !d.owner_us, "t={t}: raised floor must hand back THIS tick");
            assert_eq!(d.hub4mode, Some(1), "t={t}: restore stock mode");
            assert!(!sg.force_charge, "t={t}: BL still 10 — the floor alone triggered it");
        }
        19461..=19530 => {
            assert!(!d.owner_us, "t={t}: handed back for the whole dispatch");
            assert_eq!(d.write, Write::Nothing, "t={t}: no writes while handed back");
            assert_eq!(d.hub4mode, None, "t={t}: no mode flapping");
            assert!(sg.force_charge && sg.tpimf, "t={t}: BL-12 regime");
            assert_eq!(sg.max_discharge_w, 0.0, "t={t}: BL-12 inhibits discharge");
            if t >= 19470.0 {
                // Steady charge: law is negative, discharge-inhibited to 0 (matches
                // the captured telemetry cmd = -0.0 on every one of these rows). On
                // the ramp tick (19465) the law is legitimately +1474.7 — grid lags
                // the charge step — also matching the capture, so it is skipped here.
                assert!(d.command.abs() < 1e-9, "t={t}: cmd {}", d.command);
            }
        }
        19531 => {
            assert!(!d.owner_us, "t={t}: SystemState 258 must block re-entry this tick");
            assert!(!sg.force_charge, "t={t}: BL already back to 10");
        }
        19532 => {
            assert!(d.flipped && d.owner_us, "t={t}: re-take once 258 clears");
            assert_eq!(d.hub4mode, Some(HUB4MODE_EXTERNAL));
        }
        19534 => {
            // Meter-lag tick: raw law = 1252 + 5 − 5310.4 = −4053.4. The stock
            // write EMA absorbs it: prev = −287 (the direct first write on
            // re-take at t=19532), so the write is −287 + 0.5·(−3766.4) ≈ −2170 —
            // exactly how stock rides out one-tick meter glitches. The W/s slew
            // guard stays as a backstop and must NOT need to fire here.
            let prev = last_setpoint.expect("was writing before the transient");
            assert!((prev - (-287.0)).abs() < 1.0, "t={t}: re-take wrote {prev}");
            match d.write {
                Write::Setpoint(v) => {
                    assert!((v - (-2170.0)).abs() < 2.0,
                        "t={t}: EMA must halve the step {prev} -> {v} (raw −4053.4)");
                    assert!(v > -4053.0, "t={t}: raw transient must not pass through");
                }
                w => panic!("t={t}: expected a setpoint write, got {w:?}"),
            }
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
assert!(st.owner_us, "we own again after the dispatch");
assert_eq!(st.sanity_trips, 0, "the dispatch must not count as a sanity trip");
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
    soc_hyst: 3.0,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    state_recharge: sysstate::RECHARGE,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
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
    soc_hyst: 3.0,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    state_recharge: sysstate::RECHARGE,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
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
    };
    let d = decide(&snap, &mut st, &c, t);
    if t >= 55929.0 {
        worst = worst.max((d.display_cmd - expect).abs());
    }
    prev_t = t;
}
assert!(worst <= 2.0, "window-exit replay drifted from the live capture: worst {worst:.1} W");
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
    soc_hyst: 3.0,
    min_dwell_s: 30.0,
    trim_ki: 0.02,
    orig_mode: 1,
    state_recharge: sysstate::RECHARGE,
    slew_w_per_s: sl.slew_w_per_s,
    sanity_band_w: sl.sanity_band_w,
    sanity_secs: sl.sanity_secs,
    ema_adaptive: false,
    // Captured under symmetric 0.5 write pacing — pinned for tick-exactness.
    ema_gain_up: 0.5,
    ema_gain_down: 0.5,
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
    bus.set(VEBUS, socguard::P_TPIMF, 1.0);
    bus.set(BMS, P_BMS_CCL, or(getf(&f, c_ccl), 280.0));
    bus.set(BMS, P_BMS_DCL, or(getf(&f, c_dcl), 280.0));
    bus.set(BMS, socguard::P_CHARGE_REQUEST, or(getf(&f, c_chgreq), 0.0));
    bus.set(SETTINGS, P_MAXCHARGE, -1.0);
    bus.set(SETTINGS, P_MAXDISCHARGE, -1.0);
    bus.set(SETTINGS, P_MAXCHARGE_PCT, 100.0);
    bus.set(SETTINGS, P_MAXDISCHARGE_PCT, 100.0);
    bus.set(SETTINGS, P_BL_STATE, or(getf(&f, c_bls), 0.0));
    bus.set(SETTINGS, P_BL_MINSOC, or(getf(&f, c_blm), 10.0));
    bus.set(socguard::HUB4, socguard::P_OVR_FORCECHARGE, if hub4_fc { 1.0 } else { 0.0 });

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
