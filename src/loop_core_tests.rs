use super::*;
use crate::control::Kind;
use crate::states::{bl, sysstate};

fn sg_free() -> SocGuardOut {
    SocGuardOut {
        force_charge: false,
        max_discharge_w: f64::INFINITY,
        cmd_override: None,
        tpimf: false,
        charge_floor_w: 0.0,
    }
}

fn snap(stage_gm: bool) -> Snapshot {
    Snapshot {
        s: Sensors { grid: 100.0, reported: 0.0, target: 20.0, soc: 60.0, state: sysstate::DISCHARGING },
        dt: 1.0,
        dt_clamped: false,
        min_soc: 15.0,
        lo: -HARD_CLAMP_W,
        hi: HARD_CLAMP_W,
        effective_target: 20.0,
        sg: sg_free(),
        feed_block_charge: false,
        feed_force_flat: false,
        gen_disable_export: false,
        gm_should_write: stage_gm,
        shore_out: ShoreOut::default(),
        hub4_actual: None,
            ac_out_w: f64::NAN,
            hub4mode_live: None,
    }
}

fn state(stage_kind: Kind) -> LoopState {
    LoopState {
        ctrl: Controller::new(stage_kind),
        owner_us: false,
        last_flip_t: -1000.0,
        trim_override: None,
        last_out: None,
        oob_since: None,
        prev_force_charge: false,
        ceded_external: false,
        sanity_trips: 0,
    }
}

fn cfg(stage: Stage) -> DecideCfg {
    // Safety params derived from the design anchor, exactly as the runtime does.
    let sl = SafetyLimits::derive(HARD_CLAMP_W, 1.0, None, None);
    DecideCfg {
        stage,
        soc_hyst: 3.0,
        min_dwell_s: 30.0,
        trim_ki: 0.02,
        orig_mode: 1,
        state_recharge: sysstate::RECHARGE,
        slew_w_per_s: sl.slew_w_per_s,
        sanity_band_w: sl.sanity_band_w,
        sanity_secs: sl.sanity_secs,
        ema_adaptive: false,
    }
}

#[test]
fn takeover_takes_over_above_floor_and_writes_setpoint() {
    let mut st = state(Kind::Stock);
    let d = decide(&snap(true), &mut st, &cfg(Stage::Takeover), 0.0);
    assert!(d.owner_us && d.flipped);
    assert_eq!(d.hub4mode, Some(HUB4MODE_EXTERNAL));
    assert!(matches!(d.write, Write::Setpoint(_)));
}

#[test]
fn takeover_holds_write_when_meter_unusable() {
    let mut st = state(Kind::Stock);
    st.owner_us = true; // already owning, no flip
    let d = decide(&snap(false), &mut st, &cfg(Stage::Takeover), 100.0);
    assert!(d.owner_us);
    assert_eq!(d.write, Write::Nothing, "no setpoint write when meter unusable");
}

#[test]
fn below_floor_hands_back_immediately() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let mut sp = snap(true);
    sp.s.soc = 10.0; // <= min_soc
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 1.0); // dwell not elapsed, but handback is never throttled
    assert!(!d.owner_us && d.flipped);
    assert_eq!(d.hub4mode, Some(1));
}

#[test]
fn takeover_entry_is_dwell_throttled() {
    let mut st = state(Kind::Stock);
    st.last_flip_t = 0.0; // just flipped
    let d = decide(&snap(true), &mut st, &cfg(Stage::Takeover), 5.0); // 5s < 30s dwell
    assert!(!d.owner_us, "take-over throttled until dwell elapses");
}

#[test]
fn trim_integrates_toward_setpoint() {
    let mut st = state(Kind::Stock);
    let mut sp = snap(true);
    sp.s.grid = 500.0; // grid above the 20 W target => positive error pushes override up
    sp.s.soc = 60.0;
    let c = cfg(Stage::Trim);
    // first call takes over + seeds override at target
    let d1 = decide(&sp, &mut st, &c, 0.0);
    assert!(d1.owner_us);
    // subsequent calls integrate: override moves away from the seed toward compensating.
    let d2 = decide(&sp, &mut st, &c, 1.0);
    let d3 = decide(&sp, &mut st, &c, 2.0);
    match (d2.write, d3.write) {
        (Write::Override(o2), Write::Override(o3)) => {
            assert!(o2 < sp.s.target && o3 < o2, "integral drives override down: {o2} then {o3}");
        }
        _ => panic!("trim must write override"),
    }
}

#[test]
fn trim_suspends_during_force_charge() {
    let mut st = state(Kind::Stock);
    let mut sp = snap(true);
    sp.sg.force_charge = true;
    sp.s.grid = 500.0;
    let c = cfg(Stage::Trim);
    decide(&sp, &mut st, &c, 0.0); // take over
    let d = decide(&sp, &mut st, &c, 1.0);
    assert_eq!(d.write, Write::Override(sp.s.target), "hold neutral, don't fight stock");
}

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
    use crate::socguard;
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

/// The window-tail hold (live 2026-08-16): ForceCharge already 0, but
/// `/Overrides/MaxDischargePower` = 1 W pins the battery while a 1.9 kW load runs.
/// Trim must hold neutral — integrating against a deliberately-held battery only
/// winds the override up and bites when the hold clears at window end.
#[test]
fn trim_suspends_during_window_tail_hold() {
    let mut st = state(Kind::Stock);
    let mut sp = snap(true);
    sp.sg.max_discharge_w = 1.0; // external hold, NOT force-charge
    sp.s.grid = 1908.9;
    let c = cfg(Stage::Trim);
    decide(&sp, &mut st, &c, 0.0); // take over
    let d = decide(&sp, &mut st, &c, 1.0);
    assert_eq!(d.write, Write::Override(sp.s.target), "hold neutral during the tail hold");
}

#[test]
fn sanity_is_suppressed_during_window_tail_hold() {
    // Same scenario in takeover: grid legitimately 1.9 kW off-target while the hold
    // is active — must not accumulate toward a sanity trip / spurious hand-back.
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    st.prev_force_charge = true; // set by the previous tick's hold (see decide())
    let mut sp = snap(true);
    sp.sg.max_discharge_w = 1.0;
    sp.s.grid = 1908.9;
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
    assert!(st.oob_since.is_none(), "hold must suppress the out-of-band timer");
    assert!(d.owner_us, "no hand-back during the tail hold");
    // And the flag re-arms for the next tick from the cap itself.
    assert!(st.prev_force_charge);
}

#[test]
fn shadow_never_writes() {
    let mut st = state(Kind::Stock);
    let d = decide(&snap(true), &mut st, &cfg(Stage::Shadow), 0.0);
    assert_eq!(d.write, Write::Nothing);
    assert_eq!(d.hub4mode, None);
}

#[test]
fn force_charge_command_overrides_discharge() {
    // SocGuard force-charge with a finite ceiling: composed command must be the charge cmd.
    let mut sp = snap(true);
    sp.sg = SocGuardOut {
        force_charge: true,
        max_discharge_w: 0.0,
        cmd_override: Some(4700.0),
        tpimf: false,
        charge_floor_w: 0.0,
    };
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    assert_eq!(cmd, 4700.0);
}

/// Live capture 2026-08-16 ~04:15, scheduled-charge window MID (SystemState 259,
/// `/Overrides/ForceCharge`=1, TPIMF firmware): the stock daemon keeps writing the
/// raw normal-law value — observed `AcPowerSetpoint` = −230 W at grid 2 771.7 W /
/// reported +2 513 W / target 5 W (the firmware charges maximally on its own; the
/// setpoint only caps feed-in). Our first implementation clamped this to ≥ 0 and
/// deviated by ~house-loads (~+200 W median) for the whole 6 h window.
#[test]
fn golden_tpimf_force_charge_passes_normal_law_through() {
    let mut sp = snap(true);
    sp.s = Sensors { grid: 2771.7, reported: 2513.0, target: 5.0, soc: 73.0, state: sysstate::SCHEDULED_CHARGE };
    sp.effective_target = 5.0;
    sp.sg = SocGuardOut {
        force_charge: true,
        max_discharge_w: f64::INFINITY, // schedule sets NO discharge cap while charging
        cmd_override: Some(crate::socguard::FORCE_CHARGE_SENTINEL_W),
        tpimf: true,
        charge_floor_w: 0.0,
    };
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    // Normal law: reported + (target − grid) = 2513 + 5 − 2771.7 = −253.7 W.
    // The stock daemon wrote −230 W on this row (Δ = meter/tick noise).
    assert!((cmd - (-253.7)).abs() < 1.0, "normal law must pass through, got {cmd}");
    assert!((cmd - (-230.0)).abs() < 100.0, "must track the stock daemon's live write, got {cmd}");
}

/// Live capture 2026-08-16 ~04:30, scheduled-charge window TAIL: target SOC (90 %)
/// reached with the window still open. systemcalc drops `/Overrides/ForceCharge` to 0
/// and sets `/Overrides/MaxDischargePower` = max(1, 0.8·pv) = 1 W (no PV at night) —
/// "hold the charge". Stock held ~+48 W against a 1.9 kW morning load while the raw
/// normal law wanted −1 868 W. Without honoring the override we would have discharged
/// the freshly charged battery into the house for the remaining ~37 min of the window.
#[test]
fn golden_window_tail_discharge_hold_is_honored() {
    let mut sp = snap(true);
    sp.s = Sensors { grid: 1908.9, reported: 35.0, target: 5.0, soc: 90.0, state: sysstate::SCHEDULED_CHARGE };
    sp.effective_target = 5.0;
    sp.sg = sg_free();
    sp.sg.max_discharge_w = 1.0; // enact() of /Overrides/MaxDischargePower = 1
    sp.ac_out_w = 49.0; // measured Multi AC-out that morning
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    // The stock hold write = clamp(−1 W) + acOut = +48 W — the EXACT value the stock
    // daemon wrote on this live row (see composed_command for the frame).
    assert!((cmd - 48.0).abs() < 1e-9, "hold must float at −cap + acOut, got {cmd}");

    // acOut unread (NaN): degrade to the bare cap — still no discharge.
    sp.ac_out_w = f64::NAN;
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    assert!((cmd - (-1.0)).abs() < 1e-9, "NaN acOut degrades to the bare cap, got {cmd}");

    // Same row with the override cleared (window exit, or an AllowDischarge schedule
    // >1 % over target): normal discharge resumes; acOut must NOT leak into the law.
    sp.sg = sg_free();
    sp.ac_out_w = 49.0;
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    assert!(cmd < -1500.0, "cleared override must discharge normally, got {cmd}");
}

/// The AC-out feedforward composes with every battery-frame floor, exactly as the
/// stock single clamp does: BL-state discharge inhibit (floor 0 → write = acOut) and
/// the state-6 minimum-charge floor (write = dcV·5 + acOut).
#[test]
fn ac_out_feedforward_lifts_state_floors() {
    let mut sp = snap(true);
    sp.s.grid = 1500.0; // big load: raw law is strongly negative
    sp.sg = sg_free();
    sp.sg.max_discharge_w = 0.0; // BL no-discharge state
    sp.ac_out_w = 15.0;
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    assert!((cmd - 15.0).abs() < 1e-9, "inhibit floor lifts to acOut, got {cmd}");

    sp.sg.charge_floor_w = 260.0; // state 6: dcV·5A
    let mut ctrl = Controller::new(Kind::Stock);
    let cmd = composed_command(&sp, &mut ctrl, false);
    assert!((cmd - 275.0).abs() < 1e-9, "charge floor lifts to dcV·5 + acOut, got {cmd}");
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
///   meter still shows the collapsing 2.3 kW charge). The stock daemon coasts through
///   it on its own write smoothing (−192 W held). We pass the raw law through — a
///   KNOWN, bounded divergence (≈2 ticks, ≲1 Wh, well inside the slew envelope) that
///   this test pins so any future "fix" is a deliberate decision, not drift.
#[test]
fn golden_schedule_window_edges_dual_override_and_hold_gap() {
    use crate::socguard;
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
#[test]
fn golden_window_tail_ema_replay_is_tick_exact() {
    use crate::socguard;
    // (t, grid, reported, soc, ovr_fc, ovr_maxdis, acout, expected_write) — live rows;
    // stock's own writes tracked the same path within its 2.5 s cadence.
    #[rustfmt::skip]
    let rows: &[(f64, f64, f64, f64, bool, f64, f64, f64)] = &[
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

/// End-to-end learn-gate test THROUGH decide(): identical steady discharge inputs must
/// leave the adaptive model untouched in shadow and trim (our command isn't actuated
/// there — learning would train on a phantom regime)...
#[test]
fn adaptive_gate_shadow_and_trim_never_learn_e2e() {
    use crate::control::AdaptiveCfg;
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
    use crate::control::AdaptiveCfg;
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

/// Persistently NON-FINITE grid while driving counts as out-of-band: we cannot verify
/// sanity, so after sanity_secs we must trip and hand back (a NaN must never silently
/// reset the out-of-band timer).
#[test]
fn sanity_nan_grid_counts_out_of_band() {
    let mut st = state(Kind::Pi { ki: 0.05, i_max: 300.0 });
    st.owner_us = true;
    let c = cfg(Stage::Takeover);
    let mut trip_t = None;
    for i in 0..100 {
        let mut sp = snap(true);
        sp.s.grid = f64::NAN;
        let d = decide(&sp, &mut st, &c, i as f64);
        if d.safety.sanity_tripped {
            trip_t = Some(i);
            // At the trip tick the supervisor must have forced the hand-back.
            assert!(!st.owner_us, "trip must hand back immediately");
            assert_eq!(st.sanity_trips, 1);
            break;
        }
    }
    let trip_t = trip_t.expect("persistent NaN grid must trip the sanity supervisor");
    assert!(
        (trip_t as f64) >= c.sanity_secs,
        "trip only after the sanity dwell, got t={trip_t}"
    );
}

/// Sanity-trip latch: each trip doubles the re-entry dwell, so a persistently-wrong
/// controller escalates instead of flip-flopping mode 3<->1 every min_dwell.
#[test]
fn sanity_trip_backoff_doubles_reentry_dwell() {
    let mut st = state(Kind::Pi { ki: 0.05, i_max: 300.0 });
    st.owner_us = true;
    let c = cfg(Stage::Takeover);
    let mut t = 0.0;
    // Grossly out-of-band until the supervisor trips and hands back.
    while st.owner_us {
        let mut sp = snap(true);
        sp.s.grid = sp.s.target + 5000.0;
        decide(&sp, &mut st, &c, t);
        t += 1.0;
        assert!(t < 100.0, "should have tripped within sanity_secs");
    }
    assert_eq!(st.sanity_trips, 1);
    let handback_t = t;
    // Healthy again: re-entry must now wait 2x min_dwell, not 1x.
    while !st.owner_us {
        let mut sp = snap(true);
        sp.s.grid = sp.s.target; // in band
        decide(&sp, &mut st, &c, t);
        if st.owner_us {
            break;
        }
        t += 1.0;
        assert!(t < handback_t + 500.0, "never re-entered");
    }
    let waited = t - handback_t;
    assert!(
        waited >= 2.0 * c.min_dwell_s - 1.0,
        "after one trip re-entry must wait 2x dwell ({}s), waited {waited}s",
        2.0 * c.min_dwell_s
    );
}

// ---- write-phase safety features (each is observable via `Decision.safety`) ----

#[test]
fn slew_guard_limits_a_huge_jump_and_flags_it() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    st.last_out = Some(0.0);
    let mut c = cfg(Stage::Takeover);
    c.slew_w_per_s = 100.0; // tight for the test
    let mut sp = snap(true);
    sp.effective_target = 5000.0; // Stock law => command ~ +5000 in one step
    sp.s.grid = 0.0;
    sp.s.reported = 0.0;
    let d = decide(&sp, &mut st, &c, 100.0);
    match d.write {
        Write::Setpoint(v) => assert!((v - 100.0).abs() < 1e-6, "slewed 0->100, got {v}"),
        other => panic!("expected slewed setpoint, got {other:?}"),
    }
    assert!(d.safety.slew_clamped);
    assert_eq!(d.safety.alarm_level(), 1);
    assert_eq!(d.safety.reason(), "slew-clamped");
}

#[test]
fn stock_write_ema_gain_schedule() {
    // First write: passthrough (rounded).
    assert_eq!(stock_write_ema(None, -286.9, false), -287.0);
    // Flat mode: 0.5 regardless of step size.
    assert_eq!(stock_write_ema(Some(0.0), 5000.0, false), 2500.0);
    // Small step (< 10 W): 0.5 even in adaptive mode.
    assert_eq!(stock_write_ema(Some(0.0), 8.0, true), 4.0);
    // Adaptive: Δ=100 → gain 0.25·ln(10) = 0.5756…
    let v = stock_write_ema(Some(0.0), 100.0, true);
    assert_eq!(v, (0.25f64 * 10.0f64.ln() * 100.0).round());
    // Adaptive: Δ=50 → 0.25·ln(5) = 0.402 → clamped UP to 0.5.
    assert_eq!(stock_write_ema(Some(0.0), 50.0, true), 25.0);
    // Adaptive: Δ=1000 → 0.25·ln(100) = 1.151 → clamped to 1.1 (overshoot!).
    assert_eq!(stock_write_ema(Some(0.0), 1000.0, true), 1100.0);
    // Fixed point: integer target reached and held exactly.
    assert_eq!(stock_write_ema(Some(30.0), 30.0, false), 30.0);
}

/// The live hold-entry convergence (2026-08-17, stock wrote −160 → −80 → −38 → +30 at
/// its 2.5 s cadence): flat-0.5 EMA from the same start toward the same target must
/// converge monotonically with no overshoot and settle ON the integer target.
#[test]
fn stock_write_ema_converges_monotone_flat() {
    let mut v = -160.0;
    let mut prev = v;
    for _ in 0..12 {
        v = stock_write_ema(Some(v), 30.0, false);
        assert!(v >= prev && v <= 30.0, "monotone, no overshoot: {prev} -> {v}");
        prev = v;
    }
    assert_eq!(v, 30.0, "must settle exactly on the target");
}

/// EMA through decide(): first write after take-over is direct; subsequent writes
/// halve toward the command and are integer watts.
#[test]
fn takeover_write_is_ema_smoothed_and_integer() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let c = cfg(Stage::Takeover);
    let mut sp = snap(true);
    sp.s.grid = 100.0; // law: 0 + 20 − 100 = −80
    let d1 = decide(&sp, &mut st, &c, 0.0);
    let Write::Setpoint(v1) = d1.write else { panic!("expected setpoint") };
    assert_eq!(v1, -80.0, "first write is direct (stock passthrough)");
    sp.s.grid = 900.0; // law: −880
    let d2 = decide(&sp, &mut st, &c, 1.0);
    let Write::Setpoint(v2) = d2.write else { panic!("expected setpoint") };
    assert_eq!(v2, -480.0, "second write halves toward the command");
    assert_eq!(v2.fract(), 0.0, "writes are integer watts");
}

/// Shadow simulates the identical write path (for stock-comparable telemetry) but
/// still never writes.
#[test]
fn shadow_simulates_write_path_without_writing() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let c = cfg(Stage::Shadow);
    let mut sp = snap(true);
    sp.s.grid = 100.0;
    let d1 = decide(&sp, &mut st, &c, 0.0);
    assert_eq!(d1.write, Write::Nothing);
    assert_eq!(d1.display_cmd, -80.0);
    sp.s.grid = 900.0;
    let d2 = decide(&sp, &mut st, &c, 1.0);
    assert_eq!(d2.write, Write::Nothing, "shadow must never write");
    assert_eq!(d2.hub4mode, None);
    assert_eq!(d2.display_cmd, -480.0, "displayed value follows the real write law");
}

#[test]
fn external_mode_flip_cedes_ownership_immediately() {
    // User/VRM/another tool flips Hub4Mode away while we own: stock is running
    // again — cede this tick, write no mode (never fight the external change).
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    st.last_flip_t = -1000.0;
    let mut sp = snap(true);
    sp.hub4mode_live = Some(1);
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
    assert!(!d.owner_us && d.flipped, "must cede when the mode is taken from us");
    assert_eq!(d.hub4mode, None, "must not write the mode back");
    assert_eq!(d.write, Write::Nothing);
    // Sticky: hours later, with take-over conditions perfect, we still defer to
    // the operator's choice — only a process restart re-arms the stage.
    sp.hub4mode_live = Some(1);
    let d2 = decide(&sp, &mut st, &cfg(Stage::Takeover), 50_000.0);
    assert!(!d2.owner_us && d2.hub4mode.is_none(), "cede must be sticky");
}

#[test]
fn failed_handback_mode_is_reasserted_until_it_sticks() {
    // Handed back (e.g. after a crash-restart) but the persistent setting still
    // reads EXTERNAL: stock gates its whole loop on it, so keep restoring.
    let mut st = state(Kind::Stock);
    st.owner_us = false;
    let mut sp = snap(true);
    sp.s.soc = 10.0; // below floor: no take-over wanted
    sp.hub4mode_live = Some(HUB4MODE_EXTERNAL);
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
    assert_eq!(d.hub4mode, Some(1), "must re-assert the stock mode");
    assert!(!d.owner_us);
}

#[test]
fn takeover_first_write_is_seeded_from_stock_setpoint() {
    // Take-over landing on a meter-lag tick must not write the raw transient:
    // the EMA seeds from stock's last written setpoint.
    let mut st = state(Kind::Stock);
    let mut sp = snap(true);
    sp.s.grid = 100.0; // law: 0 + 20 − 100 = −80
    sp.hub4_actual = Some(-500.0); // stock's last write
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 0.0);
    assert!(d.owner_us && d.flipped);
    let Write::Setpoint(v) = d.write else { panic!("expected setpoint") };
    assert_eq!(v, -290.0, "EMA from stock's −500 toward law −80, got {v}");
}

#[test]
fn write_gap_preserves_ema_memory() {
    // A meter-guard write gap must not reset the EMA: the post-gap write
    // continues from the last actuated value instead of stepping raw.
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let c = cfg(Stage::Takeover);
    let mut sp = snap(true);
    sp.s.grid = 100.0; // law −80
    let d1 = decide(&sp, &mut st, &c, 0.0);
    let Write::Setpoint(v1) = d1.write else { panic!() };
    assert_eq!(v1, -80.0);
    sp.gm_should_write = false; // guard gap
    let d2 = decide(&sp, &mut st, &c, 1.0);
    assert_eq!(d2.write, Write::Nothing);
    sp.gm_should_write = true;
    sp.s.grid = 900.0; // law −880
    let d3 = decide(&sp, &mut st, &c, 2.0);
    let Write::Setpoint(v3) = d3.write else { panic!() };
    assert_eq!(v3, -480.0, "post-gap write must be EMA'd from −80, got {v3}");
}

#[test]
fn sanity_supervisor_hands_back_after_sustained_divergence() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let c = cfg(Stage::Takeover);
    let mut sp = snap(true);
    sp.s.grid = 3000.0; // 3000 vs setpoint 20 => far outside the 1000 W band

    assert!(decide(&sp, &mut st, &c, 0.0).owner_us, "not tripped instantly");
    assert!(decide(&sp, &mut st, &c, 10.0).owner_us, "still within the dwell");
    let d = decide(&sp, &mut st, &c, 31.0); // past the 30 s sanity dwell
    assert!(!d.owner_us && d.flipped, "sanity forces hand-back");
    assert!(d.safety.sanity_tripped);
    assert_eq!(d.safety.alarm_level(), 2);
    assert_eq!(d.hub4mode, Some(1), "handed the setpoint loop back to stock");
}

#[test]
fn sanity_is_suppressed_during_force_charge() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    st.prev_force_charge = true; // stock force-charging last tick => grid legitimately diverges
    let c = cfg(Stage::Takeover);
    let mut sp = snap(true);
    sp.s.grid = 3000.0;
    let d = decide(&sp, &mut st, &c, 31.0);
    assert!(d.owner_us, "must not trip while force-charging");
    assert!(!d.safety.sanity_tripped);
}

#[test]
fn safety_params_derive_from_the_design_anchor() {
    // No overrides: slew = full range / 2 s, sanity band = 15 % of design, dt-clamp floor.
    let sl = SafetyLimits::derive(HARD_CLAMP_W, 1.0, None, None);
    assert!((sl.slew_w_per_s - HARD_CLAMP_W / SLEW_TRAVERSE_S).abs() < 1e-6);
    assert!((sl.sanity_band_w - HARD_CLAMP_W * SANITY_FRACTION).abs() < 1e-6);
    assert_eq!(sl.max_dt_s, MIN_DT_CLAMP_S); // max(5*1, 5)
    // A larger interval widens the dt-clamp; overrides win over the derived value.
    let sl2 = SafetyLimits::derive(4000.0, 3.0, Some(999.0), Some(500.0));
    assert_eq!(sl2.slew_w_per_s, 999.0);
    assert_eq!(sl2.sanity_band_w, 500.0);
    assert_eq!(sl2.max_dt_s, 15.0); // 5 * 3 s
}

#[test]
fn dt_clamp_flag_propagates_to_safety() {
    let mut st = state(Kind::Stock);
    st.owner_us = true;
    let mut sp = snap(true);
    sp.dt_clamped = true;
    let d = decide(&sp, &mut st, &cfg(Stage::Takeover), 100.0);
    assert!(d.safety.dt_clamped && d.safety.any());
}

// Full-loop replay (§3.2): drive the WHOLE composed decision over the real captured trace
// (both discharge 256 + scheduled-charge 259 regimes). Crucially the per-row command bounds
// and force-charge decision come from the PRODUCTION code (battery_limits::BatteryBroker +
// socguard::SocGuard fed by the row's own dbus values) — not hand-rolled constants — so this
// is a genuine replay: every invariant is checked against the limits the real system derived.
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
