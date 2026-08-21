use super::*;
use crate::testbus::MockBus;

const PV1: &str = "com.victronenergy.pvinverter.pv1";

/// A bus with one producing AC PV inverter (in-position, running).
fn one_pv(ac_power: f64, power_limit: f64, max_power: f64) -> MockBus {
    let mut b = MockBus::new();
    b.add_service(PV1);
    b.set(PV1, P_AC_POWER, ac_power);
    b.set(PV1, P_AC_POWERLIMIT, power_limit);
    b.set(PV1, P_AC_MAXPOWER, max_power);
    b.set(PV1, P_POSITION, 0.0);
    b.set(PV1, P_STATUSCODE, 7.0); // running
    b.set(SETTINGS, P_HUB4MODE, 1.0); // ESS mode -> limiter enabled
    b
}

fn active_ctx() -> PvContext {
    PvContext { charge_headroom_w: 0.0, battery_full: false, export_blocked: false, limiter_enabled: true }
}

// ---- no-PV stub: the behaviourally-exact path for THIS install ----

#[test]
fn no_pv_returns_constant_zero_stub() {
    let bus = MockBus::new(); // no pvinverter services
    let mut pv = PvLimiter::new(&bus);
    // Even with an adversarial context, the stub is exact and writes nothing.
    let ctx = PvContext { charge_headroom_w: 0.0, battery_full: true, export_blocked: true, limiter_enabled: true };
    let out = pv.tick(&bus, ctx);
    assert_eq!(out, PvOut::STUB);
    assert_eq!(out, PvOut { pv_disable: 0, pv_limiter_active: 0, pv_offset_w: 0.0 });
    assert!(pv.last_writes().is_empty(), "stub must not curtail anything");
}

#[test]
fn mode3_disables_limiter_even_with_pv() {
    let mut bus = one_pv(3000.0, 5000.0, 5000.0);
    bus.set(SETTINGS, P_HUB4MODE, 3.0); // External Control
    let mut pv = PvLimiter::new(&bus);
    let out = pv.tick(&bus, PvContext { charge_headroom_w: 0.0, battery_full: true, export_blocked: true, limiter_enabled: true });
    assert_eq!(out.pv_disable, 0);
    assert_eq!(out.pv_limiter_active, 0);
    assert!(pv.last_writes().is_empty(), "mode 3 must not curtail");
    // Feed-forward still tracks measured PV (separate timer, not mode-gated).
    assert!((out.pv_offset_w - 3000.0).abs() < 1e-9);
}

// ---- hard curtail (no sink) ----

#[test]
fn battery_full_export_blocked_hard_curtails_and_disables() {
    let bus = one_pv(4000.0, 5000.0, 5000.0);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, battery_full: true, export_blocked: true, limiter_enabled: true };
    let out = pv.tick(&bus, ctx);
    assert_eq!(out.pv_disable, 1);
    assert_eq!(out.pv_limiter_active, 1);
    // Producing inverter cut to zero.
    assert_eq!(pv.last_writes(), &[(PV1.to_string(), CURTAIL_W)]);
}

#[test]
fn idle_inverter_released_to_maxpower_on_hard_curtail() {
    let mut bus = one_pv(0.0, 0.0, 5000.0);
    bus.set(PV1, P_STATUSCODE, 0.0); // not running + Ac/Power 0 => idle
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, battery_full: true, export_blocked: true, limiter_enabled: true };
    let out = pv.tick(&bus, ctx);
    assert_eq!(out.pv_disable, 1);
    // Idle inverter opened to nameplate so it can restart when a sink returns.
    assert_eq!(pv.last_writes(), &[(PV1.to_string(), 5000.0)]);
}

// ---- release sentinel (nothing to limit) ----

#[test]
fn limiter_disabled_releases_with_200kw_sentinel() {
    let bus = one_pv(4000.0, 1000.0, 5000.0); // currently capped at 1000
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { limiter_enabled: false, ..Default::default() };
    let out = pv.tick(&bus, ctx);
    assert_eq!(out.pv_disable, 0);
    assert_eq!(out.pv_limiter_active, 0);
    assert_eq!(pv.last_writes(), &[(PV1.to_string(), RELEASE_W)]);
    assert_eq!(pv.last_writes()[0].1, 200_000.0);
}

// ---- proportional / ramp-toward-headroom ----

#[test]
fn ramps_down_toward_headroom_in_one_step() {
    // 5 kW inverter wide open, only 1 kW of sink -> ramp DOWN by max(1%·5000,50)=50 W.
    let bus = one_pv(5000.0, 5000.0, 5000.0);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 1000.0, ..active_ctx() };
    let out = pv.tick(&bus, ctx);
    let (svc, w) = &pv.last_writes()[0];
    assert_eq!(svc, PV1);
    assert!((w - 4950.0).abs() < 1e-9, "one 50 W step down, got {w}");
    assert_eq!(out.pv_limiter_active, 1); // capping below nameplate
    assert_eq!(out.pv_disable, 0);
}

#[test]
fn ramp_step_is_one_percent_when_larger_than_50w() {
    // 20 kW inverter -> step = max(1%·20000, 50) = 200 W.
    let bus = one_pv(20000.0, 20000.0, 20000.0);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, ..active_ctx() };
    let out = pv.tick(&bus, ctx);
    let w = pv.last_writes()[0].1;
    assert!((w - 19800.0).abs() < 1e-9, "200 W step, got {w}");
    assert_eq!(out.pv_limiter_active, 1);
}

#[test]
fn ample_headroom_holds_at_nameplate_not_active() {
    // Plenty of sink -> ramp target >= nameplate, hold open, not limiting.
    let bus = one_pv(3000.0, 5000.0, 5000.0);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 100_000.0, ..active_ctx() };
    let out = pv.tick(&bus, ctx);
    assert_eq!(pv.last_writes()[0].1, 5000.0);
    assert_eq!(out.pv_limiter_active, 0);
}

#[test]
fn position1_ac_output_inverter_released_to_maxpower() {
    let mut bus = one_pv(5000.0, 5000.0, 5000.0);
    bus.set(PV1, P_POSITION, 1.0); // AC-output
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, ..active_ctx() };
    let out = pv.tick(&bus, ctx);
    assert_eq!(pv.last_writes()[0].1, 5000.0); // opened, not proportionally limited
    assert_eq!(out.pv_limiter_active, 0);
}

#[test]
fn battery_full_export_allowed_fast_throttles() {
    // Battery full but export permitted: aggressive cut toward export cap.
    let mut bus = one_pv(5000.0, 5000.0, 5000.0);
    bus.set(SETTINGS, P_MAXFEEDIN, 800.0);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, battery_full: true, export_blocked: false, limiter_enabled: true };
    let out = pv.tick(&bus, ctx);
    // cur·0.015 = 75 W, floored at the 800 W export share -> 800 W.
    let w = pv.last_writes()[0].1;
    assert!((w - 800.0).abs() < 1e-9, "fast-throttle to export cap, got {w}");
    assert_eq!(out.pv_limiter_active, 1);
}

#[test]
fn uncontrollable_inverter_without_nameplate_is_released_not_curtailed() {
    // Safety: never curtail blind. No Ac/MaxPower -> release sentinel.
    let mut bus = one_pv(4000.0, 4000.0, 5000.0);
    bus.clear(PV1, P_AC_MAXPOWER);
    let mut pv = PvLimiter::new(&bus);
    let ctx = PvContext { charge_headroom_w: 0.0, ..active_ctx() };
    pv.tick(&bus, ctx);
    assert_eq!(pv.last_writes(), &[(PV1.to_string(), RELEASE_W)]);
}

// ---- feed-forward EMA ----

#[test]
fn offset_ema_seeds_then_blends_0_6_0_4() {
    // First sample seeds directly; subsequent samples blend 0.6·new + 0.4·prev.
    let mut pv = PvLimiter::new(&MockBus::new());
    let mk = |p: f64| {
        let mut inv = PvInverter::read(&one_pv(p, 5000.0, 5000.0), PV1.to_string());
        inv.ac_power = Some(p);
        inv
    };
    // seed at 1000
    let o1 = pv.update_power(&[mk(1000.0)]);
    assert!((o1 - 1000.0).abs() < 1e-9);
    // 0.6·2000 + 0.4·1000 = 1600
    let o2 = pv.update_power(&[mk(2000.0)]);
    assert!((o2 - 1600.0).abs() < 1e-9, "got {o2}");
    // 0.6·2000 + 0.4·1600 = 1840 (converging toward the steady input)
    let o3 = pv.update_power(&[mk(2000.0)]);
    assert!((o3 - 1840.0).abs() < 1e-9, "got {o3}");
}

#[test]
fn offset_ema_resets_when_pv_disappears() {
    // Going back to the no-PV stub clears the EMA state and zeroes the offset.
    let bus = one_pv(3000.0, 5000.0, 5000.0);
    let mut pv = PvLimiter::new(&bus);
    let o = pv.tick(&bus, active_ctx());
    assert!(o.pv_offset_w > 0.0);
    let empty = MockBus::new();
    let out = pv.tick(&empty, PvContext::default());
    assert_eq!(out, PvOut::STUB);
    // A subsequent PV re-appearance re-seeds rather than blending stale state.
    let out2 = pv.tick(&bus, active_ctx());
    assert!((out2.pv_offset_w - 3000.0).abs() < 1e-9);
}

#[test]
fn only_producing_inverters_contribute_to_feed_forward() {
    let mut bus = one_pv(0.0, 5000.0, 5000.0);
    bus.set(PV1, P_STATUSCODE, 0.0); // idle, no output
    let mut pv = PvLimiter::new(&bus);
    let out = pv.tick(&bus, active_ctx());
    assert_eq!(out.pv_offset_w, 0.0);
}
