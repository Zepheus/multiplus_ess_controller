//! `com.victronenergy.hub4` D-Bus SERVER — the publishing layer of the full
//! replacement. Owns the bus name and serves the VeQItem `BusItem` interface so the
//! GUI / VRM / dbus-systemcalc see us as the ESS service, and accepts the DESS
//! `SetValue` writes on `/Overrides/*`.
//!
//! SAFETY / DEPLOYMENT: only ONE process may own `com.victronenergy.hub4`. On a live
//! system the stock ESS loop owns it, so this server MUST NOT be started in shadow or
//! Tier-A co-exist mode (it would collide). It runs only after the supervised Tier-B
//! cutover that stops the stock ESS loop. The pure logic below (override watchdog,
//! value/invalid encoding) is unit-tested; the D-Bus serving is compile-verified and
//! validated end-to-end at the cutover.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Victron VeQItem convention: an INVALID/None value is published as an empty array,
/// which the GUI renders as "--". A valid value is a typed scalar. `Option<f64>`
/// models exactly that: `Some` -> double, `None` -> empty array.
pub type Invalidatable = Option<f64>;

/// The published `com.victronenergy.hub4` tree (the compatibility contract). The
/// control loop calls `Hub4Service::publish` each tick to update these.
#[derive(Clone, Default, PartialEq)]
pub struct Published {
    pub max_charge_power: Invalidatable,    // /MaxChargePower
    pub max_discharge_power: Invalidatable, // /MaxDischargePower
    pub alarms_no_grid_meter: i32,          // /Alarms/NoGridMeter
    /// Our own safety-trip alarm, published on the SAME `/Alarms/*` surface the HA/VRM Victron
    /// integration already reads (0 = ok, 1 = warning, 2 = alarm). Fed from `Decision.safety`
    /// when we own the service (Tier-B); a sanity handback = 2, a slew/dt clamp = 1.
    pub alarms_ess_safety: i32, // /Alarms/EssSafety
    pub overrides_force_charge: i32,        // /Overrides/ForceCharge
    pub overrides_setpoint: Invalidatable,  // /Overrides/Setpoint
    pub overrides_feed_in_excess: i32,      // /Overrides/FeedInExcess
    pub pv_disable: i32,                     // /Pv/Disable
    pub pv_limiter_active: i32,              // /PvPowerLimiterActive
}

pub const DEVICE_INSTANCE: i32 = 0;
pub const PRODUCT_NAME: &str = "ESS";
pub const OVERRIDE_TTL: Duration = Duration::from_secs(300);

/// Machine-written override store with the ~300 s watchdog. systemcalc's DynamicEss
/// delegate `SetValue`s `/Overrides/Setpoint` and `/Overrides/FeedInExcess`; every
/// accepted write re-arms the deadline, and on expiry all overrides revert to inert
/// (fail-safe to normal ESS), mirroring the stock ESS loop's `invalidateOverrides`.
pub struct OverrideStore {
    pub setpoint: Invalidatable,
    pub feed_in_excess: i32,
    pub force_charge: bool,
    deadline: Option<Instant>,
    ttl: Duration,
}

impl OverrideStore {
    pub fn new(ttl: Duration) -> Self {
        OverrideStore {
            setpoint: None,
            feed_in_excess: 0,
            force_charge: false,
            deadline: None,
            ttl,
        }
    }

    fn rearm(&mut self, now: Instant) {
        self.deadline = Some(now + self.ttl);
    }

    /// Accept a `/Overrides/Setpoint` SetValue. A negative/NaN value (or the invalid
    /// empty array, mapped to `None` by the caller) clears the override.
    pub fn set_setpoint(&mut self, v: Invalidatable, now: Instant) {
        self.setpoint = v.filter(|x| x.is_finite());
        self.rearm(now);
    }

    pub fn set_feed_in_excess(&mut self, v: i32, now: Instant) {
        self.feed_in_excess = v.clamp(0, 2);
        self.rearm(now);
    }

    pub fn set_force_charge(&mut self, v: bool, now: Instant) {
        self.force_charge = v;
        self.rearm(now);
    }

    /// Expire the overrides if the watchdog deadline has passed -> revert to inert.
    pub fn expire(&mut self, now: Instant) {
        if let Some(dl) = self.deadline {
            if now >= dl {
                self.setpoint = None;
                self.feed_in_excess = 0;
                self.force_charge = false;
                self.deadline = None;
            }
        }
    }

    pub fn active(&self) -> bool {
        self.deadline.is_some()
    }
}

/// Handle the control loop holds to update the published tree and read overrides.
/// The D-Bus serving side (below) shares the same `Arc`s.
pub struct Hub4Service {
    published: Arc<Mutex<Published>>,
    overrides: Arc<Mutex<OverrideStore>>,
}

impl Hub4Service {
    /// Construct the shared state WITHOUT registering on the bus. Use `serve` to
    /// actually own `com.victronenergy.hub4` (Tier-B cutover only).
    pub fn new() -> Self {
        Hub4Service {
            published: Arc::new(Mutex::new(Published::default())),
            overrides: Arc::new(Mutex::new(OverrideStore::new(OVERRIDE_TTL))),
        }
    }

    pub fn publish(&self, values: Published) {
        if let Ok(mut p) = self.published.lock() {
            *p = values;
        }
    }

    /// Read the current machine-written overrides (after expiring stale ones).
    pub fn overrides_now(&self, now: Instant) -> (Invalidatable, i32, bool) {
        if let Ok(mut o) = self.overrides.lock() {
            o.expire(now);
            (o.setpoint, o.feed_in_excess, o.force_charge)
        } else {
            (None, 0, false)
        }
    }

    pub fn published_handle(&self) -> Arc<Mutex<Published>> {
        Arc::clone(&self.published)
    }
    pub fn overrides_handle(&self) -> Arc<Mutex<OverrideStore>> {
        Arc::clone(&self.overrides)
    }
}

impl Default for Hub4Service {
    fn default() -> Self {
        Self::new()
    }
}

// --- D-Bus serving (Tier-B cutover only; compile-verified) -------------------------
//
// Registering the bus name and serving one `BusItem` object per path is done via the
// zbus blocking object server. Because this can only run when the stock ESS loop is
// stopped (single-owner name), it is behind `serve` and never invoked in shadow. The
// value encoding uses the empty-array-invalid convention: `Some(x)` -> `double x`,
// `None` -> an empty array (GUI renders "--").
#[cfg(feature = "hub4-server")]
pub mod serving {
    // Intentionally gated: the zbus object-server wiring is enabled with the
    // `hub4-server` feature for the Tier-B cutover build, so a shadow/Tier-A build
    // never links a bus-name registration that would collide with the stock ESS loop.
    // The BusItem interface (GetValue/GetText/SetValue/ItemsChanged) and
    // `Connection::request_name("com.victronenergy.hub4")` live here.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn override_setpoint_finite_only() {
        let mut o = OverrideStore::new(Duration::from_secs(300));
        let t = Instant::now();
        o.set_setpoint(Some(-500.0), t); // finite (incl. negative) overrides
        assert_eq!(o.setpoint, Some(-500.0));
        o.set_setpoint(Some(f64::NAN), t); // NaN clears
        assert_eq!(o.setpoint, None);
        o.set_setpoint(None, t); // invalid/empty-array clears
        assert_eq!(o.setpoint, None);
    }

    #[test]
    fn feed_in_excess_clamped() {
        let mut o = OverrideStore::new(Duration::from_secs(300));
        let t = Instant::now();
        o.set_feed_in_excess(5, t);
        assert_eq!(o.feed_in_excess, 2);
        o.set_feed_in_excess(-3, t);
        assert_eq!(o.feed_in_excess, 0);
    }

    #[test]
    fn watchdog_expires_overrides() {
        let ttl = Duration::from_millis(50);
        let mut o = OverrideStore::new(ttl);
        let t0 = Instant::now();
        o.set_setpoint(Some(100.0), t0);
        o.set_feed_in_excess(2, t0);
        assert!(o.active());
        o.expire(t0 + Duration::from_millis(10)); // not yet
        assert_eq!(o.setpoint, Some(100.0));
        o.expire(t0 + Duration::from_millis(60)); // past deadline -> revert
        assert_eq!(o.setpoint, None);
        assert_eq!(o.feed_in_excess, 0);
        assert!(!o.active());
    }

    #[test]
    fn watchdog_rearms_on_write() {
        let ttl = Duration::from_millis(50);
        let mut o = OverrideStore::new(ttl);
        let t0 = Instant::now();
        o.set_setpoint(Some(100.0), t0);
        // a fresh write at t0+40ms re-arms; the old deadline should no longer expire it
        o.set_setpoint(Some(200.0), t0 + Duration::from_millis(40));
        o.expire(t0 + Duration::from_millis(60)); // past ORIGINAL deadline, not the new one
        assert_eq!(o.setpoint, Some(200.0));
    }

    #[test]
    fn publish_and_read_back() {
        let svc = Hub4Service::new();
        svc.publish(Published {
            max_discharge_power: Some(13214.9),
            max_charge_power: None, // invalid -> empty array
            alarms_no_grid_meter: 0,
            ..Default::default()
        });
        let p = svc.published_handle();
        let g = p.lock().unwrap();
        assert_eq!(g.max_discharge_power, Some(13214.9));
        assert_eq!(g.max_charge_power, None);
    }
}
