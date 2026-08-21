//! Dynamic ESS (DESS) override subsystem.
//!
//! Reads the two DESS overrides that systemcalc's `DynamicEss` delegate writes onto
//! the stock ESS loop's own `com.victronenergy.hub4` service and turns them into the
//! effective-target override the control law consumes. This is the CONSUMER half of
//! the `/Overrides/*` transport empirically modelled in the design notes (§4, §6):
//!
//!   * `/Overrides/Setpoint` (double, invalid = empty array) — grid-exchange target
//!     in W (+import / 0 hold / −export). The core tick's effective-setpoint getter
//!     (``) is exactly `isFinite(override) ? override : configured`, so
//!     DESS "overriding the setpoint" is literally *swapping the target input*; the
//!     feed-forward control law is untouched.
//!   * `/Overrides/FeedInExcess` (int32 {0,1,2}) — 0 = follow the user's configured
//!     feed-in policy, 1 = disable feed-in this slot, 2 = enable feed-in this slot
//!     (``, pinned on both producer and consumer side).
//!
//! Safety posture (matches the stock ESS loop's `invalidateOverrides`, `` case 9):
//!   * An absent, empty-array, or NaN setpoint override is NOT a value — we fall back
//!     to the user's configured setpoint. A finite override (including `0.0` and
//!     negatives) genuinely overrides; `0` and invalid are never conflated (§5).
//!   * The ~300 s overrides watchdog: if the optimiser stops refreshing the overrides,
//!     we fail safe back to normal ESS — setpoint → configured, feed-in → follow user.
//!
//! On this install DESS is currently OFF, so the hot path is "override absent → use
//! the configured setpoint", which this module produces with `target_override = None`.
//!
//! Tier scope: read-only consumer + watchdog. The hub4 D-Bus server that systemcalc
//! writes into (§6.2) is a sibling concern; here we read the applied override values
//! through the `Bus` trait, exactly like the battery-limit broker reads its inputs.

use crate::dbus::Bus;
use std::time::{Duration, Instant};

// --- DESS override paths on the stock ESS loop's OWN service (systemcalc is the writer) ---
/// The service the stock ESS loop owns and publishes; systemcalc `SetValue`s the overrides here.
pub const HUB4: &str = "com.victronenergy.hub4";
/// Grid-exchange target (W). Invalid/empty-array ⇒ read as absent ⇒ fall back. Double.
pub const P_OVR_SETPOINT: &str = "/Overrides/Setpoint";
/// Feed-in-excess policy this slot. int32 {0=follow, 1=disable, 2=enable}. Default 0.
pub const P_OVR_FEEDIN: &str = "/Overrides/FeedInExcess";

/// Overrides watchdog period. systemcalc refreshes every 5 s; any value comfortably
/// above that works. 300 s matches Victron's documented overrides watchdog (§4.4);
/// the exact literal is not in the analysis, so this is the safe documented value.
const WATCHDOG: Duration = Duration::from_secs(300);

/// Feed-in-excess policy — the DESS `/Overrides/FeedInExcess` int, decoded.
///
/// Producer (`dynamicess.py`): `0 if allow_feedin is None else 2 if allow_feedin else 1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedInExcess {
    /// 0 — follow the user's configured feed-in policy (PreventFeedback / MaxFeedInPower).
    Follow = 0,
    /// 1 — disable grid feed-in of excess this slot.
    Disable = 1,
    /// 2 — enable grid feed-in of excess this slot.
    Enable = 2,
}

impl FeedInExcess {
    /// Decode the raw override int. Anything other than 1/2 is `Follow` (systemcalc's
    /// default-0 "follow system setup"), so a garbage value fails safe to the user policy.
    pub fn from_i32(v: i32) -> Self {
        match v {
            1 => FeedInExcess::Disable,
            2 => FeedInExcess::Enable,
            _ => FeedInExcess::Follow,
        }
    }
}

/// The DESS override, resolved for one control tick.
///
/// `target_override` is `Some(x)` only for a *fresh, finite* setpoint override; it is
/// `None` when the override is absent, invalid (empty array), NaN, or has gone stale —
/// in every one of those cases the caller falls back to the configured setpoint.
/// `feed_in_excess` is the raw {0,1,2} value (reset to 0 by the watchdog when stale).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DessOut {
    pub target_override: Option<f64>,
    pub feed_in_excess: i32,
}

impl DessOut {
    /// Step 2 of the per-tick order: `target = override.is_finite() ? override : configured`.
    /// This is the whole point of the setpoint override — swap the target, nothing else.
    pub fn target(&self, configured_setpoint: f64) -> f64 {
        self.target_override.unwrap_or(configured_setpoint)
    }

    /// The decoded feed-in-excess policy.
    pub fn feedin(&self) -> FeedInExcess {
        FeedInExcess::from_i32(self.feed_in_excess)
    }
}

/// DESS override consumer. Reads `/Overrides/*` off the hub4 service each tick and
/// applies the ~300 s staleness watchdog, failing safe to normal ESS.
///
/// Watchdog model (poller edition): we can only observe the applied override values,
/// not systemcalc's `SetValue` timestamps, so we treat any *change* in the observed
/// override (setpoint or feed-in) as a refresh that re-arms the watchdog — the reader's
/// equivalent of the stock ESS loop's per-write QTimer re-arm (§4.4). If nothing changes for
/// longer than `WATCHDOG`, the optimiser is presumed dead and we revert to normal ESS.
/// Reverting is always fail-safe (back to the user's static setpoint and feed-in policy),
/// so a legitimately constant override that trips the watchdog only costs optimisation,
/// never safety. (When this crate also OWNS the hub4 server, re-arm on SetValue instead.)
pub struct Dess {
    last_setpoint: Option<f64>, // last observed FINITE setpoint (None = absent/invalid/NaN)
    last_feedin: i32,           // last observed raw feed-in int
    last_activity: Instant,     // last time an observed override value changed (re-arm)
    armed: bool,                // an override has been seen since the last (re)arm
}

impl Dess {
    pub fn new(_bus: &dyn Bus) -> Self {
        Dess {
            last_setpoint: None,
            last_feedin: 0,
            last_activity: Instant::now(),
            armed: false,
        }
    }

    /// Read the live overrides and resolve them for this tick. Cheap; call once per tick.
    pub fn tick(&mut self, bus: &dyn Bus) -> DessOut {
        self.tick_at(bus, Instant::now())
    }

    /// Time-injected core, so the watchdog is unit-testable without sleeping.
    fn tick_at(&mut self, bus: &dyn Bus, now: Instant) -> DessOut {
        // Setpoint: a numeric read that is NON-finite (NaN) is not a value; the empty-array
        // "invalid" encoding surfaces as `None` from get_f64. Both fall back to configured.
        let setpoint = match bus.get_f64(HUB4, P_OVR_SETPOINT) {
            Some(x) if x.is_finite() => Some(x), // finite incl. 0.0 and negatives overrides
            _ => None,                           // absent / empty-array / NaN ⇒ no override
        };
        // FeedInExcess defaults to 0 ("follow user") when absent, matching the DESS-off state.
        let feedin = bus.get_i32(HUB4, P_OVR_FEEDIN).unwrap_or(0);

        // Re-arm the watchdog whenever the producer moves either override. A first-ever
        // observation of any active override also arms and stamps activity.
        let active = setpoint.is_some() || feedin != 0;
        let changed = setpoint != self.last_setpoint || feedin != self.last_feedin;
        if changed {
            self.last_setpoint = setpoint;
            self.last_feedin = feedin;
            self.last_activity = now;
            self.armed = active;
        } else if active && !self.armed {
            // Override present from the very first tick (no prior change edge to catch).
            self.last_activity = now;
            self.armed = true;
        }

        // Watchdog: if an override was armed and has gone stale, fail safe to normal ESS —
        // exactly the stock ESS loop's invalidateOverrides: setpoint → configured, feed-in → follow.
        let stale = self.armed && now.duration_since(self.last_activity) > WATCHDOG;
        if stale {
            self.armed = false;
            return DessOut { target_override: None, feed_in_excess: 0 };
        }

        DessOut {
            target_override: setpoint,
            feed_in_excess: feedin,
        }
    }
}

#[cfg(test)]
#[path = "dess_tests.rs"]
mod tests;
