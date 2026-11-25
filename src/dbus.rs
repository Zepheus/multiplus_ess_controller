//! Native D-Bus access for Venus OS via `zbus` (pure Rust, no libdbus).
//!
//! A single persistent system-bus connection is opened once and reused for every
//! GetValue/SetValue, replacing the old fork+exec-per-call approach. Kept behind
//! the `Bus` trait so control logic never sees the transport.

use zbus::blocking::Connection;
use zbus::zvariant::Value;

/// Well-known services / paths on this system (verified read-only on the target GX).
pub const SYS: &str = "com.victronenergy.system";
pub const SETTINGS: &str = "com.victronenergy.settings";
pub const VEBUS: &str = "com.victronenergy.vebus.ttyS3";

pub const P_GRID: &str = "/Ac/Grid/L1/Power";
pub const P_ACTIVEIN: &str = "/Ac/ActiveIn/L1/P";
pub const P_ACOUT: &str = "/Ac/Out/L1/P";
pub const P_STATE: &str = "/SystemState/State";
pub const P_SOC: &str = "/Dc/Battery/Soc";
/// Battery DC power (W, + = charging) as systemcalc aggregates it; the hold supervisor's
/// independent witness that a discharge inhibit is actually being honoured.
pub const P_DC_BATTERY_POWER: &str = "/Dc/Battery/Power";
pub const P_SETPOINT_SETTING: &str = "/Settings/CGwacs/AcPowerSetPoint";
pub const P_HUB4MODE: &str = "/Settings/CGwacs/Hub4Mode";
pub const P_MAXDISCHARGE: &str = "/Settings/CGwacs/MaxDischargePower";
pub const P_MAXCHARGE: &str = "/Settings/CGwacs/MaxChargePower";
/// The per-phase command we write in mode 3 (on the vebus service).
pub const P_HUB4_SETPOINT: &str = "/Hub4/L1/AcPowerSetpoint";
/// Inverter pair's advertised nominal power (W); saturation reference for the loop.
pub const P_NOMINAL_INVERTER: &str = "/Ac/Out/L1/NominalInverterPower";

// --- battery-limit broker inputs (Tier A) ---
pub const P_DVCC: &str = "/Control/Dvcc"; // SYS, bool
pub const P_FIRMWARE: &str = "/FirmwareVersion"; // VEBUS, int (hex-coded; >0x414 => regular)
pub const P_DC_VOLTAGE: &str = "/Dc/0/Voltage"; // VEBUS, V
pub const P_SUSTAIN: &str = "/Hub4/Sustain"; // VEBUS, bool (firmware-owned; read only)
pub const P_MULTI_MAXCHG_I: &str = "/Dc/0/MaxChargeCurrent"; // VEBUS, A (legacy %-fallback)
pub const P_PV_CURRENT: &str = "/Dc/Pv/Current"; // SYS, A (MPPT headroom)
pub const P_ACTIVE_BMS: &str = "/ActiveBmsService"; // SYS, string (resolves BMS service)
pub const P_MAXCHARGE_PCT: &str = "/Settings/CGwacs/MaxChargePercentage";
pub const P_MAXDISCHARGE_PCT: &str = "/Settings/CGwacs/MaxDischargePercentage";
pub const P_BL_STATE: &str = "/Settings/CGwacs/BatteryLife/State";
pub const P_BL_MINSOC: &str = "/Settings/CGwacs/BatteryLife/MinimumSocLimit";
// BMS Info paths — the service is the runtime-resolved BMS name (P_ACTIVE_BMS).
pub const P_BMS_CCL: &str = "/Info/MaxChargeCurrent"; // A
pub const P_BMS_DCL: &str = "/Info/MaxDischargeCurrent"; // A
pub const BATTERY_PREFIX: &str = "com.victronenergy.battery.";

// ---------------------------------------------------------------------------
// Every D-Bus service name and path the daemon touches lives HERE, grouped by
// service. Modules import with `use crate::dbus::*;` — one canonical name per
// path, no duplicates.

// --- hub4 (com.victronenergy.hub4): the ESS loop's OWN service; systemcalc's
//     schedule/DESS delegates SetValue the override surface below ---
pub const HUB4: &str = "com.victronenergy.hub4";
/// Grid-exchange target override (W, double). Invalid/empty ⇒ absent ⇒ fall back to
/// the setpoint setting. Also the Stage-1 "trim" write path (guarded by the 300 s
/// override watchdog).
pub const P_OVR_SETPOINT: &str = "/Overrides/Setpoint";
/// Feed-in-excess policy this slot: int32 {0 = follow setting, 1 = disable, 2 = enable}.
pub const P_OVR_FEEDIN: &str = "/Overrides/FeedInExcess";
/// External force-charge trigger (e.g. the scheduled off-peak window). Same surface
/// DESS uses.
pub const P_OVR_FORCECHARGE: &str = "/Overrides/ForceCharge";
/// External discharge-power cap (W). The schedule delegate's post-target hold: once a
/// scheduled-charge window reaches its target SOC it sets `max(1, 0.8..0.95 × DC-PV)` —
/// with no PV that is **1 W** = "hold, don't discharge for the rest of the window".
/// `< 0` or invalid = no cap. Live-verified 2026-08-16 (window tail held ~+48 W against
/// a 1.9 kW load while the unclamped law wanted ≈ −1.86 kW).
pub const P_OVR_MAXDISCHARGE: &str = "/Overrides/MaxDischargePower";

// --- settings (com.victronenergy.settings) ---
pub const P_OVERVOLTAGE_FEEDIN: &str = "/Settings/CGwacs/OvervoltageFeedIn";
/// System feed-in cap in WATTS (−1 = unlimited): distributed per phase to the Multi's
/// `/Hub4/L{n}/MaxFeedInPower`; also the PV limiter's "allowed total" (NaN if unset).
pub const P_MAXFEEDIN_SETTING: &str = "/Settings/CGwacs/MaxFeedInPower";
/// Export-direction AC-in CURRENT limit in Amps (shore/peak-shave subsystem) — NOT the
/// MaxFeedInPower setting (Watts).
pub const P_AC_EXPORT_LIMIT: &str = "/Settings/CGwacs/AcExportLimit";
/// Always-peak-shave flag (int bool 0/1): firmware "always vs only above min-SOC" policy.
pub const P_ALWAYS_PEAKSHAVE: &str = "/Settings/CGwacs/AlwaysPeakShave";
pub const P_RUN_WITHOUT_METER: &str = "/Settings/CGwacs/RunWithoutGridMeter";
pub const P_GRID_METER_REQUIRED: &str = "/Settings/CGwacs/GridMeterRequired";
/// AC-input (shore/genset) current limit in AMPS; −1 = unlimited (getter → NaN).
pub const P_AC_INPUT_LIMIT: &str = "/Settings/CGwacs/AcInputLimit";

// --- system (com.victronenergy.system) ---
/// AC-input type enum: 0 = n/a, 1 = grid, 2 = genset, 3 = shore, 240 = island. Read on
/// SYS (not the vebus service).
pub const P_ACIN_SOURCE: &str = "/Ac/ActiveIn/Source";
/// Battery charge-voltage limit (V), read on SYS.
pub const P_CHARGE_VOLTAGE: &str = "/Dc/Battery/ChargeVoltage";

// --- vebus (Multi) ---
pub const P_NUM_PHASES: &str = "/Ac/NumberOfPhases";
pub const P_MULTI_STATE: &str = "/State";
pub const P_DISABLE_FEEDIN: &str = "/Hub4/DisableFeedIn";
pub const P_DNFIO: &str = "/Hub4/DoNotFeedInOvervoltage";
/// "Setpoint becomes the max-feed-in cap" flag (int 0/1). Its *validity* (does the
/// firmware expose it) is the "Multi supports TPIMF" gate.
pub const P_TPIMF: &str = "/Hub4/TargetPowerIsMaxFeedIn";
pub const P_FIX_SOLAR: &str = "/Hub4/FixSolarOffsetTo100mV";
/// Active input index (240 = disconnected).
pub const P_ACTIVE_INPUT: &str = "/Ac/ActiveIn/ActiveInput";

// --- grid meter services ---
/// Grid-meter services to enumerate/adopt (`ListNames`, first in list order).
pub const GRID_PREFIX: &str = "com.victronenergy.grid.";
/// Aggregate AC power: total grid-meter power on an adopted `com.victronenergy.grid.*`
/// service, and the production of a `com.victronenergy.pvinverter.*` service.
pub const P_AC_POWER: &str = "/Ac/Power";
/// Single-phase fallback when a meter has no aggregate `/Ac/Power`.
pub const P_METER_L1_POWER: &str = "/Ac/L1/Power";

// --- PV inverter services ---
/// Prefix enumerated to discover controllable AC PV inverters.
pub const PVINV_PREFIX: &str = "com.victronenergy.pvinverter.";
/// Writable actuator: the curtailment limit (W) on a PV inverter service.
pub const P_AC_POWERLIMIT: &str = "/Ac/PowerLimit";
pub const P_AC_MAXPOWER: &str = "/Ac/MaxPower";
/// Inverter position: 0 = AC input 1, 1 = AC output, 2 = AC input 2.
pub const P_POSITION: &str = "/Position";
pub const P_STATUSCODE: &str = "/StatusCode";

// --- battery / BMS services (the runtime-resolved active BMS service) ---
/// BMS charge request (int → bool).
pub const P_CHARGE_REQUEST: &str = "/Info/ChargeRequest";

const IFACE: &str = "com.victronenergy.BusItem";

pub trait Bus {
    fn get_f64(&self, service: &str, path: &str) -> Option<f64>;
    fn get_str(&self, service: &str, path: &str) -> Option<String>;
    /// Enumerate bus names starting with `prefix` (e.g. `com.victronenergy.battery.`).
    fn list_services(&self, prefix: &str) -> Vec<String>;
    /// Write a value. Present on the trait but the shadow caller never calls it.
    fn set_i32(&self, service: &str, path: &str, value: i32) -> Result<(), String>;
    fn set_f64(&self, service: &str, path: &str, value: f64) -> Result<(), String>;

    /// Whether the item exists on the bus, valid or not. An item published as invalid
    /// (empty array) exists; a path the service does not have does not.
    fn exists(&self, service: &str, path: &str) -> bool {
        self.get_f64(service, path).is_some()
    }
    /// The first transport fault (timeout, lost connection) since the previous call, if
    /// any. After a fault, reads may have answered "absent" for values that are present,
    /// so nothing read since the previous call may be acted on.
    fn take_fault(&self) -> Option<BusFault> {
        None
    }

    // Thin coercions over get_f64 (BusItem bools/ints arrive numeric).
    fn get_bool(&self, service: &str, path: &str) -> Option<bool> {
        self.get_f64(service, path).map(|v| v.is_finite() && v != 0.0)
    }
    fn get_i32(&self, service: &str, path: &str) -> Option<i32> {
        self.get_f64(service, path).map(|v| v as i32)
    }
}

/// Longest a single bus call may wait for its reply. A peer that is hung but still
/// connected then surfaces as a transport fault instead of freezing the loop.
pub const METHOD_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(3);

/// Persistent system-bus connection.
pub struct SystemBus {
    conn: Connection,
    fault: std::cell::RefCell<Option<BusFault>>,
}

/// A transport fault: the call got no answer.
#[derive(Debug, Clone, PartialEq)]
pub struct BusFault {
    pub what: String,
    /// The connection itself is gone (e.g. dbus-daemon restarted): only a new one helps.
    pub connection_lost: bool,
}

impl std::fmt::Display for BusFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.what)
    }
}

/// zbus reports a method timeout and a lost connection as `InputOutput`. Error replies
/// from the peer (unknown service or object, invalid value) are answers, not faults.
fn is_fault(e: &zbus::Error) -> bool {
    matches!(e, zbus::Error::InputOutput(_))
}

fn is_connection_lost(e: &zbus::Error) -> bool {
    use std::io::ErrorKind::*;
    matches!(e, zbus::Error::InputOutput(io)
        if matches!(io.kind(), BrokenPipe | ConnectionReset | ConnectionAborted | NotConnected | UnexpectedEof))
}

impl SystemBus {
    pub fn connect() -> Result<Self, String> {
        let conn = zbus::blocking::connection::Builder::system()
            .map(|b| b.method_timeout(METHOD_TIMEOUT))
            .and_then(|b| b.build())
            .map_err(|e| format!("system bus connect: {e}"))?;
        Ok(SystemBus { conn, fault: std::cell::RefCell::new(None) })
    }

    /// Run one call unless a fault is already latched (the tick is void then, and every
    /// further call would only add another timeout), recording a new fault.
    fn guarded(
        &self,
        what: &str,
        service: &str,
        path: &str,
        call: impl FnOnce() -> zbus::Result<zbus::Message>,
    ) -> Option<zbus::Message> {
        if self.fault.borrow().is_some() {
            return None;
        }
        self.note(what, service, path, call())
    }

    /// Pass a call result through, recording a transport fault for `take_fault`.
    fn note(
        &self,
        what: &str,
        service: &str,
        path: &str,
        r: zbus::Result<zbus::Message>,
    ) -> Option<zbus::Message> {
        match r {
            Ok(m) => Some(m),
            Err(e) => {
                if is_fault(&e) {
                    self.fault.borrow_mut().get_or_insert_with(|| BusFault {
                        what: format!("{what} {service}{path}: {e}"),
                        connection_lost: is_connection_lost(&e),
                    });
                }
                None
            }
        }
    }

    fn get_value(&self, service: &str, path: &str) -> Option<zbus::Message> {
        self.guarded("GetValue", service, path, || {
            self.conn.call_method(Some(service), path, Some(IFACE), "GetValue", &())
        })
    }

    fn set_variant(&self, service: &str, path: &str, v: Value) -> Result<(), String> {
        let reply = self
            .guarded("SetValue", service, path, || {
                self.conn.call_method(Some(service), path, Some(IFACE), "SetValue", &v)
            })
            .ok_or_else(|| format!("SetValue {service}{path} failed"))?;
        let ret: i32 = reply
            .body()
            .deserialize()
            .map_err(|e| format!("SetValue {service}{path} reply: {e}"))?;
        if ret == 0 {
            Ok(())
        } else {
            Err(format!("SetValue {service}{path} returned {ret}"))
        }
    }
}

impl Bus for SystemBus {
    fn get_f64(&self, service: &str, path: &str) -> Option<f64> {
        let reply = self.get_value(service, path)?;
        let body = reply.body();
        let v: Value = body.deserialize().ok()?;
        value_to_f64(&v)
    }

    fn get_str(&self, service: &str, path: &str) -> Option<String> {
        let reply = self.get_value(service, path)?;
        let body = reply.body();
        let v: Value = body.deserialize().ok()?;
        value_to_str(&v)
    }

    fn list_services(&self, prefix: &str) -> Vec<String> {
        let Some(reply) = self.guarded("ListNames", "org.freedesktop.DBus", "", || {
            self.conn.call_method(
                Some("org.freedesktop.DBus"),
                "/org/freedesktop/DBus",
                Some("org.freedesktop.DBus"),
                "ListNames",
                &(),
            )
        }) else {
            return Vec::new();
        };
        let names: Vec<String> = reply.body().deserialize().unwrap_or_default();
        names.into_iter().filter(|n| n.starts_with(prefix)).collect()
    }

    fn exists(&self, service: &str, path: &str) -> bool {
        self.get_value(service, path).is_some()
    }

    fn take_fault(&self) -> Option<BusFault> {
        self.fault.borrow_mut().take()
    }

    fn set_i32(&self, service: &str, path: &str, value: i32) -> Result<(), String> {
        self.set_variant(service, path, Value::I32(value))
    }

    fn set_f64(&self, service: &str, path: &str, value: f64) -> Result<(), String> {
        self.set_variant(service, path, Value::F64(value))
    }
}

/// Coerce any numeric D-Bus variant to f64 (BusItem values arrive as double,
/// int32, etc.). Unwraps a nested variant if the value is doubly wrapped.
fn value_to_f64(v: &Value) -> Option<f64> {
    match v {
        Value::F64(x) => Some(*x),
        Value::I16(x) => Some(*x as f64),
        Value::U16(x) => Some(*x as f64),
        Value::I32(x) => Some(*x as f64),
        Value::U32(x) => Some(*x as f64),
        Value::I64(x) => Some(*x as f64),
        Value::U64(x) => Some(*x as f64),
        Value::U8(x) => Some(*x as f64),
        Value::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
        Value::Value(inner) => value_to_f64(inner),
        _ => None,
    }
}

/// Extract a string from a BusItem variant (e.g. `/ActiveBmsService`).
fn value_to_str(v: &Value) -> Option<String> {
    match v {
        Value::Str(s) => Some(s.to_string()),
        Value::Value(inner) => value_to_str(inner),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeouts_and_lost_connections_are_faults_peer_replies_are_not() {
        let io = |k| zbus::Error::from(std::io::Error::new(k, "x"));
        assert!(is_fault(&io(std::io::ErrorKind::TimedOut)));
        assert!(is_fault(&io(std::io::ErrorKind::BrokenPipe)));
        assert!(!is_fault(&zbus::Error::Failure("no such object".into())));
        assert!(!is_fault(&zbus::Error::InvalidReply));
    }

    /// A real zbus connection to a peer that never answers: the method timeout must
    /// surface as a fault, and a peer that went away likewise.
    #[test]
    fn a_silent_or_vanished_peer_is_a_fault_through_real_zbus() {
        use zbus::blocking::connection::Builder;
        let (a, b) = std::os::unix::net::UnixStream::pair().unwrap();
        let guid = zbus::Guid::generate();
        let server = std::thread::spawn(move || {
            Builder::async_io_unix_stream(b).server(guid).unwrap().p2p().build().unwrap()
        });
        let conn = Builder::async_io_unix_stream(a)
            .p2p()
            .method_timeout(std::time::Duration::from_millis(200))
            .build()
            .unwrap();
        let peer = server.join().unwrap(); // connected, never serves the object
        let bus = SystemBus { conn, fault: std::cell::RefCell::new(None) };

        // The peer runs no object server, so the call goes unanswered and must time out.
        let t0 = std::time::Instant::now();
        assert_eq!(bus.get_f64("org.example", "/x"), None);
        // While the fault is latched the tick is void: further calls return at once.
        let t1 = std::time::Instant::now();
        assert_eq!(bus.get_f64("org.example", "/y"), None);
        assert!(bus.set_i32("org.example", "/y", 1).is_err());
        assert!(t1.elapsed() < std::time::Duration::from_millis(50));
        let f = bus.take_fault().expect("timeout is a fault");
        assert!(f.what.contains("GetValue") && !f.connection_lost, "{f:?}");
        assert!(t0.elapsed() < std::time::Duration::from_secs(2));

        drop(peer);
        std::thread::sleep(std::time::Duration::from_millis(100));
        assert_eq!(bus.get_f64("org.example", "/x"), None);
        let f = bus.take_fault().expect("a vanished peer is a fault");
        assert!(f.connection_lost, "{f:?}");
    }
}
