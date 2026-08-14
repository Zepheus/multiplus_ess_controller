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
pub const P_STATE: &str = "/SystemState/State";
pub const P_SOC: &str = "/Dc/Battery/Soc";
pub const P_SETPOINT_SETTING: &str = "/Settings/CGwacs/AcPowerSetPoint";
pub const P_HUB4MODE: &str = "/Settings/CGwacs/Hub4Mode";
pub const P_MAXDISCHARGE: &str = "/Settings/CGwacs/MaxDischargePower";
pub const P_MAXCHARGE: &str = "/Settings/CGwacs/MaxChargePower";
/// The per-phase command we write in mode 3 (on the vebus service).
pub const P_HUB4_SETPOINT: &str = "/Hub4/L1/AcPowerSetpoint";

const IFACE: &str = "com.victronenergy.BusItem";

pub trait Bus {
    fn get_f64(&self, service: &str, path: &str) -> Option<f64>;
    /// Write a value. Present on the trait but the shadow caller never calls it.
    fn set_i32(&self, service: &str, path: &str, value: i32) -> Result<(), String>;
    fn set_f64(&self, service: &str, path: &str, value: f64) -> Result<(), String>;
}

/// Persistent system-bus connection.
pub struct SystemBus {
    conn: Connection,
}

impl SystemBus {
    pub fn connect() -> Result<Self, String> {
        let conn = Connection::system().map_err(|e| format!("system bus connect: {e}"))?;
        Ok(SystemBus { conn })
    }

    fn set_variant(&self, service: &str, path: &str, v: Value) -> Result<(), String> {
        let reply = self
            .conn
            .call_method(Some(service), path, Some(IFACE), "SetValue", &v)
            .map_err(|e| format!("SetValue {service}{path}: {e}"))?;
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
        let reply = self
            .conn
            .call_method(Some(service), path, Some(IFACE), "GetValue", &())
            .ok()?;
        let body = reply.body();
        let v: Value = body.deserialize().ok()?;
        value_to_f64(&v)
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
