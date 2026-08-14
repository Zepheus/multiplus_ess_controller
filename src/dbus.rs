//! D-Bus access for Venus OS, via the device's own `dbus-send` CLI.
//!
//! Kept behind the `Bus` trait so the transport can later be swapped for native
//! `zbus` without touching control logic.  Reads use `GetValue`; writes use
//! `SetValue` and are ONLY reachable through `Bus::set_*`, which the shadow-mode
//! caller never invokes.

use std::process::Command;

/// Well-known services / paths on this system (verified read-only on venus.local).
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

pub trait Bus {
    fn get_f64(&self, service: &str, path: &str) -> Option<f64>;
    /// Write a value. Present on the trait but the shadow caller never calls it.
    fn set_i32(&self, service: &str, path: &str, value: i32) -> Result<(), String>;
    fn set_f64(&self, service: &str, path: &str, value: f64) -> Result<(), String>;
}

/// Talks to the local system bus by shelling out to `dbus-send`.
/// Fast (native C tool, ~5 ms) and always present on Venus OS.
pub struct DbusSend;

impl DbusSend {
    fn get_raw(&self, service: &str, path: &str) -> Option<String> {
        let out = Command::new("dbus-send")
            .args([
                "--system",
                "--print-reply=literal",
                &format!("--dest={service}"),
                path,
                "com.victronenergy.BusItem.GetValue",
            ])
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        Some(String::from_utf8_lossy(&out.stdout).to_string())
    }
}

/// Parse the numeric tail of a dbus-send literal reply, e.g.
/// "   variant       double 83.2"  ->  83.2 ;  "   variant  int32 256" -> 256
fn parse_number(reply: &str) -> Option<f64> {
    reply.split_whitespace().last()?.parse::<f64>().ok()
}

impl Bus for DbusSend {
    fn get_f64(&self, service: &str, path: &str) -> Option<f64> {
        parse_number(&self.get_raw(service, path)?)
    }

    fn set_i32(&self, service: &str, path: &str, value: i32) -> Result<(), String> {
        run_set(service, path, &format!("variant:int32:{value}"))
    }

    fn set_f64(&self, service: &str, path: &str, value: f64) -> Result<(), String> {
        run_set(service, path, &format!("variant:double:{value}"))
    }
}

fn run_set(service: &str, path: &str, variant: &str) -> Result<(), String> {
    let out = Command::new("dbus-send")
        .args([
            "--system",
            "--print-reply",
            &format!("--dest={service}"),
            path,
            "com.victronenergy.BusItem.SetValue",
            variant,
        ])
        .output()
        .map_err(|e| format!("dbus-send exec failed: {e}"))?;
    if out.status.success() {
        Ok(())
    } else {
        Err(format!(
            "SetValue {service}{path} := {variant} failed: {}",
            String::from_utf8_lossy(&out.stderr).trim()
        ))
    }
}
