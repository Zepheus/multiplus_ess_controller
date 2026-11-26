//! Where the loop's grid reading comes from, and whose publish can pace the loop.
//!
//! - `Meter`: the adopted grid-meter service (`com.victronenergy.grid.*`, `/Ac/L1/Power`).
//!   The freshest reading on the bus: the meter service publishes each update once per
//!   second.
//! - `System`: systemcalc's republished `/Ac/Grid/L1/Power`. Since systemcalc ticks on
//!   whole seconds (v3.85 and later), its tick fires just before it takes in the meter's
//!   latest publish, so this copy is one meter update (~1 s) older than the meter's own.
//!
//! The same choice names the service whose `ItemsChanged` paces the loop in event mode,
//! so a tick always follows the publish of the value it reads.

use crate::dbus::{Bus, GRID_PREFIX, P_GRID, P_METER_L1_POWER, SYS};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridSource {
    Meter,
    System,
}

impl GridSource {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "meter" => Some(GridSource::Meter),
            "systemcalc" => Some(GridSource::System),
            _ => None,
        }
    }

    /// Status-service code: 0 meter, 1 systemcalc.
    pub fn code(self) -> i32 {
        match self {
            GridSource::Meter => 0,
            GridSource::System => 1,
        }
    }

    /// The (service, path) to read this tick. `meter` is the grid-meter guard's adopted
    /// service; until it has adopted one, the meter source reads systemcalc's copy.
    pub fn read_path(self, meter: Option<&str>) -> (&str, &'static str) {
        match (self, meter) {
            (GridSource::Meter, Some(m)) => (m, P_METER_L1_POWER),
            _ => (SYS, P_GRID),
        }
    }

    /// The well-known name whose publishes pace the loop, picked from the bus's names.
    pub fn publisher(self, names: &[String]) -> Option<String> {
        match self {
            // First in list order: the meter the grid-meter guard adopts.
            GridSource::Meter => names.iter().find(|n| n.starts_with(GRID_PREFIX)).cloned(),
            GridSource::System => names.iter().find(|n| *n == SYS).cloned(),
        }
    }
}

/// The grid reading from `source`, or the "service/path" that gave no finite value.
pub fn read(bus: &dyn Bus, source: GridSource, meter: Option<&str>) -> Result<f64, String> {
    let (svc, path) = source.read_path(meter);
    bus.get_f64(svc, path).filter(|v| v.is_finite()).ok_or_else(|| format!("{svc}{path}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    const METER: &str = "com.victronenergy.grid.cgwacs_ttyUSB0_mb1";

    #[test]
    fn a_non_finite_reading_is_no_reading() {
        let mut bus = crate::testbus::MockBus::new();
        bus.set(METER, P_METER_L1_POWER, f64::NAN);
        assert_eq!(read(&bus, GridSource::Meter, Some(METER)), Err(format!("{METER}{P_METER_L1_POWER}")));
        bus.set(METER, P_METER_L1_POWER, -40.0);
        assert_eq!(read(&bus, GridSource::Meter, Some(METER)), Ok(-40.0));
    }

    #[test]
    fn flags_parse_to_sources() {
        assert_eq!(GridSource::parse("meter"), Some(GridSource::Meter));
        assert_eq!(GridSource::parse("systemcalc"), Some(GridSource::System));
        assert_eq!(GridSource::parse("grid"), None);
    }

    #[test]
    fn the_meter_source_reads_the_adopted_meter_and_systemcalc_until_then() {
        assert_eq!(GridSource::Meter.read_path(Some(METER)), (METER, P_METER_L1_POWER));
        assert_eq!(GridSource::Meter.read_path(None), (SYS, P_GRID));
        assert_eq!(GridSource::System.read_path(Some(METER)), (SYS, P_GRID));
    }

    #[test]
    fn the_publisher_is_the_service_the_reading_comes_from() {
        let names: Vec<String> =
            ["org.freedesktop.DBus", SYS, METER, "com.victronenergy.grid.other"].map(String::from).into();
        assert_eq!(GridSource::Meter.publisher(&names).as_deref(), Some(METER));
        assert_eq!(GridSource::System.publisher(&names).as_deref(), Some(SYS));
        assert_eq!(GridSource::Meter.publisher(&[SYS.to_string()]), None);
    }
}
