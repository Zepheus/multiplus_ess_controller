//! Controller status and alarms on D-Bus, for the GX's MQTT bridge and anything behind it
//! (Home Assistant, Node-RED).
//!
//! The service `com.victronenergy.essctl.controller` follows the Victron item conventions
//! (`GetItems` and `ItemsChanged` on `/`, `GetValue`/`GetText` per path, `/DeviceInstance`),
//! so dbus-flashmq publishes it as `N/<portal id>/essctl/0/<path>` with no configuration.
//!
//! Its own thread recomputes every value once per second from what the control loop
//! leaves in [`Shared`]. A loop that stops running shows up as ages that keep growing and
//! an alarm. `/Status/Ticks` counts loop iterations, so a consumer can also tell a frozen
//! process (nothing changes at all) from a healthy one (the counter moves every second).
//! If the process exits, the service leaves the bus and its MQTT topics go empty.
//!
//! Alarm values use the Victron convention: 0 ok, 1 warning, 2 alarm.

use crate::dbus::IFACE;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use zbus::zvariant::{OwnedValue, Value};

pub const SERVICE: &str = "com.victronenergy.essctl.controller";
const REFRESH: Duration = Duration::from_secs(1);
const RETRY: Duration = Duration::from_secs(10);

/// The loop has not completed an iteration (of any kind) for this long: it is stuck.
pub const LOOP_STALL_S: f64 = 10.0;
/// No complete tick (all core inputs read) for this long: warning, then alarm.
pub const INPUTS_WARN_S: f64 = 3.0;
pub const INPUTS_ALARM_S: f64 = 10.0;
/// The grid reading has not changed for this long: the source is frozen.
pub const GRID_FROZEN_WARN_S: f64 = 10.0;
pub const GRID_FROZEN_ALARM_S: f64 = 30.0;
/// Owning the loop without a setpoint write for this long. The Multi drops to passthru
/// after 60 s without a setpoint.
pub const NO_WRITE_WARN_S: f64 = 10.0;
pub const NO_WRITE_ALARM_S: f64 = 60.0;

/// What the loop reports. Timestamps rather than ages, so every age is computed when it
/// is published.
#[derive(Debug, Clone)]
pub struct Status {
    pub started: Instant,
    /// Stage: 0 shadow, 1 trim, 2 takeover.
    pub stage: i32,
    /// Grid source: 0 meter, 1 systemcalc.
    pub grid_source: i32,
    /// Whether ticks follow the grid source's publishes.
    pub events_mode: bool,
    pub owner_us: bool,
    pub ceded_external: bool,
    pub sanity_trips: u32,
    /// The latest wait ended on a publish (true) or on the timer (false).
    pub on_events: bool,
    /// Loop iterations so far.
    pub ticks: u32,
    /// End of the loop's latest iteration, whether or not it could read its inputs.
    pub loop_alive: Option<Instant>,
    /// Latest tick that read every core input.
    pub good_tick: Option<Instant>,
    /// Latest grid reading, and when it last differed from the one before.
    pub grid: Option<f64>,
    pub grid_changed: Option<Instant>,
    /// Latest setpoint write that reached the Multi.
    pub wrote: Option<Instant>,
}

impl Status {
    pub fn new(started: Instant, stage: i32, source: crate::gridsource::GridSource, events_mode: bool) -> Self {
        Status {
            started,
            stage,
            grid_source: source.code(),
            events_mode,
            owner_us: false,
            ceded_external: false,
            sanity_trips: 0,
            on_events: false,
            ticks: 0,
            loop_alive: None,
            good_tick: None,
            grid: None,
            grid_changed: None,
            wrote: None,
        }
    }

    /// Record a tick that read every core input.
    pub fn good_tick(&mut self, now: Instant, grid: f64, wrote_setpoint: bool) {
        self.good_tick = Some(now);
        if self.grid != Some(grid) {
            self.grid = Some(grid);
            self.grid_changed = Some(now);
        }
        if wrote_setpoint {
            self.wrote = Some(now);
        }
    }
}

pub type Shared = Arc<Mutex<Status>>;

/// Lock without letting a panicked writer take the status service down with it.
pub fn lock(s: &Shared) -> std::sync::MutexGuard<'_, Status> {
    s.lock().unwrap_or_else(|e| e.into_inner())
}

/// One published value. `None` is a Victron "invalid" (an empty array on the bus).
#[derive(Debug, Clone, PartialEq)]
pub enum Val {
    Int(i32),
    Num(f64),
    Text(String),
    None,
}

/// Seconds since `at`, measured from `started` while `at` is unset (nothing seen yet).
fn age(now: Instant, at: Option<Instant>, started: Instant) -> f64 {
    now.duration_since(at.unwrap_or(started)).as_secs_f64()
}

fn level(age: f64, warn: f64, alarm: f64) -> i32 {
    if age > alarm {
        2
    } else if age > warn {
        1
    } else {
        0
    }
}

/// Every published path and its value at `now`. Pure, so the alarm rules are testable.
pub fn items(s: &Status, now: Instant) -> Vec<(&'static str, Val)> {
    let loop_age = age(now, s.loop_alive, s.started);
    let input_age = age(now, s.good_tick, s.started);
    let grid_age = age(now, s.grid_changed, s.started);
    let write_age = age(now, s.wrote, s.started);
    let takeover = s.stage == 2;

    let loop_stalled = if loop_age > LOOP_STALL_S { 2 } else { 0 };
    let inputs = level(input_age, INPUTS_WARN_S, INPUTS_ALARM_S);
    let grid_frozen = level(grid_age, GRID_FROZEN_WARN_S, GRID_FROZEN_ALARM_S);
    let not_in_control = i32::from(takeover && (!s.owner_us || s.ceded_external));
    let not_writing = if takeover && s.owner_us {
        level(write_age, NO_WRITE_WARN_S, NO_WRITE_ALARM_S)
    } else {
        0
    };
    let summary = [loop_stalled, inputs, grid_frozen, not_in_control, not_writing].into_iter().max().unwrap_or(0);

    vec![
        ("/Mgmt/ProcessName", Val::Text(env!("CARGO_PKG_NAME").into())),
        ("/Mgmt/ProcessVersion", Val::Text(env!("CARGO_PKG_VERSION").into())),
        ("/Mgmt/Connection", Val::Text("internal".into())),
        ("/DeviceInstance", Val::Int(0)),
        ("/ProductName", Val::Text("ESS controller".into())),
        ("/Connected", Val::Int(1)),
        ("/Status/Stage", Val::Int(s.stage)),
        ("/Status/Owner", Val::Int(i32::from(s.owner_us))),
        ("/Status/CededExternal", Val::Int(i32::from(s.ceded_external))),
        ("/Status/SanityTrips", Val::Int(s.sanity_trips as i32)),
        ("/Status/GridSource", Val::Int(s.grid_source)),
        // 0 ticking on publishes, 1 on the failsafe timer, 2 timer pacing configured.
        ("/Status/Pacing", Val::Int(if !s.events_mode { 2 } else if s.on_events { 0 } else { 1 })),
        ("/Status/Ticks", Val::Int(s.ticks as i32)),
        ("/Status/LoopAge", Val::Num(loop_age.floor())),
        ("/Status/InputAge", Val::Num(input_age.floor())),
        ("/Status/GridUnchangedFor", Val::Num(grid_age.floor())),
        ("/Status/WriteAge", if takeover { Val::Num(write_age.floor()) } else { Val::None }),
        ("/Alarms/LoopStalled", Val::Int(loop_stalled)),
        ("/Alarms/InputsStale", Val::Int(inputs)),
        ("/Alarms/GridFrozen", Val::Int(grid_frozen)),
        ("/Alarms/NotInControl", Val::Int(not_in_control)),
        ("/Alarms/NotWriting", Val::Int(not_writing)),
        ("/Alarms/Alarm", Val::Int(summary)),
    ]
}

fn to_value(v: &Val) -> OwnedValue {
    let value = match v {
        Val::Int(i) => Value::from(*i),
        Val::Num(f) => Value::from(*f),
        Val::Text(t) => Value::from(t.as_str()),
        Val::None => Value::from(Vec::<i32>::new()),
    };
    value.try_to_owned().expect("plain values always convert")
}

fn to_text(v: &Val) -> String {
    match v {
        Val::Int(i) => i.to_string(),
        Val::Num(f) => format!("{f:.0}"),
        Val::Text(t) => t.clone(),
        Val::None => "---".into(),
    }
}

type ItemMap = HashMap<String, HashMap<String, OwnedValue>>;

fn item_map(list: &[(&'static str, Val)]) -> ItemMap {
    list.iter()
        .map(|(p, v)| {
            let entry = HashMap::from([
                ("Value".to_string(), to_value(v)),
                ("Text".to_string(), Value::from(to_text(v)).try_to_owned().expect("text converts")),
            ]);
            (p.to_string(), entry)
        })
        .collect()
}

struct Root {
    shared: Shared,
}

#[zbus::interface(name = "com.victronenergy.BusItem")]
impl Root {
    fn get_items(&self) -> ItemMap {
        let s = lock(&self.shared).clone();
        item_map(&items(&s, Instant::now()))
    }
}

struct Leaf {
    path: &'static str,
    shared: Shared,
}

impl Leaf {
    fn current(&self) -> Val {
        let s = lock(&self.shared).clone();
        items(&s, Instant::now())
            .into_iter()
            .find(|(p, _)| *p == self.path)
            .map_or(Val::None, |(_, v)| v)
    }
}

#[zbus::interface(name = "com.victronenergy.BusItem")]
impl Leaf {
    fn get_value(&self) -> OwnedValue {
        to_value(&self.current())
    }
    fn get_text(&self) -> String {
        to_text(&self.current())
    }
}

/// Publish the service from a thread of its own, registering again after any failure
/// (the name held by another instance, a lost connection). The control loop never
/// depends on it.
pub fn spawn(shared: Shared) {
    let spawned = std::thread::Builder::new().name("status-service".into()).spawn(move || {
        let mut last_error = String::new();
        loop {
            if let Err(e) = serve(&shared) {
                let e = e.to_string();
                if e != last_error {
                    eprintln!("status service: {e}; retrying every {} s", RETRY.as_secs());
                    last_error = e;
                }
            }
            std::thread::sleep(RETRY);
        }
    });
    if let Err(e) = spawned {
        eprintln!("status service: cannot start ({e}); alarms are not published");
    }
}

/// Register on a fresh connection and publish changes until something fails.
fn serve(shared: &Shared) -> zbus::Result<()> {
    let mut builder = zbus::blocking::connection::Builder::system()?
        .method_timeout(crate::dbus::METHOD_TIMEOUT)
        .serve_at("/", Root { shared: shared.clone() })?;
    for (path, _) in items(&lock(shared).clone(), Instant::now()) {
        builder = builder.serve_at(path, Leaf { path, shared: shared.clone() })?;
    }
    let conn = builder.name(SERVICE)?.build()?;
    eprintln!("status service: {SERVICE} registered");

    let mut sent: Vec<(&'static str, Val)> = Vec::new();
    loop {
        std::thread::sleep(REFRESH);
        let s = lock(shared).clone();
        let now = items(&s, Instant::now());
        let changed: Vec<(&'static str, Val)> =
            now.iter().filter(|item| !sent.contains(item)).cloned().collect();
        if !changed.is_empty() {
            conn.emit_signal(None::<&str>, "/", IFACE, "ItemsChanged", &item_map(&changed))?;
        }
        sent = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gridsource::GridSource;

    const S: fn(u64) -> Duration = Duration::from_secs;

    fn get(list: &[(&'static str, Val)], path: &str) -> Val {
        list.iter().find(|(p, _)| *p == path).unwrap().1.clone()
    }

    /// Owning in takeover, everything fresh at `t`.
    fn healthy(t0: Instant, t: Instant) -> Status {
        let mut s = Status::new(t0, 2, GridSource::Meter, false);
        s.owner_us = true;
        s.loop_alive = Some(t);
        s.good_tick(t, 100.0, true);
        s
    }

    #[test]
    fn a_healthy_controller_raises_nothing() {
        let t0 = Instant::now();
        let i = items(&healthy(t0, t0), t0 + S(1));
        assert_eq!(get(&i, "/Alarms/Alarm"), Val::Int(0));
        assert_eq!(get(&i, "/Status/Pacing"), Val::Int(2));
    }

    #[test]
    fn a_stuck_loop_alarms_from_ages_alone() {
        let t0 = Instant::now();
        let i = items(&healthy(t0, t0), t0 + S(11));
        assert_eq!(get(&i, "/Alarms/LoopStalled"), Val::Int(2));
        assert_eq!(get(&i, "/Alarms/InputsStale"), Val::Int(2));
        assert_eq!(get(&i, "/Alarms/Alarm"), Val::Int(2));
        assert_eq!(get(&i, "/Status/LoopAge"), Val::Num(11.0));
    }

    #[test]
    fn failing_reads_warn_then_alarm_while_the_loop_runs() {
        let t0 = Instant::now();
        let mut s = healthy(t0, t0);
        s.loop_alive = Some(t0 + S(4));
        assert_eq!(get(&items(&s, t0 + S(4)), "/Alarms/InputsStale"), Val::Int(1));
        s.loop_alive = Some(t0 + S(11));
        let i = items(&s, t0 + S(11));
        assert_eq!(get(&i, "/Alarms/InputsStale"), Val::Int(2));
        assert_eq!(get(&i, "/Alarms/LoopStalled"), Val::Int(0));
    }

    #[test]
    fn a_frozen_grid_reading_warns_then_alarms() {
        let t0 = Instant::now();
        let mut s = healthy(t0, t0);
        for k in 1..=31 {
            s.loop_alive = Some(t0 + S(k));
            s.good_tick(t0 + S(k), 100.0, true); // same value every tick
        }
        assert_eq!(get(&items(&s, t0 + S(11)), "/Alarms/GridFrozen"), Val::Int(1));
        assert_eq!(get(&items(&s, t0 + S(31)), "/Alarms/GridFrozen"), Val::Int(2));
        s.good_tick(t0 + S(31), 101.0, true);
        assert_eq!(get(&items(&s, t0 + S(31)), "/Alarms/GridFrozen"), Val::Int(0));
    }

    #[test]
    fn owning_without_writing_warns_then_alarms() {
        let t0 = Instant::now();
        let mut s = healthy(t0, t0);
        for k in 1..=61 {
            s.loop_alive = Some(t0 + S(k));
            s.good_tick(t0 + S(k), k as f64, false); // the guard holds writes back
        }
        assert_eq!(get(&items(&s, t0 + S(11)), "/Alarms/NotWriting"), Val::Int(1));
        assert_eq!(get(&items(&s, t0 + S(61)), "/Alarms/NotWriting"), Val::Int(2));
        s.owner_us = false; // stock is writing: nothing for us to write
        assert_eq!(get(&items(&s, t0 + S(61)), "/Alarms/NotWriting"), Val::Int(0));
    }

    #[test]
    fn losing_control_in_takeover_is_a_warning() {
        let t0 = Instant::now();
        let mut s = healthy(t0, t0);
        s.owner_us = false;
        assert_eq!(get(&items(&s, t0), "/Alarms/NotInControl"), Val::Int(1));
        s.owner_us = true;
        s.ceded_external = true;
        assert_eq!(get(&items(&s, t0), "/Alarms/NotInControl"), Val::Int(1));
        s.stage = 0; // shadow: stock owning is the design
        s.owner_us = false;
        s.ceded_external = false;
        assert_eq!(get(&items(&s, t0), "/Alarms/NotInControl"), Val::Int(0));
        assert_eq!(get(&items(&s, t0), "/Status/WriteAge"), Val::None);
    }

    #[test]
    fn nothing_seen_yet_counts_from_start() {
        let t0 = Instant::now();
        let s = Status::new(t0, 2, GridSource::Meter, true);
        let i = items(&s, t0 + S(12));
        assert_eq!(get(&i, "/Alarms/LoopStalled"), Val::Int(2));
        assert_eq!(get(&i, "/Alarms/GridFrozen"), Val::Int(1));
        assert_eq!(get(&i, "/Status/Pacing"), Val::Int(1));
    }

    #[test]
    fn items_have_the_victron_bus_shape() {
        let t0 = Instant::now();
        let m = item_map(&items(&healthy(t0, t0), t0));
        assert_eq!(
            zbus::zvariant::Signature::from(<ItemMap as zbus::zvariant::Type>::SIGNATURE).to_string(),
            "a{sa{sv}}"
        );
        assert!(m.contains_key("/DeviceInstance"));
        assert!(m.values().all(|e| e.len() == 2 && e.contains_key("Value") && e.contains_key("Text")));
        assert_eq!(m["/Status/Owner"]["Value"], OwnedValue::from(1i32));
        assert_eq!(
            m["/Status/WriteAge"]["Value"].value_signature().to_string(),
            "d",
            "ages are doubles"
        );
    }
}
