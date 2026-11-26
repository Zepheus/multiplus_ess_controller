//! Tick pacing: tick when the grid source publishes, with a timer as the failsafe.
//!
//! The grid source (the meter service, or systemcalc: see `gridsource`) publishes once
//! per second (`ItemsChanged` on `/`). Ticking on that signal reads each update as soon as
//! it exists, instead of drifting against it and reading it up to a second late.
//!
//! A listener thread on its own bus connection forwards the arrival time of each publish
//! over a small bounded channel. The loop never depends on it: if no usable event
//! arrives by the deadline (`FALLBACK_FACTOR` x interval after the previous tick), the
//! loop ticks anyway. A dead listener, a restarting meter service or a missed publish
//! costs latency, never a stall.
//!
//! `Mode::Timer` keeps the plain cadence: sleep one interval after each tick's work.

use crate::dbus::{IFACE, METHOD_TIMEOUT};
use crate::gridsource::GridSource;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::time::{Duration, Instant};

/// Deadline after the previous tick, as a multiple of the interval.
pub const FALLBACK_FACTOR: f64 = 1.5;
/// Events closer than this multiple of the interval to the previous tick are coalesced,
/// so a source publishing faster than the interval cannot make the loop tick faster (the
/// per-tick gains assume about one interval per tick).
pub const MIN_SPACING_FACTOR: f64 = 0.9;
/// Longest single blocking wait, so a shutdown request is seen promptly.
const SLICE: Duration = Duration::from_millis(100);
/// Pause before subscribing again after the listener's connection fails.
const RESUBSCRIBE_DELAY: Duration = Duration::from_secs(5);
/// Arrival times buffered between ticks; the loop needs only the latest, so a full buffer
/// drops new ones instead of growing.
const CHANNEL_CAP: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Events,
    Timer,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wake {
    Event,
    Timer,
    Shutdown,
}

/// Whether one received event should start a tick. `last_tick`: when the previous tick
/// started. `last_event`: the stamp of the event that started it, if one did; spacing is
/// then measured between two stamps from the same thread, so listener delay cannot
/// shift it.
fn accept_event(event_at: Instant, last_tick: Instant, last_event: Option<Instant>, min_spacing: Duration) -> bool {
    // Published before the previous tick's reads began: already consumed.
    if event_at < last_tick {
        return false;
    }
    let since = last_event.unwrap_or(last_tick);
    event_at.saturating_duration_since(since) >= min_spacing
}

pub struct Pacer {
    interval: Duration,
    min_spacing: Duration,
    fallback: Duration,
    rx: Option<Receiver<Instant>>,
    /// Stamp of the event that started the previous tick (`None` after a timer wake).
    last_event: Option<Instant>,
    /// Kind of the previous wake, for logging only when the pacing source changes.
    last_kind: Option<Wake>,
}

impl Pacer {
    pub fn new(
        interval_s: f64,
        mode: Mode,
        source: GridSource,
    ) -> Self {
        let rx = match mode {
            Mode::Timer => None,
            Mode::Events => Some(spawn_listener(source)),
        };
        Self::with_receiver(interval_s, rx)
    }

    fn with_receiver(interval_s: f64, rx: Option<Receiver<Instant>>) -> Self {
        Pacer {
            interval: Duration::from_secs_f64(interval_s),
            min_spacing: Duration::from_secs_f64(interval_s * MIN_SPACING_FACTOR),
            fallback: Duration::from_secs_f64(interval_s * FALLBACK_FACTOR),
            rx,
            last_event: None,
            last_kind: None,
        }
    }

    /// Block until the next tick is due. `last_tick` is when the previous tick began.
    pub fn wait(&mut self, last_tick: Instant, shutdown: &AtomicBool) -> Wake {
        let had_events = self.rx.is_some();
        let wake = self.wait_inner(last_tick, shutdown);
        if wake != Wake::Event {
            self.last_event = None;
        }
        if had_events && wake != Wake::Shutdown && self.last_kind != Some(wake) {
            match wake {
                Wake::Event => eprintln!("pacing: ticking on grid-source publishes"),
                Wake::Timer => eprintln!(
                    "pacing: no grid-source publish within {:.1} s of the previous tick, ticking on the failsafe timer",
                    self.fallback.as_secs_f64()
                ),
                Wake::Shutdown => {}
            }
            self.last_kind = Some(wake);
        }
        wake
    }

    fn wait_inner(&mut self, last_tick: Instant, shutdown: &AtomicBool) -> Wake {
        let Some(rx) = &self.rx else {
            // Timer cadence: one full interval after this tick's work.
            return sleep_until(Instant::now() + self.interval, shutdown);
        };
        let deadline = last_tick + self.fallback;
        loop {
            if shutdown.load(Ordering::SeqCst) {
                return Wake::Shutdown;
            }
            let now = Instant::now();
            if now >= deadline {
                return Wake::Timer;
            }
            match rx.recv_timeout((deadline - now).min(SLICE)) {
                Ok(at) if accept_event(at, last_tick, self.last_event, self.min_spacing) => {
                    self.last_event = Some(at);
                    return Wake::Event;
                }
                Ok(_) | Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => {
                    eprintln!("pacing: grid listener gone, timer pacing from now on");
                    self.rx = None;
                    return sleep_until(deadline.min(Instant::now() + self.interval), shutdown);
                }
            }
        }
    }
}

fn sleep_until(until: Instant, shutdown: &AtomicBool) -> Wake {
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return Wake::Shutdown;
        }
        let now = Instant::now();
        if now >= until {
            return Wake::Timer;
        }
        std::thread::sleep((until - now).min(SLICE));
    }
}

fn spawn_listener(source: GridSource) -> Receiver<Instant> {
    let (tx, rx) = mpsc::sync_channel(CHANNEL_CAP);
    let spawned = std::thread::Builder::new()
        .name("grid-listener".into())
        .spawn(move || listen(tx, source));
    if let Err(e) = spawned {
        eprintln!("pacing: cannot start the meter listener ({e}); timer pacing");
    }
    rx
}

/// Forward the arrival time of each grid-source publish until the receiver is dropped.
fn listen(tx: SyncSender<Instant>, source: GridSource) {
    loop {
        match listen_once(&tx, source) {
            Ok(()) => return, // receiver dropped: the loop has ended
            Err(e) => eprintln!("pacing: grid listener: {e}; retrying"),
        }
        std::thread::sleep(RESUBSCRIBE_DELAY);
    }
}

/// One subscription to the source's `ItemsChanged` on `/`. The service's well-known name
/// is looked up before subscribing, so no signal can queue up behind that call. The match
/// rule names it as sender, so the bus daemon delivers only that service's signals and
/// follows the name across a restart of the service. If the name itself goes away (a meter
/// moved to another port), publishes stop and the loop runs on its failsafe timer.
fn listen_once(tx: &SyncSender<Instant>, source: GridSource) -> Result<(), String> {
    let conn = zbus::blocking::connection::Builder::system()
        .map(|b| b.method_timeout(METHOD_TIMEOUT))
        .and_then(|b| b.build())
        .map_err(|e| e.to_string())?;
    let names: Vec<String> = conn
        .call_method(
            Some("org.freedesktop.DBus"),
            "/org/freedesktop/DBus",
            Some("org.freedesktop.DBus"),
            "ListNames",
            &(),
        )
        .and_then(|reply| reply.body().deserialize())
        .map_err(|e| e.to_string())?;
    let name = source.publisher(&names).ok_or("grid source not on the bus yet")?;
    let rule = zbus::MatchRule::builder()
        .msg_type(zbus::message::Type::Signal)
        .sender(name.as_str())
        .and_then(|b| b.interface(IFACE))
        .and_then(|b| b.member("ItemsChanged"))
        .and_then(|b| b.path("/"))
        .map_err(|e| e.to_string())?
        .build();
    let signals = zbus::blocking::MessageIterator::for_match_rule(rule, &conn, Some(16))
        .map_err(|e| e.to_string())?;
    eprintln!("pacing: following publishes of {name}");
    for msg in signals {
        msg.map_err(|e| e.to_string())?;
        match tx.try_send(Instant::now()) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => return Ok(()),
        }
    }
    Err("signal stream ended".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: fn(u64) -> Duration = Duration::from_millis;

    #[test]
    fn events_before_the_previous_tick_are_already_consumed() {
        let t0 = Instant::now();
        let last = t0 + MS(1000);
        assert!(!accept_event(t0 + MS(990), last, None, MS(500)));
    }

    #[test]
    fn events_too_close_to_the_previous_tick_are_coalesced() {
        let last = Instant::now();
        assert!(!accept_event(last + MS(200), last, None, MS(500)));
        assert!(accept_event(last + MS(500), last, None, MS(500)));
        assert!(accept_event(last + MS(1000), last, None, MS(500)));
    }

    #[test]
    fn spacing_after_an_event_tick_counts_from_that_event() {
        // The previous tick started 150 ms after its event was stamped (listener delay);
        // the next publish, 1 s after the previous one, is still accepted.
        let ev = Instant::now();
        let tick = ev + MS(150);
        assert!(accept_event(ev + MS(1000), tick, Some(ev), MS(900)));
        assert!(!accept_event(ev + MS(1000), tick, None, MS(900)), "measured from the tick it would be dropped");
    }

    #[test]
    fn an_event_wakes_the_loop_before_the_deadline() {
        let (tx, rx) = mpsc::sync_channel(CHANNEL_CAP);
        let mut p = Pacer::with_receiver(0.2, Some(rx));
        let last = Instant::now();
        let keep = std::thread::spawn(move || {
            std::thread::sleep(MS(80)); // too soon after the previous tick: coalesced
            let _ = tx.send(Instant::now());
            std::thread::sleep(MS(110));
            let _ = tx.send(Instant::now());
            std::thread::sleep(MS(300)); // keep the sender alive past the wait
        });
        assert_eq!(p.wait(last, &AtomicBool::new(false)), Wake::Event);
        let waited = last.elapsed();
        assert!(waited >= MS(190) && waited < MS(290), "woke after {waited:?}");
        keep.join().unwrap();
    }

    #[test]
    fn a_silent_meter_falls_back_to_the_failsafe_timer() {
        let (tx, rx) = mpsc::sync_channel::<Instant>(CHANNEL_CAP);
        let mut p = Pacer::with_receiver(0.1, Some(rx));
        let last = Instant::now();
        assert_eq!(p.wait(last, &AtomicBool::new(false)), Wake::Timer);
        let waited = last.elapsed();
        assert!(waited >= MS(150) && waited < MS(250), "fell back after {waited:?}");
        drop(tx);
    }

    #[test]
    fn a_dead_listener_degrades_to_timer_pacing() {
        let (tx, rx) = mpsc::sync_channel::<Instant>(CHANNEL_CAP);
        drop(tx);
        let mut p = Pacer::with_receiver(0.1, Some(rx));
        let last = Instant::now();
        assert_eq!(p.wait(last, &AtomicBool::new(false)), Wake::Timer);
        assert!(last.elapsed() < MS(150));
        assert!(p.rx.is_none());
    }

    #[test]
    fn timer_mode_sleeps_one_interval_after_the_tick_work() {
        let mut p = Pacer::with_receiver(0.1, None);
        let last = Instant::now();
        std::thread::sleep(MS(30)); // the tick's work
        assert_eq!(p.wait(last, &AtomicBool::new(false)), Wake::Timer);
        let waited = last.elapsed();
        assert!(waited >= MS(130) && waited < MS(180), "ticked after {waited:?}");
    }

    #[test]
    fn a_full_channel_drops_new_events_instead_of_growing() {
        let (tx, rx) = mpsc::sync_channel(CHANNEL_CAP);
        for _ in 0..CHANNEL_CAP {
            tx.try_send(Instant::now()).unwrap();
        }
        assert!(matches!(tx.try_send(Instant::now()), Err(TrySendError::Full(_))));
        drop(rx);
    }

    #[test]
    fn shutdown_interrupts_the_wait() {
        let (_tx, rx) = mpsc::sync_channel::<Instant>(CHANNEL_CAP);
        let mut p = Pacer::with_receiver(10.0, Some(rx));
        assert_eq!(p.wait(Instant::now(), &AtomicBool::new(true)), Wake::Shutdown);
        let mut t = Pacer::with_receiver(10.0, None);
        assert_eq!(t.wait(Instant::now(), &AtomicBool::new(true)), Wake::Shutdown);
    }
}
