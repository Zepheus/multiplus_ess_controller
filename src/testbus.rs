//! Shared in-memory `Bus` for off-host unit/regression tests. No zbus, no device.
//! Every subsystem module's tests use this so behaviour can be verified on the host
//! and real captured snapshots can be replayed.

use crate::dbus::{Bus, BusFault};
use std::collections::HashMap;

pub struct MockBus {
    pub f: HashMap<(String, String), f64>,
    pub s: HashMap<(String, String), String>,
    pub services: Vec<String>,
    /// Items that exist but are published invalid (GetValue answers an empty array).
    pub invalid: Vec<(String, String)>,
    /// Transport fault handed out once by `take_fault`.
    pub fault: std::cell::RefCell<Option<BusFault>>,
    /// Every write that was sent, in order.
    pub writes: std::cell::RefCell<Vec<(String, String, f64)>>,
    /// Writes to these paths time out (latching a fault).
    pub hung_writes: Vec<String>,
}

impl MockBus {
    pub fn new() -> Self {
        MockBus {
            f: HashMap::new(),
            s: HashMap::new(),
            services: Vec::new(),
            invalid: Vec::new(),
            fault: std::cell::RefCell::new(None),
            writes: std::cell::RefCell::new(Vec::new()),
            hung_writes: Vec::new(),
        }
    }
    pub fn set(&mut self, svc: &str, path: &str, v: f64) {
        self.f.insert((svc.into(), path.into()), v);
    }
    pub fn set_str(&mut self, svc: &str, path: &str, v: &str) {
        self.s.insert((svc.into(), path.into()), v.into());
    }
    pub fn clear(&mut self, svc: &str, path: &str) {
        self.f.remove(&(svc.into(), path.into()));
    }
    pub fn latch_on_write(&mut self, path: &str) {
        self.hung_writes.push(path.into());
    }
    /// Like `SystemBus`: while a fault is latched every call fails at once; a write to a
    /// hung path latches one.
    fn write(&self, svc: &str, path: &str, v: f64) -> Result<(), String> {
        if self.fault.borrow().is_some() {
            return Err("fault latched".into());
        }
        if self.hung_writes.iter().any(|p| p == path) {
            *self.fault.borrow_mut() = Some(BusFault { what: format!("SetValue {path}: timed out"), connection_lost: false });
            return Err("timed out".into());
        }
        self.writes.borrow_mut().push((svc.into(), path.into(), v));
        Ok(())
    }
    pub fn add_service(&mut self, name: &str) {
        self.services.push(name.into());
    }
}

impl Bus for MockBus {
    fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
        if self.fault.borrow().is_some() {
            return None;
        }
        self.f.get(&(svc.into(), path.into())).copied()
    }
    fn get_str(&self, svc: &str, path: &str) -> Option<String> {
        self.s.get(&(svc.into(), path.into())).cloned()
    }
    fn exists(&self, svc: &str, path: &str) -> bool {
        let key = (svc.to_string(), path.to_string());
        self.f.contains_key(&key) || self.s.contains_key(&key) || self.invalid.contains(&key)
    }
    fn take_fault(&self) -> Option<BusFault> {
        self.fault.borrow_mut().take()
    }
    fn list_services(&self, prefix: &str) -> Vec<String> {
        self.services.iter().filter(|s| s.starts_with(prefix)).cloned().collect()
    }
    fn set_i32(&self, svc: &str, path: &str, v: i32) -> Result<(), String> {
        self.write(svc, path, f64::from(v))
    }
    fn set_f64(&self, svc: &str, path: &str, v: f64) -> Result<(), String> {
        self.write(svc, path, v)
    }
}
