//! Shared in-memory `Bus` for off-host unit/regression tests. No zbus, no device.
//! Every subsystem module's tests use this so behaviour can be verified on the host
//! and real captured snapshots can be replayed.

use crate::dbus::Bus;
use std::collections::HashMap;

pub struct MockBus {
    pub f: HashMap<(String, String), f64>,
    pub s: HashMap<(String, String), String>,
    pub services: Vec<String>,
}

impl MockBus {
    pub fn new() -> Self {
        MockBus { f: HashMap::new(), s: HashMap::new(), services: Vec::new() }
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
    pub fn add_service(&mut self, name: &str) {
        self.services.push(name.into());
    }
}

impl Bus for MockBus {
    fn get_f64(&self, svc: &str, path: &str) -> Option<f64> {
        self.f.get(&(svc.into(), path.into())).copied()
    }
    fn get_str(&self, svc: &str, path: &str) -> Option<String> {
        self.s.get(&(svc.into(), path.into())).cloned()
    }
    fn list_services(&self, prefix: &str) -> Vec<String> {
        self.services.iter().filter(|s| s.starts_with(prefix)).cloned().collect()
    }
    fn set_i32(&self, _: &str, _: &str, _: i32) -> Result<(), String> {
        Ok(())
    }
    fn set_f64(&self, _: &str, _: &str, _: f64) -> Result<(), String> {
        Ok(())
    }
}
