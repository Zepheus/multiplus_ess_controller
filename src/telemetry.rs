//! Bounded tmpfs telemetry writer + the eMMC-protection path guard.
//! Pure machinery — no control logic lives here.

use std::io::Write;

/// Size-capped, self-rotating CSV writer for RAM-backed (tmpfs) telemetry.
/// Total on-disk footprint is bounded to 2*cap, so it can never grow into an OOM
/// even if left running for months. Refuses to touch flash-backed paths.
pub struct Telemetry {
    path: String,
    cap: u64,
    written: u64,
    file: std::fs::File,
}

/// Column header, written at the top of every generation (including after rotation)
/// so each file is independently parseable.
pub const HEADER: &str =
    "t,grid,reported,target,soc,state,command,owner,actual,fc,maxdis,acout,reason,minsoc,dcbatt";

impl Telemetry {
    /// cap_bytes is per-file; with one rotation the total is at most 2*cap_bytes.
    pub fn open(path: &str, cap_bytes: u64) -> Result<Self, String> {
        if is_flash_path(path) {
            return Err(format!(
                "refusing telemetry to '{path}': resolves onto eMMC flash. \
                 Use a tmpfs path like /run/… or /tmp/… instead."
            ));
        }
        if let Some(parent) = std::path::Path::new(path).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)
            .map_err(|e| format!("cannot open telemetry '{path}': {e}"))?;
        let mut t = Telemetry { path: path.to_string(), cap: cap_bytes, written: 0, file };
        let _ = t.write_line(HEADER);
        Ok(t)
    }

    pub fn write_line(&mut self, line: &str) -> std::io::Result<()> {
        let bytes = line.len() as u64 + 1;
        if self.written + bytes > self.cap {
            self.rotate();
        }
        writeln!(self.file, "{line}")?;
        self.written += bytes;
        Ok(())
    }

    fn rotate(&mut self) {
        // Keep exactly one previous generation: path -> path.1 (overwrite), reopen fresh
        // and re-emit the header so the new generation parses on its own.
        let _ = std::fs::rename(&self.path, format!("{}.1", self.path));
        if let Ok(f) = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&self.path)
        {
            self.file = f;
            self.written = 0;
            let bytes = HEADER.len() as u64 + 1;
            if writeln!(self.file, "{HEADER}").is_ok() {
                self.written += bytes;
            }
        }
    }
}

/// True if the path lives on the eMMC (directly under /data, or via the
/// /var/log -> /data/log symlink). Checks the canonicalised parent directory.
pub fn is_flash_path(path: &str) -> bool {
    let p = std::path::Path::new(path);
    let probe = p.parent().filter(|x| !x.as_os_str().is_empty()).unwrap_or(p);
    let resolved = std::fs::canonicalize(probe)
        .map(|r| r.to_string_lossy().into_owned())
        .unwrap_or_else(|_| path.to_string());
    resolved == "/data" || resolved.starts_with("/data/")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> String {
        let d = std::env::temp_dir().join(format!("venus-ess-telemetry-{}-{name}", std::process::id()));
        let _ = std::fs::create_dir_all(&d);
        d.join("t.csv").to_string_lossy().into_owned()
    }

    #[test]
    fn every_generation_starts_with_the_header() {
        let path = tmp("rot");
        // Cap small enough that a handful of rows forces a rotation.
        let mut t = Telemetry::open(&path, (HEADER.len() as u64 + 1) * 3).unwrap();
        for i in 0..10 {
            t.write_line(&format!("{i},0,0,5,50,252,0,US,,0,,0,")).unwrap();
        }
        let cur = std::fs::read_to_string(&path).unwrap();
        let prev = std::fs::read_to_string(format!("{path}.1")).unwrap();
        assert!(prev.starts_with(HEADER), "rotated-out generation lost its header");
        assert!(cur.starts_with(HEADER), "fresh generation after rotation has no header:\n{cur}");
        // Rows are never lost or duplicated across the rotation boundary.
        let rows: Vec<&str> = prev.lines().chain(cur.lines()).filter(|l| *l != HEADER).collect();
        assert_eq!(rows.len(), 10);
        assert!(rows[0].starts_with("0,") && rows[9].starts_with("9,"));
        // Header lines count toward the cap: no generation exceeds it.
        assert!(prev.len() as u64 <= (HEADER.len() as u64 + 1) * 3);
    }
}
