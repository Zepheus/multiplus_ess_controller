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
        let _ = t.write_line(
            "t,grid,reported,target,soc,state,command,owner,actual,fc,maxdis,acout,reason",
        );
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
        // Keep exactly one previous generation: path -> path.1 (overwrite), reopen fresh.
        let _ = std::fs::rename(&self.path, format!("{}.1", self.path));
        if let Ok(f) = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&self.path)
        {
            self.file = f;
            self.written = 0;
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
