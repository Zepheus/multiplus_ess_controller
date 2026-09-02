//! Bounded tmpfs telemetry writer + the eMMC-protection path guard.
//! Pure machinery — no control logic lives here.

use std::io::Write;

/// Size-capped, self-rotating CSV writer for RAM-backed (tmpfs) telemetry.
/// Total on-disk footprint is bounded to 2*cap, so it can never grow into an OOM
/// even if left running for months. Refuses to touch flash-backed paths for the
/// per-tick stream.
///
/// Durable archive (optional): the ONE sanctioned flash writer. Whole generations
/// are copied to `archive_dir` as bulk files — on rotation (every `cap` bytes,
/// ~2 days at 1 Hz), on restart (the previous run's file would otherwise be
/// truncated away), and as a periodic snapshot of the live file (so a reboot
/// loses at most `snapshot_every`). Bulk writes of a few MB per day are
/// negligible eMMC wear; per-tick writes are what the flash rule forbids.
pub struct Telemetry {
    path: String,
    cap: u64,
    written: u64,
    file: std::fs::File,
    archive: Option<Archive>,
}

pub struct Archive {
    pub dir: String,
    pub keep: usize,
    pub snapshot_every: std::time::Duration,
    last_snapshot: std::time::Instant,
    seq: u32,
    warned: bool,
}

impl Archive {
    pub fn new(dir: &str, keep: usize, snapshot_every: std::time::Duration) -> Self {
        if let Err(e) = std::fs::create_dir_all(dir) {
            eprintln!("telemetry archive: cannot create '{dir}': {e}");
        }
        Archive {
            dir: dir.to_string(),
            keep,
            snapshot_every,
            last_snapshot: std::time::Instant::now(),
            seq: 0,
            warned: false,
        }
    }

    /// Report an archive failure once per run (the archive is the only durability
    /// mechanism; a silent failure would be discovered after the reboot).
    fn fail(&mut self, what: &str, e: &dyn std::fmt::Display) {
        if !self.warned {
            eprintln!("telemetry archive FAILED ({what}) in '{}': {e} — further failures suppressed", self.dir);
            self.warned = true;
        }
    }
}

/// Marker beside the rotated generation: it was already archived at rotation, so a
/// restart must not archive it again (an exact duplicate per restart).
fn archived_marker(path: &str) -> String {
    format!("{path}.1.archived")
}

/// Copy via a temp name + rename so a partial copy (ENOSPC, reboot mid-copy) never
/// looks like a complete archive; the temp is removed on failure.
fn copy_atomic(src: &str, dst: &str) -> std::io::Result<()> {
    let tmp = format!("{dst}.part");
    let r = std::fs::copy(src, &tmp).and_then(|_| std::fs::rename(&tmp, dst));
    if r.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    r
}

fn unix_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

impl Telemetry {
    /// cap_bytes is per-file; with one rotation the total is at most 2*cap_bytes.
    pub fn open(path: &str, cap_bytes: u64, archive: Option<Archive>) -> Result<Self, String> {
        if is_flash_path(path) {
            return Err(format!(
                "refusing telemetry to '{path}': resolves onto eMMC flash. \
                 Use a tmpfs path like /run/… or /tmp/… instead."
            ));
        }
        if let Some(parent) = std::path::Path::new(path).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        // A previous run's live file (and its rotated generation, unless rotation
        // already archived it) would be lost by the truncate below: archive first.
        let mut archive = archive;
        if let Some(a) = archive.as_mut() {
            let prev = format!("{path}.1");
            if std::path::Path::new(&archived_marker(path)).exists() {
                let _ = std::fs::remove_file(archived_marker(path));
            } else {
                Self::archive_file(a, cap_bytes, &prev, "restart-prev");
            }
            Self::archive_file(a, cap_bytes, path, "restart");
        }
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)
            .map_err(|e| format!("cannot open telemetry '{path}': {e}"))?;
        let mut t = Telemetry { path: path.to_string(), cap: cap_bytes, written: 0, file, archive };
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
        self.maybe_snapshot();
        Ok(())
    }

    fn rotate(&mut self) {
        // The outgoing generation is the archive's unit of record: copy it out
        // before it becomes `.1` (and the old `.1` is overwritten).
        let _ = self.file.flush();
        if let Some(a) = self.archive.as_mut() {
            if Self::archive_file(a, self.cap, &self.path, "rotate") {
                let _ = std::fs::write(archived_marker(&self.path), b"");
            }
        }
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

    /// Periodic bulk snapshot of the live file so an unclean stop (reboot, power
    /// loss) loses at most `snapshot_every`. One file per run, overwritten.
    fn maybe_snapshot(&mut self) {
        let Some(a) = self.archive.as_mut() else { return };
        if a.last_snapshot.elapsed() < a.snapshot_every {
            return;
        }
        a.last_snapshot = std::time::Instant::now();
        let _ = self.file.flush();
        if self.written < 128 {
            return;
        }
        let dst = format!("{}/telemetry-live-snapshot.csv", a.dir);
        if let Err(e) = copy_atomic(&self.path, &dst) {
            a.fail("snapshot", &e);
        }
    }

    /// Archive the live file now (clean shutdown: after the Hub4Mode restore, with
    /// no deadline, so a graceful stop loses nothing). Removes the stale snapshot.
    pub fn archive_now(&mut self, why: &str) {
        let _ = self.file.flush();
        if let Some(a) = self.archive.as_mut() {
            if Self::archive_file(a, self.cap, &self.path, why) {
                let _ = std::fs::remove_file(format!("{}/telemetry-live-snapshot.csv", a.dir));
            }
        }
    }

    /// Copy `src` into the archive as `telemetry-<unix>-<seq>-<why>.csv` (unique even
    /// within one second; skipping empty or header-only files), atomically via a temp
    /// name and rename so an interrupted copy never masquerades as a complete archive,
    /// then prune. Returns true on success.
    fn archive_file(a: &mut Archive, cap: u64, src: &str, why: &str) -> bool {
        let Ok(meta) = std::fs::metadata(src) else { return false };
        if meta.len() < 128 {
            return false; // empty or header-only: nothing worth a flash write
        }
        a.seq += 1;
        let mut dst = format!("{}/telemetry-{}-{:03}-{}.csv", a.dir, unix_now(), a.seq, why);
        while std::path::Path::new(&dst).exists() {
            a.seq += 1;
            dst = format!("{}/telemetry-{}-{:03}-{}.csv", a.dir, unix_now(), a.seq, why);
        }
        match copy_atomic(src, &dst) {
            Ok(()) => {
                Self::prune(a, cap);
                true
            }
            Err(e) => {
                a.fail(why, &e);
                false
            }
        }
    }

    /// Size-aware prune: real generations (>= cap/4) and small restart fragments are
    /// pruned as SEPARATE populations, each to the newest `keep`, so a crash loop of
    /// fragments can never evict real history. The live snapshot is never pruned.
    fn prune(a: &mut Archive, cap: u64) {
        let Ok(rd) = std::fs::read_dir(&a.dir) else { return };
        let mut gens: Vec<(String, u64)> = vec![];
        let mut frags: Vec<(String, u64)> = vec![];
        for e in rd.flatten() {
            let n = e.file_name().to_string_lossy().into_owned();
            if !(n.starts_with("telemetry-") && n.ends_with(".csv")) || n.contains("live-snapshot") {
                continue;
            }
            let len = e.metadata().map(|m| m.len()).unwrap_or(0);
            if len >= cap / 4 { gens.push((n, len)) } else { frags.push((n, len)) }
        }
        for pop in [&mut gens, &mut frags] {
            pop.sort(); // unix-seq prefix => chronological
            while pop.len() > a.keep {
                let (victim, _) = pop.remove(0);
                if let Err(e) = std::fs::remove_file(format!("{}/{}", a.dir, victim)) {
                    a.fail("prune", &e);
                }
            }
        }
    }

    /// Number of archived generations on disk (tests / diagnostics).
    #[cfg(test)]
    pub fn archived(a: &Archive) -> usize {
        std::fs::read_dir(&a.dir)
            .map(|rd| {
                rd.flatten()
                    .filter(|e| {
                        let n = e.file_name().to_string_lossy().into_owned();
                        n.starts_with("telemetry-") && n.ends_with(".csv") && !n.contains("live-snapshot")
                    })
                    .count()
            })
            .unwrap_or(0)
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
        let d = std::env::temp_dir().join(format!("venus-tel-{}-{}", name, std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d.to_string_lossy().into_owned()
    }

    #[test]
    fn rotation_archives_the_outgoing_generation() {
        let d = tmp("rotate");
        let live = format!("{d}/live.csv");
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let mut t = Telemetry::open(&live, 400, Some(a)).unwrap();
        for i in 0..40 {
            t.write_line(&format!("{i},1,2,3,4,5,6,US,,0,,7,")).unwrap();
        }
        let a = t.archive.as_ref().unwrap();
        assert!(Telemetry::archived(a) >= 1, "each rotation must leave a bulk copy in the archive");
        assert!(std::path::Path::new(&format!("{live}.1")).exists(), "tmpfs generation still rotates");
    }

    #[test]
    fn restart_archives_the_previous_run_instead_of_truncating_it() {
        let d = tmp("restart");
        let live = format!("{d}/live.csv");
        std::fs::write(&live, "t,grid\n".repeat(64)).unwrap(); // a previous run's data
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let t = Telemetry::open(&live, 1 << 20, Some(a)).unwrap();
        assert_eq!(Telemetry::archived(t.archive.as_ref().unwrap()), 1);
    }

    #[test]
    fn archive_prunes_to_keep_newest() {
        let d = tmp("prune");
        let live = format!("{d}/live.csv");
        let a = Archive::new(&format!("{d}/archive"), 2, std::time::Duration::from_secs(3600));
        let mut t = Telemetry::open(&live, 300, Some(a)).unwrap();
        for i in 0..200 {
            t.write_line(&format!("{i},1,2,3,4,5,6,US,,0,,7,")).unwrap();
        }
        assert!(Telemetry::archived(t.archive.as_ref().unwrap()) <= 2, "must prune to keep");
    }

    #[test]
    fn empty_or_header_only_files_are_not_archived() {
        let d = tmp("empty");
        let live = format!("{d}/live.csv");
        std::fs::write(&live, "t,grid\n").unwrap(); // header only
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let t = Telemetry::open(&live, 1 << 20, Some(a)).unwrap();
        assert_eq!(Telemetry::archived(t.archive.as_ref().unwrap()), 0);
    }

    #[test]
    fn restart_with_both_generations_keeps_both_with_right_contents() {
        // Prosecution #1: `.1` and live archived in the same second collided on the
        // timestamp filename and the live copy overwrote the older generation.
        let d = tmp("both");
        let live = format!("{d}/live.csv");
        std::fs::write(format!("{live}.1"), "OLD\n".repeat(64)).unwrap();
        std::fs::write(&live, "NEW\n".repeat(64)).unwrap();
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let t = Telemetry::open(&live, 1 << 20, Some(a)).unwrap();
        let a = t.archive.as_ref().unwrap();
        assert_eq!(Telemetry::archived(a), 2, "both generations must survive a restart");
        let mut contents: Vec<String> = std::fs::read_dir(&a.dir).unwrap().flatten()
            .filter(|e| !e.file_name().to_string_lossy().contains("live-snapshot"))
            .map(|e| std::fs::read_to_string(e.path()).unwrap().chars().take(3).collect()).collect();
        contents.sort();
        assert_eq!(contents, vec!["NEW".to_string(), "OLD".to_string()]);
    }

    #[test]
    fn restart_does_not_rearchive_a_generation_already_archived_at_rotation() {
        // Prosecution #3: rotate() archives the outgoing generation, then a restart
        // archived `.1` again — an exact 8 MB duplicate per restart.
        let d = tmp("dup");
        let live = format!("{d}/live.csv");
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let mut t = Telemetry::open(&live, 400, Some(a)).unwrap();
        for i in 0..20 {
            t.write_line(&format!("{i},1,2,3,4,5,6,US,,0,,7,")).unwrap();
        }
        let n_after_rotation = Telemetry::archived(t.archive.as_ref().unwrap());
        assert!(n_after_rotation >= 1);
        drop(t);
        // restart: the live file is new data (archive it), `.1` was already archived
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let t2 = Telemetry::open(&live, 400, Some(a)).unwrap();
        assert_eq!(
            Telemetry::archived(t2.archive.as_ref().unwrap()),
            n_after_rotation + 1,
            "restart must add exactly the live file, not a duplicate of .1"
        );
    }

    #[test]
    fn restart_fragments_cannot_evict_real_generations() {
        // Prosecution #2: count-based prune let a crash loop of tiny restart
        // fragments push out the 8 MB generations. Prune must be size-aware.
        let d = tmp("frag");
        let live = format!("{d}/live.csv");
        let adir = format!("{d}/archive");
        // one "real" generation of ~cap bytes already archived
        std::fs::create_dir_all(&adir).unwrap();
        std::fs::write(format!("{adir}/telemetry-1000000000-0-rotate.csv"), "x".repeat(4000)).unwrap();
        for k in 0..30 {
            std::fs::write(&live, format!("t,grid\n{k},1,2,3,4,5,6,US,,0,,7,\n").repeat(8)).unwrap();
            let a = Archive::new(&adir, 3, std::time::Duration::from_secs(3600));
            let _t = Telemetry::open(&live, 4000, Some(a)).unwrap(); // each restart archives a fragment
        }
        assert!(
            std::path::Path::new(&format!("{adir}/telemetry-1000000000-0-rotate.csv")).exists(),
            "30 restart fragments must not evict the real generation"
        );
        let frags = std::fs::read_dir(&adir).unwrap().flatten()
            .filter(|e| e.file_name().to_string_lossy().contains("restart")).count();
        assert!(frags <= 3, "fragments themselves are capped at keep: {frags}");
    }

    #[test]
    fn clean_shutdown_archives_the_live_file() {
        // Prosecution #6: a graceful stop lost up to snapshot_every of data.
        let d = tmp("shutdown");
        let live = format!("{d}/live.csv");
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_secs(3600));
        let mut t = Telemetry::open(&live, 1 << 20, Some(a)).unwrap();
        for i in 0..10 {
            t.write_line(&format!("{i},1,2,3,4,5,6,US,,0,,7,")).unwrap();
        }
        t.archive_now("shutdown");
        assert_eq!(Telemetry::archived(t.archive.as_ref().unwrap()), 1);
    }

    #[test]
    fn periodic_snapshot_copies_the_live_file() {
        let d = tmp("snap");
        let live = format!("{d}/live.csv");
        let a = Archive::new(&format!("{d}/archive"), 14, std::time::Duration::from_millis(1));
        let mut t = Telemetry::open(&live, 1 << 20, Some(a)).unwrap();
        for i in 0..8 {
            t.write_line(&format!("{i},1,2,3,4,5,6,US,,0,,7,")).unwrap(); // > 128 B of data
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
        t.write_line("9,1,2,3,4,5,6,US,,0,,7,").unwrap();
        assert!(std::path::Path::new(&format!("{d}/archive/telemetry-live-snapshot.csv")).exists());
    }
}
