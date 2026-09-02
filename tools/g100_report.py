#!/usr/bin/env python3
"""Post-hoc export-limit compliance report (EREC G100 Issue 2 rules) from telemetry.

Replays the G100 Issue 2 operational rules over the RAW grid-meter column of
controller telemetry CSVs (t,grid,reported,target,soc,state,command,owner,
actual,fc,maxdis,acout,reason; 1 Hz; grid < 0 = export in W; `t` = seconds since
the controller process started, so every file is its own run). Nothing here runs
at control time; it is the durable evidence layer chosen after the 2026-09-01
adversarial design review (docs/G100-DESIGN-REVIEW-2026-09-01.md), which rejected
a runtime state machine. Default MEL = the G98 per-phase allowance (16 A x 230 V
= 3680 W) x --phases.

Rules replayed (Issue 2 as quoted by GivEnergy Form B / Alternergy):
  excursion : export beyond MEL + deadband; a dip back inside the limit shorter
              than --bridge-s does NOT end it (hysteresis; 0 = strict)
  State 2   : any excursion. "Reducing within 5 s" = export 5 s after onset is
              shallower than at onset (or the excursion already ended); "back
              within 60 s" is the residential recovery bound.
  recorded  : an excursion shorter than 15 s is merely recorded
  State 3   : (a) an excursion > 60 s; (b) three excursions >= 15 s within 24 h;
              (c) two excursions >= 15 s within 10 min.  A State-3 entry is ONE
              event: it clears the breach history (the device would cease export
              and need a reset), so rules never double count the same excursions.
  lockout   : more than three State-3 entries in the file span (30-day window
              in the spec; files here are shorter)

Durations are inclusive wall-clock spans (t_end - t0 + 1, each 1 Hz sample
standing for its second); sample counts are shown alongside. Time base and
energies integrate over t with per-row dt capped at --gap-s.

Usage:
  tools/g100_report.py [--mel W ...] [--phases N] [--deadband W] [--bridge-s S] files...
  tools/g100_report.py --selftest
"""
import argparse, csv, glob, os, sys
from collections import defaultdict

G98_PER_PHASE_W = 3680.0
THRESHOLDS = (100, 300, 1000, 2000)

def load(path, gap_s):
    """Rows of one file: (t, grid, target, owner, fc, hold, dt). Breaks on t going backwards."""
    rows, prev_t = [], None
    with open(path) as f:
        for p in csv.reader(f):
            if not p or p[0] == "t" or len(p) < 12:
                continue
            try:
                t = int(p[0]); g = float(p[1]); tgt = float(p[3]) if p[3] else 0.0
            except ValueError:
                continue
            if prev_t is not None and t < prev_t:
                sys.exit(f"{path}: t goes backwards at {t} (mixed runs in one file?) — split the file")
            dt = 1.0 if prev_t is None else min(float(t - prev_t), gap_s)
            rows.append((t, g, tgt, p[7] or "?", p[9] == "1", p[10] not in ("", "0"), dt))
            prev_t = t
    return rows

def excursions(rows, limit_w, bridge_s, gap_s):
    """Contiguous runs with export beyond limit_w (dips <= bridge_s bridged; telemetry
    gaps > gap_s end a run). Yields dicts with t0, t_end, dur, samples, peak, owner set,
    fc, hold, reducing5."""
    out, cur, inside_since = [], None, None
    for t, g, _tgt, owner, fc, hold, _dt in rows:
        exp = g < -limit_w
        if cur is not None:
            if t - cur["t_last"] > gap_s:
                out.append(cur); cur = None
            elif exp:
                inside_since = None
            else:
                if inside_since is None:
                    inside_since = t
                if t - inside_since >= bridge_s:
                    out.append(cur); cur = None; inside_since = None
        if exp:
            if cur is None:
                cur = dict(t0=t, t_last=t, t_end=t, samples=0, peak=g, g0=g, g5=None,
                           owners=set(), fc=fc, hold=hold)
            cur["t_end"] = t; cur["peak"] = min(cur["peak"], g)
        if cur is not None:
            cur["t_last"] = t; cur["samples"] += 1; cur["owners"].add(owner)
            cur["fc"] |= fc; cur["hold"] |= hold
            if cur["g5"] is None and t - cur["t0"] >= 5:
                cur["g5"] = g
    if cur: out.append(cur)
    for e in out:
        e["dur"] = e["t_end"] - e["t0"] + 1
        # reducing within 5 s: ended before 5 s, or shallower at +5 s than at onset
        e["reducing5"] = e["dur"] <= 5 or (e["g5"] is not None and e["g5"] > e["g0"])
        e["owner"] = "US" if e["owners"] == {"US"} else ("hub4" if "US" not in e["owners"] else "mixed")
    return out

def replay_state3(exc):
    """Proper event model: chronological; each State-3 entry clears the history."""
    events, hist = [], []   # hist: t0 of >=15 s excursions since the last State 3
    for e in sorted(exc, key=lambda x: x["t0"]):
        if e["dur"] > 60:
            events.append(("excursion > 60 s", e)); hist = []
            continue
        if e["dur"] >= 15:
            if hist and e["t0"] - hist[-1] <= 600:
                events.append(("2 x >=15 s within 10 min", e)); hist = []
                continue
            hist = [h for h in hist if e["t0"] - h <= 86400] + [e["t0"]]
            if len(hist) >= 3:
                events.append(("3 x >=15 s within 24 h", e)); hist = []
    return events

def hours_of(rows):
    return sum(r[6] for r in rows) / 3600.0

def energies(rows):
    exp = sum(-g * dt for _, g, _, _, _, _, dt in rows if g < 0) / 3600
    imp = sum((g - tgt) * dt for _, g, tgt, _, _, _, dt in rows if g > tgt) / 3600
    ours = sum(r[6] for r in rows if r[3] == "US") / max(1e-9, sum(r[6] for r in rows)) * 100
    return exp, imp, ours

def bucket(d):
    return "<5s" if d < 5 else "5-15s" if d < 15 else "15-60s" if d <= 60 else ">60s"

def report_file(path, rows, mels, deadband, bridge, gap):
    hours = hours_of(rows); exp, imp, ours = energies(rows)
    g = [r[1] for r in rows]
    print(f"\n##### {path}")
    print(f"{len(rows)} rows = {hours:.1f} h by t; ours-owned {ours:.1f}% of time; export {exp:.0f} Wh ({exp/max(hours,1e-9)*24:.0f} Wh/day); import-above-target {imp:.0f} Wh; deepest export {min(g):.0f} W")
    print(f"excursion table (bridge {bridge}s), count by inclusive duration; attributable = while our loop owned the setpoint, no force-charge/hold:")
    print(f"{'beyond':>8} | {'<5s':>5} {'5-15s':>6} {'15-60s':>7} {'>60s':>5} | {'total':>5} {'attrib':>6} {'max':>4} {'peak':>7}")
    for th in THRESHOLDS:
        ex = excursions(rows, th, bridge, gap); b = defaultdict(int)
        for e in ex: b[bucket(e["dur"])] += 1
        attrib = sum(1 for e in ex if e["owner"] == "US" and not e["fc"] and not e["hold"])
        print(f"{th:>7}W | {b['<5s']:>5} {b['5-15s']:>6} {b['15-60s']:>7} {b['>60s']:>5} | {len(ex):>5} {attrib:>6} {max((e['dur'] for e in ex), default=0):>3}s {min((e['peak'] for e in ex), default=0):>7.0f}")
    for mel in mels:
        ex = excursions(rows, mel + deadband, bridge, gap)
        ev = replay_state3(ex)
        long15 = [e for e in ex if e["dur"] >= 15]
        red5 = sum(1 for e in ex if e["reducing5"])
        print(f"\n=== G100 replay, MEL {mel:.0f} W (+{deadband:.0f} W deadband, bridge {bridge}s) ===")
        print(f"State-2 excursions: {len(ex)} ({len(ex)/max(hours,1e-9)*24:.1f}/day); reducing within 5 s: {red5}/{len(ex)}; ended within 5 s: {sum(1 for e in ex if e['dur']<=5)}/{len(ex)}")
        print(f">=15 s: {len(long15)} (samples<15 in {sum(1 for e in long15 if e['samples']<15)} of them); longest {max((e['dur'] for e in ex), default=0)} s; not attributable to our loop: {sum(1 for e in ex if e['owner']!='US' or e['fc'] or e['hold'])}")
        print(f"State-3 events: {len(ev)}" + ("  -> LOCKOUT (>3)" if len(ev) > 3 else ""))
        for why, e in ev[:10]:
            print(f"   {why:>26}: t={e['t0']} dur {e['dur']} s ({e['samples']} samples) peak {e['peak']:.0f} W owner {e['owner']}{' fc' if e['fc'] else ''}{' hold' if e['hold'] else ''}")
        # sensitivity grid: the MEL-0 verdict depends on these two choices; show it
        print("   sensitivity (>=15 s excursions / State-3 events):", end="")
        for db in (0.0, 50.0, 100.0):
            for br in (0, 2):
                exs = excursions(rows, mel + db, br, gap); evs = replay_state3(exs)
                print(f"  db{db:.0f}/br{br}: {sum(1 for e in exs if e['dur']>=15)}/{len(evs)}", end="")
        print()

def selftest():
    """Hand-made excursion lists exercising the State-3 event model."""
    def mk(t0, dur): return dict(t0=t0, dur=dur)
    # 10 excursions >=15 s, 5 min apart: pairs -> State 3 at #2, reset, #4, #6, #8, #10 = 5 events
    ev = replay_state3([mk(300 * i, 20) for i in range(10)])
    assert len(ev) == 5 and all(w.startswith("2 x") for w, _ in ev), ev
    # 3 x 61 s hourly: rule (a) three times, and NEVER rule (b) on top (no double count)
    ev = replay_state3([mk(3600 * i, 61) for i in range(3)])
    assert len(ev) == 3 and all(w.startswith("excursion") for w, _ in ev), ev
    # 3 x 20 s spread 2 h apart: rule (b) exactly once (then history cleared)
    ev = replay_state3([mk(7200 * i, 20) for i in range(3)])
    assert len(ev) == 1 and ev[0][0].startswith("3 x"), ev
    # 14 s excursions never count
    assert replay_state3([mk(60 * i, 14) for i in range(20)]) == []
    # bridging: a 1-tick dip inside a 19 s run is one excursion when bridge=2, two when 0
    rows = [(t, -500.0 if t != 10 else 0.0, 5.0, "US", False, False, 1.0) for t in range(1, 20)]
    assert len(excursions(rows, 50, 2, 3)) == 1 and len(excursions(rows, 50, 0, 3)) == 2
    print("selftest OK")

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*")
    ap.add_argument("--mel", type=float, action="append", help=f"Maximum Export Limit in W (repeatable); default G98 {G98_PER_PHASE_W:.0f} W x --phases")
    ap.add_argument("--phases", type=int, default=1)
    ap.add_argument("--deadband", type=float, default=0.0, help="W beyond MEL before an excursion counts (0 = strict; the spec has none)")
    ap.add_argument("--bridge-s", type=int, default=0, help="dips back inside the limit up to this long do not end an excursion (0 = strict)")
    ap.add_argument("--gap-s", type=float, default=3.0, help="telemetry gaps longer than this end an excursion and are not integrated")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest(); return
    files = [f for pat in a.files for f in sorted(glob.glob(pat))]
    if not files:
        sys.exit("no input files")
    mels = a.mel or [G98_PER_PHASE_W * a.phases]
    for fn in files:
        rows = load(fn, a.gap_s)
        if rows:
            report_file(fn, rows, mels, a.deadband, a.bridge_s, a.gap_s)
        else:
            print(f"\n##### {fn}: no rows")

if __name__ == "__main__":
    main()
