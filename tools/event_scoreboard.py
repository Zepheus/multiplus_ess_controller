#!/usr/bin/env python3
"""Event library + matched-pair scoreboard for the venus-ess controller.

Extracts every load edge > EDGE_W from capture CSVs, reconstructs the
controller-independent load profile around each event, and replays each event
through the VALIDATED plant model (postdiction-checked against captured live
events; see the postdiction gate tests) under one or more controller configs.
Comparisons are per-event on identical loads — never aggregate medians.

Usage:
  event_scoreboard.py extract <capture.csv> [...]        # list events
  event_scoreboard.py score   <capture.csv> [...]        # scoreboard: all configs
  event_scoreboard.py ab      <session.csv> <baseline.csv> [...]
        # A/B: per-event metrics of a live session vs matched-magnitude events
        # from a baseline capture (see docs/AB-PROTOCOL.md)

Metrics per event: export Wh (grid < 0), import-above-target Wh, peak export W,
export duration s (grid < -300 W). The DURATION column is the perceptual metric.
"""
import csv, json, math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
_FIT_FALLBACK = {  # identified 2026-08-22 from live actuation traces
    "static": {"a": 4.7, "b": 0.9562},
    "dynamics": {"first_order_alpha": 0.8, "transport_delay_s": 1},
    "meter": {"lag_s": 1},
}
try:
    FIT = json.load(open(os.path.join(HERE, "..", "captures",
                                      "2026-08-21-live-takeover", "plantfit-v2.json")))
except OSError:
    FIT = _FIT_FALLBACK
A = FIT["static"]["a"]; B = FIT["static"]["b"]
ALPHA = FIT["dynamics"]["first_order_alpha"]; DELAY = FIT["dynamics"]["transport_delay_s"]
METER_LAG = FIT["meter"]["lag_s"]
EDGE_W, PRE_S, POST_S, TARGET = 1000.0, 20, 45, 5.0

COLS = "t,grid,reported,target,soc,state,command,owner,actual,fc,maxdis,acout,reason".split(",")

def load_capture(fn):
    rows = {}
    with open(fn) as f:
        for parts in csv.reader(f):
            if not parts or parts[0] == "t" or len(parts) < 13:
                continue
            d = dict(zip(COLS, parts))
            try:
                if d["fc"] == "1" or d["maxdis"]:
                    continue  # charge regimes excluded from the transient library
                w = float(d["command"]) if d["command"] else float(d["actual"] or "nan")
                rows[int(d["t"])] = dict(grid=float(d["grid"]), rep=float(d["reported"]),
                                         w=w, ao=float(d["acout"]) if d["acout"] else 20.0)
            except ValueError:
                pass
    return rows

def find_events(byt):
    ts = sorted(byt)
    evs, last = [], -10**9
    for t in ts:
        p = byt.get(t - 3)
        if p is None or t - last <= 30:
            continue
        dload = (byt[t]["grid"] - byt[t]["rep"]) - (p["grid"] - p["rep"])
        if abs(dload) > EDGE_W:
            evs.append((t, dload))
            last = t
    return evs

def reconstruct_load(byt, t0, t1):
    load, prev = {}, 0.0
    for t in range(t0, t1 + 1):
        if t in byt:
            prev = byt[t]["grid"] - byt[t]["rep"]
        load[t] = prev
    return load

def simulate(byt, t0, t1, cfg):
    """cfg: dict(name, kind='ours'|'stock', ki, pace, cadence_s, smith, meter_lag)."""
    load = reconstruct_load(byt, t0, t1)
    prev_w = byt[t0]["w"] if not math.isnan(byt[t0]["w"]) else -(load[t0])
    weff = prev_w
    writes = {t0 - 1: prev_w, t0: prev_w}
    weff_hist = {t0: weff}
    grid_hist = {t: byt[t]["grid"] for t in range(t0 - 3, t0 + 1) if t in byt}
    i_state, ph = 0.0, 0
    out = []
    mlag = cfg.get("meter_lag", METER_LAG)
    for t in range(t0 + 1, t1 + 1):
        weff += ALPHA * (writes.get(t - 1 - DELAY, prev_w) - weff)
        weff_hist[t] = weff
        rep = A + B * weff
        grid_true = load[t] + rep
        grid_hist[t] = grid_true
        gm = grid_hist.get(t - mlag, grid_true)
        rep_meas = rep
        if cfg.get("smith"):
            # Lag compensation: we KNOW our write history and the plant model, so
            # correct the stale meter reading by the battery-side change since then.
            rep_at_meter = A + B * weff_hist.get(t - mlag, weff)
            gm = gm + (rep - rep_at_meter)
        law = rep_meas + TARGET - gm
        ao = byt[t]["ao"] if t in byt else 20.0
        if cfg["kind"] == "ours":
            ff = -4.9 - 0.0458 * abs(rep_meas)
            e = max(-100.0, min(100.0, TARGET - gm))
            i_state = max(-300.0, min(300.0, i_state + cfg["ki"] * e))
            cmd = min(law + ff + i_state, ao)
        else:
            cmd = (law if law < 0 else 0.0) + ao
        ph += 1
        cad = cfg.get("cadence_s", 1)
        do_write = True if cad == 1 else (ph % 2 == 0 or ph % 5 == 0)
        if do_write:
            prev_w = round(prev_w + cfg["pace"] * (cmd - prev_w))
        writes[t] = prev_w
        out.append((t, grid_true))
    return out

def metrics(traj):
    exp = sum(-g / 3600 for _, g in traj if g < 0)
    imp = sum((g - TARGET) / 3600 for _, g in traj if g > TARGET)
    peak = min(g for _, g in traj)
    dur = sum(1 for _, g in traj if g < -300)
    return exp, imp, peak, dur

CONFIGS = [
    dict(name="stock(model)", kind="stock", pace=0.5, cadence_s=2.5),
    dict(name="ours-current", kind="ours", ki=0.02, pace=0.242, cadence_s=1),
]

def real_metrics(byt, t0, t1):
    tr = [(t, byt[t]["grid"]) for t in range(t0 + 1, t1 + 1) if t in byt]
    return metrics(tr) if tr else None

def main():
    mode, files = sys.argv[1], sys.argv[2:]
    if mode == "ab":
        files = files[:1] + files[1:]
    header = f"{'event':>10} {'dload':>7} | " + " | ".join(
        f"{c['name']:>13}" for c in CONFIGS) + " |     real(capture)"
    print(header)
    print(f"{'':>20} | " + " | ".join(f"{'exp/pk/dur':>13}" for _ in CONFIGS) + " |")
    agg = {c["name"]: [0.0, 0.0, 0] for c in CONFIGS}
    agg["real"] = [0.0, 0.0, 0]
    n = 0
    for fn in files:
        byt = load_capture(fn)
        for t, dload in find_events(byt):
            t0, t1 = t - PRE_S, t + POST_S
            if t0 not in byt:
                continue
            cells = []
            for c in CONFIGS:
                e, i, p, d = metrics(simulate(byt, t0, t1, c))
                cells.append(f"{e:4.1f}/{p:+5.0f}/{d:2d}")
                agg[c["name"]][0] += e; agg[c["name"]][1] += i; agg[c["name"]][2] += d
            rm = real_metrics(byt, t0, t1)
            real = f"{rm[0]:4.1f}/{rm[2]:+5.0f}/{rm[3]:2d}" if rm else "-"
            if rm:
                agg["real"][0] += rm[0]; agg["real"][1] += rm[1]; agg["real"][2] += rm[3]
            n += 1
            print(f"{os.path.basename(fn)[:6]}@{t:<6} {dload:+7.0f} | "
                  + " | ".join(f"{x:>13}" for x in cells) + f" | {real}")
    print(f"\nTOTALS over {n} events (export Wh / import Wh / export-duration s):")
    for k, (e, i, d) in agg.items():
        print(f"  {k:>14}: {e:6.1f} / {i:6.1f} / {d:5d}")

if __name__ == "__main__":
    main()
