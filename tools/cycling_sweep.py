"""Cycling-detector variants, replayed over the live event library.

The cycling gate (loop_core::cycling_update) switches the write pacing back to the
stock-parity base gain and freezes the integrator whenever >= 2 load edges land inside
a 30 s window. It exists for modulating loads (hob thermostats), where full-authority
err-gain pacing chases the modulation. Live 2026-09-06 15:26 it also fired on a load
that came ON in two stages and then dropped 3.8 kW eight seconds later: the step-off
was arrested at half-step pacing and the meter-lag export reached -4.0 kW for 3 ticks.

This replays every load edge > 1 kW in the given captures through the CURRENT law on
the identified plant (same model as event_scoreboard.py) under detector variants:

  current     edges |dL| > 300 W, 3 s debounce, >= 2 in 30 s (the rule before 2026-09-07)
  alternate   as current, but only counts when consecutive edges alternate in sign
              (a staged ramp-UP is not a cycle; on/off/on is)
  bigstep     as current, but a tick whose own |dL| > BIG_W keeps full authority
  count3      as current with >= 3 edges (SHIPPED: loop_core::CYC_COUNT = 3 - export -8..10 %,
              duration -10 %, import +1 %, hob oscillation guard unchanged, across ~870 events)
  nogate      no cycling gate (err-gain always)
  base        no err-gain at all (stock-parity 0.5 pacing) - the reference

Metrics per config: export Wh, import-above-target Wh, seconds below -300 W, worst
peak, and the worst single-event import (the hob oscillation guard).

Usage: python3 tools/cycling_sweep.py <capture.csv> [more ...]  [--postdict t]
"""
import sys, math, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import event_scoreboard as es

TARGET = es.TARGET
A, B, ALPHA, DELAY = es.A, es.B, es.ALPHA, es.DELAY
SLEW = 3515.0
EDGE_W, DEB_S, WIN_S = 300.0, 3.0, 30.0
BIG_W = 1500.0
ERR_DEAD, ERR_SLOPE, ERR_CAP = 300.0, 2000.0, 1.0


def simulate(byt, t0, t1, variant, errgain=True, mlag=1):
    load = es.reconstruct_load(byt, t0, t1)
    prev_w = byt[t0]["w"] if not math.isnan(byt[t0]["w"]) else -(load[t0])
    weff = prev_w
    writes = {t0 - 1: prev_w, t0: prev_w}
    weff_hist = {t0: weff}
    grid_hist = {t: byt[t]["grid"] for t in range(t0 - 3, t0 + 1) if t in byt}
    i_state = 0.0
    edges = []  # (t, sign)
    prev_load = float("nan")
    out = []
    for t in range(t0 + 1, t1 + 1):
        weff += ALPHA * (writes.get(t - 1 - DELAY, prev_w) - weff)
        weff_hist[t] = weff
        rep = A + B * weff
        grid_true = load[t] + rep
        grid_hist[t] = grid_true
        gm_raw = grid_hist.get(t - mlag, grid_true)
        corr = rep - (A + B * weff_hist.get(t - mlag, weff))
        gm = gm_raw + corr
        # --- cycling detector (decided from edges BEFORE this tick) ---
        load_est = gm - rep
        live = [(e, s) for e, s in edges if t - e <= WIN_S]
        need = 3 if variant == "count3" else 2
        cyc = len(live) >= need
        if variant == "alternate" and cyc:
            cyc = live[-1][1] != live[-2][1]
        d_load = load_est - prev_load if not math.isnan(prev_load) else 0.0
        newest = max((e for e, _ in edges), default=-1e9)
        if abs(d_load) > EDGE_W and t - newest > DEB_S:
            edges.append((t, 1 if d_load > 0 else -1))
        prev_load = load_est
        if variant == "bigstep" and abs(d_load) > BIG_W:
            cyc = False
        if variant == "nogate":
            cyc = False
        # --- law ---
        law = rep + TARGET - gm
        ao = byt[t]["ao"] if t in byt else 20.0
        ff = -4.9 - 0.0458 * abs(rep)
        if not cyc:
            e = max(-100.0, min(100.0, TARGET - gm))
            i_state = max(-300.0, min(300.0, i_state + 0.02 * e))
        cmd = min(law + ff + i_state, ao)
        # --- write pacing ---
        err = abs(gm - TARGET)
        if errgain and not cyc and err > ERR_DEAD:
            g = min(0.5 + (err - ERR_DEAD) / ERR_SLOPE, ERR_CAP)
        else:
            g = 0.5
        ema = prev_w + g * (cmd - prev_w)
        step = max(-SLEW, min(SLEW, ema - prev_w))
        prev_w = round(prev_w + step)
        writes[t] = prev_w
        out.append((t, grid_true))
    return out


def main():
    argv = sys.argv[1:]
    post = None
    if "--postdict" in argv:
        i = argv.index("--postdict")
        post = int(argv[i + 1])
        del argv[i:i + 2]
    args = [a for a in argv if not a.startswith("--")]
    byt = {}
    for cap in args:
        byt.update(es.load_capture(cap))
    events = [(t, d) for t, d in es.find_events(byt) if t - es.PRE_S in byt]
    print(f"{len(events)} events from {len(args)} file(s)\n")
    configs = [("base", "current", False), ("current", "current", True), ("alternate", "alternate", True),
               ("bigstep", "bigstep", True), ("count3", "count3", True), ("nogate", "nogate", True)]
    print(f"{'config':>10} | exp Wh | imp Wh | dur s | worst-peak W | worst-event imp Wh")
    for name, variant, eg in configs:
        E = I = D = 0.0
        P = 0.0
        WI = 0.0
        for t, _ in events:
            t0, t1 = t - es.PRE_S, t + es.POST_S
            e, i, p, d = es.metrics(simulate(byt, t0, t1, variant, eg))
            E += e; I += i; D += d; P = min(P, p); WI = max(WI, i)
        print(f"{name:>10} | {E:6.1f} | {I:6.1f} | {D:5.0f} | {P:+8.0f}     | {WI:6.2f}")
    if post is not None:
        t0, t1 = post - es.PRE_S, post + es.POST_S
        sim = dict(simulate(byt, t0, t1, "current", True))
        print(f"\npostdiction around t={post} (real grid vs sim current):")
        for t in range(post - 4, post + 10):
            r = byt.get(t)
            print(f"  t={t}  real {r['grid'] if r else float('nan'):8.0f}   sim {sim.get(t, float('nan')):8.0f}")


if __name__ == "__main__":
    main()
