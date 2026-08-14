"""
ess_lab runner: simulate, replay, and shadow-validate ESS controllers.

Modes
-----
  sim      Closed-loop simulation against the validated plant model.
           Synthetic load steps by default, or --csv to replay a real load trace.
  shadow   Connect to the live Venus GX READ-ONLY, run a controller on live dbus
           inputs, and log what it WOULD command next to what the stock ESS loop actually
           commanded.  Never writes.  This is how we validate the reimplementation
           (controller=stock should match the stock loop) and evaluate candidates
           on real data before ever going live.

Nothing in this file writes to the GX in any mode.  A future --apply path would be
a separate, explicitly-gated tool; it does not exist here on purpose.

Examples
--------
  python3 -m ess_lab.run selfcheck
  python3 -m ess_lab.run sim --controller stock
  python3 -m ess_lab.run sim --controller pi --csv ../capture.csv
  python3 -m ess_lab.run compare --csv ../capture.csv
  python3 -m ess_lab.run shadow --host root@venus.local --controller stock --seconds 120
"""

import argparse
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass

from .plant import Plant, K_DEFICIT, steady_state_grid
from . import controllers as C


# --------------------------------------------------------------------------- #
# Metrics
# --------------------------------------------------------------------------- #

@dataclass
class Metrics:
    grid_mean_tail: float      # mean grid over the last third (steady state)
    grid_abs_tail: float       # mean |grid - target| over the tail
    grid_max: float
    grid_min: float
    export_energy_wh: float     # energy exported (grid<0), i.e. unpaid give-away
    import_excess_wh: float     # energy imported ABOVE target (the leak), Wh
    settle_s: float             # time to reach & stay within band of target


def compute_metrics(ts, grid, target, band=10.0):
    n = len(grid)
    tail = grid[max(0, 2 * n // 3):]
    tgt = target if isinstance(target, (int, float)) else target[-1]
    dt_tot = (ts[-1] - ts[0]) / max(1, n - 1)
    exp_wh = sum(-g for g in grid if g < 0) * dt_tot / 3600.0
    imp_excess = sum(max(0.0, g - tgt) for g in grid) * dt_tot / 3600.0
    # settling: last time grid left the band, measured from the end
    settle = 0.0
    for i in range(n - 1, -1, -1):
        if abs(grid[i] - tgt) > band:
            settle = ts[i] - ts[0]
            break
    return Metrics(
        grid_mean_tail=statistics.fmean(tail),
        grid_abs_tail=statistics.fmean(abs(g - tgt) for g in tail),
        grid_max=max(grid), grid_min=min(grid),
        export_energy_wh=exp_wh, import_excess_wh=imp_excess, settle_s=settle,
    )


# --------------------------------------------------------------------------- #
# Load profiles
# --------------------------------------------------------------------------- #

def synthetic_load(duration_s=600, dt=1.0):
    """Evening-ish profile with steps, to exercise transient + steady behaviour."""
    ts, load = [], []
    t = 0.0
    while t <= duration_s:
        if t < 120:      L = 300.0
        elif t < 240:    L = 1700.0     # kettle/oven step
        elif t < 360:    L = 850.0
        elif t < 480:    L = 2600.0     # heavy step
        else:            L = 600.0
        ts.append(t); load.append(L)
        t += dt
    return ts, load


def load_from_csv(path, discharge_only=True, target=20.0):
    """Replay the real load trace.  load = grid - reported (= consumption column).

    CSV columns: ts, grid_w, vebus_acin_w, consumption_w, soc, batt_w, hub4cmd_w, state
    NB: column 6 is the Hub4 *command* output, NOT the grid setpoint setting. The grid
    setpoint during this capture was a constant +20 W (not logged), so we impose it.
    """
    ts, load, tgt = [], [], []
    t0 = None
    with open(path) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) < 8:
                continue
            try:
                t = float(p[0]); grid = float(p[1]); rep = float(p[2])
                state = int(float(p[7]))
            except ValueError:
                continue
            if discharge_only and state != 256:
                continue
            if t0 is None:
                t0 = t
            ts.append(t - t0)
            load.append(grid - rep)        # KCL: load = grid - reported
            tgt.append(target)
    return ts, load, tgt


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #

def simulate(controller, ts, load, target, k=K_DEFICIT, lo=-9000.0, hi=9000.0,
             tau=0.0):
    plant = Plant(k=k, tau=tau)
    controller.reset()
    plant.reset(0.0)
    grid_hist, cmd_hist, rep_hist = [], [], []
    grid = load[0]
    reported = 0.0
    for i in range(len(ts)):
        dt = 1.0 if i == 0 else (ts[i] - ts[i - 1]) or 1.0
        tgt = target[i] if isinstance(target, list) else target
        cmd = controller.update(grid, reported, tgt, dt, lo, hi)
        grid, reported = plant.step(cmd, load[i], dt)
        grid_hist.append(grid); cmd_hist.append(cmd); rep_hist.append(reported)
    return grid_hist, cmd_hist, rep_hist


def _sparkline(vals, lo=None, hi=None, width=60):
    blocks = "▁▂▃▄▅▆▇█"
    if not vals:
        return ""
    step = max(1, len(vals) // width)
    s = vals[::step]
    lo = min(s) if lo is None else lo
    hi = max(s) if hi is None else hi
    rng = (hi - lo) or 1.0
    return "".join(blocks[min(7, max(0, int((v - lo) / rng * 7)))] for v in s)


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #

def cmd_selfcheck(args):
    """Prove the simulator reproduces the captured operating point and the
    closed-form fixed points.  If this fails, don't trust any experiment."""
    print("Self-check: simulator vs closed-form vs reference capture\n")
    k = K_DEFICIT
    L, tgt = 1700.0, 20.0
    ts = list(range(400))
    load = [L] * len(ts)

    ok = True
    # 1) stock law steady state must match closed form and the capture (~83 W)
    g, cmd, rep = simulate(C.StockLaw(), ts, load, tgt, k=k)
    cf = steady_state_grid("stock", L, tgt, k)
    print(f"  stock   grid_ss sim={g[-1]:7.2f}  closed-form={cf:7.2f}  "
          f"capture=83.2   cmd_ss={cmd[-1]:8.1f} (capture -1680)")
    ok &= abs(g[-1] - cf) < 0.5 and abs(g[-1] - 83.2) < 1.5 and abs(cmd[-1] + 1680) < 3

    # 2) true-integral & pi must drive grid to target
    for name in ("true-integral", "pi", "target-trim"):
        g, cmd, rep = simulate(C.REGISTRY[name](), ts, load, tgt, k=k)
        print(f"  {name:13s} grid_ss sim={g[-1]:7.2f}  (target {tgt})")
        ok &= abs(g[-1] - tgt) < 1.0

    print("\n  RESULT:", "PASS ✅" if ok else "FAIL ❌")
    return 0 if ok else 1


def cmd_sim(args):
    if args.csv:
        ts, load, tgt = load_from_csv(args.csv)
        target = tgt
        src = f"replay {args.csv} ({len(ts)} discharge samples)"
    else:
        ts, load = synthetic_load(args.seconds)
        target = args.target
        src = f"synthetic {args.seconds}s"
    ctrl = C.REGISTRY[args.controller]()
    g, cmd, rep = simulate(ctrl, ts, load, target, k=args.k, tau=args.tau)
    m = compute_metrics(ts, g, target)
    tgt0 = target if isinstance(target, (int, float)) else statistics.fmean(target)
    print(f"controller={args.controller}  source={src}  k={args.k}\n")
    print(f"  grid  {_sparkline(g)}")
    print(f"  load  {_sparkline(load)}\n")
    print(f"  steady-state grid   : {m.grid_mean_tail:7.2f} W   (target {tgt0:.0f})")
    print(f"  mean |grid-target|  : {m.grid_abs_tail:7.2f} W")
    print(f"  grid range          : {m.grid_min:7.1f} .. {m.grid_max:.1f} W")
    print(f"  settling time       : {m.settle_s:7.1f} s")
    print(f"  excess import energy : {m.import_excess_wh:6.3f} Wh over the run")
    print(f"  exported (give-away) : {m.export_energy_wh:6.3f} Wh over the run")
    return 0


def cmd_compare(args):
    if args.csv:
        ts, load, tgt = load_from_csv(args.csv)
        target = tgt
        src = f"replay {args.csv}"
    else:
        ts, load = synthetic_load(args.seconds)
        target = args.target
        src = f"synthetic {args.seconds}s"
    tgt0 = target if isinstance(target, (int, float)) else statistics.fmean(target)
    dur_h = (ts[-1] - ts[0]) / 3600.0 or 1e-9
    print(f"Comparison  source={src}  k={args.k}  target≈{tgt0:.0f} W\n")
    print(f"  {'controller':14s} {'grid_ss':>8} {'|err|':>7} {'settle':>7} "
          f"{'excess/day':>11} {'export/day':>11}")
    print("  " + "-" * 62)
    for name in ("stock", "true-integral", "pi", "pid", "target-trim"):
        ctrl = C.REGISTRY[name]()
        g, cmd, rep = simulate(ctrl, ts, load, target, k=args.k, tau=args.tau)
        m = compute_metrics(ts, g, target)
        excess_day = m.import_excess_wh / dur_h * 24.0 / 1000.0
        export_day = m.export_energy_wh / dur_h * 24.0 / 1000.0
        print(f"  {name:14s} {m.grid_mean_tail:7.1f}W {m.grid_abs_tail:6.1f}W "
              f"{m.settle_s:6.0f}s {excess_day:9.3f}kWh {export_day:9.3f}kWh")
    print("\n  excess/day = energy imported above setpoint (the leak, at peak rate)")
    print("  export/day = energy given away (grid<0); watch this on aggressive gains")
    return 0


# ---- shadow: live read-only validation ------------------------------------ #

SHADOW_REMOTE = r'''
gv() { dbus-send --system --print-reply=literal --dest="$1" "$2" \
  com.victronenergy.BusItem.GetValue 2>/dev/null \
  | sed -E 's/.*(variant|int32|double|boolean)[[:space:]]+//; s/[[:space:]]*$//'; }
VE=$(dbus -y 2>/dev/null | grep -m1 'com.victronenergy.vebus' )
: "${VE:=com.victronenergy.vebus.ttyS3}"
while :; do
  G=$(gv com.victronenergy.system /Ac/Grid/L1/Power)
  R=$(gv "$VE" /Ac/ActiveIn/L1/P)
  A=$(gv "$VE" /Hub4/L1/AcPowerSetpoint)
  T=$(gv com.victronenergy.settings /Settings/CGwacs/AcPowerSetPoint)
  S=$(gv com.victronenergy.system /SystemState/State)
  echo "$(date +%s),$G,$R,$A,$T,$S"
  sleep 1
done
'''


def cmd_shadow(args):
    """Run READ-ONLY against the live GX. Log proposed vs actual command.

    For controller=stock this measures reimplementation fidelity: our proposed
    command should track the stock ESS loop's actual /Hub4/L1/AcPowerSetpoint.
    """
    ctrl = C.REGISTRY[args.controller]()
    ctrl.reset()
    out = open(args.out, "w") if args.out else None
    if out:
        out.write("ts,grid,reported,actual_cmd,target,state,proposed_cmd,predicted_grid\n")
    print(f"SHADOW (read-only) host={args.host} controller={args.controller} "
          f"seconds={args.seconds}\n  ctrl+c to stop. NO writes are performed.\n")
    print(f"  {'t':>4} {'grid':>7} {'rep':>8} {'actual':>8} {'propose':>8} "
          f"{'Δcmd':>7} {'state':>5}")
    proc = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", args.host, "sh"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    proc.stdin.write(SHADOW_REMOTE)
    proc.stdin.close()
    t_start = time.time()
    prev_t = None
    prev_inputs = None          # (grid, reported, target) from the previous tick
    diffs, diffs_lag, n = [], [], 0
    lo, hi = -args.max_discharge, args.max_charge
    try:
        for line in proc.stdout:
            p = line.strip().split(",")
            if len(p) != 6:
                continue
            try:
                ts = float(p[0]); grid = float(p[1]); rep = float(p[2])
                actual = float(p[3]); target = float(p[4]); state = int(float(p[5]))
            except ValueError:
                continue
            dt = 1.0 if prev_t is None else (ts - prev_t) or 1.0
            prev_t = ts
            proposed = ctrl.update(grid, rep, target, dt, lo, hi)
            pred_grid = None
            if isinstance(ctrl, C.StockLaw):
                # instantaneous: actual vs proposed from THIS tick's inputs
                diffs.append(actual - proposed); n += 1
                # phase-aligned: the stock ESS loop's actual[n] responds to inputs[n-1]
                if prev_inputs is not None:
                    pg, pr, pt = prev_inputs
                    diffs_lag.append(actual - (pt - pg + pr))
            prev_inputs = (grid, rep, target)
            row = f"  {ts - t_start:4.0f} {grid:7.1f} {rep:8.1f} {actual:8.1f} " \
                  f"{proposed:8.1f} {actual - proposed:7.1f} {state:5d}"
            print(row)
            if out:
                out.write(f"{ts},{grid},{rep},{actual},{target},{state},"
                          f"{proposed},{pred_grid}\n")
            if time.time() - t_start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        if out:
            out.close()
    if diffs:
        md = statistics.fmean(abs(d) for d in diffs)
        ml = statistics.fmean(abs(d) for d in diffs_lag) if diffs_lag else float("nan")
        print(f"\n  fidelity (stock vs real the stock ESS loop), {n} samples:")
        print(f"    instantaneous  mean|actual-proposed| = {md:5.1f} W")
        print(f"    phase-aligned  mean|actual-proposed| = {ml:5.1f} W  "
              f"(actual[n] vs law(inputs[n-1]))")
        best = min(md, ml)
        print("    phase-aligned <~10 W on a transient => reimplementation faithful ✅"
              if best < 12 else
              "    gap persists after alignment => worth a look ⚠️")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="ess_lab", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    sp = sub.add_parser("selfcheck", help="validate the simulator vs the capture")
    sp.set_defaults(func=cmd_selfcheck)

    for name, fn in (("sim", cmd_sim), ("compare", cmd_compare)):
        sp = sub.add_parser(name)
        sp.add_argument("--controller", default="stock", choices=list(C.REGISTRY))
        sp.add_argument("--csv", help="replay a capture CSV instead of synthetic load")
        sp.add_argument("--seconds", type=float, default=600)
        sp.add_argument("--target", type=float, default=20.0)
        sp.add_argument("--k", type=float, default=K_DEFICIT)
        sp.add_argument("--tau", type=float, default=0.0)
        sp.set_defaults(func=fn)

    sp = sub.add_parser("shadow", help="live READ-ONLY validation, never writes")
    sp.add_argument("--host", default="root@venus.local")
    sp.add_argument("--controller", default="stock", choices=list(C.REGISTRY))
    sp.add_argument("--seconds", type=float, default=120)
    sp.add_argument("--out", help="write a CSV log")
    sp.add_argument("--max-discharge", type=float, default=9000.0)
    sp.add_argument("--max-charge", type=float, default=9000.0)
    sp.set_defaults(func=cmd_shadow)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
