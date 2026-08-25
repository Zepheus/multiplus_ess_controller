"""Pareto sweep for the export-conditional write pacing.

How the shipped values (--export-fast-gain 0.9, --export-fast-dead-w 100) were
derived, and how to re-derive them for YOUR system:

1. Build an event library: run a takeover (or shadow) day with --telemetry and
   feed the CSV(s) here. Events are load edges > 1 kW (see event_scoreboard.py,
   which this reuses: same identified plant model, same metrics).
2. This script replays every event through the control law with the pacing rule
     gain_up = fast   if meter shows export beyond a deadband AND the step
                      raises the setpoint (reduces export)
             = 0.5    otherwise (stock-parity symmetric pace)
   sweeping fast-gain x deadband, and prints export Wh / import Wh / export
   duration / worst peak per config.
3. Pick from the front: the shipped 0.9/100 W is the knee — most of the
   duration/export win, smallest import give-back, peak shallower. Validate on
   a capture with rapidly CYCLING loads (hob) before trusting any faster gain:
   the worst-single-event import column is the oscillation guard.
4. Confirm live with a supervised A/B session (docs/AB-PROTOCOL.md) — the sim's
   absolute durations are optimistic; only the relative deltas are trusted.

Usage: python3 tools/pace_sweep.py <capture.csv> [more.csv ...]
"""
import sys, math, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import event_scoreboard as es

CAPS=sys.argv[1:] or [os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','captures','2026-08-25-fullday-live.csv')]
TARGET=es.TARGET; A,B,ALPHA,DELAY=es.A,es.B,es.ALPHA,es.DELAY

def simulate_cond(byt,t0,t1,pace,pace_fast,dead,ki=0.02,smith=True,mlag=1):
    load=es.reconstruct_load(byt,t0,t1)
    prev_w=byt[t0]["w"] if not math.isnan(byt[t0]["w"]) else -(load[t0])
    weff=prev_w; writes={t0-1:prev_w,t0:prev_w}; weff_hist={t0:weff}
    grid_hist={t:byt[t]["grid"] for t in range(t0-3,t0+1) if t in byt}
    i_state=0.0; out=[]
    for t in range(t0+1,t1+1):
        weff+=ALPHA*(writes.get(t-1-DELAY,prev_w)-weff); weff_hist[t]=weff
        rep=A+B*weff; grid_true=load[t]+rep; grid_hist[t]=grid_true
        gm=grid_hist.get(t-mlag,grid_true)
        if smith:
            gm=gm+(rep-(A+B*weff_hist.get(t-mlag,weff)))
        law=rep+TARGET-gm
        ao=byt[t]["ao"] if t in byt else 20.0
        ff=-4.9-0.0458*abs(rep)
        e=max(-100.0,min(100.0,TARGET-gm))
        i_state=max(-300.0,min(300.0,i_state+ki*e))
        cmd=min(law+ff+i_state,ao)
        # export-conditional pacing: measured export beyond deadband AND the step
        # raises the setpoint (reduces export) -> fast gain
        g=pace_fast if (gm<-dead and cmd>prev_w) else pace
        prev_w=round(prev_w+g*(cmd-prev_w)); writes[t]=prev_w
        out.append((t,grid_true))
    return out

byt={}
for cap in CAPS:
    byt.update(es.load_capture(cap))
events=[(t,d) for t,d in es.find_events(byt) if t-es.PRE_S in byt]
print(f"{len(events)} events\n")
print(f"{'config':>22} | exp Wh | imp Wh | dur s | worst-peak W")

def run(name,fn):
    E=I=D=0.0; P=0
    for t,_ in events:
        t0,t1=t-es.PRE_S,t+es.POST_S
        e,i,p,d=es.metrics(fn(byt,t0,t1))
        E+=e;I+=i;D+=d;P=min(P,p)
    print(f"{name:>22} | {E:6.1f} | {I:6.1f} | {D:5.0f} | {P:+8.0f}")
    return E,I,D,P

run("stock(model)", lambda b,a,z: es.simulate(b,a,z,dict(kind="stock",pace=0.5,cadence_s=2.5)))
run("live-baseline 0.5sym", lambda b,a,z: simulate_cond(b,a,z,0.5,0.5,1e9))
for dead in (20,50,100,200):
    for pf in (0.7,0.9,1.1):
        run(f"fast={pf} dead={dead}W", lambda b,a,z,pf=pf,dead=dead: simulate_cond(b,a,z,0.5,pf,dead))
