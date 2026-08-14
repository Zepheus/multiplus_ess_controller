# ess_lab — an ESS control-loop sandbox for the Venus GX

A faithful, readable model of the stock ESS AC-in power loop, wrapped so you can
**experiment with different integrals** and **validate against reality** before anything
ever writes to the live system.

It is a compact model of the ~30 lines of control law that matter, derived from the
stock loop's observed input/output behaviour (grid meter, inverter AC-in power, and the
resulting setpoint command) and fitted to 1 Hz measurements.

## The model (validated to the watt against a reference capture)

```
reported = k * command            # the Multi under-achieves its command, k ≈ 0.9625
grid     = load + reported        # KCL at the AC-in bus
stock law:  command = target - grid + reported     # what the stock ESS loop actually does
=> grid_ss = (1-k)*load + k*target                 # the leak: (1-k)*load ≈ 3.75% of load
```

`selfcheck` proves the simulator reproduces the captured operating point
(grid 83 W, command −1680 W at load 1700 W) and the closed-form fixed points.

## Files

| file | what |
|------|------|
| `plant.py` | the validated plant model (deficit `k`, 400 W/s slew, optional lag/noise) |
| `controllers.py` | **edit this** — Stock, TrueIntegral, PI, PID, TargetTrim, all pluggable |
| `run.py` | modes: `selfcheck`, `sim`, `compare`, `shadow` |

## Modes

```bash
# 1. prove the sim is faithful
python3 -m ess_lab.run selfcheck

# 2. experiment offline (synthetic load, or replay your real capture)
python3 -m ess_lab.run sim --controller pi --seconds 600
python3 -m ess_lab.run sim --controller target-trim --csv ../capture.csv

# 3. compare every controller side by side
python3 -m ess_lab.run compare --csv ../capture.csv

# 4. SHADOW MODE — live, READ-ONLY validation on the GX (never writes)
python3 -m ess_lab.run shadow --host root@venus.local --controller stock --seconds 120 \
        --out shadow.csv
```

### Shadow mode is the safety spine

`shadow` connects to the live GX read-only (only `GetValue`), runs a controller on the
real dbus inputs each second, and logs **what it would command** next to **what
the stock ESS loop actually commanded**. It performs **no writes** in any code path.

Two jobs:

1. **Fidelity check.** With `--controller stock`, our proposed command should track the
   real `/Hub4/L1/AcPowerSetpoint` sample-by-sample. Mean |actual − proposed| below a
   few watts (noise level) proves the reimplementation is faithful. Only meaningful in
   the discharge regime (SystemState 256).
2. **Candidate evaluation.** With `--controller pi`/`target-trim`, log what a candidate
   *would* do on live data — de-risking before any go-live.

## Experimenting with integrals

Everything interesting is in `controllers.py`. To try a new idea, add a class with an
`update(grid, reported, target, dt, lo, hi) -> command` method and register it in
`REGISTRY`. Guidelines the model already encodes:

- **`stock`** — the bug, for reference. No integral, leaks `(1-k)*load`.
- **`true-integral`** — the *one-line firmware-style fix*: key the correction on the
  previous **command** instead of `reported`. Drives grid→target regardless of `k`.
  This is what a corrected the stock ESS loop would do.
- **`pi`** — keep stock's feed-forward (instant load-step rejection) **and** add a slow
  integral to kill the steady leak. Best steady + transient combination.
- **`target-trim`** — the **deployable, low-risk** path. Does *not* replace the stock ESS loop;
  a slow integrator (minutes) slides `Settings/CGwacs/AcPowerSetPoint` so the stock law
  lands on target. If it dies, the setting just freezes = today's behaviour, never worse.

Watch **export/day** when raising `ki`: aggressive integral action overshoots into
(unpaid) export on load downswings. `target-trim`'s EMA + slow `ki` trades settling
speed for gentleness on purpose.

## Safety

- **No mode writes to the GX.** Not `sim`, not `compare`, not `shadow`. There is
  deliberately no `--apply` here.
- Going live later is a *separate*, explicitly-gated step. The safe on-device form is
  the `target-trim` setting write (`Settings/CGwacs/AcPowerSetPoint = target + I`), a
  single bounded (±1 MW allowed, we clamp to ±200 W) watchdog-free value whose only
  consumer is the stock ESS loop's control math — never firmware. The risky form (writing the
  vebus `/Hub4/L1/AcPowerSetpoint` directly under Hub4Mode 3) is out of scope.
