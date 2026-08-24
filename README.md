# multiplus_ess_controller

An experimental replacement for the grid-setpoint control loop of a Victron
MultiPlus-II ESS on a Venus GX. It runs its own control law (with proper integral
action) instead of the stock behaviour, to hold the grid meter at the configured
setpoint.

---

## ⚠️ Disclaimer — personal-use experimentation only

**This is a personal hobby experiment, provided with NO WARRANTY of any kind and NO
LIABILITY accepted by the author.** It is not a product, not supported, and not fit
for any particular purpose.

- It controls grid-tied battery inverters and a home battery. Misconfiguration or bugs
  can damage equipment, discharge/charge the battery incorrectly, cause unintended grid
  import/export, and **may void your Victron warranty** or violate local grid
  regulations.
- It takes over a safety- and money-relevant control loop (`Hub4Mode 3`). You are
  solely responsible for anything it does on your system.
- **Not affiliated with, endorsed by, or connected to Victron Energy.** "Victron",
  "MultiPlus", "Venus" etc. are trademarks of their respective owners.

**Use entirely at your own risk.** By building or running this you accept that the
author is not liable for any damage, loss, cost, or injury of any kind. If that is not
acceptable to you, do not use it.

---

## What it does

Under ESS control the stock loop leaves a small, load-proportional gap: the grid meter
settles a few percent of house load *above* the setpoint (e.g. ~20 W target but ~60–100 W
actual import), because the inverters slightly under-achieve their commanded AC-input
power and the stock loop has no integral term to trim that residual out. Over a day this
is a steady, avoidable import cost.

This controller takes the setpoint loop (`Hub4Mode` 1 → 3) and runs the stock law
**plus three additions the stock loop never had**:

1. **Integral action** — drives the grid meter onto the setpoint and kills the
   standing leak (live-validated: median error ~+1 W vs stock's +19 W).
2. **A leak feed-forward** — your system's leak-vs-load curve, either learned
   online (`--ff-mode observe` to learn without acting) or measured once and
   frozen (`--ff-mode frozen --ff-a <a> --ff-b <b>`, the recommended config),
   so the correction is instant after load changes.
3. **Smith-predictor meter-lag compensation** (`--no-smith` to disable) — grid
   meters report ~1 s stale; during the controller's own ramps that staleness
   causes overshoot and export spikes at appliance step-offs. The compensation
   corrects the stale reading using the controller's own write history and an
   identified inverter-response model. On a 500+-event library this beats the
   stock loop on export, import AND export duration simultaneously.

Everything else deliberately mimics the stock loop exactly (write smoothing,
no-charge rule outside charge regimes, force-charge/window handling — all
verified against live captures to the watt). It only ever discharges/idles
within limits and hands control back to the stock firmware for anything outside
a safe envelope (see Safety).

Layout: the Rust crate is the on-device controller (static ARM binary, no
runtime deps); `src/tests/` holds the cross-module golden suites including a
replay harness with the identified plant + meter model and **postdiction
gates** (the simulator must reproduce captured live events before its
predictions count); `tools/event_scoreboard.py` scores any live session
per-event against baselines; `docs/AB-PROTOCOL.md` is the pre-registered
live-testing protocol. (`simulator/` is the original Python sandbox, kept for
reference — the maintained simulation is the Rust harness with the identified
plant model.)

## Build (cross-compile from macOS/Linux to the GX)

```bash
rustup target add armv7-unknown-linux-musleabihf
# macOS: brew install messense/macos-cross-toolchains/armv7-unknown-linux-musleabihf
cargo build --release   # -> target/armv7-unknown-linux-musleabihf/release/multiplus_ess_controller… (static)
```

Produces a ~1 MB fully static musl binary that runs on any Venus OS regardless of glibc.
Needs Rust ≥ 1.87 (the `zbus` 5 MSRV); any current stable works.

## Run

```bash
# shadow (default, READ-ONLY): compute & log what it WOULD command, never write
./multiplus_ess_controller --stage shadow --controller stock    --seconds 30
./multiplus_ess_controller --stage shadow --controller adaptive --telemetry /run/ess.csv --quiet

# live (double-gated): takes Hub4Mode=3 within the safety envelope, restores on exit.
# Recommended config: frozen feed-forward at your measured leak curve + Smith:
VENUS_ESS_LIVE=I_UNDERSTAND ./multiplus_ess_controller --stage takeover --confirm \
    --controller adaptive --ff-mode frozen --ff-a -4.9 --ff-b -0.046 --ki 0.02 \
    --seconds 1800 --min-soc 20
```

Controllers: `stock` (reference, no fix), `true-integral`, `pi`, and `adaptive`
(`pi` + the leak feed-forward). `adaptive` has three `--ff-mode`s: `frozen`
(apply a measured curve — recommended), `observe`/`learn-only` (act as plain
PI while the model learns for inspection), and `apply` (closed-loop learning —
carries a documented cold-start fragility; see the notes in `src/tests/replay.rs`
before using it unattended). Smith compensation is on by default (`--no-smith`).

## Install as a service (Venus OS, daemontools)

Venus OS supervises everything with daemontools (`svscan` on `/service`), and runs
`/data/rc.local` at every boot. Only `/data` survives reboots **and** firmware
updates, so the service lives there and re-installs itself into the tmpfs service
tree each boot — supervise state and logs never touch the eMMC flash. Ready-made
files are in [`deploy/`](deploy/).

```bash
# from the repo root, with the cross-compiled binary built:
scp target/armv7-unknown-linux-musleabihf/release/multiplus_ess_controller \
    root@<gx-ip>:/data/venus-ess/venus-ess-controller
scp -r deploy/service root@<gx-ip>:/data/venus-ess/
scp deploy/rc.local  root@<gx-ip>:/data/rc.local
ssh root@<gx-ip> 'chmod +x /data/rc.local /data/venus-ess/service/run \
    /data/venus-ess/service/log/run /data/venus-ess/venus-ess-controller \
    && /data/rc.local'
```

`svscan` picks the new service up within ~5 s — no reboot needed. Check and drive
it with the daemontools tools:

```bash
svstat /service/venus-ess           # up (pid ...) N seconds
svc -t /service/venus-ess           # restart (e.g. after deploying a new binary)
svc -d /service/venus-ess           # stop (supervise keeps it down)
svc -u /service/venus-ess           # start again
tail -f /run/venus-ess-svclog/current   # timestamped stderr/stdout (tmpfs, bounded)
```

`deploy/` also ships **`guard-service/`** — an independent watchdog that
restores the stock loop's authority (`Hub4Mode=1`) within 60 s whenever the
controller is not running, so a killed process can never leave the system
ownerless; `rc.local` installs it alongside and adds a boot-time restore.

The shipped `deploy/service/run` starts the **shadow stage (read-only)**. To
promote a validated install, edit the `exec` line's flags (`--stage`, the
double-gate env var, controller choice) — the service supervises whatever stage
you configure. Crash recovery: supervise restarts the process within seconds;
on restart the daemon sanitizes a stale external `Hub4Mode` left by an unclean
death, so the stock loop is always recoverable.

To uninstall: `svc -d /service/venus-ess`, remove the `/service/venus-ess`
symlink, delete `/data/venus-ess`, and remove (or edit) `/data/rc.local`.

## Tuning it to your system

Every ESS install leaks by a different amount — it depends on your inverter model, how
many units, and your wiring (a system with two inverters is not the same as one). This
tool does **not** assume any particular number. The `adaptive` controller discovers
*your* leak on its own and needs no manual tuning.

**1. Just run it (recommended).** The integral measures your grid error every tick and
trims it out; on top of that, `adaptive` fits a two-number model of your leak versus load —
a fixed offset and a percent-of-load slope — and feeds it forward so a sudden load change
is corrected in the same tick instead of a few ticks later.

```bash
./multiplus_ess_controller --stage takeover --confirm --controller adaptive
```

It starts knowing nothing: with no data the model has no authority and it behaves like a
plain integral (`pi`). As it gathers **clean, steady-state** samples it grows confident and
takes over the bulk of the correction, leaving the integral to mop up the residual. The
model **can only ever make it faster, never less correct** — if it is empty, uncertain, or
wrong, the integral still holds the grid on target.

*What it needs to learn:* ordinary household load variation over roughly **1–3 hours** — not
a deliberate power sweep. It only needs to see your load at a *spread* of levels (e.g. quiet
baseload and something heavier), not the full inverter range; because the model is a straight
line it correctly extrapolates to loads it has never sat at. The slope is only trusted once it
has seen enough spread to pin it down — until then the offset alone is used. A few hundred
clean samples is plenty.

**2. Seed it, if you already know your numbers.** If you have measured your offset (W) and
slope (fraction of load) before, hand them over to skip the warm-up. It still keeps adapting.

```bash
./multiplus_ess_controller --stage takeover --confirm --controller adaptive \
    --ff-a 25 --ff-b 0.04         # ~25 W fixed + 4 % of load
```

**3. Watch it learn without trusting it (`--ff-learn-only`).** Drive with the plain
integral while the model learns on the side, so you can watch it converge before letting
the feed-forward act. The learned model is printed periodically:

```bash
VENUS_ESS_LIVE=I_UNDERSTAND ./multiplus_ess_controller --stage takeover --confirm \
    --controller adaptive --ff-learn-only
# ... MODEL: a=24.8W b=0.0391 (3.9% of load) conf=0.72 @load=1500W
```

> **Learning requires actuation.** The model only learns while this controller is *really
> driving* the inverter (takeover, owning the loop, meter healthy) — a loop that isn't
> actuating can't know what its corrections would have done, so training there would
> poison the model. In `--stage shadow` (and trim) the model therefore deliberately stays
> null (`MODEL: a=0.0 b=0.0000`) — that is by design, not a malfunction. Shadow is for
> validating the *stock-law fidelity* (`--controller stock`), not for training.

Use `--ff-mode frozen` (with `--ff-a`/`--ff-b`) to pin a known-good model and stop learning —
handy for reproducible A/B runs.

### Reading the model line — is it working?

Every ~minute the controller prints its current model:

```
MODEL: a=24.8W b=0.0391 (3.9% of load) conf=0.72 @load=1500W
```

- **`a`** — the fixed offset (watts) it will always add. Converges first (needs only a
  settled reading at any load).
- **`b`** — the slope, as a fraction of throughput (`3.9%` here). Needs a *spread* of loads
  before it moves off zero; until then only `a` is applied.
- **`conf`** — how much of the model is actually being actuated right now, `0`–`1`. This is
  the important number: `conf` near `0` means "still learning, behaving like plain `pi`";
  `conf` climbing past ~`0.5` means the feed-forward is carrying the bulk and load-step
  recovery is now fast. In `--ff-learn-only` it shows what authority the model *would* take.
- **`@load`** — the throughput the confidence was evaluated at (confidence is per-load; it is
  high where you have data, low where you don't).

You'll know it has converged when `a` and `b` settle near your leak (compare against the live
gap the plain `pi`/`stock` controllers show in shadow) and `conf` sits high across your usual
load range. If `conf` never rises, your load isn't varying enough to pin the slope — that's
fine, the offset + integral still hold target.

### Suggested rollout

1. **Shadow with `stock`** for a few days — no writes; confirm the
   reimplementation matches your device's real commands (expect a few watts
   median) and collect the transient event library.
2. **Fit your leak curve** from the shadow data: regress the inverter's
   reported power against the written setpoint on steady rows; the residual
   line `bias(x) = a + b·x` is your `--ff-a`/`--ff-b`. (Learn-only takeover
   sessions can do this online instead: `--ff-mode observe`, watch `MODEL:`.)
3. **Short supervised takeover sessions** with `--ff-mode frozen` + your curve,
   following `docs/AB-PROTOCOL.md`: pre-registered abort criteria, and judge
   each session with `tools/event_scoreboard.py` per-event against your own
   stock baseline — never by eyeball or aggregate medians alone.
4. Promote to the daily driver once ~10 transient events score at-or-better
   than your stock baseline. Note: the `MODEL:` log line's `conf` shows the
   covariance snapshot and reads low in frozen mode — frozen actuates at full
   authority regardless.

### Advanced knobs (rarely needed — defaults are sensible)

| flag | default | what it does / when to touch |
|------|---------|------------------------------|
| `--ki` | `0.05` | Integral gain. Higher = the residual/backstop integral corrects faster but is twitchier. Leave unless steady-state convergence feels slow. |
| `--ff-lambda` | `0.999` | RLS forgetting factor; memory ≈ `1/(1-λ)` samples (~1000). Lower it (e.g. `0.99`) to adapt faster to a *changed* system (inverter swap) at the cost of more noise; raise toward `1` for a rock-steady plant. |
| `--ff-conf-scale-w` | `20` | The prediction-σ (W) at which the model is given 50 % authority. Lower = stricter (waits for more certainty before acting); higher = trusts the model sooner. |
| `--i-max` | `300` | Clamp on the integral term (W). |

All are optional overrides; none need setting for normal use.

**Memory.** The model lives entirely in RAM — nothing is written to the eMMC flash. It resets
if the process restarts, but that is harmless: the integral holds the grid on target the whole
time and the model re-converges within an hour or two. (The Venus GX itself rarely reboots; the
process restarts mainly when you deploy a new build.)

**Safety.** Everything the model does is bounded and still passes through the same hard clamps,
slew-rate limit, and sanity supervisor as every other command (see Safety). The learning layer
**cannot widen its own safety envelope** — those limits are derived from the inverter's design
power and are independent of anything learned. A corrupt or diverging estimate is caught
downstream and cannot leave the envelope.

## Safety model

- **Double-gated live:** needs both `--confirm` and env `VENUS_ESS_LIVE=I_UNDERSTAND`.
- **Envelope:** only discharges/idles within limits; below `--min-soc` (hysteresis) or on
  a force-charge state it hands control back to the stock firmware so charging / sustain /
  scheduling still work.
- **Clamps:** commands bounded by configured charge/discharge power and a hard sanity bound.
- **Watchdog:** in `Hub4Mode 3` the inverter reverts to passthru if no setpoint arrives
  within 60 s (official docs for older firmware say as low as 10 s; 60 s is the conservative
  upper bound), so any crash/kill → passthru within a minute (house on grid, battery idle).
  Clean exit / SIGINT / SIGTERM also restore `Hub4Mode`.
- **Flash-safe logging:** never logs per-tick to eMMC. Per-tick telemetry goes only to a
  bounded RAM (tmpfs) file and refuses flash paths; `--quiet` silences per-tick stdout.

## Implementation note — D-Bus transport

Talks to Venus D-Bus over a **native pure-Rust `zbus`** client — a single persistent
system-bus connection reused for every GetValue/SetValue, behind a small `Bus` trait.
`zbus` needs no `libdbus` C dependency, so the whole stack still cross-compiles to a
fully static musl binary (~1 MB). The `Bus` trait keeps the transport isolated from the
control logic.

## Tested against

Developed and validated on:

- **Venus OS v3.75** (kernel `6.12.90-venus-2`, `armv7l`) on a Venus GX.
- 2 × MultiPlus-II in an ESS, grid-metered.

Other Venus hardware/firmware and inverter counts should work — nothing is hardcoded to
the above (the leak and safety limits are learned/derived per system) — but this is the
only configuration it has actually run on. Treat anything else as untested.

## Status

Experimental but **live-validated**. On the reference system: five days of
shadow at watt-level fidelity to the stock loop (including scheduled-charge
windows and low-SOC dispatches), followed by supervised live takeover
sessions. Measured live: standing grid error ~+1 W vs the stock loop's +19 W
(the leak, fixed), appliance step-off export peaks at-or-shallower than the
stock loop's own class, zero charging outside charge regimes, clean
hand-backs. The full-adaptive learning mode remains the one experimental
corner (cold-start fragility, documented in the test suite) — use `frozen`
or `observe`. Run live sessions supervised and follow `docs/AB-PROTOCOL.md`.
