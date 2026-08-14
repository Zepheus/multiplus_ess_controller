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

This controller takes the setpoint loop (`Hub4Mode` 1 → 3) and runs a loop **with
integral action**, driving the grid meter onto the setpoint regardless of the
inverter's few-percent deficit. It only ever discharges/idles within limits and hands
control back to the stock firmware for anything outside a safe envelope (see Safety).

Two parts:

- **`/` (Rust crate)** — the on-device controller. Static ARM binary, no runtime deps.
- **`simulator/` (Python)** — an offline sandbox with a validated plant model to
  experiment with control laws (stock / integral / PI) before running anything live.

## Build (cross-compile from macOS/Linux to the GX)

```bash
rustup target add armv7-unknown-linux-musleabihf
# macOS: brew install messense/macos-cross-toolchains/armv7-unknown-linux-musleabihf
cargo build --release   # -> target/armv7-unknown-linux-musleabihf/release/multiplus_ess_controller… (static)
```

Produces a ~400 KB fully static musl binary that runs on any Venus OS regardless of glibc.

## Run

```bash
# shadow (default, READ-ONLY): compute & log what it WOULD command, never write
./venus-ess-controller --mode shadow --controller stock  --seconds 30
./venus-ess-controller --mode shadow --controller pi     --telemetry /run/ess.csv --quiet

# live (double-gated): takes Hub4Mode=3 within the safety envelope, restores on exit
VENUS_ESS_LIVE=I_UNDERSTAND ./venus-ess-controller --mode live --confirm \
    --controller pi --seconds 900 --min-soc 20
```

Controllers: `stock` (reference), `true-integral`, `pi` (default).

## Safety model

- **Double-gated live:** needs both `--confirm` and env `VENUS_ESS_LIVE=I_UNDERSTAND`.
- **Envelope:** only discharges/idles within limits; below `--min-soc` (hysteresis) or on
  a force-charge state it hands control back to the stock firmware so charging / sustain /
  scheduling still work.
- **Clamps:** commands bounded by configured charge/discharge power and a hard sanity bound.
- **Watchdog:** in `Hub4Mode 3` the inverter reverts to passthru if no setpoint arrives
  within 60 s, so any crash/kill → passthru within a minute (house on grid, battery idle).
  Clean exit / SIGINT / SIGTERM also restore `Hub4Mode`.
- **Flash-safe logging:** never logs per-tick to eMMC. Per-tick telemetry goes only to a
  bounded RAM (tmpfs) file and refuses flash paths; `--quiet` silences per-tick stdout.

## Implementation note — D-Bus transport

Talks to Venus D-Bus over a **native pure-Rust `zbus`** client — a single persistent
system-bus connection reused for every GetValue/SetValue, behind a small `Bus` trait.
`zbus` needs no `libdbus` C dependency, so the whole stack still cross-compiles to a
fully static musl binary (~1 MB). The `Bus` trait keeps the transport isolated from the
control logic.

## Status

Experimental. Built and validated in **shadow mode** on real hardware; the stock law
reproduces the device's actual commands within a few watts. **Not run live** in this
initial cut — the mode-1→3 cutover is intended to be done supervised.
