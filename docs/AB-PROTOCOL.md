# Live A/B protocol — pre-registered, no eyeball verdicts

Adopted 2026-08-22 after the first takeover day: aggregate medians and live
impressions both misled (in opposite directions). Every future live session
follows this protocol; verdicts come from the scoreboard, not from anyone's
recollection — the operator's or the assistant's.

## Before the session (pre-registration)

1. ONE variable changes per session. Name it in the session note.
2. Predict the outcome from the replay harness FIRST: run the candidate over
   the event library (`tools/event_scoreboard.py score …`) and the postdiction
   gates (`cargo test postdiction`). No green gates -> no deploy.
3. Write down the abort criteria before starting. Defaults:
   - any single grid excursion beyond ±5.5 kW (M_WORST_TRANSIENT), or
   - any battery charging outside charge regimes (rep > +150 W with fc=0), or
   - sanity trip / unexpected hand-back, or
   - export-duration per event visibly exceeding the stock baseline class
     (> 20 s beyond -300 W) on two events.
   Abort = `svc -t` back to the shadow exec (stock authoritative in <30 s).

## During

- Session telemetry stays on (`/run/venus-ess.csv`); no config edits mid-run.
- Minimum useful session: 2 h or 10 transient events, whichever first.

## After (the verdict)

1. Pull the CSV, then: `tools/event_scoreboard.py ab <session.csv> <baseline>`
   — per-event export Wh / import Wh / peak / DURATION vs both the modeled
   stock and the real stock baseline events.
2. The candidate wins only if it is >= stock on export AND duration AND import
   per-event distributions. "Median improved" alone is not a win.
3. Archive the session CSV + scoreboard output into `captures/` with a line in
   its README. Update the event library if new event classes appeared.

## Standing reference numbers (233-event library, identified plant)

| config                    | export Wh | import Wh | duration s |
|---------------------------|-----------|-----------|------------|
| stock (model, lower bound)| 702       | 851       | 1990       |
| ki.02 / 0.242 (reverted)  | 777       | 713       | 2071       |
| smith + 0.5 (candidate)   | 623       | 598       | 1686       |
| any + fast meter (0 lag)  | 504       | 483       | 1458       |

The fast-meter row is the EM540/VM-3P75CT ceiling: Smith on the ET112 already
captures ~60 % of it in software.
