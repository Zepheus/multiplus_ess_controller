"""
Controllers for the ESS AC-in loop.  THIS is the file to experiment in.

Every controller implements:

    update(grid, reported, target, dt, lo, hi) -> command

    grid     : measured grid power (W, +import)
    reported : Multi's reported AC-in power (W, neg=discharge)
    target   : grid setpoint (W), e.g. +20
    dt       : seconds since last tick
    lo, hi   : command clamp (W) = (-MaxDischargePower, +MaxChargePower)
    returns  : new AC-in power command (W, neg=discharge)

Two families:
  * "law" controllers output the Multi command directly (mode-3 style; they REPLACE
    the stock ESS loop).  Stock, TrueIntegral, PI, PID.
  * "trim" controllers output an adjusted *target* to feed the stock law (mode-2 style;
    they run ALONGSIDE the stock ESS loop, only nudging Settings/CGwacs/AcPowerSetPoint).
    Wrap them with StockLaw for closed-loop simulation.  See TargetTrim.
"""

from dataclasses import dataclass, field


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# --------------------------------------------------------------------------- #
# LAW controllers  (output the Multi command directly)
# --------------------------------------------------------------------------- #

class StockLaw:
    """Model of the stock ESS loop's observed behaviour.

        command = target - grid + reported

    One-step feed-forward keyed on the Multi's *reported* achieved power.
    No integral state -> steady-state grid = (1-k)*load + k*target  (the leak).
    Used in shadow mode to check our model matches the stock loop's commands.
    """
    name = "stock"

    def reset(self):
        pass

    def update(self, grid, reported, target, dt, lo, hi):
        return _clamp(target - grid + reported, lo, hi)


@dataclass
class TrueIntegral:
    """The one-line fix: key the correction on the PREVIOUS COMMAND, not on the
    Multi's reported power, so the firmware deficit is inside the loop.

        command += -ki * (grid - target)

    ki=1.0 is the natural "deadbeat" gain analogous to the stock structure and
    drives grid -> target regardless of k.  Lower ki = gentler, more robust.
    """
    ki: float = 1.0
    name: str = "true-integral"
    _cmd: float = None

    def reset(self):
        self._cmd = None

    def update(self, grid, reported, target, dt, lo, hi):
        if self._cmd is None:
            self._cmd = target - grid + reported   # warm-start at the stock command
        self._cmd = _clamp(self._cmd - self.ki * (grid - target), lo, hi)
        return self._cmd


@dataclass
class PI:
    """Feed-forward stock law PLUS a slow integral trim on the residual.

        command = (target - grid + reported)          # keeps stock's fast load rejection
                + I ;  I += ki * (target - grid) * dt  # kills the steady leak

    This is the best of both: instant response to load steps (feed-forward) and
    zero steady-state error (integral).  kp adds optional proportional action.
    Anti-windup clamps I to +/- i_max.
    """
    ki: float = 0.05          # W of trim per (W of error * s)
    kp: float = 0.0
    i_max: float = 300.0      # anti-windup bound on the integral term (W)
    name: str = "pi"
    _i: float = 0.0

    def reset(self):
        self._i = 0.0

    def update(self, grid, reported, target, dt, lo, hi):
        err = target - grid
        self._i = _clamp(self._i + self.ki * err * dt, -self.i_max, self.i_max)
        ff = target - grid + reported
        return _clamp(ff + self.kp * err + self._i, lo, hi)


@dataclass
class PID:
    """Direct PID on grid error producing the command (no feed-forward).

        command = kp*err + ki*∫err + kd*d(err)/dt ,  err = target - grid
    """
    kp: float = 0.5
    ki: float = 0.1
    kd: float = 0.0
    i_max: float = 2000.0
    name: str = "pid"
    _i: float = 0.0
    _prev: float = None

    def reset(self):
        self._i = 0.0
        self._prev = None

    def update(self, grid, reported, target, dt, lo, hi):
        err = target - grid
        self._i = _clamp(self._i + err * dt, -self.i_max, self.i_max)
        d = 0.0 if (self._prev is None or dt <= 0) else (err - self._prev) / dt
        self._prev = err
        return _clamp(self.kp * err + self.ki * self._i + self.kd * d, lo, hi)


# --------------------------------------------------------------------------- #
# TRIM controllers  (mode-2: output an adjusted TARGET, stock law stays in place)
# --------------------------------------------------------------------------- #

@dataclass
class TargetTrim:
    """The deployable, low-risk add-on.  Does NOT replace the stock ESS loop; instead it
    slowly slides the grid setpoint SETTING so the stock law lands grid on target.

        target_eff = target + I
        I += ki * (target - grid_avg) * dt      # slow integral, minutes time-constant

    In simulation we compose it with the real StockLaw as the inner plant-facing
    controller.  On-device, `I` is applied by writing Settings/CGwacs/AcPowerSetPoint
    = target + I (a single, bounded, watchdog-free setting write).

    Emits `.setting(target)` = what to write on-device; `.update(...)` = closed-loop
    command for simulation.
    """
    ki: float = 0.02
    i_max: float = 200.0       # clamp on the trim, matches the safe setpoint window
    ema_tau: float = 20.0      # s; smooth grid before integrating (reject transients)
    name: str = "target-trim"
    _inner: StockLaw = field(default_factory=StockLaw)
    _i: float = 0.0
    _grid_ema: float = None

    def reset(self):
        self._i = 0.0
        self._grid_ema = None
        self._inner.reset()

    def _advance_integral(self, grid, target, dt):
        if self._grid_ema is None:
            self._grid_ema = grid
        a = dt / (self.ema_tau + dt)
        self._grid_ema += a * (grid - self._grid_ema)
        self._i = _clamp(self._i + self.ki * (target - self._grid_ema) * dt,
                         -self.i_max, self.i_max)

    def setting(self, target):
        """The value to write to Settings/CGwacs/AcPowerSetPoint on-device."""
        return target + self._i

    def update(self, grid, reported, target, dt, lo, hi):
        self._advance_integral(grid, target, dt)
        return self._inner.update(grid, reported, self.setting(target), dt, lo, hi)


# Registry for the CLI (--controller name).  Edit freely.
REGISTRY = {
    "stock":         lambda: StockLaw(),
    "true-integral": lambda: TrueIntegral(ki=1.0),
    "pi":            lambda: PI(ki=0.05, i_max=300.0),
    "pid":           lambda: PID(kp=0.5, ki=0.1),
    "target-trim":   lambda: TargetTrim(ki=0.02, ema_tau=20.0, i_max=200.0),
}
