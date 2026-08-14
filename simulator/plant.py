"""
Plant model for the Victron ESS AC-in power loop.

Empirically modelled from the stock ESS loop's observed input/output behaviour and
*validated against 1 Hz measurements* to the watt:

    reported = k * command                 # Multi under-achieves its command
    grid     = load + reported             # KCL at the AC-in bus

Sign conventions (all watts, all AC-in referred):
    command  : commanded Multi AC-in power. NEGATIVE = discharge (Multi injects),
               POSITIVE = charge (Multi draws from the bus).
    reported : Multi's own /Ac/ActiveIn/L1/P. Same sign convention as command.
    grid     : grid-meter power. POSITIVE = import, NEGATIVE = export.
    load     : house consumption on the AC-in bus. POSITIVE = consuming.
    target   : the grid setpoint setting (Settings/CGwacs/AcPowerSetPoint), +20 W.

Validation (capture operating point): load=1700, target=20, k=0.9625
    stock command = target - grid + reported = 20 - 83 + (-1617) = -1680  (measured -1680)
    grid_ss       = (1-k)*load + k*target    = 0.0375*1700 + 0.9625*20 = 83.0 (measured 83.2)

The (1-k)*load term is exactly the observed "~4% of load" leak.
"""

from dataclasses import dataclass


# Deficit factor measured in the discharge regime of a reference capture.
# reported/command = -1617/-1680 = 0.9625  ->  ~3.75% shortfall.
K_DEFICIT = 0.9625

# Hard slew limit baked into VE.Bus firmware (Victron docs: ~400 W/s).
SLEW_W_PER_S = 400.0


@dataclass
class Plant:
    """One-phase model of the two paralleled Multis + AC-in bus + grid meter.

    Stateful because of the firmware slew limit: the achieved command can only
    move SLEW_W_PER_S each second toward the requested command.
    """

    k: float = K_DEFICIT
    slew: float = SLEW_W_PER_S
    # First-order lag (s) of achieved power toward the (slewed) command. 0 = instant.
    tau: float = 0.0
    # Multiplicative + additive measurement noise on the grid meter (ET112 ~1%).
    grid_noise_frac: float = 0.0
    grid_noise_w: float = 0.0

    _cmd_actual: float = 0.0   # slew-limited command actually in effect
    _reported: float = 0.0     # achieved AC-in power (what the Multi reports)

    def reset(self, command: float = 0.0) -> None:
        self._cmd_actual = command
        self._reported = self.k * command

    def step(self, command: float, load: float, dt: float, rng=None):
        """Advance one tick. Returns (grid, reported).

        command : what the controller wants (W, AC-in, neg=discharge)
        load    : house load this tick (W, positive)
        dt      : seconds since last tick
        rng     : optional random.Random for meter noise
        """
        # 1) firmware slew limit on how fast the command can move
        max_step = self.slew * dt
        delta = command - self._cmd_actual
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self._cmd_actual += delta

        # 2) the Multi under-achieves the (slewed) command by factor k
        target_reported = self.k * self._cmd_actual
        if self.tau > 0.0:
            a = dt / (self.tau + dt)
            self._reported += a * (target_reported - self._reported)
        else:
            self._reported = target_reported

        # 3) node balance sets the grid meter reading
        grid = load + self._reported

        # 4) optional grid-meter noise
        if rng is not None and (self.grid_noise_frac or self.grid_noise_w):
            grid += grid * self.grid_noise_frac * (2 * rng.random() - 1)
            grid += self.grid_noise_w * (2 * rng.random() - 1)

        return grid, self._reported


def steady_state_grid(controller_kind: str, load: float, target: float,
                      k: float = K_DEFICIT) -> float:
    """Closed-form steady-state grid power, for cross-checking the simulator.

    'stock'    : the stock ESS loop as-is        -> (1-k)*load + k*target   (leaks)
    'integral' : any loop with true I-action on (grid-target) -> target exactly
    """
    if controller_kind == "stock":
        return (1.0 - k) * load + k * target
    if controller_kind == "integral":
        return target
    raise ValueError(controller_kind)
