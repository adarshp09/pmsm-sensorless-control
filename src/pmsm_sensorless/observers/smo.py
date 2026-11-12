"""Shared sliding-mode observer machinery (paper Eqs. (3)-(7), pp.2-3).

This module holds everything the conventional and sigmoid observers share:
validated parameters, persistent state, the diagnostic output record, and the
discrete-time state update. The two concrete observers
(:mod:`~pmsm_sensorless.observers.conventional_smo`,
:mod:`~pmsm_sensorless.observers.sigmoid_smo`) differ only in the switching
function, so their comparison is fair by construction.

Paper equations implemented
---------------------------
- Sliding surface, Eq.(3), p.2::

      S(x) = i_hat_s - i_s            (per axis: S_alpha, S_beta)

- Current observer, Eq.(4), p.2 (written here component-wise with the paper's
  single ``Ld``)::

      d(i_hat_alpha)/dt = (u_alpha - Rs*i_hat_alpha - k*g(S_alpha)) / Ld
      d(i_hat_beta )/dt = (u_beta  - Rs*i_hat_beta  - k*g(S_beta )) / Ld

  where ``g`` is ``sign`` for the conventional observer and the sigmoid of
  Eq.(5) for the improved observer.

- Back-EMF estimate, Eq.(7), p.2::

      E_hat_alpha = k*g(S_alpha),   E_hat_beta = k*g(S_beta)

Current-error dynamics, Eq.(6), p.2, confirm the injection polarity: with
``i_bar = i_hat - i``,

.. math::

    L_d \\frac{d\\bar i}{dt} = -R_s \\bar i + E - k\\,g(S),

so a positive ``S`` produces a negative current correction and, at sliding,
``k*g(S)`` reconstructs the back-EMF ``E``.

Discrete-time convention
------------------------
The plant of Phase 01 has a solver the paper does not specify; here we adopt the
standard explicit **forward-Euler** update for the observer at the control period
``dt`` (the paper's ``Ts = 100 us``)::

    i_hat[k+1] = i_hat[k] + dt * f(u[k], i[k], i_hat[k])

The measurement ``i[k]``/``u[k]`` is held constant over the step (zero-order
hold), as in a digital implementation. ``step`` advances the persistent state and
returns the estimate/diagnostics at the *new* sample.

Units and the paper's per-unit frame
------------------------------------
Parameters are SI: ``stator_resistance`` [ohm], ``ld`` [H], ``switching_gain_k``
[V] and ``sigmoid_slope_a`` [1/A]. The current/voltage signals passed to ``step``
must be in the same system (SI recommended).

The paper reports ``k = 1`` and ``a = 4`` (p.5) but also states that the back-EMF
"is output in per unit, and the reference value is ``311/sqrt(3)``" (p.5). Those
values are therefore *per-unit* gains, not SI gains: ``k`` is a voltage gain and
``a`` scales the sliding surface (a current), so the SI equivalents are
``k_si = k_pu * V_base`` and ``a_si = a_pu / I_base``. :meth:`from_config`
applies exactly this conversion using bases derived from the validated machine
ratings; see :func:`per_unit_bases`. The current base is *derived*, not printed
by the paper, and is flagged in ``docs/implementation_decisions.md``.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from pmsm_sensorless.config import get_parameter, resolve_parameter

__all__ = [
    "SMOObserverOutput",
    "SMOObserverParameters",
    "SMOObserverState",
    "SlidingModeObserver",
    "per_unit_bases",
]


@dataclass(frozen=True)
class SMOObserverParameters:
    """Validated sliding-mode observer parameters (SI units).

    Attributes:
        stator_resistance: Stator resistance ``Rs`` [ohm]; must match the plant.
        ld: d/q stator inductance ``Ld`` [H]; must match the plant (``Ld = Lq``).
        switching_gain_k: Switching gain ``k`` [V] (Eq.(4)-(7)).
        sigmoid_slope_a: Sigmoid slope ``a`` [1/A] (Eq.(5)); required only by the
            sigmoid observer, else ``None``.
    """

    stator_resistance: float
    ld: float
    switching_gain_k: float
    sigmoid_slope_a: float | None = None

    def __post_init__(self) -> None:
        if self.stator_resistance <= 0.0:
            raise ValueError(f"stator_resistance must be positive, got {self.stator_resistance}")
        if self.ld <= 0.0:
            raise ValueError(f"ld must be positive, got {self.ld}")
        if self.switching_gain_k <= 0.0:
            raise ValueError(f"switching_gain_k must be positive, got {self.switching_gain_k}")
        if self.sigmoid_slope_a is not None and self.sigmoid_slope_a <= 0.0:
            raise ValueError(f"sigmoid_slope_a must be positive, got {self.sigmoid_slope_a}")

    @classmethod
    def from_config(
        cls,
        pmsm_config: dict[str, Any],
        observer_config: dict[str, Any],
        *,
        per_unit_scaling: bool = True,
    ) -> "SMOObserverParameters":
        """Build observer parameters from validated configuration mappings.

        Reads ``Rs``/``Ld`` from ``pmsm_config`` and ``k``/``a`` from
        ``observer_config``. Because the paper's ``k``/``a`` are per-unit (p.5),
        by default they are converted to SI using :func:`per_unit_bases`; pass
        ``per_unit_scaling=False`` to keep the raw values.

        Args:
            pmsm_config: Loaded ``pmsm.yaml`` mapping (machine + rating sections).
            observer_config: Loaded ``observer.yaml`` mapping.
            per_unit_scaling: Convert the paper's per-unit ``k``/``a`` to SI.

        Raises:
            ConfigError: If a required value is absent, unresolved, or unit-mismatched.
        """
        stator_resistance = resolve_parameter(
            pmsm_config, "machine.stator_resistance_rs", unit="ohm"
        )
        ld = resolve_parameter(pmsm_config, "machine.stator_inductance_ld", unit="H")
        k_raw = resolve_parameter(observer_config, "smo.switching_gain_k", unit="-")
        a_raw = get_parameter(observer_config, "smo.sigmoid_slope_a", unit="-")

        if per_unit_scaling:
            voltage_base, current_base = per_unit_bases(pmsm_config)
            k = k_raw * voltage_base
            a = None if a_raw is None else a_raw / current_base
        else:
            k, a = k_raw, a_raw

        return cls(
            stator_resistance=stator_resistance,
            ld=ld,
            switching_gain_k=k,
            sigmoid_slope_a=a,
        )


def per_unit_bases(pmsm_config: dict[str, Any]) -> tuple[float, float]:
    """Return the paper's peak per-unit bases ``(V_base, I_base)``.

    The paper states the back-EMF reference is ``311/sqrt(3)`` V (p.5), i.e. the
    peak phase voltage ``sqrt(2)*V_ll_rms/sqrt(3)``. The matching peak phase
    current base ``sqrt(2)*P/(sqrt(3)*V_ll_rms)`` is derived from Table III
    (rated voltage 220 V line, rated power 2 kW); it is a documented derivation,
    not a paper-printed number.

    Raises:
        ConfigError: If ``rating.rated_voltage`` or ``rating.rated_power`` is
            absent or unresolved.
    """
    v_ll_rms = resolve_parameter(pmsm_config, "rating.rated_voltage", unit="V")
    power = resolve_parameter(pmsm_config, "rating.rated_power", unit="W")
    voltage_base = math.sqrt(2.0) * v_ll_rms / math.sqrt(3.0)
    current_base = math.sqrt(2.0) * power / (math.sqrt(3.0) * v_ll_rms)
    return voltage_base, current_base


@dataclass
class SMOObserverState:
    """Persistent observer state: the estimated stator current in alpha-beta.

    Attributes:
        i_alpha_hat, i_beta_hat: Estimated currents [A].
    """

    i_alpha_hat: float
    i_beta_hat: float


@dataclass(frozen=True)
class SMOObserverOutput:
    """Diagnostics for one observer update.

    Attributes:
        i_alpha_hat, i_beta_hat: Estimated currents [A] **after** the update.
        s_alpha, s_beta: Sliding-surface errors ``i_hat - i`` [A] at the sample
            instant (Eq.(3)), i.e. the values used to form the applied injection.
        back_emf_alpha, back_emf_beta: Back-EMF estimates ``k*g(S)`` [V] (Eq.(7))
            from those sample-instant errors; they are the injection actually
            applied in Eq.(4) (times ``-Ld``), not a post-update recomputation.
    """

    i_alpha_hat: float
    i_beta_hat: float
    s_alpha: float
    s_beta: float
    back_emf_alpha: float
    back_emf_beta: float


class SlidingModeObserver(ABC):
    """Base class for the paper's current-observer (Eqs.(3)-(7)).

    Subclasses implement :meth:`_switching` only. The observer consumes measured
    ``alpha-beta`` voltages and currents and never the plant's true rotor angle,
    speed or back-EMF.

    Args:
        parameters: Validated SI parameters.
        initial_state: Optional initial estimate; defaults to zero current.
    """

    def __init__(
        self,
        parameters: SMOObserverParameters,
        initial_state: SMOObserverState | None = None,
    ) -> None:
        self.parameters = parameters
        self._state = initial_state if initial_state is not None else SMOObserverState(0.0, 0.0)

    @property
    def state(self) -> SMOObserverState:
        """The persistent observer state (a copy is not returned; treat as read-only)."""
        return self._state

    def reset(self, state: SMOObserverState | None = None) -> None:
        """Reset the persistent estimate to ``state`` (or zero current)."""
        self._state = state if state is not None else SMOObserverState(0.0, 0.0)

    @abstractmethod
    def _switching(self, s: float) -> float:
        """Switching function ``g(s)``: ``sign`` (Eq.(4)) or sigmoid (Eq.(5))."""

    def step(
        self,
        u_alpha: float,
        u_beta: float,
        i_alpha: float,
        i_beta: float,
        dt: float,
    ) -> SMOObserverOutput:
        """Advance the observer by one forward-Euler step and return diagnostics.

        Args:
            u_alpha, u_beta: Measured stator voltages [V].
            i_alpha, i_beta: Measured stator currents [A].
            dt: Timestep [s]; must be positive.

        Returns:
            :class:`SMOObserverOutput` with the sample-instant sliding surface and
            back-EMF (the applied injection), and the current estimate **after** the
            update.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0.0:
            raise ValueError(f"timestep dt must be positive, got {dt}")

        parameters = self.parameters
        gain_per_ld = parameters.switching_gain_k / parameters.ld
        s_alpha = self._state.i_alpha_hat - i_alpha
        s_beta = self._state.i_beta_hat - i_beta
        # Switching at the sample instant, i.e. the injection actually applied in
        # Eq.(4). Reporting g(i_hat_new - i) instead corrupts the signal for a
        # discontinuous switching function (see D-025).
        switching_alpha = self._switching(s_alpha)
        switching_beta = self._switching(s_beta)

        d_alpha = (u_alpha - parameters.stator_resistance * self._state.i_alpha_hat) / parameters.ld
        d_beta = (u_beta - parameters.stator_resistance * self._state.i_beta_hat) / parameters.ld
        d_alpha -= gain_per_ld * switching_alpha
        d_beta -= gain_per_ld * switching_beta

        self._state = SMOObserverState(
            i_alpha_hat=self._state.i_alpha_hat + dt * d_alpha,
            i_beta_hat=self._state.i_beta_hat + dt * d_beta,
        )

        return SMOObserverOutput(
            i_alpha_hat=self._state.i_alpha_hat,
            i_beta_hat=self._state.i_beta_hat,
            s_alpha=s_alpha,
            s_beta=s_beta,
            back_emf_alpha=parameters.switching_gain_k * switching_alpha,
            back_emf_beta=parameters.switching_gain_k * switching_beta,
        )
