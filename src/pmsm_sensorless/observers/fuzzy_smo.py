"""Fuzzy-adaptive sigmoid SMO (paper Section III-A, pp.3-4).

Same observer structure as :class:`~pmsm_sensorless.observers.sigmoid_smo.SigmoidSMO`,
but the switching gain and sigmoid slope are adapted **per step and per axis**:

.. math::

    k_t = k_F \\cdot k_0, \\qquad a_t = a_F \\cdot a_0

where ``(k_F, a_F)`` come from the FLC (``fuzzy.engine``) and ``k_0``/``a_0`` are
the fixed values from D-015 (already converted to SI in ``parameters``). The
reported back-EMF is the injection actually applied, ``k_t * g(S_pre; a_t)``,
consistent with D-025.

The FLC sees only the observer's own current error ``S = i_hat - i`` and its
backward difference; no plant/ground-truth signals are used. The existing
``SigmoidSMO``/``ConventionalSMO`` behaviour is untouched (this is a subclass).

Assumptions (see ``configs/fuzzy_assumptions.yaml``): ``Ke``/``Kec`` normalisation,
``s_dot`` by backward difference (zero on the first step), one independent FLC per
axis, and the Mamdani operators/CoG grid.
"""

from __future__ import annotations

import math
from typing import Any

from pmsm_sensorless.fuzzy.engine import FuzzyEngine
from pmsm_sensorless.observers.smo import (
    SMOObserverOutput,
    SMOObserverParameters,
    SMOObserverState,
    SlidingModeObserver,
)

__all__ = ["FuzzySMO"]


class FuzzySMO(SlidingModeObserver):
    """Sigmoid SMO with per-step, per-axis FLC adaptation of ``k`` and ``a``.

    Args:
        parameters: Validated SI parameters; ``sigmoid_slope_a`` must be set (it is
            ``a_0``). ``switching_gain_k`` is ``k_0``.
        engine: A fuzzy engine exposing ``evaluate(s_n, s_dot_n) -> (kF, aF)`` and
            ``parameters.ke`` / ``parameters.kec``.
        initial_state: Optional initial estimate; defaults to zero current.

    Raises:
        ValueError: If ``parameters.sigmoid_slope_a`` is ``None``.
    """

    def __init__(
        self,
        parameters: SMOObserverParameters,
        engine: FuzzyEngine,
        initial_state: SMOObserverState | None = None,
    ) -> None:
        if parameters.sigmoid_slope_a is None:
            raise ValueError("FuzzySMO requires a resolved sigmoid slope a0")
        super().__init__(parameters, initial_state)
        self.engine = engine
        self._previous_s: tuple[float, float] | None = None
        self.last_kf: tuple[float, float] | None = None
        self.last_af: tuple[float, float] | None = None
        self.last_k: tuple[float, float] | None = None
        self.last_a: tuple[float, float] | None = None

    @classmethod
    def from_config(
        cls,
        pmsm_config: dict[str, Any],
        observer_config: dict[str, Any],
        *,
        per_unit_scaling: bool = True,
        fuzzy_directory: Any = None,
    ) -> "FuzzySMO":
        """Build from the validated configs and the FLC YAML files."""
        return cls(
            SMOObserverParameters.from_config(
                pmsm_config, observer_config, per_unit_scaling=per_unit_scaling
            ),
            FuzzyEngine.from_yaml(fuzzy_directory),
        )

    def reset(self, state: SMOObserverState | None = None) -> None:
        """Reset the estimate, the ``s_dot`` memory and the last-gain diagnostics."""
        super().reset(state)
        self._previous_s = None
        self.last_kf = self.last_af = self.last_k = self.last_a = None

    def _switching(self, s: float) -> float:
        # Satisfies the abstract base; the adaptive step overrides it per axis.
        slope = self.parameters.sigmoid_slope_a
        assert slope is not None
        return math.tanh(0.5 * slope * s)

    def step(
        self,
        u_alpha: float,
        u_beta: float,
        i_alpha: float,
        i_beta: float,
        dt: float,
    ) -> SMOObserverOutput:
        """Advance one step with FLC-adapted ``k`` and ``a``.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0.0:
            raise ValueError(f"timestep dt must be positive, got {dt}")

        parameters = self.parameters
        k0 = parameters.switching_gain_k
        a0 = parameters.sigmoid_slope_a
        assert a0 is not None
        s_alpha = self._state.i_alpha_hat - i_alpha
        s_beta = self._state.i_beta_hat - i_beta
        if self._previous_s is None:
            s_dot_alpha = s_dot_beta = 0.0
        else:
            s_dot_alpha = (s_alpha - self._previous_s[0]) / dt
            s_dot_beta = (s_beta - self._previous_s[1]) / dt

        ke, kec = self.engine.parameters.ke, self.engine.parameters.kec
        kf_alpha, af_alpha = self.engine.evaluate(ke * s_alpha, kec * s_dot_alpha)
        kf_beta, af_beta = self.engine.evaluate(ke * s_beta, kec * s_dot_beta)
        k_alpha, k_beta = kf_alpha * k0, kf_beta * k0
        a_alpha, a_beta = af_alpha * a0, af_beta * a0

        switching_alpha = math.tanh(0.5 * a_alpha * s_alpha)
        switching_beta = math.tanh(0.5 * a_beta * s_beta)
        d_alpha = (u_alpha - parameters.stator_resistance * self._state.i_alpha_hat) / parameters.ld
        d_beta = (u_beta - parameters.stator_resistance * self._state.i_beta_hat) / parameters.ld
        d_alpha -= (k_alpha / parameters.ld) * switching_alpha
        d_beta -= (k_beta / parameters.ld) * switching_beta

        self._state = SMOObserverState(
            i_alpha_hat=self._state.i_alpha_hat + dt * d_alpha,
            i_beta_hat=self._state.i_beta_hat + dt * d_beta,
        )
        self._previous_s = (s_alpha, s_beta)
        self.last_kf = (kf_alpha, kf_beta)
        self.last_af = (af_alpha, af_beta)
        self.last_k = (k_alpha, k_beta)
        self.last_a = (a_alpha, a_beta)

        return SMOObserverOutput(
            i_alpha_hat=self._state.i_alpha_hat,
            i_beta_hat=self._state.i_beta_hat,
            s_alpha=s_alpha,
            s_beta=s_beta,
            back_emf_alpha=k_alpha * switching_alpha,
            back_emf_beta=k_beta * switching_beta,
        )
