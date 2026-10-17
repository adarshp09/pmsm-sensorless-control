"""Sigmoid-switching sliding-mode observer (the paper's improved SMO).

Same observer structure as :mod:`pmsm_sensorless.observers.conventional_smo`; the
only change is that the discontinuous ``sign`` of Eq.(4) is replaced by the
sigmoid of Eq.(5), p.2:

.. math::

    g(s, a) = \\frac{2}{1 + e^{-a s}} - 1 = \\tanh\\!\\left(\\frac{a s}{2}\\right)

The tanh form is used because it is mathematically identical to the paper's
expression yet numerically stable for large ``a*s`` (no overflow in ``exp``).
This is the paper's *fixed*-parameter sigmoid observer: ``k`` and ``a`` are held
constant. The fuzzy adaptation of ``k`` and ``a`` (Eqs. of the FLC, pp.3-4) is a
later phase and is deliberately not implemented here.

Assumptions: per :mod:`pmsm_sensorless.observers.smo`, ``k`` is in volts and ``a``
in inverse amperes. The paper's ``a = 4`` (p.5) is a per-unit slope; its SI value
depends on the current per-unit base, which the paper does not report (see
``docs/implementation_decisions.md``).
"""

from __future__ import annotations

import math
from typing import Any

from pmsm_sensorless.observers.smo import (
    SMOObserverParameters,
    SMOObserverState,
    SlidingModeObserver,
)

__all__ = ["SigmoidSMO", "sigmoid_switching"]


def sigmoid_switching(s: float, a: float) -> float:
    """Return the paper's sigmoid ``2/(1+exp(-a*s)) - 1`` (Eq.(5), p.2).

    Implemented as ``tanh(a*s/2)``, which is algebraically identical and free of
    overflow for large ``|a*s|``.

    Args:
        s: Sliding-surface value (any real number).
        a: Positive slope parameter.

    Returns:
        A value in ``[-1, 1]``, monotone in ``s`` and odd in ``s``.
    """
    return math.tanh(0.5 * a * s)


class SigmoidSMO(SlidingModeObserver):
    """SMO using the sigmoid switching function (Eq.(5), p.2).

    Args:
        parameters: Validated SI parameters; ``sigmoid_slope_a`` must be set.
        initial_state: Optional initial estimate; defaults to zero current.

    Raises:
        ValueError: If ``parameters.sigmoid_slope_a`` is ``None`` (the slope is a
            required parameter and is never defaulted).
    """

    def __init__(
        self,
        parameters: SMOObserverParameters,
        initial_state: SMOObserverState | None = None,
    ) -> None:
        if parameters.sigmoid_slope_a is None:
            raise ValueError(
                "SigmoidSMO requires a resolved sigmoid slope a "
                "(smo.sigmoid_slope_a); none was supplied"
            )
        super().__init__(parameters, initial_state)

    @classmethod
    def from_config(
        cls,
        pmsm_config: dict[str, Any],
        observer_config: dict[str, Any],
        *,
        per_unit_scaling: bool = True,
    ) -> "SigmoidSMO":
        """Build the observer from validated configuration mappings."""
        return cls(
            SMOObserverParameters.from_config(
                pmsm_config, observer_config, per_unit_scaling=per_unit_scaling
            )
        )

    def _switching(self, s: float) -> float:
        slope = self.parameters.sigmoid_slope_a
        assert slope is not None  # guaranteed by __init__
        return sigmoid_switching(s, slope)
