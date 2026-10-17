"""Conventional sliding-mode observer with a ``sign`` switching function.

This is the paper's baseline observer, Eqs.(3)-(4), p.2, using the discontinuous
sign function *before* the sigmoid replacement of Eq.(5). The shared dynamics,
state, output record and forward-Euler convention live in
:mod:`pmsm_sensorless.observers.smo`; only the switching function is defined here.

.. math::

    \\operatorname{sign}(s) = \\begin{cases} +1 & s > 0 \\\\ 0 & s = 0 \\\\ -1 & s < 0 \\end{cases}

The back-EMF estimate of Eq.(7) is therefore the bang-bang signal
``E_hat = k*sign(S)``; its average over a switching cycle reconstructs the
back-EMF but the instantaneous signal chatters (the phenomenon the paper sets out
to reduce). No low-pass filter is applied: the paper gives no cutoff frequency
(``smo.observed_emf_lpf_cutoff`` is unresolved), so filtering is out of scope.
"""

from __future__ import annotations

from typing import Any

from pmsm_sensorless.observers.smo import (
    SMOObserverParameters,
    SMOObserverState,
    SlidingModeObserver,
)

__all__ = ["ConventionalSMO", "sign_switching"]


def sign_switching(s: float) -> float:
    """Return ``sign(s)`` with the convention ``sign(0) = 0`` (Eq.(4), p.2)."""
    if s > 0.0:
        return 1.0
    if s < 0.0:
        return -1.0
    return 0.0


class ConventionalSMO(SlidingModeObserver):
    """Conventional SMO using the sign switching function (Eqs.(3)-(4), p.2).

    Args:
        parameters: Validated SI parameters.
        initial_state: Optional initial estimate; defaults to zero current.
    """

    def __init__(
        self,
        parameters: SMOObserverParameters,
        initial_state: SMOObserverState | None = None,
    ) -> None:
        super().__init__(parameters, initial_state)

    @classmethod
    def from_config(
        cls,
        pmsm_config: dict[str, Any],
        observer_config: dict[str, Any],
        *,
        per_unit_scaling: bool = True,
    ) -> "ConventionalSMO":
        """Build the observer from validated configuration mappings."""
        return cls(
            SMOObserverParameters.from_config(
                pmsm_config, observer_config, per_unit_scaling=per_unit_scaling
            )
        )

    def _switching(self, s: float) -> float:
        return sign_switching(s)
