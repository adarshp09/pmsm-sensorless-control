"""Small-signal discrete stability of the sigmoid observer's linear region.

With ``g(S) = tanh(aS/2) ~ (a/2)S`` for small ``S``, the current-error dynamics of
Eq.(6) linearise to ``Ld dS/dt = -(Rs + k a/2) S + E``. The forward-Euler update
(D-016) is ``S[k+1] = (1 - dt*lambda) S[k]`` with ``lambda = (Rs + k a/2)/Ld``, so
it is stable **iff** ``dt*(Rs + k a/2)/Ld < 2`` (threshold ``Ts* = 2 Ld/(Rs + k a/2)``).
"""

from __future__ import annotations

import pytest

from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO
from pmsm_sensorless.observers.smo import SMOObserverParameters, SMOObserverState

RS, LD, K, A = 0.5, 0.01, 10.0, 2.0
LAMBDA = (RS + K * A / 2.0) / LD


def _one_step_ratio(dt: float, i0: float = 1e-6) -> float:
    """``S[k+1]/S[k]`` for a tiny perturbation (linear region; E=0, i=0, u=0)."""
    observer = SigmoidSMO(
        SMOObserverParameters(RS, LD, K, sigmoid_slope_a=A), SMOObserverState(i0, i0)
    )
    out = observer.step(0.0, 0.0, 0.0, 0.0, dt)
    return out.i_alpha_hat / i0


@pytest.mark.parametrize("dt", [1e-4, 5e-4, 1e-3])
def test_linear_region_decay_is_one_minus_dt_lambda(dt: float) -> None:
    assert _one_step_ratio(dt) == pytest.approx(1.0 - dt * LAMBDA, rel=1e-3)


def test_stability_boundary_is_dt_lambda_equal_two() -> None:
    stable_dt = 1.9 / LAMBDA
    unstable_dt = 2.1 / LAMBDA
    assert abs(_one_step_ratio(stable_dt)) < 1.0  # decays
    assert abs(_one_step_ratio(unstable_dt)) > 1.0  # grows -> period-2 limit cycle
    # at exactly the boundary the magnitude is ~1 (marginal)
    assert abs(_one_step_ratio(2.0 / LAMBDA)) == pytest.approx(1.0, abs=1e-6)


def test_threshold_formula() -> None:
    assert 2.0 / LAMBDA == pytest.approx(2.0 * LD / (RS + K * A / 2.0))
