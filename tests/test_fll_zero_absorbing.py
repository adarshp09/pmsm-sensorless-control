"""Phase 03e: omega' = 0 is an absorbing fixed point of the DSOGI-FLL.

At omega' = 0 the prewarped biquad numerators vanish (``sin(0)=0`` and
``1-cos(0)=0``), so ``D = Q = 0``, both SOGI outputs are zero, and therefore
``epsilon_f = qv'(v-v') = 0`` -> ``d omega'/dt = 0``. Once the loop reaches zero it
cannot leave (the collapse observed in Phase 03d).
"""

from __future__ import annotations

import math

import pytest

from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters
from pmsm_sensorless.filters.sogi import SOGIQSG, sogi_coefficients

K_S = math.sqrt(2.0)
GAMMA = 50.0
DT = 1e-4


def test_sogi_numerators_vanish_at_zero_frequency() -> None:
    coefficients = sogi_coefficients(K_S, 0.0, DT)
    assert coefficients.b0_in_phase == coefficients.b1_in_phase == coefficients.b2_in_phase == 0.0
    assert (
        coefficients.b0_quadrature
        == coefficients.b1_quadrature
        == coefficients.b2_quadrature
        == 0.0
    )


@pytest.mark.parametrize("v", [-10.0, 0.0, 3.5])
def test_sogi_outputs_are_zero_at_zero_frequency(v: float) -> None:
    sogi = SOGIQSG(K_S)
    for _ in range(50):
        out = sogi.step(v, 0.0, DT)
    assert out.in_phase == 0.0
    assert out.quadrature == 0.0


def test_epsilon_f_and_omega_do_not_move_at_zero_frequency() -> None:
    # initial_omega must be > 0 in the constructor; use a tiny positive value.
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 1e-9))
    omega_after = fll.omega
    for k in range(2000):
        out = fll.step(10.0 * math.cos(0.01 * k), 10.0 * math.sin(0.01 * k), DT)
        omega_after = fll.omega
    assert abs(out.epsilon_f) < 1e-12
    assert omega_after == pytest.approx(1e-9, abs=1e-12)  # stays absorbed at ~0
