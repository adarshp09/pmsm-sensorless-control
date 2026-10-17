"""Phase 03f Step 2: SOGI + positive-sequence response off resonance vs closed form.

With ``omega'`` frozen (no FLL) and a positive-sequence input ``z = e^{j omega t}``,
the implemented SOGI outputs must match ``D(j omega)`` and ``Q(j omega)`` (Eqs.9-10)
and the positive-sequence combination must equal ``½(D + jQ) = ½ D (1 + omega'/omega)``.
Tolerances are set at 0.2-0.3 % (the prewarped biquad matches the continuous form to
~1e-4 at ``omega' Ts = 0.063``; the bound leaves margin for off-resonance warping).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from pmsm_sensorless.filters.sogi import (
    SOGIQSG,
    in_phase_transfer,
    quadrature_transfer,
)

K_S = math.sqrt(2.0)
OMEGA_PRIME = 2.0 * math.pi * 100.0
DT = 1e-4
SAMPLES = 40000
START = 20000


def _responses(ratio: float):
    omega = ratio * OMEGA_PRIME
    alpha = SOGIQSG(K_S)
    beta = SOGIQSG(K_S)
    pd, pq, pz = [], [], []
    for k in range(SAMPLES):
        z = np.exp(1j * omega * k * DT)
        out_a = alpha.step(float(z.real), OMEGA_PRIME, DT)
        out_b = beta.step(float(z.imag), OMEGA_PRIME, DT)
        if k >= START:
            phasor = np.exp(-1j * omega * k * DT)
            pd.append(out_a.in_phase * phasor)
            pq.append(out_a.quadrature * phasor)
            positive = 0.5 * (out_a.in_phase - out_b.quadrature) + 1j * 0.5 * (
                out_a.quadrature + out_b.in_phase
            )
            pz.append(positive * phasor)
    n = len(pd)
    return (
        2.0 / n * np.sum(pd),
        2.0 / n * np.sum(pq),
        1.0 / n * np.sum(pz),  # complex signal -> 1/N
    )


@pytest.mark.parametrize("ratio", [0.8, 0.9, 1.0, 1.1, 1.25])
def test_sogi_and_pns_match_closed_form_off_resonance(ratio: float) -> None:
    omega = ratio * OMEGA_PRIME
    measured_d, measured_q, measured_positive = _responses(ratio)
    d = in_phase_transfer(omega, K_S, OMEGA_PRIME)
    q = quadrature_transfer(omega, K_S, OMEGA_PRIME)
    expected_positive = 0.5 * (d + 1j * q)

    assert abs(measured_d - d) <= 2e-3 * abs(d)
    assert abs(measured_q - q) <= 2e-3 * abs(q)
    assert abs(measured_positive - expected_positive) <= 3e-3 * abs(expected_positive)
    # closed-form identity from the task
    assert expected_positive == pytest.approx(0.5 * d * (1.0 + OMEGA_PRIME / omega))
