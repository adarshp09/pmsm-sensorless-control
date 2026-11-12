"""Phase 04 Step 0b: independent check of the SOGI D-phase sign.

``D(j omega)`` and ``Q(j omega)`` are evaluated with ``scipy.signal.freqs`` on the
continuous transfer functions (independent of the repo's closed-form helper) and
compared with the implemented prewarped biquad at ``z = exp(j omega Ts)``. The
D-phase must equal ``+atan((r^2 - 1)/(k_s r))`` with ``r = omega' / omega`` (lead
when ``omega' > omega``); e.g. ``omega'=1, omega=0.9`` gives ``+8.5 deg``.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy import signal

from pmsm_sensorless.filters.sogi import sogi_coefficients

K_S = math.sqrt(2.0)


def _discrete_response(coefficients, omega: float, dt: float) -> tuple[complex, complex]:
    z_inv = np.exp(-1j * omega * dt)
    denominator = 1.0 + coefficients.a1 * z_inv + coefficients.a2 * z_inv**2
    d = (
        coefficients.b0_in_phase
        + coefficients.b1_in_phase * z_inv
        + coefficients.b2_in_phase * z_inv**2
    ) / denominator
    q = (
        coefficients.b0_quadrature
        + coefficients.b1_quadrature * z_inv
        + coefficients.b2_quadrature * z_inv**2
    ) / denominator
    return d, q


def test_d_phase_equals_plus_atan_of_r_squared_minus_one() -> None:
    omega_prime, omega, dt = 1.0, 0.9, 1e-3
    coefficients = sogi_coefficients(K_S, omega_prime, dt)
    d_discrete, q_discrete = _discrete_response(coefficients, omega, dt)

    # continuous via scipy.signal.freqs (independent of the repo helper)
    _, d_continuous = signal.freqs(
        [K_S * omega_prime, 0.0], [1.0, K_S * omega_prime, omega_prime**2], worN=[omega]
    )
    _, q_continuous = signal.freqs(
        [K_S * omega_prime**2], [1.0, K_S * omega_prime, omega_prime**2], worN=[omega]
    )
    d_continuous = d_continuous[0]
    q_continuous = q_continuous[0]

    r = omega_prime / omega  # r = omega'/omega
    expected_phase = math.degrees(math.atan((r * r - 1.0) / (K_S * r)))
    assert expected_phase == pytest.approx(8.5, abs=0.1)

    # continuous (scipy) and discrete (implemented biquad) must both match the
    # closed-form phase, with the *positive* sign
    assert math.degrees(np.angle(d_continuous)) == pytest.approx(expected_phase, abs=1e-9)
    assert math.degrees(np.angle(d_discrete)) == pytest.approx(expected_phase, abs=1.0)

    # magnitudes agree closely at small omega'Ts
    assert abs(d_discrete) == pytest.approx(abs(d_continuous), rel=2e-3)
    assert abs(q_discrete) == pytest.approx(abs(q_continuous), rel=2e-3)
