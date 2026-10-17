"""Analytical tests for the SOGI-QSG (paper Eqs.(9)-(10), discretized via Eq.(17))."""

from __future__ import annotations

import cmath
import math

import numpy as np
import pytest

from pmsm_sensorless.filters.sogi import (
    SOGIQSG,
    in_phase_transfer,
    quadrature_transfer,
    sogi_coefficients,
)

K_S = math.sqrt(2.0)
OMEGA_0 = 2.0 * math.pi * 100.0
DT = 1e-4


def _phasor_of_driven_sogi(omega: float, output: str, amplitude: float = 1.0) -> complex:
    """Steady-state phasor of the SOGI output when driven at ``omega`` on resonance."""
    sogi = SOGIQSG(K_S)
    samples: list[float] = []
    indices: list[int] = []
    for k in range(20000):
        result = sogi.step(amplitude * math.cos(omega * k * DT), OMEGA_0, DT)
        if k >= 10000:
            indices.append(k)
            samples.append(result.in_phase if output == "in_phase" else result.quadrature)
    idx = np.asarray(indices)
    sig = np.asarray(samples)
    return 2.0 / len(samples) * np.sum(sig * np.exp(-1j * omega * idx * DT))


# --- closed-form transfer functions, Eqs.(9)-(10) ----------------------------


def test_closed_form_matches_the_paper_transfer_functions() -> None:
    """D(jw) = k w' s / (s^2 + k w' s + w'^2), Q = k w'^2 / (...), s = jw."""
    for omega in (0.5 * OMEGA_0, OMEGA_0, 3.0 * OMEGA_0):
        s = 1j * omega
        denominator = s**2 + K_S * OMEGA_0 * s + OMEGA_0**2
        assert in_phase_transfer(omega, K_S, OMEGA_0) == pytest.approx(
            K_S * OMEGA_0 * s / denominator
        )
        assert quadrature_transfer(omega, K_S, OMEGA_0) == pytest.approx(
            K_S * OMEGA_0**2 / denominator
        )


def test_closed_form_resonance_at_omega_prime() -> None:
    """At s = j w': |D| = 1, angle(D) = 0; |Q| = 1, angle(Q) = -90 deg."""
    d = in_phase_transfer(OMEGA_0, K_S, OMEGA_0)
    q = quadrature_transfer(OMEGA_0, K_S, OMEGA_0)
    assert abs(d) == pytest.approx(1.0)
    assert math.degrees(cmath.phase(d)) == pytest.approx(0.0, abs=1e-9)
    assert abs(q) == pytest.approx(1.0)
    assert math.degrees(cmath.phase(q)) == pytest.approx(-90.0)


def test_dc_and_gain_values() -> None:
    """D(0) = 0 (band-pass), Q(0) = k_s (low-pass DC gain)."""
    assert in_phase_transfer(0.0, K_S, OMEGA_0) == pytest.approx(0.0)
    assert quadrature_transfer(0.0, K_S, OMEGA_0) == pytest.approx(K_S)


# --- discretized realization --------------------------------------------------


def test_discrete_resonance_is_exact() -> None:
    """Prewarping makes the discrete response exact at omega' (Eq.(17))."""
    d = _phasor_of_driven_sogi(OMEGA_0, "in_phase")
    q = _phasor_of_driven_sogi(OMEGA_0, "quadrature")
    assert abs(d) == pytest.approx(1.0, rel=1e-3)
    assert math.degrees(cmath.phase(d)) == pytest.approx(0.0, abs=0.1)
    assert abs(q) == pytest.approx(1.0, rel=1e-3)
    assert math.degrees(cmath.phase(q)) == pytest.approx(-90.0, abs=0.1)


def test_discrete_dc_gain_matches_closed_form() -> None:
    """Biquad DC gains: D(1) = 0, Q(1) = k_s."""
    coefficients = sogi_coefficients(K_S, OMEGA_0, DT)
    denominator = 1.0 + coefficients.a1 + coefficients.a2
    d = (coefficients.b0_in_phase + coefficients.b1_in_phase + coefficients.b2_in_phase) / denominator
    q = (
        coefficients.b0_quadrature + coefficients.b1_quadrature + coefficients.b2_quadrature
    ) / denominator
    assert d == pytest.approx(0.0)
    assert q == pytest.approx(K_S)


@pytest.mark.parametrize("harmonic, tolerance", [(5.0, 0.03), (10.0, 0.10)])
def test_high_frequency_roll_off_against_closed_form(harmonic: float, tolerance: float) -> None:
    """Discrete magnitude tracks the closed-form transfer far above resonance."""
    for output, transfer in (("in_phase", in_phase_transfer), ("quadrature", quadrature_transfer)):
        omega = harmonic * OMEGA_0
        discrete = abs(_phasor_of_driven_sogi(omega, output))
        closed = abs(transfer(omega, K_S, OMEGA_0))
        assert discrete == pytest.approx(closed, rel=tolerance)


def test_dc_input_is_blocked_by_the_band_pass() -> None:
    sogi = SOGIQSG(K_S)
    for _ in range(5000):
        result = sogi.step(1.0, OMEGA_0, DT)
    assert abs(result.in_phase) < 1e-3  # band-pass rejects DC
    assert result.quadrature == pytest.approx(K_S, rel=1e-2)  # low-pass passes DC


# --- state, determinism, validation ------------------------------------------


def test_state_is_persistent_and_reset_works() -> None:
    sogi = SOGIQSG(K_S)
    first = sogi.step(1.0, OMEGA_0, DT)
    assert sogi.state.in_phase_z1 != 0.0
    sogi.reset()
    assert sogi.state.in_phase_z1 == 0.0
    assert sogi.step(1.0, OMEGA_0, DT) == first  # reset replays identically


def test_update_is_deterministic() -> None:
    def run() -> list[float]:
        sogi = SOGIQSG(K_S)
        return [sogi.step(math.cos(0.01 * k), OMEGA_0, DT).in_phase for k in range(500)]

    assert run() == run()


def test_invalid_coefficient_arguments_are_rejected() -> None:
    with pytest.raises(ValueError):
        sogi_coefficients(0.0, OMEGA_0, DT)
    with pytest.raises(ValueError):
        sogi_coefficients(K_S, -1.0, DT)
    with pytest.raises(ValueError):
        sogi_coefficients(K_S, OMEGA_0, 0.0)
    with pytest.raises(ValueError):
        SOGIQSG(0.0)
