"""Phase 05a Step 1: read-only ``v'`` accessor on ``DSOGIFLLOutput``.

Additive only: ``v_alpha``/``v_beta`` and the FLL state keep their semantics, and
``omega'(t)`` does not depend on ``positive_sequence`` (the sequence combination
is output-only).
"""

from __future__ import annotations

import math

import numpy as np

from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters

K_S = math.sqrt(2.0)
GAMMA = 50.0
INITIAL_OMEGA = 2.0 * math.pi * 50.0


def _inputs(n: int = 600, dt: float = 1e-4) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(n) * dt
    v_alpha = 50.0 * np.cos(2.0 * math.pi * 50.0 * t) + 3.0 * np.cos(2.0 * math.pi * 300.0 * t)
    v_beta = 50.0 * np.sin(2.0 * math.pi * 50.0 * t) - 2.0 * np.sin(2.0 * math.pi * 120.0 * t)
    return v_alpha, v_beta


def _run(positive_sequence: bool):
    params = DSOGIFLLParameters(K_S, GAMMA, INITIAL_OMEGA, positive_sequence=positive_sequence)
    fll = DSOGIFLL(params)
    v_alpha, v_beta = _inputs()
    out = [fll.step(float(a), float(b), 1e-4) for a, b in zip(v_alpha, v_beta)]
    return out


def test_v_prime_accessor_equals_per_axis_mode_bit_identically() -> None:
    pns = _run(positive_sequence=True)
    per_axis = _run(positive_sequence=False)
    for o_pns, o_axis in zip(pns, per_axis):
        # the new fields are exactly the per-axis v' that mode False returns
        assert o_pns.v_alpha_prime == o_axis.v_alpha
        assert o_pns.v_beta_prime == o_axis.v_beta
        # and mode False's new fields equal its own outputs
        assert o_axis.v_alpha_prime == o_axis.v_alpha
        assert o_axis.v_beta_prime == o_axis.v_beta


def test_omega_is_identical_across_positive_sequence() -> None:
    pns = _run(positive_sequence=True)
    per_axis = _run(positive_sequence=False)
    for o_pns, o_axis in zip(pns, per_axis):
        assert o_pns.omega == o_axis.omega
        assert o_pns.frequency_hz == o_axis.frequency_hz
        assert o_pns.epsilon_f == o_axis.epsilon_f
        assert o_pns.qv_alpha == o_axis.qv_alpha
        assert o_pns.qv_beta == o_axis.qv_beta


def test_positive_sequence_output_uses_the_accessor_fields() -> None:
    for out in _run(positive_sequence=True):
        assert out.v_alpha == 0.5 * (out.v_alpha_prime - out.qv_beta)
        assert out.v_beta == 0.5 * (out.qv_alpha + out.v_beta_prime)


def test_existing_fields_unchanged_for_per_axis_mode() -> None:
    for out in _run(positive_sequence=False):
        assert out.v_alpha == out.v_alpha_prime
        assert out.v_beta == out.v_beta_prime


def test_accessor_fields_are_finite() -> None:
    for out in _run(positive_sequence=True):
        assert math.isfinite(out.v_alpha_prime) and math.isfinite(out.v_beta_prime)
