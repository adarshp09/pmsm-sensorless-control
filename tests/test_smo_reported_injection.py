"""Regression tests for D-025: the observer reports the *applied* injection.

The reported ``back_emf = k*g(S)`` must use the sample-instant error ``S_pre`` (the
quantity injected in Eq.(4)), not a post-update recomputation ``g(S_post)``. The
tests below drive the observer with a **sinusoidal** back-EMF (the plant at a
constant 1000 rpm) and check the reported fundamental gain/phase against the truth.

Tolerances follow the measured applied-injection values (``0.996 / -0.2 deg`` at
``Ts = 10 us``; ``0.957 / -1.6 deg`` at ``Ts = 100 us``).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO
from pmsm_sensorless.observers.smo import SMOObserverParameters, SMOObserverState
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState

RPM = 1000.0


@pytest.fixture
def configs(config_dir):
    return (
        config.load_config("pmsm", directory=config_dir),
        config.load_config("observer", directory=config_dir),
    )


@pytest.fixture
def plant(configs) -> PMSMPlant:
    return PMSMPlant(PMSMParameters.from_config(configs[0]))


def _reported_gain_phase(plant: PMSMPlant, observer, dt: float, duration: float = 0.1):
    """Fundamental gain/phase of the reported back-EMF vs the true back-EMF."""
    omega = plant.parameters.pole_pairs * RPM * 2.0 * math.pi / 60.0
    steps = int(round(duration / dt))
    state = PMSMState(0.0, 0.0, omega, 0.0)
    reported = np.empty(steps, dtype=complex)
    true = np.empty(steps, dtype=complex)
    theta = np.empty(steps)
    for k in range(steps):
        emf = plant.back_emf(state.theta_e, state.omega_e)
        true[k] = emf[0] + 1j * emf[1]
        theta[k] = state.theta_e
        out = observer.step(0.0, 0.0, state.i_alpha, state.i_beta, dt)
        reported[k] = out.back_emf_alpha + 1j * out.back_emf_beta
        state = plant.step(state, 0.0, 0.0, dt)
    steady = slice(steps // 2, steps)
    phase_ref = np.exp(-1j * theta[steady])
    phasor = np.mean(reported[steady] * phase_ref)
    phasor_true = np.mean(true[steady] * phase_ref)
    return abs(phasor) / abs(phasor_true), math.degrees(np.angle(phasor / phasor_true))


def test_reported_back_emf_uses_the_sample_instant_error() -> None:
    """S_pre = -1 (S_post = 0): the reported signal must be k*sign(S_pre)."""
    observer = ConventionalSMO(
        SMOObserverParameters(stator_resistance=0.5, ld=0.01, switching_gain_k=10.0),
        SMOObserverState(0.0, 0.0),
    )
    out = observer.step(0.0, 0.0, 1.0, 0.0, 1e-3)
    assert out.s_alpha == pytest.approx(-1.0)
    assert out.back_emf_alpha == pytest.approx(-10.0)


def test_sign_reported_back_emf_matches_truth_at_100us(plant, configs) -> None:
    gain, phase = _reported_gain_phase(
        plant, ConventionalSMO.from_config(*configs), 100e-6
    )
    assert gain == pytest.approx(0.957, abs=0.04)
    assert phase == pytest.approx(-1.6, abs=3.0)


def test_sign_reported_back_emf_matches_truth_at_10us(plant, configs) -> None:
    gain, phase = _reported_gain_phase(
        plant, ConventionalSMO.from_config(*configs), 10e-6, duration=0.05
    )
    assert gain == pytest.approx(0.996, abs=0.01)
    assert phase == pytest.approx(-0.2, abs=1.0)


def test_sigmoid_reported_back_emf_matches_truth_at_100us(plant, configs) -> None:
    gain, phase = _reported_gain_phase(plant, SigmoidSMO.from_config(*configs), 100e-6)
    assert gain == pytest.approx(0.962, abs=0.05)
    assert phase == pytest.approx(-1.3, abs=3.0)
