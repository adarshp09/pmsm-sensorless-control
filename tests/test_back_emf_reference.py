"""Phase 03b reference and time-alignment checks for the true back-EMF.

Confirms Eq.(2), p.2: ``E = omega_e psi_f [-sin(theta_e), cos(theta_e)]`` -- its
magnitude ``|E| = psi_f omega_e``, its frame/sign, and that ``E[k]`` is evaluated
at the *current-sample* angle (a one-sample offset shows up as ``omega_e*dt`` of
phase). These are evaluation-layer properties; no observer/filter code is used.
"""

from __future__ import annotations

import cmath
import math

import numpy as np
import pytest

from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState


@pytest.fixture
def parameters(config_dir):
    from pmsm_sensorless import config

    return PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir))


def _phasor(alpha, beta, theta) -> complex:
    z = np.asarray(alpha) + 1j * np.asarray(beta)
    return complex(np.mean(z * np.exp(-1j * np.asarray(theta))))


@pytest.mark.parametrize("rpm", [100.0, 200.0, 500.0, 1000.0])
def test_back_emf_magnitude_equals_psi_f_times_electrical_speed(parameters, rpm: float) -> None:
    plant = PMSMPlant(parameters)
    omega_e = parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
    emf = plant.back_emf(0.3, omega_e)
    assert math.hypot(*emf) == pytest.approx(parameters.psi_f * omega_e, rel=1e-12)


def test_back_emf_frame_and_sign(parameters) -> None:
    plant = PMSMPlant(parameters)
    theta_e, omega_e = 0.7, 123.0
    emf = plant.back_emf(theta_e, omega_e)
    amplitude = parameters.psi_f * omega_e
    assert emf == pytest.approx([-amplitude * math.sin(theta_e), amplitude * math.cos(theta_e)])
    # as a complex vector, E = j omega psi exp(j theta_e)
    z = emf[0] + 1j * emf[1]
    assert cmath.phase(z) == pytest.approx(math.pi / 2 + theta_e, abs=1e-12)


def test_one_sample_offset_shows_up_as_omega_e_times_dt(parameters) -> None:
    """A one-sample misalignment costs ``omega_e*dt`` of phase (alignment convention)."""
    plant = PMSMPlant(parameters)
    dt, omega_e, steps = 1e-4, 419.0, 2000
    theta = omega_e * np.arange(steps) * dt
    emf = np.array([plant.back_emf(t, omega_e) for t in theta])
    aligned = _phasor(emf[:, 0], emf[:, 1], theta)
    shifted = _phasor(emf[1:, 0], emf[1:, 1], theta[:-1])
    assert abs(aligned) == pytest.approx(parameters.psi_f * omega_e, rel=1e-9)
    assert cmath.phase(aligned) == pytest.approx(math.pi / 2, abs=1e-9)  # residual theta = 0
    assert cmath.phase(shifted / aligned) == pytest.approx(omega_e * dt, abs=1e-9)


def test_plant_back_emf_uses_the_current_sample_angle(parameters) -> None:
    """The rollout's ``emf[k]`` is ``back_emf(theta_e[k])`` -- no off-by-one."""
    plant = PMSMPlant(parameters)
    dt, omega_e = 1e-4, 300.0
    state = PMSMState(0.0, 0.0, omega_e, 0.0)
    first = plant.back_emf(state.theta_e, state.omega_e)
    advanced = plant.step(state, 0.0, 0.0, dt)
    second = plant.back_emf(advanced.theta_e, advanced.omega_e)
    delta = cmath.phase((second[0] + 1j * second[1]) / (first[0] + 1j * first[1]))
    assert delta == pytest.approx(omega_e * dt, abs=1e-9)  # one sample of rotation
