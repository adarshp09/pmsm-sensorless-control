"""PMSM plant tests: Eq.(1)-(2) consistency, deterministic updates, limiting cases.

Tolerances follow from the numerical method. The electrical subsystem is linear,
so a constant-input case has a closed-form exponential solution; the plant uses a
fourth-order Runge-Kutta step, hence the trajectory matches that solution to well
below ``1e-6`` relative for the timesteps used here. The opt-in mechanical model
is validated against its closed-form friction decay.
"""

from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.plant.pmsm import (
    PMSMPlant,
    PMSMParameters,
    PMSMState,
    rk4_step,
    wrap_angle,
)


@pytest.fixture
def plant(config_dir) -> PMSMPlant:
    cfg = config.load_config("pmsm", directory=config_dir)
    return PMSMPlant.from_config(cfg)


# --- parameters ---------------------------------------------------------------


def test_parameters_come_from_validated_config(plant: PMSMPlant) -> None:
    parameters = plant.parameters
    assert parameters.pole_pairs == 4
    assert parameters.stator_resistance == pytest.approx(0.505, rel=1e-9)
    assert parameters.ld == pytest.approx(0.00147, rel=1e-9)
    assert parameters.lq == pytest.approx(0.00147, rel=1e-9)
    assert parameters.psi_f == pytest.approx(0.1326, rel=1e-9)
    assert parameters.inertia == pytest.approx(0.00153, rel=1e-9)
    assert parameters.viscous_friction is None  # unresolved in the paper


def test_salient_machine_is_rejected() -> None:
    with pytest.raises(ValueError):
        PMSMParameters(4, 0.5, 0.001, 0.002, 0.13)


def test_invalid_parameter_values_are_rejected() -> None:
    with pytest.raises(ValueError):
        PMSMParameters(0, 0.5, 0.001, 0.001, 0.13)
    with pytest.raises(ValueError):
        PMSMParameters(4, -0.5, 0.001, 0.001, 0.13)
    with pytest.raises(ValueError):
        PMSMParameters(4, 0.5, 0.001, 0.001, 0.13, viscous_friction=-1.0)


# --- Eq.(2) back-EMF ----------------------------------------------------------


@pytest.mark.parametrize(
    "theta_e, omega_e",
    [(0.0, 0.0), (0.0, 100.0), (math.pi / 6.0, 200.0), (-1.2, 50.0), (2.5, -30.0)],
)
def test_back_emf_matches_equation_2(plant: PMSMPlant, theta_e: float, omega_e: float) -> None:
    psi_f = plant.parameters.psi_f
    amplitude = omega_e * psi_f
    expected = np.array([-amplitude * math.sin(theta_e), amplitude * math.cos(theta_e)])
    assert plant.back_emf(theta_e, omega_e) == pytest.approx(expected)


def test_back_emf_is_zero_at_standstill(plant: PMSMPlant) -> None:
    assert plant.back_emf(1.234, 0.0) == pytest.approx([0.0, 0.0])


def test_steady_state_current_is_un_minus_emf_over_rs(plant: PMSMPlant) -> None:
    u_alpha, u_beta, theta_e, omega_e = 10.0, -20.0, 0.3, 77.0
    expected = (np.array([u_alpha, u_beta]) - plant.back_emf(theta_e, omega_e)) / (
        plant.parameters.stator_resistance
    )
    assert plant.steady_state_current(u_alpha, u_beta, theta_e, omega_e) == pytest.approx(expected)


# --- Eq.(1) dynamics ----------------------------------------------------------


def test_free_response_matches_analytic_exponential(plant: PMSMPlant) -> None:
    """At standstill with u=[U,0], i_alpha(t) = (U/Rs)(1 - exp(-t/tau))."""
    parameters = plant.parameters
    tau = parameters.ld / parameters.stator_resistance
    u_alpha = 5.0
    dt = 1e-5
    state = PMSMState(0.0, 0.0, 0.0, 0.0)
    steps = round(8.0 * tau / dt)
    for _ in range(steps):
        state = plant.step(state, u_alpha, 0.0, dt)
    t = steps * dt
    i_ss = u_alpha / parameters.stator_resistance
    expected = i_ss * (1.0 - math.exp(-t / tau))
    assert state.i_alpha == pytest.approx(expected, rel=1e-6)
    assert state.i_beta == pytest.approx(0.0, abs=1e-12)


def test_rotating_back_emf_steady_state_amplitude(plant: PMSMPlant) -> None:
    """With u=0 and constant electrical speed the current settles on a rotating
    vector of magnitude ``omega_e*psi_f / sqrt(Rs^2 + (omega_e*Ld)^2)``."""
    parameters = plant.parameters
    omega_e = 209.44  # ~500 rpm mechanical with p=4
    dt = 1e-4
    state = PMSMState(0.0, 0.0, omega_e, 0.0)
    for _ in range(2000):  # ~ 0.2 s, many electrical time constants
        state = plant.step(state, 0.0, 0.0, dt)
    expected = (omega_e * parameters.psi_f) / math.sqrt(
        parameters.stator_resistance**2 + (omega_e * parameters.ld) ** 2
    )
    magnitude = math.hypot(state.i_alpha, state.i_beta)
    assert magnitude == pytest.approx(expected, rel=1e-3)


def test_short_circuit_rotating_current_matches_closed_form(plant: PMSMPlant) -> None:
    """u=0, constant electrical speed: i(t) = A e^{j w t} - A e^{-t/tau}.

    This pins the substage consistency of the angle integration: an update that
    froze the back-EMF at the step-start angle would only be first-order accurate
    and miss the 1e-6 tolerance by orders of magnitude.
    """
    parameters = plant.parameters
    tau = parameters.ld / parameters.stator_resistance
    omega_e = 209.44
    dt = 1e-4
    steps = 1000
    a_ss = -1j * omega_e * parameters.psi_f / (
        parameters.stator_resistance + 1j * omega_e * parameters.ld
    )
    state = PMSMState(0.0, 0.0, omega_e, 0.0)
    worst = 0.0
    for k in range(1, steps + 1):
        state = plant.step(state, 0.0, 0.0, dt)
        t = k * dt
        reference = a_ss * np.exp(1j * omega_e * t) - a_ss * math.exp(-t / tau)
        worst = max(worst, math.hypot(state.i_alpha - reference.real, state.i_beta - reference.imag))
    assert worst < 1e-5


def test_prescribed_kinematics_advance_the_angle(plant: PMSMPlant) -> None:
    omega_e = 40.0
    dt = 1e-4
    state = PMSMState(0.0, 0.0, omega_e, 0.0)
    advanced = plant.step(state, 0.0, 0.0, dt)
    assert advanced.omega_e == pytest.approx(omega_e)  # speed is prescribed, not integrated
    assert advanced.theta_e == pytest.approx(float(wrap_angle(omega_e * dt)))


def test_electromagnetic_torque_formula(plant: PMSMPlant) -> None:
    parameters = plant.parameters
    theta_e = 0.4
    i_alpha, i_beta = 3.0, -2.0
    i_q = -i_alpha * math.sin(theta_e) + i_beta * math.cos(theta_e)
    expected = 1.5 * parameters.pole_pairs * parameters.psi_f * i_q
    assert plant.electromagnetic_torque([i_alpha, i_beta], theta_e) == pytest.approx(expected)


# --- determinism, limits, argument checking ----------------------------------


def test_step_is_deterministic(plant: PMSMPlant) -> None:
    dt, steps = 5e-5, 400
    state = PMSMState(0.0, 0.0, 60.0, 0.0)

    def run() -> list[float]:
        current = state
        trace = []
        for _ in range(steps):
            current = plant.step(current, 3.0, -1.0, dt)
            trace.append(current.i_alpha)
        return trace

    first = run()
    second = run()
    assert first == second  # bit-for-bit identical


def test_zero_inputs_remain_zero_and_finite(plant: PMSMPlant) -> None:
    state = PMSMState(0.0, 0.0, 0.0, 0.0)
    for _ in range(1000):
        state = plant.step(state, 0.0, 0.0, 1e-4)
    assert (state.i_alpha, state.i_beta, state.theta_e) == (0.0, 0.0, 0.0)


def test_outputs_are_finite_under_rotating_field(plant: PMSMPlant) -> None:
    state = PMSMState(0.0, 0.0, 300.0, 0.0)
    for _ in range(5000):
        state = plant.step(state, 50.0, -25.0, 1e-4)
    assert all(math.isfinite(v) for v in (state.i_alpha, state.i_beta, state.omega_e, state.theta_e))
    assert -math.pi <= state.theta_e < math.pi


def test_nonpositive_timestep_is_rejected(plant: PMSMPlant) -> None:
    state = PMSMState(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        plant.step(state, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        plant.step(state, 0.0, 0.0, -1e-4)


def test_wrap_angle_stays_in_half_open_interval() -> None:
    for raw in np.linspace(-4.0 * math.pi, 4.0 * math.pi, 101):
        wrapped = float(wrap_angle(raw))
        assert -math.pi <= wrapped < math.pi
        assert wrapped == pytest.approx(float(wrap_angle(raw + 2.0 * math.pi)))


def test_rk4_step_matches_exponential_for_linear_decay() -> None:
    dt = 0.1
    y1 = rk4_step(lambda y: -y, np.array([1.0]), dt)
    assert float(y1[0]) == pytest.approx(math.exp(-dt), rel=1e-6)


# --- opt-in mechanical extension (derived; not the paper's model) ------------


def test_mechanical_mode_requires_explicit_friction(plant: PMSMPlant) -> None:
    # viscous_friction_b is unresolved -> must not be defaulted silently
    with pytest.raises(ValueError):
        PMSMPlant(plant.parameters, integrate_mechanics=True)


def test_mechanical_mode_requires_inertia() -> None:
    parameters = PMSMParameters(4, 0.505, 0.00147, 0.00147, 0.1326, inertia=None)
    with pytest.raises(ValueError):
        PMSMPlant(parameters, integrate_mechanics=True, viscous_friction=1e-3)


def test_mechanical_friction_decay_matches_analytic() -> None:
    """With zero torque, d(omega_e)/dt = -(B/J) omega_e."""
    parameters = PMSMParameters(4, 0.505, 0.00147, 0.00147, 0.1326, inertia=1.53e-3)
    friction = 1e-3
    plant = PMSMPlant(parameters, integrate_mechanics=True, viscous_friction=friction)
    omega_e0 = 50.0
    dt = 1e-6
    steps = round(0.01 / dt)
    state = PMSMState(0.0, 0.0, omega_e0, 0.0)
    for _ in range(steps):
        emf = plant.back_emf(state.theta_e, state.omega_e)
        state = plant.step(state, float(emf[0]), float(emf[1]), dt)  # u = E -> T_e ~ 0
    t = steps * dt
    tau_m = parameters.inertia / friction
    assert state.omega_e == pytest.approx(omega_e0 * math.exp(-t / tau_m), rel=1e-4)


def test_speed_derivative_formula() -> None:
    parameters = PMSMParameters(4, 0.505, 0.00147, 0.00147, 0.1326, inertia=2e-3)
    plant = PMSMPlant(parameters, integrate_mechanics=True, viscous_friction=4e-3)
    domega = plant.speed_derivative(omega_e=10.0, torque=5.0, load_torque=1.0)
    expected = (parameters.pole_pairs * (5.0 - 1.0) - 4e-3 * 10.0) / 2e-3
    assert domega == pytest.approx(expected)


def test_from_config_rejects_unresolved_required_parameter(config_dir) -> None:
    cfg = config.load_config("pmsm", directory=config_dir)
    broken = copy.deepcopy(cfg)
    broken["parameters"]["machine.stator_resistance_rs"]["value"] = None
    with pytest.raises(config.ConfigError):
        PMSMParameters.from_config(broken)


def test_from_config_rejects_non_integer_pole_pairs(config_dir) -> None:
    cfg = config.load_config("pmsm", directory=config_dir)
    broken = copy.deepcopy(cfg)
    broken["parameters"]["machine.pole_pairs"]["value"] = 4.5
    with pytest.raises(config.ConfigError):
        PMSMParameters.from_config(broken)


# --- load-torque profiles -----------------------------------------------------


def test_constant_load_torque_is_constant() -> None:
    from pmsm_sensorless.plant.load_models import ConstantLoadTorque

    profile = ConstantLoadTorque(3.85)
    assert profile(0.0) == 3.85
    assert profile(10.0) == 3.85


def test_step_load_torque_switches_at_step_time() -> None:
    from pmsm_sensorless.plant.load_models import StepLoadTorque

    profile = StepLoadTorque(initial_torque=0.0, final_torque=3.85, step_time=1.5)
    assert profile(1.499) == 0.0
    assert profile(1.5) == 3.85
    assert profile(2.0) == 3.85


def test_step_load_torque_rejects_negative_step_time() -> None:
    from pmsm_sensorless.plant.load_models import StepLoadTorque

    with pytest.raises(ValueError):
        StepLoadTorque(0.0, 1.0, step_time=-0.1)
