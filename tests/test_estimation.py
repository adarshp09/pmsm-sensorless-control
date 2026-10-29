"""Phase 05b Step 1: position/speed estimators (Eq.(8) and Fig.1 Q-PLL).

Analytical tests with justified tolerances; a structure test (no plant/observer/
filter/fuzzy imports) and determinism/reset.
"""

from __future__ import annotations

import importlib
import inspect
import math
import types

import pytest

from pmsm_sensorless import config
from pmsm_sensorless.estimation import (
    ArctanEstimator,
    QPLL,
    QPLLParameters,
    electrical_to_mechanical_speed,
    wrap_angle_pi_closed,
)

TWO_PI = 2.0 * math.pi
P = 4
PSI_F = 0.1326
OMEGA_1000 = P * 1000.0 * TWO_PI / 60.0          # electrical rad/s at 1000 rpm
AMPLITUDE_1000 = OMEGA_1000 * PSI_F               # |E| at 1000 rpm = 55.543 V


def _emf(theta: float, amplitude: float = AMPLITUDE_1000) -> tuple[float, float]:
    """Eq.(2): E = amplitude * [-sin theta, cos theta]."""
    return -amplitude * math.sin(theta), amplitude * math.cos(theta)


def _qpll_params(amplitude: float = AMPLITUDE_1000) -> QPLLParameters:
    zeta, f_n = 0.707, 20.0
    w_n = TWO_PI * f_n
    return QPLLParameters(kp=2.0 * zeta * w_n / amplitude, ki=w_n * w_n / amplitude)


# --- wrap convention ----------------------------------------------------------


def test_wrap_angle_pi_closed_convention() -> None:
    assert wrap_angle_pi_closed(0.0) == pytest.approx(0.0)
    assert wrap_angle_pi_closed(math.pi) == pytest.approx(math.pi)
    assert wrap_angle_pi_closed(-math.pi) == pytest.approx(math.pi)      # (-pi, pi]
    assert wrap_angle_pi_closed(3.0 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle_pi_closed(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    for x in (-20.0, -7.0, -3.2, 0.0, 2.0, 6.5, 20.0):
        assert -math.pi < wrap_angle_pi_closed(x) <= math.pi


def test_electrical_to_mechanical_speed() -> None:
    assert electrical_to_mechanical_speed(OMEGA_1000, P) == pytest.approx(OMEGA_1000 / P)
    with pytest.raises(ValueError):
        electrical_to_mechanical_speed(1.0, 0)


# --- arctan estimator ---------------------------------------------------------


def test_arctan_is_exact_for_ideal_rotating_emf() -> None:
    """Eq.(8) via atan2(-E_alpha, E_beta) is exact for Eq.(2) in every quadrant."""
    estimator = ArctanEstimator()
    worst = 0.0
    for k in range(2001):
        theta = -math.pi + TWO_PI * k / 2000.0
        ea, eb = _emf(theta)
        out = estimator.step(ea, eb, 1e-4)
        worst = max(worst, abs(wrap_angle_pi_closed(out.theta_hat - theta)))
    assert worst < 1e-12
    assert out.omega_hat is None


def test_arctan_wraps_to_the_closed_interval() -> None:
    estimator = ArctanEstimator()
    # theta_e = +pi maps to +pi (not -pi)
    ea, eb = _emf(math.pi - 1e-15)
    assert estimator.step(ea, eb, 1e-4).theta_hat == pytest.approx(math.pi)


def test_arctan_rejects_nonpositive_dt() -> None:
    with pytest.raises(ValueError):
        ArctanEstimator().step(1.0, 1.0, 0.0)


# --- QPLL ---------------------------------------------------------------------


def test_qpll_zero_steady_error_for_constant_speed() -> None:
    """Constant omega: the Q-PLL locks with zero steady angle error (<1e-3 rad).

    Settling: w_n = 125.66 rad/s, zeta = 0.707 -> tau ~ 1/(zeta w_n) = 11 ms; the
    0.5 s run is ~45 tau.
    """
    qpll = QPLL(_qpll_params(), initial_theta=0.0)
    dt, omega = 1e-4, OMEGA_1000
    err = 0.0
    for k in range(5000):
        theta = omega * k * dt
        out = qpll.step(*_emf(theta), dt)
        if k >= 4000:
            err = max(err, abs(wrap_angle_pi_closed(out.theta_hat - theta)))
    assert err < 1e-3


def test_qpll_ramp_lag_matches_analytical() -> None:
    """Speed ramp: steady lag = domega/dt / (Ki*A) = domega/dt / w_n^2.

    With A = |E|(1000 rpm), Ki*A = w_n^2 = 15791 rad/s^2. Ramp alpha = 2000 rad/s^2
    -> lag = 0.1267 rad. Tolerance 5 % (forward-Euler discretization).
    """
    params = _qpll_params()
    qpll = QPLL(params, initial_theta=0.0)
    dt, omega0, alpha = 1e-4, OMEGA_1000, 2000.0
    lags = []
    for k in range(6000):
        t = k * dt
        theta = omega0 * t + 0.5 * alpha * t * t
        out = qpll.step(*_emf(theta), dt)
        if k >= 5000:
            lags.append(wrap_angle_pi_closed(theta - out.theta_hat))
    measured = sum(lags) / len(lags)
    predicted = alpha / (params.ki * AMPLITUDE_1000)
    assert measured == pytest.approx(predicted, rel=0.05)
    assert measured == pytest.approx(alpha / (TWO_PI * 20.0) ** 2, rel=0.05)


def test_qpll_error_has_restoring_sign() -> None:
    """delta_e = |E| sin(theta - theta_hat); theta > theta_hat -> delta_e > 0."""
    qpll = QPLL(_qpll_params(), initial_theta=0.3)
    ea, eb = _emf(0.4)  # theta = 0.4 > theta_hat = 0.3
    assert qpll.error(ea, eb) > 0.0
    out = qpll.step(ea, eb, 1e-4)
    assert out.omega_hat > 0.0


def test_qpll_converges_from_90_degree_offset() -> None:
    qpll = QPLL(_qpll_params(), initial_theta=math.pi / 2.0)
    dt, omega = 1e-4, OMEGA_1000
    for k in range(5000):
        theta = omega * k * dt
        out = qpll.step(*_emf(theta), dt)
    assert abs(wrap_angle_pi_closed(out.theta_hat - theta)) < 1e-3


def test_qpll_loop_gain_is_proportional_to_amplitude() -> None:
    """With normalization OFF, delta_e = |E| sin(theta - theta_hat) ~ |E| * offset."""
    theta, offset = 0.5, 0.05
    params = _qpll_params()
    e_small = QPLL(params, initial_theta=theta - offset)
    e_big = QPLL(params, initial_theta=theta - offset)
    ea_s, eb_s = _emf(theta, AMPLITUDE_1000)
    ea_b, eb_b = _emf(theta, 2.0 * AMPLITUDE_1000)
    ratio = e_big.error(ea_b, eb_b) / e_small.error(ea_s, eb_s)
    assert ratio == pytest.approx(2.0, rel=1e-9)


def test_qpll_normalization_option_removes_amplitude_dependence() -> None:
    theta, offset = 0.5, 0.05
    norm = QPLLParameters(_qpll_params().kp, _qpll_params().ki, normalize=True)
    e_small = QPLL(norm, initial_theta=theta - offset)
    e_big = QPLL(norm, initial_theta=theta - offset)
    ea_s, eb_s = _emf(theta, AMPLITUDE_1000)
    ea_b, eb_b = _emf(theta, 2.0 * AMPLITUDE_1000)
    ratio = e_big.error(ea_b, eb_b) / e_small.error(ea_s, eb_s)
    assert ratio == pytest.approx(1.0, rel=1e-12)


def test_qpll_rejects_nonpositive_dt() -> None:
    with pytest.raises(ValueError):
        QPLL(_qpll_params()).step(1.0, 1.0, -1e-4)


# --- determinism / reset ------------------------------------------------------


def test_estimators_are_deterministic_and_reset() -> None:
    def run() -> list[float]:
        qpll = QPLL(_qpll_params(), initial_theta=0.0)
        return [qpll.step(*_emf(OMEGA_1000 * k * 1e-4), 1e-4).theta_hat for k in range(300)]

    assert run() == run()
    qpll = QPLL(_qpll_params(), initial_theta=0.0)
    first = qpll.step(*_emf(0.2), 1e-4).theta_hat
    qpll.reset(theta=0.0)
    assert qpll.theta_hat == 0.0
    assert qpll.step(*_emf(0.2), 1e-4).theta_hat == first


# --- structure ----------------------------------------------------------------


def test_estimation_imports_no_plant_observer_filter_fuzzy() -> None:
    for name in ("estimation", "estimation.estimators"):
        module = importlib.import_module(f"pmsm_sensorless.{name}")
        for value in vars(module).values():
            if isinstance(value, types.ModuleType):
                root = value.__name__.split(".")[1] if value.__name__.startswith("pmsm_sensorless.") else ""
                assert root not in {"plant", "observers", "filters", "fuzzy"}, value.__name__


def test_step_signature_is_back_emf_and_timestep() -> None:
    for cls in (ArctanEstimator, QPLL):
        params = inspect.signature(cls.step).parameters
        assert list(params) == ["self", "e_alpha", "e_beta", "dt"], cls


# --- configuration ------------------------------------------------------------


@pytest.fixture
def estimator_assumptions(config_dir):
    return config.load_config("estimator_assumptions", directory=config_dir)


def test_qpll_parameters_from_config(estimator_assumptions) -> None:
    params = QPLLParameters.from_configs(estimator_assumptions, design_amplitude_v=AMPLITUDE_1000)
    w_n = TWO_PI * 20.0
    assert params.kp == pytest.approx(2.0 * 0.707 * w_n / AMPLITUDE_1000)
    assert params.ki == pytest.approx(w_n * w_n / AMPLITUDE_1000)
    assert params.normalize is False


def test_qpll_parameters_null_assumption_raises(estimator_assumptions) -> None:
    import copy

    broken = copy.deepcopy(estimator_assumptions)
    broken["parameters"]["qpll.damping_zeta"]["value"] = None
    with pytest.raises(config.ConfigError):
        QPLLParameters.from_configs(broken, design_amplitude_v=AMPLITUDE_1000)
