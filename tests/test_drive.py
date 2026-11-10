"""Phase 06a: closed-loop FOC drive with encoder (true-angle) feedback.

Analytical tests with justified tolerances, a structure test (no plant-controller
coupling to observers/filters/fuzzy/estimation), determinism, and the Phase 06a
GATE. The current loop is checked against the exact discrete (ZOH plant + discrete
PI + one-sample delay) model and against the continuous ``alpha_c/(s+alpha_c)``
ideal; the speed loop against the designed second-order response.
"""

from __future__ import annotations

import importlib
import inspect
import math
import types

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.plant.load_models import ConstantLoadTorque
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState
from pmsm_sensorless.simulation.drive import (
    DriveParameters,
    DriveSample,
    TrueSource,
    run_drive,
)

TWO_PI = 2.0 * math.pi
OMEGA_1000_M = 1000.0 * TWO_PI / 60.0          # mechanical rad/s at 1000 rpm
I_RATED = 7.7 / (1.5 * 4 * 0.1326)             # 9.678 A


@pytest.fixture(scope="module")
def params(config_dir) -> DriveParameters:
    return DriveParameters.from_configs(
        config.load_config("drive", directory=config_dir),
        config.load_config("drive_assumptions", directory=config_dir),
        config.load_config("pmsm", directory=config_dir),
    )


@pytest.fixture(scope="module")
def prescribed_plant(config_dir) -> PMSMPlant:
    return PMSMPlant(
        PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir))
    )


@pytest.fixture(scope="module")
def mechanical_plant(config_dir) -> PMSMPlant:
    return PMSMPlant(
        PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir)),
        integrate_mechanics=True,
        viscous_friction=0.0,
    )


def _discrete_current_step(params: DriveParameters, i_ref: float, n: int) -> np.ndarray:
    """Exact discrete model: ZOH plant + discrete PI + one-sample delay.

    ``i[k+1] = a i[k] + b v[k-1]`` with ``a = exp(-Rs Ts/Ld)``, ``b = (1-a)/Rs``;
    ``v[k] = Kp (i_ref - i[k]) + I[k]`` and ``I[k+1] = I[k] + Ki Ts (i_ref - i[k])``.
    """
    a = math.exp(-params.stator_resistance * params.ts_current / params.ld)
    b = (1.0 - a) / params.stator_resistance
    i, integral, v_prev = 0.0, 0.0, 0.0
    out = np.empty(n)
    for k in range(n):
        out[k] = i
        err = i_ref - i
        v = params.kp_current * err + integral
        integral += params.ki_current * err * params.ts_current
        i = a * i + b * v_prev
        v_prev = v
    return out


# --- gains / config -----------------------------------------------------------


def test_gains_follow_the_design_formulas(params: DriveParameters) -> None:
    assert params.torque_constant == pytest.approx(0.7956, rel=1e-3)
    assert params.kp_current == pytest.approx(TWO_PI * 500.0 * params.ld)
    assert params.ki_current == pytest.approx(TWO_PI * 500.0 * params.stator_resistance)
    assert params.kp_speed == pytest.approx(
        2.0 * 1.0 * TWO_PI * 50.0 * params.inertia / params.torque_constant
    )
    assert params.ki_speed == pytest.approx(
        (TWO_PI * 50.0) ** 2 * params.inertia / params.torque_constant
    )
    assert params.v_limit == pytest.approx(311.0 / math.sqrt(3.0))
    assert params.i_q_clamp == pytest.approx(2.0 * I_RATED, rel=1e-3)
    assert params.speed_divider == 10
    assert params.n_substeps >= 4


def test_null_assumption_raises_config_error(config_dir) -> None:
    import copy

    drive = config.load_config("drive", directory=config_dir)
    assumptions = config.load_config("drive_assumptions", directory=config_dir)
    pmsm = config.load_config("pmsm", directory=config_dir)
    broken = copy.deepcopy(assumptions)
    broken["parameters"]["current_loop.bandwidth_alpha_c_hz"]["value"] = None
    with pytest.raises(config.ConfigError):
        DriveParameters.from_configs(drive, broken, pmsm)


def test_cannot_set_speed_faster_than_current(params: DriveParameters) -> None:
    import dataclasses

    with pytest.raises(ValueError):
        dataclasses.replace(params, speed_loop_hz=20000.0)
    with pytest.raises(ValueError):
        dataclasses.replace(params, n_substeps=2)


# --- current loop -------------------------------------------------------------


def test_current_step_matches_exact_discrete_model(params, prescribed_plant) -> None:
    """The current loop equals the ZOH-plant + discrete-PI + 1-sample-delay model.

    The plant is a single RL pole at ``Rs/Ld``; at fixed speed the decoupling and
    back-EMF enter as a constant, so the plain current step is modelled exactly.
    Measured at zero speed (no back-EMF disturbance; D-047).
    """
    i_ref = 3.0
    n = 100
    result = run_drive(
        prescribed_plant, params, duration=n * params.ts_current,
        i_q_ref=lambda t: i_ref if t >= 0.0005 else 0.0, omega_e_fixed=0.0,
    )
    model = _discrete_current_step(params, i_ref, n - 5)  # 5 = step index
    assert np.max(np.abs(result.i_q[5:] - model)) < 1e-6


def test_current_step_first_order_shape_bounds(params, prescribed_plant) -> None:
    """First-order ``alpha_c/(s+alpha_c)`` with the delay, within justified bounds.

    Continuous ideal: rise time 10-90% = ln(9)/alpha_c = 699 us plus 1.5 Ts = 150 us
    dead time -> 849 us. The realised *discrete* loop at ``alpha_c Ts = 0.31`` is
    faster (the discrete PI zero only approximately cancels the plant pole), so the
    band brackets both; overshoot stays within a few percent and the steady error is
    zero (the PI zero cancels the plant pole exactly in continuous time).
    """
    i_ref = 3.0
    result = run_drive(
        prescribed_plant, params, duration=0.01,
        i_q_ref=lambda t: i_ref if t >= 0.0005 else 0.0, omega_e_fixed=0.0,
    )
    t, i_q = result.t, result.i_q
    steady = float(np.mean(i_q[t >= 0.008]))
    assert steady == pytest.approx(i_ref, rel=1e-3)
    lo = t[np.where(i_q >= 0.1 * steady)[0][0]]
    hi = t[np.where(i_q >= 0.9 * steady)[0][0]]
    rise = hi - lo
    ideal = math.log(9.0) / (TWO_PI * params.alpha_c_hz)
    assert 0.3 * ideal <= rise <= 1.4 * ideal, rise
    assert 0.0 <= (np.max(i_q) - steady) / steady < 0.05


# --- speed loop ---------------------------------------------------------------


def test_speed_step_settles_per_alpha_w(mechanical_plant, params) -> None:
    """A small speed step settles within a few closed-loop time constants.

    Closed loop ``s^2 + 2 alpha_w s + alpha_w^2`` (zeta = 1): ~1/(alpha_w) = 3.2 ms.
    A 0 -> 100 rpm step is well inside the i_q clamp so the response is linear; the
    speed must reach the reference with zero steady error.
    """
    result = run_drive(mechanical_plant, params, duration=0.2, speed_ref_rpm=100.0)
    err = (result.omega_m - result.omega_m_ref) * 60.0 / TWO_PI
    tau = 1.0 / (TWO_PI * params.alpha_w_hz)
    settle = 0.05
    assert np.max(np.abs(err[result.t >= settle])) < 0.5        # rpm
    assert abs(np.mean(err[result.t >= 0.15])) < 0.05
    bad = np.where(np.abs(err) > 1.0)[0]                        # to +-1 rpm
    assert result.t[bad[-1] + 1] < 6.0 * tau


def test_speed_error_is_zero_with_viscous_friction(config_dir, params) -> None:
    """With b > 0 (test-only) the integral action still drives the speed error to 0.

    Viscous friction adds a speed-dependent load; the steady i_q becomes
    ``b omega_e / (p Kt)`` but the speed error is still zero.
    """
    friction = 0.01
    plant = PMSMPlant(
        PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir)),
        integrate_mechanics=True, viscous_friction=friction,
    )
    result = run_drive(plant, params, duration=0.2, speed_ref_rpm=100.0)
    err = (result.omega_m - result.omega_m_ref) * 60.0 / TWO_PI
    assert abs(np.mean(err[result.t >= 0.15])) < 0.05
    omega_e = 4.0 * OMEGA_1000_M / 10.0  # 100 rpm electrical
    expected_iq = friction * omega_e / (4.0 * params.torque_constant)
    assert np.mean(result.i_q[result.t >= 0.15]) == pytest.approx(expected_iq, rel=0.05)


def test_steady_state_currents_and_power_balance(mechanical_plant, params) -> None:
    """At rated load and 1000 rpm: i_q = T_L/Kt, i_d = 0, power balance < 1 %."""
    load = 7.7
    result = run_drive(
        mechanical_plant, params, duration=1.0, speed_ref_rpm=1000.0,
        load_torque=ConstantLoadTorque(load),
        initial_state=PMSMState(0.0, 0.0, 4.0 * OMEGA_1000_M, 0.0),
    )
    m = result.t >= 0.8
    assert np.mean(result.i_q[m]) == pytest.approx(load / params.torque_constant, rel=0.01)
    assert np.max(np.abs(result.i_d[m])) < 0.01 * I_RATED
    p_in = 1.5 * np.mean(result.v_d[m] * result.i_d[m] + result.v_q[m] * result.i_q[m])
    p_cu = 1.5 * params.stator_resistance * np.mean(result.i_d[m] ** 2 + result.i_q[m] ** 2)
    p_mech = np.mean(result.torque_e[m] * result.omega_m[m])
    assert abs(p_in - p_cu - p_mech) / abs(p_in) < 0.01
    assert not np.any(result.saturated[m])       # no voltage-limit saturation


# --- voltage saturation -------------------------------------------------------


def test_voltage_saturation_has_no_windup(prescribed_plant, params) -> None:
    """A 50 ms saturated interval leaves bounded overshoot and a clean recovery.

    At zero speed a 1000 A command demands ~505 V >> V_dc/sqrt3; the output pins at
    the limit and the integrator freezes (conditional integration). After the
    command is released the current returns to ~0 with small overshoot.
    """
    result = run_drive(
        prescribed_plant, params, duration=0.2,
        i_q_ref=lambda t: 1000.0 if 0.01 <= t < 0.06 else 0.0, omega_e_fixed=0.0,
    )
    assert np.max(result.u_cmd_magnitude) <= params.v_limit + 1e-9
    assert np.mean(result.saturated[(result.t >= 0.01) & (result.t < 0.06)]) > 0.99
    after = result.t > 0.0605
    saturated_value = params.v_limit / params.stator_resistance
    assert np.min(result.i_q[after]) > -0.1 * saturated_value     # bounded undershoot
    tail = result.t >= 0.15
    assert np.max(np.abs(result.i_q[tail])) < 0.02 * saturated_value


# --- structure ----------------------------------------------------------------


def test_drive_imports_no_observer_filter_fuzzy_estimation() -> None:
    for name in ("simulation.drive",):
        module = importlib.import_module(f"pmsm_sensorless.{name}")
        for value in vars(module).values():
            if isinstance(value, types.ModuleType):
                root = value.__name__.split(".")[1] if value.__name__.startswith("pmsm_sensorless.") else ""
                assert root not in {"observers", "filters", "fuzzy", "estimation"}, value.__name__


def test_true_source_is_the_only_ground_truth_path(prescribed_plant, params) -> None:
    """TrueSource returns the plant truth; the source is pluggable."""
    state = PMSMState(0.1, 0.2, 3.0, 0.4)
    sample = DriveSample(0.0, 1e-4, 0.1, 0.2, 0.0, 0.0)
    assert TrueSource()(sample, state) == (state.theta_e, state.omega_e)

    class _FixedSource:
        name = "fixed"

        def __call__(self, sample, truth):
            return 1.234, 5.678

    result = run_drive(
        prescribed_plant, params, duration=0.01, i_q_ref=0.0,
        omega_e_fixed=1000.0, source=_FixedSource(),
    )
    # the controller used the source angle (fixed): i_d/i_q are rotated by it
    assert result.theta_e[0] == 0.0            # true plant angle is untouched
    assert np.all(np.isfinite(result.i_q))


def test_run_drive_signature_has_no_ground_truth_estimator() -> None:
    names = set(inspect.signature(run_drive).parameters)
    assert names == {
        "plant", "parameters", "ts", "duration", "load_torque", "source",
        "speed_ref_rpm", "i_q_ref", "omega_e_fixed", "initial_state", "angle_advance",
    }


def test_drive_is_deterministic(mechanical_plant, params) -> None:
    def run():
        return run_drive(mechanical_plant, params, duration=0.2, speed_ref_rpm=1000.0)

    first, second = run(), run()
    for name in ("i_alpha", "i_q", "u_alpha", "omega_m", "theta_e"):
        assert np.array_equal(getattr(first, name), getattr(second, name))


# --- GATE ---------------------------------------------------------------------


def test_gate_speed_step_and_rated_load(mechanical_plant, params) -> None:
    """GATE (Phase 06a Step 2): speed step 0 -> 1000 rpm no load, and rated load
    step at 1000 rpm.

    Requires steady speed error < 0.1 rpm, |i_d| < 3 % I_rated, i_q within 1 % of
    T_L/Kt, and no voltage-limit saturation after the transient.
    """
    step = run_drive(mechanical_plant, params, duration=1.0, speed_ref_rpm=1000.0)
    m = step.t >= 0.8
    err = (step.omega_m - step.omega_m_ref) * 60.0 / TWO_PI
    assert np.max(np.abs(err[m])) < 0.1
    assert np.max(np.abs(step.i_d[m])) < 0.03 * I_RATED
    assert not np.any(step.saturated[m])

    load = run_drive(
        mechanical_plant, params, duration=1.0, speed_ref_rpm=1000.0,
        load_torque=ConstantLoadTorque(7.7),
        initial_state=PMSMState(0.0, 0.0, 4.0 * OMEGA_1000_M, 0.0),
    )
    m2 = load.t >= 0.8
    err2 = (load.omega_m - load.omega_m_ref) * 60.0 / TWO_PI
    assert np.max(np.abs(err2[m2])) < 0.1
    assert np.max(np.abs(load.i_d[m2])) < 0.03 * I_RATED
    assert np.mean(load.i_q[m2]) == pytest.approx(7.7 / params.torque_constant, rel=0.01)
    assert not np.any(load.saturated[m2])
