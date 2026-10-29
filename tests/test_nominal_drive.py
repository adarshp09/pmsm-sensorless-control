"""Tests for the Phase 04c nominal-drive generator (open-loop ideal FOC).

Plant-side only: the generator may use ground truth, but it never reads an
observer. The GATE test checks that the stable fixed sigmoid at Ts = 10 us
reconstructs the true back-EMF within 2 % gain and 1 deg.
"""

from __future__ import annotations

import importlib
import inspect
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters
from pmsm_sensorless.simulation.nominal_drive import (
    NominalDriveResult,
    electrical_speed_e,
    nominal_drive_rollout,
    rated_current_from_torque,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

from validation_metrics import vector_metrics  # noqa: E402

RPM, RAMP_S, DURATION_S = 1000.0, 1.5, 2.0
DT_GATE = 10e-6


@pytest.fixture(scope="module")
def pmsm_config(config_dir):
    return config.load_config("pmsm", directory=config_dir)


@pytest.fixture(scope="module")
def observer_config(config_dir):
    return config.load_config("observer", directory=config_dir)


@pytest.fixture(scope="module")
def plant(pmsm_config) -> PMSMPlant:
    return PMSMPlant(PMSMParameters.from_config(pmsm_config))


@pytest.fixture(scope="module")
def i_rated(pmsm_config) -> float:
    return rated_current_from_torque(
        config.resolve_parameter(pmsm_config, "rating.rated_torque", unit="N.m"),
        int(config.resolve_parameter(pmsm_config, "machine.pole_pairs", unit="-")),
        config.resolve_parameter(pmsm_config, "machine.pm_flux_linkage_psi_f", unit="Wb"),
    )


# --- rated current ------------------------------------------------------------


def test_rated_current_formula_matches_derivation(pmsm_config, i_rated) -> None:
    torque = config.resolve_parameter(pmsm_config, "rating.rated_torque", unit="N.m")
    p = config.resolve_parameter(pmsm_config, "machine.pole_pairs", unit="-")
    psi = config.resolve_parameter(pmsm_config, "machine.pm_flux_linkage_psi_f", unit="Wb")
    assert i_rated == pytest.approx(torque / (1.5 * p * psi))
    assert i_rated == pytest.approx(9.678, abs=5e-3)  # 7.7 / (1.5*4*0.1326)


def test_rated_current_rejects_nonpositive() -> None:
    with pytest.raises(ValueError):
        rated_current_from_torque(0.0, 4, 0.1326)
    with pytest.raises(ValueError):
        rated_current_from_torque(7.7, 4, 0.0)


def test_electrical_speed_conversion() -> None:
    assert electrical_speed_e(1000.0, 4) == pytest.approx(4 * 1000.0 * 2 * math.pi / 60.0)


# --- tracking (plant side) ----------------------------------------------------


def test_plant_current_tracks_reference_within_2_percent(plant, i_rated) -> None:
    """At Ts = 10 us the ZOH feedforward tracks i_ref well inside 2 % of I_q.

    Justification: the feedforward is the exact inverse of Eq.(1) evaluated at the
    plant's own angle, so the only residual is the zero-order hold of ``u`` over one
    step (plus RK4 truncation). Over one 10 us step the electrical angle advances
    ``omega_e Ts = 4.2e-3 rad`` (0.24 deg) at 1000 rpm, making the ZOH residual
    second order in ``omega_e Ts``; 2 % of I_q is therefore a generous bound.
    """
    result = nominal_drive_rollout(
        plant, i_q=i_rated, dt=DT_GATE, duration=DURATION_S, rpm_start=500.0, rpm_end=RPM
    )
    assert result.tracking_error_max_a <= 0.02 * i_rated


def test_initial_current_is_the_reference(plant, i_rated) -> None:
    result = nominal_drive_rollout(plant, i_q=i_rated, dt=1e-4, duration=0.05)
    assert result.i_alpha[0] == pytest.approx(result.i_ref_alpha[0])
    assert result.i_beta[0] == pytest.approx(result.i_ref_beta[0])
    assert np.hypot(result.i_alpha[0], result.i_beta[0]) == pytest.approx(i_rated)


def test_reference_is_in_phase_with_back_emf(plant, i_rated) -> None:
    """i_ref and E_true are parallel and q-axis: cross product ~0, i_q = I_q."""
    result = nominal_drive_rollout(plant, i_q=i_rated, dt=1e-4, duration=0.2)
    cross = result.i_ref_alpha * result.e_beta - result.i_ref_beta * result.e_alpha
    assert np.max(np.abs(cross)) < 1e-6 * i_rated * np.max(np.hypot(result.e_alpha, result.e_beta))


# --- structure: no observer input --------------------------------------------


def test_generator_does_not_import_observers_or_filters() -> None:
    module = importlib.import_module("pmsm_sensorless.simulation.nominal_drive")
    for value in vars(module).values():
        if isinstance(value, types.ModuleType):
            assert not value.__name__.startswith("pmsm_sensorless.observers")
            assert not value.__name__.startswith("pmsm_sensorless.filters")
            assert not value.__name__.startswith("pmsm_sensorless.fuzzy")


def test_rollout_signature_has_no_observer() -> None:
    signature = inspect.signature(nominal_drive_rollout)
    forbidden = {"observer", "smo", "estimator", "fll", "dsogi", "e_hat"}
    assert forbidden.isdisjoint(signature.parameters)
    # the plant is the only object-like argument and it is required positional
    assert "plant" in signature.parameters


# --- determinism --------------------------------------------------------------


def test_rollout_is_deterministic(plant, i_rated) -> None:
    def run() -> NominalDriveResult:
        return nominal_drive_rollout(plant, i_q=i_rated, dt=1e-4, duration=0.2)

    first, second = run(), run()
    for name in ("u_alpha", "i_alpha", "e_alpha", "theta_e", "omega_e"):
        assert np.array_equal(getattr(first, name), getattr(second, name))


# --- GATE: fixed sigmoid reconstructs E at Ts = 10 us -------------------------


def test_gate_sigmoid_reconstructs_back_emf_at_10us(plant, i_rated, pmsm_config, observer_config) -> None:
    """GATE (Phase 04c Step 1): stable fixed sigmoid, Ts = 10 us, I_q = I_rated.

    Raw E_hat fundamental gain within 2 % and phase within 1 deg of E_true in the
    steady window (8 whole electrical cycles at 1000 rpm).
    """
    result = nominal_drive_rollout(
        plant, i_q=i_rated, dt=DT_GATE, duration=DURATION_S, rpm_start=500.0, rpm_end=RPM
    )
    observer = SigmoidSMO.from_config(pmsm_config, observer_config)

    steps = len(result.t)
    ea = np.empty(steps)
    eb = np.empty(steps)
    for n in range(steps):
        out = observer.step(
            result.u_alpha[n], result.u_beta[n], result.i_alpha[n], result.i_beta[n], DT_GATE
        )
        ea[n], eb[n] = out.back_emf_alpha, out.back_emf_beta

    pole_pairs = plant.parameters.pole_pairs
    period = 1.0 / (pole_pairs * RPM / 60.0)
    mask = (result.t >= DURATION_S - 8.0 * period) & (result.t <= DURATION_S)
    vm = vector_metrics(
        ea[mask], eb[mask], result.e_alpha[mask], result.e_beta[mask], result.theta_e[mask]
    )
    assert abs(vm["fundamental_gain"] - 1.0) <= 0.02, vm
    assert abs(vm["fundamental_phase_deg"]) <= 1.0, vm
