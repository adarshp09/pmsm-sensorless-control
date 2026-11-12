"""Tests for the conventional (sign) sliding-mode observer, Eqs.(3)-(7), pp.2-3."""

from __future__ import annotations

import copy
import importlib
import inspect
import types

import pytest

from pmsm_sensorless import config
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO, sign_switching
from pmsm_sensorless.observers.smo import (
    SMOObserverParameters,
    SMOObserverState,
    per_unit_bases,
)


@pytest.fixture
def pmsm_config(config_dir):
    return config.load_config("pmsm", directory=config_dir)


@pytest.fixture
def observer_config(config_dir):
    return config.load_config("observer", directory=config_dir)


# --- switching function, Eq.(4) ----------------------------------------------


@pytest.mark.parametrize(
    "s, expected", [(-3.0, -1.0), (-0.1, -1.0), (0.0, 0.0), (0.1, 1.0), (7.0, 1.0)]
)
def test_sign_switching_for_negative_zero_positive(s: float, expected: float) -> None:
    assert sign_switching(s) == expected


# --- manual single-step example ----------------------------------------------


def test_single_step_matches_hand_calculation() -> None:
    """Rs=0.5, Ld=0.01, k=10, i_hat=0, i=1, u=0, dt=1e-3.

    Sample-instant error S = 0 - 1 = -1 -> sign = -1
    d(i_hat)/dt = (0 - 0.5*0)/0.01 - (10/0.01)*(-1) = 1000
    i_hat_new = 0 + 1e-3*1000 = 1.0
    Reported s_alpha is the *sample-instant* error (-1) and the reported back-EMF is
    the applied injection k*sign(S) = -10 V (D-025); NOT the post-update S=0.
    """
    observer = ConventionalSMO(
        SMOObserverParameters(stator_resistance=0.5, ld=0.01, switching_gain_k=10.0),
        SMOObserverState(0.0, 0.0),
    )
    out = observer.step(u_alpha=0.0, u_beta=0.0, i_alpha=1.0, i_beta=0.0, dt=1e-3)
    assert out.i_alpha_hat == pytest.approx(1.0)
    assert out.i_beta_hat == pytest.approx(0.0)
    assert out.s_alpha == pytest.approx(-1.0)
    assert out.back_emf_alpha == pytest.approx(-10.0)


def test_injection_polarity_opposes_sliding_error() -> None:
    """A positive sliding error must produce a negative current correction."""
    observer = ConventionalSMO(
        SMOObserverParameters(stator_resistance=0.5, ld=0.01, switching_gain_k=10.0),
        SMOObserverState(2.0, 0.0),  # i_hat > i  => S > 0
    )
    out = observer.step(0.0, 0.0, 0.0, 0.0, 1e-3)
    assert out.i_alpha_hat < 2.0  # pulled back toward the measurement
    assert out.s_alpha > 0.0
    assert out.back_emf_alpha == pytest.approx(10.0)  # +k*sign(S), S>0


# --- back-EMF reconstruction, Eq.(7) -----------------------------------------


def test_back_emf_reconstruction_averages_to_true_emf() -> None:
    """Feeding the equilibrium current for a known E, <k*sign(S)> reconstructs E.

    The tolerance is in **volts** (E and E_hat are back-EMFs). The measured
    duty-cycle bias of the forward-Euler sliding mode here is ~0.02 V, so 0.1 V
    (~1.7 % of |E| = 6 V, ~5x the bias) is a justified bound for the discretized
    switching, not a placeholder.
    """
    rs, ld, k = 0.5, 0.01, 20.0
    emf = (6.0, 0.0)
    i_at_equilibrium = (-emf[0] / rs, -emf[1] / rs)  # u = 0 equilibrium of Eq.(1)
    observer = ConventionalSMO(SMOObserverParameters(rs, ld, k), SMOObserverState(0.0, 0.0))
    samples = []
    for n in range(20000):
        out = observer.step(0.0, 0.0, i_at_equilibrium[0], i_at_equilibrium[1], 1e-4)
        if n >= 15000:
            samples.append(out.back_emf_alpha)
    assert sum(samples) / len(samples) == pytest.approx(emf[0], abs=0.1)


# --- state, determinism, validation ------------------------------------------


def test_state_is_persistent_and_reset_works() -> None:
    observer = ConventionalSMO(SMOObserverParameters(0.5, 0.01, 10.0))
    first = observer.step(0.0, 0.0, 1.0, 0.0, 1e-3)
    assert observer.state.i_alpha_hat == first.i_alpha_hat
    second = observer.step(0.0, 0.0, 0.0, 0.0, 1e-3)
    assert second.i_alpha_hat != first.i_alpha_hat  # state carried forward
    observer.reset()
    assert (observer.state.i_alpha_hat, observer.state.i_beta_hat) == (0.0, 0.0)


def test_update_is_deterministic() -> None:
    def run() -> list[float]:
        observer = ConventionalSMO(SMOObserverParameters(0.5, 0.01, 10.0))
        return [observer.step(1.0, -2.0, 0.5, 0.5, 1e-4).i_alpha_hat for _ in range(200)]

    assert run() == run()


def test_nonpositive_timestep_is_rejected() -> None:
    observer = ConventionalSMO(SMOObserverParameters(0.5, 0.01, 10.0))
    with pytest.raises(ValueError):
        observer.step(0.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        observer.step(0.0, 0.0, 0.0, 0.0, -1e-4)


def test_invalid_parameters_are_rejected() -> None:
    with pytest.raises(ValueError):
        SMOObserverParameters(0.0, 0.01, 10.0)
    with pytest.raises(ValueError):
        SMOObserverParameters(0.5, -0.01, 10.0)
    with pytest.raises(ValueError):
        SMOObserverParameters(0.5, 0.01, 0.0)


# --- configuration ------------------------------------------------------------


def test_from_config_reads_and_converts_parameters(pmsm_config, observer_config) -> None:
    observer = ConventionalSMO.from_config(pmsm_config, observer_config)
    v_base, _ = per_unit_bases(pmsm_config)
    assert observer.parameters.stator_resistance == pytest.approx(0.505)
    assert observer.parameters.ld == pytest.approx(0.00147)
    assert observer.parameters.switching_gain_k == pytest.approx(1.0 * v_base)


def test_from_config_without_scaling_keeps_raw_values(pmsm_config, observer_config) -> None:
    observer = ConventionalSMO.from_config(pmsm_config, observer_config, per_unit_scaling=False)
    assert observer.parameters.switching_gain_k == pytest.approx(1.0)


def test_from_config_rejects_unresolved_gain(pmsm_config, observer_config) -> None:
    broken = copy.deepcopy(observer_config)
    broken["parameters"]["smo.switching_gain_k"]["value"] = None
    with pytest.raises(config.ConfigError):
        ConventionalSMO.from_config(pmsm_config, broken)


# --- no ground-truth dependency ----------------------------------------------


def test_observer_modules_do_not_import_the_plant() -> None:
    for name in ("smo", "conventional_smo", "sigmoid_smo"):
        module = importlib.import_module(f"pmsm_sensorless.observers.{name}")
        for value in vars(module).values():
            if isinstance(value, types.ModuleType):
                assert not value.__name__.startswith("pmsm_sensorless.plant")


def test_step_signature_takes_only_voltages_currents_and_timestep() -> None:
    signature = inspect.signature(ConventionalSMO.step)
    forbidden = {"theta", "theta_e", "omega", "omega_e", "back_emf", "state_truth"}
    assert forbidden.isdisjoint(signature.parameters)
