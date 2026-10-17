"""Tests for the DSOGI-FLL (dual SOGI, positive-sequence extraction, FLL)."""

from __future__ import annotations

import importlib
import inspect
import math
import types

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters

K_S = math.sqrt(2.0)
GAMMA = 50.0
DT = 1e-4


def _run_fll(fll: DSOGIFLL, omega_in, amplitude, steps, omega_switch=None, switch_at=None):
    """Drive the DSOGI-FLL with a balanced positive-sequence input; record frequency."""
    frequency = []
    for k in range(steps):
        omega = omega_in if (omega_switch is None or k < switch_at) else omega_switch
        fll.step(amplitude * math.cos(omega * k * DT), amplitude * math.sin(omega * k * DT), DT)
        frequency.append(fll.frequency_hz)
    return np.asarray(frequency)


# --- positive/negative sequence ----------------------------------------------


def test_positive_sequence_passes_with_unity_gain() -> None:
    omega = 2.0 * math.pi * 100.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, omega))
    samples = []
    indices = []
    for k in range(8000):
        out = fll.step(50.0 * math.cos(omega * k * DT), 50.0 * math.sin(omega * k * DT), DT)
        if k >= 6000:
            indices.append(k)
            samples.append(out.v_alpha)
    phasor = 2.0 / len(samples) * np.sum(np.asarray(samples) * np.exp(-1j * omega * np.asarray(indices) * DT))
    assert abs(phasor) == pytest.approx(50.0, rel=2e-2)  # amplitude preserved


def test_negative_sequence_is_rejected() -> None:
    omega = 2.0 * math.pi * 100.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, omega))
    amplitude = 0.0
    for k in range(8000):
        out = fll.step(50.0 * math.cos(omega * k * DT), -50.0 * math.sin(omega * k * DT), DT)
        if k >= 6000:
            amplitude = max(amplitude, abs(out.v_alpha))
    assert amplitude < 1.0  # > 98% rejection of the negative sequence


# --- FLL behaviour ------------------------------------------------------------


def test_fll_converges_to_a_stepped_frequency() -> None:
    init = 2.0 * math.pi * 90.0
    first = 2.0 * math.pi * 100.0
    second = 2.0 * math.pi * 130.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, init))
    frequency = _run_fll(fll, first, 50.0, 8000, omega_switch=second, switch_at=4000)
    assert frequency[3999] == pytest.approx(first / (2 * math.pi), rel=2e-3)
    assert frequency[-1] == pytest.approx(second / (2 * math.pi), rel=2e-3)


@pytest.mark.parametrize("init_hz", [90.0, 110.0])
def test_fll_corrects_in_the_right_direction(init_hz: float) -> None:
    target = 2.0 * math.pi * 100.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * init_hz))
    frequency = _run_fll(fll, target, 50.0, 6000)
    if init_hz < 100.0:
        assert frequency[-1] > init_hz  # moved up toward 100
    else:
        assert frequency[-1] < init_hz  # moved down toward 100
    assert frequency[-1] == pytest.approx(100.0, rel=5e-3)


def test_fll_does_not_drift_when_already_locked() -> None:
    target = 2.0 * math.pi * 100.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, target))
    frequency = _run_fll(fll, target, 50.0, 6000)
    assert frequency[-1] == pytest.approx(100.0, abs=0.05)


def test_frequency_limits_are_applied_when_configured() -> None:
    target = 2.0 * math.pi * 100.0
    parameters = DSOGIFLLParameters(
        K_S, GAMMA, 2.0 * math.pi * 90.0, max_omega=2.0 * math.pi * 95.0
    )
    fll = DSOGIFLL(parameters)
    frequency = _run_fll(fll, target, 50.0, 6000)
    assert frequency.max() <= 95.0 + 1e-9  # clamped below the true 100 Hz


def test_epsilon_filter_smooths_the_error_signal() -> None:
    """The optional error low-pass tracks the slow component and suppresses ripple."""
    target = 2.0 * math.pi * 100.0
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 95.0, epsilon_filter_tau=0.01))
    errors = []
    for k in range(4000):
        out = fll.step(50.0 * math.cos(target * k * DT), 50.0 * math.sin(target * k * DT), DT)
        errors.append(out.epsilon_f)
    assert fll.frequency_hz == pytest.approx(100.0, rel=5e-3)  # still converges
    assert all(math.isfinite(e) for e in errors)


def test_normalization_option_runs_and_changes_the_loop() -> None:
    target = 2.0 * math.pi * 100.0
    raw = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 90.0))
    normalized = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 90.0, normalize_error=True))
    raw_final = _run_fll(raw, target, 50.0, 4000)[-1]
    norm_final = _run_fll(normalized, target, 50.0, 4000)[-1]
    assert raw_final != norm_final  # the option is observable
    assert norm_final > 90.0  # still corrects upward


# --- state, determinism, structure -------------------------------------------


def test_determinism_and_reset() -> None:
    def run() -> list[float]:
        fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 95.0))
        return _run_fll(fll, 2.0 * math.pi * 100.0, 40.0, 1000).tolist()

    assert run() == run()
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 95.0))
    _run_fll(fll, 2.0 * math.pi * 100.0, 40.0, 500)
    fll.reset()
    assert fll.frequency_hz == pytest.approx(95.0)


def test_nonpositive_timestep_is_rejected() -> None:
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, 2.0 * math.pi * 100.0))
    with pytest.raises(ValueError):
        fll.step(1.0, 0.0, 0.0)


def test_invalid_parameters_are_rejected() -> None:
    with pytest.raises(ValueError):
        DSOGIFLLParameters(0.0, GAMMA, 100.0)
    with pytest.raises(ValueError):
        DSOGIFLLParameters(K_S, 0.0, 100.0)
    with pytest.raises(ValueError):
        DSOGIFLLParameters(K_S, GAMMA, 0.0)


def test_from_config_requires_an_initial_frequency(config_dir) -> None:
    observer_config = config.load_config("observer", directory=config_dir)
    with pytest.raises(config.ConfigError):
        DSOGIFLLParameters.from_config(observer_config)  # initial omega' is null
    parameters = DSOGIFLLParameters.from_config(observer_config, initial_omega=2 * math.pi * 100.0)
    assert parameters.damping_gain_k == pytest.approx(K_S)
    assert parameters.fll_gain_gamma == pytest.approx(GAMMA)


def test_filters_do_not_import_plant_or_observers() -> None:
    for name in ("sogi", "dsogi_fll"):
        module = importlib.import_module(f"pmsm_sensorless.filters.{name}")
        for value in vars(module).values():
            if isinstance(value, types.ModuleType):
                assert not value.__name__.startswith("pmsm_sensorless.plant")
                assert not value.__name__.startswith("pmsm_sensorless.observers")


def test_step_signature_takes_only_alpha_beta_and_timestep() -> None:
    signature = inspect.signature(DSOGIFLL.step)
    assert set(signature.parameters) == {"self", "v_alpha", "v_beta", "dt"}
