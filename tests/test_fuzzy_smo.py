"""Tests for the FLC-adaptive observer (FuzzySMO)."""

from __future__ import annotations

import importlib
import inspect
import types

import pytest

from pmsm_sensorless.fuzzy.engine import FuzzyEngine
from pmsm_sensorless.observers.fuzzy_smo import FuzzySMO
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO
from pmsm_sensorless.observers.smo import SMOObserverParameters, SMOObserverState


class _ConstantEngine:
    """Stub FLC returning a constant ``(kF, aF)``."""

    def __init__(self, value: float) -> None:
        self.value = value
        self.parameters = types.SimpleNamespace(ke=1.0, kec=0.0)

    def evaluate(self, s_n: float, s_dot_n: float) -> tuple[float, float]:
        return self.value, self.value


def _params(k: float = 10.0, a: float = 2.0) -> SMOObserverParameters:
    return SMOObserverParameters(0.5, 0.01, k, sigmoid_slope_a=a)


def test_forced_unit_gain_reproduces_fixed_sigmoid_bit_identically() -> None:
    fixed = SigmoidSMO(_params(), SMOObserverState(0.1, 0.2))
    fuzzy = FuzzySMO(_params(), _ConstantEngine(1.0), SMOObserverState(0.1, 0.2))
    for _ in range(100):
        out_fixed = fixed.step(1.0, -0.5, 0.01, 0.02, 1e-4)
        out_fuzzy = fuzzy.step(1.0, -0.5, 0.01, 0.02, 1e-4)
        assert out_fixed.i_alpha_hat == out_fuzzy.i_alpha_hat
        assert out_fixed.i_beta_hat == out_fuzzy.i_beta_hat
        assert out_fixed.s_alpha == out_fuzzy.s_alpha
        assert out_fixed.back_emf_alpha == out_fuzzy.back_emf_alpha
        assert out_fixed.back_emf_beta == out_fuzzy.back_emf_beta


def test_constant_gain_reproduces_scaled_fixed_observer() -> None:
    # FuzzySMO with kF=aF=0.5 must equal a fixed SigmoidSMO with k=0.5*k0, a=0.5*a0.
    fuzzy = FuzzySMO(_params(10.0, 2.0), _ConstantEngine(0.5), SMOObserverState(0.1, 0.2))
    fixed = SigmoidSMO(_params(5.0, 1.0), SMOObserverState(0.1, 0.2))
    for _ in range(100):
        out_fuzzy = fuzzy.step(1.0, -0.5, 0.01, 0.02, 1e-4)
        out_fixed = fixed.step(1.0, -0.5, 0.01, 0.02, 1e-4)
        assert out_fuzzy.i_alpha_hat == pytest.approx(out_fixed.i_alpha_hat, abs=1e-12)
        assert out_fuzzy.back_emf_alpha == pytest.approx(out_fixed.back_emf_alpha, abs=1e-12)


def test_missing_slope_is_rejected() -> None:
    with pytest.raises(ValueError):
        FuzzySMO(SMOObserverParameters(0.5, 0.01, 10.0), _ConstantEngine(1.0))


def test_real_engine_produces_finite_adapted_gains(config_dir) -> None:
    engine = FuzzyEngine.from_yaml(config_dir)
    observer = FuzzySMO(_params(), engine)
    for _ in range(200):
        out = observer.step(1.0, -0.5, 0.01, 0.02, 1e-4)
        assert abs(out.i_alpha_hat) < 1e6
    kf_alpha, kf_beta = observer.last_kf
    af_alpha, af_beta = observer.last_af
    for value in (kf_alpha, kf_beta, af_alpha, af_beta):
        assert 0.0 <= value <= 1.0
    # k_t = kF*k0, a_t = aF*a0
    assert observer.last_k[0] == pytest.approx(kf_alpha * 10.0)
    assert observer.last_a[0] == pytest.approx(af_alpha * 2.0)


def test_reset_clears_the_sdot_memory() -> None:
    engine = _ConstantEngine(1.0)
    used = FuzzySMO(_params(), engine, SMOObserverState(0.5, 0.5))
    for _ in range(10):
        used.step(1.0, -0.5, 0.01, 0.02, 1e-4)
    used.reset(SMOObserverState(0.5, 0.5))
    fresh = FuzzySMO(_params(), engine, SMOObserverState(0.5, 0.5))
    assert used.step(1.0, -0.5, 0.01, 0.02, 1e-4) == fresh.step(1.0, -0.5, 0.01, 0.02, 1e-4)


def test_determinism() -> None:
    engine = _ConstantEngine(1.0)

    def run() -> list[float]:
        observer = FuzzySMO(_params(), engine, SMOObserverState(0.1, 0.2))
        return [observer.step(1.0, -0.5, 0.01, 0.02, 1e-4).i_alpha_hat for _ in range(200)]

    assert run() == run()


def test_no_ground_truth_dependency() -> None:
    signature = inspect.signature(FuzzySMO.step)
    assert set(signature.parameters) == {"self", "u_alpha", "u_beta", "i_alpha", "i_beta", "dt"}
    module = importlib.import_module("pmsm_sensorless.observers.fuzzy_smo")
    for value in vars(module).values():
        if isinstance(value, types.ModuleType):
            assert not value.__name__.startswith("pmsm_sensorless.plant")
