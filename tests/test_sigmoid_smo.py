"""Tests for the sigmoid-switching SMO, Eq.(5), p.2 and its observer."""

from __future__ import annotations

import math

import pytest

from pmsm_sensorless.observers.conventional_smo import ConventionalSMO
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO, sigmoid_switching
from pmsm_sensorless.observers.smo import SMOObserverParameters, SMOObserverState


# --- the switching function, Eq.(5) ------------------------------------------


def test_sigmoid_matches_the_paper_expression() -> None:
    for s in (-2.0, -0.3, 0.0, 0.3, 2.0):
        for a in (0.5, 4.0):
            assert sigmoid_switching(s, a) == pytest.approx(2.0 / (1.0 + math.exp(-a * s)) - 1.0)


def test_sigmoid_saturates_to_plus_minus_one() -> None:
    assert sigmoid_switching(1e6, 4.0) == pytest.approx(1.0)
    assert sigmoid_switching(-1e6, 4.0) == pytest.approx(-1.0)


def test_sigmoid_is_stable_for_extreme_inputs() -> None:
    """Large arguments must not raise overflow or produce NaN/inf."""
    for s in (-1e9, -1e300, 1e300, 1e9):
        value = sigmoid_switching(s, 1e6)
        assert math.isfinite(value)
        assert -1.0 <= value <= 1.0


def test_sigmoid_is_monotone_and_odd() -> None:
    a = 4.0
    values = [sigmoid_switching(s, a) for s in (-1.0, -0.5, 0.0, 0.5, 1.0)]
    assert values == sorted(values)
    assert sigmoid_switching(0.0, a) == 0.0
    for s in (-3.0, -0.7, 0.2, 5.0):
        assert sigmoid_switching(-s, a) == pytest.approx(-sigmoid_switching(s, a))


def test_sigmoid_zero_matches_sign_zero() -> None:
    """At S=0 both switching functions vanish, so both observers coincide there."""
    assert sigmoid_switching(0.0, 4.0) == 0.0
    assert sign_and_sigmoid_agree_at_zero()


def sign_and_sigmoid_agree_at_zero() -> bool:
    from pmsm_sensorless.observers.conventional_smo import sign_switching

    return sign_switching(0.0) == sigmoid_switching(0.0, 4.0) == 0.0


# --- manual single-step example ----------------------------------------------


def test_single_step_matches_hand_calculation() -> None:
    """Rs=0.5, Ld=0.01, k=10, a=2, i_hat=0, i=1, u=0, dt=1e-3.

    S = -1 -> g = tanh(-1) = -0.7615941559557649
    d(i_hat)/dt = 0 - (10/0.01)*g = 761.5941559557649
    i_hat_new = 0 + 1e-3 * 761.594... = 0.7615941559557649
    """
    observer = SigmoidSMO(
        SMOObserverParameters(
            stator_resistance=0.5, ld=0.01, switching_gain_k=10.0, sigmoid_slope_a=2.0
        ),
        SMOObserverState(0.0, 0.0),
    )
    out = observer.step(0.0, 0.0, 1.0, 0.0, 1e-3)
    assert out.i_alpha_hat == pytest.approx(0.7615941559557649)
    # Reported sliding surface / back-EMF are the sample-instant values (D-025):
    assert out.s_alpha == pytest.approx(-1.0)
    assert out.back_emf_alpha == pytest.approx(10.0 * math.tanh(-1.0))


def test_missing_slope_is_rejected() -> None:
    with pytest.raises(ValueError):
        SigmoidSMO(SMOObserverParameters(0.5, 0.01, 10.0))  # a = None


def test_invalid_slope_is_rejected() -> None:
    with pytest.raises(ValueError):
        SMOObserverParameters(0.5, 0.01, 10.0, sigmoid_slope_a=-1.0)


# --- fairness with the conventional observer ---------------------------------


def test_both_variants_coincide_when_sliding_surface_is_zero() -> None:
    """With i_hat == i the switching term is zero for both, so updates agree."""
    params_conv = SMOObserverParameters(0.5, 0.01, 10.0)
    params_sig = SMOObserverParameters(0.5, 0.01, 10.0, sigmoid_slope_a=4.0)
    conv = ConventionalSMO(params_conv, SMOObserverState(1.0, 2.0))
    sig = SigmoidSMO(params_sig, SMOObserverState(1.0, 2.0))
    out_conv = conv.step(3.0, 4.0, 1.0, 2.0, 1e-4)  # i == i_hat
    out_sig = sig.step(3.0, 4.0, 1.0, 2.0, 1e-4)
    assert out_conv.i_alpha_hat == pytest.approx(out_sig.i_alpha_hat)
    assert out_conv.i_beta_hat == pytest.approx(out_sig.i_beta_hat)


def test_update_is_deterministic() -> None:
    def run() -> list[float]:
        observer = SigmoidSMO(SMOObserverParameters(0.5, 0.01, 10.0, sigmoid_slope_a=2.0))
        return [observer.step(1.0, -2.0, 0.5, 0.5, 1e-4).i_alpha_hat for _ in range(200)]

    assert run() == run()


def test_back_emf_estimate_is_bounded_by_gain() -> None:
    observer = SigmoidSMO(
        SMOObserverParameters(0.5, 0.01, 10.0, sigmoid_slope_a=2.0),
        SMOObserverState(1e3, 0.0),
    )
    out = observer.step(0.0, 0.0, 0.0, 0.0, 1e-4)
    assert abs(out.back_emf_alpha) <= 10.0
