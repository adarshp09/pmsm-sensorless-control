"""Clarke transform tests: scaling, reconstruction and rejection of bad input.

The scaling convention is unresolved in the paper, so these tests pin the two
*documented* conventions (amplitude- and power-invariant) rather than claiming a
paper value. Where a numeric tolerance is used it follows from exact linear
algebra, so it is tight.
"""

from __future__ import annotations

import numpy as np
import pytest

from pmsm_sensorless.transforms import clarke


def _balanced(theta: float, amplitude: float) -> np.ndarray:
    """A balanced (zero-sum) three-phase set ``amplitude * cos(theta - k 2pi/3)``."""
    return amplitude * np.array(
        [
            np.cos(theta),
            np.cos(theta - 2.0 * np.pi / 3.0),
            np.cos(theta + 2.0 * np.pi / 3.0),
        ]
    )


@pytest.mark.parametrize("scaling", clarke.SCALING_CONVENTIONS)
def test_forward_then_inverse_reconstructs_zero_sequence_free_signal(
    scaling: str,
) -> None:
    x_abc = _balanced(0.4, 2.0)
    x_ab = clarke.clarke_abc_to_alphabeta(x_abc, scaling=scaling)
    reconstructed = clarke.clarke_alphabeta_to_abc(x_ab, scaling=scaling)
    assert np.allclose(reconstructed, x_abc, atol=1e-12)


@pytest.mark.parametrize("scaling", clarke.SCALING_CONVENTIONS)
def test_zero_sequence_maps_to_zero(scaling: str) -> None:
    assert np.allclose(clarke.clarke_abc_to_alphabeta([3.0, 3.0, 3.0], scaling=scaling), 0.0)


def test_amplitude_invariant_gives_peak_cos_sin() -> None:
    theta, amplitude = 0.9, 4.0
    x_ab = clarke.clarke_abc_to_alphabeta(_balanced(theta, amplitude))
    assert x_ab == pytest.approx([amplitude * np.cos(theta), amplitude * np.sin(theta)])
    assert float(np.sum(x_ab**2)) == pytest.approx(amplitude**2)


def test_amplitude_invariant_sum_of_squares_relation() -> None:
    x_abc = _balanced(-1.1, 3.0)
    x_ab = clarke.clarke_abc_to_alphabeta(x_abc)
    assert float(np.sum(x_ab**2)) == pytest.approx((2.0 / 3.0) * float(np.sum(x_abc**2)))


def test_power_invariant_preserves_norm() -> None:
    x_abc = _balanced(1.3, 5.0)
    x_ab = clarke.clarke_abc_to_alphabeta(x_abc, scaling="power_invariant")
    assert float(np.sum(x_ab**2)) == pytest.approx(float(np.sum(x_abc**2)))


def test_power_invariant_is_sqrt_three_over_two_times_amplitude() -> None:
    x_abc = _balanced(0.2, 2.5)
    amplitude = clarke.clarke_abc_to_alphabeta(x_abc)
    power = clarke.clarke_abc_to_alphabeta(x_abc, scaling="power_invariant")
    assert power == pytest.approx(np.sqrt(3.0 / 2.0) * amplitude)


@pytest.mark.parametrize("scaling", clarke.SCALING_CONVENTIONS)
def test_inverse_matrix_is_the_moore_penrose_pseudoinverse(scaling: str) -> None:
    forward = clarke.clarke_matrix(scaling)
    assert forward.shape == (2, 3)
    assert clarke.inverse_clarke_matrix(scaling).shape == (3, 2)
    assert clarke.inverse_clarke_matrix(scaling) == pytest.approx(np.linalg.pinv(forward))


def test_amplitude_matrix_values() -> None:
    expected = np.array(
        [
            [2.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0],
            [0.0, 1.0 / np.sqrt(3.0), -1.0 / np.sqrt(3.0)],
        ]
    )
    assert clarke.clarke_matrix() == pytest.approx(expected)


@pytest.mark.parametrize("scaling", clarke.SCALING_CONVENTIONS)
def test_batch_input_is_supported(scaling: str) -> None:
    thetas = np.linspace(0.0, 2.0 * np.pi, 7, endpoint=False)
    batch = np.stack([_balanced(t, 1.0) for t in thetas])
    out = clarke.clarke_abc_to_alphabeta(batch, scaling=scaling)
    assert out.shape == (7, 2)
    assert np.allclose(out, [clarke.clarke_abc_to_alphabeta(row, scaling=scaling) for row in batch])


def test_unknown_scaling_is_rejected() -> None:
    for call in (
        lambda: clarke.clarke_matrix("nope"),
        lambda: clarke.inverse_clarke_matrix("nope"),
        lambda: clarke.clarke_abc_to_alphabeta([1.0, 2.0, 3.0], scaling="nope"),
    ):
        with pytest.raises(ValueError):
            call()


def test_bad_input_shape_is_rejected() -> None:
    with pytest.raises(ValueError):
        clarke.clarke_abc_to_alphabeta(np.zeros(2))
    with pytest.raises(ValueError):
        clarke.clarke_alphabeta_to_abc(np.zeros(3))
    with pytest.raises(ValueError):
        clarke.clarke_abc_to_alphabeta(1.0)
