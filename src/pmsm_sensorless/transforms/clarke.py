"""Clarke transform between the three-phase and stationary two-axis frames.

The source paper works in the stationary :math:`\\alpha\\beta` frame, Eqs.(1)-(2),
p.2, but never prints a Clarke matrix. The scaling is therefore **not** fixed by
the paper; following the Phase 01 rules this module exposes it as an explicit,
configurable convention and never claims paper fidelity for it.

Two conventions are provided, selected by ``scaling``:

**Amplitude-invariant** (default, ``"amplitude_invariant"``) preserves peak values,
so a balanced set :math:`x_a = A\\cos\\theta` maps to :math:`x_\\alpha = A\\cos\\theta`,
:math:`x_\\beta = A\\sin\\theta`. For a zero-sequence-free set the squared magnitude is
:math:`x_\\alpha^2 + x_\\beta^2 = \\tfrac{2}{3}(x_a^2 + x_b^2 + x_c^2)`:

.. math::

    \\begin{bmatrix} x_\\alpha \\\\ x_\\beta \\end{bmatrix} =
    \\begin{bmatrix} 1 & -1/2 & -1/2 \\\\ 0 & \\sqrt{3}/2 & -\\sqrt{3}/2 \\end{bmatrix}
    \\begin{bmatrix} x_a \\\\ x_b \\\\ x_c \\end{bmatrix}

**Power-invariant** (``"power_invariant"``) preserves the Euclidean norm of a
zero-sequence-free set, :math:`x_\\alpha^2 + x_\\beta^2 = x_a^2 + x_b^2 + x_c^2`. It is
:math:`\\sqrt{3/2}` times the amplitude-invariant result (equivalently,
:math:`\\sqrt{2/3}` applied to the unscaled factors above).

Both conventions deliberately drop the zero-sequence, so the inverse reconstructs
a three-phase set only up to its zero-sequence part. The :math:`\\alpha` axis is
aligned with phase ``a`` and :math:`\\beta` leads :math:`\\alpha` by :math:`+90^\\circ`,
which is consistent with the paper's back-EMF orientation in Eq.(2),
:math:`[E_\\alpha, E_\\beta] = \\omega_e \\psi_f [-\\sin\\theta_e, \\cos\\theta_e]`.

Status: PROVISIONAL. See ``docs/equations_and_conventions.md`` and
``docs/implementation_decisions.md`` (D-010).
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "SCALING_CONVENTIONS",
    "clarke_abc_to_alphabeta",
    "clarke_alphabeta_to_abc",
    "clarke_matrix",
    "inverse_clarke_matrix",
]

#: Unscaled Clarke factors (the amplitude-invariant matrix divided by 2/3).
_BASE = np.array(
    [
        [1.0, -0.5, -0.5],
        [0.0, float(np.sqrt(3.0) / 2.0), float(-np.sqrt(3.0) / 2.0)],
    ]
)

_FORWARD_SCALE = {
    "amplitude_invariant": 2.0 / 3.0,
    "power_invariant": float(np.sqrt(2.0 / 3.0)),
}
_INVERSE_SCALE = {
    "amplitude_invariant": 1.0,
    "power_invariant": float(np.sqrt(2.0 / 3.0)),
}

#: Supported Clarke scaling conventions.
SCALING_CONVENTIONS = tuple(_FORWARD_SCALE)


def _check_scaling(scaling: str) -> str:
    if scaling not in _FORWARD_SCALE:
        raise ValueError(
            f"unknown Clarke scaling {scaling!r}; expected one of {SCALING_CONVENTIONS}"
        )
    return scaling


def clarke_matrix(scaling: str = "amplitude_invariant") -> np.ndarray:
    """Return the ``2x3`` forward Clarke matrix for ``scaling``.

    Raises:
        ValueError: If ``scaling`` is not one of :data:`SCALING_CONVENTIONS`.
    """
    return _FORWARD_SCALE[_check_scaling(scaling)] * _BASE


def inverse_clarke_matrix(scaling: str = "amplitude_invariant") -> np.ndarray:
    """Return the ``3x2`` inverse Clarke matrix for ``scaling``.

    This is the Moore-Penrose inverse of :func:`clarke_matrix`; it reconstructs
    the zero-sequence-free part of a three-phase set.

    Raises:
        ValueError: If ``scaling`` is not one of :data:`SCALING_CONVENTIONS`.
    """
    return _INVERSE_SCALE[_check_scaling(scaling)] * _BASE.T


def clarke_abc_to_alphabeta(
    x_abc: object, *, scaling: str = "amplitude_invariant"
) -> np.ndarray:
    """Transform three-phase quantities to ``(alpha, beta)``.

    Args:
        x_abc: Array-like whose last axis has length 3.
        scaling: One of :data:`SCALING_CONVENTIONS`.

    Returns:
        Array with the last axis of length 2.

    Raises:
        ValueError: If the last axis is not length 3, or ``scaling`` is unknown.
    """
    array = np.asarray(x_abc, dtype=float)
    if array.ndim == 0 or array.shape[-1] != 3:
        raise ValueError(
            f"x_abc must have last axis of length 3, got shape {array.shape}"
        )
    return array @ clarke_matrix(scaling).T


def clarke_alphabeta_to_abc(
    x_alphabeta: object, *, scaling: str = "amplitude_invariant"
) -> np.ndarray:
    """Transform ``(alpha, beta)`` back to three-phase quantities.

    Args:
        x_alphabeta: Array-like whose last axis has length 2.
        scaling: One of :data:`SCALING_CONVENTIONS`.

    Returns:
        Array with the last axis of length 3.

    Raises:
        ValueError: If the last axis is not length 2, or ``scaling`` is unknown.
    """
    array = np.asarray(x_alphabeta, dtype=float)
    if array.ndim == 0 or array.shape[-1] != 2:
        raise ValueError(
            f"x_alphabeta must have last axis of length 2, got shape {array.shape}"
        )
    return array @ inverse_clarke_matrix(scaling).T
