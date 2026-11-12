"""Second-order generalized integrator (SOGI-QSG), paper Eqs.(9)-(10), (17)-(19).

The SOGI-QSG is a quadrature-signal generator driven by one input ``v`` at
resonance frequency ``omega_prime``:

.. math::

    D(s) = \\frac{v_o}{v_{in}}  = \\frac{k_s \\omega' s}{s^2 + k_s \\omega' s + \\omega'^2}
    \\qquad (9)
.. math::

    Q(s) = \\frac{qv_o}{v_{in}} = \\frac{k_s \\omega'^2}{s^2 + k_s \\omega' s + \\omega'^2}
    \\qquad (10)

``D`` is band-pass (zero gain at DC, unity gain at ``omega_prime``) and ``Q`` is
low-pass; ``qv_o`` lags ``v_o`` by 90 deg.

Discretization
--------------
The paper discretizes with **Tustin with prewarping**, Eq.(17), p.4: each
integrator ``1/s`` becomes ``tan(omega' Ts/2)/omega' * (1 + z^-1)/(1 - z^-1)``.
Applying that to Eqs.(9)-(10) gives (derivation in
``docs/implementation_decisions.md``, D-020):

.. math::

    D(z) = \\frac{k_s \\sin(\\omega' T_s)\\,(1 - z^{-2})}
                {(2 + k_s \\sin \\omega' T_s) - 4\\cos(\\omega' T_s)\\,z^{-1}
                 + (2 - k_s \\sin \\omega' T_s)\\,z^{-2}}
.. math::

    Q(z) = \\frac{k_s (1 - \\cos \\omega' T_s)\\,(1 + z^{-1})^2}
                {(2 + k_s \\sin \\omega' T_s) - 4\\cos(\\omega' T_s)\\,z^{-1}
                 + (2 - k_s \\sin \\omega' T_s)\\,z^{-2}}

This preserves the exact resonance property for any ``omega' Ts`` (prewarping):
at ``z = exp(j omega' Ts)`` the discrete responses equal the continuous ones at
``s = j omega'``, so ``|D| = 1``, ``angle(D) = 0``, ``|Q| = 1``,
``angle(Q) = -90 deg``, and ``Q(1) = k_s``.

Each relation is realized as a Direct-Form-II-transposed biquad with coefficients
recomputed every step from the online ``omega_prime``.

Note: the coefficients printed in the paper's Eqs.(18)-(19) (as extracted from
``docs/act.pdf``) carry an extra ``k_s`` in the ``z^-1`` denominator term and a
factor 2 in the ``D`` numerator; those make ``|D| = 0.01`` and ``Q(1) != k_s`` at
resonance, contradicting Eq.(17). The prewarped-bilinear coefficients above are
therefore used; the discrepancy is flagged in ``docs/implementation_decisions.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "SOGICoefficients",
    "SOGIQSG",
    "SOGIQSGOutput",
    "SOGIQSGState",
    "in_phase_transfer",
    "quadrature_transfer",
    "sogi_coefficients",
]


@dataclass(frozen=True)
class SOGICoefficients:
    """Discrete biquad coefficients (already normalized by ``a0``).

    The in-phase (``D``) and quadrature (``Q``) numerators share the denominator
    ``1 + a1 z^-1 + a2 z^-2``.
    """

    b0_in_phase: float
    b1_in_phase: float
    b2_in_phase: float
    b0_quadrature: float
    b1_quadrature: float
    b2_quadrature: float
    a1: float
    a2: float


def sogi_coefficients(damping_gain_k: float, omega_prime: float, dt: float) -> SOGICoefficients:
    """Return the prewarped-Tustin biquad coefficients for ``D(z)`` and ``Q(z)``.

    Args:
        damping_gain_k: SOGI damping gain ``k_s`` (dimensionless, > 0).
        omega_prime: Resonance frequency ``omega'`` [rad/s] (>= 0).
        dt: Sample period ``Ts`` [s] (> 0).

    Raises:
        ValueError: If any argument is out of range.
    """
    if damping_gain_k <= 0.0:
        raise ValueError(f"damping_gain_k must be positive, got {damping_gain_k}")
    if omega_prime < 0.0:
        raise ValueError(f"omega_prime must be non-negative, got {omega_prime}")
    if dt <= 0.0:
        raise ValueError(f"timestep dt must be positive, got {dt}")

    x = omega_prime * dt
    sin_x, cos_x = math.sin(x), math.cos(x)
    a0 = 2.0 + damping_gain_k * sin_x
    a1 = -4.0 * cos_x / a0
    a2 = (2.0 - damping_gain_k * sin_x) / a0
    d_gain = damping_gain_k * sin_x / a0
    q_gain = damping_gain_k * (1.0 - cos_x) / a0
    return SOGICoefficients(
        b0_in_phase=d_gain,
        b1_in_phase=0.0,
        b2_in_phase=-d_gain,
        b0_quadrature=q_gain,
        b1_quadrature=2.0 * q_gain,
        b2_quadrature=q_gain,
        a1=a1,
        a2=a2,
    )


def in_phase_transfer(omega: float, damping_gain_k: float, omega_prime: float) -> complex:
    """Closed-form band-pass ``D(j omega)`` from Eq.(9)."""
    s = 1j * omega
    return (
        damping_gain_k
        * omega_prime
        * s
        / (s**2 + damping_gain_k * omega_prime * s + omega_prime**2)
    )


def quadrature_transfer(omega: float, damping_gain_k: float, omega_prime: float) -> complex:
    """Closed-form low-pass ``Q(j omega)`` from Eq.(10)."""
    s = 1j * omega
    return (
        damping_gain_k * omega_prime**2 / (s**2 + damping_gain_k * omega_prime * s + omega_prime**2)
    )


@dataclass
class SOGIQSGState:
    """Internal biquad state (Direct Form II transposed), two per output."""

    in_phase_z1: float = 0.0
    in_phase_z2: float = 0.0
    quadrature_z1: float = 0.0
    quadrature_z2: float = 0.0


@dataclass(frozen=True)
class SOGIQSGOutput:
    """SOGI outputs at one sample: ``v_o`` (in phase) and ``qv_o`` (quadrature)."""

    in_phase: float
    quadrature: float


class SOGIQSG:
    """Discrete SOGI-QSG for one axis (Eqs.(9)-(10) with Eq.(17) prewarping).

    Args:
        damping_gain_k: SOGI damping gain ``k_s`` (> 0).
        initial_state: Optional biquad state.
    """

    def __init__(self, damping_gain_k: float, initial_state: SOGIQSGState | None = None) -> None:
        if damping_gain_k <= 0.0:
            raise ValueError(f"damping_gain_k must be positive, got {damping_gain_k}")
        self.damping_gain_k = damping_gain_k
        self._state = initial_state if initial_state is not None else SOGIQSGState()

    @property
    def state(self) -> SOGIQSGState:
        """The persistent biquad state."""
        return self._state

    def reset(self, state: SOGIQSGState | None = None) -> None:
        """Reset the biquad state (default zero)."""
        self._state = state if state is not None else SOGIQSGState()

    def step(self, v: float, omega_prime: float, dt: float) -> SOGIQSGOutput:
        """Advance one sample driven by ``v`` at resonance ``omega_prime``.

        Raises:
            ValueError: If ``dt`` is not positive or ``omega_prime`` is negative.
        """
        coefficients = sogi_coefficients(self.damping_gain_k, omega_prime, dt)
        state = self._state

        in_phase = coefficients.b0_in_phase * v + state.in_phase_z1
        state.in_phase_z1 = (
            coefficients.b1_in_phase * v + state.in_phase_z2 - coefficients.a1 * in_phase
        )
        state.in_phase_z2 = coefficients.b2_in_phase * v - coefficients.a2 * in_phase

        quadrature = coefficients.b0_quadrature * v + state.quadrature_z1
        state.quadrature_z1 = (
            coefficients.b1_quadrature * v + state.quadrature_z2 - coefficients.a1 * quadrature
        )
        state.quadrature_z2 = coefficients.b2_quadrature * v - coefficients.a2 * quadrature

        return SOGIQSGOutput(in_phase=in_phase, quadrature=quadrature)
