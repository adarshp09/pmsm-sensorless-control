"""PMSM plant: the stationary :math:`\\alpha\\beta` electrical model of the paper.

The paper ignores cross-coupling magnetic saturation, iron loss and magnet
eddy-current loss, and reaches the two-axis stationary model of Eq.(1), p.2
(printed with a single :math:`L_d`, and the reported machine has :math:`L_d = L_q`):

.. math::

    L_d \\frac{d}{dt} \\begin{bmatrix} i_\\alpha \\\\ i_\\beta \\end{bmatrix}
    = -R_s \\begin{bmatrix} i_\\alpha \\\\ i_\\beta \\end{bmatrix}
    + \\begin{bmatrix} u_\\alpha \\\\ u_\\beta \\end{bmatrix}
    - \\begin{bmatrix} E_\\alpha \\\\ E_\\beta \\end{bmatrix}

with the back-EMF of Eq.(2), p.2:

.. math::

    \\begin{bmatrix} E_\\alpha \\\\ E_\\beta \\end{bmatrix}
    = \\omega_e \\psi_f \\begin{bmatrix} -\\sin\\theta_e \\\\ \\cos\\theta_e \\end{bmatrix}

Units are explicit throughout: stator resistance :math:`R_s` in ohm,
inductances in henry, flux linkage :math:`\\psi_f` in weber, currents in ampere,
voltages in volt, electrical angle :math:`\\theta_e` in radian and electrical
speed :math:`\\omega_e` in rad/s. Quantities carrying an ``_e`` suffix are
electrical; an ``_m`` suffix is mechanical.

The paper's printed model treats :math:`\\theta_e` and :math:`\\omega_e` as inputs,
so the default plant integrates the **electrical** states with externally
prescribed kinematics. An **opt-in** mechanical extension (``integrate_mechanics``)
is also provided. It is *not* printed in the paper and must not be read as a
reproduction of it:

.. math::

    J \\frac{d\\omega_m}{dt} = T_e - T_L - B\\,\\omega_m, \\qquad
    \\frac{d\\theta_m}{dt} = \\omega_m, \\qquad
    \\omega_e = p\\,\\omega_m

.. math::

    T_e = \\tfrac{3}{2} p \\,\\psi_f\\, i_q, \\qquad
    i_q = -i_\\alpha \\sin\\theta_e + i_\\beta \\cos\\theta_e \\quad (L_d = L_q)

The viscous-friction coefficient :math:`B` is unresolved in the paper and is never
defaulted here silently: the mechanical mode requires an explicit value. See
``docs/implementation_decisions.md`` (D-008, D-009, D-011).

Numerical integration uses a classical fourth-order Runge-Kutta step
(:func:`rk4_step`); the paper does not specify the plant solver, so this is a
documented implementation choice.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from pmsm_sensorless.config import ConfigError, get_parameter, resolve_parameter

__all__ = [
    "PMSMParameters",
    "PMSMState",
    "PMSMPlant",
    "rk4_step",
    "wrap_angle",
]

_TWO_PI = 2.0 * math.pi


def wrap_angle(angle: Any) -> Any:
    """Wrap angle(s) into ``[-pi, pi)`` (see ``docs/equations_and_conventions.md``).

    Uses the documented formula ``((x + pi) mod 2*pi) - pi``. Accepts scalars or
    arrays and returns the same shape.
    """
    return (angle + math.pi) % _TWO_PI - math.pi


def rk4_step(rhs: Callable[[np.ndarray], np.ndarray], y: Any, dt: float) -> np.ndarray:
    """Advance ``dy/dt = rhs(y)`` by one classical fourth-order Runge-Kutta step.

    Args:
        rhs: Right-hand side mapping a state vector to its derivative.
        y: Current state vector (array-like).
        dt: Timestep in seconds (must be positive).

    Returns:
        The state vector at ``t + dt`` as a new array.
    """
    if dt <= 0.0:
        raise ValueError(f"timestep dt must be positive, got {dt}")
    y = np.asarray(y, dtype=float)
    k1 = rhs(y)
    k2 = rhs(y + 0.5 * dt * k1)
    k3 = rhs(y + 0.5 * dt * k2)
    k4 = rhs(y + dt * k3)
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


@dataclass(frozen=True)
class PMSMParameters:
    """Validated electrical (and optional mechanical) machine parameters.

    Attributes:
        pole_pairs: Pole pairs :math:`p` (dimensionless, >= 1).
        stator_resistance: Stator resistance :math:`R_s` in ohm.
        ld: d-axis inductance :math:`L_d` in henry.
        lq: q-axis inductance :math:`L_q` in henry (must equal ``ld`` for the
            paper's non-salient model of Eq.(1)).
        psi_f: Permanent-magnet flux linkage :math:`\\psi_f` in weber.
        inertia: Rotor inertia :math:`J` in kg.m^2 (mechanical mode only).
        viscous_friction: Viscous friction :math:`B` in N.m.s/rad (mechanical
            mode only). ``None`` means the paper value is unresolved.
    """

    pole_pairs: int
    stator_resistance: float
    ld: float
    lq: float
    psi_f: float
    inertia: float | None = None
    viscous_friction: float | None = None

    def __post_init__(self) -> None:
        if self.pole_pairs < 1:
            raise ValueError(f"pole_pairs must be >= 1, got {self.pole_pairs}")
        if self.stator_resistance <= 0.0:
            raise ValueError(f"stator_resistance must be positive, got {self.stator_resistance}")
        if self.ld <= 0.0 or self.lq <= 0.0:
            raise ValueError(f"inductances must be positive, got ld={self.ld}, lq={self.lq}")
        if not math.isclose(self.lq, self.ld, rel_tol=1e-9):
            raise ValueError(
                f"Eq.(1) is the non-salient model with Ld = Lq; got ld={self.ld}, lq={self.lq}"
            )
        if self.psi_f <= 0.0:
            raise ValueError(f"psi_f must be positive, got {self.psi_f}")
        if self.inertia is not None and self.inertia <= 0.0:
            raise ValueError(f"inertia must be positive, got {self.inertia}")
        if self.viscous_friction is not None and self.viscous_friction < 0.0:
            raise ValueError(f"viscous_friction must be non-negative, got {self.viscous_friction}")

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "PMSMParameters":
        """Build parameters from a validated ``pmsm`` configuration mapping.

        Required values (``pole_pairs``, ``Rs``, ``Ld``, ``Lq``, ``psi_f``) must
        be resolved; unresolved mechanical values are carried through as ``None``.

        Raises:
            ConfigError: If a required value is absent, unresolved, unit-mismatched,
                or ``pole_pairs`` is not a whole number.
        """
        pole_pairs_value = resolve_parameter(config, "machine.pole_pairs", unit="-")
        if float(pole_pairs_value) != int(pole_pairs_value):
            raise ConfigError(f"machine.pole_pairs must be a whole number, got {pole_pairs_value}")
        return cls(
            pole_pairs=int(pole_pairs_value),
            stator_resistance=resolve_parameter(config, "machine.stator_resistance_rs", unit="ohm"),
            ld=resolve_parameter(config, "machine.stator_inductance_ld", unit="H"),
            lq=resolve_parameter(config, "machine.stator_inductance_lq", unit="H"),
            psi_f=resolve_parameter(config, "machine.pm_flux_linkage_psi_f", unit="Wb"),
            inertia=get_parameter(config, "mech.inertia_j", unit="kg.m^2"),
            viscous_friction=get_parameter(config, "mech.viscous_friction_b", unit="N.m.s/rad"),
        )


@dataclass
class PMSMState:
    """Plant state in the stationary frame.

    Attributes:
        i_alpha, i_beta: Stator currents in ampere.
        omega_e: Electrical speed in rad/s.
        theta_e: Electrical angle in radian.
    """

    i_alpha: float
    i_beta: float
    omega_e: float
    theta_e: float


class PMSMPlant:
    """Deterministic PMSM plant (Eq.(1)-(2) plus optional mechanical dynamics).

    Args:
        parameters: Validated machine parameters.
        integrate_mechanics: If ``True``, also integrate rotor speed/angle from the
            torque balance (a derived extension, not printed in the paper).
        viscous_friction: Explicit friction override in N.m.s/rad. If omitted, the
            value from ``parameters`` is used. Mechanical mode requires a
            non-``None`` friction; this prevents a missing paper value from being
            silently replaced by zero.

    Raises:
        ValueError: If mechanical mode is requested without inertia or friction.
    """

    def __init__(
        self,
        parameters: PMSMParameters,
        *,
        integrate_mechanics: bool = False,
        viscous_friction: float | None = None,
    ) -> None:
        self.parameters = parameters
        self.integrate_mechanics = integrate_mechanics
        self._friction = (
            viscous_friction if viscous_friction is not None else parameters.viscous_friction
        )
        if self._friction is not None and self._friction < 0.0:
            raise ValueError(f"viscous_friction must be non-negative, got {self._friction}")
        if integrate_mechanics:
            if parameters.inertia is None:
                raise ValueError(
                    "mechanical mode requires a resolved inertia "
                    "(mech.inertia_j); none was supplied"
                )
            if self._friction is None:
                raise ValueError(
                    "mechanical mode requires an explicit viscous friction "
                    "(mech.viscous_friction_b is unresolved); pass "
                    "viscous_friction=... to opt in deliberately"
                )

    @classmethod
    def from_config(cls, config: dict[str, Any], **kwargs: Any) -> "PMSMPlant":
        """Build a plant from a validated ``pmsm`` configuration mapping."""
        return cls(PMSMParameters.from_config(config), **kwargs)

    def electrical_from_mechanical(self, omega_m: float) -> float:
        """Convert mechanical speed ``omega_m`` [rad/s] to electrical speed [rad/s]."""
        return self.parameters.pole_pairs * omega_m

    def mechanical_from_electrical(self, omega_e: float) -> float:
        """Convert electrical speed ``omega_e`` [rad/s] to mechanical speed [rad/s]."""
        return omega_e / self.parameters.pole_pairs

    def back_emf(self, theta_e: float, omega_e: float) -> np.ndarray:
        """Return ``[E_alpha, E_beta]`` in volt from Eq.(2), p.2."""
        amplitude = omega_e * self.parameters.psi_f
        return np.array([-amplitude * math.sin(theta_e), amplitude * math.cos(theta_e)])

    def current_derivative(self, i: Any, u: Any, theta_e: float, omega_e: float) -> np.ndarray:
        """Return ``d[i_alpha, i_beta]/dt`` in A/s from Eq.(1), p.2."""
        emf = self.back_emf(theta_e, omega_e)
        return (
            -self.parameters.stator_resistance * np.asarray(i, dtype=float)
            + np.asarray(u, dtype=float)
            - emf
        ) / self.parameters.ld

    def steady_state_current(
        self, u_alpha: float, u_beta: float, theta_e: float, omega_e: float
    ) -> np.ndarray:
        """Return the equilibrium current ``(u - E)/Rs`` in ampere (for constant inputs)."""
        emf = self.back_emf(theta_e, omega_e)
        return (np.array([u_alpha, u_beta], dtype=float) - emf) / self.parameters.stator_resistance

    def electromagnetic_torque(self, i: Any, theta_e: float) -> float:
        """Return the electromagnetic torque ``T_e`` in N.m (derived; Ld = Lq)."""
        i_alpha, i_beta = float(i[0]), float(i[1])
        i_q = -i_alpha * math.sin(theta_e) + i_beta * math.cos(theta_e)
        return 1.5 * self.parameters.pole_pairs * self.parameters.psi_f * i_q

    def speed_derivative(self, omega_e: float, torque: float, load_torque: float) -> float:
        """Return ``d(omega_e)/dt`` in rad/s^2 from the (derived) torque balance."""
        inertia = self.parameters.inertia
        if inertia is None or self._friction is None:
            raise ValueError("speed_derivative requires resolved inertia and an explicit friction")
        pole_pairs = self.parameters.pole_pairs
        return (pole_pairs * (torque - load_torque) - self._friction * omega_e) / inertia

    def step(
        self,
        state: PMSMState,
        u_alpha: float,
        u_beta: float,
        dt: float,
        *,
        load_torque: float = 0.0,
    ) -> PMSMState:
        """Advance the deterministic state by ``dt`` seconds.

        In prescribed-kinematics mode (default) ``state.theta_e`` is advanced at the
        constant ``state.omega_e`` supplied by the caller; the caller owns the speed
        profile. In mechanical mode the electrical speed and angle are integrated
        from the torque balance, with ``load_torque`` [N.m] opposing positive
        rotation.

        Args:
            state: Current plant state.
            u_alpha, u_beta: Stator voltages in volt.
            dt: Timestep in seconds.
            load_torque: Load torque in N.m (mechanical mode only).

        Returns:
            The new :class:`PMSMState`.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        u = np.array([u_alpha, u_beta], dtype=float)

        if self.integrate_mechanics:

            def rhs(y: np.ndarray) -> np.ndarray:
                i = y[:2]
                omega_e, theta_e = float(y[2]), float(y[3])
                di = self.current_derivative(i, u, theta_e, omega_e)
                torque = self.electromagnetic_torque(i, theta_e)
                domega = self.speed_derivative(omega_e, torque, load_torque)
                return np.array([di[0], di[1], domega, omega_e])

            y0 = np.array([state.i_alpha, state.i_beta, state.omega_e, state.theta_e], dtype=float)
            y1 = rk4_step(rhs, y0, dt)
            return PMSMState(
                i_alpha=float(y1[0]),
                i_beta=float(y1[1]),
                omega_e=float(y1[2]),
                theta_e=float(wrap_angle(y1[3])),
            )

        def rhs_prescribed(y: np.ndarray) -> np.ndarray:
            # The angle y[2] evolves within the step, so the back-EMF must be
            # evaluated at the substage angle for a consistent high-order update.
            di = self.current_derivative(y[:2], u, float(y[2]), state.omega_e)
            return np.array([di[0], di[1], state.omega_e])

        y0 = np.array([state.i_alpha, state.i_beta, state.theta_e], dtype=float)
        y1 = rk4_step(rhs_prescribed, y0, dt)
        return PMSMState(
            i_alpha=float(y1[0]),
            i_beta=float(y1[1]),
            omega_e=state.omega_e,
            theta_e=float(wrap_angle(y1[2])),
        )
