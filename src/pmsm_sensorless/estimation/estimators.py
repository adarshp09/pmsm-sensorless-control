"""Rotor position and speed estimators (paper Section II-B, p.825).

Two estimators of the rotor electrical position from the stationary back-EMF:

- :class:`ArctanEstimator` -- the paper's Eq.(8) direct form;
- :class:`QPLL` -- the normalized quadrature PLL of Fig.1 (block structure only).

Both consume **only** the ``alpha-beta`` back-EMF and the timestep. This package
imports no plant, observers, filters or fuzzy modules; the chain
observer -> filter -> estimator is composed only in ``experiments/``.

The paper prints the estimator **structure** (Eq.(8), Fig.1) but **no numeric gains,
error algebra, normalization or angle convention**. Every such value is an explicit
ASSUMPTION in ``configs/estimator_assumptions.yaml``; ``null`` raises
:class:`~pmsm_sensorless.config.ConfigError`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from pmsm_sensorless.config import ConfigError, resolve_parameter

__all__ = [
    "ArctanEstimator",
    "EstimatorOutput",
    "QPLL",
    "QPLLParameters",
    "electrical_to_mechanical_speed",
    "wrap_angle_pi_closed",
]

_TWO_PI = 2.0 * math.pi


def wrap_angle_pi_closed(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]`` (the estimator's documented convention).

    Uses ``((x + pi) mod 2pi) - pi`` (which returns ``[-pi, pi)``) and maps ``-pi``
    to ``+pi`` so the half-open boundary is the closed one.
    """
    wrapped = (angle + math.pi) % _TWO_PI - math.pi
    if wrapped == -math.pi:
        return math.pi
    return wrapped


def electrical_to_mechanical_speed(omega_e: float, pole_pairs: int) -> float:
    """DERIVED conversion ``omega_m = omega_e / p`` (Eq. not printed; ``p`` printed).

    Raises:
        ValueError: If ``pole_pairs`` is not >= 1.
    """
    if pole_pairs < 1:
        raise ValueError(f"pole_pairs must be >= 1, got {pole_pairs}")
    return omega_e / pole_pairs


@dataclass(frozen=True)
class EstimatorOutput:
    """One estimator update.

    Attributes:
        theta_hat: estimated electrical position [rad], wrapped to ``(-pi, pi]``.
        omega_hat: estimated electrical speed [rad/s], or ``None`` if the estimator
            does not estimate speed (the instantaneous arctan estimator).
    """

    theta_hat: float
    omega_hat: float | None


class ArctanEstimator:
    """Direct position estimate, Eq.(8), p.825.

    The paper prints ``theta_hat_e = -arctan(E_alpha_hat / E_beta_hat)`` (a
    one-argument ``arctan`` with the minus outside). Implemented in four-quadrant
    form as ``atan2(-E_alpha, E_beta)``, which for the paper's convention
    ``E = omega_e psi_f[-sin theta_e, cos theta_e]`` (Eq.(2)) returns ``theta_e``
    exactly; the result is wrapped to ``(-pi, pi]``.

    Args:
        initial_theta: value held before the first update [rad].
    """

    def __init__(self, initial_theta: float = 0.0) -> None:
        self._theta = wrap_angle_pi_closed(initial_theta)

    @property
    def theta_hat(self) -> float:
        """Current estimated electrical position [rad]."""
        return self._theta

    def reset(self, theta: float = 0.0) -> None:
        """Reset the estimate."""
        self._theta = wrap_angle_pi_closed(theta)

    def step(self, e_alpha: float, e_beta: float, dt: float) -> EstimatorOutput:
        """Return the Eq.(8) position estimate for one sample.

        ``dt`` is accepted for a uniform estimator interface and is not used (the
        arctan estimate is instantaneous).

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0.0:
            raise ValueError(f"timestep dt must be positive, got {dt}")
        self._theta = wrap_angle_pi_closed(math.atan2(-e_alpha, e_beta))
        return EstimatorOutput(theta_hat=self._theta, omega_hat=None)


@dataclass(frozen=True)
class QPLLParameters:
    """Validated Q-PLL gains.

    Attributes:
        kp: proportional gain [rad/(s.V)].
        ki: integral gain [rad/(s^2.V)].
        normalize: divide the error by ``|E|`` (off by default; the paper's
            "gain normalization" algebra is unprinted). With ``normalize=False``
            the small-signal loop gain is proportional to ``|E|``.

    The paper prints neither gains; they are designed from the assumptions
    ``zeta`` and ``f_n`` (see :meth:`from_configs`).
    """

    kp: float
    ki: float
    normalize: bool = False

    def __post_init__(self) -> None:
        if self.kp < 0.0:
            raise ValueError(f"kp must be non-negative, got {self.kp}")
        if self.ki < 0.0:
            raise ValueError(f"ki must be non-negative, got {self.ki}")

    @classmethod
    def from_design(
        cls, *, zeta: float, f_n_hz: float, amplitude_v: float, normalize: bool = False
    ) -> "QPLLParameters":
        """Design gains from ``zeta``/``f_n`` at a back-EMF amplitude ``A``.

        ``Kp = 2 zeta w_n / A`` and ``Ki = w_n^2 / A`` with ``w_n = 2 pi f_n``.

        Raises:
            ValueError: If any argument is out of range.
        """
        if amplitude_v <= 0.0:
            raise ValueError(f"amplitude_v must be positive, got {amplitude_v}")
        if zeta <= 0.0 or f_n_hz <= 0.0:
            raise ValueError(f"zeta and f_n_hz must be positive, got {zeta}, {f_n_hz}")
        w_n = _TWO_PI * f_n_hz
        return cls(kp=2.0 * zeta * w_n / amplitude_v, ki=w_n * w_n / amplitude_v, normalize=normalize)

    @classmethod
    def from_configs(
        cls,
        assumptions_config: dict[str, Any],
        *,
        design_amplitude_v: float,
    ) -> "QPLLParameters":
        """Design the gains at a back-EMF amplitude ``design_amplitude_v``.

        Reads ``qpll.damping_zeta``, ``qpll.natural_frequency_hz`` and
        ``qpll.normalize_error`` from the assumptions; ``A = |E|`` at 1000 rpm is
        DERIVED.

        Raises:
            ConfigError: If an assumption is absent or null; ValueError if
                ``design_amplitude_v`` is not positive.
        """
        if design_amplitude_v <= 0.0:
            raise ValueError(f"design_amplitude_v must be positive, got {design_amplitude_v}")
        zeta = resolve_parameter(assumptions_config, "qpll.damping_zeta", unit="-")
        f_n = resolve_parameter(assumptions_config, "qpll.natural_frequency_hz", unit="Hz")
        normalize = bool(resolve_parameter(assumptions_config, "qpll.normalize_error", unit="-"))
        if zeta <= 0.0 or f_n <= 0.0:
            raise ConfigError(f"qpll zeta/f_n must be positive, got {zeta}, {f_n}")
        return cls.from_design(
            zeta=zeta, f_n_hz=f_n, amplitude_v=design_amplitude_v, normalize=normalize
        )


class QPLL:
    """Quadrature PLL of Fig.1 (p.825).

    Structure (from the figure): ``E_alpha, E_beta`` are multiplied by ``cos theta_hat``
    and ``sin theta_hat``; the difference is the error ``delta_e``; a PI block produces
    ``omega_hat``; an integrator ``1/s`` produces ``theta_hat``.

    Error form (derived from the figure and the Eq.(8) convention; the paper prints no
    algebra)::

        delta_e = -E_alpha cos(theta_hat) - E_beta sin(theta_hat) = |E| sin(theta - theta_hat)

    so for a rotating ``E`` a positive ``delta_e`` means ``theta > theta_hat`` and drives
    ``omega_hat`` up (restoring sign).

    Discretization (ASSUMPTION): **predict-then-correct**, forward Euler. Each call
    first advances ``theta_hat`` by ``omega_hat * dt`` so the estimate corresponds to
    the current sample, forms the error at that ``theta_hat``, then updates the PI
    integrator and ``omega_hat``. At lock ``theta_hat[k] = theta[k]`` exactly.

    Args:
        parameters: validated gains.
        initial_theta: ``theta_hat(0)`` [rad].
        initial_omega: initial PI integrator value ``omega_hat(0)`` [rad/s].
    """

    def __init__(
        self,
        parameters: QPLLParameters,
        *,
        initial_theta: float = 0.0,
        initial_omega: float = 0.0,
    ) -> None:
        self.parameters = parameters
        self._theta = wrap_angle_pi_closed(initial_theta)
        self._integral = float(initial_omega)
        self._omega = float(initial_omega)
        self.last_error = 0.0

    @property
    def theta_hat(self) -> float:
        """Current estimated electrical position [rad]."""
        return self._theta

    @property
    def omega_hat(self) -> float:
        """Current estimated electrical speed [rad/s]."""
        return self._omega

    def reset(self, theta: float | None = None, omega: float = 0.0) -> None:
        """Reset the estimate (position to ``theta`` or 0) and the PI integrator."""
        self._theta = wrap_angle_pi_closed(0.0 if theta is None else theta)
        self._integral = float(omega)
        self._omega = float(omega)
        self.last_error = 0.0

    def error(self, e_alpha: float, e_beta: float) -> float:
        """Return ``delta_e`` at the current ``theta_hat`` (Fig.1 form)."""
        error = -e_alpha * math.cos(self._theta) - e_beta * math.sin(self._theta)
        if self.parameters.normalize:
            norm = math.hypot(e_alpha, e_beta)
            return error / norm if norm > 0.0 else 0.0
        return error

    def step(self, e_alpha: float, e_beta: float, dt: float) -> EstimatorOutput:
        """Advance the Q-PLL by one forward-Euler step.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0.0:
            raise ValueError(f"timestep dt must be positive, got {dt}")
        # predict the previous estimate to the current sample, then correct
        self._theta = wrap_angle_pi_closed(self._theta + self._omega * dt)
        error = self.error(e_alpha, e_beta)
        self.last_error = error
        self._integral += self.parameters.ki * error * dt
        self._omega = self.parameters.kp * error + self._integral
        return EstimatorOutput(theta_hat=self._theta, omega_hat=self._omega)
