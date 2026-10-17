"""DSOGI-FLL back-EMF filter: dual SOGI + positive-sequence extraction + FLL.

Replaces the conventional low-pass filter of the SMO with two SOGIs (one per
stationary axis) sharing one frequency-locked loop, per Section III-B of the
paper.

Components
----------
1. **SOGI-QSG per axis** (Eqs.(9)-(10), (17)): see
   :mod:`pmsm_sensorless.filters.sogi`.
2. **Positive-sequence extraction (DSOGI)**: from the two quadrature pairs,

   .. math::

       \\begin{bmatrix} v_\\alpha^{+} \\\\ v_\\beta^{+} \\end{bmatrix}
       = \\tfrac12 \\begin{bmatrix} v_\\alpha' - qv_\\beta' \\\\ qv_\\alpha' + v_\\beta' \\end{bmatrix}

   This rejects the negative sequence / second harmonic. *The paper uses the
   DSOGI structure but does not print these combination equations (item 8,
   ``docs/paper_method.md``); the standard form above is a documented derived
   equation.* Set ``positive_sequence=False`` to use the per-axis band-pass of
   the paper's Eq.(12) instead.
3. **FLL** (Fig.5, p.4): the frequency-error signal is

   .. math::

       \\varepsilon_f = qv_\\alpha' (v_\\alpha - v_\\alpha')
                      + qv_\\beta'  (v_\\beta  - v_\\beta')

   and ``omega'`` is integrated with the paper's negative-gain integrator,
   ``d\\omega'/dt = -\\Gamma \\varepsilon_f``. *The paper shows the FLL block and
   ``\\Gamma`` (p.5) but not the algebraic error signal; the standard form above is
   a documented derived equation.*

Options (off by default, per the "do not add what the paper omits" rule)
-----------------------------------------------------------------------
- ``normalize_error``: divide ``epsilon_f`` by the summed squared SOGI outputs
  (the paper's "gain normalization unit", Fig.5, whose algebra is not printed).
  Off by default.
- ``min_omega`` / ``max_omega``: frequency saturation (the paper gives no
  limits). ``None`` = no clamp.
- ``epsilon_filter_tau``: first-order low-pass time constant [s] on
  ``epsilon_f`` (the paper shows none). ``None`` = no smoothing. Needed when the
  input still carries the SMO's switching ripple, which would otherwise drive the
  un-normalized loop unstable.

Units: signals are used as given; the FLL gain ``Gamma`` has units
``1/(signal^2 s)``. The paper's ``Gamma = 50`` is used verbatim in SI; because
the error signal is quadratic in amplitude, the loop bandwidth scales with the
squared signal amplitude (the paper's normalization removes that dependence).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from pmsm_sensorless.config import ConfigError, get_parameter, resolve_parameter
from pmsm_sensorless.filters.sogi import SOGIQSG, SOGIQSGOutput

__all__ = ["DSOGIFLL", "DSOGIFLLOutput", "DSOGIFLLParameters"]

_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class DSOGIFLLParameters:
    """Validated DSOGI-FLL parameters.

    Attributes:
        damping_gain_k: SOGI damping gain ``k_s`` (> 0).
        fll_gain_gamma: FLL integrator gain ``Gamma`` (> 0).
        initial_omega: Initial resonance frequency ``omega'_0`` [rad/s] (> 0).
            Unresolved in the paper; must be supplied.
        min_omega, max_omega: Optional frequency limits [rad/s]; ``None`` = none.
        normalize_error: Enable the (paper-mentioned, algebra-unresolved) error
            normalization. Off by default.
        positive_sequence: Use DSOGI positive-sequence extraction (default);
            ``False`` uses the per-axis band-pass of Eq.(12).
    """

    damping_gain_k: float
    fll_gain_gamma: float
    initial_omega: float
    min_omega: float | None = None
    max_omega: float | None = None
    normalize_error: bool = False
    epsilon_filter_tau: float | None = None
    positive_sequence: bool = True

    def __post_init__(self) -> None:
        if self.damping_gain_k <= 0.0:
            raise ValueError(f"damping_gain_k must be positive, got {self.damping_gain_k}")
        if self.fll_gain_gamma <= 0.0:
            raise ValueError(f"fll_gain_gamma must be positive, got {self.fll_gain_gamma}")
        if self.initial_omega <= 0.0:
            raise ValueError(f"initial_omega must be positive, got {self.initial_omega}")
        if self.min_omega is not None and self.min_omega <= 0.0:
            raise ValueError(f"min_omega must be positive, got {self.min_omega}")
        if self.max_omega is not None and self.min_omega is not None and self.max_omega < self.min_omega:
            raise ValueError("max_omega must be >= min_omega")
        if self.epsilon_filter_tau is not None and self.epsilon_filter_tau <= 0.0:
            raise ValueError(
                f"epsilon_filter_tau must be positive, got {self.epsilon_filter_tau}"
            )

    @classmethod
    def from_config(
        cls,
        observer_config: dict[str, Any],
        *,
        initial_omega: float | None = None,
        normalize_error: bool = False,
        positive_sequence: bool = True,
    ) -> "DSOGIFLLParameters":
        """Build parameters from the validated ``observer.yaml`` mapping.

        ``k_s`` and ``Gamma`` are read verbatim (resolved, p.5). ``omega'_0`` is
        unresolved in the paper: it is taken from ``fll.initial_frequency`` or
        ``dsogi.nominal_center_frequency`` if those are resolved, otherwise
        ``initial_omega`` must be supplied explicitly, else a :class:`ConfigError`
        is raised (no silent default). Frequency limits are applied only if
        resolved.

        Args:
            observer_config: Loaded ``observer.yaml`` mapping.
            initial_omega: Explicit initial ``omega'`` [rad/s] (required if the
                config value is unresolved).
            normalize_error: Enable error normalization (off by default).
            positive_sequence: Use DSOGI positive-sequence extraction.

        Raises:
            ConfigError: If a required value is unresolved or missing.
        """
        damping_gain_k = resolve_parameter(observer_config, "dsogi.damping_gain_k", unit="-")
        fll_gain_gamma = resolve_parameter(observer_config, "fll.gamma_gain", unit="-")

        if initial_omega is None:
            initial_hz = get_parameter(observer_config, "fll.initial_frequency", unit="Hz")
            if initial_hz is None:
                initial_hz = get_parameter(
                    observer_config, "dsogi.nominal_center_frequency", unit="Hz"
                )
            if initial_hz is None:
                raise ConfigError(
                    "initial omega' is unresolved (fll.initial_frequency and "
                    "dsogi.nominal_center_frequency are both null); pass "
                    "initial_omega=... explicitly"
                )
            initial_omega = _TWO_PI * initial_hz

        min_hz = get_parameter(observer_config, "fll.min_frequency", unit="Hz")
        max_hz = get_parameter(observer_config, "fll.max_frequency", unit="Hz")
        return cls(
            damping_gain_k=damping_gain_k,
            fll_gain_gamma=fll_gain_gamma,
            initial_omega=initial_omega,
            min_omega=None if min_hz is None else _TWO_PI * min_hz,
            max_omega=None if max_hz is None else _TWO_PI * max_hz,
            normalize_error=normalize_error,
            positive_sequence=positive_sequence,
        )


@dataclass(frozen=True)
class DSOGIFLLOutput:
    """DSOGI-FLL diagnostics at one sample.

    Attributes:
        v_alpha, v_beta: Filtered outputs (positive sequence, or per-axis band-pass).
        qv_alpha, qv_beta: Quadrature SOGI outputs per axis.
        omega: Resonance frequency ``omega'`` [rad/s] after the FLL update.
        frequency_hz: ``omega' / (2 pi)``.
        epsilon_f: FLL frequency-error signal at this sample.
    """

    v_alpha: float
    v_beta: float
    qv_alpha: float
    qv_beta: float
    omega: float
    frequency_hz: float
    epsilon_f: float


class DSOGIFLL:
    """Dual SOGI + positive-sequence extraction + FLL.

    The observer sees only the ``alpha-beta`` signal and ``dt``. State (two SOGI
    biquads and ``omega'``) persists across calls.

    Args:
        parameters: Validated DSOGI-FLL parameters.
    """

    def __init__(self, parameters: DSOGIFLLParameters) -> None:
        self.parameters = parameters
        self._alpha = SOGIQSG(parameters.damping_gain_k)
        self._beta = SOGIQSG(parameters.damping_gain_k)
        self._omega = parameters.initial_omega
        self._epsilon_filtered = 0.0

    @property
    def omega(self) -> float:
        """Current resonance frequency ``omega'`` [rad/s]."""
        return self._omega

    @property
    def frequency_hz(self) -> float:
        """Current resonance frequency [Hz]."""
        return self._omega / _TWO_PI

    def reset(self, omega: float | None = None) -> None:
        """Reset the SOGI states, the error filter and (optionally) the frequency."""
        self._alpha.reset()
        self._beta.reset()
        self._epsilon_filtered = 0.0
        self._omega = self.parameters.initial_omega if omega is None else omega

    def step(self, v_alpha: float, v_beta: float, dt: float) -> DSOGIFLLOutput:
        """Filter one ``alpha-beta`` sample and update ``omega'``.

        Args:
            v_alpha, v_beta: Input signals (e.g. estimated back-EMF components).
            dt: Sample period [s] (> 0).

        Returns:
            :class:`DSOGIFLLOutput` at the new sample.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0.0:
            raise ValueError(f"timestep dt must be positive, got {dt}")

        parameters = self.parameters
        alpha: SOGIQSGOutput = self._alpha.step(v_alpha, self._omega, dt)
        beta: SOGIQSGOutput = self._beta.step(v_beta, self._omega, dt)

        epsilon_f = alpha.quadrature * (v_alpha - alpha.in_phase) + beta.quadrature * (
            v_beta - beta.in_phase
        )
        if parameters.normalize_error:
            norm = (
                alpha.in_phase**2
                + alpha.quadrature**2
                + beta.in_phase**2
                + beta.quadrature**2
            )
            epsilon_f = epsilon_f / norm if norm > 0.0 else 0.0
        if parameters.epsilon_filter_tau is not None:
            self._epsilon_filtered += (dt / parameters.epsilon_filter_tau) * (
                epsilon_f - self._epsilon_filtered
            )
            epsilon_f = self._epsilon_filtered
        else:
            self._epsilon_filtered = epsilon_f

        self._omega += -parameters.fll_gain_gamma * epsilon_f * dt
        if parameters.min_omega is not None:
            self._omega = max(self._omega, parameters.min_omega)
        if parameters.max_omega is not None:
            self._omega = min(self._omega, parameters.max_omega)

        if parameters.positive_sequence:
            out_alpha = 0.5 * (alpha.in_phase - beta.quadrature)
            out_beta = 0.5 * (alpha.quadrature + beta.in_phase)
        else:
            out_alpha, out_beta = alpha.in_phase, beta.in_phase

        return DSOGIFLLOutput(
            v_alpha=out_alpha,
            v_beta=out_beta,
            qv_alpha=alpha.quadrature,
            qv_beta=beta.quadrature,
            omega=self._omega,
            frequency_hz=self._omega / _TWO_PI,
            epsilon_f=epsilon_f,
        )
