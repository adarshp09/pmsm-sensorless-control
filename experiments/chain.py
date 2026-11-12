"""Phase 06b: estimator chain as a drive angle/speed source (experiments only).

Composes the decoupled ``src`` modules into the paper's chain

    SMO ``E_hat``  ->  DSOGI-FLL ``v'``  ->  Q-PLL ``theta_hat, omega_hat``

and exposes it through the Phase 06a ``AngleSpeedSource`` interface. The composition
lives here so that ``src/`` stays decoupled (D-049/D-052): ``drive.py`` imports no
observer/filter/estimation module, and this module imports no plant.

Voltage alignment (Gate 1). The observer consumes the voltage **actually applied over
the interval that produced the current sample**: the drive's ``DriveSample`` carries
``u_alpha/u_beta`` = the last commanded (ZOH, delayed-by-one) voltage and
``i_alpha/i_beta`` = the current at the same sample instant. Feeding ``(sample.u,
sample.i)`` to ``observer.step`` reproduces the open-loop Phase 04c/05b alignment
exactly, which Gate 1 verifies against the 04c nominal-drive values.

The lock criterion in :class:`HandoverSource` uses only chain-internal signals
(``omega_hat`` and the FLL ``omega'``); it reads no ground truth. The encoder
start-up phase is the sole, labelled ground-truth path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from pmsm_sensorless import config
from pmsm_sensorless.estimation import QPLL, QPLLParameters
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters
from pmsm_sensorless.fuzzy.engine import FuzzyEngine
from pmsm_sensorless.observers.fuzzy_smo import FuzzySMO
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO
from pmsm_sensorless.observers.smo import SMOObserverParameters, per_unit_bases

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
K_S = math.sqrt(2.0)
GAMMA = 50.0
MATCHED = (1.0, 0.4)  # Phase 05b/06a nominal cell (kF, aF)
CELLS = ("matched_sigmoid", "flc_nominal")


@dataclass(frozen=True)
class Stack:
    """Validated configs and derived constants shared by the chain cells."""

    config_dir: Any
    pmsm_config: dict[str, Any]
    observer_config: dict[str, Any]
    assumptions_config: dict[str, Any]
    pole_pairs: int
    stator_resistance: float
    ld: float
    psi_f: float
    k0: float
    a0: float
    i_rated: float
    design_amp_v: float
    qpll_parameters: QPLLParameters

    def initial_omega_at(self, rpm: float) -> float:
        """Electrical ``omega_e`` for a mechanical ``rpm`` (design constant)."""
        return self.pole_pairs * rpm * RPM_TO_RAD_S


def build_stack(config_dir: Any = None) -> Stack:
    """Load the pmsm/observer/estimator configs and derive the chain constants."""
    directory = config_dir if config_dir is not None else config.default_config_dir()
    pmsm = config.load_config("pmsm", directory=directory)
    observer = config.load_config("observer", directory=directory)
    assumptions = config.load_config("estimator_assumptions", directory=directory)
    pole_pairs = int(config.resolve_parameter(pmsm, "machine.pole_pairs", unit="-"))
    psi_f = config.resolve_parameter(pmsm, "machine.pm_flux_linkage_psi_f", unit="Wb")
    v_base, i_base = per_unit_bases(pmsm)
    k0 = config.resolve_parameter(observer, "smo.switching_gain_k", unit="-") * v_base
    a0 = config.resolve_parameter(observer, "smo.sigmoid_slope_a", unit="-") / i_base
    i_rated = config.resolve_parameter(pmsm, "rating.rated_torque", unit="N.m") / (
        1.5 * pole_pairs * psi_f
    )
    design_amp = (pole_pairs * 1000.0 * RPM_TO_RAD_S) * psi_f
    return Stack(
        config_dir=directory,
        pmsm_config=pmsm,
        observer_config=observer,
        assumptions_config=assumptions,
        pole_pairs=pole_pairs,
        stator_resistance=config.resolve_parameter(
            pmsm, "machine.stator_resistance_rs", unit="ohm"
        ),
        ld=config.resolve_parameter(pmsm, "machine.stator_inductance_ld", unit="H"),
        psi_f=psi_f,
        k0=k0,
        a0=a0,
        i_rated=i_rated,
        design_amp_v=design_amp,
        qpll_parameters=QPLLParameters.from_configs(assumptions, design_amplitude_v=design_amp),
    )


def build_observer(cell: str, stack: Stack):
    """Build the SMO for a named cell (matched fixed sigmoid, or FLC nominal)."""
    if cell == "matched_sigmoid":
        return SigmoidSMO(
            SMOObserverParameters(
                stack.stator_resistance,
                stack.ld,
                MATCHED[0] * stack.k0,
                sigmoid_slope_a=MATCHED[1] * stack.a0,
            )
        )
    if cell == "flc_nominal":
        return FuzzySMO.from_config(
            stack.pmsm_config, stack.observer_config, fuzzy_directory=stack.config_dir
        )
    raise ValueError(f"unknown cell {cell!r}; expected one of {CELLS}")


def build_fll(
    initial_omega: float, *, min_omega: float | None = None, max_omega: float | None = None
) -> DSOGIFLL:
    """DSOGI-FLL at library defaults except optional frequency clamps.

    ``min_omega``/``max_omega`` (``None`` = no clamp) are electrical rad/s and are the
    only non-default options (Phase 06b-5).
    """
    return DSOGIFLL(
        DSOGIFLLParameters(K_S, GAMMA, initial_omega, min_omega=min_omega, max_omega=max_omega)
    )


def fll_min_omega_rad_s(stack: "Stack") -> float | None:
    """Lower ``omega'`` clamp from ``configs/drive_assumptions.yaml`` (mechanical rpm)."""
    cfg = config.load_config("drive_assumptions", directory=stack.config_dir)
    rpm = config.resolve_parameter(cfg, "fll.min_omega_rpm", unit="rpm")
    return stack.pole_pairs * rpm * RPM_TO_RAD_S


def build_qpll(stack: Stack, *, initial_theta: float = 0.0, initial_omega: float = 0.0) -> QPLL:
    return QPLL(stack.qpll_parameters, initial_theta=initial_theta, initial_omega=initial_omega)


def run_chain_open_loop(
    observer, fll, qpll, *, u_alpha, u_beta, i_alpha, i_beta, dt: float
) -> dict:
    """Run the chain over a fixed record (open loop); returns the recorded arrays."""
    n = len(u_alpha)
    out = {name: np.empty(n) for name in ("theta_hat", "omega_hat", "freq_hz", "e_alpha", "e_beta")}
    for k in range(n):
        s = observer.step(
            float(u_alpha[k]), float(u_beta[k]), float(i_alpha[k]), float(i_beta[k]), dt
        )
        f = fll.step(s.back_emf_alpha, s.back_emf_beta, dt)
        q = qpll.step(f.v_alpha_prime, f.v_beta_prime, dt)
        out["theta_hat"][k], out["omega_hat"][k] = q.theta_hat, q.omega_hat
        out["freq_hz"][k] = f.frequency_hz
        out["e_alpha"][k], out["e_beta"][k] = s.back_emf_alpha, s.back_emf_beta
    return out


class EstimatorSource:
    """The chain behind the Phase 06a ``AngleSpeedSource`` interface.

    ``__call__`` returns ``(theta_hat [rad], omega_hat [rad/s])`` with **electrical**
    speed, matching :class:`TrueSource`; the mechanical estimate is ``omega_hat / p``
    (exposed as :attr:`omega_hat_mechanical_rpm`). It ignores ``truth``.
    """

    name = "estimator_chain"

    def __init__(self, *, observer, fll, qpll, pole_pairs: int) -> None:
        self.observer = observer
        self.fll = fll
        self.qpll = qpll
        self.pole_pairs = pole_pairs
        self.theta_hat = 0.0
        self.omega_hat = float(qpll.omega_hat)
        self.last_freq_hz = math.nan
        self.s_alpha = 0.0
        self.s_beta = 0.0
        self.status = "ok"
        self.first_guard_fail_s: float | None = None
        self.last_e_mag = 0.0
        self.last_qpll_error = 0.0

    @property
    def omega_prime_rad_s(self) -> float:
        """FLL resonance frequency ``omega'`` [rad/s electrical]."""
        return TWO_PI * self.last_freq_hz if math.isfinite(self.last_freq_hz) else 0.0

    @property
    def omega_hat_mechanical_rpm(self) -> float:
        return self.omega_hat * 60.0 / (TWO_PI * self.pole_pairs)

    def __call__(self, sample, truth) -> tuple[float, float]:  # noqa: ARG002 (truth ignored)
        s = self.observer.step(
            sample.u_alpha, sample.u_beta, sample.i_alpha, sample.i_beta, sample.dt
        )
        try:
            f = self.fll.step(s.back_emf_alpha, s.back_emf_beta, sample.dt)
            self.last_freq_hz = f.frequency_hz
            v_alpha_prime, v_beta_prime = f.v_alpha_prime, f.v_beta_prime
        except ValueError:
            # The DSOGI guards ``omega' >= 0``; at library defaults (no clamps) a
            # diverging FLL reaches it. Freeze omega' and keep the last filtered
            # output so a run can be observed instead of aborting (experiments only).
            self.status = "guard-fail"
            if self.first_guard_fail_s is None:
                self.first_guard_fail_s = sample.t
            v_alpha_prime, v_beta_prime = s.back_emf_alpha, s.back_emf_beta
        q = self.qpll.step(v_alpha_prime, v_beta_prime, sample.dt)
        self.theta_hat, self.omega_hat = q.theta_hat, q.omega_hat
        self.s_alpha, self.s_beta = s.s_alpha, s.s_beta
        self.last_e_mag = math.hypot(v_alpha_prime, v_beta_prime)
        self.last_qpll_error = float(getattr(self.qpll, "last_error", 0.0))
        return q.theta_hat, q.omega_hat


@dataclass(frozen=True)
class HandoverAssumptions:
    """Start-up / handover constants (ASSUMPTION — not from paper; D-053)."""

    omega_h_rpm: float = 500.0
    speed_tolerance_rpm: float = 2.0
    lock_time_s: float = 0.05


def handover_assumptions(stack: Stack) -> HandoverAssumptions:
    """Read the handover assumptions from ``configs/drive_assumptions.yaml``."""
    cfg = config.load_config("drive_assumptions", directory=stack.config_dir)
    return HandoverAssumptions(
        omega_h_rpm=config.resolve_parameter(cfg, "handover.omega_h_rpm", unit="rpm"),
        speed_tolerance_rpm=config.resolve_parameter(
            cfg, "handover.speed_tolerance_rpm", unit="rpm"
        ),
        lock_time_s=config.resolve_parameter(cfg, "handover.lock_time_s", unit="s"),
    )


class HandoverSource:
    """Encoder-driven start-up, then switch to the estimator when the chain locks.

    The lock criterion is internal to the chain (no ground truth)::

        omega_hat_m >= omega_h  and  |omega_hat_m - omega'_m| < tol   for ``t_lock``

    (all in mechanical rpm). During start-up the source returns the plant truth
    (labelled encoder baseline); after handover it returns the estimates.
    """

    name = "handover"

    def __init__(
        self,
        chain: EstimatorSource,
        *,
        pole_pairs: int,
        assumptions: HandoverAssumptions,
        initial_theta: float = 0.0,
        initial_omega: float = 0.0,
        generic: bool = False,
        fixed_time_s: float | None = None,
    ) -> None:
        self.chain = chain
        self.pole_pairs = pole_pairs
        self.assumptions = assumptions
        self.generic = generic
        self.fixed_time_s = fixed_time_s
        self.mode = "encoder"
        self.lock_accumulator = 0.0
        self.switch_time_s: float | None = None
        self.theta_out = initial_theta
        self.omega_out = initial_omega

    @property
    def handed_over(self) -> bool:
        return self.mode == "estimator"

    def _to_rpm(self, omega_electrical: float) -> float:
        return omega_electrical * 60.0 / (TWO_PI * self.pole_pairs)

    def __call__(self, sample, truth) -> tuple[float, float]:
        theta_hat, omega_hat = self.chain(sample, truth)
        if self.mode == "encoder":
            if self.fixed_time_s is not None:
                # Fixed-time handover (Phase 06c-2, default OFF): identical for all
                # chains, chain runs in the background from the encoder start-up.
                if sample.t >= self.fixed_time_s:
                    self.mode = "estimator"
                    self.switch_time_s = sample.t
            else:
                omega_rpm = self._to_rpm(omega_hat)
                if self.generic:
                    # Generic criterion for chains without an FLL: no ground truth — Q-PLL
                    # phase error |Delta e|/|E_hat| < sin(5 deg) (Phase 06c, default OFF).
                    mag = max(getattr(self.chain, "last_e_mag", 0.0), 1e-9)
                    sin_phi = abs(getattr(self.chain, "last_qpll_error", 0.0)) / mag
                    locked = omega_rpm >= self.assumptions.omega_h_rpm and sin_phi < math.sin(
                        math.radians(5.0)
                    )
                else:
                    omega_prime_rpm = self._to_rpm(self.chain.omega_prime_rad_s)
                    locked = (
                        omega_rpm >= self.assumptions.omega_h_rpm
                        and abs(omega_rpm - omega_prime_rpm) < self.assumptions.speed_tolerance_rpm
                    )
                self.lock_accumulator = self.lock_accumulator + sample.dt if locked else 0.0
                if self.lock_accumulator >= self.assumptions.lock_time_s:
                    self.mode = "estimator"
                    self.switch_time_s = sample.t
            self.theta_out, self.omega_out = truth.theta_e, truth.omega_e
        else:
            self.theta_out, self.omega_out = theta_hat, omega_hat
        return self.theta_out, self.omega_out
