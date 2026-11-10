"""Closed-loop field-oriented (FOC) drive with encoder (true-angle) feedback.

Phase 06a: the speed and current loops are closed on the **true** rotor
angle/speed (an "encoder baseline"), so the controller and the mechanical loop are
validated *before* any estimator enters the loop (Phase 06b). No observer, filter
or estimator is used in this module.

Signal chain (Fig.7, p.828, printed): speed PI -> ``i_q*``; ``i_d* = 0``; two
current PIs -> ``dq/alpha-beta`` -> voltage limit -> inverter -> PMSM. The
controller never imports an observer, filter, fuzzy or estimation module; it
receives the sampled ``i_alpha/beta`` and an ``(theta_e, omega_e)`` pair through a
pluggable :class:`AngleSpeedSource`. Only :class:`TrueSource` (ground truth,
labelled *encoder baseline*) reads the plant state, so Phase 06b can swap in an
estimator-based source without touching the controller.

Plant (Eq.(1)-(2), p.2) in the rotor frame, with ``Ld = Lq`` (the paper's machine
is non-salient), ``K_t = (3/2) p psi_f`` and back-EMF amplitude ``omega_e psi_f``:

.. math::

    L_d \\dot i_d = v_d - R_s i_d + \\omega_e L_d i_q, \\qquad
    L_d \\dot i_q = v_q - R_s i_q - \\omega_e L_d i_d - \\omega_e \\psi_f

    J \\dot\\omega_m = \\tfrac{3}{2} p \\psi_f i_q - T_L - B \\omega_e / p

Design formulas (ASSUMPTION — not from paper; D-047). Current loops:
``Kp_c = \\alpha_c L_d``, ``Ki_c = \\alpha_c R_s`` with ``\\alpha_c = 2\\pi 500`` Hz;
the PI zero cancels the plant pole ``R_s/L_d`` so the open loop is ``\\alpha_c/s``.
Speed loop: ``Kp_\\omega = 2\\zeta\\alpha_\\omega J/K_t``,
``Ki_\\omega = \\alpha_\\omega^2 J/K_t`` with ``\\zeta = 1`` and
``\\alpha_\\omega = 2\\pi 50`` Hz. Decoupling is ON, back-EMF feed-forward is OFF.

Inverter (ASSUMPTION): average-value, phase-voltage limit ``V_dc/\\sqrt3``, and a
**one-sample computation delay** -- the voltage computed at step ``k`` is applied
over ``[k+1, k+2)``. The plant is advanced with RK4 over ``n_sub >= 4`` sub-steps
per control period and a zero-order-held voltage. No dead time or PWM here.

Angle convention: ``theta_e`` uses the plant/paper convention ``E = omega_e psi_f
[-\\sin\\theta_e, \\cos\\theta_e]``; the amplitude-invariant Clarke/Park pair
(``alpha`` aligned with phase ``a``) is used, so ``i_d = i_q = 0`` at rest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Protocol

import numpy as np

from pmsm_sensorless.config import resolve_parameter
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMState
from pmsm_sensorless.simulation.nominal_drive import rated_current_from_torque

__all__ = [
    "AngleSpeedSource",
    "CurrentController",
    "DriveParameters",
    "DriveResult",
    "DriveSample",
    "SpeedController",
    "TrueSource",
    "run_drive",
]

_SQRT3 = math.sqrt(3.0)
_TWO_PI = 2.0 * math.pi
_RPM_TO_RAD_S = _TWO_PI / 60.0


def _park(i_alpha: float, i_beta: float, theta_e: float) -> tuple[float, float]:
    """Amplitude-invariant Park transform ``(i_alpha, i_beta) -> (i_d, i_q)``."""
    c, s = math.cos(theta_e), math.sin(theta_e)
    return i_alpha * c + i_beta * s, -i_alpha * s + i_beta * c


def _inv_park(v_d: float, v_q: float, theta_e: float) -> tuple[float, float]:
    """Inverse Park transform ``(v_d, v_q) -> (v_alpha, v_beta)``."""
    c, s = math.cos(theta_e), math.sin(theta_e)
    return v_d * c - v_q * s, v_d * s + v_q * c


@dataclass(frozen=True)
class DriveSample:
    """One sampled instant handed to an :class:`AngleSpeedSource`.

    Attributes:
        t: time [s].
        dt: control period [s].
        i_alpha, i_beta: measured stator currents [A].
        u_alpha, u_beta: last applied (ZOH) stator voltages [V].
    """

    t: float
    dt: float
    i_alpha: float
    i_beta: float
    u_alpha: float
    u_beta: float


class AngleSpeedSource(Protocol):
    """Pluggable rotor angle/speed source.

    Returns ``(theta_e, omega_e)`` in rad and rad/s. The ``truth`` argument is the
    ground-truth plant state and **must be read only by :class:`TrueSource`**; an
    estimator-based source (Phase 06b) derives the pair from ``sample`` alone.
    """

    name: str

    def __call__(self, sample: DriveSample, truth: PMSMState) -> tuple[float, float]:
        ...  # pragma: no cover - protocol


class TrueSource:
    """Encoder baseline: feeds the plant's true electrical angle and speed.

    This is the **only** ground-truth angle/speed path in the drive. It is
    deliberately labelled so the Phase 06b estimator source can replace it.
    """

    name = "true_encoder"

    def __call__(self, sample: DriveSample, truth: PMSMState) -> tuple[float, float]:
        return truth.theta_e, truth.omega_e


@dataclass(frozen=True)
class DriveParameters:
    """Validated drive/controller parameters and the gains derived from them.

    Printed items come from ``configs/drive.yaml``; every other value is an
    ASSUMPTION in ``configs/drive_assumptions.yaml`` (see D-047).
    """

    # Machine (printed; pmsm.yaml)
    pole_pairs: int
    stator_resistance: float
    ld: float
    psi_f: float
    inertia: float
    # Assumed machine/mechanical
    viscous_friction: float
    # Inverter (assumed)
    v_dc: float
    delay_samples: int
    # Loops (frequencies printed; bandwidths assumed)
    current_loop_hz: float
    speed_loop_hz: float
    alpha_c_hz: float
    alpha_w_hz: float
    zeta_speed: float
    i_d_ref: float
    i_q_clamp: float
    n_substeps: int
    decoupling: bool
    back_emf_compensation: bool

    def __post_init__(self) -> None:
        if self.pole_pairs < 1:
            raise ValueError(f"pole_pairs must be >= 1, got {self.pole_pairs}")
        for name in ("stator_resistance", "ld", "psi_f", "inertia", "v_dc"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        if self.viscous_friction < 0.0:
            raise ValueError(f"viscous_friction must be non-negative, got {self.viscous_friction}")
        if self.current_loop_hz <= 0.0 or self.speed_loop_hz <= 0.0:
            raise ValueError("loop frequencies must be positive")
        if self.speed_loop_hz > self.current_loop_hz:
            raise ValueError("the speed loop cannot be faster than the current loop")
        if self.current_loop_hz % self.speed_loop_hz != 0.0:
            raise ValueError(
                f"current_loop_hz ({self.current_loop_hz}) must be an integer multiple "
                f"of speed_loop_hz ({self.speed_loop_hz})"
            )
        if self.alpha_c_hz <= 0.0 or self.alpha_w_hz <= 0.0:
            raise ValueError("loop bandwidths must be positive")
        if self.zeta_speed <= 0.0:
            raise ValueError(f"zeta_speed must be positive, got {self.zeta_speed}")
        if self.i_q_clamp <= 0.0:
            raise ValueError(f"i_q_clamp must be positive, got {self.i_q_clamp}")
        if self.n_substeps < 4:
            raise ValueError(f"n_substeps must be >= 4 (>=4 per Ts), got {self.n_substeps}")
        if self.delay_samples < 0:
            raise ValueError(f"delay_samples must be >= 0, got {self.delay_samples}")

    # --- derived quantities -------------------------------------------------
    @property
    def ts_current(self) -> float:
        """Current-loop / control period [s]."""
        return 1.0 / self.current_loop_hz

    @property
    def ts_speed(self) -> float:
        """Speed-loop period [s]."""
        return 1.0 / self.speed_loop_hz

    @property
    def speed_divider(self) -> int:
        """Current-loop steps per speed-loop update."""
        return int(round(self.current_loop_hz / self.speed_loop_hz))

    @property
    def v_limit(self) -> float:
        """Phase-voltage amplitude limit ``V_dc / sqrt(3)`` [V]."""
        return self.v_dc / _SQRT3

    @property
    def torque_constant(self) -> float:
        """``K_t = (3/2) p psi_f`` [N.m/A]."""
        return 1.5 * self.pole_pairs * self.psi_f

    @property
    def kp_current(self) -> float:
        """Current-loop proportional gain ``alpha_c Ld`` [V/A]."""
        return _TWO_PI * self.alpha_c_hz * self.ld

    @property
    def ki_current(self) -> float:
        """Current-loop integral gain ``alpha_c Rs`` [V/(A.s)]."""
        return _TWO_PI * self.alpha_c_hz * self.stator_resistance

    @property
    def kp_speed(self) -> float:
        """Speed-loop proportional gain ``2 zeta alpha_w J / Kt`` [A/(rad/s)]."""
        return 2.0 * self.zeta_speed * _TWO_PI * self.alpha_w_hz * self.inertia / self.torque_constant

    @property
    def ki_speed(self) -> float:
        """Speed-loop integral gain ``alpha_w^2 J / Kt`` [A/rad]."""
        return (_TWO_PI * self.alpha_w_hz) ** 2 * self.inertia / self.torque_constant

    @classmethod
    def from_configs(
        cls,
        drive_config: dict[str, Any],
        assumptions_config: dict[str, Any],
        pmsm_config: dict[str, Any],
    ) -> "DriveParameters":
        """Build the drive parameters from printed + assumed YAML configs.

        Raises:
            ConfigError: If any required printed or assumed value is absent or null.
        """
        pole_pairs = resolve_parameter(pmsm_config, "machine.pole_pairs", unit="-")
        psi_f = resolve_parameter(pmsm_config, "machine.pm_flux_linkage_psi_f", unit="Wb")
        i_rated = rated_current_from_torque(
            resolve_parameter(pmsm_config, "rating.rated_torque", unit="N.m"),
            int(pole_pairs),
            psi_f,
        )
        return cls(
            pole_pairs=int(pole_pairs),
            stator_resistance=resolve_parameter(pmsm_config, "machine.stator_resistance_rs", unit="ohm"),
            ld=resolve_parameter(pmsm_config, "machine.stator_inductance_ld", unit="H"),
            psi_f=psi_f,
            inertia=resolve_parameter(pmsm_config, "mech.inertia_j", unit="kg.m^2"),
            viscous_friction=resolve_parameter(
                assumptions_config, "machine.viscous_friction_b", unit="N.m.s/rad"
            ),
            v_dc=resolve_parameter(assumptions_config, "inverter.dc_bus_voltage", unit="V"),
            delay_samples=int(resolve_parameter(assumptions_config, "inverter.delay_samples", unit="-")),
            current_loop_hz=resolve_parameter(
                drive_config, "control.current_loop_frequency", unit="Hz"
            ),
            speed_loop_hz=resolve_parameter(drive_config, "control.speed_loop_frequency", unit="Hz"),
            alpha_c_hz=resolve_parameter(
                assumptions_config, "current_loop.bandwidth_alpha_c_hz", unit="Hz"
            ),
            alpha_w_hz=resolve_parameter(
                assumptions_config, "speed_loop.bandwidth_alpha_w_hz", unit="Hz"
            ),
            zeta_speed=resolve_parameter(assumptions_config, "speed_loop.damping_zeta", unit="-"),
            i_d_ref=resolve_parameter(drive_config, "control.i_d_reference", unit="A"),
            i_q_clamp=resolve_parameter(
                assumptions_config, "speed_loop.i_q_clamp_factor", unit="-"
            )
            * i_rated,
            n_substeps=int(resolve_parameter(assumptions_config, "integration.substeps_per_ts", unit="-")),
            decoupling=bool(resolve_parameter(assumptions_config, "current_loop.decoupling", unit="-")),
            back_emf_compensation=bool(
                resolve_parameter(assumptions_config, "current_loop.back_emf_compensation", unit="-")
            ),
        )


class CurrentController:
    """Two dq current PIs with decoupling and vector voltage limiting.

    Anti-windup: conditional integration. The integrator is only advanced when the
    (unclamped) output is inside the limit or the error would move it back inside;
    the integrator and the output are clamped at the voltage limit ``V_dc/sqrt3``.
    """

    def __init__(self, parameters: DriveParameters) -> None:
        self.parameters = parameters
        self._integral_d = 0.0
        self._integral_q = 0.0

    def reset(self) -> None:
        """Zero both integrators."""
        self._integral_d = 0.0
        self._integral_q = 0.0

    @property
    def integral_d(self) -> float:
        return self._integral_d

    @property
    def integral_q(self) -> float:
        return self._integral_q

    def step(
        self,
        i_d: float,
        i_q: float,
        i_d_ref: float,
        i_q_ref: float,
        omega_e: float,
        dt: float,
    ) -> tuple[float, float, bool]:
        """One current-control update.

        Returns:
            ``(v_d, v_q, saturated)`` in volt; ``saturated`` is true when the
            voltage vector hit the ``V_dc/sqrt(3)`` limit.
        """
        p = self.parameters
        limit = p.v_limit
        err_d = i_d_ref - i_d
        err_q = i_q_ref - i_q

        v_d = p.kp_current * err_d + self._integral_d
        v_q = p.kp_current * err_q + self._integral_q
        if p.decoupling:
            v_d -= omega_e * p.ld * i_q
            v_q += omega_e * p.ld * i_d
        if p.back_emf_compensation:
            v_q += omega_e * p.psi_f

        magnitude = math.hypot(v_d, v_q)
        saturated = magnitude > limit
        if saturated:
            scale = limit / magnitude
            v_d *= scale
            v_q *= scale
        else:
            # conditional integration: the integrator only advances when the
            # output is inside the limit (freeze at saturation), and is clamped.
            self._integral_d = _clamp(self._integral_d + p.ki_current * err_d * dt, limit)
            self._integral_q = _clamp(self._integral_q + p.ki_current * err_q * dt, limit)
        return v_d, v_q, saturated


class SpeedController:
    """Mechanical-speed PI producing ``i_q*``, clamped at ``+/- i_q_clamp``.

    Anti-windup: conditional integration with the integrator clamped at the
    ``i_q*`` limit.
    """

    def __init__(self, parameters: DriveParameters) -> None:
        self.parameters = parameters
        self._integral = 0.0

    def reset(self) -> None:
        """Zero the integrator."""
        self._integral = 0.0

    @property
    def integral(self) -> float:
        return self._integral

    def step(self, omega_m_ref: float, omega_m: float, dt: float) -> tuple[float, bool]:
        """Return ``(i_q_ref, saturated)`` for one speed-loop update."""
        p = self.parameters
        limit = p.i_q_clamp
        err = omega_m_ref - omega_m
        i_q_ref = p.kp_speed * err + self._integral
        saturated = abs(i_q_ref) > limit
        i_q_ref = _clamp(i_q_ref, limit)
        if not saturated:
            # conditional integration: advance only when not clamped.
            self._integral = _clamp(self._integral + p.ki_speed * err * dt, limit)
        return i_q_ref, saturated


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


@dataclass(frozen=True)
class DriveResult:
    """One drive rollout; all arrays share the time base ``t``.

    ``u_alpha/u_beta`` are the **applied** (delayed, ZOH) voltages the plant sees;
    ``u_cmd_alpha/u_cmd_beta`` are the controller commands. ``i_d/i_q`` and the
    ``v_d/v_q`` derived from them use the **true** angle (evaluation layer).
    """

    t: np.ndarray
    u_alpha: np.ndarray
    u_beta: np.ndarray
    u_cmd_alpha: np.ndarray
    u_cmd_beta: np.ndarray
    i_alpha: np.ndarray
    i_beta: np.ndarray
    i_d: np.ndarray
    i_q: np.ndarray
    i_d_ref: np.ndarray
    i_q_ref: np.ndarray
    v_d: np.ndarray
    v_q: np.ndarray
    theta_e: np.ndarray
    omega_e: np.ndarray
    omega_m: np.ndarray
    omega_m_ref: np.ndarray
    torque_e: np.ndarray
    load_torque: np.ndarray
    saturated: np.ndarray
    v_limit: float

    @property
    def u_cmd_magnitude(self) -> np.ndarray:
        """Commanded voltage-vector magnitude [V]."""
        return np.hypot(self.u_cmd_alpha, self.u_cmd_beta)

    @property
    def saturation_fraction(self) -> float:
        """Fraction of steps with the voltage command at the limit."""
        return float(np.mean(self.saturated)) if len(self.t) else 0.0


def _as_callable(reference: float | Callable[[float], float]) -> Callable[[float], float]:
    if callable(reference):
        return reference
    return lambda _t: float(reference)


def run_drive(
    plant: PMSMPlant,
    parameters: DriveParameters,
    *,
    ts: float | None = None,
    duration: float,
    load_torque: Callable[[float], float] = lambda _t: 0.0,
    source: AngleSpeedSource | None = None,
    speed_ref_rpm: float | Callable[[float], float] | None = None,
    i_q_ref: float | Callable[[float], float] | None = None,
    omega_e_fixed: float | None = None,
    initial_state: PMSMState | None = None,
) -> DriveResult:
    """Run the closed-loop drive for ``duration`` seconds.

    Exactly one of ``speed_ref_rpm`` (closed speed loop; mechanical plant) or
    ``i_q_ref`` (direct q-current mode) must be given. In direct mode a
    prescribed-kinematics plant needs ``omega_e_fixed``.

    Args:
        plant: The machine. Mechanical mode integrates speed/angle; prescribed
            kinematics holds ``omega_e``.
        parameters: Validated drive parameters.
        ts: Control period [s]; defaults to ``parameters.ts_current``.
        duration: Rollout length [s].
        load_torque: ``T_L(t)`` in N.m (used only in mechanical mode).
        source: Angle/speed source; defaults to :class:`TrueSource`.
        speed_ref_rpm: Mechanical speed reference [r/min], constant or ``t -> rpm``.
        i_q_ref: Direct q-current reference [A], constant or ``t -> A``.
        omega_e_fixed: Prescribed electrical speed [rad/s] (prescribed plant).
        initial_state: Starting :class:`PMSMState`; defaults to rest at
            ``omega_e_fixed``.

    Raises:
        ValueError: On an inconsistent mode/plant combination.

    Returns:
        :class:`DriveResult`.
    """
    p = parameters
    dt = p.ts_current if ts is None else ts
    if dt <= 0.0 or duration <= 0.0:
        raise ValueError(f"ts and duration must be positive, got ts={dt}, duration={duration}")
    speed_mode = speed_ref_rpm is not None
    current_mode = i_q_ref is not None
    if speed_mode == current_mode:
        raise ValueError("exactly one of speed_ref_rpm / i_q_ref must be given")
    if speed_mode and not plant.integrate_mechanics:
        raise ValueError("the speed loop requires plant.integrate_mechanics=True")
    if current_mode and not plant.integrate_mechanics and omega_e_fixed is None:
        raise ValueError("direct current mode on a prescribed plant requires omega_e_fixed")

    source = source or TrueSource()
    steps = int(round(duration / dt))
    n_sub = p.n_substeps
    dt_sub = dt / n_sub
    speed_ref = _as_callable(speed_ref_rpm) if speed_mode else None
    i_q_ref_fn = _as_callable(i_q_ref) if current_mode else None

    if initial_state is None:
        omega0 = omega_e_fixed if omega_e_fixed is not None else 0.0
        state = PMSMState(0.0, 0.0, omega0, 0.0)
    else:
        state = initial_state

    current = CurrentController(p)
    speed = SpeedController(p)

    names = (
        "u_alpha", "u_beta", "u_cmd_alpha", "u_cmd_beta", "i_alpha", "i_beta",
        "i_d", "i_q", "i_d_ref", "i_q_ref", "v_d", "v_q", "theta_e", "omega_e",
        "omega_m", "omega_m_ref", "torque_e", "load_torque",
    )
    out = {name: np.empty(steps) for name in names}
    saturated = np.zeros(steps, dtype=bool)

    u_applied = np.zeros(2)
    i_q_cmd = 0.0
    for k in range(steps):
        t = k * dt
        sample = DriveSample(
            t=t, dt=dt, i_alpha=state.i_alpha, i_beta=state.i_beta,
            u_alpha=float(u_applied[0]), u_beta=float(u_applied[1]),
        )
        theta_src, omega_src = source(sample, state)

        i_d, i_q = _park(state.i_alpha, state.i_beta, theta_src)
        if speed_mode:
            if k % p.speed_divider == 0:
                omega_m_ref = speed_ref(t) * _RPM_TO_RAD_S
                i_q_cmd, _ = speed.step(omega_m_ref, omega_src / p.pole_pairs, p.ts_speed)
            i_q_ref_k = i_q_cmd
        else:
            omega_m_ref = omega_src / p.pole_pairs
            i_q_ref_k = i_q_ref_fn(t)

        v_d, v_q, sat = current.step(i_d, i_q, p.i_d_ref, i_q_ref_k, omega_src, dt)
        u_cmd = _inv_park(v_d, v_q, theta_src)

        out["u_alpha"][k], out["u_beta"][k] = u_applied
        out["u_cmd_alpha"][k], out["u_cmd_beta"][k] = u_cmd
        out["i_alpha"][k], out["i_beta"][k] = state.i_alpha, state.i_beta
        out["i_d"][k], out["i_q"][k] = i_d, i_q
        out["i_d_ref"][k], out["i_q_ref"][k] = p.i_d_ref, i_q_ref_k
        out["v_d"][k], out["v_q"][k] = _park(u_applied[0], u_applied[1], state.theta_e)
        out["theta_e"][k], out["omega_e"][k] = state.theta_e, state.omega_e
        out["omega_m"][k] = state.omega_e / p.pole_pairs
        out["omega_m_ref"][k] = omega_m_ref
        out["torque_e"][k] = plant.electromagnetic_torque(
            (state.i_alpha, state.i_beta), state.theta_e
        )
        out["load_torque"][k] = load_torque(t)
        saturated[k] = sat

        for sub in range(n_sub):
            state = plant.step(
                state, float(u_applied[0]), float(u_applied[1]), dt_sub,
                load_torque=load_torque(t + sub * dt_sub),
            )
        u_applied = np.array(u_cmd)

    return DriveResult(t=np.arange(steps) * dt, saturated=saturated, v_limit=p.v_limit, **out)
