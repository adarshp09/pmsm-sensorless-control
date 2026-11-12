"""Nominal-drive scenario: an open-loop, ideal-FOC feedforward for the plant.

This module drives the plant with the voltage a perfect current controller *would*
apply, so that the observer's voltage input is exercised in an integrated run
instead of the ``u = 0`` short-circuit scenario used through Phase 04b.

It is **plant-side only** and may use ground truth (the true angle/speed/back-EMF),
because the observer never sees its output: nothing from an observer feeds back
into the generator (there is no controller and no closed loop). The plant is
advanced with the deterministic RK4 path of :class:`~pmsm_sensorless.plant.pmsm.PMSMPlant`.

Ideal feedforward
-----------------
The reference current is placed on the q-axis (``i_d = 0``) and made in phase with
the back-EMF of Eq.(2), p.2,

.. math::

    i_\\mathrm{ref}(t) = I_q\\,[-\\sin\\theta_e(t),\\; \\cos\\theta_e(t)] ,

so that, under the amplitude-invariant Clarke convention (``alpha`` aligned with
phase ``a``, ``beta`` leading by 90 deg), the q-axis current equals the frame
amplitude, ``i_q = I_q``. The stator voltage that makes the plant current track
``i_ref`` exactly is the inverse of Eq.(1), p.2:

.. math::

    u_\\alpha\\beta = R_s\\, i_\\mathrm{ref} + L_d\\, \\frac{d i_\\mathrm{ref}}{dt}
                    + E_\\mathrm{true}, \\qquad
    \\frac{d i_\\mathrm{ref}}{dt} = I_q\\,\\omega_e\\,[-\\cos\\theta_e,\\; -\\sin\\theta_e] .

``u`` is held constant over each sample period (zero-order hold, inverter-like), so
the tracked current lags the reference by the usual discretization residual -- the
Ts = 10 us test bounds it well inside 2 % of ``I_q`` (see ``tests/test_nominal_drive.py``).

The rated (q-axis) current is derived from the paper's rated torque, pole pairs and
flux linkage, :math:`I_\\mathrm{rated} = T_\\mathrm{rated} / (\\tfrac{3}{2} p \\psi_f)`
(valid for ``i_d = 0`` with ``L_d = L_q``), not taken from any printed current rating.

Scope: raw plant/observer signals only. No DSOGI-FLL, no rotor position/speed
estimation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMState

__all__ = [
    "NominalDriveResult",
    "electrical_speed_e",
    "nominal_drive_rollout",
    "rated_current_from_torque",
]

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def electrical_speed_e(rpm: float, pole_pairs: int) -> float:
    """Return the electrical speed ``omega_e`` [rad/s] for a mechanical ``rpm``."""
    return pole_pairs * rpm * _RPM_TO_RAD_S


def rated_current_from_torque(rated_torque: float, pole_pairs: int, psi_f: float) -> float:
    """Return the q-axis rated current ``I_rated`` [A] for ``i_d = 0``.

    From the (derived) torque relation ``T_e = (3/2) p psi_f i_q`` used by
    :meth:`PMSMPlant.electromagnetic_torque`: ``I_rated = T_rated /
    ((3/2) p psi_f)``.

    Raises:
        ValueError: If any argument is non-positive.
    """
    if rated_torque <= 0.0:
        raise ValueError(f"rated_torque must be positive, got {rated_torque}")
    if pole_pairs < 1:
        raise ValueError(f"pole_pairs must be >= 1, got {pole_pairs}")
    if psi_f <= 0.0:
        raise ValueError(f"psi_f must be positive, got {psi_f}")
    return rated_torque / (1.5 * pole_pairs * psi_f)


@dataclass(frozen=True)
class NominalDriveResult:
    """One nominal-drive rollout (all arrays share the time base ``t``).

    Attributes:
        t: sample instants [s].
        u_alpha, u_beta: applied (ZOH) stator voltages [V].
        i_alpha, i_beta: plant stator currents [A] at each sample instant.
        i_ref_alpha, i_ref_beta: reference currents [A] in phase with the back-EMF.
        e_alpha, e_beta: true back-EMF [V] (ground truth; plant side only).
        theta_e: true electrical angle [rad].
        omega_e: prescribed electrical speed [rad/s].
        i_q: reference amplitude ``I_q`` [A].
    """

    t: np.ndarray
    u_alpha: np.ndarray
    u_beta: np.ndarray
    i_alpha: np.ndarray
    i_beta: np.ndarray
    i_ref_alpha: np.ndarray
    i_ref_beta: np.ndarray
    e_alpha: np.ndarray
    e_beta: np.ndarray
    theta_e: np.ndarray
    omega_e: np.ndarray
    i_q: float

    @property
    def tracking_error_max_a(self) -> float:
        """Max ``|i - i_ref|`` over the rollout [A]."""
        return float(
            np.max(np.hypot(self.i_alpha - self.i_ref_alpha, self.i_beta - self.i_ref_beta))
        )


class _SpeedProfile:
    """Prescribed no-load speed ramp (default 500 -> 1000 rpm over 1.5 s)."""

    def __init__(self, rpm_start: float, rpm_end: float, ramp_time: float, pole_pairs: int):
        if pole_pairs < 1:
            raise ValueError(f"pole_pairs must be >= 1, got {pole_pairs}")
        if ramp_time <= 0.0:
            raise ValueError(f"ramp_time must be positive, got {ramp_time}")
        self.rpm_start = rpm_start
        self.rpm_end = rpm_end
        self.ramp_time = ramp_time
        self._k = pole_pairs * _RPM_TO_RAD_S

    def rpm(self, t: float) -> float:
        ramp = min(t / self.ramp_time, 1.0) if t > 0.0 else 0.0
        return self.rpm_start + ramp * (self.rpm_end - self.rpm_start)

    def omega_e(self, t: float) -> float:
        return self._k * self.rpm(t)

    def theta_e(self, t: float) -> float:
        """Analytic integral of ``omega_e`` (continuous, unwrapped)."""
        delta = self.rpm_end - self.rpm_start
        if t <= self.ramp_time:
            return self._k * (self.rpm_start * t + delta * t * t / (2.0 * self.ramp_time))
        theta_at_ramp = self._k * self.ramp_time * (self.rpm_start + 0.5 * delta)
        return theta_at_ramp + self._k * self.rpm_end * (t - self.ramp_time)


def nominal_drive_rollout(
    plant: PMSMPlant,
    *,
    i_q: float,
    dt: float,
    duration: float,
    rpm_start: float = 500.0,
    rpm_end: float = 1000.0,
    ramp_time: float = 1.5,
) -> NominalDriveResult:
    """Run the plant under the ideal-FOC feedforward for ``duration`` seconds.

    The plant starts at ``i = i_ref(0)``. Each sample applies the ZOH voltage
    ``u = Rs i_ref + Ld di_ref/dt + E_true`` evaluated at the plant's true angle.

    Args:
        plant: A prescribed-kinematics plant (mechanical mode is not used here).
        i_q: reference current amplitude ``I_q`` [A] (> 0).
        dt: sample period [s] (> 0).
        duration: rollout length [s] (> 0).
        rpm_start, rpm_end: ramp endpoints [r/min].
        ramp_time: duration of the ramp [s] (> 0); speed is held after it.

    Raises:
        ValueError: If ``i_q``, ``dt`` or ``duration`` is non-positive.

    Returns:
        :class:`NominalDriveResult`.
    """
    if i_q <= 0.0:
        raise ValueError(f"i_q must be positive, got {i_q}")
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    if duration <= 0.0:
        raise ValueError(f"duration must be positive, got {duration}")

    profile = _SpeedProfile(rpm_start, rpm_end, ramp_time, plant.parameters.pole_pairs)
    steps = int(round(duration / dt))
    t = np.arange(steps) * dt
    out = {
        name: np.empty(steps)
        for name in (
            "u_alpha",
            "u_beta",
            "i_alpha",
            "i_beta",
            "i_ref_alpha",
            "i_ref_beta",
            "e_alpha",
            "e_beta",
            "theta_e",
            "omega_e",
        )
    }

    stator_resistance = plant.parameters.stator_resistance
    ld = plant.parameters.ld

    theta = profile.theta_e(0.0)
    omega = profile.omega_e(0.0)
    i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
    state = PMSMState(i_ref[0], i_ref[1], omega, theta)

    for n in range(steps):
        omega = profile.omega_e(t[n])
        state.omega_e = omega
        theta = state.theta_e
        i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
        di_ref = i_q * omega * np.array([-math.cos(theta), -math.sin(theta)])
        emf = plant.back_emf(theta, omega)
        u = stator_resistance * i_ref + ld * di_ref + emf

        out["u_alpha"][n], out["u_beta"][n] = u
        out["i_alpha"][n], out["i_beta"][n] = state.i_alpha, state.i_beta
        out["i_ref_alpha"][n], out["i_ref_beta"][n] = i_ref
        out["e_alpha"][n], out["e_beta"][n] = emf
        out["theta_e"][n], out["omega_e"][n] = theta, omega

        state = plant.step(state, float(u[0]), float(u[1]), dt)

    return NominalDriveResult(t=t, i_q=i_q, **out)
