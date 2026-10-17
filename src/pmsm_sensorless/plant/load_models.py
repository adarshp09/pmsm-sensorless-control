"""Deterministic load-torque profiles for the optional mechanical plant.

The paper applies a load step but does not print the mechanical equation, so these
profiles are simply deterministic functions of time used only with the opt-in
mechanical extension of :class:`~pmsm_sensorless.plant.pmsm.PMSMPlant`. A profile
is any callable ``T_L(t) -> float`` in N.m, positive opposing positive rotation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

__all__ = ["ConstantLoadTorque", "LoadTorque", "StepLoadTorque"]


class LoadTorque(Protocol):
    """A load-torque profile: ``t`` [s] -> ``T_L`` [N.m]."""

    def __call__(self, t: float) -> float:  # pragma: no cover - protocol
        ...


@dataclass(frozen=True)
class ConstantLoadTorque:
    """A constant load torque.

    Args:
        torque: Torque in N.m.
    """

    torque: float

    def __call__(self, t: float) -> float:
        return self.torque


@dataclass(frozen=True)
class StepLoadTorque:
    """A load torque that steps from one value to another at ``step_time``.

    Args:
        initial_torque: Torque in N.m before the step.
        final_torque: Torque in N.m at and after the step.
        step_time: Step instant in seconds (>= 0).
    """

    initial_torque: float
    final_torque: float
    step_time: float

    def __post_init__(self) -> None:
        if self.step_time < 0.0:
            raise ValueError(f"step_time must be non-negative, got {self.step_time}")

    def __call__(self, t: float) -> float:
        return self.final_torque if t >= self.step_time else self.initial_torque
