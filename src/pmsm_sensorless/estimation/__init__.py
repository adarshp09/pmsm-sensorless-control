"""Rotor position and speed estimation.

Phase 05b implements the paper's position estimators: the direct arctan form of
Eq.(8) and the quadrature PLL (Q-PLL) of Fig.1. Both take only the stationary
``alpha-beta`` back-EMF and a timestep, and import no plant, observers, filters or
fuzzy modules (the chain observer -> filter -> estimator is composed in
``experiments/``).
"""

from __future__ import annotations

from pmsm_sensorless.estimation.estimators import (
    ArctanEstimator,
    EstimatorOutput,
    QPLL,
    QPLLParameters,
    electrical_to_mechanical_speed,
    wrap_angle_pi_closed,
)

__all__ = [
    "ArctanEstimator",
    "EstimatorOutput",
    "QPLL",
    "QPLLParameters",
    "electrical_to_mechanical_speed",
    "wrap_angle_pi_closed",
]
