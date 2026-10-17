"""PMSM plant model.

Phase 01 implements the stationary (alpha-beta) electrical model of the paper's
Eq.(1)-(2) in :mod:`pmsm_sensorless.plant.pmsm`, plus deterministic load-torque
profiles in :mod:`pmsm_sensorless.plant.load_models`. An opt-in, explicitly
labelled mechanical extension is available but is not part of the paper's model.
"""

from __future__ import annotations
