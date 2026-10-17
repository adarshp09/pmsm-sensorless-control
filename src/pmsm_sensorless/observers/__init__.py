"""Sliding-mode observers.

Phase 02 implements the paper's current-observer, Eqs.(3)-(7), pp.2-3, in
:mod:`pmsm_sensorless.observers.conventional_smo` (sign switching) and
:mod:`pmsm_sensorless.observers.sigmoid_smo` (Eq.(5) sigmoid switching), sharing
:mod:`pmsm_sensorless.observers.smo`. The fuzzy controller, SOGI/DSOGI, FLL and the
rotor position/speed estimator are not implemented in this phase.
"""
