"""SOGI / DSOGI / DSOGI-FLL filters.

Phase 03 implements the SOGI-QSG (:mod:`pmsm_sensorless.filters.sogi`, Eqs.(9)-(10)
discretized with the paper's Tustin-with-prewarping, Eq.(17)) and the DSOGI-FLL
(:mod:`pmsm_sensorless.filters.dsogi_fll`: dual SOGI, positive-sequence extraction,
frequency-locked loop). The fuzzy controller and the position/speed estimator are
not implemented in this phase.
"""
