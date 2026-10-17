"""Fuzzy logic controller.

Phase 04 implements the Mamdani FLC in :mod:`pmsm_sensorless.fuzzy.engine`:
triangular Ruspini membership, the printed 5x5 rule bases (Tables I-II), min/max
inference and centre-of-gravity defuzzification. Printed items live in
``configs/fuzzy.yaml``; every unprinted item is an explicit assumption in
``configs/fuzzy_assumptions.yaml`` and raises if null.
"""
