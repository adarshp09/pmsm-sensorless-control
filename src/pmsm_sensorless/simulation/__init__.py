"""Simulation scenarios and integration loop.

Holds scenario generators that drive the plant with realistic inputs.

- :mod:`pmsm_sensorless.simulation.nominal_drive` (Phase 04c) is an open-loop
  ideal-FOC feedforward drive: it exercises the observer's voltage input, but it is
  plant-side only -- no controller and no closed loop, nothing from an observer
  feeds back into the plant.
- :mod:`pmsm_sensorless.simulation.drive` (Phase 06a) is a closed-loop FOC drive
  with true-angle (encoder) feedback. The controller imports no observer/filter/
  fuzzy/estimation module; the estimated angle/speed enters only through a pluggable
  ``AngleSpeedSource``, whose sole implementation (``TrueSource``) is the encoder
  baseline.
"""
