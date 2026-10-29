"""Simulation scenarios and integration loop.

Holds scenario generators that drive the plant with realistic inputs.
:mod:`pmsm_sensorless.simulation.nominal_drive` (Phase 04c) is an open-loop
ideal-FOC feedforward drive: it exercises the observer's voltage input, but it is
plant-side only -- no controller and no closed loop, nothing from an observer
feeds back into the plant.
"""
