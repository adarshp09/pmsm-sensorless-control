"""Phase 03b verification metrics (replaces the one-period moving-average metric).

Definitions (see ``docs/phase03.md``)
-------------------------------------
Represent an ``alpha-beta`` vector as ``z = e_alpha + j e_beta`` and let
``theta`` be the true electrical angle (evaluation layer only; the plant provides
it via ``E_true = j omega_e psi_f exp(j theta_e)``).

- **fundamental gain / phase** -- least-squares fit of the fundamental at the true
  electrical angle: ``P = mean(z exp(-j theta))`` (the minimiser of
  ``sum |z - P exp(j theta)|^2``). ``fundamental_gain = |P| / |P_true|`` and
  ``fundamental_phase_deg = angle(P / P_true)``.
- **ripple_rms_v** -- RMS of the residual after removing the fitted fundamental,
  ``sqrt(mean(|z - P exp(j theta)|^2))``. This is the switching/harmonic content.
- **total_error_rms_v** -- RMS of ``z - z_true``, i.e. the total deviation from the
  plant's true back-EMF.

All three are evaluation-layer quantities; they use ``E_true`` and ``theta_e``,
which the observer/filter never see.
"""

from __future__ import annotations

import math

import numpy as np


def electrical_angle(e_alpha, e_beta) -> np.ndarray:
    """True ``theta_e`` recovered from the true back-EMF (``E = j omega psi_f e^{j theta}``)."""
    z = np.asarray(e_alpha, dtype=float) + 1j * np.asarray(e_beta, dtype=float)
    return np.unwrap(np.angle(z)) - 0.5 * math.pi


def fundamental_phasor(e_alpha, e_beta, theta) -> complex:
    """Least-squares fundamental phasor ``P = mean(z exp(-j theta))``."""
    z = np.asarray(e_alpha, dtype=float) + 1j * np.asarray(e_beta, dtype=float)
    return complex(np.mean(z * np.exp(-1j * np.asarray(theta, dtype=float))))


def vector_metrics(e_alpha, e_beta, true_alpha, true_beta, theta) -> dict:
    """Fundamental gain/phase, ripple and total error for one signal vs the truth."""
    z = np.asarray(e_alpha, dtype=float) + 1j * np.asarray(e_beta, dtype=float)
    z_true = np.asarray(true_alpha, dtype=float) + 1j * np.asarray(true_beta, dtype=float)
    theta = np.asarray(theta, dtype=float)
    phasor = fundamental_phasor(e_alpha, e_beta, theta)
    phasor_true = fundamental_phasor(true_alpha, true_beta, theta)
    return {
        "fundamental_gain": float(abs(phasor) / abs(phasor_true)),
        "fundamental_phase_deg": float(np.degrees(np.angle(phasor / phasor_true))),
        "ripple_rms_v": float(np.sqrt(np.mean(np.abs(z - phasor * np.exp(1j * theta)) ** 2))),
        "total_error_rms_v": float(np.sqrt(np.mean(np.abs(z - z_true) ** 2))),
    }
