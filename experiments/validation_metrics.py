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


def sliding_window_ramp_metrics(
    e_alpha,
    e_beta,
    true_alpha,
    true_beta,
    omega_prime_hz,
    f_true_hz,
    t,
    window_s: float = 0.06,
) -> list[dict]:
    """Per-window complex fit ``z ~ (G0 + G1 (t - t_c)) z_true`` over a ramp.

    A single stationary phasor is invalid while the frequency ramps, so each
    ``window_s`` window is fitted with a locally linear complex gain. ``G0`` gives the
    gain/phase at the window centre; the RMS residual is the ripple; the ``omega'``
    error is ``omega' - omega_true`` over the window.
    """
    z = np.asarray(e_alpha, dtype=float) + 1j * np.asarray(e_beta, dtype=float)
    z_true = np.asarray(true_alpha, dtype=float) + 1j * np.asarray(true_beta, dtype=float)
    t = np.asarray(t, dtype=float)
    omega_prime_hz = np.asarray(omega_prime_hz, dtype=float)
    f_true_hz = np.asarray(f_true_hz, dtype=float)
    samples = max(2, int(round(window_s / float(t[1] - t[0]))))
    rows: list[dict] = []
    start = 0
    while start + samples <= len(z):
        sl = slice(start, start + samples)
        zz, zz_true, tt = z[sl], z_true[sl], t[sl]
        centre = tt.mean()
        delta = tt - centre
        weight = np.abs(zz_true) ** 2
        normal = np.array(
            [[np.sum(weight), np.sum(delta * weight)], [np.sum(delta * weight), np.sum(delta**2 * weight)]]
        )
        rhs = np.array(
            [np.sum(zz * np.conj(zz_true)), np.sum(delta * zz * np.conj(zz_true))]
        )
        g0, g1 = np.linalg.solve(normal, rhs)
        residual = zz - (g0 + g1 * delta) * zz_true
        omega_error = omega_prime_hz[sl] - f_true_hz[sl]
        rows.append(
            {
                "t_center_s": float(centre),
                "gain": float(abs(g0)),
                "phase_deg": float(np.degrees(np.angle(g0))),
                "ripple_rms_v": float(np.sqrt(np.mean(np.abs(residual) ** 2))),
                "omega_prime_error_hz_mean": float(np.mean(omega_error)),
                "omega_prime_error_hz_std": float(np.std(omega_error)),
            }
        )
        start += samples
    return rows


def summarize_ramp(rows: list[dict]) -> dict:
    """Mean and worst (max-ripple) window of :func:`sliding_window_ramp_metrics`."""
    if not rows:
        return {}
    worst = max(rows, key=lambda row: row["ripple_rms_v"])
    keys = ("gain", "phase_deg", "ripple_rms_v", "omega_prime_error_hz_mean", "omega_prime_error_hz_std")
    return {
        "n_windows": len(rows),
        "mean": {key: float(np.mean([row[key] for row in rows])) for key in keys},
        "worst": dict(worst),
    }


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
