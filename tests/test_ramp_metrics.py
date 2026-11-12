"""Phase 03f Step 6: the windowed ramp fit ``z ~ (G0 + G1 (t-t_c)) z_true``."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

from validation_metrics import sliding_window_ramp_metrics, summarize_ramp  # noqa: E402


def test_windowed_fit_recovers_known_complex_gain() -> None:
    dt, n = 1e-4, 600
    t = np.arange(n) * dt
    t = t - t.mean()
    theta = 2.0 * math.pi * 50.0 * t
    z_true = np.exp(1j * theta)
    g0 = 0.9 + 0.1j
    g1 = 0.2 - 0.3j
    z = (g0 + g1 * t) * z_true
    rows = sliding_window_ramp_metrics(
        z.real, z.imag, z_true.real, z_true.imag, np.zeros(n), np.zeros(n), t, window_s=n * dt
    )
    assert len(rows) == 1
    row = rows[0]
    assert row["gain"] == pytest.approx(abs(g0), rel=1e-9)
    assert row["phase_deg"] == pytest.approx(math.degrees(math.atan2(g0.imag, g0.real)), abs=1e-9)
    assert row["ripple_rms_v"] == pytest.approx(0.0, abs=1e-9)


def test_summary_reports_mean_and_worst_window() -> None:
    rows = [
        {
            "gain": 1.0,
            "phase_deg": 0.0,
            "ripple_rms_v": 1.0,
            "omega_prime_error_hz_mean": 0.0,
            "omega_prime_error_hz_std": 0.1,
            "t_center_s": 0.0,
        },
        {
            "gain": 1.1,
            "phase_deg": 2.0,
            "ripple_rms_v": 3.0,
            "omega_prime_error_hz_mean": 1.0,
            "omega_prime_error_hz_std": 0.2,
            "t_center_s": 0.06,
        },
    ]
    summary = summarize_ramp(rows)
    assert summary["n_windows"] == 2
    assert summary["mean"]["gain"] == pytest.approx(1.05)
    assert summary["worst"]["ripple_rms_v"] == 3.0
