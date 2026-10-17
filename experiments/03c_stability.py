#!/usr/bin/env python3
"""Phase 03c stability sweep for the sigmoid observer (post-D-025 fix).

At 1000 rpm, sweep ``Ts`` = 100..40 us and report the reported back-EMF fundamental
gain/phase/ripple and a limit-cycle flag (period-2 / Nyquist content of the sliding
surface S). Compare the observed boundary with the analytical

    Ts* = 2 Ld / (Rs + k a / 2)

which follows from the forward-Euler linear-region error dynamics
``S[k+1] = (1 - Ts (Rs + k a/2)/Ld) S[k]`` being stable iff
``Ts (Rs + k a/2)/Ld < 2``. Defaults and D-016 (forward Euler) are unchanged.

Outputs go to ``experiments/output/03c_stability/``.
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_EXPERIMENTS = Path(__file__).resolve().parent
sys.path.insert(0, str(_EXPERIMENTS))

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import per_unit_bases  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
import config_diff  # noqa: E402
from validation_metrics import electrical_angle, vector_metrics  # noqa: E402

TS_SWEEP = (100e-6, 80e-6, 65e-6, 60e-6, 55e-6, 50e-6, 40e-6)
RPM = 1000.0
A_PU = 4.0
LIMIT_CYCLE_THRESHOLD = 0.5


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _EXPERIMENTS / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def period2_content(signal: np.ndarray) -> float:
    """Demodulated Nyquist content: ``|mean(s*(-1)^k)| / std(s)`` (1 = pure period-2)."""
    signal = np.asarray(signal, dtype=float)
    alternating = signal * ((-1.0) ** np.arange(len(signal)))
    std = float(np.std(signal))
    return float(abs(np.mean(alternating)) / std) if std > 0.0 else 0.0


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)

    _v_base, i_base = per_unit_bases(pmsm_cfg)
    k = config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-") * _v_base
    a = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / i_base
    ts_star = 2.0 * parameters.ld / (parameters.stator_resistance + k * a / 2.0)
    a_max_100us = 2.0 * (2.0 * parameters.ld / 100e-6 - parameters.stator_resistance) / k

    omega_e = parameters.pole_pairs * RPM * 2.0 * math.pi / 60.0
    rows = []
    for dt in TS_SWEEP:
        duration = 0.15
        t, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega_e, duration, dt)
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        observer = SigmoidSMO.from_config(pmsm_cfg, observer_cfg)
        sm = phase02.observer_rollout(observer, u, i, dt)
        start = len(t) // 2
        metrics = vector_metrics(
            sm["ea"][start:], sm["eb"][start:], emf[start:, 0], emf[start:, 1], theta[start:]
        )
        p2 = period2_content(sm["sa"][start:])
        rows.append(
            {
                "ts_us": dt * 1e6,
                "ts_lambda_over_2": dt * (parameters.stator_resistance + k * a / 2.0) / parameters.ld,
                "fundamental_gain": metrics["fundamental_gain"],
                "fundamental_phase_deg": metrics["fundamental_phase_deg"],
                "ripple_rms_v": metrics["ripple_rms_v"],
                "period2_content": p2,
                "limit_cycle": p2 > LIMIT_CYCLE_THRESHOLD,
            }
        )

    output_dir = config.find_project_root() / "experiments" / "output" / "03c_stability"
    output_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    ts = [row["ts_us"] for row in rows]
    for ax, key, label, ref in (
        (axes[0], "fundamental_gain", "fundamental gain", 1.0),
        (axes[1], "period2_content", "period-2 content", None),
    ):
        ax.plot(ts, [row[key] for row in rows], "o-")
        if ref is not None:
            ax.axhline(ref, color="k", ls=":")
        ax.axvline(ts_star * 1e6, color="crimson", ls="--", label=f"Ts* = {ts_star*1e6:.1f} us")
        ax.set(xlabel="Ts (us)", ylabel=label, title=label + " vs Ts (1000 rpm, sigmoid)")
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.suptitle("observer-only (no DSOGI-FLL); epsilon_filter_tau=OFF (n/a)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "stability_sweep.png", dpi=130)
    plt.close(fig)

    diff = config_diff.summary(
        "03c_stability",
        {},
        scenario={"rpm": RPM, "ts_us": [x * 1e6 for x in TS_SWEEP], "fll_used": False},
    )
    diagnostics = {
        "note": "Phase 03c stability sweep (sigmoid, post-fix). Defaults/D-016 unchanged. epsilon_filter_tau=OFF (no DSOGI-FLL in this script).",
        "config_diff": diff,
        "k_v": k,
        "a_per_a": a,
        "i_base_a": i_base,
        "ts_star_us": ts_star * 1e6,
        "largest_a_stable_at_100us_per_a": a_max_100us,
        "implied_i_base_for_a_pu_4_a": A_PU / a_max_100us,
        "rows": rows,
    }
    (output_dir / "stability.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")
    config_diff.emit(diff)
    print(json.dumps(diagnostics, indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
