#!/usr/bin/env python3
"""Phase 02b verification of the sliding-mode observers (no new features).

This re-runs exactly the Phase 02 scenario (see ``02_smo_validation.py``) and adds:

1. Metrics restricted to a **steady-state window** ``t in [1.75, 2.0] s`` (the
   paper speed ramp ends at 1.5 s; the extra 0.25 s lets the current settle).
2. A **chattering metric**: the standard deviation of ``E_hat`` minus a moving
   average of ``E_hat`` over one electrical period, per axis and combined.
3. A **zoomed plot of S(t)** over a short steady-state interval.
4. A **sweep of the sigmoid slope `a`** (equivalently of the current base
   ``I_base +/- 50%``), reporting steady-state RMSE, |S| and chattering.
   Defaults are not changed anywhere.

Nothing in this script estimates rotor position or speed. Outputs go to
``experiments/output/02b_smo_verification/``.
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover - developer convenience
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import config_diff  # noqa: E402
from validation_metrics import electrical_angle, vector_metrics  # noqa: E402

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import SMOObserverParameters, per_unit_bases  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402

RPM_TO_RAD_S = 2.0 * math.pi / 60.0
STEADY_SETTLE_S = 0.25  # extra time after the ramp before the steady window opens


def _load_phase02_module():
    path = Path(__file__).with_name("02_smo_validation.py")
    spec = importlib.util.spec_from_file_location("phase02_smo_validation", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _moving_average(x: np.ndarray, window: int) -> np.ndarray:
    window = max(1, int(window))
    return np.convolve(x, np.ones(window) / window, mode="same")


def steady_window_metrics(result, i, t, t0, t1) -> dict:
    mask = (t >= t0) & (t <= t1)
    error = np.hypot(result["ia"] - i[:, 0], result["ib"] - i[:, 1])[mask]
    surface = np.hypot(result["sa"], result["sb"])[mask]
    return {
        "rmse_current_a": float(np.sqrt(np.mean(error**2))),
        "max_abs_current_error_a": float(np.max(error)),
        "mean_abs_sliding_surface_a": float(np.mean(surface)),
        "max_abs_sliding_surface_a": float(np.max(surface)),
    }


def back_emf_metrics(result, emf, theta, t, t0, t1) -> dict:
    """Fundamental gain/phase, ripple and total error over ``[t0, t1]`` (Phase 03b).

    Replaces the one-period-moving-average "chatter" metric; see
    ``experiments/validation_metrics.py`` and ``docs/phase03.md`` for definitions.
    """
    mask = (t >= t0) & (t <= t1)
    return vector_metrics(
        result["ea"][mask], result["eb"][mask], emf[mask, 0], emf[mask, 1], theta[mask]
    )


def make_zoom_plot(t, conv, sig, t_zoom, output_dir: Path) -> None:
    mask = (t >= t_zoom[0]) & (t <= t_zoom[1])
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    axes[0].plot(t[mask], conv["sa"][mask], label="conventional (sign)", alpha=0.85)
    axes[0].plot(t[mask], sig["sa"][mask], label="sigmoid", alpha=0.85)
    axes[0].set(
        xlabel="time (s)",
        ylabel="S_alpha (A)",
        title=f"Sliding surface, steady window {t_zoom[0]:.3f}-{t_zoom[1]:.3f} s",
    )
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    fine = (t >= t_zoom[1] - 0.01) & (t <= t_zoom[1])
    axes[1].plot(t[fine] * 1e3, conv["sa"][fine], "o-", markersize=3, label="conventional (sign)")
    axes[1].plot(t[fine] * 1e3, sig["sa"][fine], "s-", markersize=3, label="sigmoid")
    axes[1].set(xlabel="time (ms)", ylabel="S_alpha (A)", title="Zoom: final 10 ms")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("observer-only (no DSOGI-FLL); epsilon_filter_tau=OFF (n/a)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "sliding_surface_zoom.png", dpi=130)
    plt.close(fig)


def make_sweep_plot(rows, base_a, output_dir: Path) -> None:
    a = [row["a_si_per_a"] for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    axes[0].semilogx(a, [row["rmse_current_a"] for row in rows], "o-", label="RMSE |i_hat-i|")
    axes[0].axvline(base_a, color="k", ls=":", label="default a")
    axes[0].set(xlabel="sigmoid slope a (1/A)", ylabel="steady RMSE (A)", title="RMSE vs a")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].semilogx(
        a, [row["ripple_rms_v"] for row in rows], "s-", color="crimson", label="ripple rms"
    )
    axes[1].axvline(base_a, color="k", ls=":", label="default a")
    axes[1].set(xlabel="sigmoid slope a (1/A)", ylabel="ripple rms (V)", title="Ripple vs a")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    fig.suptitle("observer-only (no DSOGI-FLL); epsilon_filter_tau=OFF (n/a)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "sigmoid_slope_sweep.png", dpi=130)
    plt.close(fig)


def main() -> int:
    phase02 = _load_phase02_module()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    experiments_cfg = config.load_config("experiments", directory=directory)

    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    dt = config.resolve_parameter(experiments_cfg, "sim.timestep_ts", unit="s")
    v_base, i_base = per_unit_bases(pmsm_cfg)

    profile = experiments_cfg["specs"]["speed_profile"][0]
    from_rpm, to_rpm = float(profile["from_rpm"]), float(profile["to_rpm"])
    ramp_time, duration = float(profile["time_s"]), float(profile["steady_state_s"])

    def omega_e_fn(t: float) -> float:
        ramp = min(t / ramp_time, 1.0)
        return parameters.pole_pairs * (from_rpm + ramp * (to_rpm - from_rpm)) * RPM_TO_RAD_S

    t, u, i, emf = phase02.plant_rollout(plant, omega_e_fn, duration, dt)
    conventional = ConventionalSMO.from_config(pmsm_cfg, observer_cfg)
    sigmoid = SigmoidSMO.from_config(pmsm_cfg, observer_cfg)
    conv = phase02.observer_rollout(conventional, u, i, dt)
    sig = phase02.observer_rollout(sigmoid, u, i, dt)

    t0, t1 = ramp_time + STEADY_SETTLE_S, duration
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    ramp_window = (0.0, ramp_time)

    conv_steady = steady_window_metrics(conv, i, t, t0, t1)
    sig_steady = steady_window_metrics(sig, i, t, t0, t1)
    conv_fund = back_emf_metrics(conv, emf, theta, t, t0, t1)
    sig_fund = back_emf_metrics(sig, emf, theta, t, t0, t1)
    conv_ramp = back_emf_metrics(conv, emf, theta, t, *ramp_window)
    sig_ramp = back_emf_metrics(sig, emf, theta, t, *ramp_window)

    # --- sigmoid slope sweep (defaults unchanged; k is independent of I_base) --
    base_a = sigmoid.parameters.sigmoid_slope_a
    assert base_a is not None
    a_values = sorted(
        {
            0.05,
            0.1,
            0.2,
            base_a / 1.5,
            base_a / 1.25,
            base_a,
            base_a / 0.75,
            base_a / 0.5,
            2.0,
            4.0,
        }
    )
    sweep = []
    for a in a_values:
        params = SMOObserverParameters(
            stator_resistance=sigmoid.parameters.stator_resistance,
            ld=sigmoid.parameters.ld,
            switching_gain_k=sigmoid.parameters.switching_gain_k,
            sigmoid_slope_a=a,
        )
        result = phase02.observer_rollout(SigmoidSMO(params), u, i, dt)
        steady = steady_window_metrics(result, i, t, t0, t1)
        fund = back_emf_metrics(result, emf, theta, t, t0, t1)
        sweep.append(
            {
                "a_si_per_a": a,
                "a_pu_equivalent": a * i_base,
                "i_base_multiplier_equivalent": base_a / a,
                "rmse_current_a": steady["rmse_current_a"],
                "mean_abs_sliding_surface_a": steady["mean_abs_sliding_surface_a"],
                "max_abs_sliding_surface_a": steady["max_abs_sliding_surface_a"],
                "fundamental_gain": fund["fundamental_gain"],
                "fundamental_phase_deg": fund["fundamental_phase_deg"],
                "ripple_rms_v": fund["ripple_rms_v"],
            }
        )

    output_dir = config.find_project_root() / "experiments" / "output" / "02b_smo_verification"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_zoom_plot(t, conv, sig, (t0, t0 + 0.05), output_dir)
    make_sweep_plot(sweep, base_a, output_dir)

    diff = config_diff.summary(
        "02b_smo_verification",
        {"per_unit_scaling": True},
        scenario={"ts_s": dt, "a_si_per_a": base_a, "fll_used": False},
    )
    diagnostics = {
        "note": (
            "Phase 03b metric: fundamental gain/phase, ripple (std about the fitted "
            "fundamental) and total error, replacing the one-period-MA chatter. "
            "Defaults unchanged. No position/speed estimation. epsilon_filter_tau=OFF "
            "(no DSOGI-FLL in this script)."
        ),
        "config_diff": diff,
        "metric_definitions": "see experiments/validation_metrics.py and docs/phase03.md",
        "steady_window": {"t0_s": t0, "t1_s": t1, "note": "ramp ends at 1.5 s; +0.25 s settle"},
        "ramp_window": {"t0_s": ramp_window[0], "t1_s": ramp_window[1]},
        "conventional_sign": {
            "steady": conv_steady,
            "fundamental_steady": conv_fund,
            "fundamental_ramp": conv_ramp,
        },
        "sigmoid": {
            "steady": sig_steady,
            "fundamental_steady": sig_fund,
            "fundamental_ramp": sig_ramp,
        },
        "steady_rmse_reduction_percent": 100.0
        * (conv_steady["rmse_current_a"] - sig_steady["rmse_current_a"])
        / conv_steady["rmse_current_a"],
        "steady_ripple_reduction_percent": 100.0
        * (conv_fund["ripple_rms_v"] - sig_fund["ripple_rms_v"])
        / conv_fund["ripple_rms_v"],
        "sigmoid_slope_sweep": sweep,
        "defaults": {
            "a_si_per_a": base_a,
            "i_base_a": i_base,
            "k_v": sigmoid.parameters.switching_gain_k,
        },
    }
    (output_dir / "verification.json").write_text(
        json.dumps(diagnostics, indent=2), encoding="utf-8"
    )
    config_diff.emit(diff)
    print(json.dumps(diagnostics, indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
