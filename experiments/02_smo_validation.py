#!/usr/bin/env python3
"""Phase 02 validation: conventional (sign) vs sigmoid sliding-mode observers.

Both observers are driven with *identical* measured signals ``(u_alpha, u_beta,
i_alpha, i_beta)`` and identical initial estimates, produced by the Phase 01
plant under the paper's no-load acceleration profile (500 -> 1000 r/min).
Terminals are short-circuited (``u = 0``) so the plant draws the back-EMF-driven
current and the observer must reconstruct the back-EMF from the switching signal
alone -- it never sees the plant's true angle, speed or back-EMF.

This script estimates *currents* and the *back-EMF switching signal* only. No
rotor position or speed is estimated (that is a later phase). The reported timing
is a software benchmark, not evidence of hardware real-time feasibility.

Scaling note: the paper's ``k=1`` and ``a=4`` (p.5) are per-unit gains with the
back-EMF referenced to ``311/sqrt(3)`` V. They are converted to SI in
``SMOObserverParameters.from_config`` using peak bases derived from the machine
ratings; see ``docs/implementation_decisions.md`` (D-014, D-015).

Outputs are written to ``experiments/output/02_smo_validation/``.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

try:  # allow running the script without an editable install
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover - developer convenience
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import per_unit_bases  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState  # noqa: E402

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def plant_rollout(plant: PMSMPlant, omega_e_fn, duration: float, dt: float):
    """Run the plant with a prescribed speed profile and short-circuited terminals."""
    steps = int(round(duration / dt))
    t = np.arange(steps) * dt
    u = np.zeros((steps, 2))
    i = np.zeros((steps, 2))
    emf = np.zeros((steps, 2))
    state = PMSMState(0.0, 0.0, omega_e_fn(0.0), 0.0)
    for n in range(steps):
        state.omega_e = omega_e_fn(t[n])
        emf[n] = plant.back_emf(state.theta_e, state.omega_e)
        i[n] = (state.i_alpha, state.i_beta)
        state = plant.step(state, 0.0, 0.0, dt)
    return t, u, i, emf


def observer_rollout(observer, u: np.ndarray, i: np.ndarray, dt: float):
    """Feed identical signals to an observer; record estimates and diagnostics."""
    steps = len(u)
    out = {key: np.empty(steps) for key in ("ia", "ib", "sa", "sb", "ea", "eb")}
    start = time.perf_counter()
    for n in range(steps):
        result = observer.step(u[n, 0], u[n, 1], i[n, 0], i[n, 1], dt)
        out["ia"][n], out["ib"][n] = result.i_alpha_hat, result.i_beta_hat
        out["sa"][n], out["sb"][n] = result.s_alpha, result.s_beta
        out["ea"][n], out["eb"][n] = result.back_emf_alpha, result.back_emf_beta
    out["runtime_s"] = time.perf_counter() - start
    return out


def metrics(result, i: np.ndarray, dt: float, skip_fraction: float = 0.5) -> dict:
    """Current-error and sliding-surface statistics over the steady window."""
    skip = int(round(skip_fraction * len(i)))
    error = np.hypot(result["ia"] - i[:, 0], result["ib"] - i[:, 1])[skip:]
    surface = np.hypot(result["sa"], result["sb"])[skip:]
    return {
        "rmse_current_a": float(np.sqrt(np.mean(error**2))),
        "max_abs_current_error_a": float(np.max(error)),
        "mean_sliding_surface_a": float(np.mean(surface)),
        "mean_back_emf_amplitude_v": float(np.mean(np.hypot(result["ea"], result["eb"])[skip:])),
        "runtime_per_update_us": 1e6 * result["runtime_s"] / len(i),
    }


def make_plots(t, i, emf, conv, sig, v_base, output_dir: Path) -> None:
    def envelope(x, w=200):
        kernel = np.ones(w) / w
        return np.convolve(x, kernel, mode="same")

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    axes[0].plot(
        t, np.hypot(conv["ia"] - i[:, 0], conv["ib"] - i[:, 1]), label="conventional (sign)"
    )
    axes[0].plot(t, np.hypot(sig["ia"] - i[:, 0], sig["ib"] - i[:, 1]), label="sigmoid")
    axes[0].set(xlabel="time (s)", ylabel="|i_hat - i| (A)", title="Current estimation error")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, conv["sa"], label="conventional (sign)", alpha=0.8)
    axes[1].plot(t, sig["sa"], label="sigmoid", alpha=0.8)
    axes[1].set(xlabel="time (s)", ylabel="S_alpha (A)", title="Sliding-surface error (alpha)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "current_error_and_sliding_surface.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    axes[0].plot(t, emf[:, 0], "k", label="true E_alpha")
    axes[0].plot(t, conv["ea"], label="conventional (sign)", alpha=0.6)
    axes[0].plot(t, sig["ea"], label="sigmoid", alpha=0.9)
    axes[0].set(xlabel="time (s)", ylabel="E_alpha (V)", title="Back-EMF estimate vs true (raw)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, emf[:, 1], "k", label="true E_beta")
    axes[1].plot(t, envelope(conv["ea"]), label="conventional (moving avg)", alpha=0.8)
    axes[1].plot(t, envelope(sig["ea"]), label="sigmoid (moving avg)", alpha=0.9)
    axes[1].set(xlabel="time (s)", ylabel="E_alpha (V)", title="Back-EMF estimate (smoothed)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "back_emf_estimate.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 1, figsize=(5.2, 5.2))
    axes.plot(emf[:, 0] / v_base, emf[:, 1] / v_base, "k", label="true (p.u.)")
    axes.plot(conv["ea"] / v_base, conv["eb"] / v_base, label="conventional", alpha=0.4)
    axes.plot(sig["ea"] / v_base, sig["eb"] / v_base, label="sigmoid", alpha=0.8)
    axes.set(xlabel="E_alpha (p.u.)", ylabel="E_beta (p.u.)", title="Back-EMF locus")
    axes.legend(fontsize=8)
    axes.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "back_emf_locus.png", dpi=130)
    plt.close(fig)


def main() -> int:
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
        rpm = from_rpm + ramp * (to_rpm - from_rpm)
        return parameters.pole_pairs * rpm * RPM_TO_RAD_S

    t, u, i, emf = plant_rollout(plant, omega_e_fn, duration, dt)

    conventional = ConventionalSMO.from_config(pmsm_cfg, observer_cfg)
    sigmoid = SigmoidSMO.from_config(pmsm_cfg, observer_cfg)
    conv = observer_rollout(conventional, u, i, dt)
    sig = observer_rollout(sigmoid, u, i, dt)

    conv_metrics = metrics(conv, i, dt)
    sig_metrics = metrics(sig, i, dt)

    output_dir = config.find_project_root() / "experiments" / "output" / "02_smo_validation"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_plots(t, i, emf, conv, sig, v_base, output_dir)

    np.savez(
        output_dir / "states.npz",
        t=t,
        u_alpha=u[:, 0],
        u_beta=u[:, 1],
        i_alpha=i[:, 0],
        i_beta=i[:, 1],
        e_alpha=emf[:, 0],
        e_beta=emf[:, 1],
        conv_i_alpha=conv["ia"],
        conv_i_beta=conv["ib"],
        conv_s_alpha=conv["sa"],
        conv_s_beta=conv["sb"],
        conv_e_alpha=conv["ea"],
        conv_e_beta=conv["eb"],
        sig_i_alpha=sig["ia"],
        sig_i_beta=sig["ib"],
        sig_s_alpha=sig["sa"],
        sig_s_beta=sig["sb"],
        sig_e_alpha=sig["ea"],
        sig_e_beta=sig["eb"],
    )

    diagnostics = {
        "note": (
            "Phase 02 observer validation on identical signals; currents and the "
            "back-EMF switching signal only. No rotor position/speed estimation. "
            "Timing is a software benchmark, not hardware real-time evidence."
        ),
        "parameters": {
            "stator_resistance_ohm": conventional.parameters.stator_resistance,
            "ld_h": conventional.parameters.ld,
            "switching_gain_k_v": conventional.parameters.switching_gain_k,
            "sigmoid_slope_a_per_a": sigmoid.parameters.sigmoid_slope_a,
            "per_unit_voltage_base_v": v_base,
            "per_unit_current_base_a": i_base,
        },
        "scenario": {
            "description": "short-circuit (u=0), no-load speed ramp from paper profile",
            "from_rpm": from_rpm,
            "to_rpm": to_rpm,
            "ramp_time_s": ramp_time,
            "duration_s": duration,
            "timestep_s": dt,
            "steps": int(len(t)),
        },
        "conventional_sign": conv_metrics,
        "sigmoid": sig_metrics,
        "sigmoid_rmse_reduction_percent": 100.0
        * (conv_metrics["rmse_current_a"] - sig_metrics["rmse_current_a"])
        / conv_metrics["rmse_current_a"],
    }
    (output_dir / "diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2), encoding="utf-8"
    )
    print(json.dumps(diagnostics, indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
