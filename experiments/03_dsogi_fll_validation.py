#!/usr/bin/env python3
"""Phase 03 validation: DSOGI-FLL filtering the SMO back-EMF estimate.

Identical SMO outputs from *both* Phase 02 observers (same short-circuit,
500 -> 1000 r/min scenario) are fed into the DSOGI-FLL. Metrics are filter
metrics only -- no rotor position or speed is estimated here (Phase 05).

Reported per observer:
- frequency error vs the true electrical frequency (steady window and over the ramp),
- amplitude and phase error of the filtered back-EMF vs the plant's true back-EMF
  (the true back-EMF is used only in this evaluation layer),
- residual chatter, filtered vs the raw SMO estimate.

Repeated at Ts = 100 us (the paper's value), 50 us and 25 us, defaults unchanged.

The paper *does* state a sample/step time: Ts = 100 us from Eq.(20), p.5, and the
10 kHz sampling/current-loop frequency in Table III, p.6.

Outputs go to ``experiments/output/03_dsogi_fll_validation/``.
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
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402

RPM_TO_RAD_S = 2.0 * math.pi / 60.0
STEADY_SETTLE_S = 0.25
TS_VALUES = (100e-6, 50e-6, 25e-6)


# Options the paper does not state. Enabled here (explicitly, not by default) because
# the SMO back-EMF estimate is a near-bang-bang switching signal whose ripple would
# otherwise drive the un-normalized FLL unstable. See docs/implementation_decisions.md.
FLL_EPSILON_FILTER_TAU = 0.01  # s
FLL_MIN_FREQUENCY_HZ = 2.0


def build_fll_parameters(observer_config, initial_omega) -> DSOGIFLLParameters:
    """k_s and Gamma straight from config; documented options for the rest."""
    return DSOGIFLLParameters(
        config.resolve_parameter(observer_config, "dsogi.damping_gain_k", unit="-"),
        config.resolve_parameter(observer_config, "fll.gamma_gain", unit="-"),
        initial_omega,
        min_omega=2.0 * math.pi * FLL_MIN_FREQUENCY_HZ,
        epsilon_filter_tau=FLL_EPSILON_FILTER_TAU,
    )


def _load_phase02():
    path = Path(__file__).with_name("02_smo_validation.py")
    spec = importlib.util.spec_from_file_location("phase02_smo_validation", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def fll_rollout(fll: DSOGIFLL, e_alpha: np.ndarray, e_beta: np.ndarray, dt: float) -> dict:
    steps = len(e_alpha)
    out = {k: np.empty(steps) for k in ("va", "vb", "freq", "eps")}
    for k in range(steps):
        result = fll.step(float(e_alpha[k]), float(e_beta[k]), dt)
        out["va"][k], out["vb"][k] = result.v_alpha, result.v_beta
        out["freq"][k] = result.frequency_hz
        out["eps"][k] = result.epsilon_f
    return out


def _fundamental_block(
    result, e_true_alpha, e_true_beta, e_raw_alpha, e_raw_beta, theta, mask
) -> dict:
    """Raw and filtered fundamental gain/phase, ripple and total error over ``mask``.

    Definitions in ``experiments/validation_metrics.py`` / ``docs/phase03.md``.
    ``amplitude_error`` is ``|fundamental_gain - 1|`` (relative fundamental
    amplitude error); ``phase_error_deg`` is ``fundamental_phase_deg``.
    """
    raw = vector_metrics(
        e_raw_alpha[mask], e_raw_beta[mask], e_true_alpha[mask], e_true_beta[mask], theta[mask]
    )
    filt = vector_metrics(
        result["va"][mask], result["vb"][mask], e_true_alpha[mask], e_true_beta[mask], theta[mask]
    )
    return {
        "raw_fundamental_gain": raw["fundamental_gain"],
        "raw_fundamental_phase_deg": raw["fundamental_phase_deg"],
        "raw_amplitude_error": abs(raw["fundamental_gain"] - 1.0),
        "raw_ripple_rms_v": raw["ripple_rms_v"],
        "raw_total_error_rms_v": raw["total_error_rms_v"],
        "filtered_fundamental_gain": filt["fundamental_gain"],
        "filtered_fundamental_phase_deg": filt["fundamental_phase_deg"],
        "filtered_amplitude_error": abs(filt["fundamental_gain"] - 1.0),
        "filtered_ripple_rms_v": filt["ripple_rms_v"],
        "filtered_total_error_rms_v": filt["total_error_rms_v"],
        "ripple_reduction_percent": 100.0
        * (raw["ripple_rms_v"] - filt["ripple_rms_v"])
        / raw["ripple_rms_v"],
    }


def filter_metrics(
    result,
    e_true_alpha,
    e_true_beta,
    e_raw_alpha,
    e_raw_beta,
    f_true,
    theta,
    steady_mask,
    ramp_mask,
) -> dict:
    """Frequency error and fundamental metrics over the steady and ramp windows."""
    steady_freq = result["freq"][steady_mask] - f_true[steady_mask]
    ramp_freq = result["freq"][ramp_mask] - f_true[ramp_mask]
    return {
        "steady": {
            "frequency_error_mean_hz": float(np.mean(steady_freq)),
            "frequency_error_std_hz": float(np.std(steady_freq)),
            "frequency_error_max_abs_hz": float(np.max(np.abs(steady_freq))),
            **_fundamental_block(
                result, e_true_alpha, e_true_beta, e_raw_alpha, e_raw_beta, theta, steady_mask
            ),
        },
        "ramp": {
            "frequency_error_mean_hz": float(np.mean(ramp_freq)),
            "frequency_error_std_hz": float(np.std(ramp_freq)),
            "frequency_error_max_abs_hz": float(np.max(np.abs(ramp_freq))),
            **_fundamental_block(
                result, e_true_alpha, e_true_beta, e_raw_alpha, e_raw_beta, theta, ramp_mask
            ),
        },
    }


def make_plots(t, f_true, runs, output_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    axes[0].plot(t, f_true, "k", label="true electrical frequency")
    for label, run in runs.items():
        if run["dt"] == TS_VALUES[0]:
            axes[0].plot(t, run["conv"]["freq"], label=f"conv -> DSOGI-FLL ({label})", alpha=0.8)
            axes[0].plot(t, run["sig"]["freq"], label="sigmoid -> DSOGI-FLL", alpha=0.8)
    axes[0].set(
        xlabel="time (s)", ylabel="frequency (Hz)", title="FLL frequency tracking (Ts=100us)"
    )
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    run = next(r for r in runs.values() if r["dt"] == TS_VALUES[0])
    labels = ["conv raw", "conv filtered", "sig raw", "sig filtered"]
    values = [
        run["conv"]["metrics"]["steady"]["raw_ripple_rms_v"],
        run["conv"]["metrics"]["steady"]["filtered_ripple_rms_v"],
        run["sig"]["metrics"]["steady"]["raw_ripple_rms_v"],
        run["sig"]["metrics"]["steady"]["filtered_ripple_rms_v"],
    ]
    axes[1].bar(labels, values)
    axes[1].set(ylabel="ripple RMS (V)", title="Ripple: raw SMO vs DSOGI-FLL filtered")
    axes[1].tick_params(axis="x", rotation=20)
    axes[1].grid(True, alpha=0.3, axis="y")
    fig.suptitle(
        "epsilon_filter_tau=0.01 s (NON-DEFAULT, D-028) - RETRACTED; see 03e re-base",
        fontsize=10,
        color="crimson",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "frequency_and_chatter.png", dpi=130)
    plt.close(fig)


def main() -> int:
    phase02 = _load_phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    experiments_cfg = config.load_config("experiments", directory=directory)

    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    profile = experiments_cfg["specs"]["speed_profile"][0]
    from_rpm, to_rpm = float(profile["from_rpm"]), float(profile["to_rpm"])
    ramp_time, duration = float(profile["time_s"]), float(profile["steady_state_s"])

    def rpm_at(t: float) -> float:
        return from_rpm + min(t / ramp_time, 1.0) * (to_rpm - from_rpm)

    results = {}
    for dt in TS_VALUES:
        t = np.arange(int(round(duration / dt))) * dt
        omega_e_fn = lambda tt: parameters.pole_pairs * rpm_at(tt) * RPM_TO_RAD_S  # noqa: E731
        _, u, i, emf = phase02.plant_rollout(plant, omega_e_fn, duration, dt)
        f_true = parameters.pole_pairs * np.array([rpm_at(tt) for tt in t]) / 60.0
        t0, t1 = ramp_time + STEADY_SETTLE_S, duration
        steady_mask = (t >= t0) & (t <= t1)
        ramp_mask = (t >= 0.0) & (t <= ramp_time)
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        init_omega = 2.0 * math.pi * parameters.pole_pairs * from_rpm / 60.0

        entry = {"dt": dt, "t": t, "f_true": f_true}
        for label, observer in (
            ("conv", ConventionalSMO.from_config(pmsm_cfg, observer_cfg)),
            ("sig", SigmoidSMO.from_config(pmsm_cfg, observer_cfg)),
        ):
            sm = phase02.observer_rollout(observer, u, i, dt)
            fll = DSOGIFLL(build_fll_parameters(observer_cfg, init_omega))
            filt = fll_rollout(fll, sm["ea"], sm["eb"], dt)
            metrics = filter_metrics(
                filt,
                emf[:, 0],
                emf[:, 1],
                sm["ea"],
                sm["eb"],
                f_true,
                theta,
                steady_mask,
                ramp_mask,
            )
            entry[label] = {"freq": filt["freq"], "va": filt["va"], "metrics": metrics}
        results[f"{int(round(dt * 1e6))}us"] = entry

    output_dir = config.find_project_root() / "experiments" / "output" / "03_dsogi_fll_validation"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_plots(results["100us"]["t"], results["100us"]["f_true"], results, output_dir)

    diff = config_diff.summary(
        "03_dsogi_fll_validation",
        {
            "epsilon_filter_tau": FLL_EPSILON_FILTER_TAU,
            "min_omega": 2.0 * math.pi * FLL_MIN_FREQUENCY_HZ,
        },
        scenario={"ts_us": [x * 1e6 for x in TS_VALUES]},
    )
    diagnostics = {
        "note": (
            "RETRACTED (D-028): these filtered numbers used epsilon_filter_tau=10 ms, a "
            "non-paper option. See Phase 03e (03e_filter_rebase) for the library-default "
            "re-base. No rotor position or speed estimation (Phase 05)."
        ),
        "config_diff": diff,
        "retracted": True,
        "retraction_reason": "epsilon_filter_tau=10 ms is a non-paper option (D-028); see docs/phase03.md Step 2",
        "paper_sample_time": "Ts=100us (Eq.(20), p.5); 10 kHz (Table III, p.6)",
        "parameters": {
            "damping_gain_k": config.resolve_parameter(
                observer_cfg, "dsogi.damping_gain_k", unit="-"
            ),
            "fll_gain_gamma": config.resolve_parameter(observer_cfg, "fll.gamma_gain", unit="-"),
            "steady_window_s": [float(ramp_time + STEADY_SETTLE_S), float(duration)],
            "defaults_unchanged": True,
            "documented_options": {
                "epsilon_filter_tau_s": FLL_EPSILON_FILTER_TAU,
                "min_frequency_hz": FLL_MIN_FREQUENCY_HZ,
                "note": "not stated by the paper; required for the near-bang-bang SMO input",
            },
            "positive_sequence": True,
            "normalize_error": False,
        },
        "results": {
            key: {obs: entry[obs]["metrics"] for obs in ("conv", "sig")}
            for key, entry in results.items()
        },
    }
    (output_dir / "diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2), encoding="utf-8"
    )
    np.savez(
        output_dir / "states.npz",
        t=results["100us"]["t"],
        f_true=results["100us"]["f_true"],
        conv_freq=results["100us"]["conv"]["freq"],
        sig_freq=results["100us"]["sig"]["freq"],
        conv_filtered_alpha=results["100us"]["conv"]["va"],
        sig_filtered_alpha=results["100us"]["sig"]["va"],
    )
    config_diff.emit(diff)
    print(json.dumps(diagnostics, indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
