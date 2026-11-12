#!/usr/bin/env python3
"""Phase 03b diagnosis: reference check and raw-SMO fundamental sweeps.

No plant/observer/filter code or defaults are changed. This script:

1. checks ``|E_true| = psi_f * omega_e`` (sign/frame), and the time alignment of
   ``E_true`` vs ``E_hat`` (one-sample offset test);
2. sweeps the raw SMO back-EMF fundamental gain/phase (both observers) over speed
   (100/200/500/1000 rpm at Ts=100 us) and over Ts (100..2.5 us at 1000 rpm);
3. runs a continuous-limit check (Ts down to 0.25 us) to see whether gain -> 1 and
   phase -> 0;
4. reproduces the *applied* vs *reported* injection of the sign observer to locate
   why its fundamental does not reach unity. **This is reported as a bug in the
   Phase 02 reporting convention; it is not fixed here.**

Outputs go to ``experiments/output/03b_diagnosis/``.
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
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
import config_diff  # noqa: E402
from validation_metrics import electrical_angle, vector_metrics  # noqa: E402

SPEEDS_RPM = (100.0, 200.0, 500.0, 1000.0)
TS_COARSE = (100e-6, 50e-6, 25e-6, 10e-6, 5e-6, 2.5e-6)
TS_TINY = (1e-6, 0.5e-6, 0.25e-6)


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _EXPERIMENTS / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _observers(pmsm_cfg, observer_cfg):
    return {
        "sign": ConventionalSMO.from_config(pmsm_cfg, observer_cfg),
        "sigmoid": SigmoidSMO.from_config(pmsm_cfg, observer_cfg),
    }


def raw_fundamental(plant, phase02, pmsm_cfg, observer_cfg, rpm, dt, duration, steady_fraction=0.5):
    """Return fundamental gain/phase (and |E_true|) for both observers at one point."""
    parameters = plant.parameters
    omega_e = parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
    t, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega_e, duration, dt)
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    start = int(steady_fraction * len(t))
    result = {"|E_true|_v": float(np.hypot(emf[start:, 0], emf[start:, 1]).mean())}
    for name, observer in _observers(pmsm_cfg, observer_cfg).items():
        sm = phase02.observer_rollout(observer, u, i, dt)
        result[name] = vector_metrics(
            sm["ea"][start:], sm["eb"][start:], emf[start:, 0], emf[start:, 1], theta[start:]
        )
    return result


def applied_vs_reported(plant, phase02, pmsm_cfg, observer_cfg, rpm, dt, duration=0.15):
    """Sign observer: fundamental of the applied ``g(S_pre)`` vs the reported ``g(S_post)``."""
    parameters = plant.parameters
    k = (
        config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-")
        * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V")
        * math.sqrt(2.0)
        / math.sqrt(3.0)
    )
    rs, ld = parameters.stator_resistance, parameters.ld
    omega_e = parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
    t, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega_e, duration, dt)
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    n = len(t)
    applied_a, applied_b = np.empty(n), np.empty(n)
    reported_a, reported_b = np.empty(n), np.empty(n)
    i_hat_a = i_hat_b = 0.0
    for idx in range(n):
        s_a, s_b = i_hat_a - i[idx, 0], i_hat_b - i[idx, 1]
        g_a = math.copysign(1.0, s_a) if s_a != 0.0 else 0.0
        g_b = math.copysign(1.0, s_b) if s_b != 0.0 else 0.0
        applied_a[idx], applied_b[idx] = k * g_a, k * g_b
        i_hat_a += dt * ((-rs * i_hat_a + u[idx, 0]) / ld - (k / ld) * g_a)
        i_hat_b += dt * ((-rs * i_hat_b + u[idx, 1]) / ld - (k / ld) * g_b)
        s_a2, s_b2 = i_hat_a - i[idx, 0], i_hat_b - i[idx, 1]
        reported_a[idx] = k * (math.copysign(1.0, s_a2) if s_a2 != 0.0 else 0.0)
        reported_b[idx] = k * (math.copysign(1.0, s_b2) if s_b2 != 0.0 else 0.0)
    start = n // 2
    return {
        "applied_sign_S_pre": vector_metrics(
            applied_a[start:], applied_b[start:], emf[start:, 0], emf[start:, 1], theta[start:]
        ),
        "reported_sign_S_post": vector_metrics(
            reported_a[start:], reported_b[start:], emf[start:, 0], emf[start:, 1], theta[start:]
        ),
    }


def alignment_check(plant, phase02, pmsm_cfg, observer_cfg, rpm=1000.0, dt=100e-6, duration=0.5):
    """One-sample offset test: E_true follows theta_e[k]; shifting by one sample costs w_e*Ts."""
    parameters = plant.parameters
    omega_e = parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
    t, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega_e, duration, dt)
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    shift = math.degrees(omega_e * dt)  # phase change for a one-sample offset
    base = vector_metrics(emf[:, 0], emf[:, 1], emf[:, 0], emf[:, 1], theta)
    plus = vector_metrics(emf[1:, 0], emf[1:, 1], emf[:-1, 0], emf[:-1, 1], theta[:-1])
    return {
        "true_self_phase_deg": base["fundamental_phase_deg"],
        "true_shifted_phase_deg": plus["fundamental_phase_deg"],
        "omega_e_times_Ts_deg": shift,
        "true_gain": base["fundamental_gain"],
    }


def make_plot(speed_sweep, ts_sweep, tiny, output_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    rpms = [row["rpm"] for row in speed_sweep]
    for name in ("sign", "sigmoid"):
        axes[0].plot(rpms, [row[name]["fundamental_gain"] for row in speed_sweep], "o-", label=name)
        axes[1].plot(
            rpms, [row[name]["fundamental_phase_deg"] for row in speed_sweep], "o-", label=name
        )
    axes[0].axhline(1.0, color="k", ls=":")
    axes[0].set(
        xlabel="speed (rpm)", ylabel="fundamental gain", title="Raw E_hat gain vs speed (Ts=100us)"
    )
    axes[1].axhline(0.0, color="k", ls=":")
    axes[1].set(
        xlabel="speed (rpm)", ylabel="phase (deg)", title="Raw E_hat phase vs speed (Ts=100us)"
    )
    for ax in axes:
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.suptitle("observer-only (no DSOGI-FLL); epsilon_filter_tau=OFF (n/a)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "speed_sweep.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    ts = [row["ts_us"] for row in ts_sweep]
    for name in ("sign", "sigmoid"):
        axes[0].semilogx(ts, [row[name]["fundamental_gain"] for row in ts_sweep], "o-", label=name)
        axes[1].semilogx(
            ts, [row[name]["fundamental_phase_deg"] for row in ts_sweep], "o-", label=name
        )
    axes[0].axhline(1.0, color="k", ls=":")
    axes[0].set(
        xlabel="Ts (us)", ylabel="fundamental gain", title="Raw E_hat gain vs Ts (1000 rpm)"
    )
    axes[1].axhline(0.0, color="k", ls=":")
    axes[1].set(xlabel="Ts (us)", ylabel="phase (deg)", title="Raw E_hat phase vs Ts (1000 rpm)")
    for ax in axes:
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.suptitle("observer-only (no DSOGI-FLL); epsilon_filter_tau=OFF (n/a)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "ts_sweep.png", dpi=130)
    plt.close(fig)


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)

    reference = {
        f"{int(rpm)}rpm": {
            "|E_true|_v": float(parameters.psi_f * parameters.pole_pairs * rpm * 2 * math.pi / 60),
            "psi_f_omega_e_v": float(
                parameters.psi_f * parameters.pole_pairs * rpm * 2 * math.pi / 60
            ),
        }
        for rpm in SPEEDS_RPM
    }
    # also verify the plant actually produces that magnitude
    for rpm in SPEEDS_RPM:
        omega_e = parameters.pole_pairs * rpm * 2 * math.pi / 60
        t, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega_e, 0.05, 100e-6)
        reference[f"{int(rpm)}rpm"]["|E_true|_measured_v"] = float(
            np.hypot(emf[:, 0], emf[:, 1]).mean()
        )

    speed_sweep = []
    for rpm in SPEEDS_RPM:
        row = {"rpm": rpm}
        row.update(raw_fundamental(plant, phase02, pmsm_cfg, observer_cfg, rpm, 100e-6, 0.2))
        speed_sweep.append(row)

    ts_sweep = []
    for dt in TS_COARSE:
        duration = 0.12 if dt >= 10e-6 else 0.06
        row = {"ts_us": dt * 1e6}
        row.update(raw_fundamental(plant, phase02, pmsm_cfg, observer_cfg, 1000.0, dt, duration))
        ts_sweep.append(row)

    tiny = []
    for dt in TS_TINY:
        row = {"ts_us": dt * 1e6}
        row.update(raw_fundamental(plant, phase02, pmsm_cfg, observer_cfg, 1000.0, dt, 0.04))
        tiny.append(row)

    diff = config_diff.summary(
        "03b_dsogi_fll_diagnosis",
        {},
        scenario={
            "speeds_rpm": SPEEDS_RPM,
            "ts_coarse_us": [x * 1e6 for x in TS_COARSE],
            "fll_used": False,
        },
    )
    diagnosis = {
        "note": "Phase 03b diagnosis; no code/defaults changed. Filter metrics only. epsilon_filter_tau=OFF (no DSOGI-FLL in this script).",
        "config_diff": diff,
        "reference_check": reference,
        "alignment_check": alignment_check(plant, phase02, pmsm_cfg, observer_cfg),
        "speed_sweep_100us": speed_sweep,
        "ts_sweep_1000rpm": ts_sweep,
        "continuous_limit_1000rpm": tiny,
        "sign_injection_pre_vs_post_100us": applied_vs_reported(
            plant, phase02, pmsm_cfg, observer_cfg, 1000.0, 100e-6
        ),
        "sign_injection_pre_vs_post_10us": applied_vs_reported(
            plant, phase02, pmsm_cfg, observer_cfg, 1000.0, 10e-6
        ),
    }

    output_dir = config.find_project_root() / "experiments" / "output" / "03b_diagnosis"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_plot(speed_sweep, ts_sweep, tiny, output_dir)
    (output_dir / "diagnosis.json").write_text(json.dumps(diagnosis, indent=2), encoding="utf-8")
    config_diff.emit(diff)

    for key in ("reference_check", "alignment_check"):
        print(f"== {key} ==")
        print(json.dumps(diagnosis[key], indent=2))
    for key in ("speed_sweep_100us", "ts_sweep_1000rpm", "continuous_limit_1000rpm"):
        print(f"== {key} ==")
        for row in diagnosis[key]:
            print("  " + json.dumps(row))
    for key in ("sign_injection_pre_vs_post_100us", "sign_injection_pre_vs_post_10us"):
        print(f"== {key} ==")
        print(json.dumps(diagnosis[key], indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
