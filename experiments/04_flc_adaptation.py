#!/usr/bin/env python3
"""Phase 04: FLC adaptation of k and a (RAW E_hat only; no DSOGI-FLL).

Observers: fixed sign, fixed sigmoid, FLC-sigmoid. Ts in {100, 50} us. Windows:
steady = 8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s; ramp = the 03f
sliding-window fit over [0, 1.500] s (500 -> 1000 rpm).

Nominal FLC assumptions (ASSUMPTION — not from paper): Ke = 1/I_base = 0.13472 1/A,
Kec = Ts/I_base = 1.3472e-5 s/A, Ku = 1, CoG grid = 1001. Sensitivity grid:
Ke, Kec in {0.1, 1, 10} x nominal (9 cells) per Ts.

No position/speed claims; FLC conclusions are conditional on the assumed values.
Outputs: experiments/output/04_flc_adaptation/.
"""

from __future__ import annotations

import dataclasses
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

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import config_diff  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.fuzzy.engine import FuzzyEngine  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.fuzzy_smo import FuzzySMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import SMOObserverParameters  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from validation_metrics import (  # noqa: E402
    electrical_angle,
    sliding_window_ramp_metrics,
    summarize_ramp,
    vector_metrics,
)

RPM_START, RPM_END, RAMP_S, DURATION_S = 500.0, 1000.0, 1.5, 2.0
STEADY_CYCLES = 8
TS_VALUES = (100e-6, 50e-6)
GRID_MULTS = (0.1, 1.0, 10.0)


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rollout(observer, u, i, dt, *, record_gains=False):
    n = len(u)
    out = {key: np.empty(n) for key in ("ia", "ib", "sa", "sb", "ea", "eb")}
    if record_gains:
        out["kf"] = np.empty(n); out["af"] = np.empty(n)
        out["k"] = np.empty(n); out["a"] = np.empty(n)
    try:
        for k in range(n):
            result = observer.step(u[k, 0], u[k, 1], i[k, 0], i[k, 1], dt)
            out["ia"][k], out["ib"][k] = result.i_alpha_hat, result.i_beta_hat
            out["sa"][k], out["sb"][k] = result.s_alpha, result.s_beta
            out["ea"][k], out["eb"][k] = result.back_emf_alpha, result.back_emf_beta
            if record_gains:
                out["kf"][k] = 0.5 * (observer.last_kf[0] + observer.last_kf[1])
                out["af"][k] = 0.5 * (observer.last_af[0] + observer.last_af[1])
                out["k"][k] = 0.5 * (observer.last_k[0] + observer.last_k[1])
                out["a"][k] = 0.5 * (observer.last_a[0] + observer.last_a[1])
    except ValueError:
        pass
    return out


def _period2_content(signal) -> float:
    signal = np.asarray(signal, dtype=float)
    alternated = signal * ((-1.0) ** np.arange(len(signal)))
    std = float(np.std(signal))
    return float(abs(np.mean(alternated)) / std) if std > 0 else 0.0


def _steady_metrics(sm, emf, theta, mask, i, dt):
    vm = vector_metrics(sm["ea"][mask], sm["eb"][mask], emf[mask, 0], emf[mask, 1], theta[mask])
    error = np.hypot(sm["ia"][mask] - i[mask, 0], sm["ib"][mask] - i[mask, 1])
    return {
        "fundamental_gain": vm["fundamental_gain"],
        "fundamental_phase_deg": vm["fundamental_phase_deg"],
        "ripple_rms_v": vm["ripple_rms_v"],
        "current_rmse_a": float(np.sqrt(np.mean(error**2))),
        "mean_abs_surface_a": float(np.mean(np.hypot(sm["sa"][mask], sm["sb"][mask]))),
        "period2_flag": _period2_content(sm["sa"][mask]) > 0.5,
    }


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    experiments_cfg = config.load_config("experiments", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    base_engine = FuzzyEngine.from_yaml(directory)
    k0 = config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-") * (
        math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V") / math.sqrt(3.0)
    )
    a0 = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / (
        math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_power", unit="W")
        / (math.sqrt(3.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V"))
    )
    nominal_ke = base_engine.parameters.ke
    nominal_kec = base_engine.parameters.kec

    def rpm_at(t):
        return RPM_START + min(t / RAMP_S, 1.0) * (RPM_END - RPM_START)

    runs = {}
    plant_data = {}
    for dt in TS_VALUES:
        t = np.arange(int(round(DURATION_S / dt))) * dt
        _, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60, DURATION_S, dt
        )
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        f_true = parameters.pole_pairs * np.array([rpm_at(tt) for tt in t]) / 60.0
        omega_e = 2 * math.pi * f_true
        period = 1.0 / (parameters.pole_pairs * RPM_END / 60.0)
        steady_mask = (t >= DURATION_S - STEADY_CYCLES * period) & (t <= DURATION_S)
        ramp_mask = (t >= 0.0) & (t <= RAMP_S)
        bound = 2.0 * (2.0 * parameters.ld / dt - parameters.stator_resistance) / (k0 * a0)
        plant_data[dt] = (t, u, i, emf, theta, f_true, omega_e, steady_mask, ramp_mask, bound)

        def record(label, observer, *, gains=False):
            sm = _rollout(observer, u, i, dt, record_gains=gains)
            entry = {
                "ts_us": dt * 1e6,
                "observer": label,
                "steady": _steady_metrics(sm, emf, theta, steady_mask, i, dt),
                "ramp": summarize_ramp(
                    sliding_window_ramp_metrics(
                        sm["ea"][ramp_mask], sm["eb"][ramp_mask], emf[ramp_mask, 0], emf[ramp_mask, 1],
                        f_true[ramp_mask], f_true[ramp_mask], t[ramp_mask], window_s=0.06,
                    )
                ),
            }
            if gains:
                kf, af = sm["kf"][steady_mask], sm["af"][steady_mask]
                k, a = sm["k"][steady_mask], sm["a"][steady_mask]
                e_true = np.abs(omega_e[steady_mask] * parameters.psi_f)
                entry["gains"] = {
                    "kf_mean": float(np.mean(kf)), "kf_min": float(np.min(kf)), "kf_max": float(np.max(kf)),
                    "af_mean": float(np.mean(af)), "af_min": float(np.min(af)), "af_max": float(np.max(af)),
                    "kf_af_mean": float(np.mean(kf * af)),
                    "fraction_below_stability_bound": float(np.mean(kf * af < bound)),
                    "fraction_k_below_emf": float(np.mean(kf * k0 < e_true)),
                }
                entry["bound"] = bound
                stride = max(1, len(t) // 2000)
                steady_idx = np.where(steady_mask)[0]
                s_stride = max(1, len(steady_idx) // 500)
                entry["traces"] = {
                    "t": t[::stride].tolist(),
                    "kf": sm["kf"][::stride].tolist(),
                    "af": sm["af"][::stride].tolist(),
                    "ka": (sm["kf"][::stride] * sm["af"][::stride]).tolist(),
                    "t_steady": t[steady_idx][::s_stride].tolist(),
                    "ehat_alpha_steady": sm["ea"][steady_idx][::s_stride].tolist(),
                    "etrue_alpha_steady": emf[steady_idx, 0][::s_stride].tolist(),
                }
            runs[f"Ts{dt*1e6:.0f}_{label}"] = entry

        record("fixed_sign", ConventionalSMO.from_config(pmsm_cfg, observer_cfg))
        record("fixed_sigmoid", SigmoidSMO.from_config(pmsm_cfg, observer_cfg))
        for ke_mult in GRID_MULTS:
            for kec_mult in GRID_MULTS:
                params = dataclasses.replace(
                    base_engine.parameters, ke=nominal_ke * ke_mult, kec=nominal_kec * kec_mult
                )
                engine = FuzzyEngine(params)
                observer = FuzzySMO(
                    SMOObserverParameters.from_config(pmsm_cfg, observer_cfg), engine
                )
                label = f"flc_ke{ke_mult:g}_kec{kec_mult:g}"
                record(label, observer, gains=True)

    output_dir = config.find_project_root() / "experiments" / "output" / "04_flc_adaptation"
    output_dir.mkdir(parents=True, exist_ok=True)

    def plot_nominal(dt):
        entry = runs[f"Ts{dt*1e6:.0f}_flc_ke1_kec1"]
        traces = entry["traces"]
        fig, axes = plt.subplots(2, 2, figsize=(13, 7))
        axes[0, 0].plot(traces["t"], traces["kf"], label="kF")
        axes[0, 0].plot(traces["t"], traces["af"], label="aF")
        axes[0, 0].set(xlabel="time (s)", ylabel="normalised gain", title="FLC outputs (Ke=1x, Kec=1x)")
        axes[0, 0].legend(); axes[0, 0].grid(True, alpha=0.3)
        axes[0, 1].plot(traces["t"], traces["ka"], label="kF*aF")
        axes[0, 1].axhline(entry["bound"], color="crimson", ls="--", label=f"bound {entry['bound']:.3f}")
        axes[0, 1].set(xlabel="time (s)", ylabel="kF*aF", title="FLC product vs stability bound")
        axes[0, 1].legend(); axes[0, 1].grid(True, alpha=0.3)
        axes[1, 0].plot(traces["t_steady"], traces["etrue_alpha_steady"], "k", label="E_true alpha")
        axes[1, 0].plot(traces["t_steady"], traces["ehat_alpha_steady"], label="E_hat alpha (FLC)")
        axes[1, 0].set(xlabel="time (s)", ylabel="V", title="Steady-window E_hat vs E_true (alpha)")
        axes[1, 0].legend(fontsize=8); axes[1, 0].grid(True, alpha=0.3)
        axes[1, 1].text(
            0.02,
            0.5,
            f"fixed sign ripple {runs[f'Ts{dt*1e6:.0f}_fixed_sign']['steady']['ripple_rms_v']:.1f} V\n"
            f"fixed sigmoid ripple {runs[f'Ts{dt*1e6:.0f}_fixed_sigmoid']['steady']['ripple_rms_v']:.1f} V\n"
            f"FLC ripple {entry['steady']['ripple_rms_v']:.1f} V",
            fontsize=9,
        )
        axes[1, 1].axis("off")
        fig.suptitle(f"Phase 04 FLC adaptation, Ts={dt*1e6:.0f}us; ASSUMPTION values Ke=0.13472, Kec=1.3472e-5", fontsize=9)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        fig.savefig(output_dir / f"flc_traces_{dt*1e6:.0f}us.png", dpi=130)
        plt.close(fig)

    for dt in TS_VALUES:
        plot_nominal(dt)

    diff = config_diff.summary(
        "04_flc_adaptation",
        {"epsilon_filter_tau": None, "min_omega": None, "normalize_error": False, "positive_sequence": True},
        scenario={"ts_us": [x * 1e6 for x in TS_VALUES], "steady_cycles": STEADY_CYCLES,
                  "grid_multipliers": GRID_MULTS,
                  "assumptions": {"ke": nominal_ke, "kec": nominal_kec, "ku": base_engine.parameters.ku,
                                  "cog_grid_points": base_engine.parameters.cog_grid_points}},
    )
    diagnostics = {
        "note": "Phase 04 FLC adaptation; RAW E_hat only (no DSOGI-FLL). No position/speed claims. All FLC scaling/MF/operator/grid values are ASSUMPTIONS.",
        "assumptions": {"ke_1_per_a": nominal_ke, "kec_s_per_a": nominal_kec,
                        "ku": base_engine.parameters.ku, "cog_grid_points": base_engine.parameters.cog_grid_points},
        "config_diff": diff,
        "k0_v": k0, "a0_per_a": a0,
        "runs": runs,
    }
    (output_dir / "flc.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")
    config_diff.emit(diff)

    # nominal headline + grid summary
    for dt in TS_VALUES:
        base = runs[f"Ts{dt*1e6:.0f}_fixed_sigmoid"]["steady"]
        print(f"== Ts={dt*1e6:.0f}us (ASSUMPTION Ke={nominal_ke:.5f}, Kec={nominal_kec:.3e}) ==")
        for label in ("fixed_sign", "fixed_sigmoid", "flc_ke1_kec1"):
            s = runs[f"Ts{dt*1e6:.0f}_{label}"]["steady"]
            print(f"  {label:18s}: gain={s['fundamental_gain']:.3f} phase={s['fundamental_phase_deg']:+6.2f} "
                  f"ripple={s['ripple_rms_v']:7.2f} rmse={s['current_rmse_a']:.3f} |S|={s['mean_abs_surface_a']:.3f} "
                  f"p2={s['period2_flag']}")
        cells = []
        for ke_mult in GRID_MULTS:
            for kec_mult in GRID_MULTS:
                cell = runs[f"Ts{dt*1e6:.0f}_flc_ke{ke_mult:g}_kec{kec_mult:g}"]
                cells.append((f"ke{ke_mult:g}_kec{kec_mult:g}", cell))
        beats = [name for name, cell in cells
                 if cell["steady"]["ripple_rms_v"] < base["ripple_rms_v"]
                 and abs(cell["steady"]["fundamental_gain"] - 1.0) < abs(base["fundamental_gain"] - 1.0)]
        print(f"  cells beating fixed sigmoid on ripple AND gain error: {len(beats)}/9 {beats}")
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
