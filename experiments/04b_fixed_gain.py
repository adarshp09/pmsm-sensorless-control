#!/usr/bin/env python3
"""Phase 04b: matched fixed-gain control, unified S metrics, transient metric and
FLC diagnostics (experiments/docs only; no src/configs changes).

RAW E_hat only; no DSOGI-FLL; no position/speed claims. All FLC scaling / MF /
operator / grid values are ASSUMPTIONS: Ke=0.13472 1/A, Kec=1.3472e-5 s/A, Ku=1,
CoG grid=1001, Ruspini MF feet, AND=min, aggregation=max, per-axis FLC,
backward-difference s_dot.

Steady window: 8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s.
Ramp: 03f sliding-window (60 ms) fit over [0, 1.500] s.
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
CYCLES = 8
TS_VALUES = (100e-6, 50e-6)
KF_GRID = (0.35, 0.5, 0.75, 1.0)
AF_GRID = (0.1, 0.2, 0.4, 0.6, 1.0)
FLC_MEAN = (0.355, 0.591)
GRID_MULTS = (0.1, 1.0, 10.0)
KE = 0.13472
KEC = 1.3472e-5
E_MAX = 55.5


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rollout(observer, u, i, dt, record_gains=False):
    n = len(u)
    out = {key: np.empty(n) for key in ("ia", "ib", "sa", "sb", "ea", "eb")}
    if record_gains:
        out["kf"] = np.empty(n); out["af"] = np.empty(n)
    for k in range(n):
        result = observer.step(u[k, 0], u[k, 1], i[k, 0], i[k, 1], dt)
        out["ia"][k], out["ib"][k] = result.i_alpha_hat, result.i_beta_hat
        out["sa"][k], out["sb"][k] = result.s_alpha, result.s_beta
        out["ea"][k], out["eb"][k] = result.back_emf_alpha, result.back_emf_beta
        if record_gains:
            out["kf"][k] = 0.5 * (observer.last_kf[0] + observer.last_kf[1])
            out["af"][k] = 0.5 * (observer.last_af[0] + observer.last_af[1])
    return out


def _surface_metrics(sa, sb, mask):
    a, b = sa[mask], sb[mask]
    magnitude = np.hypot(a, b)
    return {
        "rms_s_alpha_a": float(np.sqrt(np.mean(a**2))),
        "rms_s_beta_a": float(np.sqrt(np.mean(b**2))),
        "mean_abs_s_a": float(np.mean(magnitude)),
        "rms_abs_s_a": float(np.sqrt(np.mean(magnitude**2))),
    }


def _period2(signal) -> float:
    signal = np.asarray(signal, dtype=float)
    alternating = signal * ((-1.0) ** np.arange(len(signal)))
    std = float(np.std(signal))
    return float(abs(np.mean(alternating)) / std) if std > 0 else 0.0


def _transient(sa, sb, t, steady_mask):
    magnitude = np.hypot(sa, sb)
    threshold = 1.5 * float(np.sqrt(np.mean(magnitude[steady_mask] ** 2)))
    bad = np.where(magnitude >= threshold)[0]
    if len(bad) == 0:
        t_conv = float(t[0])
    elif bad[-1] + 1 < len(t):
        t_conv = float(t[bad[-1] + 1])
    else:
        t_conv = None
    return {
        "t_conv_s": t_conv,
        "t_conv_rule": "first time after which |S| stays below 1.5x its steady-window RMS",
        "threshold_v": threshold,
        "max_abs_s_first_20ms_v": float(np.max(magnitude[t <= 0.02])),
    }


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    base_engine = FuzzyEngine.from_yaml(directory)
    v_base = math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V") / math.sqrt(3.0)
    i_base = math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_power", unit="W") / (
        math.sqrt(3.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V")
    )
    k0 = config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-") * v_base
    a0 = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / i_base

    def rpm_at(t):
        return RPM_START + min(t / RAMP_S, 1.0) * (RPM_END - RPM_START)

    runs = {}
    diagnostics = {}
    for dt in TS_VALUES:
        t = np.arange(int(round(DURATION_S / dt))) * dt
        _, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60, DURATION_S, dt
        )
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        period = 1.0 / (parameters.pole_pairs * RPM_END / 60.0)
        steady_mask = (t >= DURATION_S - CYCLES * period) & (t <= DURATION_S)
        ramp_mask = (t >= 0.0) & (t <= RAMP_S)
        bound = 2.0 * (2.0 * parameters.ld / dt - parameters.stator_resistance) / (k0 * a0)

        def record(label, observer, *, gains=False, kf_af=None):
            sm = _rollout(observer, u, i, dt, record_gains=gains)
            vm = vector_metrics(
                sm["ea"][steady_mask], sm["eb"][steady_mask],
                emf[steady_mask, 0], emf[steady_mask, 1], theta[steady_mask],
            )
            entry = {
                "ts_us": dt * 1e6,
                "label": label,
                "steady": {
                    "fundamental_gain": vm["fundamental_gain"],
                    "fundamental_phase_deg": vm["fundamental_phase_deg"],
                    "ripple_rms_v": vm["ripple_rms_v"],
                    **_surface_metrics(sm["sa"], sm["sb"], steady_mask),
                    "period2_flag": _period2(sm["sa"][steady_mask]) > 0.5,
                    "post_update_current_rmse_a_old": float(
                        np.sqrt(np.mean(
                            np.hypot(sm["ia"][steady_mask] - i[steady_mask, 0],
                                     sm["ib"][steady_mask] - i[steady_mask, 1]) ** 2
                        ))
                    ),
                },
                "transient": _transient(sm["sa"], sm["sb"], t, steady_mask),
                "ramp": summarize_ramp(
                    sliding_window_ramp_metrics(
                        sm["ea"][ramp_mask], sm["eb"][ramp_mask],
                        emf[ramp_mask, 0], emf[ramp_mask, 1],
                        np.array([rpm_at(x) for x in t[ramp_mask]]) * parameters.pole_pairs / 60.0,
                        np.array([rpm_at(x) for x in t[ramp_mask]]) * parameters.pole_pairs / 60.0,
                        t[ramp_mask], window_s=0.06,
                    )
                ),
                "kf_af_product": kf_af,
                "in_stability_bound": None if kf_af is None else bool(kf_af < bound),
                "bound": bound,
            }
            if gains:
                entry["gains"] = {
                    "kf_mean": float(np.mean(sm["kf"][steady_mask])),
                    "kf_min": float(np.min(sm["kf"][steady_mask])),
                    "kf_max": float(np.max(sm["kf"][steady_mask])),
                    "af_mean": float(np.mean(sm["af"][steady_mask])),
                    "af_min": float(np.min(sm["af"][steady_mask])),
                    "af_max": float(np.max(sm["af"][steady_mask])),
                }
                if label == "flc_ke1_kec1":
                    sdot = np.zeros_like(sm["sa"])
                    sdot[1:] = (sm["sa"][1:] - sm["sa"][:-1]) / dt  # engine's backward difference
                    diagnostics[f"Ts{dt*1e6:.0f}_visited"] = {
                        "sn": (KE * sm["sa"]).tolist(),
                        "sdotn": (KEC * sdot).tolist(),
                    }
            runs[f"Ts{dt*1e6:.0f}_{label}"] = entry

        record("fixed_sign", ConventionalSMO.from_config(pmsm_cfg, observer_cfg))
        record("fixed_sigmoid", SigmoidSMO.from_config(pmsm_cfg, observer_cfg))
        for kf in KF_GRID:
            for af in AF_GRID:
                record(
                    f"fixed_kf{kf:g}_af{af:g}",
                    SigmoidSMO(SMOObserverParameters(
                        parameters.stator_resistance, parameters.ld, kf * k0, sigmoid_slope_a=af * a0
                    )),
                    kf_af=kf * af,
                )
        record(
            "fixed_flc_mean",
            SigmoidSMO(SMOObserverParameters(
                parameters.stator_resistance, parameters.ld, FLC_MEAN[0] * k0,
                sigmoid_slope_a=FLC_MEAN[1] * a0,
            )),
            kf_af=FLC_MEAN[0] * FLC_MEAN[1],
        )
        for ke_mult in GRID_MULTS:
            for kec_mult in GRID_MULTS:
                params = dataclasses.replace(
                    base_engine.parameters, ke=base_engine.parameters.ke * ke_mult,
                    kec=base_engine.parameters.kec * kec_mult,
                )
                label = f"flc_ke{ke_mult:g}_kec{kec_mult:g}"
                record(label, FuzzySMO(
                    SMOObserverParameters.from_config(pmsm_cfg, observer_cfg), FuzzyEngine(params)
                ), gains=True)

    # ---- FLC engine diagnostics (dense grid + visited samples, 100 us nominal)
    grid = np.linspace(-1.0, 1.0, 201)
    sn, sdn = np.meshgrid(grid, grid, indexing="ij")
    kf_surface = np.array([[base_engine.evaluate(float(a), float(b))[0] for b in grid] for a in grid])
    af_surface = np.array([[base_engine.evaluate(float(a), float(b))[1] for b in grid] for a in grid])
    visited = diagnostics.get("Ts100_visited", {"sn": [], "sdotn": []})
    sn_v = np.asarray(visited["sn"]); sdn_v = np.asarray(visited["sdotn"])
    saturated = float(np.mean((np.abs(sn_v) >= 0.999) | (np.abs(sdn_v) >= 0.999))) if len(sn_v) else 0.0
    engine_diag = {
        "achievable_kf_range": [float(kf_surface.min()), float(kf_surface.max())],
        "achievable_af_range": [float(af_surface.min()), float(af_surface.max())],
        "observed_kf_range": [runs["Ts100_flc_ke1_kec1"]["gains"]["kf_min"],
                              runs["Ts100_flc_ke1_kec1"]["gains"]["kf_max"]],
        "observed_af_range": [runs["Ts100_flc_ke1_kec1"]["gains"]["af_min"],
                              runs["Ts100_flc_ke1_kec1"]["gains"]["af_max"]],
        "visited_sn_range": [float(np.min(sn_v)), float(np.max(sn_v))] if len(sn_v) else None,
        "visited_sdotn_range": [float(np.min(sdn_v)), float(np.max(sdn_v))] if len(sdn_v) else None,
        "saturation_fraction": saturated,
        "min_achievable_k_v": float(kf_surface.min()) * k0,
        "e_max_v": E_MAX,
        "min_k_vs_e_max": "below" if float(kf_surface.min()) * k0 < E_MAX else "above",
    }

    # ---- dominance / Pareto
    flc_nominal = runs["Ts100_flc_ke1_kec1"]["steady"]
    fixed_cells = {name: entry for name, entry in runs.items()
                   if "_fixed_kf" in name or name.endswith("_fixed_flc_mean")}
    dominators = []
    for name, entry in fixed_cells.items():
        s = entry["steady"]
        if (s["ripple_rms_v"] <= flc_nominal["ripple_rms_v"]
                and abs(s["fundamental_gain"] - 1.0) <= abs(flc_nominal["fundamental_gain"] - 1.0)
                and abs(s["fundamental_phase_deg"]) <= abs(flc_nominal["fundamental_phase_deg"])
                and s["rms_abs_s_a"] <= flc_nominal["rms_abs_s_a"]):
            dominators.append(name)

    def pareto_flags(transform):
        fixed = [(entry["transient"]["t_conv_s"], transform(entry["steady"]))
                 for entry in fixed_cells.values() if entry["transient"]["t_conv_s"] is not None]
        flags = {}
        for name, entry in runs.items():
            if "flc_" not in name or entry["transient"]["t_conv_s"] is None:
                continue
            point = (entry["transient"]["t_conv_s"], transform(entry["steady"]))
            dominated = any(a <= point[0] and b <= point[1] and (a < point[0] or b < point[1])
                            for a, b in fixed)
            flags[name] = not dominated
        return flags

    result = {
        "note": "Phase 04b; RAW E_hat only. All FLC values are ASSUMPTIONS (Ke=0.13472 1/A, Kec=1.3472e-5 s/A, Ku=1, CoG grid=1001, Ruspini feet, min/max, per-axis, backward-difference s_dot).",
        "assumptions": {"ke_1_per_a": KE, "kec_s_per_a": KEC, "ku": 1.0, "cog_grid_points": 1001},
        "k0_v": k0, "a0_per_a": a0,
        "config_diff": config_diff.summary(
            "04b_fixed_gain",
            {"epsilon_filter_tau": None, "min_omega": None, "normalize_error": False,
             "positive_sequence": True},
            scenario={"ts_us": [x * 1e6 for x in TS_VALUES], "cycles": CYCLES},
        ),
        "runs": runs,
        "flc_diagnostics": engine_diag,
        "dominators_of_flc_nominal": dominators,
        "pareto_beyond_front_ripple": pareto_flags(lambda s: s["ripple_rms_v"]),
        "pareto_beyond_front_gain_error": pareto_flags(lambda s: abs(s["fundamental_gain"] - 1.0)),
    }
    output_dir = config.find_project_root() / "experiments" / "output" / "04b_fixed_gain"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "fixed_gain.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    # plots
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    im0 = axes[0].imshow(kf_surface.T, origin="lower", extent=(-1, 1, -1, 1), aspect="auto")
    axes[0].scatter(sn_v[::20], sdn_v[::20], s=1, c="crimson", alpha=0.5)
    axes[0].set(xlabel="s_n", ylabel="s_dot_n", title="kF(s_n, s_dot_n) + visited samples")
    fig.colorbar(im0, ax=axes[0])
    im1 = axes[1].imshow(af_surface.T, origin="lower", extent=(-1, 1, -1, 1), aspect="auto")
    axes[1].scatter(sn_v[::20], sdn_v[::20], s=1, c="crimson", alpha=0.5)
    axes[1].set(xlabel="s_n", ylabel="s_dot_n", title="aF(s_n, s_dot_n) + visited samples")
    fig.colorbar(im1, ax=axes[1])
    fig.suptitle("Phase 04b FLC surfaces; ASSUMPTION Ke=0.13472 1/A, Kec=1.3472e-5 s/A", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "flc_surfaces.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    fixed_names = [n for n in fixed_cells]
    for ax, objective, ylabel in (
        (axes[0], "ripple_rms_v", "steady ripple (V)"),
        (axes[1], "fundamental_gain", "|gain - 1|"),
    ):
        xs = [runs[n]["transient"]["t_conv_s"] for n in fixed_names
              if runs[n]["transient"]["t_conv_s"] is not None]
        ys = [runs[n]["steady"][objective] for n in fixed_names
              if runs[n]["transient"]["t_conv_s"] is not None]
        if objective == "fundamental_gain":
            ys = [abs(y - 1.0) for y in ys]
        ax.scatter(xs, ys, label="fixed-gain cells", alpha=0.7)
        flc = [n for n in runs if "flc_" in n and runs[n]["transient"]["t_conv_s"] is not None]
        ax.scatter([runs[n]["transient"]["t_conv_s"] for n in flc],
                   [runs[n]["steady"][objective] if objective != "fundamental_gain"
                    else abs(runs[n]["steady"][objective] - 1.0) for n in flc],
                   marker="x", label="FLC grid cells")
        nominal = runs["Ts100_flc_ke1_kec1"]
        ax.scatter([nominal["transient"]["t_conv_s"]],
                   [nominal["steady"][objective] if objective != "fundamental_gain"
                    else abs(nominal["steady"][objective] - 1.0)],
                   marker="*", s=140, label="FLC nominal (100 us)")
        ax.set(xlabel="t_conv (s)", ylabel=ylabel, title=f"t_conv vs {ylabel}")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.suptitle("Phase 04b transient vs steady trade-off (Ts=100 us shown for FLC nominal)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output_dir / "transient_tradeoff.png", dpi=130)
    plt.close(fig)

    config_diff.emit(result["config_diff"])
    print("== unified steady metrics (Ts=100us) ==")
    for name in ("fixed_sign", "fixed_sigmoid", "fixed_flc_mean", "flc_ke1_kec1"):
        s = runs[f"Ts100_{name}"]["steady"]
        t_conv = runs[f"Ts100_{name}"]["transient"]["t_conv_s"]
        print(f"  {name:16s}: gain={s['fundamental_gain']:.3f} phase={s['fundamental_phase_deg']:+6.2f} "
              f"ripple={s['ripple_rms_v']:7.2f} rmsS={s['rms_abs_s_a']:.3f} meanS={s['mean_abs_s_a']:.3f} "
              f"tconv={t_conv} maxS20={runs[f'Ts100_{name}']['transient']['max_abs_s_first_20ms_v']:.2f}")
    print("== dominators of FLC nominal:", dominators)
    print("== FLC diag:", json.dumps(engine_diag, indent=1))
    print("== Pareto beyond fixed-gain front (ripple):", result["pareto_beyond_front_ripple"])
    print("== Pareto beyond fixed-gain front (gain err):", result["pareto_beyond_front_gain_error"])
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
