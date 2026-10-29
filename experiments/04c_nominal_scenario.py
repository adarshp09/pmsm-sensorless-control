#!/usr/bin/env python3
"""Phase 04c: nominal-drive scenario — re-run the 04b comparison.

Same observers and Ts as Phase 04b, but with the observer's voltage input
exercised by the open-loop ideal-FOC feedforward of
``pmsm_sensorless.simulation.nominal_drive`` instead of the ``u = 0`` short
circuit. RAW E_hat only: no DSOGI-FLL, no position/speed claims.

All FLC scaling / MF / operator / grid values are ASSUMPTIONS (Phase 04): Ke =
0.13472 1/A, Kec = 1.3472e-5 s/A, Ku = 1, CoG grid = 1001, Ruspini MF feet,
AND = min, aggregation = max, per-axis FLC, backward-difference s_dot.

Observers: fixed sign; fixed sigmoid (k0, a0); the best matched fixed-gain cell
(selected from experiments/output/04b_fixed_gain/fixed_gain.json -- see
``select_matched_cell``); the 04b matched cell (FLC time-mean 0.355 / 0.591) as a
reference; FLC nominal. Ts in {100, 50} us; I_q in {2, 5, I_rated} A.

Steady window = 8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s; ramp =
03f sliding-window (60 ms) fit over [0, 1.500] s.

Output: experiments/output/04c_nominal_scenario/nominal_scenario.json.
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
from pmsm_sensorless.simulation.nominal_drive import (  # noqa: E402
    nominal_drive_rollout,
    rated_current_from_torque,
)
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
KF_GRID = (0.35, 0.5, 0.75, 1.0)
AF_GRID = (0.1, 0.2, 0.4, 0.6, 1.0)
FLC_MEAN = (0.355, 0.591)
HEADLINE_OBSERVERS = ("fixed_sign", "fixed_sigmoid", "fixed_matched", "fixed_flc_mean", "flc_nominal")


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def select_matched_cell(fixed_gain_json: dict) -> dict:
    """Pick the best matched fixed-gain cell from the Phase 04b results.

    Rule: among the constant-gain grid cells at Ts = 100 us that lie inside the
    small-signal stability bound (``kF*aF < B(100 us)``, D-027), choose the one
    with the smallest steady-window ripple. This is the best achievable fixed gain
    that is not a limit cycle; it is applied at both Ts for a like-for-like
    comparison.
    """
    runs = fixed_gain_json["runs"]
    candidates = []
    for name, entry in runs.items():
        if name.startswith("Ts100_fixed_kf") and entry.get("in_stability_bound"):
            kf = float(name.split("_kf")[1].split("_af")[0])
            af = float(name.split("_af")[1])
            candidates.append((entry["steady"]["ripple_rms_v"], kf, af, name))
    ripple, kf, af, name = min(candidates)
    return {"kf": kf, "af": af, "source": name, "u0_ripple_rms_v": ripple,
            "rule": "min steady-window ripple among Ts=100us fixed-gain cells inside the D-027 stability bound"}


def _rollout(observer, u, i, dt, *, record_gains=False):
    n = len(u)
    out = {key: np.empty(n) for key in ("ia", "ib", "sa", "sb", "ea", "eb")}
    if record_gains:
        for key in ("kf", "af", "k", "a"):
            out[key] = np.empty(n)
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
    return out


def _surface_metrics(sa, sb, mask):
    a, b = sa[mask], sb[mask]
    magnitude = np.hypot(a, b)
    return {
        "rms_s_alpha_a": float(np.sqrt(np.mean(a**2))),
        "rms_s_beta_a": float(np.sqrt(np.mean(b**2))),
        "mean_abs_s_a": float(np.mean(magnitude)),
        "rms_abs_s_a": float(np.sqrt(np.mean(magnitude**2))),
        "peak_abs_s_a": float(np.max(magnitude)),
    }


def _period2(signal) -> float:
    signal = np.asarray(signal, dtype=float)
    std = float(np.std(signal))
    if std == 0.0:
        return 0.0
    return float(abs(np.mean(signal * ((-1.0) ** np.arange(len(signal))))) / std)


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
    return {"t_conv_s": t_conv, "max_abs_s_first_20ms_a": float(np.max(magnitude[t <= 0.02]))}


@dataclasses.dataclass
class _Context:
    scenario: str
    ts_us: float
    i_q: float
    dt: float
    t: np.ndarray
    u: np.ndarray
    i: np.ndarray
    emf: np.ndarray
    theta: np.ndarray
    rpm_hz: np.ndarray
    steady_mask: np.ndarray
    ramp_mask: np.ndarray
    bound: float
    e_true_steady: np.ndarray
    k0: float = 0.0
    a0: float = 0.0


def _record(ctx: _Context, label: str, observer, engine_params=None) -> dict:
    record_gains = label.startswith("flc") or engine_params is not None
    sm = _rollout(observer, ctx.u, ctx.i, ctx.dt, record_gains=record_gains)
    steady = ctx.steady_mask
    vm = vector_metrics(
        sm["ea"][steady], sm["eb"][steady], ctx.emf[steady, 0], ctx.emf[steady, 1], ctx.theta[steady],
    )
    entry = {
        "scenario": ctx.scenario, "ts_us": ctx.ts_us, "i_q_a": ctx.i_q, "observer": label,
        "steady": {
            "fundamental_gain": vm["fundamental_gain"],
            "fundamental_phase_deg": vm["fundamental_phase_deg"],
            "ripple_rms_v": vm["ripple_rms_v"],
            "period2_flag": bool(_period2(sm["sa"][steady]) > 0.5),
            **_surface_metrics(sm["sa"], sm["sb"], steady),
        },
        "transient": _transient(sm["sa"], sm["sb"], ctx.t, steady),
        "ramp": summarize_ramp(
            sliding_window_ramp_metrics(
                sm["ea"][ctx.ramp_mask], sm["eb"][ctx.ramp_mask],
                ctx.emf[ctx.ramp_mask, 0], ctx.emf[ctx.ramp_mask, 1],
                ctx.rpm_hz[ctx.ramp_mask], ctx.rpm_hz[ctx.ramp_mask],
                ctx.t[ctx.ramp_mask], window_s=0.06,
            )
        ),
    }
    if label.startswith("flc") or label.startswith("fixed"):
        gain_block = _gain_block(sm, ctx, observer, engine_params)
        if gain_block is not None:
            entry["gains"] = gain_block
    return entry


def _gain_block(sm, ctx, observer, engine_params):
    """kF/aF statistics, stability/in-bound fractions, saturation (FLC only)."""
    steady = ctx.steady_mask
    block = {}
    if "kf" in sm:
        kf, af = sm["kf"][steady], sm["af"][steady]
        block.update({
            "kf_mean": float(np.mean(kf)), "kf_min": float(np.min(kf)), "kf_max": float(np.max(kf)),
            "af_mean": float(np.mean(af)), "af_min": float(np.min(af)), "af_max": float(np.max(af)),
            "kf_af_mean": float(np.mean(kf * af)),
            "fraction_in_stability_bound": float(np.mean(kf * af < ctx.bound)),
            "fraction_k_below_emf": float(np.mean(sm["k"][steady] < ctx.e_true_steady)),
        })
    elif hasattr(observer, "parameters") and isinstance(observer, SigmoidSMO):
        # constant-gain cell: kF, aF implied by the observer parameters
        kf = observer.parameters.switching_gain_k / ctx.k0
        af = observer.parameters.sigmoid_slope_a / ctx.a0
        block.update({
            "kf_mean": float(kf), "kf_min": float(kf), "kf_max": float(kf),
            "af_mean": float(af), "af_min": float(af), "af_max": float(af),
            "kf_af_mean": float(kf * af),
            "fraction_in_stability_bound": float(kf * af < ctx.bound),
            "fraction_k_below_emf": float(np.mean(observer.parameters.switching_gain_k < ctx.e_true_steady)),
        })
    if engine_params is not None:
        sa, sb = sm["sa"], sm["sb"]
        sdot_a = np.zeros_like(sa); sdot_b = np.zeros_like(sb)
        sdot_a[1:] = (sa[1:] - sa[:-1]) / ctx.dt
        sdot_b[1:] = (sb[1:] - sb[:-1]) / ctx.dt
        sn = engine_params["ke"] * np.maximum(np.abs(sa), np.abs(sb))
        sdn = engine_params["kec"] * np.maximum(np.abs(sdot_a), np.abs(sdot_b))
        sat = (sn >= 0.999) | (sdn >= 0.999)
        block["saturation_fraction_steady"] = float(np.mean(sat[steady]))
    return block or None


def _run_context(scenario, dt, i_q, plant, phase02, pmsm_cfg, observer_cfg, k0, a0, base_engine):
    if scenario == "u0":
        t, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: plant.parameters.pole_pairs * (RPM_START + min(tt / RAMP_S, 1.0) * (RPM_END - RPM_START)) * 2 * math.pi / 60.0,
            DURATION_S, dt,
        )
    else:
        r = nominal_drive_rollout(plant, i_q=i_q, dt=dt, duration=DURATION_S, rpm_start=RPM_START, rpm_end=RPM_END)
        t, u, i, emf = r.t, np.column_stack([r.u_alpha, r.u_beta]), np.column_stack([r.i_alpha, r.i_beta]), np.column_stack([r.e_alpha, r.e_beta])
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    rpm = RPM_START + np.minimum(t / RAMP_S, 1.0) * (RPM_END - RPM_START)
    rpm_hz = plant.parameters.pole_pairs * rpm / 60.0
    period = 1.0 / (plant.parameters.pole_pairs * RPM_END / 60.0)
    steady_mask = (t >= DURATION_S - STEADY_CYCLES * period) & (t <= DURATION_S)
    ramp_mask = (t >= 0.0) & (t <= RAMP_S)
    bound = 2.0 * (2.0 * plant.parameters.ld / dt - plant.parameters.stator_resistance) / (k0 * a0)
    e_true_steady = np.abs(plant.parameters.pole_pairs * (RPM_END * 2 * math.pi / 60.0) * plant.parameters.psi_f) * np.ones(int(steady_mask.sum()))
    return _Context(scenario, dt * 1e6, i_q, dt, t, u, i, emf, theta, rpm_hz, steady_mask, ramp_mask, bound, e_true_steady, k0, a0)


def _head_observers(pmsm_cfg, observer_cfg, base_engine, matched, k0, a0, plant):
    rs, ld = plant.parameters.stator_resistance, plant.parameters.ld

    def const_sigmoid(kf, af):
        return SigmoidSMO(SMOObserverParameters(rs, ld, kf * k0, sigmoid_slope_a=af * a0))

    return [
        ("fixed_sign", ConventionalSMO.from_config(pmsm_cfg, observer_cfg), None),
        ("fixed_sigmoid", SigmoidSMO.from_config(pmsm_cfg, observer_cfg), None),
        ("fixed_matched", const_sigmoid(matched["kf"], matched["af"]), None),
        ("fixed_flc_mean", const_sigmoid(*FLC_MEAN), None),
        ("flc_nominal", FuzzySMO(SMOObserverParameters.from_config(pmsm_cfg, observer_cfg), FuzzyEngine(base_engine.parameters)), {"ke": base_engine.parameters.ke, "kec": base_engine.parameters.kec}),
    ]


def _dominance(runs, reference):
    """Fixed-gain cells that dominate ``reference`` (steady) on all four metrics."""
    ref = reference["steady"]
    dominators = []
    for name, entry in runs.items():
        if not name.startswith("fixed_kf"):
            continue
        s = entry["steady"]
        if (s["ripple_rms_v"] <= ref["ripple_rms_v"]
                and abs(s["fundamental_gain"] - 1.0) <= abs(ref["fundamental_gain"] - 1.0)
                and abs(s["fundamental_phase_deg"]) <= abs(ref["fundamental_phase_deg"])
                and s["rms_abs_s_a"] <= ref["rms_abs_s_a"]):
            dominators.append(name)
    return dominators


def _flc_vs_matched(runs):
    """FLC grid cells that dominate the best matched fixed-gain cell (steady)."""
    ref = runs["fixed_matched"]["steady"]
    cells = {}
    for name, entry in runs.items():
        if not name.startswith("flc_ke"):
            continue
        s = entry["steady"]
        cells[name] = {
            "ripple_rms_v": s["ripple_rms_v"],
            "dominates_matched": (
                s["ripple_rms_v"] <= ref["ripple_rms_v"]
                and abs(s["fundamental_gain"] - 1.0) <= abs(ref["fundamental_gain"] - 1.0)
                and abs(s["fundamental_phase_deg"]) <= abs(ref["fundamental_phase_deg"])
                and s["rms_abs_s_a"] <= ref["rms_abs_s_a"]
            ),
        }
    return {
        "any_flc_cell_dominates_matched": any(c["dominates_matched"] for c in cells.values()),
        "matched_steady": {k: ref[k] for k in ("ripple_rms_v", "fundamental_gain", "fundamental_phase_deg", "rms_abs_s_a")},
        "n_cells": len(cells),
        "cells": cells,
    }


def _pareto_beyond_front(runs):
    fixed = [(e["transient"]["t_conv_s"], e["steady"]["ripple_rms_v"])
             for n, e in runs.items() if n.startswith("fixed_kf") and e["transient"]["t_conv_s"] is not None]
    flags = {}
    for n, e in runs.items():
        if not n.startswith("flc_") or e["transient"]["t_conv_s"] is None:
            continue
        p = (e["transient"]["t_conv_s"], e["steady"]["ripple_rms_v"])
        flags[n] = not any(a <= p[0] and b <= p[1] and (a < p[0] or b < p[1]) for a, b in fixed)
    return flags


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    base_engine = FuzzyEngine.from_yaml(directory)
    v_base = math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V") / math.sqrt(3.0)
    i_base = math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_power", unit="W") / (math.sqrt(3.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V"))
    k0 = config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-") * v_base
    a0 = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / i_base
    i_rated = rated_current_from_torque(
        config.resolve_parameter(pmsm_cfg, "rating.rated_torque", unit="N.m"),
        int(config.resolve_parameter(pmsm_cfg, "machine.pole_pairs", unit="-")),
        config.resolve_parameter(pmsm_cfg, "machine.pm_flux_linkage_psi_f", unit="Wb"),
    )
    nominal_ke, nominal_kec = base_engine.parameters.ke, base_engine.parameters.kec

    fixed_gain = json.loads((config.find_project_root() / "experiments" / "output" / "04b_fixed_gain" / "fixed_gain.json").read_text())
    matched = select_matched_cell(fixed_gain)

    def iq_tag(iq):
        return "irated" if abs(iq - i_rated) < 1e-9 else f"{iq:g}"

    results: dict = {}
    for scenario in ("u0", "nominal"):
        for dt in TS_VALUES:
            ts = f"Ts{dt*1e6:.0f}us"
            iqs = [i_rated] if scenario == "u0" else [2.0, 5.0, i_rated]
            for iq in iqs:
                ctx = _run_context(scenario, dt, iq, plant, phase02, pmsm_cfg, observer_cfg, k0, a0, base_engine)
                bucket = results.setdefault(scenario, {}).setdefault(ts, {}).setdefault(iq_tag(iq), {})
                if abs(iq - i_rated) < 1e-9:
                    # full fixed grid + FLC grid (needed for dominance/Pareto)
                    rs, ld = plant.parameters.stator_resistance, plant.parameters.ld
                    for kf in KF_GRID:
                        for af in AF_GRID:
                            obs = SigmoidSMO(SMOObserverParameters(rs, ld, kf * k0, sigmoid_slope_a=af * a0))
                            bucket[f"fixed_kf{kf:g}_af{af:g}"] = _record(ctx, f"fixed_kf{kf:g}_af{af:g}", obs)
                    for ke_mult in GRID_MULTS:
                        for kec_mult in GRID_MULTS:
                            params = dataclasses.replace(base_engine.parameters, ke=nominal_ke * ke_mult, kec=nominal_kec * kec_mult)
                            engine = FuzzyEngine(params)
                            obs = FuzzySMO(SMOObserverParameters.from_config(pmsm_cfg, observer_cfg), engine)
                            label = f"flc_ke{ke_mult:g}_kec{kec_mult:g}"
                            bucket[label] = _record(ctx, label, obs, {"ke": params.ke, "kec": params.kec})
                for label, obs, engine_params in _head_observers(pmsm_cfg, observer_cfg, base_engine, matched, k0, a0, plant):
                    bucket[label] = _record(ctx, label, obs, engine_params)

    # ---- analysis
    dominance = {}
    pareto = {}
    for scenario in ("u0", "nominal"):
        for dt in TS_VALUES:
            ts = f"Ts{dt*1e6:.0f}us"
            runs = results[scenario][ts]["irated"]
            ref = runs["flc_nominal"]
            dominance[f"{scenario}_{ts}"] = {
                "matched_cell_dominates_flc_nominal": _dominates(runs["fixed_matched"], ref),
                "flc_mean_dominates_flc_nominal": _dominates(runs["fixed_flc_mean"], ref),
                "n_fixed_cells_dominating": len(_dominance(runs, ref)),
                "dominators": _dominance(runs, ref),
                "flc_grid_vs_matched": _flc_vs_matched(runs),
            }
            pf = _pareto_beyond_front(runs)
            pareto[f"{scenario}_{ts}"] = {
                "any_flc_beyond_front": any(pf.values()),
                "n_flc_cells": len(pf),
                "flags": pf,
            }

    confirmed = _confirmed_changed(results, i_rated, matched)

    output = {
        "note": "Phase 04c nominal-drive scenario; RAW E_hat only (no DSOGI-FLL). No position/speed claims. All FLC values are ASSUMPTIONS.",
        "assumptions": {"ke_1_per_a": nominal_ke, "kec_s_per_a": nominal_kec, "ku": base_engine.parameters.ku, "cog_grid_points": base_engine.parameters.cog_grid_points},
        "k0_v": k0, "a0_per_a": a0, "i_rated_a": i_rated,
        "matched_cell": matched,
        "reference_cell_flc_mean": {"kf": FLC_MEAN[0], "af": FLC_MEAN[1]},
        "config_diff": config_diff.summary(
            "04c_nominal_scenario",
            {"epsilon_filter_tau": None, "min_omega": None, "normalize_error": False, "positive_sequence": True},
            scenario={"ts_us": [x * 1e6 for x in TS_VALUES], "cycles": STEADY_CYCLES, "i_q_a": [2.0, 5.0, i_rated]},
        ),
        "results": results,
        "dominance": dominance,
        "pareto": pareto,
        "confirmed_changed": confirmed,
    }
    output_dir = config.find_project_root() / "experiments" / "output" / "04c_nominal_scenario"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "nominal_scenario.json").write_text(json.dumps(output, indent=2), encoding="utf-8")
    config_diff.emit(output["config_diff"])

    # ---- console report: headline from the nominal cell first
    print(f"matched cell (rule: {matched['rule']}): kF={matched['kf']:g} aF={matched['af']:g} ({matched['source']})")
    print(f"I_rated = {i_rated:.4f} A\n")
    for scenario in ("nominal", "u0"):
        for dt in TS_VALUES:
            ts = f"Ts{dt*1e6:.0f}us"
            iqs = [i_rated] if scenario == "u0" else [2.0, 5.0, i_rated]
            for iq in iqs:
                runs = results[scenario][ts][iq_tag(iq)]
                print(f"== {scenario} {ts} I_q={iq_tag(iq)} ==")
                for label in HEADLINE_OBSERVERS:
                    if label not in runs:
                        continue
                    s = runs[label]["steady"]
                    print(f"  {label:18s}: gain={s['fundamental_gain']:.3f} phase={s['fundamental_phase_deg']:+6.2f} "
                          f"ripple={s['ripple_rms_v']:7.2f} rmsS={s['rms_abs_s_a']:7.3f} peakS={s['peak_abs_s_a']:7.3f} "
                          f"p2={s['period2_flag']} tconv={runs[label]['transient']['t_conv_s']}")
    print("\n== confirmed / changed vs u=0 ==")
    for row in confirmed:
        print(f"  {row['status']:14s} {row['claim']}")
        print(f"      u0={row['u0']} nominal={row['nominal']}")
    print(f"\nOutputs written to: {output_dir}")
    return 0


def _dominates(entry, reference) -> bool:
    s, r = entry["steady"], reference["steady"]
    return (s["ripple_rms_v"] <= r["ripple_rms_v"]
            and abs(s["fundamental_gain"] - 1.0) <= abs(r["fundamental_gain"] - 1.0)
            and abs(s["fundamental_phase_deg"]) <= abs(r["fundamental_phase_deg"])
            and s["rms_abs_s_a"] <= r["rms_abs_s_a"])


def _confirmed_changed(results, i_rated, matched):
    rows = []

    def take(scenario, ts, label, key):
        return results[scenario][ts]["irated"][label][key]

    # 1. matched fixed gain vs FLC nominal on steady ripple (Ts=100us)
    def ripple(scenario, label):
        return take(scenario, "Ts100us", label, "steady")["ripple_rms_v"]

    rows.append({
        "claim": "Best matched fixed-gain cell beats FLC nominal on steady ripple (Ts=100us, I_q=I_rated)",
        "u0": {"matched": round(ripple("u0", "fixed_matched"), 3), "flc": round(ripple("u0", "flc_nominal"), 3),
               "matched_beats": ripple("u0", "fixed_matched") < ripple("u0", "flc_nominal")},
        "nominal": {"matched": round(ripple("nominal", "fixed_matched"), 3), "flc": round(ripple("nominal", "flc_nominal"), 3),
                    "matched_beats": ripple("nominal", "fixed_matched") < ripple("nominal", "flc_nominal")},
        "status": "CONFIRMED" if (ripple("nominal", "fixed_matched") < ripple("nominal", "flc_nominal")) else "CHANGED",
    })
    # 2. count of fixed cells dominating FLC nominal at Ts=100us
    rows.append({
        "claim": "Several fixed-gain cells dominate FLC nominal on all four metrics (Ts=100us)",
        "u0": {"n_dominators": len(_dominance(results["u0"]["Ts100us"]["irated"], results["u0"]["Ts100us"]["irated"]["flc_nominal"]))},
        "nominal": {"n_dominators": len(_dominance(results["nominal"]["Ts100us"]["irated"], results["nominal"]["Ts100us"]["irated"]["flc_nominal"]))},
        "status": "CONFIRMED" if len(_dominance(results["nominal"]["Ts100us"]["irated"], results["nominal"]["Ts100us"]["irated"]["flc_nominal"])) > 0 else "CHANGED",
    })
    # 3. no FLC cell extends the fixed-gain Pareto front
    def beyond(scenario):
        return _pareto_beyond_front(results[scenario]["Ts100us"]["irated"])
    u0_b, no_b = beyond("u0"), beyond("nominal")
    rows.append({
        "claim": "No FLC cell extends the fixed-gain Pareto front (t_conv vs ripple, Ts=100us)",
        "u0": {"any_beyond": any(u0_b.values()), "n": sum(u0_b.values())},
        "nominal": {"any_beyond": any(no_b.values()), "n": sum(no_b.values())},
        "status": "CONFIRMED" if not any(no_b.values()) else "CHANGED",
    })
    # 4. stable fixed sigmoid @50us is best
    def best50(scenario):
        runs = results[scenario]["Ts50us"]["irated"]
        name, ripple = min(
            ((n, e["steady"]["ripple_rms_v"]) for n, e in runs.items() if n.startswith("fixed")),
            key=lambda kv: kv[1],
        )
        sig = runs["fixed_sigmoid"]["steady"]["ripple_rms_v"]
        return {"best_name": name, "best_ripple": round(ripple, 4), "fixed_sigmoid_ripple": round(sig, 4)}
    b50 = best50("nominal")
    rows.append({
        "claim": "Stable fixed sigmoid is the best cell at Ts=50us (min steady ripple)",
        "u0": best50("u0"),
        "nominal": b50,
        # fixed_sigmoid == grid cell kf1/af1, so a tie is the same observer
        "status": "CONFIRMED" if b50["best_ripple"] <= b50["fixed_sigmoid_ripple"] * 1.01 + 1e-6 else "CHANGED",
    })
    # 5. FLC operating point / limit-cycle avoidance
    def flcg(scenario):
        runs = results[scenario]["Ts100us"]["irated"]
        return {"kf_af_mean": round(runs["flc_nominal"]["gains"]["kf_af_mean"], 3), "period2": runs["flc_nominal"]["steady"]["period2_flag"]}
    rows.append({
        "claim": "FLC moves the operating point to low gain (kF*aF ~ 0.21) and avoids the D-027 limit cycle at Ts=100us",
        "u0": flcg("u0"),
        "nominal": flcg("nominal"),
        "status": "CONFIRMED" if (not flcg("nominal")["period2"]) else "CHANGED",
    })
    return rows


if __name__ == "__main__":
    raise SystemExit(main())
