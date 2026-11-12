#!/usr/bin/env python3
"""Phase 06c-2 Steps 2, 3, 5: paper-matched dips, conventional-vs-improved, normalisation.

All runs use the fixed-time handover (t_h = 0.30 s) from c2_protocol; a chain is
*lock-valid* if max |theta_hat-theta| over [t_h-50ms, t_h] < 30 deg (evaluation only).

Output: experiments/output/06c2_suite/suite.json
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

import numpy as np  # noqa: E402

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import c2_protocol as P  # noqa: E402
import chain as C  # noqa: E402
import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


CMP = _load("c06c", "06c_comparison.py")
D06 = _load("d06", "06d_diagnosis.py")


def _matched():
    a = C.handover_assumptions(E_STACK)
    return C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", E_STACK),
        fll=C.build_fll(
            E_STACK.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(E_STACK)
        ),
        qpll=C.build_qpll(E_STACK),
        pole_pairs=E_STACK.pole_pairs,
    )


def _flc():
    a = C.handover_assumptions(E_STACK)
    return C.EstimatorSource(
        observer=C.build_observer("flc_nominal", E_STACK),
        fll=C.build_fll(
            E_STACK.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(E_STACK)
        ),
        qpll=C.build_qpll(E_STACK),
        pole_pairs=E_STACK.pole_pairs,
    )


def _arm1(fn=20.0):
    a = C.handover_assumptions(E_STACK)
    return D06.make_source(
        1,
        E_STACK,
        f_n=fn,
        initial_omega=E_STACK.initial_omega_at(a.omega_h_rpm),
        min_omega=C.fll_min_omega_rad_s(E_STACK),
    )


def _norm(fn=20.0):
    a = C.handover_assumptions(E_STACK)
    return C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", E_STACK),
        fll=C.build_fll(
            E_STACK.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(E_STACK)
        ),
        qpll=QPLL(
            QPLLParameters.from_design(
                zeta=0.707, f_n_hz=fn, amplitude_v=E_STACK.design_amp_v, normalize=True
            )
        ),
        pole_pairs=E_STACK.pole_pairs,
    )


def _summary(rec, source, res, spec):
    ok, peak_lock = P.lock_valid(rec)
    m = P.metrics(
        rec,
        source,
        res,
        dynamic=spec["dynamic"],
        steady=spec["steady"],
        disturbance_s=spec["disturbance_s"],
    )
    return {
        "lock_valid": ok,
        "lock_peak_deg": peak_lock,
        "steady_theta_max_deg": m["steady_theta"]["max"],
        "steady_omega_max_rpm": m["steady_omega"]["max"],
        "dynamic_theta_max_deg": m["dynamic_theta"]["max"],
        "dynamic_theta_rms_deg": m["dynamic_theta"]["rms"],
        "dynamic_omega_max_rpm": m["dynamic_omega"]["max"],
        "dip_rpm": m.get("dip_rpm"),
        "recovery_10rpm_s": m.get("recovery_10rpm_s"),
        "peak_i_q_a": m["peak_i_q_a"],
        "current_ripple_rms_a": m.get("current_ripple_rms_a"),
        "saturation_after_transient": m["saturation_after_transient"],
        "fll_status": m["fll_status"],
        "handed_over": m["handed_over"],
    }


def main() -> int:
    global E_STACK
    stack, params, plant, rule = E.load()
    E_STACK = stack
    scenarios = E.paper_scenarios(rule)
    alpha_grid = (3.0, 4.0, 5.0, 7.0, 10.0, 15.0, 20.0)

    # ---- Step 2: paper-matched dips (matched cell)
    step2 = {}
    for scen_name in ("S2_sim_50pct_1000", "S3_exp_20pct_1000"):
        spec = scenarios[scen_name]
        for aw in alpha_grid:
            for advance in (False, True):
                rec, src, res = P.run_fixed(
                    _matched, stack, params, plant, spec, alpha_w=aw, advance=advance
                )
                row = _summary(rec, src, res, spec)
                ratio = (
                    abs(row["dynamic_omega_max_rpm"]) / abs(row["dip_rpm"])
                    if row["dip_rpm"]
                    else None
                )
                row["ratio"] = ratio
                step2[f"{scen_name}|aw{aw:g}|adv{'on' if advance else 'off'}"] = row

    # interpolate dip and ratio vs alpha_w (advance off) to the paper dips
    def matched_alpha(scen_name, paper_dip):
        pts = [
            (aw, step2[f"{scen_name}|aw{aw:g}|advoff"]["dip_rpm"])
            for aw in alpha_grid
            if step2[f"{scen_name}|aw{aw:g}|advoff"]["dip_rpm"] is not None
        ]
        # dip ~ 1/alpha_w -> interpolate 1/|dip| vs alpha_w
        aws = np.array([p[0] for p in pts])
        inv = np.array([1.0 / abs(p[1]) for p in pts])
        a_match = float(np.interp(1.0 / paper_dip, inv[::-1], aws[::-1]))
        ratios = [(aw, step2[f"{scen_name}|aw{aw:g}|advoff"]["ratio"]) for aw in alpha_grid]
        r_match = float(np.interp(a_match, [r[0] for r in ratios], [r[1] for r in ratios]))
        return {"alpha_w_hz": a_match, "ratio": r_match, "predicted_error_rpm": r_match * paper_dip}

    matched_dips = {
        "S2_paper_dip_350": matched_alpha("S2_sim_50pct_1000", 350.0),
        "S3_paper_dip_46": matched_alpha("S3_exp_20pct_1000", 46.0),
    }
    s2_alpha = float(np.clip(matched_dips["S2_paper_dip_350"]["alpha_w_hz"], 3.0, 20.0))

    # ---- Step 3: conventional vs improved under t_h
    chains = {
        "matched_nominal": _matched,
        "flc_nominal": _flc,
        "arm1_raw_matched": _arm1,
        "qpll_normalized": _norm,
    }
    for fc in (100, 200, 400):
        for comp in (False, True):
            chains[f"conv_fc{fc}_{'comp' if comp else 'raw'}"] = lambda fc=fc, comp=comp: (
                CMP.ConventionalEstimator(stack, lpf_hz=float(fc), compensate=comp)
            )
    chains["conv_arctan_fc200_raw"] = lambda: CMP.ConventionalEstimator(
        stack, lpf_hz=200.0, compensate=False, estimator="arctan"
    )

    scen_runs = {
        "S1_nominal": (dict(scenarios["S1_no_load_ramp"]), 10.0),
        "S1_2p5": (dict(scenarios["S1_no_load_ramp"]), 2.5),
        "S3": (dict(scenarios["S3_exp_20pct_1000"]), 10.0),
        "S4": (dict(scenarios["S4_steady_1000"]), 10.0),
        "S2_matched": (dict(scenarios["S2_sim_50pct_1000"]), s2_alpha),
    }
    step3 = {}
    for cname, builder in chains.items():
        for rname, (spec, aw0) in scen_runs.items():
            rec, src, res = P.run_fixed(builder, stack, params, plant, spec, alpha_w=aw0)
            step3[f"{cname}|{rname}"] = _summary(rec, src, res, spec)

    # ---- Step 5: normalisation spot check
    step5 = {}
    s500_20 = dict(
        speed_ref=500.0,
        load=StepLoadTorque(0.0, 0.2 * E.RATED_TORQUE, 0.5),
        duration=2.0,
        alpha_w=10.0,
        dynamic=(0.5, 1.0),
        steady=(1.8, 2.0),
        disturbance_s=0.5,
    )
    for fn in (20.0, 40.0):
        for label, builder in (("normalized", lambda fn=fn: _norm(fn)), ("unnormalized", _matched)):
            for rname, spec in (("S3", scenarios["S3_exp_20pct_1000"]), ("S500_20", s500_20)):
                rec, src, res = P.run_fixed(builder, stack, params, plant, spec, alpha_w=10.0)
                step5[f"{label}|fn{fn:g}|{rname}"] = _summary(rec, src, res, spec)

    out = {
        "note": (
            "Phase 06c-2 fixed-time handover t_h=0.30 s (ASSUMPTION); lock-valid = "
            "max|theta_hat-theta|<30 deg over [t_h-50ms,t_h]. Matched cell, advance "
            "off unless stated. No tuning toward the paper."
        ),
        "protocol": {
            "t_h_s": P.T_H,
            "lock_window_s": P.LOCK_WINDOW_S,
            "lock_limit_deg": P.LOCK_LIMIT_DEG,
        },
        "step2_dips": step2,
        "matched_dips": matched_dips,
        "s2_matched_alpha_w": s2_alpha,
        "step3": step3,
        "step5": step5,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06c2_suite"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "suite.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== Step 2: matched-dip verdict ==")
    for k, v in matched_dips.items():
        print(
            f"  {k}: matched alpha_w={v['alpha_w_hz']:.2f} Hz  ratio={v['ratio']:.2f}  "
            f"predicted error={v['predicted_error_rpm']:.0f} rpm"
        )
    print("\n== Step 2 grid (advance off): alpha_w, S2/S3 dip, dyn|dw|, ratio ==")
    for aw in alpha_grid:
        a = step2[f"S2_sim_50pct_1000|aw{aw:g}|advoff"]
        b = step2[f"S3_exp_20pct_1000|aw{aw:g}|advoff"]
        print(
            f"  aw={aw:4g} Hz: S2 dip={a['dip_rpm']} dynw={a['dynamic_omega_max_rpm']} r={a['ratio']} lv={a['lock_valid']} | "
            f"S3 dip={b['dip_rpm']} dynw={b['dynamic_omega_max_rpm']} r={b['ratio']} lv={b['lock_valid']}"
        )
    print("\n== Step 3 (fixed t_h): chain | scenario | lock_valid peak | steady/dyn ==")
    for key, v in step3.items():
        print(
            f"  {key:44s} lv={v['lock_valid']} lockpk={v['lock_peak_deg']} "
            f"steady={v['steady_theta_max_deg']} dyn_dth={v['dynamic_theta_max_deg']} "
            f"dyn_dw={v['dynamic_omega_max_rpm']} ripple={v['current_ripple_rms_a']}"
        )
    print("\n== Step 5: normalization ==")
    for k, v in step5.items():
        print(
            f"  {k:24s} lv={v['lock_valid']} steady={v['steady_theta_max_deg']} dyn_dw={v['dynamic_omega_max_rpm']}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


E_STACK = None
if __name__ == "__main__":
    raise SystemExit(main())
