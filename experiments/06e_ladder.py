#!/usr/bin/env python3
"""Phase 06e Step 3: ablation ladder (fixed t_h=0.30 s; dead time OFF; Ts=100 us).

Two f_n tables: 20 Hz (frozen nominal) and the Step-2 "paper-level" f_n (ASSUMPTION).
R1 conventional (sign SMO -> LPF grid {100,200,400} x comp {off,on} -> Q-PLL) + arctan;
R2 sign -> DSOGI v'; R3 fixed sigmoid -> DSOGI; R4 FLC -> DSOGI; R5 matched -> DSOGI
(nominal); R6 matched -> raw E_hat -> Q-PLL (DIAGNOSTIC).

Scenarios S1 (2.5 and nominal alpha_w), S3, S4, S2 at alpha_w=4.4; S4 also dead ON+comp.

Output: experiments/output/06e_ladder/ladder.json
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
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.fuzzy_smo import FuzzySMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import SMOObserverParameters  # noqa: E402
from pmsm_sensorless.fuzzy.engine import FuzzyEngine  # noqa: E402

STACK = None
PARAMS = None


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


CMP = _load("c06c", "06c_comparison.py")
D06 = _load("d06", "06d_diagnosis.py")


def _qpll(fn):
    return QPLL(QPLLParameters.from_design(zeta=0.707, f_n_hz=fn, amplitude_v=STACK.design_amp_v))


def _fll():
    a = C.handover_assumptions(STACK)
    return C.build_fll(
        STACK.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(STACK)
    )


def _wrap(observer, fn):
    return C.EstimatorSource(
        observer=observer, fll=_fll(), qpll=_qpll(fn), pole_pairs=STACK.pole_pairs
    )


def builders(fn):
    s = STACK
    rs, ld = PARAMS.stator_resistance, PARAMS.ld
    b = {
        "R2_sign_dsogi": lambda: _wrap(ConventionalSMO(SMOObserverParameters(rs, ld, s.k0)), fn),
        "R3_fixed_sigmoid_dsogi": lambda: _wrap(
            SigmoidSMO(SMOObserverParameters(rs, ld, s.k0, sigmoid_slope_a=s.a0)), fn
        ),
        "R4_flc_dsogi": lambda: _wrap(
            FuzzySMO.from_config(s.pmsm_config, s.observer_config, fuzzy_directory=s.config_dir), fn
        ),
        "R5_matched_dsogi": lambda: _wrap(C.build_observer("matched_sigmoid", s), fn),
        "R6_matched_raw_qpll": lambda: D06.make_source(
            1,
            s,
            f_n=fn,
            initial_omega=s.initial_omega_at(C.handover_assumptions(s).omega_h_rpm),
            min_omega=C.fll_min_omega_rad_s(s),
        ),
    }
    for fc in (100, 200, 400):
        for comp in (False, True):
            b[f"R1_conv_fc{fc}_{'comp' if comp else 'raw'}"] = lambda fc=fc, comp=comp: (
                CMP.ConventionalEstimator(s, lpf_hz=float(fc), compensate=comp, f_n=fn)
            )
    b["R1_conv_arctan_fc200_raw"] = lambda: CMP.ConventionalEstimator(
        s, lpf_hz=200.0, compensate=False, estimator="arctan", f_n=fn
    )
    return b


def _row(rec, src, res, spec):
    ok, peak = P.lock_valid(rec)
    m = P.metrics(
        rec,
        src,
        res,
        dynamic=spec["dynamic"],
        steady=spec["steady"],
        disturbance_s=spec["disturbance_s"],
    )
    return {
        "lock_valid": ok,
        "lock_peak_deg": peak,
        "steady_theta_max_deg": m["steady_theta"]["max"],
        "steady_theta_rms_deg": m["steady_theta"]["rms"],
        "dynamic_theta_max_deg": m["dynamic_theta"]["max"],
        "dynamic_omega_max_rpm": m["dynamic_omega"]["max"],
        "dip_rpm": m.get("dip_rpm"),
        "recovery_10rpm_s": m.get("recovery_10rpm_s"),
        "current_ripple_rms_a": m.get("current_ripple_rms_a"),
        "saturation_after_transient": m["saturation_after_transient"],
        "fll_status": m["fll_status"],
        "handed_over": m["handed_over"],
    }


def main() -> int:
    global STACK, PARAMS
    stack, params, plant, rule = E.load()
    STACK, PARAMS = stack, params
    scenarios = E.paper_scenarios(rule)
    s2 = dict(scenarios["S2_sim_50pct_1000"])

    # paper-level f_n from the sweep
    paper_fn = 160.0
    reached = False
    sw = config.find_project_root() / "experiments" / "output" / "06e_sweep" / "sweep.json"
    if sw.is_file():
        cells = json.loads(sw.read_text())["cells"]
        for fn in (20.0, 40.0, 80.0, 160.0):
            c = cells.get(f"S2_sim_50pct_1000|fn{fn:g}|aw4.4|advoff")
            if c and c["dynamic_omega_max_rpm"] is not None and c["dynamic_omega_max_rpm"] <= 46.0:
                paper_fn, reached = fn, True
                break

    scen_runs = {
        "S1_2p5": (dict(scenarios["S1_no_load_ramp"]), 2.5),
        "S1_nominal": (dict(scenarios["S1_no_load_ramp"]), 10.0),
        "S3": (dict(scenarios["S3_exp_20pct_1000"]), 10.0),
        "S4": (dict(scenarios["S4_steady_1000"]), 10.0),
        "S2_aw4p4": (s2, 4.4),
    }
    tables = {}
    for table, fn in (("fn20_frozen", 20.0), ("fn_paper_level", paper_fn)):
        rows = {}
        for cname, builder in builders(fn).items():
            for rname, (spec, aw) in scen_runs.items():
                rec, src, res = P.run_fixed(builder, stack, params, plant, spec, alpha_w=aw)
                rows[f"{cname}|{rname}"] = _row(rec, src, res, spec)
        # S4 dead time ON+comp for the matched chain
        for cname in ("R5_matched_dsogi",):
            spec = scen_runs["S4"][0]
            rec, src, res = P.run_fixed(
                builders(fn)[cname],
                stack,
                params,
                plant,
                spec,
                alpha_w=10.0,
                dead_time=2e-6,
                compensation=True,
            )
            rows[f"{cname}|S4_deadOnComp"] = _row(rec, src, res, spec)
        tables[table] = {"f_n_hz": fn, "rows": rows}

    out = {
        "note": (
            "Phase 06e Step 3 ablation ladder; fixed t_h=0.30 s; dead time OFF "
            "(S4 also dead ON+comp); two f_n tables. Conventional/LPF = our "
            "construction; R6 DIAGNOSTIC."
        ),
        "paper_level_f_n_hz": paper_fn,
        "paper_level_reached_within_2x": reached,
        "alpha_w_used": {k: v[1] for k, v in scen_runs.items()},
        "tables": tables,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06e_ladder"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "ladder.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"paper-level f_n = {paper_fn} Hz (reached within 2x: {reached})")
    for table, tb in tables.items():
        print(f"\n== {table} (f_n={tb['f_n_hz']:g} Hz) ==")
        for k, v in tb["rows"].items():
            print(
                f"  {k:38s} lv={v['lock_valid']}({v['lock_peak_deg']}) steady={v['steady_theta_max_deg']} "
                f"dyn_dth={v['dynamic_theta_max_deg']} dyn_dw={v['dynamic_omega_max_rpm']} "
                f"ripple={v['current_ripple_rms_a']} sat={v['saturation_after_transient']}"
            )
    print(f"\nOutputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
