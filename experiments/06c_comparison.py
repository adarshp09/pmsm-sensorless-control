#!/usr/bin/env python3
"""Phase 06c Step 3+4: conventional baseline and headline comparison.

Conventional chain ("conventional — our construction"): fixed-sign SMO (k0) -> first-order
LPF -> Q-PLL (f_n=20); LPF cutoff UNPRINTED -> ASSUMPTION grid f_c in {100,200,400} Hz ×
phase compensation {off,on}; arctan-on-filtered-E as a second estimator. Handover uses the
generic criterion (no FLL).

Chains: conventional (6 cells), matched fixed sigmoid (nominal), FLC nominal, normalised
Q-PLL variant. Scenarios S1-S4, dead time OFF (primary) and ON (second).
Also: Step 1 Q-PLL normalisation ablation, Step 0d arm-1 re-eval with the generic
criterion.

Output: experiments/output/06c_comparison/comparison.json
"""

from __future__ import annotations

import dataclasses
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

import chain as C  # noqa: E402
import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters, wrap_angle_pi_closed  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.smo import SMOObserverParameters  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

TWO_PI = 2.0 * math.pi
DEAD = 2e-6
I_TH = 0.1


class ConventionalEstimator:
    """Fixed-sign SMO -> first-order LPF -> Q-PLL (or arctan on the filtered E)."""

    def __init__(self, stack, *, lpf_hz, compensate, estimator="qpll", f_n=20.0):
        self.observer = ConventionalSMO(
            SMOObserverParameters(stack.stator_resistance, stack.ld, stack.k0)
        )
        self.f_c = lpf_hz
        self.compensate = compensate
        self.estimator = estimator
        self.qpll = QPLL(
            QPLLParameters.from_design(zeta=0.707, f_n_hz=f_n, amplitude_v=stack.design_amp_v)
        )
        self._fa = 0.0
        self._fb = 0.0
        self.omega_hat = 0.0
        self.theta_hat = 0.0
        self._prev_theta = 0.0
        self.status = "ok"
        self.first_guard_fail_s = None
        self.last_e_mag = 0.0
        self.last_qpll_error = 0.0

    def __call__(self, sample, truth):  # noqa: ARG002
        s = self.observer.step(
            sample.u_alpha, sample.u_beta, sample.i_alpha, sample.i_beta, sample.dt
        )
        a = sample.dt / (1.0 / (TWO_PI * self.f_c) + sample.dt)
        self._fa += a * (s.back_emf_alpha - self._fa)
        self._fb += a * (s.back_emf_beta - self._fb)
        ea, eb = self._fa, self._fb
        omega = self.omega_hat if self.omega_hat > 0.0 else TWO_PI * 20.0
        if self.compensate:
            lead = math.atan2(omega, TWO_PI * self.f_c)
            c, sn = math.cos(lead), math.sin(lead)
            ea, eb = ea * c - eb * sn, ea * sn + eb * c
        self.last_e_mag = math.hypot(ea, eb)
        if self.estimator == "qpll":
            q = self.qpll.step(ea, eb, sample.dt)
            self.theta_hat, self.omega_hat = q.theta_hat, q.omega_hat
            self.last_qpll_error = float(self.qpll.last_error)
        else:
            th = wrap_angle_pi_closed(math.atan2(-ea, eb))
            self.omega_hat = wrap_angle_pi_closed(th - self._prev_theta) / sample.dt
            self._prev_theta = th
            self.theta_hat = th
            self.last_qpll_error = 0.0
        return self.theta_hat, self.omega_hat


def _rec(source, stack, generic):
    assump = C.handover_assumptions(stack)
    return E.ChainRecorder(source, pole_pairs=stack.pole_pairs, assumptions=assump, generic=generic)


def _run_chain(chain_name, builder, stack, params, plant, spec, *, advance, dead_time, generic):
    prm = dataclasses.replace(params, alpha_w_hz=spec["alpha_w"])
    source = builder()
    rec = _rec(source, stack, generic)
    res = run_drive(
        plant,
        prm,
        duration=spec["duration"],
        speed_ref_rpm=spec["speed_ref"],
        load_torque=spec["load"],
        source=rec,
        angle_advance=advance,
        dead_time_s=dead_time,
    )
    m = E.estimate_metrics(rec, source, res, dynamic=spec["dynamic"], steady=spec["steady"])
    if spec["disturbance_s"] is not None:
        m.update(E.speed_metrics(stack, res, {"disturbance_s": spec["disturbance_s"]}))
    m["chain"] = chain_name
    m["passed"] = bool(
        m["handed_over"]
        and m["fll_status"] == "ok"
        and m["peak_theta_err_deg"] is not None
        and m["peak_theta_err_deg"] < 30.0
        and m["saturation_after_transient"] == 0.0
    )
    return m


def _row(m):
    return {
        "handed_over": m["handed_over"],
        "switch_time_s": m["switch_time_s"],
        "fll_status": m["fll_status"],
        "peak_theta_err_deg": m["peak_theta_err_deg"],
        "steady_theta_mean_deg": m["steady_theta_err"]["mean"],
        "steady_theta_max_deg": m["steady_theta_err"]["max"],
        "dynamic_theta_max_deg": m["dynamic_theta_err"]["max"],
        "steady_omega_max_rpm": m["steady_omega_err"]["max"],
        "dynamic_omega_max_rpm": m["dynamic_omega_err"]["max"],
        "dip_rpm": m.get("dip_rpm"),
        "recovery_10rpm_s": m.get("recovery_10rpm_s"),
        "peak_i_q_a": m["peak_i_q_a"],
        "saturation_after_transient": m["saturation_after_transient"],
        "passed": m["passed"],
    }


def main() -> int:
    stack, params, plant, rule = E.load()
    scenarios = E.paper_scenarios(rule)
    cfg = config.load_config("drive_assumptions", directory=config.default_config_dir())

    def matched():
        return C.EstimatorSource(
            observer=C.build_observer("matched_sigmoid", stack),
            fll=C.build_fll(
                stack.initial_omega_at(C.handover_assumptions(stack).omega_h_rpm),
                min_omega=C.fll_min_omega_rad_s(stack),
            ),
            qpll=C.build_qpll(stack),
            pole_pairs=stack.pole_pairs,
        )

    def flc():
        return C.EstimatorSource(
            observer=C.build_observer("flc_nominal", stack),
            fll=C.build_fll(
                stack.initial_omega_at(C.handover_assumptions(stack).omega_h_rpm),
                min_omega=C.fll_min_omega_rad_s(stack),
            ),
            qpll=C.build_qpll(stack),
            pole_pairs=stack.pole_pairs,
        )

    def normalized():
        from pmsm_sensorless.estimation import QPLLParameters as P

        return C.EstimatorSource(
            observer=C.build_observer("matched_sigmoid", stack),
            fll=C.build_fll(
                stack.initial_omega_at(C.handover_assumptions(stack).omega_h_rpm),
                min_omega=C.fll_min_omega_rad_s(stack),
            ),
            qpll=QPLL(
                P.from_design(
                    zeta=0.707, f_n_hz=20.0, amplitude_v=stack.design_amp_v, normalize=True
                )
            ),
            pole_pairs=stack.pole_pairs,
        )

    chains = {"matched_nominal": matched, "flc_nominal": flc, "qpll_normalized": normalized}
    for fc in (100, 200, 400):
        for comp in (False, True):
            chains[f"conv_fc{fc}_{'comp' if comp else 'raw'}"] = lambda fc=fc, comp=comp: (
                ConventionalEstimator(stack, lpf_hz=float(fc), compensate=comp)
            )
    chains["conv_arctan_fc200_raw"] = lambda: ConventionalEstimator(
        stack, lpf_hz=200.0, compensate=False, estimator="arctan"
    )

    results = {}
    for dead_label, dt_s in (("deadtime_off", 0.0), ("deadtime_on", DEAD)):
        for name, builder in chains.items():
            for scen_name, spec in scenarios.items():
                key = f"{dead_label}|{name}|{scen_name}"
                m = _run_chain(
                    name,
                    builder,
                    stack,
                    params,
                    plant,
                    spec,
                    advance=False,
                    dead_time=dt_s,
                    generic=name.startswith("conv"),
                )
                results[key] = _row(m)

    # Step 1: Q-PLL normalisation ablation (normalization ON) at S2/S3/500-20
    ablation = {}
    s500_20 = dict(
        speed_ref=500.0,
        load=StepLoadTorque(0.0, 0.2 * E.RATED_TORQUE, 0.5),
        duration=2.0,
        alpha_w=rule[500],
        dynamic=(0.5, 1.0),
        steady=(1.8, 2.0),
        disturbance_s=0.5,
    )
    for aw in (5.0, 10.0, 20.0):
        for fn in (20.0, 40.0):
            for advance in (False, True):
                for scen_name, spec in (
                    ("S2", scenarios["S2_sim_50pct_1000"]),
                    ("S3", scenarios["S3_exp_20pct_1000"]),
                    ("S500_20", s500_20),
                ):
                    from pmsm_sensorless.estimation import QPLLParameters as P

                    def builder(aw=aw, fn=fn):
                        return C.EstimatorSource(
                            observer=C.build_observer("matched_sigmoid", stack),
                            fll=C.build_fll(
                                stack.initial_omega_at(C.handover_assumptions(stack).omega_h_rpm),
                                min_omega=C.fll_min_omega_rad_s(stack),
                            ),
                            qpll=QPLL(
                                P.from_design(
                                    zeta=0.707,
                                    f_n_hz=fn,
                                    amplitude_v=stack.design_amp_v,
                                    normalize=True,
                                )
                            ),
                            pole_pairs=stack.pole_pairs,
                        )

                    prm_spec = dict(spec)
                    prm_spec["alpha_w"] = aw
                    m = _run_chain(
                        "norm",
                        builder,
                        stack,
                        params,
                        plant,
                        prm_spec,
                        advance=advance,
                        dead_time=0.0,
                        generic=False,
                    )
                    ablation[f"aw{aw:g}|fn{fn:g}|adv{'on' if advance else 'off'}|{scen_name}"] = (
                        _row(m)
                    )

    # Step 0d: arm-1 (raw E_hat -> QPLL) re-eval with the generic criterion
    import importlib.util

    _s = importlib.util.spec_from_file_location("d06", _DIR / "06d_diagnosis.py")
    d06 = importlib.util.module_from_spec(_s)
    sys.modules["d06"] = d06
    _s.loader.exec_module(d06)
    arm1 = {}
    for aw in (2.5, 5.0, 10.0, 20.0, 40.0):
        for fn in (20.0, 40.0):
            for scen_name in ("S2_sim_50pct_1000", "S3_exp_20pct_1000"):
                spec = dict(scenarios[scen_name])
                spec["alpha_w"] = aw
                src0 = d06.make_source(
                    1,
                    stack,
                    f_n=fn,
                    initial_omega=stack.initial_omega_at(C.handover_assumptions(stack).omega_h_rpm),
                    min_omega=C.fll_min_omega_rad_s(stack),
                )
                rec = E.ChainRecorder(
                    src0,
                    pole_pairs=stack.pole_pairs,
                    assumptions=C.handover_assumptions(stack),
                    generic=True,
                )
                res = run_drive(
                    plant,
                    dataclasses.replace(params, alpha_w_hz=aw),
                    duration=spec["duration"],
                    speed_ref_rpm=spec["speed_ref"],
                    load_torque=spec["load"],
                    source=rec,
                )
                mm = E.estimate_metrics(
                    rec, src0, res, dynamic=spec["dynamic"], steady=spec["steady"]
                )
                arm1[f"aw{aw:g}|fn{fn:g}|{scen_name}"] = {
                    "handed_over": mm["handed_over"],
                    "switch_time_s": mm["switch_time_s"],
                    "fll_status": mm["fll_status"],
                    "peak_theta_err_deg": mm["peak_theta_err_deg"],
                    "saturation": mm["saturation_after_transient"],
                }

    out = {
        "note": (
            "Phase 06c comparison. Nominal chain = 06b-6 (v'->QPLL, f_n=20, matched, "
            "alpha_w*=10, advance off). Conventional = our construction (sign SMO -> LPF "
            "-> QPLL); LPF cutoff ASSUMPTION. Dead time is an average-value model "
            "(V_dc=311, t_d=2us, f_sw=10kHz, i_th=0.1 A). No reproduction claim."
        ),
        "dead_time": {"v_dc": 311.0, "t_d_s": DEAD, "f_sw_hz": 10000.0, "i_th_a": I_TH},
        "comparison": results,
        "normalization_ablation": ablation,
        "arm1_generic": arm1,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06c_comparison"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "comparison.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== primary (dead time OFF): matched nominal vs conventional / FLC / normalized ==")
    for key, r in results.items():
        if not key.startswith("deadtime_off"):
            continue
        print(
            f"  {key[12:]:46s} hand={r['handed_over']} fll={r['fll_status']:9s} "
            f"peak={r['peak_theta_err_deg'] if r['peak_theta_err_deg'] is not None else -1:7.1f} "
            f"steady_dth={r['steady_theta_mean_deg'] if r['steady_theta_mean_deg'] is not None else -1:6.2f} "
            f"dyn_dth={r['dynamic_theta_max_deg'] if r['dynamic_theta_max_deg'] is not None else -1:7.1f} "
            f"dyn_dw={r['dynamic_omega_max_rpm'] if r['dynamic_omega_max_rpm'] is not None else -1:8.1f} "
            f"{'PASS' if r['passed'] else 'FAIL'}"
        )
    print("\n== arm1 raw->QPLL with generic criterion ==")
    for k, v in arm1.items():
        print(
            f"  {k:40s} hand={v['handed_over']} switch={v['switch_time_s']} fll={v['fll_status']} peak={v['peak_theta_err_deg']}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
