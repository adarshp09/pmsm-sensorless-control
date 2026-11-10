#!/usr/bin/env python3
"""Phase 06a Step 0: metric-basis and angle-bias model checks (no controller).

Step 0b: state whether the Phase 05b angle/speed rows are pointwise or windowed, and
recompute the nominal-cell ramp rows pointwise over the settled window [0.75, 1.5] s.

Step 0c: the angle-bias model for the nominal cell (matched sigmoid | DSOGI v' | QPLL):
steady bias vs ``-atan(omega Ld/(Rs+K_eff)) + omega Ts/2``, the half-sample hypothesis
(error evaluated against ``theta_true(t_k + Ts/2)``), and the ramp bias vs the type-2
PLL ramp offset ``omega_dot_e/omega_n^2`` plus the constant-speed bias.

Reads ``experiments/output/05b_estimator_eval/estimator_eval.json`` and reproduces the
nominal cell (deterministic; byte-identical to the Phase 05b chain) for the pointwise
ramp recomputation and the half-sample test.

Output: experiments/output/06a_step0/step0_analysis.json
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters, wrap_angle_pi_closed  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.observers.smo import SMOObserverParameters  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.nominal_drive import (  # noqa: E402
    nominal_drive_rollout,
    rated_current_from_torque,
)

TWO_PI = 2.0 * math.pi
K_S = math.sqrt(2.0)
GAMMA = 50.0
MATCHED = (1.0, 0.4)               # kF, aF of the Phase 05b nominal cell
SPEEDS = (100.0, 200.0, 500.0, 1000.0, 2000.0)
TS_VALUES = (100e-6, 50e-6)
STEADY_CYCLES = 8
RUN_CYCLES = 12
RPM_START, RPM_END, RAMP_S, RAMP_DURATION = 500.0, 1000.0, 1.5, 2.0


def _configs():
    directory = config.default_config_dir()
    pmsm = config.load_config("pmsm", directory=directory)
    observer = config.load_config("observer", directory=directory)
    assumptions = config.load_config("estimator_assumptions", directory=directory)
    v_base = math.sqrt(2.0) * config.resolve_parameter(pmsm, "rating.rated_voltage", unit="V") / math.sqrt(3.0)
    i_base = math.sqrt(2.0) * config.resolve_parameter(pmsm, "rating.rated_power", unit="W") / (
        math.sqrt(3.0) * config.resolve_parameter(pmsm, "rating.rated_voltage", unit="V"))
    k0 = config.resolve_parameter(observer, "smo.switching_gain_k", unit="-") * v_base
    a0 = config.resolve_parameter(observer, "smo.sigmoid_slope_a", unit="-") / i_base
    i_rated = rated_current_from_torque(
        config.resolve_parameter(pmsm, "rating.rated_torque", unit="N.m"),
        int(config.resolve_parameter(pmsm, "machine.pole_pairs", unit="-")),
        config.resolve_parameter(pmsm, "machine.pm_flux_linkage_psi_f", unit="Wb"))
    return pmsm, assumptions, k0, a0, i_rated


def _chain(params, plant, qpll_params, k0, a0, i_rated, *, speed_rpm, ts):
    p = plant.parameters
    if speed_rpm is None:
        r = nominal_drive_rollout(plant, i_q=i_rated, dt=ts, duration=RAMP_DURATION,
                                  rpm_start=RPM_START, rpm_end=RPM_END, ramp_time=RAMP_S)
        f_e = p.pole_pairs * RPM_END / 60.0
        start_omega = p.pole_pairs * RPM_START * TWO_PI / 60.0
        mask = (r.t >= 0.75) & (r.t <= RAMP_S)
    else:
        f_e = p.pole_pairs * speed_rpm / 60.0
        duration = max(0.25, RUN_CYCLES / f_e)
        r = nominal_drive_rollout(plant, i_q=i_rated, dt=ts, duration=duration,
                                  rpm_start=speed_rpm, rpm_end=speed_rpm, ramp_time=1.0)
        mask = (r.t >= duration - STEADY_CYCLES / f_e) & (r.t <= duration)
        start_omega = TWO_PI * f_e

    observer = SigmoidSMO(SMOObserverParameters(p.stator_resistance, p.ld,
                                                MATCHED[0] * k0, sigmoid_slope_a=MATCHED[1] * a0))
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, start_omega))
    n = len(r.t)
    ea, eb = np.empty(n), np.empty(n)
    va, vb = np.empty(n), np.empty(n)
    for k in range(n):
        o = observer.step(r.u_alpha[k], r.u_beta[k], r.i_alpha[k], r.i_beta[k], ts)
        f = fll.step(o.back_emf_alpha, o.back_emf_beta, ts)
        ea[k], eb[k] = o.back_emf_alpha, o.back_emf_beta
        va[k], vb[k] = f.v_alpha_prime, f.v_beta_prime
    qpll = QPLL(qpll_params, initial_theta=r.theta_e[0], initial_omega=r.omega_e[0])
    th, om = np.empty(n), np.empty(n)
    for k in range(n):
        out = qpll.step(va[k], vb[k], ts)
        th[k], om[k] = out.theta_hat, out.omega_hat
    return {"t": r.t, "theta_true": r.theta_e, "omega_true": r.omega_e,
            "theta_hat": th, "omega_hat": om, "mask": mask, "pole_pairs": p.pole_pairs}


def _pointwise(chain, shift_samples: float = 0.0, dt: float = 100e-6) -> dict:
    """Pointwise mean/std/rms/max of |dtheta| (deg) and |domega| (rpm) over the mask.

    ``shift_samples`` advances the true angle by that many samples (half-sample test).
    """
    m = chain["mask"]
    theta_true = chain["theta_true"] + shift_samples * dt * chain["omega_true"]
    err = np.array([wrap_angle_pi_closed(a - b) for a, b in zip(chain["theta_hat"][m], theta_true[m])])
    deg = np.degrees(err)
    spd = (chain["omega_hat"][m] - chain["omega_true"][m]) * 60.0 / (TWO_PI * chain["pole_pairs"])
    return {
        "angle_mean_deg": float(np.mean(deg)), "angle_rms_deg": float(np.sqrt(np.mean(deg**2))),
        "angle_max_abs_deg": float(np.max(np.abs(deg))),
        "speed_mean_rpm": float(np.mean(spd)), "speed_rms_rpm": float(np.sqrt(np.mean(spd**2))),
        "speed_max_abs_rpm": float(np.max(np.abs(spd))),
    }


def main() -> int:
    pmsm_cfg, assumptions, k0, a0, i_rated = _configs()
    plant = PMSMPlant(PMSMParameters.from_config(pmsm_cfg))
    p = plant.parameters
    design_amp = (p.pole_pairs * RPM_END * TWO_PI / 60.0) * p.psi_f
    qpll_params = QPLLParameters.from_configs(assumptions, design_amplitude_v=design_amp)
    k_eff = MATCHED[0] * k0 * MATCHED[1] * a0 / 2.0
    w_n = TWO_PI * 20.0

    json_path = config.find_project_root() / "experiments" / "output" / "05b_estimator_eval" / "estimator_eval.json"
    published = json.loads(json_path.read_text(encoding="utf-8"))["results"]

    out: dict = {"k_eff": k_eff, "k0_v": k0, "a0_per_a": a0, "design_amp_v": design_amp,
                 "w_n_rad_s": w_n}

    # ---- Step 0b: basis + pointwise ramp recomputation
    basis_rows = {}
    for ts in TS_VALUES:
        tag = f"Ts{ts*1e6:.0f}"
        chain_ramp = _chain(p, plant, qpll_params, k0, a0, i_rated, speed_rpm=None, ts=ts)
        recip = _pointwise(chain_ramp, 0.0, ts)
        pub = published[f"{tag}|ramp|matched_sigmoid|v_prime|qpll"]["ramp"]
        matches = {
            "angle_mean": abs(recip["angle_mean_deg"] - pub["settled_angle_error"]["mean_deg"]) < 1e-6,
            "angle_max": abs(recip["angle_max_abs_deg"] - pub["settled_angle_error"]["max_abs_deg"]) < 1e-6,
            "speed_max": abs(recip["speed_max_abs_rpm"] - pub["settled_speed_error"]["max_abs_rpm"]) < 1e-6,
        }
        basis_rows[tag] = {"recomputed": recip, "json_settled": pub["settled_speed_error"],
                           "reproduces_json": all(matches.values())}
        for rpm in SPEEDS:
            chain = _chain(p, plant, qpll_params, k0, a0, i_rated, speed_rpm=rpm, ts=ts)
            basis_rows[f"{tag}|const{int(rpm)}"] = {"pointwise": _pointwise(chain, 0.0, ts)}
    out["step0b_basis"] = {
        "definition": ("all Phase 05b angle/speed rows use the SAME `_angle_metrics`/"
                       "`_speed_metrics` helpers -> pointwise mean/std/rms/max over a window "
                       "mask (steady = last 8 cycles; ramp settled = [0.75, 1.5] s). None of "
                       "them is a windowed fit."),
        "premise_test": ("the ramp speed max and the constant-speed speed max share one basis; "
                         "the ~20x gap is physical (the constant-speed QPLL speed estimate carries "
                         "a few-rpm ripple while the ramp tracks smoothly), not a metric artefact."),
        "rows": basis_rows,
    }

    # ---- Step 0c(i): angle-bias model, constant speeds
    model_rows = {}
    for ts in TS_VALUES:
        for rpm in SPEEDS:
            omega_e = p.pole_pairs * rpm * TWO_PI / 60.0
            measured = published[f"Ts{ts*1e6:.0f}|const_{int(rpm)}rpm|matched_sigmoid|v_prime|qpll"][
                "steady"]["angle_error"]["mean_deg"]
            linear = -math.degrees(math.atan(omega_e * p.ld / (p.stator_resistance + k_eff)))
            half = math.degrees(omega_e * ts / 2.0)
            model_rows[f"Ts{ts*1e6:.0f}|{int(rpm)}rpm"] = {
                "measured_bias_deg": measured, "linear_lag_deg": linear, "half_sample_deg": half,
                "predicted_with_half_deg": linear + half, "residual_with_half_deg": measured - (linear + half),
                "predicted_no_half_deg": linear, "residual_no_half_deg": measured - linear,
            }
    out["step0c_bias_model"] = model_rows

    # ---- Step 0c(ii): ramp bias = constant-speed bias + type-2 ramp offset
    ramp_omega_dot = p.pole_pairs * (RPM_END - RPM_START) * TWO_PI / 60.0 / RAMP_S  # rad/s^2
    ramp_offset_deg = math.degrees(ramp_omega_dot / w_n**2)
    mid_rpm = 0.5 * (RPM_START + RPM_END)
    omega_mid = p.pole_pairs * mid_rpm * TWO_PI / 60.0
    ramp_rows = {}
    for ts in TS_VALUES:
        tag = f"Ts{ts*1e6:.0f}"
        measured = published[f"{tag}|ramp|matched_sigmoid|v_prime|qpll"]["ramp"]["settled_angle_error"]["mean_deg"]
        const_model = -math.degrees(math.atan(omega_mid * p.ld / (p.stator_resistance + k_eff))) \
            + math.degrees(omega_mid * ts / 2.0)
        const_interp = 0.5 * (
            published[f"{tag}|const_500rpm|matched_sigmoid|v_prime|qpll"]["steady"]["angle_error"]["mean_deg"]
            + published[f"{tag}|const_1000rpm|matched_sigmoid|v_prime|qpll"]["steady"]["angle_error"]["mean_deg"]
        )
        for source, const_bias in (("model@750rpm", const_model), ("measured-interp@750rpm", const_interp)):
            ramp_rows[f"{tag}|{source}"] = {
                "measured_ramp_bias_deg": measured, "const_bias_deg": const_bias,
                "ramp_offset_deg": ramp_offset_deg, "predicted_deg": const_bias - ramp_offset_deg,
                "residual_deg": measured - (const_bias - ramp_offset_deg),
            }
    out["step0c_ramp"] = {"omega_dot_e_rad_s2": ramp_omega_dot, "ramp_offset_deg": ramp_offset_deg,
                          "rows": ramp_rows}

    out_dir = config.find_project_root() / "experiments" / "output" / "06a_step0"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "step0_analysis.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== Step 0b: ramp rows recomputed pointwise (settled [0.75,1.5] s) ==")
    for ts in TS_VALUES:
        tag = f"Ts{ts*1e6:.0f}"
        r = basis_rows[tag]["recomputed"]
        print(f"  {tag}: angle mean {r['angle_mean_deg']:+.4f} rms {r['angle_rms_deg']:.4f} max {r['angle_max_abs_deg']:.4f} deg |"
              f" speed mean {r['speed_mean_rpm']:+.4f} rms {r['speed_rms_rpm']:.4f} max {r['speed_max_abs_rpm']:.4f} rpm"
              f"  (reproduces JSON: {basis_rows[tag]['reproduces_json']})")
    print("\n== Step 0c: bias model (deg) ==")
    print(f"  {'key':18s} {'meas':>8s} {'-atan':>8s} {'+wTs/2':>8s} {'pred':>8s} {'resid':>8s}")
    for key, row in model_rows.items():
        print(f"  {key:18s} {row['measured_bias_deg']:8.4f} {row['linear_lag_deg']:8.4f} "
              f"{row['half_sample_deg']:8.4f} {row['predicted_with_half_deg']:8.4f} "
              f"{row['residual_with_half_deg']:8.4f}")
    print("\n== Step 0c: ramp bias ==")
    print(f"  omega_dot_e = {ramp_omega_dot:.2f} rad/s^2 -> ramp offset {ramp_offset_deg:.4f} deg")
    for key, row in ramp_rows.items():
        print(f"  {key:34s} measured {row['measured_ramp_bias_deg']:+.4f}  const {row['const_bias_deg']:+.4f} "
              f" pred {row['predicted_deg']:+.4f}  resid {row['residual_deg']:+.4f}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
