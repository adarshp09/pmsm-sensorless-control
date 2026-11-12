#!/usr/bin/env python3
"""Phase 05b: open-loop position/speed estimator evaluation.

Chain (composed here only): observer -> filter -> estimator.
- observers: fixed sign; matched fixed sigmoid (kF=1, aF=0.4); FLC nominal.
- filter: none | DSOGI-FLL v' (paper-literal, PRIMARY) | PNS (non-paper, labelled;
  matched and FLC cells only).
- estimator: arctan (Eq.(8)) | QPLL (Fig.1).
Scenario: nominal drive (`simulation/nominal_drive`), I_q = I_rated. Constant
speeds {100,200,500,1000,2000} rpm and the 500->1000 rpm ramp. Ts in {100, 50} us.

FLL at library defaults (epsilon_filter_tau=OFF, no clamps, no normalization);
initial omega' = true electrical frequency at the window start.
QPLL ASSUMPTIONS: zeta=0.707, f_n=20 Hz designed at |E|(1000 rpm), normalization
off, theta_hat(0)=theta_true(0), no phase compensation. Every non-default option
and assumption is stated in the JSON.

RAW E_hat only; the estimator sees only the alpha-beta back-EMF and dt.
Output: experiments/output/05b_estimator_eval/estimator_eval.json
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

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import config_diff  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters, wrap_angle_pi_closed  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
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
    sliding_window_ramp_metrics,
    summarize_ramp,
    vector_metrics,
)

K_S = math.sqrt(2.0)
GAMMA = 50.0
RPM_START, RPM_END, RAMP_S, RAMP_DURATION = 500.0, 1000.0, 1.5, 2.0
STEADY_CYCLES = 8
RUN_TARGET_CYCLES = 12
MIN_DURATION = 0.25
LOCK_IN_THRESHOLD_DEG = 5.0  # >= the per-sample angle increment at 2000 rpm / 100 us (4.8 deg)
SPEEDS = (100.0, 200.0, 500.0, 1000.0, 2000.0)
MATCHED = (1.0, 0.4)
TS_VALUES = (100e-6, 50e-6)
F_N_MULTIPLIERS = (0.1, 1.0, 10.0)
TWO_PI = 2.0 * math.pi
OBSERVERS = ("sign", "matched_sigmoid", "flc_nominal")
FILTERS = {
    "sign": ("none", "v_prime"),
    "matched_sigmoid": ("none", "v_prime", "pns"),
    "flc_nominal": ("none", "v_prime", "pns"),
}


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rollout(observer, u, i, dt):
    n = len(u)
    out = {k: np.empty(n) for k in ("ea", "eb")}
    for k in range(n):
        r = observer.step(u[k, 0], u[k, 1], i[k, 0], i[k, 1], dt)
        out["ea"][k], out["eb"][k] = r.back_emf_alpha, r.back_emf_beta
    return out


def _fll_run(ea, eb, dt, initial_omega):
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, initial_omega))
    n = len(ea)
    freq = np.full(n, np.nan)
    vpa = np.full(n, np.nan)
    vpb = np.full(n, np.nan)
    pna = np.full(n, np.nan)
    pnb = np.full(n, np.nan)
    try:
        for k in range(n):
            o = fll.step(float(ea[k]), float(eb[k]), dt)
            freq[k] = o.frequency_hz
            vpa[k], vpb[k] = o.v_alpha_prime, o.v_beta_prime
            pna[k], pnb[k] = o.v_alpha, o.v_beta
    except ValueError as exc:
        return {
            "freq": freq,
            "v_prime": (vpa, vpb),
            "pns": (pna, pnb),
            "status": "guard-fail",
            "error": str(exc),
        }
    return {
        "freq": freq,
        "v_prime": (vpa, vpb),
        "pns": (pna, pnb),
        "status": "locked",
        "error": None,
    }


def _fll_status(freq, f_true):
    if np.isnan(freq).any():
        return "guard-fail"
    m = float(np.mean(freq)) if len(freq) else float("nan")
    if not np.isfinite(m) or m < 0.2 * f_true:
        return "collapsed"
    return "locked"


def _estimator_run(kind, params, ea, eb, dt, theta_true0, *, theta0=None, omega0=0.0):
    n = len(ea)
    theta = np.empty(n)
    omega = np.full(n, np.nan)
    if kind == "arctan":
        for k in range(n):
            th = wrap_angle_pi_closed(math.atan2(-float(ea[k]), float(eb[k])))
            theta[k] = th
        return theta, None
    qpll = QPLL(
        params, initial_theta=(theta_true0 if theta0 is None else theta0), initial_omega=omega0
    )
    for k in range(n):
        o = qpll.step(float(ea[k]), float(eb[k]), dt)
        theta[k], omega[k] = o.theta_hat, o.omega_hat
    return theta, omega


def _angle_metrics(theta_hat, theta_true, mask, pole_pairs):
    err = np.array([wrap_angle_pi_closed(a - b) for a, b in zip(theta_hat[mask], theta_true[mask])])
    deg = np.degrees(err)
    return {
        "mean_deg": float(np.mean(deg)),
        "std_deg": float(np.std(deg)),
        "rms_deg": float(np.sqrt(np.mean(deg**2))),
        "max_abs_deg": float(np.max(np.abs(deg))),
        "mech_mean_deg": float(np.mean(deg) / pole_pairs),
        "mech_max_abs_deg": float(np.max(np.abs(deg)) / pole_pairs),
    }


def _speed_metrics(omega_hat, omega_true, mask, pole_pairs):
    if omega_hat is None:
        return None
    scale = 60.0 / (TWO_PI * pole_pairs)  # electrical rad/s -> mechanical rpm
    err = (omega_hat[mask] - omega_true[mask]) * scale
    return {
        "mean_rpm": float(np.mean(err)),
        "std_rpm": float(np.std(err)),
        "max_abs_rpm": float(np.max(np.abs(err))),
    }


def _lock_in_time(theta_hat, theta_true, t, limit_deg=1.0):
    """First time after which |angle error| stays below ``limit_deg``."""
    bad = np.where(
        np.abs(np.degrees([wrap_angle_pi_closed(a - b) for a, b in zip(theta_hat, theta_true)]))
        >= limit_deg
    )[0]
    idx = 0 if len(bad) == 0 else (bad[-1] + 1)
    return float(t[idx]) if idx < len(t) else None


def _phasor_phase_deg(a, b, theta, mask):
    z = a[mask] + 1j * b[mask]
    p = np.mean(z * np.exp(-1j * theta[mask]))
    return float(np.degrees(np.angle(p)))


def main() -> int:
    _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    assumptions_cfg = config.load_config("estimator_assumptions", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    base_engine = FuzzyEngine.from_yaml(directory)

    v_base = (
        math.sqrt(2.0)
        * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V")
        / math.sqrt(3.0)
    )
    i_base = (
        math.sqrt(2.0)
        * config.resolve_parameter(pmsm_cfg, "rating.rated_power", unit="W")
        / (math.sqrt(3.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V"))
    )
    k0 = config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-") * v_base
    a0 = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / i_base
    i_rated = rated_current_from_torque(
        config.resolve_parameter(pmsm_cfg, "rating.rated_torque", unit="N.m"),
        int(config.resolve_parameter(pmsm_cfg, "machine.pole_pairs", unit="-")),
        config.resolve_parameter(pmsm_cfg, "machine.pm_flux_linkage_psi_f", unit="Wb"),
    )
    p_pairs = parameters.pole_pairs
    rd, ld = parameters.stator_resistance, parameters.ld
    f_n_nom = config.resolve_parameter(assumptions_cfg, "qpll.natural_frequency_hz", unit="Hz")
    zeta = config.resolve_parameter(assumptions_cfg, "qpll.damping_zeta", unit="-")
    design_amp = (p_pairs * RPM_END * TWO_PI / 60.0) * parameters.psi_f
    qpll_params = QPLLParameters.from_configs(assumptions_cfg, design_amplitude_v=design_amp)

    def make_observer(kind):
        if kind == "sign":
            return ConventionalSMO.from_config(pmsm_cfg, observer_cfg)
        if kind == "matched_sigmoid":
            return SigmoidSMO(
                SMOObserverParameters(rd, ld, MATCHED[0] * k0, sigmoid_slope_a=MATCHED[1] * a0)
            )
        return FuzzySMO(
            SMOObserverParameters.from_config(pmsm_cfg, observer_cfg),
            FuzzyEngine(base_engine.parameters),
        )

    def scenario(speed_rpm, ts):
        if speed_rpm is None:  # ramp
            r = nominal_drive_rollout(
                plant,
                i_q=i_rated,
                dt=ts,
                duration=RAMP_DURATION,
                rpm_start=RPM_START,
                rpm_end=RPM_END,
                ramp_time=RAMP_S,
            )
            f_e = p_pairs * RPM_END / 60.0
            start_omega = p_pairs * RPM_START * TWO_PI / 60.0
            return r, "ramp", f_e, start_omega, (r.t >= RAMP_S * 0) & (r.t <= RAMP_S)
        f_e = p_pairs * speed_rpm / 60.0
        duration = max(MIN_DURATION, RUN_TARGET_CYCLES / f_e)
        r = nominal_drive_rollout(
            plant,
            i_q=i_rated,
            dt=ts,
            duration=duration,
            rpm_start=speed_rpm,
            rpm_end=speed_rpm,
            ramp_time=1.0,
        )
        period = 1.0 / f_e
        return (
            r,
            f"const_{int(speed_rpm)}rpm",
            f_e,
            TWO_PI * f_e,
            (r.t >= duration - STEADY_CYCLES * period),
        )

    results: dict = {}
    summary_rows = []
    for ts in TS_VALUES:
        for speed in (*SPEEDS, None):
            r, label, f_e, initial_omega, mask = scenario(speed, ts)
            theta_true = r.theta_e
            omega_true = r.omega_e
            u = np.column_stack([r.u_alpha, r.u_beta])
            i = np.column_stack([r.i_alpha, r.i_beta])
            steady = label != "ramp"
            for obs_kind in OBSERVERS:
                sm = _rollout(make_observer(obs_kind), u, i, ts)
                raw = (sm["ea"], sm["eb"])
                fll = _fll_run(raw[0], raw[1], ts, initial_omega)
                fll_status = (
                    _fll_status(fll["freq"], f_e) if steady else _fll_status(fll["freq"], f_e)
                )
                for filt in FILTERS[obs_kind]:
                    if filt == "none":
                        fa, fb = raw
                        status = "n/a"
                    elif filt == "v_prime":
                        fa, fb = fll["v_prime"]
                        status = fll_status
                    else:
                        fa, fb = fll["pns"]
                        status = fll_status
                    for est in ("arctan", "qpll"):
                        theta_hat, omega_hat = _estimator_run(
                            est, qpll_params, fa, fb, ts, theta_true[0], omega0=omega_true[0]
                        )
                        key = f"Ts{ts * 1e6:.0f}|{label}|{obs_kind}|{filt}|{est}"
                        entry = {
                            "ts_us": ts * 1e6,
                            "scenario": label,
                            "observer": obs_kind,
                            "filter": filt,
                            "estimator": est,
                            "fll_status": status,
                            "i_q_a": i_rated,
                        }
                        if steady:
                            lock = None
                            if est == "qpll":
                                th90, _ = _estimator_run(
                                    "qpll",
                                    qpll_params,
                                    fa,
                                    fb,
                                    ts,
                                    theta_true[0],
                                    theta0=theta_true[0] + math.pi / 2.0,
                                    omega0=omega_true[0],
                                )
                                lock = _lock_in_time(th90, theta_true, r.t, LOCK_IN_THRESHOLD_DEG)
                            entry["steady"] = {
                                "angle_error": _angle_metrics(theta_hat, theta_true, mask, p_pairs),
                                "speed_error": _speed_metrics(omega_hat, omega_true, mask, p_pairs),
                                "lock_in_s": lock,
                                "lock_in_threshold_deg": LOCK_IN_THRESHOLD_DEG,
                            }
                            entry["budget"] = _angle_budget(
                                raw,
                                (fa, fb),
                                (r.e_alpha, r.e_beta),
                                theta_hat,
                                theta_true,
                                mask,
                                p_pairs,
                            )
                            if obs_kind == "matched_sigmoid":
                                k_eff = MATCHED[0] * k0 * MATCHED[1] * a0 / 2.0
                                entry["budget"]["linear_observer_lag_deg"] = -math.degrees(
                                    math.atan(omega_true[0] * ld / (rd + k_eff))
                                )
                                entry["budget"]["linear_lag_k_eff"] = k_eff
                        else:
                            ramp_mask = (r.t >= 0.0) & (r.t <= RAMP_S)
                            settled = (r.t >= 0.75) & (r.t <= RAMP_S)
                            entry["ramp"] = {
                                "angle_error": _angle_metrics(
                                    theta_hat, theta_true, ramp_mask, p_pairs
                                ),
                                "speed_error": _speed_metrics(
                                    omega_hat, omega_true, ramp_mask, p_pairs
                                ),
                                "settled_angle_error": _angle_metrics(
                                    theta_hat, theta_true, settled, p_pairs
                                ),
                                "settled_speed_error": _speed_metrics(
                                    omega_hat, omega_true, settled, p_pairs
                                ),
                                "settled_window_s": [0.75, RAMP_S],
                                "windowed": summarize_ramp(
                                    sliding_window_ramp_metrics(
                                        fa[ramp_mask],
                                        fb[ramp_mask],
                                        r.e_alpha[ramp_mask],
                                        r.e_beta[ramp_mask],
                                        fll["freq"][ramp_mask]
                                        if not np.isnan(fll["freq"][ramp_mask]).any()
                                        else f_e * np.ones(int(ramp_mask.sum())),
                                        f_e * np.ones(int(ramp_mask.sum())),
                                        r.t[ramp_mask],
                                        window_s=0.06,
                                    )
                                ),
                            }
                        results[key] = entry
                        summary_rows.append(entry)

    # QPLL f_n sensitivity: matched cell, v_prime, 1000 rpm + ramp
    sensitivity = {}
    for ts in TS_VALUES:
        for multiplier in F_N_MULTIPLIERS:
            params = QPLLParameters.from_design(
                zeta=zeta, f_n_hz=multiplier * f_n_nom, amplitude_v=design_amp, normalize=False
            )
            for speed in (1000.0, None):
                r, label, f_e, initial_omega, mask = scenario(speed, ts)
                u = np.column_stack([r.u_alpha, r.u_beta])
                i = np.column_stack([r.i_alpha, r.i_beta])
                sm = _rollout(make_observer("matched_sigmoid"), u, i, ts)
                fll = _fll_run(sm["ea"], sm["eb"], ts, initial_omega)
                fa, fb = fll["v_prime"]
                theta_hat, omega_hat = _estimator_run(
                    "qpll", params, fa, fb, ts, r.theta_e[0], omega0=r.omega_e[0]
                )
                m = mask if label != "ramp" else ((r.t >= 0.0) & (r.t <= RAMP_S))
                sensitivity[f"Ts{ts * 1e6:.0f}|{label}|fn{multiplier:g}x"] = _angle_metrics(
                    theta_hat, r.theta_e, m, p_pairs
                )

    output_dir = config.find_project_root() / "experiments" / "output" / "05b_estimator_eval"
    output_dir.mkdir(parents=True, exist_ok=True)
    diff = config_diff.summary(
        "05b_estimator_eval",
        {
            "epsilon_filter_tau": None,
            "min_omega": None,
            "max_omega": None,
            "normalize_error": False,
            "positive_sequence": True,
        },
        scenario={
            "ts_us": [t * 1e6 for t in TS_VALUES],
            "speeds_rpm": list(SPEEDS),
            "ramp": [RPM_START, RPM_END],
            "i_q_a": i_rated,
            "qpll": {
                "zeta": zeta,
                "f_n_hz": f_n_nom,
                "design_amp_v": design_amp,
                "normalize": False,
                "initial_theta": "theta_true(0)",
                "initial_omega": "theta_true(0) electrical frequency (DERIVED from the prescribed profile)",
            },
        },
    )
    out = {
        "note": (
            "Phase 05b open-loop estimator evaluation; nominal drive, RAW E_hat; "
            "FLL library defaults; QPLL assumptions zeta=0.707, f_n=20Hz@|E|(1000rpm), "
            "normalization off, theta_hat(0)=theta_true(0). No tuning."
        ),
        "assumptions": {
            "qpll_zeta": zeta,
            "qpll_f_n_hz": f_n_nom,
            "design_amp_v": design_amp,
            "qpll_kp": qpll_params.kp,
            "qpll_ki": qpll_params.ki,
            "normalize": qpll_params.normalize,
        },
        "k0_v": k0,
        "a0_per_a": a0,
        "i_rated_a": i_rated,
        "config_diff": diff,
        "results": results,
        "qpll_sensitivity": sensitivity,
        "paper_comparison": _paper_comparison(results, p_pairs),
    }
    (output_dir / "estimator_eval.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    config_diff.emit(diff)
    _print_nominal(results)
    print(f"\nOutputs written to: {output_dir}")
    return 0


def _angle_budget(raw, filtered, true_e, theta_hat, theta_true, mask, pole_pairs):
    """measured bias vs [raw phase] + [filter phase] + [estimator lag], all vs E_true."""
    fa, fb = filtered
    ref = _phasor_phase_deg(true_e[0], true_e[1], theta_true, mask)
    raw_phase = _phasor_phase_deg(raw[0], raw[1], theta_true, mask) - ref
    filt_phase = _phasor_phase_deg(fa, fb, theta_true, mask) - ref
    filter_delta = filt_phase - raw_phase
    theta_filt = np.array([math.atan2(-float(a), float(b)) for a, b in zip(fa, fb)])
    lag = np.mean(
        [
            math.degrees(wrap_angle_pi_closed(a - b))
            for a, b in zip(theta_hat[mask], theta_filt[mask])
        ]
    )
    measured = np.mean(
        [
            math.degrees(wrap_angle_pi_closed(a - b))
            for a, b in zip(theta_hat[mask], theta_true[mask])
        ]
    )
    return {
        "raw_phase_deg": raw_phase,
        "filter_delta_deg": filter_delta,
        "estimator_lag_deg": float(lag),
        "measured_bias_deg": float(measured),
        "residual_deg": float(measured - (raw_phase + filter_delta + lag)),
    }


def _print_nominal(results):
    print("== nominal cell (matched_sigmoid | v_prime | qpll) steady rows ==")
    for key in sorted(k for k in results if "matched_sigmoid|v_prime|qpll" in k and "const" in k):
        e = results[key]
        a = e["steady"]["angle_error"]
        sp = e["steady"]["speed_error"]
        print(
            f"  {e['scenario']:14s} Ts{e['ts_us']:3.0f}us: ang mean {a['mean_deg']:+6.2f} max {a['max_abs_deg']:5.2f} deg | "
            f"spd max {sp['max_abs_rpm']:6.2f} rpm | fll {e['fll_status']}"
        )


def _paper_comparison(results, pole_pairs):
    """Class-A paper values vs ours (no tuning); setup differences stated."""
    rows = []
    for key, e in results.items():
        if (
            e["scenario"] == "ramp"
            and e["observer"] == "matched_sigmoid"
            and e["filter"] == "v_prime"
            and e["estimator"] == "qpll"
        ):
            rows.append(
                {
                    "scenario": "no-load variable speed (paper sim 500->1000; exp 500->1000)",
                    "ours_angle_max_deg": e["ramp"]["settled_angle_error"]["max_abs_deg"],
                    "ours_angle_mean_deg": e["ramp"]["settled_angle_error"]["mean_deg"],
                    "ours_speed_max_rpm": None
                    if e["ramp"]["settled_speed_error"] is None
                    else e["ramp"]["settled_speed_error"]["max_abs_rpm"],
                    "paper_sim": "max speed ~5 rpm, steady pos ~0.07 rad, dynamic pos ~0.2 rad",
                    "paper_exp": "max speed 15 rpm, pos 0.1256 rad @500; 10 rpm, 0.08 rad @1000",
                    "setup_diff": (
                        "open-loop ideal-FOC feedforward vs paper closed-loop speed/position "
                        "control; our E_hat is the matched sigmoid SMO, paper uses FSMO; our "
                        "assumed QPLL gains/bandwidth; ideal voltage, no dead time/PWM"
                    ),
                }
            )
    for key, e in results.items():
        if (
            e["scenario"] == "const_1000rpm"
            and e["observer"] == "matched_sigmoid"
            and e["filter"] == "v_prime"
            and e["estimator"] == "qpll"
        ):
            rows.append(
                {
                    "scenario": "steady 1000 rpm (paper Table IV)",
                    "ours_angle_max_deg": e["steady"]["angle_error"]["max_abs_deg"],
                    "ours_angle_mean_deg": e["steady"]["angle_error"]["mean_deg"],
                    "ours_speed_max_rpm": None
                    if e["steady"]["speed_error"] is None
                    else e["steady"]["speed_error"]["max_abs_rpm"],
                    "paper_sim": "0.07 rad",
                    "paper_exp": "0.08 rad, 10 rpm (Table IV)",
                    "setup_diff": (
                        "open-loop feedforward vs closed loop; assumed QPLL gains; no dead time"
                    ),
                }
            )
    return rows


if __name__ == "__main__":
    raise SystemExit(main())
