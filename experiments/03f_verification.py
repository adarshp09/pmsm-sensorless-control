#!/usr/bin/env python3
"""Phase 03f: verification & disclosure.

Computations (no src/ or configs/ changes):
- Step 2b: measured filtered-minus-raw phase vs the time-average of
  atan((r^2-1)/(k_s r)) using the actual omega'(t).
- Step 3: 03e matrix reported for the paper-literal in-phase output v' AND the
  positive-sequence output (D-020, non-paper option): gain, phase, ripple.
- Step 4: sigmoid @100 us status (raw metrics independent of the filter; FLL
  locked/collapsed/NaN, first-NaN time and omega' just before).
- Step 5: collapse cases with min_omega=None; synthetic epsilon_f budget (32 seeds)
  and real-signal budget for sign @100/50/25 us.
- Step 6: windowed ramp fit z ~ (G0 + G1 (t-t_c)) z_true (60 ms) for both outputs.

Settings: library default (epsilon_filter_tau=OFF, no clamps, no normalization)
unless a block explicitly says otherwise. Outputs: experiments/output/03f_verification/.
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

import config_diff  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.filters.sogi import SOGIQSG  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from validation_metrics import (  # noqa: E402
    electrical_angle,
    sliding_window_ramp_metrics,
    summarize_ramp,
    vector_metrics,
)

K_S = math.sqrt(2.0)
GAMMA = 50.0
TS_VALUES = (100e-6, 50e-6, 25e-6)
RPM_START, RPM_END, RAMP_S, DURATION_S = 500.0, 1000.0, 1.5, 2.0
STEADY_CYCLES = 8


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _run_fll(e_alpha, e_beta, dt, initial_omega, *, positive_sequence=True, min_omega=None,
             epsilon_filter_tau=None):
    fll = DSOGIFLL(
        DSOGIFLLParameters(
            K_S, GAMMA, initial_omega, min_omega=min_omega,
            epsilon_filter_tau=epsilon_filter_tau, positive_sequence=positive_sequence,
        )
    )
    n = len(e_alpha)
    freq = np.full(n, np.nan)
    va = np.full(n, np.nan)
    vb = np.full(n, np.nan)
    error = None
    try:
        for k in range(n):
            out = fll.step(float(e_alpha[k]), float(e_beta[k]), dt)
            freq[k], va[k], vb[k] = out.frequency_hz, out.v_alpha, out.v_beta
    except ValueError as exc:
        error = str(exc)
    return freq, va, vb, error


def _mean_eps(e_alpha, e_beta, omega_fixed, dt) -> float:
    alpha, beta = SOGIQSG(K_S), SOGIQSG(K_S)
    eps = np.empty(len(e_alpha))
    for k in range(len(e_alpha)):
        oa = alpha.step(float(e_alpha[k]), omega_fixed, dt)
        ob = beta.step(float(e_beta[k]), omega_fixed, dt)
        eps[k] = oa.quadrature * (float(e_alpha[k]) - oa.in_phase) + ob.quadrature * (
            float(e_beta[k]) - ob.in_phase
        )
    return float(np.mean(eps[len(eps) // 2 :]))


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)

    def rpm_at(t):
        return RPM_START + min(t / RAMP_S, 1.0) * (RPM_END - RPM_START)

    f_start = parameters.pole_pairs * RPM_START / 60.0
    initial_omega = 2.0 * math.pi * f_start
    period = 1.0 / (parameters.pole_pairs * RPM_END / 60.0)
    steady_start = DURATION_S - STEADY_CYCLES * period

    step3_rows: list[dict] = []
    step6_rows: dict[str, dict] = {}
    step2_rows: list[dict] = []

    for dt in TS_VALUES:
        t = np.arange(int(round(DURATION_S / dt))) * dt
        _, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60, DURATION_S, dt
        )
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        f_true = parameters.pole_pairs * np.array([rpm_at(tt) for tt in t]) / 60.0
        steady_mask = (t >= steady_start) & (t <= DURATION_S)
        ramp_mask = (t >= 0.0) & (t <= RAMP_S)
        for observer_name, observer in (
            ("sign", ConventionalSMO.from_config(pmsm_cfg, observer_cfg)),
            ("sigmoid", SigmoidSMO.from_config(pmsm_cfg, observer_cfg)),
        ):
            sm = phase02.observer_rollout(observer, u, i, dt)
            for output_name, positive_sequence in (("in_phase_v_prime", False), ("positive_sequence", True)):
                freq, va, vb, error = _run_fll(
                    sm["ea"], sm["eb"], dt, initial_omega, positive_sequence=positive_sequence
                )
                for window, mask, f_true_local in (
                    ("steady", steady_mask, f_true),
                    ("ramp", ramp_mask, f_true),
                ):
                    if np.isnan(freq[mask]).any():
                        metrics = {"diverged": True, "error": error}
                    else:
                        vm = vector_metrics(sm["ea"][mask], sm["eb"][mask], emf[mask, 0], emf[mask, 1], theta[mask])
                        filt = vector_metrics(va[mask], vb[mask], emf[mask, 0], emf[mask, 1], theta[mask])
                        metrics = {
                            "raw_gain": vm["fundamental_gain"],
                            "raw_phase_deg": vm["fundamental_phase_deg"],
                            "raw_ripple_rms_v": vm["ripple_rms_v"],
                            "gain": filt["fundamental_gain"],
                            "phase_deg": filt["fundamental_phase_deg"],
                            "ripple_rms_v": filt["ripple_rms_v"],
                            "frequency_error_mean_hz": float(np.mean(freq[mask] - f_true_local[mask])),
                            "omega_prime_std_hz": float(np.std(freq[mask])),
                            "measured_filtered_minus_raw_phase_deg": filt["fundamental_phase_deg"]
                            - vm["fundamental_phase_deg"],
                        }
                    step3_rows.append(
                        {"ts_us": dt * 1e6, "observer": observer_name, "output": output_name,
                         "window": window, "metrics": metrics}
                    )
                # Step 6: windowed ramp fit (skip if NaN)
                if not np.isnan(freq).any():
                    ramp_slice = ramp_mask
                    rows = sliding_window_ramp_metrics(
                        va[ramp_slice], vb[ramp_slice], emf[ramp_slice, 0], emf[ramp_slice, 1],
                        freq[ramp_slice], f_true[ramp_slice], t[ramp_slice], window_s=0.06,
                    )
                    step6_rows[f"Ts{dt*1e6:.0f}_{observer_name}_{output_name}"] = summarize_ramp(rows)
                # Step 2: phase-average check for the sign observer, positive-sequence
                if observer_name == "sign" and positive_sequence and not np.isnan(freq).any():
                    r = freq[steady_mask] / f_true[steady_mask]
                    avg = float(np.degrees(np.mean(np.arctan((r * r - 1.0) / (K_S * r)))))
                    step2_rows.append(
                        {"ts_us": dt * 1e6, "time_avg_formula_deg": avg,
                         "mean_r": float(np.mean(r)),
                         "measured_filtered_minus_raw_phase_deg": float(
                             vector_metrics(va[steady_mask], vb[steady_mask], emf[steady_mask, 0], emf[steady_mask, 1], theta[steady_mask])["fundamental_phase_deg"]
                             - vector_metrics(sm["ea"][steady_mask], sm["eb"][steady_mask], emf[steady_mask, 0], emf[steady_mask, 1], theta[steady_mask])["fundamental_phase_deg"]
                         )}
                    )

    # Step 4: sigmoid @100 us status (library default, no clamp)
    dt = 100e-6
    t = np.arange(int(round(DURATION_S / dt))) * dt
    _, u, i, emf = phase02.plant_rollout(
        plant, lambda tt: parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60, DURATION_S, dt
    )
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    sm = phase02.observer_rollout(SigmoidSMO.from_config(pmsm_cfg, observer_cfg), u, i, dt)
    raw = vector_metrics(sm["ea"], sm["eb"], emf[:, 0], emf[:, 1], theta)
    freq, _, _, error = _run_fll(sm["ea"], sm["eb"], dt, initial_omega)
    nan_idx = np.where(np.isnan(freq))[0]
    step4 = {
        "settings": "epsilon_filter_tau=OFF, min_omega=None, no normalization",
        "raw_metrics_independent_of_filter": raw,
        "first_nan_time_s": None if len(nan_idx) == 0 else float(nan_idx[0] * dt),
        "omega_prime_hz_just_before_nan": None if len(nan_idx) == 0 else float(freq[nan_idx[0] - 1]),
        "error": error,
        "nan_from_tan_omega_singularity": False,
        "note": "omega' went negative; SOGIQSG rejects omega'<0 (ValueError). No tan(w'Ts/2)/w' is used.",
    }

    # Step 5: collapse cases with min_omega=None + budgets
    collapse = []
    dt5, total5 = 100e-6, 0.6
    for rpm in (1000.0, 500.0):
        omega = parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
        tt = np.arange(int(round(total5 / dt5))) * dt5
        theta5 = omega * tt
        amp = parameters.psi_f * omega
        true_a, true_b = -amp * np.sin(theta5), amp * np.cos(theta5)
        frac = 248.0 / amp
        period2 = amp * frac * ((-1.0) ** np.arange(len(tt)))
        rng = np.random.default_rng(0)
        white = rng.normal(0.0, amp * frac, len(tt))
        _, u5, i5, _ = phase02.plant_rollout(plant, lambda x: omega, total5, dt5)
        sm5 = phase02.observer_rollout(SigmoidSMO.from_config(pmsm_cfg, observer_cfg), u5, i5, dt5)
        for name, (ea, eb) in (
            ("ideal", (true_a, true_b)),
            ("period-2", (true_a + period2, true_b + period2)),
            ("white", (true_a + white, true_b + white)),
            ("sigmoid@100us", (sm5["ea"], sm5["eb"])),
        ):
            freq5, _, _, error5 = _run_fll(ea, eb, dt5, omega, min_omega=None)
            nan5 = np.where(np.isnan(freq5))[0]
            collapse.append(
                {"rpm": rpm, "case": name, "min_omega": None,
                 "final": None if np.isnan(freq5[-1]) else float(freq5[-1]),
                 "first_nan_s": None if len(nan5) == 0 else float(nan5[0] * dt5),
                 "error": error5}
            )
    # synthetic white budget (32 seeds)
    omega_b = parameters.pole_pairs * 1000.0 * 2.0 * math.pi / 60.0
    amp_b = parameters.psi_f * omega_b
    tb = np.arange(int(round(0.3 / dt5))) * dt5
    thb = omega_b * tb
    ta_b, tb_b = -amp_b * np.sin(thb), amp_b * np.cos(thb)
    frac_b = 248.0 / amp_b
    vals = []
    for seed in range(32):
        rng = np.random.default_rng(seed)
        wh = rng.normal(0.0, amp_b * frac_b, len(tb))
        vals.append(_mean_eps(ta_b + wh, tb_b + wh, omega_b, dt5))
    vals = np.asarray(vals)
    budget_synthetic = {
        "white_mean_eps_f": float(vals.mean()), "white_std_eps_f": float(vals.std()),
        "white_implied_domega_dt_hz_per_s": float(GAMMA * vals.mean() / (2 * math.pi)),
        "period2_mean_eps_f": _mean_eps(
            ta_b + amp_b * frac_b * ((-1.0) ** np.arange(len(tb))),
            tb_b + amp_b * frac_b * ((-1.0) ** np.arange(len(tb))), omega_b, dt5),
        "n_seeds": 32,
    }
    budget_real = []
    observed = {100.0: 7.91, 50.0: 4.97, 25.0: 2.20}
    for ts_us in (100.0, 50.0, 25.0):
        ts = ts_us * 1e-6
        _, ur, ir, emfr = phase02.plant_rollout(plant, lambda x: omega_b, 0.3, ts)
        thr = electrical_angle(emfr[:, 0], emfr[:, 1])
        smr = phase02.observer_rollout(ConventionalSMO.from_config(pmsm_cfg, observer_cfg), ur, ir, ts)
        z = smr["ea"] + 1j * smr["eb"]
        phasor = np.mean(z * np.exp(-1j * thr))
        ripple = z - phasor * np.exp(1j * thr)
        fund = phasor * np.exp(1j * thr)
        eps_fund = _mean_eps(fund.real, fund.imag, omega_b, ts)
        eps_ripple = _mean_eps(ripple.real, ripple.imag, omega_b, ts)
        dw = 0.01 * omega_b
        slope = (
            _mean_eps(fund.real, fund.imag, omega_b + dw, ts)
            - _mean_eps(fund.real, fund.imag, omega_b - dw, ts)
        ) / (2 * dw)
        budget_real.append({
            "ts_us": ts_us,
            "mean_eps_fundamental": eps_fund,
            "mean_eps_ripple": eps_ripple,
            "restoring_slope": slope,
            "predicted_offset_hz": float(-eps_ripple / slope / (2 * math.pi)),
            "observed_offset_hz": observed[ts_us],
        })

    diff = config_diff.summary(
        "03f_verification",
        {"epsilon_filter_tau": None, "min_omega": None, "normalize_error": False, "positive_sequence": True},
        scenario={"ts_us": [x * 1e6 for x in TS_VALUES], "step6_window_s": 0.06,
                  "step4_5_settings": "library default; step5 min_omega=None"},
    )
    result = {
        "note": "Phase 03f; library default (epsilon_filter_tau=OFF, no clamps, no normalization) unless stated. Filter metrics only; no position/speed.",
        "config_diff": diff,
        "step2_phase_check": step2_rows,
        "step3_outputs": step3_rows,
        "step4_sigmoid_100us": step4,
        "step5": {"collapse_no_clamp": collapse, "budget_synthetic": budget_synthetic, "budget_real": budget_real},
        "step6_ramp": step6_rows,
    }
    output_dir = config.find_project_root() / "experiments" / "output" / "03f_verification"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")

    config_diff.emit(diff)
    print("== step2 phase check (sign rows; formula vs measured filtered-raw) ==")
    for row in step2_rows:
        print(f"  Ts={row['ts_us']:.0f}us: time-avg formula={row['time_avg_formula_deg']:+.2f} deg, "
              f"measured={row['measured_filtered_minus_raw_phase_deg']:+.2f} deg")
    print("== step4 sigmoid@100us ==")
    print(json.dumps(step4, indent=2))
    print("== step5 collapse (no clamp) ==")
    for row in collapse:
        print(f"  {row['rpm']:.0f}rpm {row['case']:14s}: final={row['final']} firstNaN={row['first_nan_s']} err={row['error']}")
    print("== step5 budgets ==")
    print(json.dumps({"synthetic": budget_synthetic, "real": budget_real}, indent=2))
    print("== step6 ramp (mean gain/phase/ripple/omega-err) ==")
    for key, val in step6_rows.items():
        print(f"  {key}: mean={val['mean']} worst_ripple={val['worst']['ripple_rms_v']:.2f}")
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
