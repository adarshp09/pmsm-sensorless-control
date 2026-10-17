#!/usr/bin/env python3
"""Phase 03e: re-base the filtered metrics with the LIBRARY-DEFAULT DSOGI-FLL.

Settings: **epsilon_filter_tau=OFF (library default)**, no clamps, no normalization.
(Step 4 reproduces 03d rows and therefore uses 03d's settings for those rows; each
block states its options.)

Step 3 matrix: observers {sign, sigmoid} x Ts {100, 50, 25 us} x windows
{steady, ramp}. The steady window is the last 8 whole electrical cycles at
1000 rpm = [1.880, 2.000] s; the ramp window is [0, 1.500] s (500 -> 1000 rpm).

Step 4: collapse check for the 03d rows whose mean omega' collapsed towards zero.

Outputs go to ``experiments/output/03e_filter_rebase/``.
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
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.filters.sogi import SOGIQSG  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from validation_metrics import electrical_angle, vector_metrics  # noqa: E402

RPM_START, RPM_END = 500.0, 1000.0
RAMP_S = 1.5
DURATION_S = 2.0
TS_VALUES = (100e-6, 50e-6, 25e-6)
STEADY_CYCLES = 8
K_S = math.sqrt(2.0)
GAMMA = 50.0
STEP4_MIN_FREQ_HZ = 2.0  # 03d used this clamp; Step 3 uses no clamps


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _library_default_params(initial_omega):
    return DSOGIFLLParameters(K_S, GAMMA, initial_omega)  # all other options default


def _run_fll(e_alpha, e_beta, dt, initial_omega, *, epsilon_filter_tau=None, min_omega=None):
    fll = DSOGIFLL(
        DSOGIFLLParameters(
            K_S, GAMMA, initial_omega, epsilon_filter_tau=epsilon_filter_tau, min_omega=min_omega
        )
    )
    n = len(e_alpha)
    freq = np.full(n, np.nan)
    va = np.full(n, np.nan)
    vb = np.full(n, np.nan)
    try:
        for k in range(n):
            out = fll.step(float(e_alpha[k]), float(e_beta[k]), dt)
            freq[k], va[k], vb[k] = out.frequency_hz, out.v_alpha, out.v_beta
    except ValueError:
        pass
    return freq, va, vb


def _predicted_phase_offset(omega_mean, omega_true) -> float:
    """atan((r^2 - 1)/(k_s r)), r = omega'/omega_true (degrees)."""
    r = omega_mean / omega_true
    return math.degrees(math.atan((r * r - 1.0) / (K_S * r)))


def _regime(observer: str, dt: float, lam: float) -> str:
    if observer == "sign":
        return "bang-bang"
    return "limit cycle" if dt * lam > 2.0 else "stable"


def step3(plant, phase02, pmsm_cfg, observer_cfg, lam, dt_config):
    profile = config.load_config("experiments", directory=config.default_config_dir())["specs"]["speed_profile"][0]
    from_rpm, to_rpm = float(profile["from_rpm"]), float(profile["to_rpm"])

    def rpm_at(t):
        return from_rpm + min(t / RAMP_S, 1.0) * (to_rpm - from_rpm)

    f_start = plant.parameters.pole_pairs * from_rpm / 60.0
    initial_omega = 2.0 * math.pi * f_start

    rows = []
    for dt in TS_VALUES:
        t = np.arange(int(round(DURATION_S / dt))) * dt
        _, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: plant.parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60, DURATION_S, dt
        )
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        f_true = plant.parameters.pole_pairs * np.array([rpm_at(tt) for tt in t]) / 60.0
        period = 1.0 / (plant.parameters.pole_pairs * to_rpm / 60.0)
        steady_end = DURATION_S
        steady_start = steady_end - STEADY_CYCLES * period
        masks = {
            "steady": (t >= steady_start) & (t <= steady_end),
            "ramp": (t >= 0.0) & (t <= RAMP_S),
        }
        for observer_name, observer in (
            ("sign", ConventionalSMO.from_config(pmsm_cfg, observer_cfg)),
            ("sigmoid", SigmoidSMO.from_config(pmsm_cfg, observer_cfg)),
        ):
            sm = phase02.observer_rollout(observer, u, i, dt)
            freq, va, vb = _run_fll(sm["ea"], sm["eb"], dt, initial_omega)
            for window, mask in masks.items():
                if np.isnan(freq[mask]).any():
                    rows.append({"ts_us": dt * 1e6, "observer": observer_name, "window": window,
                                 "regime": _regime(observer_name, dt, lam), "diverged": True})
                    continue
                raw = vector_metrics(sm["ea"][mask], sm["eb"][mask], emf[mask, 0], emf[mask, 1], theta[mask])
                filt = vector_metrics(va[mask], vb[mask], emf[mask, 0], emf[mask, 1], theta[mask])
                omega_true = 2.0 * math.pi * float(np.mean(f_true[mask]))
                omega_mean = 2.0 * math.pi * float(np.mean(freq[mask]))
                stride = max(1, len(t) // 300)
                rows.append({
                    "ts_us": dt * 1e6,
                    "observer": observer_name,
                    "window": window,
                    "window_s": [float(t[mask][0]), float(t[mask][-1])],
                    "regime": _regime(observer_name, dt, lam),
                    "t_trace_s": t[::stride].tolist(),
                    "freq_trace_hz": freq[::stride].tolist(),
                    "initial_omega_rad_s": initial_omega,
                    "initial_omega_choice": "true electrical frequency at the ramp start (500 rpm); paper states none",
                    "raw": raw,
                    "filtered": filt,
                    "frequency_error_mean_hz": float(np.mean(freq[mask] - f_true[mask])),
                    "omega_prime_jitter_hz": float(np.std(freq[mask])),
                    "predicted_phase_offset_deg": _predicted_phase_offset(omega_mean, omega_true),
                    "measured_filtered_minus_raw_phase_deg": filt["fundamental_phase_deg"]
                    - raw["fundamental_phase_deg"],
                })
    return {"steady_window": {"cycles": STEADY_CYCLES, "start_s": DURATION_S - STEADY_CYCLES / (plant.parameters.pole_pairs * RPM_END / 60.0), "end_s": DURATION_S},
            "ramp_window": {"start_s": 0.0, "end_s": RAMP_S},
            "rows": rows}


def _mean_epsilon_f(v_a, v_b, omega_fixed, dt) -> float:
    sa, sb = SOGIQSG(K_S), SOGIQSG(K_S)
    eps = np.empty(len(v_a))
    for k in range(len(v_a)):
        oa = sa.step(float(v_a[k]), omega_fixed, dt)
        ob = sb.step(float(v_b[k]), omega_fixed, dt)
        eps[k] = oa.quadrature * (float(v_a[k]) - oa.in_phase) + ob.quadrature * (
            float(v_b[k]) - ob.in_phase
        )
    return float(np.mean(eps[len(eps) // 2 :]))


def step4(_plant, _phase02, _pmsm_cfg, _observer_cfg):
    """Collapse check for the 03d rows whose mean omega' collapsed towards zero."""
    dt = 100e-6
    total = 0.6
    rows = []
    budget = []
    for rpm in (1000.0, 500.0):
        omega_true = _plant.parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
        f_true = omega_true / (2.0 * math.pi)
        t = np.arange(int(round(total / dt))) * dt
        theta = omega_true * t
        amp = _plant.parameters.psi_f * omega_true
        true_a, true_b = -amp * np.sin(theta), amp * np.cos(theta)
        rng = np.random.default_rng(0)
        ripple_frac = 248.0 / amp
        period2 = amp * ripple_frac * ((-1.0) ** np.arange(len(t)))
        white = rng.normal(0.0, amp * ripple_frac, len(t))
        cases = {
            "ideal (a)": (true_a, true_b),
            "period-2 (b1)": (true_a + period2, true_b + period2),
            "white (b2)": (true_a + white, true_b + white),
            "sigmoid@100us (c)": None,  # filled from the observer below
        }
        _, u_sm, i_sm, _ = _phase02.plant_rollout(_plant, lambda tt: omega_true, total, dt)
        sm = _phase02.observer_rollout(
            SigmoidSMO.from_config(_pmsm_cfg, _observer_cfg), u_sm, i_sm, dt
        )
        cases["sigmoid@100us (c)"] = (sm["ea"], sm["eb"])
        for name, signals in cases.items():
            if signals is None:
                continue
            for lpf in (True, False):
                tau = 0.01 if lpf else None
                freq, _, _ = _run_fll(
                    signals[0], signals[1], dt, omega_true,
                    epsilon_filter_tau=tau, min_omega=2.0 * math.pi * STEP4_MIN_FREQ_HZ,
                )
                below = np.where(freq < 10.0)[0]
                first_below = float(t[below[0]]) if len(below) else None
                recovered = bool(np.any(freq[below[-1] + 1 :] >= 10.0)) if len(below) else False
                rows.append({
                    "rpm": rpm,
                    "case": name,
                    "epsilon_filter_tau": tau,
                    "final_omega_prime_hz": float(freq[-1]) if not np.isnan(freq[-1]) else None,
                    "first_below_10hz_s": first_below,
                    "recovers_above_10hz": recovered,
                    "omega_prime_trace_hz": [float(x) for x in freq[:: max(1, len(freq) // 300)]],
                })

        # (c) open-loop budget at frozen omega' = omega_true.
        eps_fund = _mean_epsilon_f(true_a, true_b, omega_true, dt)
        eps_p2 = _mean_epsilon_f(period2, period2, omega_true, dt)
        eps_white = _mean_epsilon_f(white, white, omega_true, dt)
        eps_p2_both = _mean_epsilon_f(true_a + period2, true_b + period2, omega_true, dt)
        eps_white_both = _mean_epsilon_f(true_a + white, true_b + white, omega_true, dt)
        dw = 0.01 * omega_true
        slope = (_mean_epsilon_f(true_a, true_b, omega_true + dw, dt)
                 - _mean_epsilon_f(true_a, true_b, omega_true - dw, dt)) / (2.0 * dw)
        budget.append({
            "rpm": rpm,
            "omega_true_rad_s": omega_true,
            "mean_epsilon_f_fundamental": eps_fund,
            "mean_epsilon_f_period2_only": eps_p2,
            "mean_epsilon_f_white_only": eps_white,
            "mean_epsilon_f_period2_both": eps_p2_both,
            "mean_epsilon_f_white_both": eps_white_both,
            "implied_domega_dt_period2_hz_per_s": GAMMA * eps_p2_both / (2.0 * math.pi),
            "implied_domega_dt_white_hz_per_s": GAMMA * eps_white_both / (2.0 * math.pi),
            "fundamental_restoring_slope": slope,
        })
    return {"rows": rows, "open_loop_budget": budget,
            "settings": {"epsilon_filter_tau": "on (0.01 s) and off, per row", "min_omega_hz": STEP4_MIN_FREQ_HZ}}


def make_plot(step3_rows, output_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.4))
    seen = set()
    for row in step3_rows["rows"]:
        if row.get("diverged") or row["window"] != "ramp":
            continue
        key = (row["observer"], row["ts_us"])
        if key in seen:
            continue
        seen.add(key)
        axes[0].plot(
            row["t_trace_s"], row["freq_trace_hz"], label=f"{row['observer']} {row['ts_us']:.0f}us"
        )
    axes[0].set(
        xlabel="time (s)", ylabel="omega' (Hz)",
        title="FLL omega'(t), ramp 500->1000 rpm (epsilon_filter_tau=OFF)",
    )
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    labels, ripple_raw, ripple_filt = [], [], []
    for row in step3_rows["rows"]:
        if row.get("diverged") or row["window"] != "steady":
            continue
        labels.append(f"{row['observer'][0]} {row['ts_us']:.0f}us")
        ripple_raw.append(row["raw"]["ripple_rms_v"])
        ripple_filt.append(row["filtered"]["ripple_rms_v"])
    x = np.arange(len(labels))
    axes[1].bar(x - 0.2, ripple_raw, 0.4, label="raw E_hat")
    axes[1].bar(x + 0.2, ripple_filt, 0.4, label="filtered")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=25, fontsize=8)
    axes[1].set(
        ylabel="ripple RMS (V)",
        title="Ripple, steady window (epsilon_filter_tau=OFF (library default))",
    )
    axes[1].legend()
    axes[1].grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(output_dir / "rebase.png", dpi=130)
    plt.close(fig)


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)

    # sigmoid linear-region rate for the regime label
    k = (
        config.resolve_parameter(observer_cfg, "smo.switching_gain_k", unit="-")
        * math.sqrt(2.0)
        * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V")
        / math.sqrt(3.0)
    )
    i_base = math.sqrt(2.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_power", unit="W") / (
        math.sqrt(3.0) * config.resolve_parameter(pmsm_cfg, "rating.rated_voltage", unit="V")
    )
    a = config.resolve_parameter(observer_cfg, "smo.sigmoid_slope_a", unit="-") / i_base
    lam = (parameters.stator_resistance + k * a / 2.0) / parameters.ld

    rebase = step3(plant, phase02, pmsm_cfg, observer_cfg, lam, None)
    collapse = step4(plant, phase02, pmsm_cfg, observer_cfg)

    diff = config_diff.summary(
        "03e_filter_rebase",
        {"epsilon_filter_tau": None, "min_omega": None, "max_omega": None,
         "normalize_error": False, "positive_sequence": True},
        scenario={"ts_us": [x * 1e6 for x in TS_VALUES], "steady_cycles": STEADY_CYCLES,
                  "step4_settings": collapse["settings"]},
    )
    diagnostics = {
        "note": "Phase 03e re-base; filtered metrics use epsilon_filter_tau=OFF (library default), no clamps, no normalization.",
        "config_diff": diff,
        "sigmoid_lambda_per_s": lam,
        "step3": rebase,
        "step4_collapse": collapse,
    }
    output_dir = config.find_project_root() / "experiments" / "output" / "03e_filter_rebase"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_plot(rebase, output_dir)
    (output_dir / "rebase.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")

    config_diff.emit(diff)
    print(f"step3 steady window: {rebase['steady_window']}")
    print(f"step3 ramp window: {rebase['ramp_window']}")
    for row in rebase["rows"]:
        if row.get("diverged"):
            print(f"  {row['ts_us']:5.0f}us {row['observer']:7s} {row['window']:6s}: DIVERGED")
            continue
        print(f"  {row['ts_us']:5.0f}us {row['observer']:7s} {row['window']:6s} [{row['regime']:11s}]: "
              f"ferr={row['frequency_error_mean_hz']:+7.3f} Hz jit={row['omega_prime_jitter_hz']:6.3f} "
              f"rawG={row['raw']['fundamental_gain']:.3f} rawPh={row['raw']['fundamental_phase_deg']:+6.2f} "
              f"filtG={row['filtered']['fundamental_gain']:.3f} filtPh={row['filtered']['fundamental_phase_deg']:+6.2f} "
              f"rawRip={row['raw']['ripple_rms_v']:7.2f} filtRip={row['filtered']['ripple_rms_v']:6.2f} "
              f"predPh={row['predicted_phase_offset_deg']:+6.2f} measPh={row['measured_filtered_minus_raw_phase_deg']:+6.2f}")
    print("== step4 collapse ==")
    for row in collapse["rows"]:
        print(f"  {row['rpm']:.0f}rpm {row['case']:20s} tau={row['epsilon_filter_tau']}: "
              f"final={row['final_omega_prime_hz']} first<10Hz={row['first_below_10hz_s']} recovers={row['recovers_above_10hz']}")
    print("== open-loop budget ==")
    for b in collapse["open_loop_budget"]:
        print(f"  {b['rpm']:.0f}rpm: eps_fund={b['mean_epsilon_f_fundamental']:+.4e} "
              f"eps_p2={b['mean_epsilon_f_period2_only']:+.4e} eps_white={b['mean_epsilon_f_white_only']:+.4e} "
              f"dwd_t_p2={b['implied_domega_dt_period2_hz_per_s']:+.4f} dwdt_white={b['implied_domega_dt_white_hz_per_s']:+.4f} "
              f"restoring_slope={b['fundamental_restoring_slope']:+.4e}")
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
