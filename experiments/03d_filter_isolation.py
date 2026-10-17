#!/usr/bin/env python3
"""Phase 03d: isolate the DSOGI-FLL loop from the input ripple and the observer.

Tasks (no plant/observer/filter code or defaults changed):

1. Feed the DSOGI-FLL, at 1000 and 500 rpm over a steady window of an **integer
   number of electrical cycles**: (a) ideal ``E_true``; (b) ``E_true`` + synthetic
   ripple at the measured raw-observer RMS, as period-2 (alternating) and white;
   (c) the post-fix SMO outputs (sign@100us, sigmoid@100us [limit cycle],
   sigmoid@50us [stable]). Report mean frequency error, ``omega'`` jitter,
   filtered gain/phase and residual ripple, and attribute the -5 Hz bias / 0.85
   gain ratio.
2. Amplitude dependence at 0.1x / 1x / 10x ``E_true`` (1000 rpm): settling time
   after a frequency step, and steady jitter; repeat with normalization ON.
3. Post-fix filtered metrics at Ts = 100/50/25 us, both observers (steady + ramp)
   -- read from the Phase 03 run, annotated with the limit-cycle flag.
4. Low-speed check for the sign observer at 100/200 rpm: fit gain vs number of
   electrical cycles in the window (1/2/4/8) and half-window scatter.

Outputs go to ``experiments/output/03d_filter_isolation/``.
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

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.observers.conventional_smo import ConventionalSMO  # noqa: E402
from pmsm_sensorless.observers.sigmoid_smo import SigmoidSMO  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
import config_diff  # noqa: E402
from validation_metrics import electrical_angle, vector_metrics  # noqa: E402

RPM_LIST = (1000.0, 500.0)
CYCLES = 8
SETTLE_S = 0.2
EPSILON_FILTER_TAU = 0.01
MIN_FREQ_HZ = 2.0
K_S = math.sqrt(2.0)
GAMMA = 50.0
TS_STAR_SIGMOID_US = 60.1  # D-027: 2 Ld/(Rs + k a/2)


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _fll(initial_omega: float, normalize: bool = False, lpf: bool = True) -> DSOGIFLL:
    return DSOGIFLL(
        DSOGIFLLParameters(
            K_S,
            GAMMA,
            initial_omega,
            min_omega=2.0 * math.pi * MIN_FREQ_HZ,
            epsilon_filter_tau=EPSILON_FILTER_TAU if lpf else None,
            normalize_error=normalize,
        )
    )


def _run_fll(e_alpha, e_beta, dt, initial_omega, normalize=False, lpf=True):
    fll = _fll(initial_omega, normalize, lpf)
    n = len(e_alpha)
    freq = np.full(n, np.nan)
    va = np.full(n, np.nan)
    vb = np.full(n, np.nan)
    try:
        for k in range(n):
            out = fll.step(float(e_alpha[k]), float(e_beta[k]), dt)
            freq[k], va[k], vb[k] = out.frequency_hz, out.v_alpha, out.v_beta
    except ValueError:
        pass  # diverged (omega' went non-physical); remaining samples stay NaN
    return freq, va, vb


def _metrics(freq, va, vb, true_a, true_b, theta, mask, f_true) -> dict:
    if np.isnan(freq[mask]).any() or np.isnan(va[mask]).any():
        return {"diverged": True}
    vm = vector_metrics(va[mask], vb[mask], true_a[mask], true_b[mask], theta[mask])
    return {
        "frequency_error_mean_hz": float(np.mean(freq[mask] - f_true[mask])),
        "omega_prime_jitter_hz": float(np.std(freq[mask])),
        "filtered_gain": vm["fundamental_gain"],
        "filtered_phase_deg": vm["fundamental_phase_deg"],
        "residual_ripple_v": vm["ripple_rms_v"],
    }


def _true_signals(plant, rpm, dt, total):
    """Constant-speed ideal back-EMF (E = omega psi_f [-sin, cos])."""
    omega = plant.parameters.pole_pairs * rpm * 2.0 * math.pi / 60.0
    t = np.arange(int(round(total / dt))) * dt
    theta = omega * t
    amp = plant.parameters.psi_f * omega
    return t, theta, -amp * np.sin(theta), amp * np.cos(theta), omega / (2.0 * math.pi), omega


def _observer_signals(plant, phase02, observer, omega, dt, total):
    _, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega, total, dt)
    theta = electrical_angle(emf[:, 0], emf[:, 1])
    return emf[:, 0], emf[:, 1], theta, phase02.observer_rollout(observer, u, i, dt)


def _window(n, period, dt):
    mask = np.zeros(n, dtype=bool)
    mask[n - int(round(CYCLES * period / dt)) :] = True
    return mask


def isolation(plant, phase02, pmsm_cfg, observer_cfg) -> dict:
    dt = 100e-6
    out = {}
    for rpm in RPM_LIST:
        f = plant.parameters.pole_pairs * rpm / 60.0
        period = 1.0 / f
        total = SETTLE_S + CYCLES * period
        _, theta, true_a, true_b, f_true, omega = _true_signals(plant, rpm, dt, total)
        n = len(theta)
        mask = _window(n, period, dt)
        f_true_arr = np.full(n, f_true)

        _, _, _, sm_sign = _observer_signals(
            plant, phase02, ConventionalSMO.from_config(pmsm_cfg, observer_cfg), omega, dt, total
        )
        ripple_rms = vector_metrics(
            sm_sign["ea"][mask], sm_sign["eb"][mask], true_a[mask], true_b[mask], theta[mask]
        )["ripple_rms_v"]

        rng = np.random.default_rng(0)
        alternating = ripple_rms * ((-1.0) ** np.arange(n))
        white = rng.normal(0.0, ripple_rms, n)
        cases = {}

        def add_case(label, xa, xb, dt_case, na, nb, th, mk, ft):
            # Run with the Phase 03 option (error LPF on) and with it off, to separate
            # the filter-loop configuration from the input.
            for lpf, tag in ((True, "LPF_on"), (False, "LPF_off")):
                freq, va, vb = _run_fll(xa, xb, dt_case, 2.0 * math.pi * f_true, lpf=lpf)
                cases[f"{label} [{tag}]"] = _metrics(freq, va, vb, na, nb, th, mk, ft)

        add_case("(a) ideal E_true", true_a, true_b, dt, true_a, true_b, theta, mask, f_true_arr)
        add_case(
            "(b1) E_true + period-2 ripple",
            true_a + alternating, true_b + alternating,
            dt, true_a, true_b, theta, mask, f_true_arr,
        )
        add_case(
            "(b2) E_true + white ripple",
            true_a + white, true_b + white,
            dt, true_a, true_b, theta, mask, f_true_arr,
        )
        add_case("(c) SMO sign @100us", sm_sign["ea"], sm_sign["eb"], dt,
                 true_a, true_b, theta, mask, f_true_arr)

        _, _, _, sm_sig = _observer_signals(
            plant, phase02, SigmoidSMO.from_config(pmsm_cfg, observer_cfg), omega, dt, total
        )
        add_case("(c) SMO sigmoid @100us (limit cycle)", sm_sig["ea"], sm_sig["eb"], dt,
                 true_a, true_b, theta, mask, f_true_arr)

        # sigmoid @50 us (stable) at the same rpm
        dt2 = 50e-6
        total2 = SETTLE_S + CYCLES * period
        _, theta2, true_a2, true_b2, _, _ = _true_signals(plant, rpm, dt2, total2)
        n2 = len(theta2)
        mask2 = _window(n2, period, dt2)
        _, _, _, sm_sig2 = _observer_signals(
            plant, phase02, SigmoidSMO.from_config(pmsm_cfg, observer_cfg), omega, dt2, total2
        )
        add_case("(c) SMO sigmoid @50us (stable)", sm_sig2["ea"], sm_sig2["eb"], dt2,
                 true_a2, true_b2, theta2, mask2, np.full(n2, f_true))

        out[f"{int(rpm)}rpm"] = {
            "electrical_frequency_hz": f_true,
            "window_cycles": CYCLES,
            "window_s": CYCLES * period,
            "raw_observer_ripple_rms_v": ripple_rms,
            "cases": cases,
        }
    return out


def amplitude_dependence(plant, phase02, pmsm_cfg, observer_cfg) -> dict:
    """omega' settling after a +10% frequency step vs input amplitude (1000 rpm)."""
    rpm = 1000.0
    dt = 100e-6
    f1 = plant.parameters.pole_pairs * rpm / 60.0
    f2 = 1.1 * f1
    dt_step, total = 0.3, 1.2
    t = np.arange(int(round(total / dt))) * dt
    omega_e = np.where(t < dt_step, 2 * math.pi * f1, 2 * math.pi * f2)
    theta = np.cumsum(omega_e) * dt
    e_ref = plant.parameters.psi_f * 2 * math.pi * f1
    result = {"frequency_step_hz": {"from": f1, "to": f2}}
    for scale in (0.1, 1.0, 10.0):
        amp = scale * e_ref
        ea, eb = -amp * np.sin(theta), amp * np.cos(theta)
        for normalize, tag in ((False, "default"), (True, "normalization_ON")):
            # Library default: no error LPF (the Phase 03 experiment enabled it; it
            # destabilises the loop, see the isolation cases).
            freq, _, _ = _run_fll(ea, eb, dt, 2.0 * math.pi * f1, normalize, lpf=False)
            tol = 0.05 * (f2 - f1)
            after = np.where(t >= dt_step)[0]
            settle_idx = next((k for k in after if abs(freq[k] - f2) < tol), None)
            result[f"amp_{scale}x_{tag}"] = {
                "settling_time_s": None if settle_idx is None else float(t[settle_idx] - dt_step),
                "steady_jitter_hz": float(np.std(freq[t > dt_step + 0.5])),
            }
    return result


def low_speed_fit(plant, phase02, pmsm_cfg, observer_cfg) -> dict:
    """Sign observer 100/200 rpm: fit gain vs cycles in the window + half-window scatter."""
    dt = 100e-6
    result = {}
    for rpm in (100.0, 200.0):
        f = plant.parameters.pole_pairs * rpm / 60.0
        total = SETTLE_S + 8.0 / f
        _, theta, _, _, _, omega = _true_signals(plant, rpm, dt, total)
        _, _, _, sm = _observer_signals(
            plant, phase02, ConventionalSMO.from_config(pmsm_cfg, observer_cfg), omega, dt, total
        )
        _, u, i, emf = phase02.plant_rollout(plant, lambda tt: omega, total, dt)
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        n = len(theta)
        rows = []
        for cycles in (1, 2, 4, 8):
            width = int(round(cycles / f / dt))
            start = n - width
            vm = vector_metrics(
                sm["ea"][start:], sm["eb"][start:], emf[start:, 0], emf[start:, 1], theta[start:]
            )
            half = width // 2
            scatter = [
                vector_metrics(
                    sm["ea"][start + j * half : start + j * half + half],
                    sm["eb"][start + j * half : start + j * half + half],
                    emf[start + j * half : start + j * half + half, 0],
                    emf[start + j * half : start + j * half + half, 1],
                    theta[start + j * half : start + j * half + half],
                )["fundamental_gain"]
                for j in range(2)
            ]
            rows.append(
                {
                    "cycles": cycles,
                    "fundamental_gain": vm["fundamental_gain"],
                    "fundamental_phase_deg": vm["fundamental_phase_deg"],
                    "half_window_gain_std": float(np.std(scatter)),
                }
            )
        result[f"{int(rpm)}rpm"] = {"rows": rows}
    return result


def phase03_table() -> dict:
    """Read the Phase 03 post-fix table and annotate limit-cycle rows (D-027)."""
    path = Path(__file__).resolve().parents[1] / "experiments" / "output" / "03_dsogi_fll_validation" / "diagnostics.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    table = {}
    for ts, entry in data["results"].items():
        table[ts] = {}
        for observer in ("conv", "sig"):
            row = dict(entry[observer])
            if observer == "sig" and ts == "100us":
                row["regime"] = "limit cycle (D-027)"
            else:
                row["regime"] = "stable" if observer == "sig" else "bang-bang"
            table[ts][observer] = row
    return table


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    plant = PMSMPlant(PMSMParameters.from_config(pmsm_cfg))

    diff = config_diff.summary(
        "03d_filter_isolation",
        {"epsilon_filter_tau": EPSILON_FILTER_TAU, "min_omega": 2.0 * math.pi * MIN_FREQ_HZ},
        scenario={"rpm": RPM_LIST, "cycles": CYCLES,
                  "note": "LPF on (0.01 s) and off are reported per row"},
    )
    diagnostics = {
        "note": "Phase 03d filter isolation; defaults/code unchanged. Filter metrics only. Options per row (LPF on/off).",
        "config_diff": diff,
        "isolation": isolation(plant, phase02, pmsm_cfg, observer_cfg),
        "amplitude_dependence": amplitude_dependence(plant, phase02, pmsm_cfg, observer_cfg),
        "low_speed_fit": low_speed_fit(plant, phase02, pmsm_cfg, observer_cfg),
        "phase03_table": phase03_table(),
        "sigmoid_ts_star_us": TS_STAR_SIGMOID_US,
    }
    output_dir = config.find_project_root() / "experiments" / "output" / "03d_filter_isolation"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "isolation.json").write_text(json.dumps(diagnostics, indent=2), encoding="utf-8")
    config_diff.emit(diff)

    for rpm, data in diagnostics["isolation"].items():
        print(f"== isolation {rpm} (f={data['electrical_frequency_hz']:.2f} Hz, "
              f"window {data['window_cycles']} cycles = {data['window_s']*1e3:.1f} ms, "
              f"raw ripple RMS {data['raw_observer_ripple_rms_v']:.1f} V) ==")
        for name, m in data["cases"].items():
            if m.get("diverged"):
                print(f"   {name:52s}: DIVERGED")
                continue
            print(f"   {name:52s}: ferr={m['frequency_error_mean_hz']:+7.3f} Hz  jitter={m['omega_prime_jitter_hz']:6.3f} Hz  "
                  f"gain={m['filtered_gain']:.3f}  phase={m['filtered_phase_deg']:+6.2f}  ripple={m['residual_ripple_v']:6.2f} V")
    print("== amplitude dependence ==")
    print(json.dumps(diagnostics["amplitude_dependence"], indent=2))
    print("== low-speed fit ==")
    print(json.dumps(diagnostics["low_speed_fit"], indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
