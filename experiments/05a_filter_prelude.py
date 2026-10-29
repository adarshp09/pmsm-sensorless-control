#!/usr/bin/env python3
"""Phase 05a: filter prelude — phase/bias diagnosis.

Inputs: raw E_hat of the fixed **sign** observer at Ts = 100/50/25 us and the
fixed **sigmoid** at 50 us (stable), 03e scenario (short-circuited plant, 500 ->
1000 rpm ramp), steady window = 8 whole electrical cycles at 1000 rpm
[1.880, 2.000] s.  FLL at **library defaults**: epsilon_filter_tau=OFF, no clamps
(min/max_omega=None), no normalization, positive_sequence=True; initial omega' =
true electrical frequency at the window start (500 rpm).  Every non-default option
is stated in the captions/JSON.

Step 2a: frozen omega' (FLL off) at omega' = omega_true x {0.9, 1.0, 1.135};
        measured v'/PNS gain/phase vs D(jw) and 1/2 D(1+w'/w).
Step 2b: band-split replay of the closed-loop omega'(t): mean / low band / mid
        band / full (full must reproduce the closed loop bit-for-bit).
Step 2c: two-integrator SOGI realization (Eq.(17) per integrator, algebraic loop
        solved in closed form) vs the biquad.
Step 3:  epsilon_f(omega') sweep with the four-term split; zero crossing vs the
        closed-loop mean omega'.

RAW E_hat only; no estimator, no position/speed claims.
Output: experiments/output/05a_filter_prelude/filter_prelude.json
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
from validation_metrics import electrical_angle, fundamental_phasor, vector_metrics  # noqa: E402

K_S = math.sqrt(2.0)
GAMMA = 50.0
RPM_START, RPM_END, RAMP_S, DURATION_S = 500.0, 1000.0, 1.5, 2.0
STEADY_CYCLES = 8
INPUTS = (("sign", 100e-6), ("sign", 50e-6), ("sign", 25e-6), ("sigmoid", 50e-6))
RATIOS_2A = (0.9, 1.0, 1.135)
RATIOS_3 = np.arange(0.80, 1.3001, 0.025)
SETTLE_START_S = 1.75


def _phase02():
    spec = importlib.util.spec_from_file_location("phase02", _DIR / "02_smo_validation.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------- plant + observers
def _build_inputs(plant, phase02, pmsm_cfg, observer_cfg):
    """Return {(observer, ts_us): dict} with raw E_hat, theta, t, steady mask, f_e."""
    def rpm_at(t):
        return RPM_START + min(t / RAMP_S, 1.0) * (RPM_END - RPM_START)

    out = {}
    f_start = plant.parameters.pole_pairs * RPM_START / 60.0
    initial_omega = 2.0 * math.pi * f_start
    period = 1.0 / (plant.parameters.pole_pairs * RPM_END / 60.0)
    for observer_name, dt in INPUTS:
        t = np.arange(int(round(DURATION_S / dt))) * dt
        _, u, i, emf = phase02.plant_rollout(
            plant, lambda tt: plant.parameters.pole_pairs * rpm_at(tt) * 2 * math.pi / 60.0, DURATION_S, dt
        )
        if observer_name == "sign":
            observer = ConventionalSMO.from_config(pmsm_cfg, observer_cfg)
        else:
            observer = SigmoidSMO.from_config(pmsm_cfg, observer_cfg)
        sm = phase02.observer_rollout(observer, u, i, dt)
        theta = electrical_angle(emf[:, 0], emf[:, 1])
        mask = (t >= DURATION_S - STEADY_CYCLES * period) & (t <= DURATION_S)
        z = sm["ea"] + 1j * sm["eb"]
        phasor = complex(np.mean(z[mask] * np.exp(-1j * theta[mask])))
        z_fund = phasor * np.exp(1j * theta)
        out[(observer_name, dt)] = {
            "observer": observer_name, "ts_us": dt * 1e6, "dt": dt, "t": t,
            "ea": sm["ea"], "eb": sm["eb"], "emf": emf, "theta": theta,
            "initial_omega": initial_omega, "steady_mask": mask,
            "fund_a": z_fund.real, "fund_b": z_fund.imag,
            "f_e_hz": plant.parameters.pole_pairs * RPM_END / 60.0,
            "omega_e": plant.parameters.pole_pairs * RPM_END * 2.0 * math.pi / 60.0,
        }
    return out


# ---------------------------------------------------------------- SOGI helpers
def _sogi_run(v_a, v_b, omega_of_step, dt):
    """Open-loop dual SOGI at prescribed omega'(t); returns in-phase/quadrature."""
    sa, sb = SOGIQSG(K_S), SOGIQSG(K_S)
    n = len(v_a)
    ip_a = np.empty(n); q_a = np.empty(n); ip_b = np.empty(n); q_b = np.empty(n)
    for k in range(n):
        om = omega_of_step(k)
        oa = sa.step(float(v_a[k]), om, dt)
        ob = sb.step(float(v_b[k]), om, dt)
        ip_a[k], q_a[k] = oa.in_phase, oa.quadrature
        ip_b[k], q_b[k] = ob.in_phase, ob.quadrature
    return ip_a, q_a, ip_b, q_b


def _pns(ip_a, q_a, ip_b, q_b):
    return 0.5 * (ip_a - q_b), 0.5 * (q_a + ip_b)


def _run_closed_loop(ea, eb, dt, initial_omega):
    """Library-default DSOGI-FLL; returns freq (post-update), omega_used, outputs."""
    fll = DSOGIFLL(DSOGIFLLParameters(K_S, GAMMA, initial_omega))
    n = len(ea)
    freq = np.empty(n)
    vp_a = np.empty(n); vp_b = np.empty(n)   # paper-literal v' (accessor)
    pns_a = np.empty(n); pns_b = np.empty(n)
    omega_used = np.empty(n)
    for k in range(n):
        omega_used[k] = fll.omega  # omega' used by the SOGI at this step
        o = fll.step(float(ea[k]), float(eb[k]), dt)
        freq[k] = o.frequency_hz
        vp_a[k], vp_b[k] = o.v_alpha_prime, o.v_beta_prime
        pns_a[k], pns_b[k] = o.v_alpha, o.v_beta
    return freq, omega_used, vp_a, vp_b, pns_a, pns_b


def _transfer_d(omega, om_prime):
    s = 1j * omega
    return K_S * om_prime * s / (s * s + K_S * om_prime * s + om_prime**2)


def _vm(a, b, inp, mask):
    return vector_metrics(a[mask], b[mask], inp["emf"][mask, 0], inp["emf"][mask, 1], inp["theta"][mask])


# ---------------------------------------------------------------- Step 2a
def step_2a(inp, ratios=RATIOS_2A):
    """Frozen omega' (FLL off): the SOGI transfer vs D(jw) and 1/2 D(1+w'/w).

    The SOGI is linear at frozen omega', so the *transfer* is the output fundamental
    (at the true angle) over the input fundamental -- this isolates the filter from
    the observer's own fundamental gain/phase (reported separately as vs_truth).
    """
    dt = inp["dt"]; mask = inp["steady_mask"]; omega_true = inp["omega_e"]
    ea_m, eb_m, th_m = inp["ea"][mask], inp["eb"][mask], inp["theta"][mask]
    p_in = fundamental_phasor(ea_m, eb_m, th_m)
    rows = []
    for r in ratios:
        om_prime = r * omega_true
        const = lambda k, o=om_prime: o
        ip_a, q_a, ip_b, q_b = _sogi_run(inp["ea"], inp["eb"], const, dt)
        pns_a, pns_b = _pns(ip_a, q_a, ip_b, q_b)
        ratio_v = fundamental_phasor(ip_a[mask], ip_b[mask], th_m) / p_in
        ratio_p = fundamental_phasor(pns_a[mask], pns_b[mask], th_m) / p_in
        mv = _vm(ip_a, ip_b, inp, mask)
        mp = _vm(pns_a, pns_b, inp, mask)
        d = _transfer_d(omega_true, om_prime)
        pred_p = 0.5 * d * (1.0 + om_prime / omega_true)
        rows.append({
            "ratio_r": r, "omega_prime_rad_s": om_prime,
            "v_prime_transfer": {
                "gain": abs(ratio_v), "phase_deg": math.degrees(math.atan2(ratio_v.imag, ratio_v.real)),
                "pred_gain": abs(d), "pred_phase_deg": math.degrees(math.atan2(d.imag, d.real)),
                "gain_err": abs(ratio_v) - abs(d),
                "phase_err_deg": math.degrees(math.atan2((ratio_v / d).imag, (ratio_v / d).real))},
            "pns_transfer": {
                "gain": abs(ratio_p), "phase_deg": math.degrees(math.atan2(ratio_p.imag, ratio_p.real)),
                "pred_gain": abs(pred_p), "pred_phase_deg": math.degrees(math.atan2(pred_p.imag, pred_p.real)),
                "gain_err": abs(ratio_p) - abs(pred_p),
                "phase_err_deg": math.degrees(math.atan2((ratio_p / pred_p).imag, (ratio_p / pred_p).real))},
            "v_prime_vs_truth": {"gain": mv["fundamental_gain"], "phase_deg": mv["fundamental_phase_deg"]},
        })
    gain_err = max(max(abs(r["v_prime_transfer"]["gain_err"]), abs(r["pns_transfer"]["gain_err"])) for r in rows)
    phase_err = max(max(abs(r["v_prime_transfer"]["phase_err_deg"]), abs(r["pns_transfer"]["phase_err_deg"])) for r in rows)
    return {"rows": rows, "max_gain_err": gain_err, "max_phase_err_deg": phase_err}


# ---------------------------------------------------------------- Step 2b
def _band_reconstruct(omega_used, dt, lo_hz, hi_hz, center):
    x = omega_used - center
    n = len(x)
    freqs = np.fft.rfftfreq(n, dt)
    spec = np.fft.rfft(x)
    keep = (freqs >= lo_hz) & (freqs < hi_hz)
    spec_b = np.where(keep, spec, 0.0)
    return center + np.fft.irfft(spec_b, n)


def step_2b(inp, closed):
    dt = inp["dt"]; mask = inp["steady_mask"]; f_e = inp["f_e_hz"]
    omega_used = closed["omega_used"]
    # mean omega' over the STEADY window (the full-run mean is skewed by the ramp)
    mu = float(np.mean(omega_used[mask]))
    variants = {
        "i_mean": np.full_like(omega_used, mu),
        "ii_mean_plus_below_we2": _band_reconstruct(omega_used, dt, 0.0, f_e / 2.0, mu),
        "iii_mean_plus_mid": _band_reconstruct(omega_used, dt, f_e / 2.0, 3.0 * f_e, mu),
        "iv_full": omega_used,
        "v_mean_plus_high": _band_reconstruct(omega_used, dt, 3.0 * f_e, 1.0 / (2.0 * dt), mu),
    }
    rows = {}
    full_ok = None
    for name, om in variants.items():
        ip_a, q_a, ip_b, q_b = _sogi_run(inp["ea"], inp["eb"], lambda k, a=om: float(a[k]), dt)
        pns_a, pns_b = _pns(ip_a, q_a, ip_b, q_b)
        mv = _vm(ip_a, ip_b, inp, mask)
        mp = _vm(pns_a, pns_b, inp, mask)
        rows[name] = {"v_prime_gain": mv["fundamental_gain"], "v_prime_phase_deg": mv["fundamental_phase_deg"],
                      "pns_gain": mp["fundamental_gain"], "pns_phase_deg": mp["fundamental_phase_deg"]}
        if name == "iv_full":
            full_ok = bool(np.array_equal(ip_a, closed["vp_a"]) and np.array_equal(ip_b, closed["vp_b"]))
    rows["iv_full_reproduces_closed_loop"] = full_ok
    rows["steady_window_mean_omega_hz"] = mu / (2.0 * math.pi)
    # measured closed-loop filtered-minus-raw phase (from the same run)
    raw = _vm(inp["ea"], inp["eb"], inp, mask)
    rows["closed_loop"] = {
        "v_prime_minus_raw_phase_deg": closed["vp_metrics"]["fundamental_phase_deg"] - raw["fundamental_phase_deg"],
        "pns_minus_raw_phase_deg": closed["pns_metrics"]["fundamental_phase_deg"] - raw["fundamental_phase_deg"],
    }
    return rows


# ---------------------------------------------------------------- Step 2c
class TwoIntegratorSOGI:
    """SOGI-QSG: v'dot = w'[ks(v - v') - qv'], qv'dot = w' v'.

    Each integrator is discretized with Eq.(17), ``T = tan(w'Ts/2)/w'`` and a
    trapezoidal accumulation ``y[k] = y[k-1] + T (g[k] + g[k-1])``; the algebraic
    loop between v' and qv' is solved in closed form.
    """

    def __init__(self, ks: float):
        self.ks = ks
        self.reset()

    def reset(self):
        self.a = 0.0; self.b = 0.0
        self.pv = 0.0; self.pa = 0.0; self.pb = 0.0; self.pom = 0.0

    def step(self, v: float, omega: float, dt: float):
        if omega <= 0.0:
            raise ValueError(f"omega must be positive, got {omega}")
        tau = math.tan(omega * dt / 2.0)
        r_a_prev = self.pom * (self.ks * (self.pv - self.pa) - self.pb)
        r_b_prev = self.pom * self.pa
        a = (self.pa + tau * self.ks * v - tau * self.pb
             - (tau * tau / omega) * r_b_prev + (tau / omega) * r_a_prev) / (1.0 + tau * self.ks + tau * tau)
        b = self.pb + tau * a + (tau / omega) * r_b_prev
        self.pv, self.pa, self.pb, self.pom = v, a, b, omega
        return a, b


def _two_integrator_run(v_a, v_b, omega_of_step, dt):
    sa, sb = TwoIntegratorSOGI(K_S), TwoIntegratorSOGI(K_S)
    n = len(v_a)
    ip_a = np.empty(n); q_a = np.empty(n); ip_b = np.empty(n); q_b = np.empty(n)
    for k in range(n):
        om = omega_of_step(k)
        a1, b1 = sa.step(float(v_a[k]), om, dt)
        a2, b2 = sb.step(float(v_b[k]), om, dt)
        ip_a[k], q_a[k], ip_b[k], q_b[k] = a1, b1, a2, b2
    return ip_a, q_a, ip_b, q_b


def step_2c(inp, closed):
    dt = inp["dt"]; mask = inp["steady_mask"]; omega_true = inp["omega_e"]
    # (1) constant omega' equivalence with the biquad, omega/omega' in 0.8..1.25
    ratios = (1 / 1.25, 1 / 1.1, 1.0, 1 / 0.9, 0.8)
    equiv = []
    for om_ratio in ratios:
        om = om_ratio * omega_true
        ip_a, q_a, ip_b, q_b = _sogi_run(inp["ea"], inp["eb"], lambda k, o=om: o, dt)
        ta, tq_a, tb, tq_b = _two_integrator_run(inp["ea"], inp["eb"], lambda k, o=om: o, dt)
        mv = _vm(ip_a, ip_b, inp, mask); mt = _vm(ta, tb, inp, mask)
        equiv.append({
            "omega_over_omega_prime": omega_true / om,
            "biquad_gain": mv["fundamental_gain"], "ti_gain": mt["fundamental_gain"],
            "biquad_phase_deg": mv["fundamental_phase_deg"], "ti_phase_deg": mt["fundamental_phase_deg"],
            "rel_gain_diff": abs(mt["fundamental_gain"] - mv["fundamental_gain"]) / abs(mv["fundamental_gain"]),
            "phase_diff_deg": abs(mt["fundamental_phase_deg"] - mv["fundamental_phase_deg"]),
        })
    # (2) closed-loop FLL with the two-integrator SOGI on the same inputs
    om = inp["initial_omega"]
    n = len(inp["ea"])
    freq = np.empty(n); ip_a = np.empty(n); q_a = np.empty(n); ip_b = np.empty(n); q_b = np.empty(n)
    sa, sb = TwoIntegratorSOGI(K_S), TwoIntegratorSOGI(K_S)
    for k in range(n):
        a1, b1 = sa.step(float(inp["ea"][k]), om, dt)
        a2, b2 = sb.step(float(inp["eb"][k]), om, dt)
        eps = b1 * (float(inp["ea"][k]) - a1) + b2 * (float(inp["eb"][k]) - a2)
        om += -GAMMA * eps * dt
        freq[k] = om / (2.0 * math.pi)
        ip_a[k], q_a[k], ip_b[k], q_b[k] = a1, b1, a2, b2
    pns_a, pns_b = _pns(ip_a, q_a, ip_b, q_b)
    f_true = omega_true / (2.0 * math.pi)
    mv = _vm(ip_a, ip_b, inp, mask); mp = _vm(pns_a, pns_b, inp, mask)
    return {
        "constant_omega_equivalence": equiv,
        "max_rel_gain_diff": max(e["rel_gain_diff"] for e in equiv),
        "max_phase_diff_deg": max(e["phase_diff_deg"] for e in equiv),
        "closed_loop_two_integrator": {
            "omega_prime_mean_hz": float(np.mean(freq[mask])),
            "omega_prime_bias_hz": float(np.mean(freq[mask]) - f_true),
            "omega_prime_std_hz": float(np.std(freq[mask])),
            "v_prime_gain": mv["fundamental_gain"], "v_prime_phase_deg": mv["fundamental_phase_deg"],
            "pns_gain": mp["fundamental_gain"], "pns_phase_deg": mp["fundamental_phase_deg"],
        },
        "closed_loop_biquad": {
            "omega_prime_mean_hz": closed["omega_prime_bias_hz"] + f_true,
            "omega_prime_bias_hz": closed["omega_prime_bias_hz"],
            "omega_prime_std_hz": closed["omega_prime_std_hz"],
            "v_prime_gain": closed["vp_metrics"]["fundamental_gain"],
            "v_prime_phase_deg": closed["vp_metrics"]["fundamental_phase_deg"],
            "pns_gain": closed["pns_metrics"]["fundamental_gain"],
            "pns_phase_deg": closed["pns_metrics"]["fundamental_phase_deg"],
        },
        "paper_figure": ("Fig.4 (p.826) shows a SOGI built from two integrators "
                         "(v_o and qv_o) with k_s and omega' gains — the two-integrator form."),
    }


# ---------------------------------------------------------------- Step 3
def _mean_eps_and_split(fund_a, fund_b, res_a, res_b, omega_fixed, dt, mask):
    """Mean eps_f over the window plus its four-term split (linear at frozen omega')."""
    find = lambda k, o=omega_fixed: o
    ipf_a, qf_a, ipf_b, qf_b = _sogi_run(fund_a, fund_b, find, dt)
    ipr_a, qr_a, ipr_b, qr_b = _sogi_run(res_a, res_b, find, dt)
    ef_a = fund_a - ipf_a; ef_b = fund_b - ipf_b
    er_a = res_a - ipr_a; er_b = res_b - ipr_b
    t_ff = qf_a * ef_a + qf_b * ef_b
    t_fr = qf_a * er_a + qf_b * er_b
    t_rf = qr_a * ef_a + qr_b * ef_b
    t_rr = qr_a * er_a + qr_b * er_b
    return {
        "eps_f": float(np.mean(t_ff[mask] + t_fr[mask] + t_rf[mask] + t_rr[mask])),
        "fund_x_fund": float(np.mean(t_ff[mask])),
        "fund_x_ripple": float(np.mean(t_fr[mask])),
        "ripple_x_fund": float(np.mean(t_rf[mask])),
        "ripple_x_ripple": float(np.mean(t_rr[mask])),
    }


def _zero_crossing(ratios, means):
    for i in range(len(means) - 1):
        if means[i] == 0.0:
            return ratios[i]
        if means[i] * means[i + 1] < 0.0:
            frac = means[i] / (means[i] - means[i + 1])
            return float(ratios[i] + frac * (ratios[i + 1] - ratios[i]))
    return None


def step_3(inp, closed):
    dt = inp["dt"]; mask = inp["steady_mask"]; omega_true = inp["omega_e"]
    f_true = omega_true / (2.0 * math.pi)
    # fundamental + residual of the raw E_hat in the steady window
    z = inp["ea"] + 1j * inp["eb"]
    phasor = complex(np.mean(z[mask] * np.exp(-1j * inp["theta"][mask])))
    z_fund = phasor * np.exp(1j * inp["theta"])
    fund_a, fund_b = z_fund.real, z_fund.imag
    res_a, res_b = inp["ea"] - fund_a, inp["eb"] - fund_b
    rows = []
    means = []
    for r in RATIOS_3:
        om = float(r) * omega_true
        m = _mean_eps_and_split(fund_a, fund_b, res_a, res_b, om, dt, mask)
        means.append(m["eps_f"])
        rows.append({"ratio_r": float(r), "omega_prime_hz": float(r) * f_true, **m})
    means = np.asarray(means)
    r_cross = _zero_crossing(list(RATIOS_3), means)
    crossing_hz = None if r_cross is None else r_cross * f_true
    return {
        "rows": rows,
        "zero_crossing_ratio": r_cross,
        "zero_crossing_hz": crossing_hz,
        "zero_crossing_bias_hz": None if crossing_hz is None else crossing_hz - f_true,
        "closed_loop_mean_bias_hz": closed["omega_prime_bias_hz"],
        "ratio_crossing_vs_closed_loop": (
            None if crossing_hz is None else abs((crossing_hz - f_true) - closed["omega_prime_bias_hz"])
            / max(1e-9, abs(closed["omega_prime_bias_hz"]))
        ),
    }


def main() -> int:
    phase02 = _phase02()
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    observer_cfg = config.load_config("observer", directory=directory)
    parameters = PMSMParameters.from_config(pmsm_cfg)
    plant = PMSMPlant(parameters)
    inputs = _build_inputs(plant, phase02, pmsm_cfg, observer_cfg)

    results = {"step2a": {}, "step2b": {}, "step2c": {}, "step3": {}, "observers_seen": []}
    eps_curves = {}
    for key, inp in inputs.items():
        label = f"{inp['observer']}@{inp['ts_us']:.0f}us"
        results["observers_seen"].append(label)
        freq, omega_used, vp_a, vp_b, pns_a, pns_b = _run_closed_loop(inp["ea"], inp["eb"], inp["dt"], inp["initial_omega"])
        close_mask = inp["steady_mask"]
        f_true = inp["omega_e"] / (2.0 * math.pi)
        closed = {
            "omega_used": omega_used, "vp_a": vp_a, "vp_b": vp_b, "pns_a": pns_a, "pns_b": pns_b,
            "omega_prime_bias_hz": float(np.mean(freq[close_mask]) - f_true),
            "omega_prime_std_hz": float(np.std(freq[close_mask])),
            "vp_metrics": _vm(vp_a, vp_b, inp, close_mask),
            "pns_metrics": _vm(pns_a, pns_b, inp, close_mask),
        }
        results["step2a"][label] = step_2a(inp)
        results["step2b"][label] = step_2b(inp, closed)
        results["step2c"][label] = step_2c(inp, closed)
        s3 = step_3(inp, closed)
        results["step3"][label] = s3
        eps_curves[label] = ([r["omega_prime_hz"] for r in s3["rows"]], [r["eps_f"] for r in s3["rows"]],
                             s3["closed_loop_mean_bias_hz"] + f_true)

    output_dir = config.find_project_root() / "experiments" / "output" / "05a_filter_prelude"
    output_dir.mkdir(parents=True, exist_ok=True)

    diff = config_diff.summary(
        "05a_filter_prelude",
        {"epsilon_filter_tau": None, "min_omega": None, "max_omega": None,
         "normalize_error": False, "positive_sequence": True},
        scenario={"ts_us": [t * 1e6 for _, t in INPUTS], "steady_cycles": STEADY_CYCLES,
                  "ratios_2a": list(RATIOS_2A), "ratios_3": [float(r) for r in RATIOS_3]},
    )
    results["config_diff"] = diff
    results["note"] = ("Phase 05a filter prelude; RAW E_hat only, no estimator. FLL at library "
                       "defaults (epsilon_filter_tau=OFF, no clamps, no normalization); initial omega' "
                       "= true electrical frequency at the 500 rpm start.")
    (output_dir / "filter_prelude.json").write_text(json.dumps(results, indent=2), encoding="utf-8")

    # plot epsilon_f(omega') with the closed-loop mean marked
    fig, ax = plt.subplots(figsize=(9, 5))
    for label, (xs, ys, cl_hz) in eps_curves.items():
        ax.plot(xs, ys, marker=".", label=f"{label}")
        ax.axvline(cl_hz, ls=":", alpha=0.5)
    ax.axhline(0.0, color="k", lw=0.8)
    ax.set(xlabel="frozen omega'/2pi (Hz)", ylabel="mean eps_f", title="eps_f(omega') sweep; dotted = closed-loop mean")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "eps_f_sweep.png", dpi=130)
    plt.close(fig)

    config_diff.emit(diff)
    for label in results["observers_seen"]:
        a = results["step2a"][label]
        b = results["step2b"][label]
        c = results["step2c"][label]
        s3 = results["step3"][label]
        print(f"== {label} ==")
        print(f"  2a transfer max gain err {a['max_gain_err']:.3e}, phase err {a['max_phase_err_deg']:.4f} deg")
        print(f"  2b PNS phase: mean {b['i_mean']['pns_phase_deg']:+.2f}, low {b['ii_mean_plus_below_we2']['pns_phase_deg']:+.2f}, "
              f"mid {b['iii_mean_plus_mid']['pns_phase_deg']:+.2f}, high {b['v_mean_plus_high']['pns_phase_deg']:+.2f}, "
              f"full {b['iv_full']['pns_phase_deg']:+.2f} (closed filtered-raw {b['closed_loop']['pns_minus_raw_phase_deg']:+.2f}) repro={b['iv_full_reproduces_closed_loop']}")
        print(f"  2c equiv max relG {c['max_rel_gain_diff']:.2e}, max phase {c['max_phase_diff_deg']:.3e} deg | "
              f"TI closed bias {c['closed_loop_two_integrator']['omega_prime_bias_hz']:+.3f} vs biquad {c['closed_loop_biquad']['omega_prime_bias_hz']:+.3f} Hz")
        print(f"  3  crossing {s3['zero_crossing_hz']} Hz (bias {s3['zero_crossing_bias_hz']}), closed-loop mean bias {s3['closed_loop_mean_bias_hz']:+.3f} Hz")
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
