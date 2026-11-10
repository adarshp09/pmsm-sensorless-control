"""Shared Phase 06b-6 sensorless-loop evaluation helpers (experiments only).

Chain B (DSOGI v' -> Q-PLL), matched sigmoid observer, handover as in Phase 06b.
Records the post-handover estimation errors and the true-speed metrics.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

_DIR = Path(__file__).resolve().parent
if str(_DIR) not in sys.path:
    sys.path.insert(0, str(_DIR))

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

import chain as C  # noqa: E402

TWO_PI = 2.0 * math.pi
RATED_TORQUE = 7.7


class ChainRecorder(C.HandoverSource):
    """HandoverSource that records post-handover estimation errors."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.t: list[float] = []
        self.theta_err_deg: list[float] = []
        self.omega_err_rpm: list[float] = []

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        if self.mode != "encoder":
            self.t.append(sample.t)
            self.theta_err_deg.append(math.degrees((out[0] - truth.theta_e + math.pi) % TWO_PI - math.pi))
            self.omega_err_rpm.append((out[1] - truth.omega_e) * 60.0 / (TWO_PI * self.pole_pairs))
        return out


def load(config_dir=None):
    directory = config_dir if config_dir is not None else config.default_config_dir()
    stack = C.build_stack(directory)
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        config.load_config("pmsm", directory=directory))
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config), integrate_mechanics=True,
                      viscous_friction=0.0)
    cfg = config.load_config("drive_assumptions", directory=directory)
    rule = {500: config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_500rpm", unit="Hz"),
            1000: config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_1000rpm", unit="Hz")}
    return stack, params, plant, rule


def run_sensorless(stack, params, plant, *, speed_ref, load_torque, duration, alpha_w,
                   advance=False, clamp=True, cell="matched_sigmoid", f_n=None,
                   initial_state=None):
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    assump = C.handover_assumptions(stack)
    min_omega = C.fll_min_omega_rad_s(stack) if clamp else None
    fll = C.build_fll(stack.initial_omega_at(assump.omega_h_rpm), min_omega=min_omega)
    if f_n is None:
        qpll = C.build_qpll(stack)
    else:
        qpll = QPLL(QPLLParameters.from_design(zeta=0.707, f_n_hz=f_n,
                                              amplitude_v=stack.design_amp_v))
    chain_source = C.EstimatorSource(observer=C.build_observer(cell, stack), fll=fll,
                                     qpll=qpll, pole_pairs=stack.pole_pairs)
    source = ChainRecorder(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(plant, prm, duration=duration, speed_ref_rpm=speed_ref,
                    load_torque=load_torque, source=source,
                    angle_advance=advance, initial_state=initial_state)
    return source, chain_source, res


def paper_scenarios(rule):
    """The four printed scenarios + Table IV steady run (Phase 06b-6 Step 0d)."""
    from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque

    def ramp(t):
        if t < 0.5:
            return 500.0
        if t < 1.5:
            return 500.0 + (t - 0.5) * 500.0
        return 1000.0

    return {
        "S1_no_load_ramp": dict(
            speed_ref=ramp, load=ConstantLoadTorque(0.0), duration=2.5, alpha_w=rule[500],
            dynamic=(0.5, 1.5), steady=(2.0, 2.5), disturbance_s=0.5,
            paper="max speed err ~5 rpm; steady pos ~0.07 rad; dynamic pos ~0.2 rad"),
        "S2_sim_50pct_1000": dict(
            speed_ref=1000.0, load=StepLoadTorque(0.0, 0.5 * RATED_TORQUE, 0.5), duration=2.0,
            alpha_w=rule[1000], dynamic=(0.5, 1.0), steady=(1.8, 2.0), disturbance_s=0.5,
            paper="dynamic max speed 23 rpm; steady mean 5 rpm; dynamic pos 0.22 rad; steady pos 0.08-0.1 rad"),
        "S3_exp_20pct_1000": dict(
            speed_ref=1000.0, load=StepLoadTorque(0.0, 0.2 * RATED_TORQUE, 0.5), duration=2.0,
            alpha_w=rule[1000], dynamic=(0.5, 1.0), steady=(1.8, 2.0), disturbance_s=0.5,
            paper="speed err ~10 rpm; position err 0.152 rad"),
        "S4_steady_1000": dict(
            speed_ref=1000.0, load=ConstantLoadTorque(0.0), duration=1.6, alpha_w=rule[1000],
            dynamic=(0.6, 1.6), steady=(0.6, 1.6), disturbance_s=None,
            paper="0.08 rad, 10 rpm (Table IV)"),
    }


def run_encoder(stack, params, plant, *, speed_ref, load_torque, duration, alpha_w,
                advance=False, initial_state=None):
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    return run_drive(plant, prm, duration=duration, speed_ref_rpm=speed_ref,
                     load_torque=load_torque, angle_advance=advance,
                     initial_state=initial_state)


def _err_stats(values):
    if not len(values):
        return {"mean": None, "rms": None, "max": None}
    a = np.abs(np.asarray(values))
    return {"mean": float(np.mean(a)), "rms": float(np.sqrt(np.mean(a ** 2))), "max": float(np.max(a))}


def speed_metrics(stack, res, windows):
    """True-speed dip / overshoot / recovery over a disturbance window."""
    err = (res.omega_m - res.omega_m_ref) * 60.0 / TWO_PI
    ref = float(np.mean(np.asarray(res.omega_m_ref))) * 60.0 / TWO_PI
    step = windows["disturbance_s"]
    after = res.t >= step
    out = {"dip_rpm": float(np.min(err[after])), "overshoot_rpm": float(np.max(err[after]))}
    for label, thresh in (("recovery_10rpm_s", 10.0), ("recovery_1pct_s", 0.01 * abs(ref))):
        bad = np.where(np.abs(err[after]) > thresh)[0]
        out[label] = (float(res.t[np.where(after)[0][bad[-1] + 1]] - step)
                      if len(bad) and bad[-1] + 1 < int(after.sum()) else 0.0)
    return out


def estimate_metrics(source, chain_source, res, *, dynamic, steady):
    t = np.asarray(source.t) if source.t else np.zeros(0)
    th = np.asarray(source.theta_err_deg) if source.theta_err_deg else np.zeros(0)
    om = np.asarray(source.omega_err_rpm) if source.omega_err_rpm else np.zeros(0)
    switch = source.switch_time_s

    def win(lo, hi):
        m = (t >= lo) & (t <= hi)
        return _err_stats(th[m]), _err_stats(om[m])

    dyn_th, dyn_om = win(*dynamic)
    st_th, st_om = win(*steady)
    post = th[t >= (switch + 0.2)] if switch is not None else np.zeros(0)
    return {
        "handed_over": bool(source.handed_over), "switch_time_s": switch,
        "fll_status": chain_source.status, "guard_fail_time_s": chain_source.first_guard_fail_s,
        "peak_theta_err_deg": float(np.max(np.abs(th))) if th.size else None,
        "steady_theta_err_max_deg": float(np.max(np.abs(post))) if post.size else None,
        "dynamic_theta_err": dyn_th, "dynamic_omega_err": dyn_om,
        "steady_theta_err": st_th, "steady_omega_err": st_om,
        "peak_i_q_a": float(np.max(np.abs(res.i_q))),
        "saturation_fraction": float(np.mean(res.saturated)),
        "saturation_after_transient": float(np.mean(res.saturated[res.t >= steady[0]])),
        "peak_voltage_v": float(np.max(res.u_cmd_magnitude)), "v_limit_v": res.v_limit,
    }
