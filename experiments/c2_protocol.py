"""Phase 06c-2 shared comparison protocol (experiments only).

Fixed-time handover t_h (ASSUMPTION): every chain runs in the background from the encoder
start-up and hands over at t_h = 0.30 s, independent of any lock criterion. A chain is
*lock-valid* if max |theta_hat - theta| over [t_h-50 ms, t_h] < 30 deg — computed in the
evaluation layer from the background estimate, never used by the controller.
"""

from __future__ import annotations

import math

import numpy as np

import chain as C
import sensorless_eval as E

T_H = 0.30
LOCK_LIMIT_DEG = 30.0
LOCK_WINDOW_S = 0.05
TWO_PI = 2.0 * math.pi


class FullRecorder(C.HandoverSource):
    """HandoverSource recording the background estimate vs truth for all samples."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.t: list[float] = []
        self.theta_err: list[float] = []
        self.omega_err: list[float] = []
        self.post: list[bool] = []

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        self.t.append(sample.t)
        self.theta_err.append(
            math.degrees((self.chain.theta_hat - truth.theta_e + math.pi) % TWO_PI - math.pi)
        )
        self.omega_err.append(
            (self.chain.omega_hat - truth.omega_e) * 60.0 / (TWO_PI * self.pole_pairs)
        )
        self.post.append(self.mode != "encoder")
        return out


def run_fixed(
    builder,
    stack,
    params,
    plant,
    spec,
    *,
    alpha_w,
    advance=False,
    dead_time=0.0,
    compensation=False,
    t_h=T_H,
):
    import dataclasses

    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    source = builder()
    rec = FullRecorder(
        source,
        pole_pairs=stack.pole_pairs,
        assumptions=C.handover_assumptions(stack),
        fixed_time_s=t_h,
    )
    from pmsm_sensorless.simulation.drive import run_drive

    res = run_drive(
        plant,
        prm,
        duration=spec["duration"],
        speed_ref_rpm=spec["speed_ref"],
        load_torque=spec["load"],
        source=rec,
        angle_advance=advance,
        dead_time_s=dead_time,
        dead_time_compensation=compensation,
    )
    return rec, source, res


def lock_valid(rec, t_h=T_H):
    t = np.asarray(rec.t) if rec.t else np.zeros(0)
    e = np.abs(np.asarray(rec.theta_err)) if rec.theta_err else np.zeros(0)
    m = (t >= t_h - LOCK_WINDOW_S) & (t < t_h)
    return bool(m.any() and np.max(e[m]) < LOCK_LIMIT_DEG), (
        float(np.max(e[m])) if m.any() else None
    )


def metrics(rec, source, res, *, dynamic, steady, disturbance_s):
    t = np.asarray(rec.t) if rec.t else np.zeros(0)
    post = np.asarray(rec.post, dtype=bool) if rec.post else np.zeros(0, dtype=bool)
    th = np.asarray(rec.theta_err) if rec.theta_err else np.zeros(0)
    om = np.asarray(rec.omega_err) if rec.omega_err else np.zeros(0)

    def stats(v):
        return {
            "mean": float(np.mean(np.abs(v))) if v.size else None,
            "rms": float(np.sqrt(np.mean(v**2))) if v.size else None,
            "max": float(np.max(np.abs(v))) if v.size else None,
        }

    def win(lo, hi):
        m = (t >= lo) & (t <= hi) & post
        return stats(th[m]), stats(om[m])

    dyn_th, dyn_om = win(*dynamic)
    st_th, st_om = win(*steady)
    out = {
        "handed_over": bool(rec.handed_over),
        "switch_time_s": rec.switch_time_s,
        "fll_status": source.status,
        "peak_theta_err_deg": stats(th[post])["max"],
        "steady_theta": st_th,
        "steady_omega": st_om,
        "dynamic_theta": dyn_th,
        "dynamic_omega": dyn_om,
        "peak_i_q_a": float(np.max(np.abs(res.i_q))),
        "saturation_after_transient": float(np.mean(res.saturated[res.t >= steady[0]])),
        "peak_voltage_v": float(np.max(res.u_cmd_magnitude)),
        "v_limit_v": res.v_limit,
    }
    if disturbance_s is not None:
        out.update(E.speed_metrics(None, res, {"disturbance_s": disturbance_s}))
    # current ripple (RMS about the fundamental fit) over the steady window
    m = res.t >= steady[0]
    z = res.i_alpha[m] + 1j * res.i_beta[m]
    if z.size:
        ph = np.mean(z * np.exp(-1j * res.theta_e[m]))
        out["current_ripple_rms_a"] = float(
            np.sqrt(np.mean(np.abs(z - ph * np.exp(1j * res.theta_e[m])) ** 2))
        )
    return out
