#!/usr/bin/env python3
"""Phase 06b-2 Step 2: speed-loop margins with the measured estimator response.

L(j w) = L_enc(j w) (06a discrete model: zeta = 1 PI, current loop with its delays,
1 kHz ZOH) x T_est(j w) (measured in Step 1, log-interpolated; no extrapolation).

Sweeps alpha_w in {5 .. 50} Hz for chains A and B (f_n = 20 Hz) at 500 and 1000 rpm and
reports PM, GM, crossover. The ALPHA_W DESIGN RULE (ASSUMPTION, frozen before Step 3):
alpha_w* = the largest grid value with PM >= 45 deg and GM >= 6 dB at BOTH speeds for
chain B at f_n = 20 Hz.

Output: experiments/output/06b2_margins/margins.json
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

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters  # noqa: E402

TWO_PI = 2.0 * math.pi
ALPHA_W_GRID = (5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 50.0)
PM_MIN = 45.0
GM_MIN_DB = 6.0


def _measured(rows: dict, freqs, rpm: float, chain: str):
    g = np.array([rows[f"rpm{int(rpm)}_fm{f:g}"]["chains"][chain]["gain"] for f in freqs])
    p = np.degrees(np.unwrap(np.radians(
        [rows[f"rpm{int(rpm)}_fm{f:g}"]["chains"][chain]["phase_deg"] for f in freqs])))
    return np.array(freqs, dtype=float), g, p


def _T_at(f, freqs, gain, phase_deg):
    """Log-interpolated T_est at ``f`` Hz; refuses extrapolation outside the grid."""
    lf = np.log10(np.asarray(f, dtype=float))
    lgrid = np.log10(freqs)
    if np.any(lf < lgrid[0] - 1e-9) or np.any(lf > lgrid[-1] + 1e-9):
        return None
    g = 10 ** np.interp(lf, lgrid, np.log10(gain))
    ph = np.radians(np.interp(lf, lgrid, phase_deg))
    return g * np.exp(1j * ph)


def _L_enc(f, params: DriveParameters, alpha_w_hz: float):
    """06a discrete speed loop: zeta=1 PI x mechanical ZOH plant x current loop."""
    jw = 1j * TWO_PI * np.asarray(f, dtype=float)
    tsw = params.ts_speed
    z = np.exp(jw * tsw)
    kp = 2.0 * 1.0 * TWO_PI * alpha_w_hz * params.inertia / params.torque_constant
    ki = (TWO_PI * alpha_w_hz) ** 2 * params.inertia / params.torque_constant
    c = kp + ki * tsw / (z - 1.0)
    plant_zoh = params.torque_constant * tsw / params.inertia / (z - 1.0)
    current = (TWO_PI * params.alpha_c_hz) / (jw + TWO_PI * params.alpha_c_hz) \
        * np.exp(-jw * 1.5 * params.ts_current)
    return c * plant_zoh * current * z ** -0.5


def _margins(f, L):
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    crossover = pm = None
    for i in range(len(f) - 1):
        if (mag[i] - 1.0) * (mag[i + 1] - 1.0) <= 0 and mag[i] != mag[i + 1]:
            lo, hi = math.log10(f[i]), math.log10(f[i + 1])
            fc = 10 ** (lo + (hi - lo) * (mag[i] - 1.0) / (mag[i] - mag[i + 1]))
            crossover = float(fc)
            pm = float(180.0 + np.interp(math.log10(fc), np.log10(f), ph))
            break
    gm_db = None
    start = 0 if crossover is None else int(np.searchsorted(f, crossover))
    for i in range(max(start, 0), len(f) - 1):
        if (ph[i] + 180.0) * (ph[i + 1] + 180.0) <= 0 and ph[i] != ph[i + 1]:
            frac = (-180.0 - ph[i]) / (ph[i + 1] - ph[i])
            f180 = 10 ** (math.log10(f[i]) + frac * (math.log10(f[i + 1]) - math.log10(f[i])))
            gm_db = float(-20.0 * np.interp(math.log10(f180), np.log10(f), 20 * np.log10(mag)))
            break
    return {"crossover_hz": crossover, "pm_deg": pm, "gm_db": gm_db}


def main() -> int:
    directory = config.default_config_dir()
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        config.load_config("pmsm", directory=directory))
    resp = json.loads((config.find_project_root() / "experiments" / "output" /
                       "06b2_estimator_response" / "estimator_response.json").read_text())
    rows = resp["rows"]
    freqs = resp["config"]["freqs_hz"]

    fgrid = np.logspace(math.log10(freqs[0]), math.log10(freqs[-1]), 4000)
    table = {}
    for chain in ("A_fn20", "B_fn20"):
        for rpm in (500.0, 1000.0):
            freqs_r, gain, phase = _measured(rows, freqs, rpm, chain)
            for aw in ALPHA_W_GRID:
                T = _T_at(fgrid, freqs_r, gain, phase)
                if T is None:
                    table[f"{chain}|rpm{int(rpm)}|aw{aw:g}"] = {"error": "crossover outside measured range"}
                    continue
                L = _L_enc(fgrid, params, aw) * T
                table[f"{chain}|rpm{int(rpm)}|aw{aw:g}"] = _margins(fgrid, L)

    def ok(entry):
        return (entry.get("pm_deg") is not None and entry["pm_deg"] >= PM_MIN
                and entry.get("gm_db") is not None and entry["gm_db"] >= GM_MIN_DB)

    candidates = [aw for aw in ALPHA_W_GRID
                  if ok(table[f"B_fn20|rpm500|aw{aw:g}"]) and ok(table[f"B_fn20|rpm1000|aw{aw:g}"])]
    alpha_w_star = max(candidates) if candidates else None

    out = {
        "note": ("Phase 06b-2 margins with the measured T_est. L = L_enc (06a discrete model, "
                 "zeta=1 PI, current loop + delays, 1 kHz ZOH) x T_est (measured, log-interp). "
                 "alpha_w* rule frozen before Step 3."),
        "rule": {"pm_min_deg": PM_MIN, "gm_min_db": GM_MIN_DB,
                 "definition": "largest grid alpha_w with PM>=45 and GM>=6 dB at both speeds, chain B f_n=20 Hz"},
        "alpha_w_grid": list(ALPHA_W_GRID),
        "alpha_w_star_hz": alpha_w_star,
        "table": table,
        "time_domain_06b": {"stable_max_alpha_w_hz_constant_load": 15.0,
                            "unstable_at_hz": 20.0,
                            "note": "06b time-domain: constant-load stable <=15 Hz; a load step destabilised even 10 Hz"},
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b2_margins"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "margins.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== margins (PM deg / GM dB / crossover Hz) ==")
    for chain in ("A_fn20", "B_fn20"):
        for rpm in (500.0, 1000.0):
            print(f"  {chain} @ {int(rpm)} rpm")
            for aw in ALPHA_W_GRID:
                e = table[f"{chain}|rpm{int(rpm)}|aw{aw:g}"]
                if "error" in e:
                    print(f"     aw={aw:4g} Hz: {e['error']}"); continue
                print(f"     aw={aw:4g} Hz: PM={e['pm_deg']:7.1f}  GM={('%6.1f' % e['gm_db']) if e['gm_db'] is not None else '   inf'}  "
                      f"fc={('%6.1f' % e['crossover_hz']) if e['crossover_hz'] else '  none'}")
    print(f"\n== alpha_w* (chain B, f_n=20, PM>={PM_MIN} & GM>={GM_MIN_DB} dB at both speeds) ==")
    print(f"  candidates={candidates} -> alpha_w* = {alpha_w_star} Hz")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
