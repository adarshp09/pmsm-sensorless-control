#!/usr/bin/env python3
"""Phase 06b-3 Step 1: verify the operating-point decoupled speed-loop bandwidths.

Verifies the proposed rule (alpha_w = 5 Hz at 500 rpm with the relaxed PM >= 10 deg, and
alpha_w = 10 Hz at 1000 rpm with PM >= 45 deg / GM >= 6 dB) against the discrete
open-loop model L(j w) = L_enc(j w) x T_est(j w), using the T_est measured in Phase 06b-2
(chain B = DSOGI v' -> Q-PLL, f_n = 20 Hz; log-interpolated, no extrapolation).

Output: experiments/output/06b3_margins/margins.json
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
CHAIN = "B_fn20"
ALPHA_W_GRID = (5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 50.0)


def _measured(rows, freqs, rpm, chain=CHAIN):
    g = np.array([rows[f"rpm{int(rpm)}_fm{f:g}"]["chains"][chain]["gain"] for f in freqs], dtype=float)
    p = np.degrees(np.unwrap(np.radians(
        [rows[f"rpm{int(rpm)}_fm{f:g}"]["chains"][chain]["phase_deg"] for f in freqs])))
    return np.array(freqs, dtype=float), g, p


def _T_at(f, freqs, gain, phase_deg):
    lf = np.log10(np.asarray(f, dtype=float))
    lg = np.log10(freqs)
    if np.any(lf < lg[0] - 1e-9) or np.any(lf > lg[-1] + 1e-9):
        return None
    return 10 ** np.interp(lf, lg, np.log10(gain)) * np.exp(
        1j * np.radians(np.interp(lf, lg, phase_deg)))


def _L_enc(f, params, alpha_w_hz):
    jw = 1j * TWO_PI * np.asarray(f, dtype=float)
    tsw = params.ts_speed
    z = np.exp(jw * tsw)
    kp = 2.0 * TWO_PI * alpha_w_hz * params.inertia / params.torque_constant
    ki = (TWO_PI * alpha_w_hz) ** 2 * params.inertia / params.torque_constant
    c = kp + ki * tsw / (z - 1.0)
    plant = params.torque_constant * tsw / params.inertia / (z - 1.0)
    current = (TWO_PI * params.alpha_c_hz) / (jw + TWO_PI * params.alpha_c_hz) \
        * np.exp(-jw * 1.5 * params.ts_current)
    return c * plant * current * z ** -0.5


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
    gm = None
    start = 0 if crossover is None else int(np.searchsorted(f, crossover))
    for i in range(start, len(f) - 1):
        if (ph[i] + 180.0) * (ph[i + 1] + 180.0) <= 0 and ph[i] != ph[i + 1]:
            frac = (-180.0 - ph[i]) / (ph[i + 1] - ph[i])
            f180 = 10 ** (math.log10(f[i]) + frac * (math.log10(f[i + 1]) - math.log10(f[i])))
            gm = float(-20.0 * np.interp(math.log10(f180), np.log10(f), 20 * np.log10(mag)))
            break
    return {"crossover_hz": crossover, "pm_deg": pm, "gm_db": gm}


def _evaluate(params, resp, rpm, alpha_w):
    freqs = resp["config"]["freqs_hz"]
    f = np.logspace(math.log10(freqs[0]), math.log10(freqs[-1]), 4000)
    fr, g, p = _measured(resp["rows"], freqs, rpm)
    T = _T_at(f, fr, g, p)
    if T is None:
        return {"error": "outside measured range"}
    return _margins(f, _L_enc(f, params, alpha_w) * T)


def main() -> int:
    directory = config.default_config_dir()
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        config.load_config("pmsm", directory=directory))
    resp = json.loads((config.find_project_root() / "experiments" / "output" /
                       "06b2_estimator_response" / "estimator_response.json").read_text())

    # proposed operating-point decoupled rule
    proposed = {"500": 5.0, "1000": 10.0}
    criteria = {"500": {"pm_min_deg": 10.0, "gm_min_db": 6.0},
                "1000": {"pm_min_deg": 45.0, "gm_min_db": 6.0}}

    grid = {}
    for rpm in (500.0, 1000.0):
        for aw in ALPHA_W_GRID:
            grid[f"rpm{int(rpm)}_aw{aw:g}"] = _evaluate(params, resp, rpm, aw)

    checks = {}
    for rpm_s, aw in proposed.items():
        entry = _evaluate(params, resp, float(rpm_s), aw)
        c = criteria[rpm_s]
        pm_ok = entry.get("pm_deg") is not None and entry["pm_deg"] >= c["pm_min_deg"]
        gm_ok = entry.get("gm_db") is not None and entry["gm_db"] >= c["gm_min_db"]
        checks[rpm_s] = {"alpha_w_hz": aw, "criteria": c, "margins": entry,
                         "pm_ok": bool(pm_ok), "gm_ok": bool(gm_ok), "pass": bool(pm_ok and gm_ok)}

    # largest grid alpha_w at 1000 rpm meeting PM>=45 & GM>=6 dB
    eligible = [aw for aw in ALPHA_W_GRID
                if (grid[f"rpm1000_aw{aw:g}"].get("pm_deg") or -1e9) >= 45.0
                and (grid[f"rpm1000_aw{aw:g}"].get("gm_db") or -1e9) >= 6.0]
    largest_1000_pm45 = max(eligible) if eligible else None

    verdict = "PASS" if all(c["pass"] for c in checks.values()) else "FAIL"
    out = {
        "note": ("Phase 06b-3 Step 1: verify the operating-point decoupled speed-loop rule "
                 "against L = L_enc x T_est (chain B, f_n=20 Hz; 06b-2 T_est). No extrapolation."),
        "proposed_rule": {"rpm500": 5.0, "rpm1000": 10.0,
                          "criteria": criteria},
        "checks": checks,
        "grid": grid,
        "largest_alpha_w_1000rpm_with_pm45": largest_1000_pm45,
        "verdict": verdict,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b3_margins"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "margins.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== proposed operating-point decoupled rule ==")
    for rpm_s, chk in checks.items():
        m = chk["margins"]
        print(f"  {rpm_s:>4s} rpm, alpha_w={chk['alpha_w_hz']:4g} Hz: "
              f"PM={m.get('pm_deg'):7.1f} (min {chk['criteria']['pm_min_deg']:.0f})  "
              f"GM={('%.1f' % m['gm_db']) if m.get('gm_db') is not None else 'inf':>6s} dB  "
              f"fc={('%.1f' % m['crossover_hz']) if m.get('crossover_hz') else 'none':>6s} Hz  "
              f"-> {'PASS' if chk['pass'] else 'FAIL'}")
    print(f"\n  largest alpha_w at 1000 rpm with PM>=45 & GM>=6 dB: {largest_1000_pm45} Hz")
    print("\n== full grid (chain B, f_n=20) ==")
    for key, m in grid.items():
        print(f"  {key:14s} PM={m.get('pm_deg'):7.1f}  GM={('%.1f' % m['gm_db']) if m.get('gm_db') is not None else 'inf':>6s}  "
              f"fc={('%.1f' % m['crossover_hz']) if m.get('crossover_hz') else 'none':>6s}")
    print(f"\n== VERDICT: {verdict} ==")
    print(f"Outputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
