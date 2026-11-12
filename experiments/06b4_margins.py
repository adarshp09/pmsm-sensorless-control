#!/usr/bin/env python3
"""Phase 06b-4 Step 1: verify the softened 500 rpm bandwidth (alpha_w = 2.5 Hz).

Reuses the Phase 06b-2 measured T_est (chain B = DSOGI v' -> Q-PLL, f_n = 20 Hz) and the
discrete open-loop model L = L_enc x T_est. Records PM/GM/crossover for a small grid
around the softened point and checks the target PM >= 30 deg at 500 rpm.

Output: experiments/output/06b4_margins/margins.json
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters  # noqa: E402


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


M = _load("m06b3", "06b3_margins.py")
GRID = (1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.5, 10.0)
PM_TARGET_DEG = 30.0
GM_TARGET_DB = 6.0


def main() -> int:
    directory = config.default_config_dir()
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        config.load_config("pmsm", directory=directory),
    )
    resp = json.loads(
        (
            config.find_project_root()
            / "experiments"
            / "output"
            / "06b2_estimator_response"
            / "estimator_response.json"
        ).read_text()
    )
    cfg = config.load_config("drive_assumptions", directory=directory)
    alpha_500 = config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_500rpm", unit="Hz")
    alpha_1000 = config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_1000rpm", unit="Hz")

    grid = {}
    for aw in GRID:
        grid[f"rpm500_aw{aw:g}"] = M._evaluate(params, resp, 500.0, aw)
    grid[f"rpm1000_aw{alpha_1000:g}"] = M._evaluate(params, resp, 1000.0, alpha_1000)

    softened = grid[f"rpm500_aw{alpha_500:g}"]
    pm_ok = softened.get("pm_deg") is not None and softened["pm_deg"] >= PM_TARGET_DEG
    gm_ok = softened.get("gm_db") is not None and softened["gm_db"] >= GM_TARGET_DB
    out = {
        "note": (
            "Phase 06b-4 Step 1: verify the softened 500 rpm bandwidth (2.5 Hz) with "
            "the 06b-2 measured T_est; L = L_enc x T_est, chain B, f_n = 20 Hz."
        ),
        "rule": {"alpha_w_500_hz": alpha_500, "alpha_w_1000_hz": alpha_1000},
        "targets": {"pm_min_deg": PM_TARGET_DEG, "gm_min_db": GM_TARGET_DB},
        "softened_500rpm": softened,
        "softened_500rpm_pass": bool(pm_ok and gm_ok),
        "grid_500rpm": {k: v for k, v in grid.items() if k.startswith("rpm500")},
        "regression_1000rpm": grid[f"rpm1000_aw{alpha_1000:g}"],
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b4_margins"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "margins.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== 06b-4 margins (chain B, f_n=20 Hz) ==")
    print(f"  rule: alpha_w(500) = {alpha_500:g} Hz, alpha_w(1000) = {alpha_1000:g} Hz")
    for aw in GRID:
        e = grid[f"rpm500_aw{aw:g}"]
        mark = "  <-- softened" if aw == alpha_500 else ""
        print(
            f"  rpm500  aw={aw:4g} Hz: PM={e.get('pm_deg'):7.1f}  "
            f"GM={('%.1f' % e['gm_db']) if e.get('gm_db') is not None else 'inf':>6s}  "
            f"fc={('%.1f' % e['crossover_hz']) if e.get('crossover_hz') else 'none':>6s}{mark}"
        )
    e = out["regression_1000rpm"]
    print(
        f"  rpm1000 aw={alpha_1000:4g} Hz: PM={e.get('pm_deg'):7.1f}  "
        f"GM={('%.1f' % e['gm_db']) if e.get('gm_db') is not None else 'inf':>6s}  "
        f"fc={('%.1f' % e['crossover_hz']) if e.get('crossover_hz') else 'none':>6s}"
    )
    print(
        f"\n  target PM >= {PM_TARGET_DEG:.0f} deg at {alpha_500:g} Hz: "
        f"{'PASS' if out['softened_500rpm_pass'] else 'FAIL'}"
    )
    print(f"Outputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
