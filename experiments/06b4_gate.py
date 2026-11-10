#!/usr/bin/env python3
"""Phase 06b-4 Step 2: staged gate at the softened low-speed bandwidth.

Rule (configs/drive_assumptions.yaml): alpha_w = 2.5 Hz at 500 rpm, 10 Hz at 1000 rpm
(D-061). Chain B (DSOGI v' -> Q-PLL, f_n = 20 Hz), matched cell, handover as in 06b,
advance off and on.

500 rpm: 2a (steady, no-load and 20 % load) and 2b (20 % load step).
1000 rpm: regression 2a/2b/2c.

Reuses the Phase 06b-3 gate machinery.

Output: experiments/output/06b4_gate/gate.json
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
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters  # noqa: E402


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = _load("g06b3", "06b3_gate.py")
RATED = G.RATED_TORQUE


def main() -> int:
    directory = config.default_config_dir()
    stack = G.C.build_stack(directory)
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        stack.pmsm_config)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config),
                      integrate_mechanics=True, viscous_friction=0.0)
    rule = G._rule(directory)

    results = {}
    # 500 rpm: 2a (no-load, 20 % load) and 2b (20 % load step)
    for advance in (False, True):
        for load_frac in (0.0, 0.2):
            src, cs, res = G._run(stack, params, plant, speed_rpm=500,
                                  load=ConstantLoadTorque(load_frac * RATED), speed_ref=500.0,
                                  duration=1.6, advance=advance, alpha_w=rule[500])
            results[f"2a|500rpm|load{int(load_frac*100)}|adv{'on' if advance else 'off'}"] = G._gate_entry(
                src, cs, res, alpha_w=rule[500], advance=advance, speed_rpm=500)
        src, cs, res = G._run(stack, params, plant, speed_rpm=500,
                              load=StepLoadTorque(0.0, 0.2 * RATED, 0.5), speed_ref=500.0,
                              duration=2.0, advance=advance, alpha_w=rule[500])
        results[f"2b|500rpm|load20|adv{'on' if advance else 'off'}"] = G._gate_entry(
            src, cs, res, step_s=0.5, alpha_w=rule[500], advance=advance, speed_rpm=500,
            require_steady_deg=None)
        # 1000 rpm regression (2a no-load/20 %, 2b 20 %, 2c 50 %)
        for load_frac in (0.0, 0.2):
            src, cs, res = G._run(stack, params, plant, speed_rpm=1000,
                                  load=ConstantLoadTorque(load_frac * RATED), speed_ref=1000.0,
                                  duration=1.6, advance=advance, alpha_w=rule[1000])
            results[f"reg2a|1000rpm|load{int(load_frac*100)}|adv{'on' if advance else 'off'}"] = G._gate_entry(
                src, cs, res, alpha_w=rule[1000], advance=advance, speed_rpm=1000)
        for tag, frac in (("reg2b", 0.2), ("reg2c", 0.5)):
            src, cs, res = G._run(stack, params, plant, speed_rpm=1000,
                                  load=StepLoadTorque(0.0, frac * RATED, 0.5), speed_ref=1000.0,
                                  duration=2.0, advance=advance, alpha_w=rule[1000])
            results[f"{tag}|1000rpm|load{int(frac*100)}|adv{'on' if advance else 'off'}"] = G._gate_entry(
                src, cs, res, step_s=0.5, alpha_w=rule[1000], advance=advance, speed_rpm=1000,
                require_steady_deg=None)

    off = [k for k in results if k.endswith("advoff")]
    gate = all(results[k]["passed"] for k in off)
    out = {
        "note": ("Phase 06b-4 gate at the softened low-speed bandwidth (500 rpm: 2.5 Hz, "
                 "1000 rpm: 10 Hz); chain B, matched cell, f_n = 20 Hz. Advance off/on."),
        "rule": {str(k): v for k, v in rule.items()},
        "gate": "PASS" if gate else "FAIL",
        "results": results,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b4_gate"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gate.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== GATE (06b-4): {out['gate']} ==  rule={out['rule']}")
    for key, e in results.items():
        dip = f" dip={e['speed_dip_rpm']:7.1f} rec10={e['recovery_to_10rpm_s']}" if "speed_dip_rpm" in e else ""
        print(f"  {key:30s} hand={e['handed_over']} fll={e['fll_status']:9s} "
              f"peak={e['peak_theta_err_deg']:7.1f} steady={e['steady_theta_err_max_deg'] if e['steady_theta_err_max_deg'] is not None else -1:7.1f} "
              f"sat={e['saturation_frac_after_transient']:.3f}{dip} {'PASS' if e['passed'] else 'FAIL'}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
