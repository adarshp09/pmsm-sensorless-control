#!/usr/bin/env python3
"""Phase 06b-6 Step 1: Gate P on the paper's scenarios (chain B, matched cell, clamp ON).

(i) no-load 500->1000 ramp (incl. handover); (ii) sim 50 % step @1000; (iii) exp 20 %
step @1000; (iv) steady 1000 rpm (Table IV). Pass: |theta_hat-theta| < 30 deg throughout,
no voltage saturation after the transient, no FLL guard trip. Ablation: clamp OFF.

Output: experiments/output/06b6_gatep/gatep.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402

ANGLE_LIMIT_DEG = 30.0


def _run(stack, params, plant, spec, *, advance, clamp):
    source, chain_source, res = E.run_sensorless(
        stack, params, plant, speed_ref=spec["speed_ref"], load_torque=spec["load"],
        duration=spec["duration"], alpha_w=spec["alpha_w"], advance=advance, clamp=clamp)
    m = E.estimate_metrics(source, chain_source, res,
                           dynamic=spec["dynamic"], steady=spec["steady"])
    if spec["disturbance_s"] is not None:
        m.update(E.speed_metrics(stack, res, {"disturbance_s": spec["disturbance_s"]}))
    m["passed"] = bool(
        m["handed_over"] and m["fll_status"] == "ok"
        and (m["peak_theta_err_deg"] is not None and m["peak_theta_err_deg"] < ANGLE_LIMIT_DEG)
        and m["saturation_after_transient"] == 0.0)
    return m


def main() -> int:
    stack, params, plant, rule = E.load()
    scenarios = E.paper_scenarios(rule)
    results = {}
    for name, spec in scenarios.items():
        for advance in (False, True):
            for clamp in (True, False):
                key = f"{name}|adv{'on' if advance else 'off'}|clamp{'on' if clamp else 'off'}"
                results[key] = _run(stack, params, plant, spec, advance=advance, clamp=clamp)

    # Gate P: clamp ON, advance OFF (advance ON reported)
    gate_keys = [k for k, _ in results.items() if k.endswith("|advoff|clampon")]
    gate_pass = all(results[k]["passed"] for k in gate_keys)
    out = {
        "note": ("Phase 06b-6 Gate P: paper scenarios S1-S4, chain B, matched cell, "
                 "clamp ON, advance off/on; ablation clamp OFF. Pass: peak |theta_hat-theta| "
                 "< 30 deg, no saturation after transient, no FLL guard trip."),
        "rule": {str(k): v for k, v in rule.items()},
        "gate": "PASS" if gate_pass else "FAIL",
        "results": results,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b6_gatep"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gatep.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== GATE P (06b-6): {out['gate']} ==")
    for key, m in results.items():
        th = m["steady_theta_err"]["max"]
        print(f"  {key:38s} hand={m['handed_over']} fll={m['fll_status']:9s} "
              f"peak={m['peak_theta_err_deg'] if m['peak_theta_err_deg'] is not None else -1:7.1f} "
              f"steady_max={th if th is not None else -1:6.1f} sat_after={m['saturation_after_transient']:.3f} "
              f"{'PASS' if m['passed'] else 'FAIL'}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
