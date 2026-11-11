#!/usr/bin/env python3
"""Audit or regenerate the Phase 06 experiment reports.

The default action is a fast audit of the expected JSON/NPZ outputs. Use
``--run`` to execute the reports in dependency order. The 06f record must
precede replay and closed-loop attribution.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPERIMENTS = ROOT / "experiments"

REPORTS = (
    ("06a_encoder_baseline.py", "experiments/output/06a_encoder_baseline/encoder_baseline.json"),
    ("06b6_gatep.py", "experiments/output/06b6_gatep/gatep.json"),
    ("06b6_scenarios.py", "experiments/output/06b6_scenarios/scenarios.json"),
    ("06c2_suite.py", "experiments/output/06c2_suite/suite.json"),
    ("06c_comparison.py", "experiments/output/06c_comparison/comparison.json"),
    ("06d_diagnosis.py", "experiments/output/06d_diagnosis/diagnosis.json"),
    ("06e_ladder.py", "experiments/output/06e_ladder/ladder.json"),
    ("06e_step1.py", "experiments/output/06e_step1/lockpoint.json"),
    ("06e_sweep.py", "experiments/output/06e_sweep/sweep.json"),
    ("06f_record.py", "experiments/output/06f_record/record.json"),
    ("06f_replay.py", "experiments/output/06f_replay/replay.json"),
    ("06f_closed.py", "experiments/output/06f_closed/closed.json"),
)


def audit() -> int:
    missing = [output for _, output in REPORTS if not (ROOT / output).is_file()]
    if missing:
        print("Missing Phase 06 outputs:")
        print("\n".join(f"  {item}" for item in missing))
        return 1
    print(f"Phase 06 report audit: {len(REPORTS)} outputs present")
    return 0


def regenerate() -> int:
    for script, _ in REPORTS:
        print(f"\n== {script} ==")
        result = subprocess.run(
            [sys.executable, str(EXPERIMENTS / script)],
            cwd=ROOT,
            check=False,
        )
        if result.returncode:
            print(f"FAILED: {script} (exit {result.returncode})")
            return result.returncode
    return audit()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="rerun all reports in dependency order")
    args = parser.parse_args()
    return regenerate() if args.run else audit()


if __name__ == "__main__":
    raise SystemExit(main())
