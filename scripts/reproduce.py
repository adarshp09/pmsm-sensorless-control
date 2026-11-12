#!/usr/bin/env python3
"""Reproducibility harness for pmsm-sensorless-control.

Usage:
    python scripts/reproduce.py --fast    # tests + the four headline reports
    python scripts/reproduce.py --full    # every experiment script, dependency order
    python scripts/reproduce.py --audit   # no runs: just check the outputs exist

Each produced JSON is compared **byte-for-byte** with the version committed in git
(``git show HEAD:<path>``). A mismatch is reported, never overwritten silently: the script
leaves the produced file on disk and prints the diff size.

Per-file tolerances: none by default. ``TOLERANCE`` lists files where a byte-identical match
is not expected; give the reason there or leave the list empty.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXP = ROOT / "experiments"

FAST = [
    "06a_encoder_baseline.py",
    "06b6_gatep.py",
    "06c_comparison.py",
    "06f_replay.py",
]

FULL = [
    "01_plant_validation.py",
    "02_smo_validation.py",
    "02b_smo_verification.py",
    "03_dsogi_fll_validation.py",
    "03b_dsogi_fll_diagnosis.py",
    "03c_stability.py",
    "03d_filter_isolation.py",
    "03e_filter_rebase.py",
    "03f_verification.py",
    "04_flc_adaptation.py",
    "04b_fixed_gain.py",
    "04c_nominal_scenario.py",
    "05a_filter_prelude.py",
    "05b_estimator_eval.py",
    "06a_step0_analysis.py",
    "06a_encoder_baseline.py",
    "06b_step0_analysis.py",
    "06b_gate2.py",
    "06b2_estimator_response.py",
    "06b2_margins.py",
    "06b3_margins.py",
    "06b3_gate.py",
    "06b4_margins.py",
    "06b4_gate.py",
    "06b5_gate.py",
    "06b6_gatep.py",
    "06b6_scenarios.py",
    "06c_comparison.py",
    "06c_deadtime.py",
    "06c2_suite.py",
    "06c2_deadtime.py",
    "06d_diagnosis.py",
    "06e_sweep.py",
    "06e_step1.py",
    "06e_ladder.py",
    "06f_record.py",
    "06f_replay.py",
    "06f_closed.py",
    "06g_figures.py",
    "profile_drive.py",
]

# Files where a byte-identical match is not expected (reason required).
TOLERANCE: dict[str, str] = {}


def _run(cmd: list[str], cwd: Path | None = None) -> tuple[int, float]:
    start = time.perf_counter()
    proc = subprocess.run(cmd, cwd=str(cwd or ROOT))
    return proc.returncode, time.perf_counter() - start


def _committed_matches(path: str) -> bool | None:
    """True/False if the working file matches git HEAD, None if not tracked."""
    proc = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", path],
        cwd=str(ROOT),
        capture_output=True,
        timeout=60,
    )
    if proc.returncode == 0:
        return True
    if proc.returncode == 1:
        return False
    return None


def _tracked_outputs() -> list[str]:
    proc = subprocess.run(
        ["git", "ls-files", "experiments/output"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=60,
    )
    return [p for p in proc.stdout.splitlines() if p.endswith((".json", ".npz"))]


def compare() -> tuple[int, int]:
    rows = []
    for rel in _tracked_outputs():
        disk = ROOT / rel
        if not disk.is_file():
            rows.append((rel, "FAIL", "missing on disk"))
            continue
        same = _committed_matches(rel)
        if same is True:
            rows.append((rel, "PASS", "byte-identical to HEAD"))
        elif rel in TOLERANCE:
            rows.append((rel, "PASS(tol)", TOLERANCE[rel]))
        elif same is False:
            rows.append((rel, "FAIL", "differs from HEAD"))
        else:
            rows.append((rel, "N/A", "not in git HEAD"))
    width = max(len(r[0]) for r in rows)
    print("\n== output comparison vs committed (HEAD) ==")
    for rel, status, note in rows:
        print(f"  {rel:<{width}}  {status:9s}  {note}")
    passed = sum(1 for r in rows if r[1].startswith("PASS"))
    print(f"\n{passed}/{len(rows)} tracked JSON outputs match the committed version.")
    return passed, len(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--fast", action="store_true")
    mode.add_argument("--full", action="store_true")
    mode.add_argument("--audit", action="store_true")
    args = ap.parse_args()

    if args.audit:
        passed, n = compare()
        return 0 if passed == n else 1

    total = 0.0
    if args.fast:
        rc, dt = _run([sys.executable, "-m", "pytest", "-q"])
        total += dt
        print(f"[pytest] exit={rc}  {dt:.1f}s")
        for script in FAST:
            rc, dt = _run([sys.executable, str(EXP / script)])
            total += dt
            print(f"[{script}] exit={rc}  {dt:.1f}s")
    else:
        for script in FULL:
            rc, dt = _run([sys.executable, str(EXP / script)])
            total += dt
            print(f"[{script}] exit={rc}  {dt:.1f}s")

    passed, n = compare()
    print(f"\nwall time: {total:.1f}s")
    return 0 if passed == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
