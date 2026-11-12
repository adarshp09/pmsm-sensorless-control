"""Tests for the Phase 03e config-diff guard (experiments/config_diff.py)."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

import config_diff  # noqa: E402


def test_reports_epsilon_filter_tau_when_set() -> None:
    block = config_diff.summary("x", {"epsilon_filter_tau": 0.01})
    assert "epsilon_filter_tau" in block["non_default_options"]
    assert block["non_default_options"]["epsilon_filter_tau"]["used"] == 0.01
    assert block["non_default_options"]["epsilon_filter_tau"]["default"] is None
    assert any("epsilon_filter_tau" in line for line in config_diff.describe(block))


def test_reports_all_defaults_when_none_overridden() -> None:
    block = config_diff.summary("x", {"epsilon_filter_tau": None, "normalize_error": False})
    assert block["non_default_options"] == {}
    assert "all filter/observer options at library defaults" in config_diff.describe(block)[1]


def test_reports_clamp_and_normalization_overrides() -> None:
    block = config_diff.summary(
        "x", {"min_omega": 12.0, "normalize_error": True, "positive_sequence": False}
    )
    assert set(block["non_default_options"]) == {
        "min_omega",
        "normalize_error",
        "positive_sequence",
    }
