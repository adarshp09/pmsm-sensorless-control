"""Tests for the strict, unit-aware parameter accessors added in Phase 01.

These guard the rule that an unresolved (``null``) paper value must surface as a
loud :class:`~pmsm_sensorless.config.ConfigError`, never as a silent default.
"""

from __future__ import annotations

import pytest

from pmsm_sensorless import config
from pmsm_sensorless.config import ConfigError


@pytest.fixture
def pmsm_cfg(config_dir):
    return config.load_config("pmsm", directory=config_dir)


def test_resolve_parameter_returns_resolved_value(pmsm_cfg) -> None:
    assert config.resolve_parameter(pmsm_cfg, "machine.stator_resistance_rs", unit="ohm") == 0.505
    assert config.resolve_parameter(pmsm_cfg, "machine.pole_pairs", unit="-") == 4


def test_get_parameter_returns_none_when_unresolved(pmsm_cfg) -> None:
    assert config.get_parameter(pmsm_cfg, "mech.viscous_friction_b") is None


def test_resolve_parameter_rejects_unresolved_value(pmsm_cfg) -> None:
    with pytest.raises(ConfigError):
        config.resolve_parameter(pmsm_cfg, "mech.viscous_friction_b")


def test_resolve_parameter_rejects_unknown_name(pmsm_cfg) -> None:
    with pytest.raises(ConfigError):
        config.resolve_parameter(pmsm_cfg, "machine.does_not_exist")


def test_resolve_parameter_rejects_unit_mismatch(pmsm_cfg) -> None:
    with pytest.raises(ConfigError):
        config.resolve_parameter(pmsm_cfg, "machine.stator_resistance_rs", unit="V")


def test_accessors_reject_malformed_parameter(tmp_path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        "parameters:\n  x: {value: high, unit: '-'}\n", encoding="utf-8"
    )
    loaded = config.load_config(path)
    with pytest.raises(ConfigError):
        config.resolve_parameter(loaded, "x")
