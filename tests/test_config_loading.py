"""Configuration loading and structural-validation tests.

These tests assert the *shape* of the configuration, never paper values. Every
parameter is expected to be unresolved in Phase 00; when the paper is extracted
and values are filled in, the "currently unresolved" test is the one that will
need updating (deliberately, as a review signal).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from pmsm_sensorless import config
from pmsm_sensorless.config import ConfigError

CONFIG_NAMES = ["pmsm", "observer", "experiments"]


@pytest.mark.parametrize("name", CONFIG_NAMES)
def test_config_file_exists(config_dir: Path, name: str) -> None:
    assert (config_dir / f"{name}.yaml").is_file()


@pytest.mark.parametrize("name", CONFIG_NAMES)
def test_config_loads_as_mapping(config_dir: Path, name: str) -> None:
    cfg = config.load_config(name, directory=config_dir)
    assert isinstance(cfg, dict)
    assert cfg["meta"]["status"] == "partially_verified"


@pytest.mark.parametrize("name", CONFIG_NAMES)
def test_parameters_are_structurally_valid(config_dir: Path, name: str) -> None:
    cfg = config.load_config(name, directory=config_dir)
    entries = list(config.iter_parameters(cfg))
    assert entries, "expected at least one parameter entry"
    for _name, value, unit in entries:
        assert value is None or isinstance(value, (int, float))
        assert isinstance(unit, str) and unit


@pytest.mark.parametrize("name", CONFIG_NAMES)
def test_parameter_resolution_matches_source_metadata(config_dir: Path, name: str) -> None:
    """Resolved values have citations; omitted values remain explicit nulls."""
    cfg = config.load_config(name, directory=config_dir)
    for parameter_name, value, _unit in config.iter_parameters(cfg):
        source = cfg["parameters"][parameter_name].get("source")
        if value is None:
            assert source is None or isinstance(source, str)
        else:
            assert isinstance(source, str) and source


def test_missing_file_raises(config_dir: Path) -> None:
    with pytest.raises(ConfigError):
        config.load_config("does_not_exist", directory=config_dir)


def test_non_mapping_root_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad_root.yaml"
    path.write_text("- just\n- a\n- list\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        config.load_config(path)


def test_empty_file_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")
    with pytest.raises(ConfigError):
        config.load_config(path)


def test_invalid_yaml_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "broken.yaml"
    path.write_text("key: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        config.load_config(path)


def test_non_numeric_value_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad_value.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": {"x": {"value": "high", "unit": "-"}}}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError):
        list(config.iter_parameters(config.load_config(path)))


def test_boolean_value_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bool_value.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": {"x": {"value": True, "unit": "-"}}}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError):
        list(config.iter_parameters(config.load_config(path)))


def test_missing_unit_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "missing_unit.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": {"x": {"value": 1.0}}}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError):
        list(config.iter_parameters(config.load_config(path)))


def test_parameters_section_must_be_a_mapping(tmp_path: Path) -> None:
    path = tmp_path / "bad_params.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": [1, 2, 3]}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError):
        list(config.iter_parameters(config.load_config(path)))


def test_valid_numeric_value_is_accepted(tmp_path: Path) -> None:
    path = tmp_path / "ok.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": {"x": {"value": 1.5, "unit": "ohm"}}}),
        encoding="utf-8",
    )
    assert list(config.iter_parameters(config.load_config(path))) == [("x", 1.5, "ohm")]


def test_config_path_without_suffix_is_resolved(tmp_path: Path) -> None:
    path = tmp_path / "suffixed.yaml"
    path.write_text(
        yaml.safe_dump({"parameters": {"x": {"value": None, "unit": "-"}}}),
        encoding="utf-8",
    )
    assert config.load_config(tmp_path / "suffixed")["parameters"]["x"]["value"] is None
