"""Package import and layout smoke tests for the Phase 00 scaffold."""

from __future__ import annotations

import importlib

import pytest

import pmsm_sensorless
from pmsm_sensorless import config

SUBPACKAGES = [
    "plant",
    "transforms",
    "observers",
    "fuzzy",
    "filters",
    "estimation",
    "simulation",
    "evaluation",
]


def test_version_is_a_non_empty_string() -> None:
    assert isinstance(pmsm_sensorless.__version__, str)
    assert pmsm_sensorless.__version__


@pytest.mark.parametrize("name", SUBPACKAGES)
def test_subpackages_are_importable_and_documented(name: str) -> None:
    module = importlib.import_module(f"pmsm_sensorless.{name}")
    assert module.__doc__, f"subpackage {name!r} should document its purpose"


def test_estimation_subpackage_is_implemented() -> None:
    """Phase 05b: ``estimation`` now carries the position/speed estimators.

    It was the last bare-stub subpackage; none remain.
    """
    module = importlib.import_module("pmsm_sensorless.estimation")
    public = [attr for attr in vars(module) if not attr.startswith("_")]
    assert "ArctanEstimator" in public and "QPLL" in public, public


def test_project_root_is_discoverable() -> None:
    root = config.find_project_root()
    assert (root / "pyproject.toml").is_file()
    assert (root / "src" / "pmsm_sensorless").is_dir()


def test_default_config_dir_exists() -> None:
    assert config.default_config_dir().is_dir()
