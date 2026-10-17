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


def test_not_yet_implemented_subpackages_are_still_stubs() -> None:
    """Phase 03 guarantee: fuzzy/estimation have no algorithms yet.

    ``observers`` (Phase 02) and ``filters`` (Phase 03) are intentionally excluded:
    importing their submodules binds them as package attributes, so a "must be
    empty" check no longer applies there.
    """
    for name in ("fuzzy", "estimation"):
        module = importlib.import_module(f"pmsm_sensorless.{name}")
        public = [attr for attr in vars(module) if not attr.startswith("_")]
        assert public == [], f"pmsm_sensorless.{name} should be empty, found {public}"


def test_project_root_is_discoverable() -> None:
    root = config.find_project_root()
    assert (root / "pyproject.toml").is_file()
    assert (root / "src" / "pmsm_sensorless").is_dir()


def test_default_config_dir_exists() -> None:
    assert config.default_config_dir().is_dir()
