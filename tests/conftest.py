"""Shared pytest fixtures."""

from __future__ import annotations

from pathlib import Path

import pytest

from pmsm_sensorless import config


@pytest.fixture(scope="session")
def config_dir() -> Path:
    """Path to the repository's ``configs/`` directory (discovered, not hardcoded)."""
    return config.default_config_dir()
