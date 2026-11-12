"""Configuration loading and structural validation.

Paper-specific numeric values are deliberately absent in Phase 00. The job of
this module is to make that absence *explicit and safe*: a parameter is either
a validated ``(value, unit, source)`` record or an explicit ``UNRESOLVED``
placeholder. Malformed values are rejected loudly rather than silently
defaulting to something plausible.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import yaml

#: Sentinel meaning "value not yet extracted from the paper".
UNRESOLVED: None = None

_REQUIRED_PARAM_KEYS = ("value", "unit")
_NUMERIC = (int, float)


class ConfigError(ValueError):
    """Raised when a configuration file is missing, malformed or invalid."""


def find_project_root(start: Path | str | None = None) -> Path:
    """Locate the repository root by walking up to the nearest ``pyproject.toml``.

    Args:
        start: Path to start searching from. Defaults to this module's file.

    Raises:
        FileNotFoundError: If no ``pyproject.toml`` is found above ``start``.
    """
    here = Path(start).resolve() if start is not None else Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / "pyproject.toml").is_file():
            return candidate
    raise FileNotFoundError(f"could not locate project root (no pyproject.toml found above {here})")


def default_config_dir() -> Path:
    """Return the repository's ``configs/`` directory."""
    return find_project_root() / "configs"


def load_config(
    source: Path | str,
    *,
    directory: Path | str | None = None,
) -> dict[str, Any]:
    """Load a YAML configuration file and require a top-level mapping.

    Args:
        source: A file name or path, with or without a ``.yaml``/``.yml`` suffix.
        directory: If given, relative ``source`` names are resolved against it
            (e.g. :func:`default_config_dir`).

    Raises:
        ConfigError: If the file is missing, empty, invalid YAML, or its root is
            not a mapping.
    """
    path = Path(source)
    if not path.is_absolute() and directory is not None:
        path = Path(directory) / path
    if path.suffix not in {".yaml", ".yml"}:
        path = path.with_suffix(".yaml")
    if not path.is_file():
        raise ConfigError(f"configuration file not found: {path}")

    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:  # pragma: no cover - message passthrough
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc

    if data is None:
        raise ConfigError(f"configuration file is empty: {path}")
    if not isinstance(data, dict):
        raise ConfigError(
            f"configuration root must be a mapping, got {type(data).__name__}: {path}"
        )
    return data


def iter_parameters(
    config: dict[str, Any],
) -> Iterator[tuple[str, float | None, str]]:
    """Yield ``(name, value, unit)`` for each entry in the ``parameters`` mapping.

    Every entry must be a mapping exposing at least ``value`` and ``unit``.
    ``value`` may be :data:`UNRESOLVED` (``None``) or a number; any other type is
    rejected so that a malformed value cannot masquerade as a real parameter.

    Raises:
        ConfigError: If ``parameters`` is absent, not a mapping, or any entry is
            malformed.
    """
    params = config.get("parameters")
    if params is None:
        raise ConfigError("configuration has no 'parameters' mapping")
    if not isinstance(params, dict):
        raise ConfigError(f"'parameters' must be a mapping, got {type(params).__name__}")

    for name, entry in params.items():
        if not isinstance(entry, dict):
            raise ConfigError(f"parameter {name!r} must be a mapping, got {type(entry).__name__}")
        missing = [key for key in _REQUIRED_PARAM_KEYS if key not in entry]
        if missing:
            raise ConfigError(f"parameter {name!r} is missing keys: {missing}")

        value = entry["value"]
        unit = entry["unit"]
        if value is not UNRESOLVED and (not isinstance(value, _NUMERIC) or isinstance(value, bool)):
            raise ConfigError(
                f"parameter {name!r} value must be numeric or null, got {type(value).__name__}"
            )
        if not isinstance(unit, str) or not unit:
            raise ConfigError(
                f"parameter {name!r} unit must be a non-empty string, got {type(unit).__name__}"
            )
        yield name, value, unit


def unresolved_parameters(config: dict[str, Any]) -> list[str]:
    """Return the names of all parameters whose value is still unresolved."""
    return [name for name, value, _ in iter_parameters(config) if value is UNRESOLVED]


def _find_parameter(config: dict[str, Any], name: str) -> tuple[float | None, str]:
    """Return ``(value, unit)`` for ``name`` or raise if it is absent."""
    for parameter_name, value, unit in iter_parameters(config):
        if parameter_name == name:
            return value, unit
    raise ConfigError(f"parameter {name!r} not found in configuration")


def get_parameter(config: dict[str, Any], name: str, *, unit: str | None = None) -> float | None:
    """Return a parameter's value, which may be :data:`UNRESOLVED` (``None``).

    Args:
        config: A loaded configuration mapping.
        name: Parameter key, e.g. ``"machine.stator_resistance_rs"``.
        unit: If given, the stored unit must match exactly.

    Raises:
        ConfigError: If the parameter is absent, structurally malformed, or its
            unit does not match the requested one.
    """
    value, stored_unit = _find_parameter(config, name)
    if unit is not None and stored_unit != unit:
        raise ConfigError(f"parameter {name!r} has unit {stored_unit!r}, expected {unit!r}")
    return value


def resolve_parameter(config: dict[str, Any], name: str, *, unit: str | None = None) -> float:
    """Return a required parameter's numeric value.

    Unlike :func:`get_parameter`, an unresolved (``null``) value is an error:
    this is the guard the plant and transforms use so a missing paper value can
    never be silently replaced by a plausible-looking default.

    Raises:
        ConfigError: If the parameter is absent, unresolved, or unit-mismatched.
    """
    value = get_parameter(config, name, unit=unit)
    if value is UNRESOLVED:
        raise ConfigError(
            f"required parameter {name!r} is unresolved (value is null); "
            "it must be extracted from the paper before use"
        )
    return value
