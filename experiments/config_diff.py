"""Diff a script's filter/observer/scenario options against the library defaults.

Every experiment that touches the DSOGI-FLL must state which non-default options it
enables (Phase 03e). This helper does that mechanically so a non-paper option such
as ``epsilon_filter_tau`` cannot silently contaminate a result again (D-028).
"""

from __future__ import annotations

from typing import Any

#: Library defaults of ``DSOGIFLLParameters`` / ``SMOObserverParameters.from_config``.
LIBRARY_DEFAULTS: dict[str, Any] = {
    "epsilon_filter_tau": None,
    "min_omega": None,
    "max_omega": None,
    "normalize_error": False,
    "positive_sequence": True,
    "per_unit_scaling": True,
}


def diff_options(used: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Return the options whose used value differs from the library default.

    Options absent from ``used`` are treated as left at their default and are not
    reported.
    """
    diff: dict[str, dict[str, Any]] = {}
    for key, default in LIBRARY_DEFAULTS.items():
        if key in used and used[key] != default:
            diff[key] = {"used": used[key], "default": default}
    return diff


def summary(
    script: str,
    used: dict[str, Any],
    *,
    scenario: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a JSON-serialisable config-diff block for a script's output."""
    return {
        "script": script,
        "non_default_options": diff_options(used),
        "library_defaults": dict(LIBRARY_DEFAULTS),
        "scenario": scenario or {},
        "note": (
            "non_default_options lists every library default overridden by this "
            "script; an empty mapping means all defaults."
        ),
    }


def describe(block: dict[str, Any]) -> list[str]:
    """Human-readable lines for printing a config-diff block."""
    lines = [f"config-diff [{block['script']}]: scenario={block.get('scenario', {})}"]
    non_default = block["non_default_options"]
    if not non_default:
        lines.append("  all filter/observer options at library defaults")
    for key, values in non_default.items():
        lines.append(f"  {key}: used={values['used']!r} (default {values['default']!r})")
    return lines


def emit(block: dict[str, Any]) -> None:
    """Print a config-diff block."""
    for line in describe(block):
        print(line)
