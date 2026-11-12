"""Fuzzy logic controller engine (paper Tables I-II, p.4).

Printed items (``configs/fuzzy.yaml``): input sets {NB,NS,Z,PS,PB} over
{-1,-0.5,0,0.5,1}; output sets {Z,PS,PM,PB} over [0,1]; triangular membership;
Mamdani inference; centre-of-gravity defuzzification; the two 5x5 rule bases; and
the mapping ``k = kF*k0``, ``a = aF*a0``.

Every *unprinted* item is an explicit assumption in
``configs/fuzzy_assumptions.yaml`` (each key commented "ASSUMPTION — not from
paper"); a null/missing assumption raises :class:`ConfigError`. The assumed items
are: the MF feet (Ruspini partition, half-width = centre spacing), the output
centres (equally spaced on [0,1]), the t-norm/aggregation (min/max), the CoG grid,
the per-axis structure (one FLC per axis) and the ``s_dot`` scheme (backward
difference, first step zero).

This module imports only NumPy and the config helpers -- never the plant or the
observers. Inputs are the observer's own normalised current error and its
derivative; no ground truth is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from pmsm_sensorless.config import ConfigError, load_config, resolve_parameter

__all__ = ["FuzzyEngine", "FuzzyParameters", "triangular_membership"]

_REQUIRED_FUZZY_SPECS = (
    "input_sets",
    "input_centres",
    "output_sets",
    "output_range",
    "rule_table_aF",
    "rule_table_kF",
)
_REQUIRED_ASSUMPTION_SPECS = (
    "input_mf_half_width",
    "output_centres",
    "output_mf_half_width",
    "and_operator",
    "aggregation_operator",
    "per_axis",
    "s_dot_scheme",
    "non_finite_input",
)


def _require(mapping: dict[str, Any], key: str, *, where: str) -> Any:
    value = mapping.get(key) if isinstance(mapping, dict) else None
    if value is None:
        raise ConfigError(
            f"{where} {key!r} is missing or null "
            "(ASSUMPTION — not from paper; it must be supplied explicitly)"
        )
    return value


def triangular_membership(x: float, centres: tuple[float, ...], half_width: float) -> np.ndarray:
    distance = np.abs(np.asarray(x, dtype=float) - np.asarray(centres, dtype=float))
    return np.maximum(0.0, 1.0 - distance / half_width)


@dataclass(frozen=True)
class FuzzyParameters:
    """Validated FLC parameters (printed + assumed)."""

    ke: float
    kec: float
    ku: float
    cog_grid_points: int
    input_sets: tuple[str, ...]
    input_centres: tuple[float, ...]
    input_half_width: float
    output_sets: tuple[str, ...]
    output_centres: tuple[float, ...]
    output_half_width: float
    output_min: float
    output_max: float
    rule_aF: dict[str, tuple[str, ...]]
    rule_kF: dict[str, tuple[str, ...]]

    @classmethod
    def from_configs(
        cls, fuzzy_config: dict[str, Any], assumptions_config: dict[str, Any]
    ) -> "FuzzyParameters":
        """Build parameters from the printed and assumption mappings.

        Raises:
            ConfigError: If a printed value or a required assumption is missing,
                null, or structurally invalid.
        """
        fuzzy_specs = fuzzy_config.get("specs") or {}
        assumption_specs = assumptions_config.get("specs") or {}
        for key in _REQUIRED_FUZZY_SPECS:
            _require(fuzzy_specs, key, where="printed fuzzy spec")
        for key in _REQUIRED_ASSUMPTION_SPECS:
            _require(assumption_specs, key, where="assumption")

        input_sets = tuple(str(x) for x in fuzzy_specs["input_sets"])
        input_centres = tuple(float(x) for x in fuzzy_specs["input_centres"])
        output_sets = tuple(str(x) for x in fuzzy_specs["output_sets"])
        output_range = tuple(float(x) for x in fuzzy_specs["output_range"])
        if len(input_sets) != len(input_centres):
            raise ConfigError("input_sets and input_centres length mismatch")
        if output_range[0] != 0.0 or output_range[1] != 1.0:
            raise ConfigError("printed output range must be [0, 1]")

        and_operator = str(assumption_specs["and_operator"])
        aggregation = str(assumption_specs["aggregation_operator"])
        if and_operator != "min" or aggregation != "max":
            raise ConfigError(
                "only the assumed Mamdani operators AND=min, aggregation=max are implemented"
            )
        if assumption_specs["per_axis"] is not True:
            raise ConfigError("only the assumed per-axis structure is implemented")
        if str(assumption_specs["s_dot_scheme"]) != "backward_difference_first_zero":
            raise ConfigError("only the assumed s_dot backward difference is implemented")
        if str(assumption_specs["non_finite_input"]) != "zero":
            raise ConfigError("only the assumed non-finite-input handling ('zero') is implemented")

        def _rules(key: str) -> dict[str, tuple[str, ...]]:
            table = fuzzy_specs[key]
            columns = [str(c) for c in table["columns"]]
            if columns != list(input_sets):
                raise ConfigError(f"{key} columns must equal the printed input sets")
            rows: dict[str, tuple[str, ...]] = {}
            for label, values in table["rows"].items():
                values = tuple(str(v) for v in values)
                if len(values) != len(columns):
                    raise ConfigError(f"{key} row {label!r} has the wrong length")
                if any(v not in output_sets for v in values):
                    raise ConfigError(f"{key} row {label!r} uses an unknown output set")
                rows[str(label)] = values
            if set(rows) != set(input_sets):
                raise ConfigError(f"{key} rows must equal the printed input sets")
            return rows

        return cls(
            ke=resolve_parameter(assumptions_config, "flc.input_error_scale_ke", unit="1/A"),
            kec=resolve_parameter(
                assumptions_config, "flc.input_delta_error_scale_kec", unit="s/A"
            ),
            ku=resolve_parameter(assumptions_config, "flc.output_scale_ku", unit="-"),
            cog_grid_points=int(
                resolve_parameter(assumptions_config, "flc.cog_grid_points", unit="-")
            ),
            input_sets=input_sets,
            input_centres=input_centres,
            input_half_width=float(assumption_specs["input_mf_half_width"]),
            output_sets=output_sets,
            output_centres=tuple(float(x) for x in assumption_specs["output_centres"]),
            output_half_width=float(assumption_specs["output_mf_half_width"]),
            output_min=output_range[0],
            output_max=output_range[1],
            rule_aF=_rules("rule_table_aF"),
            rule_kF=_rules("rule_table_kF"),
        )


class FuzzyEngine:
    """Stateless Mamdani FLC: ``(s_n, s_dot_n) -> (kF, aF)`` in [0,1].

    Args:
        parameters: Validated FLC parameters.

    Raises:
        ValueError: If the CoG grid is smaller than 2 points.
    """

    def __init__(self, parameters: FuzzyParameters) -> None:
        if parameters.cog_grid_points < 2:
            raise ValueError("cog_grid_points must be >= 2")
        self.parameters = parameters
        self._grid = np.linspace(
            parameters.output_min, parameters.output_max, parameters.cog_grid_points
        )
        self._output_memberships = np.array(
            [
                triangular_membership(self._grid, (centre,), parameters.output_half_width)
                for centre in parameters.output_centres
            ]
        )
        self._output_index = {label: i for i, label in enumerate(parameters.output_sets)}
        self._input_index = {label: i for i, label in enumerate(parameters.input_sets)}

    @classmethod
    def from_yaml(cls, directory: Path | str | None = None) -> "FuzzyEngine":
        """Load ``fuzzy.yaml`` and ``fuzzy_assumptions.yaml`` from ``directory``."""
        from pmsm_sensorless.config import default_config_dir

        base = Path(directory) if directory is not None else default_config_dir()
        return cls(
            FuzzyParameters.from_configs(
                load_config("fuzzy", directory=base),
                load_config("fuzzy_assumptions", directory=base),
            )
        )

    @staticmethod
    def _normalise(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0  # assumed handling of non-finite normalised inputs
        return min(1.0, max(-1.0, value))

    def _infer(
        self, mu_s: np.ndarray, mu_sd: np.ndarray, rules: dict[str, tuple[str, ...]]
    ) -> float:
        strengths = np.zeros(len(self.parameters.output_sets))
        for i, row_label in enumerate(self.parameters.input_sets):
            for j, column_label in enumerate(self.parameters.input_sets):
                strength = min(mu_s[i], mu_sd[j])
                index = self._output_index[rules[row_label][j]]
                strengths[index] = max(strengths[index], strength)
        aggregated = np.max(np.minimum(strengths[:, None], self._output_memberships), axis=0)
        total = float(np.sum(aggregated))
        if total <= 0.0:
            return 0.0
        return float(np.sum(aggregated * self._grid) / total)

    def evaluate(self, s_n: float, s_dot_n: float) -> tuple[float, float]:
        """Return ``(kF, aF)`` for normalised inputs ``s_n, s_dot_n``."""
        s = self._normalise(s_n)
        s_dot = self._normalise(s_dot_n)
        mu_s = triangular_membership(
            s, self.parameters.input_centres, self.parameters.input_half_width
        )
        mu_sd = triangular_membership(
            s_dot, self.parameters.input_centres, self.parameters.input_half_width
        )
        k_f = self._infer(mu_s, mu_sd, self.parameters.rule_kF)
        a_f = self._infer(mu_s, mu_sd, self.parameters.rule_aF)
        return k_f, a_f
