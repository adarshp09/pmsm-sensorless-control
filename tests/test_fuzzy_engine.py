"""Tests for the fuzzy engine (paper Tables I-II; assumptions as labelled)."""

from __future__ import annotations

import copy
import importlib
import math
import types

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.fuzzy.engine import (
    FuzzyEngine,
    FuzzyParameters,
    triangular_membership,
)


@pytest.fixture
def engine(config_dir) -> FuzzyEngine:
    return FuzzyEngine.from_yaml(config_dir)


@pytest.fixture
def raw_configs(config_dir):
    return (
        config.load_config("fuzzy", directory=config_dir),
        config.load_config("fuzzy_assumptions", directory=config_dir),
    )


# --- membership functions -----------------------------------------------------


def test_membership_one_at_peak_zero_at_neighbours(engine: FuzzyEngine) -> None:
    centres = engine.parameters.input_centres
    width = engine.parameters.input_half_width
    for i, centre in enumerate(centres):
        membership = triangular_membership(centre, centres, width)
        assert membership[i] == pytest.approx(1.0)
        assert np.sum(membership) == pytest.approx(1.0)  # Ruspini partition
        for j, other in enumerate(centres):
            if j != i:
                assert membership[j] == pytest.approx(0.0)


def test_membership_symmetry_and_partition_of_unity(engine: FuzzyEngine) -> None:
    centres = engine.parameters.input_centres
    width = engine.parameters.input_half_width
    for x in np.linspace(-1.0, 1.0, 41):
        left = triangular_membership(x, centres, width)
        right = triangular_membership(-x, centres, width)
        assert np.sum(left) == pytest.approx(1.0)
        assert np.allclose(left, right[::-1])  # symmetric set


# --- rule bases ---------------------------------------------------------------


def test_rule_tables_match_the_printed_spec(engine: FuzzyEngine, raw_configs) -> None:
    fuzzy_config, _ = raw_configs
    for key, parameter_name in (("rule_table_aF", "rule_aF"), ("rule_table_kF", "rule_kF")):
        printed = fuzzy_config["specs"][key]
        stored = getattr(engine.parameters, parameter_name)
        assert len(stored) == 5
        assert all(len(row) == 5 for row in stored.values())
        for row_label, values in printed["rows"].items():
            assert stored[row_label] == tuple(values)


# --- inference (hand-computed) ------------------------------------------------


def test_inference_at_zero_inputs(engine: FuzzyEngine) -> None:
    """s_n=0, s_dot_n=0 -> only (Z,Z) fires, output set Z for both.

    A Z triangle (peak 0, half-width 1/3) truncated to the printed [0,1] output
    universe has centroid 1/9 = 0.1111 (discrete 1001-point grid ~= 0.1108).
    """
    k_f, a_f = engine.evaluate(0.0, 0.0)
    assert k_f == pytest.approx(1.0 / 9.0, abs=2e-3)
    assert a_f == pytest.approx(1.0 / 9.0, abs=2e-3)


def test_inference_at_hand_computed_point(engine: FuzzyEngine) -> None:
    """s_n=0.5 (PS), s_dot_n=0 (Z).

    aF rule row PS, column Z = PM -> centroid 2/3.
    kF rule row PS, column Z = PS -> centroid 1/3.
    """
    k_f, a_f = engine.evaluate(0.5, 0.0)
    assert k_f == pytest.approx(1.0 / 3.0, abs=2e-3)
    assert a_f == pytest.approx(2.0 / 3.0, abs=2e-3)


def test_inference_saturates_and_stays_in_range(engine: FuzzyEngine) -> None:
    for s_n, s_dot_n in [
        (1.0, 1.0), (-1.0, -1.0), (5.0, -5.0), (-7.0, 7.0),
        (float("inf"), 0.0), (float("-inf"), 1.0), (float("nan"), 0.0),
    ]:
        k_f, a_f = engine.evaluate(s_n, s_dot_n)
        assert math.isfinite(k_f) and math.isfinite(a_f)
        assert 0.0 <= k_f <= 1.0
        assert 0.0 <= a_f <= 1.0


def test_engine_is_deterministic(engine: FuzzyEngine) -> None:
    first = [engine.evaluate(x, 0.3 * x) for x in np.linspace(-1.0, 1.0, 21)]
    second = [engine.evaluate(x, 0.3 * x) for x in np.linspace(-1.0, 1.0, 21)]
    assert first == second


# --- assumptions --------------------------------------------------------------


@pytest.mark.parametrize(
    "mutate",
    [
        lambda p: p["parameters"]["flc.input_error_scale_ke"].update(value=None),
        lambda p: p["parameters"]["flc.input_delta_error_scale_kec"].update(value=None),
        lambda p: p["parameters"]["flc.cog_grid_points"].update(value=None),
        lambda p: p["specs"].pop("input_mf_half_width"),
        lambda p: p["specs"].update(and_operator=None),
        lambda p: p["specs"].pop("s_dot_scheme"),
        lambda p: p["specs"].update(per_axis=False),
    ],
)
def test_null_or_invalid_assumption_raises(raw_configs, mutate) -> None:
    fuzzy_config, assumptions = raw_configs
    assumptions = copy.deepcopy(assumptions)
    mutate(assumptions)
    with pytest.raises(config.ConfigError):
        FuzzyParameters.from_configs(fuzzy_config, assumptions)


def test_fuzzy_engine_does_not_import_plant_or_observers() -> None:
    module = importlib.import_module("pmsm_sensorless.fuzzy.engine")
    for value in vars(module).values():
        if isinstance(value, types.ModuleType):
            assert not value.__name__.startswith("pmsm_sensorless.plant")
            assert not value.__name__.startswith("pmsm_sensorless.observers")
