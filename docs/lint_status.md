# Ruff status (post formatting-only pass)

Report-only audit. No `ruff check --fix`, no logic edits. Applied in this wrap-up:
`ruff format src tests experiments scripts` (Step 4, "style: formatting only").

## History

| Stage | `ruff check src tests experiments` |
| --- | ---: |
| Baseline (Phase 06g-2) | **694** |
| After `ruff format` | **322** |
| After the per-file E501 ignore | **231** |

`ruff format` reformatted 69 of 95 files; the **AST is unchanged for all 95** (proof in
`docs/FINAL_REPORT` / the Phase 06 final commit message: `ast.dump(ast.parse(...))`
sha256 before vs after — 95/95 identical, 0 differences).

## E501 handling — per-file ignore (not hand-wrapping)

The formatter reduced E501 from **384 to 91**. The residual 91 are unsplittable long literals
(ASCII tables printed by the experiment scripts; one docstring line in
`src/pmsm_sensorless/filters/dsogi_fll.py`). They are exempted per-file in `pyproject.toml`
`[tool.ruff.lint.per-file-ignores]` rather than hand-wrapped:

```toml
"experiments/*.py" = ["E501"]
"src/pmsm_sensorless/filters/dsogi_fll.py" = ["E501"]
```

## Residual counts (231)

```text
python -m ruff check src tests experiments --statistics
```

| Rule | Count |
| --- | ---: |
| B023 | 86 |
| I001 | 52 |
| F401 | 31 |
| B905 | 14 |
| UP031 | 13 |
| UP037 | 13 |
| F841 | 9 |
| B007 | 6 |
| UP035 | 3 |
| E731 | 2 |
| F541 | 1 |
| B008 | 1 |

100 are fixable (43 unsafe). **This is a baseline report, not a release gate.** The largest
remaining class, `B023 function-uses-loop-variable` (86), is in the experiment scripts'
closure-heavy builders and is **not** auto-fixable safely; it is deferred to a reviewed
change (see `docs/future_work.md`).
