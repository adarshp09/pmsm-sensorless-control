# Ruff status

Report-only audit run with:

```text
python -m ruff check src tests experiments --statistics
```

No fixes were applied. Ruff reported **694 errors**:

| Rule | Count |
| --- | ---: |
| E501 | 384 |
| B023 | 86 |
| E702 | 79 |
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

The report includes the new experiment script; the two additional findings are
one E501 and one I001 in that script. Ruff also reports 100 fixable findings
(43 unsafe fixes available).

This is a baseline report, not a Phase 06g code-quality gate.
