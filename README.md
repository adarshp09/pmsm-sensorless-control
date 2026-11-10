# PMSM Sensorless Control — Fuzzy Logic + DSOGI-FLL SMO

> **Status: Phases 00–05 complete (including verification sub-phases 03b–03f, 04b and
> the 05a filter prelude). Phase 06a (closed-loop encoder baseline) complete.**
>
> The PMSM plant (Eqs.1–2), the Clarke transform, the conventional (sign) /
> sigmoid / fuzzy-adaptive sliding-mode observers (Eqs.3–7), the DSOGI-FLL
> back-EMF filter (Eqs.9–19), the position/speed estimators (Eq.(8) + Q-PLL) and a
> closed-loop FOC drive with **true-angle** feedback (encoder baseline) are
> implemented and tested. The estimator is **not yet inside the control loop**
> (Phase 06b), and there is no integrated sensorless simulation yet. Results are
> **not** a reproduction of the paper's reported accuracy.

## Objective

Build a modular, test-driven Python implementation of the research paper:

> **"An Improved Sliding Mode Observer Sensorless Control for PMSM Based on Fuzzy
> Logic Controller and DSOGI-FLL."**

The eventual pipeline estimates rotor position and speed from measured stator
voltages/currents only (no shaft sensor), by combining:

- a **sliding-mode observer (SMO)** whose switching term is softened by a
  **sigmoid** function, driven by **back-EMF** estimates;

- a **fuzzy logic controller (FLC)** that adapts the observer's switching gain;
- a **dual second-order generalized integrator (DSOGI)** that filters the
  estimated back-EMF to remove harmonics/DC offset;
- a **frequency-locked loop (FLL)** that tracks the back-EMF frequency, from
  which electrical speed and rotor position are derived.

## Paper reference

| Field | Value |
| --- | --- |
| Title | An Improved Sliding Mode Observer Sensorless Control for PMSM Based on Fuzzy Logic Controller and DSOGI-FLL |
| Authors | X. Zhu et al. |
| Venue / year | IEEE TTE, vol. 11, no. 1, pp. 823–834, 2025 |
| DOI | 10.1109/TTE.2024.3396732 |
| Local copy | `docs/act.pdf` (12-page copy; text-extracted and page-rendered) |

See [`docs/paper_method.md`](docs/paper_method.md) and
[`docs/parameter_extraction.md`](docs/parameter_extraction.md) for what the paper
does and does not print, with source locations.

## Proposed algorithm architecture

```
        i_abc, u_abc
             │
        Clarke transform
             │
        ┌────▼─────┐      estimated back-EMF  ┌──────────┐
        │   SMO    ├───────────┬────────────►│  DSOGI   │
        │ (sigmoid │           │             │ (dual 2nd │
        │  switch) │           │             │  order GI)│
        └────┬─────┘           │             └────┬─────┘
             │                 │                  │  e_α', e_β' (+ quadrature)
      uncertainty/error        │             ┌────▼─────┐
             │                 │             │   FLL    │──► ω̂_e
        ┌────▼─────┐           │             └────┬─────┘
        │   FLC    │───────────┘                  │
        │ (Δgain)  │                              ▼
        └──────────┘                    atan2 / sign ⇒ θ̂_e, ω̂_m
```

The implemented forms, signs and interconnections are recorded in
[`docs/equations_and_conventions.md`](docs/equations_and_conventions.md),
[`docs/paper_method.md`](docs/paper_method.md) and
[`docs/implementation_decisions.md`](docs/implementation_decisions.md); some are
DERIVED or ASSUMPTION and are labelled there.

## Repository structure

```
pmsm-sensorless-control/
├── README.md
├── pyproject.toml
├── .gitignore
├── configs/                  # YAML parameter templates
│   ├── pmsm.yaml             # machine + rating (Table III values)
│   ├── observer.yaml         # SMO / DSOGI / FLL parameters + specs
│   ├── experiments.yaml      # simulation conditions
│   ├── fuzzy.yaml            # FLC PRINTED items (sets, rules)
│   └── fuzzy_assumptions.yaml # FLC ASSUMED items (scales, MFs, operators)
├── docs/
│   ├── act.pdf                    # source paper
│   ├── paper_method.md            # extraction status per paper item
│   ├── equations_and_conventions.md  # notation / sign / angle conventions
│   ├── parameter_extraction.md    # extraction table with source citations
│   ├── implementation_decisions.md # decision log (D-001 …)
│   ├── flc_spec.md                # FLC / DSOGI-FLL printed-vs-unprinted spec
│   └── phase02/03/04.md           # per-phase findings
├── src/pmsm_sensorless/
│   ├── config.py             # YAML loading + structural validation
│   ├── plant/                # PMSM model (Eqs.1–2) + load profiles
│   ├── transforms/           # Clarke (amplitude/power invariant)
│   ├── observers/            # sign / sigmoid / FLC-adaptive SMO (Eqs.3–7)
│   ├── filters/              # SOGI / DSOGI-FLL (Eqs.9–19)
│   ├── fuzzy/                # FLC Mamdani engine (Tables I–II)
│   ├── estimation/           # position/speed estimators (Eq.(8) alpha-arctan + Q-PLL)
│   ├── simulation/           # scenarios / integration (04c ideal FOC; 06a closed-loop drive)
│   └── evaluation/           # metrics / plots — still a docstring stub
├── experiments/              # per-phase scripts + output/ result JSONs & plots
├── tests/
└── notebooks/
```

The `evaluation` sub-package remains a docstring stub. `simulation` holds the
Phase 04c nominal-drive generator and the Phase 06a closed-loop drive (`drive.py`,
which composes the controller with a pluggable angle/speed source; `TrueSource` is
the encoder baseline).

## Environment setup

Requires **Python ≥ 3.10**.

```bash
cd pmsm-sensorless-control

# Option A — editable install (recommended)
python -m venv .venv && source .venv/bin/activate
python -m pip install -e ".[dev]"

# Option B — uv
uv venv && source .venv/bin/activate
uv pip install -e ".[dev]"

# Optional: only if you choose scikit-fuzzy instead of a hand-written FLC
python -m pip install -e ".[fuzzy]"
```

## Tests

```bash
pytest
```

Tests cover the plant (Eqs.1–2), Clarke, the conventional/sigmoid/FLC observers,
SOGI/DSOGI-FLL, the position/speed estimators, the closed-loop drive (Phase 06a),
configuration loading/validation, the discrete-stability bound (D-027), the
reported-injection fix (D-025), the ramp metrics and the layer-separation structure
invariants.

## Implementation roadmap

| Phase | Scope | Status |
| --- | --- | --- |
| **00** | Repo scaffold, spec extraction, conventions, config templates, smoke/config tests | completed |
| 01 | PMSM plant model + Clarke transform (Park not yet implemented) | completed |
| 02 | Conventional SMO (sign / sigmoid switching) | completed (+ verification 02b) |
| 03 | DSOGI + FLL back-EMF filtering and frequency tracking | completed (+ verification 03b–03f) |
| 04 | Fuzzy logic controller for switching-gain adaptation | completed (+ verification 04b) |
| 04c | Nominal-drive scenario + 04b re-check (housekeeping) | completed |
| 05a | Filter prelude (offset/bias diagnosis) + estimator spec sheet | completed |
| 05b | Position/speed estimator (Eq.(8) + Q-PLL) + open-loop evaluation | completed |
| 05 | Integration: full sensorless estimator (position/speed) | in progress (05a–05b done; estimator in loop next) |
| 06a | Closed-loop FOC drive with encoder (true-angle) feedback | completed |
| 06b | Estimator inside the control loop (sensorless) | not started |
| 06 | Simulation experiments vs. baselines | not started |
| 07 | Evaluation, plots, comparison with reported results | not started |

## Current phase and known unresolved specifications

The paper **is** available (`docs/act.pdf`) and Phases 00–04 are implemented.
What remains genuinely unprinted by the paper — and therefore DERIVED or
ASSUMPTION, labelled in
[`docs/implementation_decisions.md`](docs/implementation_decisions.md) — includes:

- the Clarke scaling convention (amplitude-invariant chosen, D-010);
- the per-unit current base `I_base` used to convert the paper's `a` (D-015);
- the FLC scaling gains, membership-function feet, t-norm/aggregation, CoG grid
  and per-axis structure (ASSUMPTION, D-030/D-031);
- the FLL error-signal algebra and normalization (derived, D-020; D-022);
- the DSOGI positive-sequence combination (derived, D-021);
- rotor position/speed estimation equations and their sign conventions (Phase 05);
- simulation/experimental conditions, baselines and reported results.

See [`docs/parameter_extraction.md`](docs/parameter_extraction.md) for the tracked
list. **Do not claim exact reproduction while these are unresolved.**
