# PMSM Sensorless Control — Fuzzy Logic + DSOGI-FLL SMO

> **Status: Phase 00 — specification scaffold. Implementation in progress.**
>
> No PMSM model, sliding-mode observer, fuzzy controller, SOGI/DSOGI or FLL
> algorithm is implemented yet. All paper-specific numbers are intentionally
> left **unresolved** (`null`) until the source paper is available. Nothing in
> this repository should be read as a reproduction of published results.

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

The paper PDF is **not present in this workspace**. The reference below is a
placeholder and must be completed once the source is supplied.

| Field | Value |
| --- | --- |
| Title | An Improved Sliding Mode Observer Sensorless Control for PMSM Based on Fuzzy Logic Controller and DSOGI-FLL |
| Authors | _unresolved_ |
| Venue / year | _unresolved_ |
| DOI | _unresolved_ |
| Local copy | _not available_ |

See [`docs/paper_method.md`](docs/paper_method.md) and
[`docs/parameter_extraction.md`](docs/parameter_extraction.md) for the exact
items that still need to be extracted, with their required source locations.

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

Exact equation forms, signs and interconnections are **unresolved** pending the
paper (see `docs/paper_method.md`).

## Repository structure

```
pmsm-sensorless-control/
├── README.md
├── pyproject.toml
├── .gitignore
├── configs/                  # YAML parameter templates (mostly unresolved)
│   ├── pmsm.yaml
│   ├── observer.yaml
│   └── experiments.yaml
├── docs/
│   ├── paper_method.md            # paper's actual method + source locations
│   ├── equations_and_conventions.md  # notation / sign / angle conventions
│   ├── parameter_extraction.md    # filled-in extraction table
│   └── implementation_decisions.md # decision log
├── src/pmsm_sensorless/
│   ├── __init__.py
│   ├── config.py             # YAML loading + structural validation
│   ├── plant/                # PMSM model                (empty in Phase 00)
│   ├── transforms/           # Clarke / Park             (empty in Phase 00)
│   ├── observers/            # SMO                      (empty in Phase 00)
│   ├── fuzzy/                # FLC                      (empty in Phase 00)
│   ├── filters/              # SOGI / DSOGI             (empty in Phase 00)
│   ├── estimation/           # FLL, position/speed      (empty in Phase 00)
│   ├── simulation/           # scenarios / integration  (empty in Phase 00)
│   └── evaluation/           # metrics / plots          (empty in Phase 00)
├── experiments/
├── tests/
└── notebooks/
```

Sub-packages documented as "empty in Phase 00" contain only a docstring — this
is deliberate; no algorithm code is written before the specification is fixed.

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

Phase 00 tests cover only package import and configuration loading/validation.

## Implementation roadmap

| Phase | Scope | Status |
| --- | --- | --- |
| **00** | Repo scaffold, spec extraction, conventions, config templates, smoke/config tests | **in progress** |
| 01 | PMSM plant model + Clarke transform (Park not yet implemented) | **in progress** |
| 02 | Conventional SMO (sign / sigmoid switching) | **in progress** |
| 03 | DSOGI + FLL back-EMF filtering and frequency tracking | **in progress** |
| 04 | Fuzzy logic controller for switching-gain adaptation | **in progress** |
| 05 | Integration: full sensorless estimator | not started |
| 06 | Simulation experiments vs. baselines | not started |
| 07 | Evaluation, plots, comparison with reported results | not started |

## Current phase and known unresolved specifications

Phase 00 is **incomplete**: the paper is not available, so the following are
deliberately blank and must not be guessed —

- all motor parameters (pole pairs, `Rs`, `Ld`, `Lq`, flux linkage, inertia, …);
- SMO equations, sliding surface, sigmoid parameters and gains;
- FLC inputs, scaling factors, membership functions, rule table, inference and
  defuzzification;
- DSOGI transfer functions and FLL error signal, adaptation law and gains;
- discretization method, sampling period and any Tustin prewarping;
- position/speed estimation equations and their sign conventions;
- simulation/experimental conditions, baselines and reported results.

See [`docs/parameter_extraction.md`](docs/parameter_extraction.md) for the
tracked list. **Do not claim exact reproduction while these are unresolved.**
