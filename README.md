# PMSM Sensorless Control — Fuzzy Logic + DSOGI-FLL SMO

> **Status: v1.0 — Phases 00–06g-2 complete.** The PMSM plant (Eqs. 1–2), the Clarke
> transform, the conventional (sign) / sigmoid / fuzzy-adaptive SMOs (Eqs. 3–7), the
> DSOGI-FLL back-EMF filter (Eqs. 9–19), the position/speed estimators (Eq. 8 + Q-PLL), a
> closed-loop FOC drive, and the full **sensorless** loop (SMO → DSOGI-FLL → Q-PLL) are
> implemented and tested (**255 tests**). Results are **not** a reproduction of the paper's
> reported accuracy — see [`docs/claims_register.md`](docs/claims_register.md) for the
> status of every claim.
>
> `phase-06f-complete` and `phase-06-complete` are **SUPERSEDED** by the final release tag
> `v1.0-final` (Phase 06g/06g-2 corrected the attribution and re-read the figures).

## Quick start

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
python -m pytest -q                 # 255 tests
python scripts/reproduce.py --audit # 39/39 committed outputs byte-identical
python scripts/reproduce.py --fast  # tests + the four headline reports (~10 min)
```

## Results summary (scoped — see [`docs/FINAL_REPORT.md`](docs/FINAL_REPORT.md))

- **Gate P PASS on S1–S4** (paper-shaped scenarios; nominal chain; `Ts = 100 µs`; dead time
  OFF) — `experiments/output/06b6_gatep/gatep.json`.
- **Steady** estimation errors are **below** the paper's; **dynamic** errors are **4–5×
  above** (S2 111.7 vs 23 rpm; S3 40.4 vs 10 rpm), **consistent with a gentler paper
  load-transition** (HYPOTHESIS; D-086/D-087).
- The **observer adds only 3–5 %** of the dynamic error (`f_n ≤ 40 Hz`); the **SOGI+FLL**
  contributes the rest (`f_n ≥ 80`) — `06f_replay/replay.json`; D-083.
- **Stress limits:** 500 rpm loaded steps and the 1000 rpm rated step lose synchronism.
- **Known defects:** the normalized Q-PLL option (`normalize=True`) — non-paper, do not use
  (D-076); the FLL `ω′ ≥ 0` guard trips on divergence (D-082).
- Every value is labelled PRINTED / DERIVED / ASSUMPTION; the unprinted register is
  [`docs/estimator_spec.md`](docs/estimator_spec.md) §"Phase 06 unprinted-parameter register".

See [`docs/claims_register.md`](docs/claims_register.md),
[`docs/FINAL_REPORT.md`](docs/FINAL_REPORT.md) and
[`docs/future_work.md`](docs/future_work.md) (Phase 07, not started).

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
| 06b | Estimator inside the control loop (sensorless) | partial: chain + Gate 1 OK; Gate 2 failed (D-054) |
| 06b-2 | Estimator-aware speed loop: measured T_est + margins | stopped at Step 2 (no feasible α_ω*, D-058) |
| 06b-3 | Operating-point decoupling + low-speed relaxation | partial: Gate 2a/2b@1000/2c pass; 2b@500 fails (D-060) |
| 06b-4 | Softened low-speed bandwidth (2.5 Hz) | partial: 500 rpm steady + all 1000 rpm pass; 500 rpm load transient trips the FLL (D-061) |
| 06b-5 | DSOGI-FLL min-frequency clamp | partial: guard trip gone, start-up-under-load fixed, all 1000 rpm pass; 500 rpm loaded step still fails (D-062) |
| 06b-6 | Re-scope to paper scenarios; sensorless suite | **Gate P PASS** on S1–S4; suite run; low-speed/high-load steps documented limitation (D-063/D-064) |
| 06d | Estimation-bandwidth diagnosis (no src change) | FLL limits the loop (oracle 2× α_ω); normalisation ASSUMPTION/narrows; f_n×0.5 row corrected (D-066–D-068) |
| 06c | Paper-fidelity items + headline comparison | dead-time model (default OFF, Gate D explained); conventional baseline cannot lock; matched/FLC headline only (D-069/D-070) |
| 06c-2 | Handover-independent comparison; matched dips; closed-loop dead time | fixed-time t_h survey; bandwidth does not close the gap; improved beats conventional; DSOGI hurts a clean observer; Gate D′ confirms dead time, compensation restores (D-071–D-074) |
| 06e | PLL bandwidth at the matched dips + ablation ladder (no src change) | f_n sweep: gap is **not** a PLL-bandwidth artefact; **KNOWN DEFECT — non-paper, do not use:** the normalized Q-PLL option (`normalize=True`) does not lock from a +90° start; ladder R1–R6 (D-075–D-078) |
| 06f | Replay decomposition of the dynamic error (no src change) | Gate R exact; **DSOGI-FLL** is the element (observer 3–5 %, FLL 1.5–9×); omega_hat path carries the instability; 06e verdict corrected; no further bandwidth work (D-079–D-082) |
| **06** | Simulation experiments vs. baselines and release documentation | **COMPLETE (PROVISIONAL)** — comparison, parameter register, limitations, regeneration audit, and `phase-06-complete` release tag |
| 06g-2 | Paper-implied transient + transient-profile test | **COMPLETE (PROVISIONAL)** — prescribed-profile generator regression exact; gentler-transition hypothesis supported; closed-loop ramp confirmation run; no `src` changes (D-085–D-087) |
| 07 | Evaluation, plots, comparison with reported results | not started (next phase) |

> **Tag status (Phase 06g Step 0e).** `phase-06f-complete` and `phase-06-complete` are
> **PROVISIONAL**: they were cut before the Phase 06g figure re-read and the attribution
> correction, and the Phase 06g STOP means **no new final tag is created**. Do not cite the
> Phase 06 comparison numbers as final while these tags are provisional.

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
