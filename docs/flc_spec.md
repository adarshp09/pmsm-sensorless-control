# FLC / DSOGI-FLL paper spec sheet (Phase 03e Step 5)

Extracted from rendered page images of `docs/act.pdf` (not just the text layer).
Every item is marked **PRINTED** (with page/figure/equation) or **UNPRINTED**.
Unprinted items are `null` in `docs/parameter_extraction.md` and must not be guessed.
The FLC is **not implemented** in this phase.

## (a) Signal chain into the DSOGI-FLL

| Item | Status | Source |
| --- | --- | --- |
| DSOGI-FLL input is the SMO back-EMF estimate `E_hat = k*g(S)` (Eq.(7)) | **PRINTED** | Eq.(7), p.2; Fig.7, p.5 |
| Filtered output is `E_hat_f(s) = E_hat(s) D(s)` | **PRINTED** | Eq.(12), p.3 |
| Two SOGIs (alpha, beta) share one FLL | **PRINTED** | Section III-B, p.4 |
| Positive/negative-sequence combination equations | **UNPRINTED** | item 8, `paper_method.md` |
| FLL has a "gain normalization unit" | **PRINTED (label only)** | Fig.5, p.4 |
| Algebra of the normalization unit | **UNPRINTED** | text, p.4 |
| An explicit `omega'` factor in the `Gamma` term | **UNPRINTED** | text/Fig.5 only |
| `Gamma = 50`, `k_s = sqrt(2)` | **PRINTED** | p.5 |

The paper says normalization makes the FLL response "only depend on `Gamma`", but
never prints the normalized error expression, so the normalization is an option,
off by default (`normalize_error`).

## (b) Fuzzy logic controller

| Item | Status | Source / value |
| --- | --- | --- |
| Inputs | **PRINTED** | `s` = current estimation error, `s_dot` = its derivative; p.3 |
| Input fuzzy sets | **PRINTED** | {NB, NS, Z, PS, PB}; p.3 |
| Input universe of discourse | **PRINTED** | {-1, -0.5, 0, 0.5, 1}; p.3 |
| Input scaling gains (`Ke`, `Kec`) into [-1,1] | **UNPRINTED** | not given |
| Per-unit base for `s` | **UNPRINTED** | only the back-EMF base 311/sqrt(3) is given |
| Sample time at which `s_dot` is computed | **UNPRINTED** | no FLC-specific rate stated |
| MF shape | **PRINTED** | triangular (Fig.2, p.3) |
| Input MF breakpoints | **UNPRINTED** | Fig.2 only, no numbers |
| Output fuzzy sets | **PRINTED** | {Z, PS, PM, PB}; p.3 |
| Output universe | **PRINTED** | [0, 1]; p.3 |
| Output MF breakpoints | **UNPRINTED** | Fig.2 |
| Rule table `aF` | **PRINTED** | Table I, p.4 (in `configs/observer.yaml` -> `specs.fuzzy_rule_table.output_aF`) |
| Rule table `kF` | **PRINTED** | Table II, p.4 (`specs.fuzzy_rule_table.output_kF`) |
| Inference | **PRINTED** | Mamdani; p.3 |
| Defuzzification | **PRINTED** | centre of gravity; p.3 |
| Output mapping | **PRINTED** | `k = kF * k0`, `a = aF * a0`; pp.3-4 |
| `k0`, `a0` numeric values | **PRINTED** | `k = 1` (p.5), `a = 4` (p.5), both per-unit |

The four rules per output (verbatim, `rows` = `s`, `columns` = `s_dot`):

`aF`: NB -> [PB,PB,PB,PM,PS], NS -> [PB,PM,PM,PS,PS], Z -> [PS,Z,Z,Z,PS],
PS -> [PS,PS,PM,PB,PB], PB -> [PS,PM,PB,PB,PB].

`kF`: NB -> [PB,PB,PM,PS,Z], NS -> [PM,PM,PS,Z,PS], Z -> [PS,Z,Z,Z,PS],
PS -> [PS,Z,PS,PM,PM], PB -> [Z,PM,PB,PB,PB].

## (c) SI ranges and constraints under D-015

D-015 bases: `k0 = 179.63 V`, `a0 = 0.539 1/A` (from `k_pu = 1`, `a_pu = 4`).

| Quantity | Range / constraint |
| --- | --- |
| `k = kF k0` | `[0, 179.63] V` |
| `a = aF a0` | `[0, 0.539] 1/A` |
| `kF, aF` | `[0, 1]` (output universe) |
| stability at `Ts = 100 us` | `k a < 2(2Ld/Ts - Rs) = 57.8` -> **`kF aF < 0.60`** |
| tracking at 1000 rpm | **`kF > |E|max / k0 = 55.54 / 179.63 = 0.31`** |
| feasible box at 100 us | `kF` in `(0.31, 1]`, `aF` in `(0, min(1, 0.60/kF)]` |

So the FLC can stabilise the 100 us loop only in the lower ~60 % of its `aF` output
range; at full `kF` the stable slope is `aF <= 0.597` (`a <= 0.322 1/A`, matching
D-027). The FLC is not implemented.
