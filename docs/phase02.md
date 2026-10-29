# Phase 02 — Observer verification findings

Verification pass over the Phase 02 observers. No new features; defaults are
unchanged (`k_si = 179.629 V`, `a_si = 0.53889 1/A`, `I_base = 7.4227 A`).

Reproduce with:

```bash
python experiments/02_smo_validation.py      # Phase 02 baseline
python experiments/02b_smo_verification.py   # this pass
```

Scenario is unchanged from Phase 02: short-circuited plant (``u = 0``), the
paper's no-load speed ramp 500 -> 1000 r/min (0 -> 1.5 s, then held), identical
inputs to both observers, both starting from zero current.

## 1. Steady-state window metrics

Ramp ends at 1.5 s; the current settles within a few electrical time constants
(`Ld/Rs = 2.9 ms`), so the window used is **`t in [1.75, 2.0] s`** (last 0.25 s;
~16.7 electrical cycles at 1000 r/min). Peak true current in the window is
`|i| = 69.75 A`.

| Metric (steady window) | conventional (sign) | sigmoid (a = 0.5389) |
| --- | --- | --- |
| RMSE \|i_hat - i\| (A) | 10.23 | 7.06 |
| max \|i_hat - i\| (A) | 18.46 | 9.02 |
| mean \|S\| (A) | 9.58 | 6.88 |
| max \|S\| (A) | 18.46 | 9.02 |

Steady-window RMSE reduction: **31.0 %**.

## 2. Chattering metric

Definition: `E_hat` minus its moving average over exactly one electrical period
(150 samples = 15 ms at 1000 r/min; the periodic fundamental is removed, the
switching ripple is not). Per-axis standard deviation and combined RMS:

| Metric | conventional (sign) | sigmoid (a = 0.5389) |
| --- | --- | --- |
| std(E_hat_alpha) (V) | 179.64 | 148.10 |
| std(E_hat_beta) (V) | 179.60 | 148.19 |
| RMS chatter (V) | 254.02 | 209.51 |

Chattering reduction at the default `a`: **17.5 %**. For the sign observer the
per-axis std equals `k` (179.63 V), i.e. near full bang-bang switching; the
sigmoid reduces it, so the chattering metric **does** support a reduction, but a
modest one at the paper-derived default `a`.

> **Re-based (Phase 03b).** The `std(E - one-period moving average)` metric above is
> ill-posed — the MA window itself attenuates and lags the fundamental. Under the
> corrected **ripple** definition (RMS about the least-squares fundamental at the
> true electrical angle; see `docs/phase03.md`), the steady-window sign-vs-sigmoid
> ripple reduction is **17.7 %** (252.6 -> 207.8 V). The conclusion is unchanged; the
> number above is superseded by 17.7 %.

## 3. Is sliding actually reached?

Steady-window statistics of `S_alpha`:

| Observer | mean S_alpha (A) | std S_alpha (A) | min / max (A) |
| --- | --- | --- | --- |
| conventional (sign) | 0.034 | 7.19 | -13.56 / 13.56 |
| sigmoid | 0.031 | 4.98 | -7.99 / 7.99 |

`S_alpha` is zero-mean and bounded, i.e. the state is on a sliding surface with a
finite chattering band (≈ ±13.6 A for sign, ≈ ±8.0 A for sigmoid), not diverging
or biased. See `sliding_surface_zoom.png`.

## 4. Sigmoid slope sweep (`a`, equivalently `I_base ± 50 %`)

`k` is independent of `I_base`; changing `I_base` by a factor only rescales `a`
(`a_si = a_pu / I_base`). Both sweeps are therefore shown as one table, with the
equivalent `I_base` multiplier in the second column. Defaults unchanged.

| `a` (1/A) | `I_base` mult. | steady RMSE (A) | mean \|S\| (A) | max \|S\| (A) | chatter RMS (V) |
| --- | --- | --- | --- | --- | --- |
| 0.0500 | 10.78 | 9.29 | 9.29 | 9.35 | 41.1 |
| 0.1000 | 5.39 | 4.18 | 4.18 | 4.22 | 37.1 |
| **0.2000** | 2.69 | **2.01** | **2.01** | **2.04** | **35.7** |
| 0.3593 | 1.50 | 3.84 | 3.53 | 5.47 | 105.9 |
| 0.4311 | 1.25 | 6.02 | 5.85 | 7.74 | 173.4 |
| 0.5389 | 1.00 (default) | 7.06 | 6.88 | 9.02 | 209.5 |
| 0.7185 | 0.75 | 7.56 | 7.20 | 12.13 | 217.8 |
| 1.0778 | 0.50 | 8.98 | 8.28 | 13.90 | 222.5 |
| 2.0000 | 0.27 | 9.51 | 8.79 | 16.05 | 233.7 |
| 4.0000 | 0.13 | 9.81 | 9.15 | 17.38 | 247.7 |

Observations:

- Both steady RMSE and chattering fall as `a` is reduced until an optimum near
  **`a ≈ 0.20 1/A`** (`I_base × 2.7`), where RMSE ≈ 2.0 A and chatter ≈ 35.7 V.
- Below the optimum (`a = 0.05`) RMSE rises again (9.29 A) even though chattering
  stays low: the switching term is too weak and sliding is barely maintained.
- The paper-derived default (`a ≈ 0.539`, `I_base × 1.0`) is well above the
  optimum because the paper's `a = 4` is per-unit and `I_base` is a *derived*
  base (D-015), not printed. The recurrence of `a`/`k` adaptation is exactly what
  Phase 04 (FLC) is required to provide.

## 5. Back-EMF reconstruction tolerance (test)

`tests/test_conventional_smo.py::test_back_emf_reconstruction_averages_to_true_emf`
compares the mean estimated back-EMF against `E = 6 V`, so the tolerance is in
**volts, not amperes**. The measured duty-cycle bias is ~0.02 V; the tolerance was
loosened to 0.1 V (≈ 1.7 % of |E|, ~5× the measured bias) and the units are now
stated in the test.

## Phase 03c re-base (post-D-025 fix)

Everything above was computed with the **pre-fix** reported signal
`back_emf = k*g(S_post)` (D-025). After the fix (report the applied `g(S_pre)`) the
numbers change as follows. RMSE is on the *current estimate*, which the fix did not
touch; `|S|` and the back-EMF metrics do change.

| Claim (above) | pre-fix | post-fix |
| --- | --- | --- |
| steady RMSE, sign / sigmoid (A) | 10.23 / 7.06 | 10.23 / 7.06 (unchanged) |
| steady RMSE reduction | 31.0 % | **30.97 %** |
| mean \|S\|, sign / sigmoid (A) | 9.58 / 6.88 | **9.82 / 7.13** |
| max \|S\|, sign / sigmoid (A) | 18.46 / 9.02 | **18.03 / 9.98** |
| sign-vs-sigmoid "chatter" reduction | 17.5 % (MA, retracted) / 17.7 % (ripple) | **21.6 %** (ripple; **limit-cycle vs bang-bang, 100 µs** — D-027/D-029) |
| steady raw `E_hat` fundamental gain / phase, sign | 0.48 / -60.1 deg | **0.958 / -1.50 deg** |
| steady raw `E_hat` fundamental gain / phase, sigmoid | 0.48 / -59.4 deg | **0.963 / -1.31 deg** |
| ramp raw `E_hat` fundamental gain / phase, sign / sigmoid | - | **0.961 / -1.66 deg ; 0.959 / -1.33 deg** |

The pre-fix `E_hat` gain/phase (~0.5 / -60 deg) were an artifact of D-025, not a
property of the observers. The sign-vs-sigmoid ripple reduction at `Ts = 100 µs` is
now **21.6 %** — but this is a **limit-cycle vs bang-bang** comparison (the sigmoid
is a period-2 limit cycle at `Ts ≥ 65 µs`, D-027/D-029), not a fair
improved-vs-conventional one; the fair comparison is the stable `Ts = 50 µs` regime
in the "Phase 03d" section below.

## Phase 03d — regime relabel and stable-regime comparison

The 100 µs sign-vs-sigmoid comparison above is **not** a fair "improved vs
conventional" comparison: at `Ts = 100 µs` the sigmoid observer is a **period-2
limit cycle** (D-027: `Ts* = 60.1 µs`), while the sign observer is **bang-bang**.
Relabel those rows **"limit-cycle vs bang-bang"**.

In the **stable regime** (`Ts = 50 µs`, same ramp scenario, steady window
`[1.75, 2.0] s`, post-D-025 fix):

| observer | RMSE (A) | mean \|S\| (A) | max \|S\| (A) | raw gain / phase | raw ripple (V) |
| --- | --- | --- | --- | --- | --- |
| sign (bang-bang) | 5.10 | 4.97 | 10.07 | 0.980 / −0.82° | 248.1 |
| sigmoid (stable) | **0.922** | **1.157** | **1.167** | 0.984 / −0.38° | **0.02** |

The sigmoid's advantage is much larger in the stable regime (RMSE −82 %, ripple
~0) than the 100 µs "limit-cycle vs bang-bang" numbers suggest.

## Phase 03e — retraction note and low-speed limitation

- **RETRACTED — non-paper option (D-028):** any *filtered* number in this project
  computed with `epsilon_filter_tau = 10 ms` (the 03d Step-3 table, the −5 to −7 Hz
  frequency errors, filtered gain ≈ 0.81, and the 90.9/88.1 % and 92.7/91.4 % ripple
  reductions) is invalid. The observer-only numbers in this document do not use the
  filter and stand. Library-default re-base: `docs/phase03.md` §13.
- **Invalid for ramp (Phase 03f):** every *ramp-window* ripple, ripple-reduction and
  ω′-std value in this document used a stationary phasor fit and is **invalid for a
  ramp**. Use the windowed fit in `docs/phase03.md` §14.6.
- **Low-speed limitation (Step 6):** the sign observer's raw back-EMF fundamental
  gain is **0.76 at 100 rpm** (0.93 at 200 rpm), and is **independent of the fit
  window length (1/2/4/8 cycles, half-window scatter ≤ 0.014)** — a real low-speed
  bias, not fit noise.

## Verdict

- Sliding is genuinely reached (zero-mean, bounded `S`).
- At the **default** sigmoid slope the chattering metric supports a **modest
  reduction (17.5 %)** alongside a 31 % steady-RMSE reduction — claimed only at
  this magnitude.
- The reduction is strongly configuration-dependent; a much larger reduction is
  available by lowering `a` (raising `I_base`), which the paper delegates to the
  FLC. No claim of reproducing the paper's reported accuracy is made: no rotor
  position/speed estimation exists yet, and the current base is a documented
  assumption (D-015).
