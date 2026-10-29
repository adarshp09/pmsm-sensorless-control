# Phase 03 — DSOGI-FLL findings (with Phase 03b verification)

Implementation of the paper's DSOGI-FLL back-EMF filter (Section III-B) and a
verification/diagnosis pass. No rotor position/speed estimator is implemented
(Phase 05). No plant/observer/filter code or defaults were changed in Phase 03b.

Reproduce:

```bash
python experiments/03_dsogi_fll_validation.py       # filter validation
python experiments/03b_dsogi_fll_diagnosis.py       # reference check + sweeps
python experiments/02b_smo_verification.py          # SMO comparison (re-based)
```

## 0. RETRACTION NOTICE (Phase 03e, D-028)

Every filtered number computed **with `epsilon_filter_tau = 10 ms`** is
**RETRACTED — non-paper option (D-028)**. This covers:

- the **−5 to −7 Hz** filtered frequency errors (sections 7, 10, 12.3),
- the **filtered gain ≈ 0.81**,
- the **90.9 % / 88.1 %** (steady) and **92.7 % / 91.4 %** (ramp) ripple reductions,
- the **03d Step-3 table** (section 12.3).

Old values are kept below, labelled, next to the library-default replacements in
section 13. The error LPF is **OFF by default** in `DSOGIFLLParameters`; only the
Phase 03/03d experiments enabled it. The library-default re-base (section 13) gives,
e.g., sigmoid @50 µs: **0.000 Hz, filtered gain 0.984**; sign @100 µs: **+7.9 Hz,
filtered gain 0.977**.

## 1. Equations implemented

| Piece | Equation | Source |
| --- | --- | --- |
| SOGI band-pass `D(s)` | `k_s w' s / (s^2 + k_s w' s + w'^2)` | Eq.(9), p.4 |
| SOGI low-pass `Q(s)` | `k_s w'^2 / (s^2 + k_s w' s + w'^2)` | Eq.(10), p.4 |
| Discretization | `tan(w'Ts/2)/w' (1+z^-1)/(1-z^-1)` | Eq.(17), p.4 |
| `D(z)`, `Q(z)` | prewarped-bilinear biquads | after Eqs.(18)-(19); D-019 |
| Positive sequence | `v_a+ = (v'_a - qv'_b)/2`, `v_b+ = (qv'_a + v'_b)/2` | derived (item 8) |
| FLL error | `eps_f = qv'_a(v_a-v'_a) + qv'_b(v_b-v'_b)` | derived (item 9) |
| FLL integrator | `d w'/dt = -Gamma eps_f` | Fig.5, p.4; `Gamma=50`, p.5 |

### Flagged inconsistency (D-019) — survives verification

The coefficients **printed** in Eqs.(18)-(19) carry an extra `k_s` in the `z^-1`
denominator term and a factor 2 in the `D` numerator. This was verified against a
rendered crop of the PDF page (not just the text layer) and **the discrepancy
survives**: evaluated at resonance, the printed form gives `|D| = 0.0135` and
`Q(1) = -0.0068`, contradicting Eq.(17) and the paper's own Fig.6 claim that the
fundamental passes unchanged. The implementation uses the prewarped-bilinear
coefficients, which satisfy `|D|=1`, `angle(D)=0`, `|Q|=1`, `angle(Q)=-90 deg`,
`Q(1)=k_s` exactly (tests in `tests/test_sogi.py`).

The paper **does** state a sample time: `Ts = 100 us` (Eq.(20), p.5), 10 kHz
(Table III, p.6).

## 2. Metrics (Phase 03b definitions)

Represent an `alpha-beta` vector as `z = e_alpha + j e_beta`; let `theta_e` be the
**true** electrical angle (evaluation layer only; recovered from the plant as
`E_true = j omega_e psi_f exp(j theta_e)`).

- **fundamental gain / phase** — least-squares fit of the fundamental at the true
  electrical angle: `P = mean(z exp(-j theta_e))`. `fundamental_gain = |P|/|P_true|`,
  `fundamental_phase_deg = angle(P/P_true)`.
- **amplitude error** — `|fundamental_gain - 1|` (relative fundamental amplitude error).
- **ripple** — `sqrt(mean(|z - P exp(j theta_e)|^2))`: RMS about the fitted
  fundamental, i.e. the switching/harmonic content. This **replaces** the earlier
  `std(E - one-period moving average)` metric, which was ill-posed (the MA window
  itself attenuates/lags the fundamental).
- **total error** — `sqrt(mean(|z - z_true|^2))`.

Implementation: `experiments/validation_metrics.py`.

## 3. Reference check

- `|E_true| = psi_f * omega_e` exactly (Eq.(2)): 100 rpm 5.554, 200 rpm 11.109,
  500 rpm 27.772, 1000 rpm 55.543 V -- measured values match to machine precision.
- Frame/sign: `z_true = j omega_e psi_f exp(j theta_e)`; `angle(z) = theta_e + 90 deg`.
- Time alignment (one-sample offset test, 1000 rpm, `Ts=100 us`): the fit at
  `theta_e[k]` gives phase `~0`; shifting by one sample gives `+omega_e*Ts = 2.4 deg`.
  The plant's `E[k]` is `back_emf(theta_e[k])` with **no off-by-one**. Regression
  test: `tests/test_back_emf_reference.py`.

## 4. Raw-SMO fundamental sweeps (`experiments/output/03b_diagnosis/`)

Speed sweep at `Ts = 100 us` (raw `E_hat`, gain / phase):

| rpm | sign gain / phase | sigmoid gain / phase |
| --- | --- | --- |
| 100 | 0.78 / -7.3 deg | 0.91 / -2.9 deg |
| 200 | 0.83 / -16.2 deg | 0.94 / -9.6 deg |
| 500 | 1.13 / -38.1 deg | 0.80 / -23.8 deg |
| 1000 | 0.61 / -58.1 deg | 0.48 / -59.5 deg |

`Ts` sweep at 1000 rpm (raw gain / phase / ripple):

| Ts | sign | sigmoid |
| --- | --- | --- |
| 100 us | 0.61 / -58.5 / 251.8 V | 0.48 / -59.4 / 207.8 V |
| 50 us | 0.50 / -46.5 / 252.5 V | 0.79 / -87.7 / 0.51 V |
| 25 us | 0.53 / -50.8 / 252.3 V | 0.65 / -37.9 / 0.50 V |
| 10 us | 0.60 / -48.6 / 251.9 V | 0.82 / -11.7 / 0.26 V |
| 5 us | 0.62 / -52.7 / 251.7 V | 0.90 / -5.7 / 0.14 V |
| 2.5 us | 0.67 / -53.8 / 251.3 V | 0.94 / -3.1 / 0.065 V |

**Answer to "first-order lag vs scales with `Ts`":** neither.
- The **sigmoid** observer's raw gain/phase are **`Ts`-dependent** (discretization /
  switching-ripple aliasing): as `Ts` shrinks it approaches `(1, 0)`; its ripple
  collapses with `Ts` (208 V -> 0.065 V).
- The **sign** observer is `Ts`-**independent** (gain ~0.6, phase ~-52 deg at every
  `Ts`, ripple ~252 V): a *structural* effect, not a lag and not `Ts` (cause below).

## 5. Continuous-limit check (1000 rpm)

| Ts | sign gain / phase | sigmoid gain / phase |
| --- | --- | --- |
| 1 us | 0.636 / -51.2 deg | 0.971 / -1.65 deg |
| 0.5 us | 0.624 / -51.7 deg | 0.980 / -1.19 deg |
| 0.25 us | 0.637 / -50.6 deg | 0.985 / -0.96 deg |

- **Sigmoid: yes** -- gain -> 1, phase -> 0 (pure discretization effect).
- **Sign: no** -- it plateaus. Cause located (next section).

## 6. Located cause — a Phase 02 reporting-convention bug (reported, not fixed)

The observer's `step` reports `back_emf = k * g(S_post)` where `S_post = i_hat_new - i`
is recomputed **after** the update, whereas the injection actually applied during the
step uses `g(S_pre)`. For the **discontinuous sign** function the one-step overshoot
flips the sign of `S`, so `g(S_post)` is corrupted. Measured fundamental of the
*applied* vs *reported* signal (1000 rpm):

| Ts | applied `k*sign(S_pre)` | reported `k*sign(S_post)` |
| --- | --- | --- |
| 100 us | gain 0.957, phase -1.6 deg | gain 0.608, phase -58.5 deg |
| 10 us | gain 0.996, phase -0.2 deg | gain 0.613, phase -48.8 deg |

So the applied injection reconstructs `E` correctly; the **reported** back-EMF does
not.

**Correction (Phase 03c Step 1):** the earlier statement "the sigmoid is nearly
unaffected" is **REFUTED**. Tabulating both observers at 100/50/25 us, 1000 rpm, the
sigmoid reported signal is also badly corrupted (100 us: reported 0.479 / -59.4 deg
vs applied 0.962 / -1.3 deg; 50 us: 0.792 / -87.7 deg vs 0.984 / -0.4 deg; 25 us:
0.653 / -37.9 deg vs 0.987 / -0.6 deg). Both variants were affected.

**Status:** fixed in Phase 03c (D-025); see section 10.

## 7. Validation results — **PRE-FIX and RETRACTED** (see §10 and §13)

> Computed with the pre-fix reported signal `k*g(S_post)` (D-025) **and** with
> `epsilon_filter_tau = 10 ms` (D-028). Every filtered number here is
> **RETRACTED — non-paper option (D-028)**. Superseded by §10 (signal) and §13
> (library-default filter).


Steady window `t in [1.75, 2.0] s`; ramp window `t in [0, 1.5] s`; both observers,
identical inputs, defaults unchanged; options `epsilon_filter_tau = 10 ms` and
`min_omega = 2*pi*2 rad/s` (documented, not paper values).

Raw -> filtered ripple reduction (100 us): sign 252.6 -> 7.3 V (**97.1 %**);
sigmoid 207.8 -> 1.2 V (**99.4 %**). Over the ramp: sign 88.8 %, sigmoid 97.6 %.

| window | observer | freq err mean (Hz) | raw gain / phase | filtered gain / phase | raw -> filtered ripple (V) |
| --- | --- | --- | --- | --- | --- |
| steady | sign | +7.26 | 0.48 / -60.1 deg | 0.50 / -42.2 deg | 252.6 -> 7.3 |
| steady | sigmoid | +0.55 | 0.48 / -59.4 deg | 0.48 / -25.0 deg | 207.8 -> 1.2 |
| ramp | sign | -17.41 | 0.59 / -40.4 deg | 0.35 / -42.5 deg | 252.8 -> 28.2 |
| ramp | sigmoid | -0.12 | 0.60 / -39.7 deg | 0.60 / -40.0 deg | 213.9 -> 5.1 |

(The sign frequency error is large in the ramp because the near-bang-bang report
both aliases and lags the fast 500->1000 rpm ramp; the sigmoid tracks it.)

## 8. Re-based claims

- **`docs/phase02.md` "17.5 % chattering reduction"** — old metric was
  `std(E - one-period MA)`, ill-posed. Re-based on the ripple definition: the
  sigmoid reduces the raw ripple by **17.7 %** versus the sign observer in the
  steady window (252.6 -> 207.8 V). The numeric result stands; the metric is now
  well defined.
- **`docs/phase03.md` "87-89 % chatter reduction"** — old metric was the same flawed
  MA. Re-based: the DSOGI-FLL reduces the raw ripple by **97.1 % (sign)** and
  **99.4 % (sigmoid)** in the steady window at `Ts = 100 us` (over the ramp:
  88.8 % / 97.6 %). The claim is **strengthened** under the corrected metric.

## 9. Limitations

- The DSOGI-FLL is a filter; it cannot recover a fundamental the SMO did not
  estimate. Filtered amplitude/phase accuracy inherits the raw estimate quality.
- The sign observer's reported back-EMF is corrupted by the reporting bug (section 6);
  its filtered fundamental is correspondingly wrong until that is fixed.
- The paper's `Gamma = 50` frame and normalization algebra are unresolved; the
  un-normalized loop bandwidth is amplitude-dependent (D-022).
- No rotor position or speed is estimated here; frequency error is a filter metric
  only. Position/speed estimation is Phase 05.

## 10. Phase 03c re-base (post-D-025 fix) — filtered numbers still **RETRACTED**

> These use the post-D-025 signal but still `epsilon_filter_tau = 10 ms`; every
> filtered value is **RETRACTED — non-paper option (D-028)**. The library-default
> replacement is §13.

After the fix, the SMO feeds the DSOGI-FLL the **applied** injection `k*g(S_pre)`
instead of the corrupted `k*g(S_post)`. Every number in sections 4-8 came from the
**pre-fix** signal; the post-fix values are:

| Claim (pre-fix) | pre-fix | post-fix |
| --- | --- | --- |
| raw `E_hat` gain / phase, sign (100 us) | 0.48 / -60.1 deg | **0.958 / -1.50 deg** |
| raw `E_hat` gain / phase, sigmoid (100 us) | 0.48 / -59.4 deg | **0.963 / -1.31 deg** |
| steady raw -> filtered ripple reduction, sign | 97.1 % | **90.9 %** |
| steady raw -> filtered ripple reduction, sigmoid | 99.4 % | **88.1 %** |
| ramp raw -> filtered ripple reduction, sign / sigmoid | 88.8 % / 97.6 % | **92.7 % / 91.4 %** |
| steady frequency error, sign / sigmoid (Hz) | +7.26 / +0.55 | **-5.21 / -6.36** |
| ramp frequency error, sign / sigmoid (Hz) | -17.41 / -0.12 | **-3.97 / -3.73** |
| steady filtered gain / phase, sign | 0.50 / -42.2 deg | **0.811 / -9.45 deg** |
| steady filtered gain / phase, sigmoid | 0.48 / -25.0 deg | **0.821 / -9.82 deg** |
| continuous-limit sign gain / phase (0.25 us) | 0.637 / -50.6 deg | **1.000 / -0.00 deg** |
| continuous-limit sigmoid gain / phase (0.25 us) | 0.985 / -0.96 deg | 0.989 / -0.74 deg |

Interpretation: the pre-fix sign "plateau" (section 4) and the pre-fix low filtered
gain (~0.5) / large phase were **artifacts of D-025**. Post-fix the raw `E_hat`
gain/phase are speed-dependent (per-speed values in the table below, not a single
"~4 %" bound): sign gain 0.783 / 0.912 / 0.936 / 0.958 and phase −1.16° / +1.91° /
−1.34° / −1.81° at 100/200/500/1000 rpm; sigmoid gain 0.970 / 0.934 / 0.949 /
0.964 and phase −1.88° / +0.05° / −0.98° / −1.40°. The sign frequency error drops
from +7.3 to -5.2 Hz; and the raw -> filtered ripple reduction is ~88-93 % at
`Ts=100 us` (it was inflated to 97-99 % by the pre-fix signal's extra chatter). Raw
`E_hat` gain/phase post-fix, both observers, 100-1000 rpm:

| rpm | sign | sigmoid |
| --- | --- | --- |
| 100 | 0.783 / -1.16 deg | 0.970 / -1.88 deg |
| 200 | 0.912 / +1.91 deg | 0.934 / +0.05 deg |
| 500 | 0.936 / -1.34 deg | 0.949 / -0.98 deg |
| 1000 | 0.958 / -1.81 deg | 0.964 / -1.40 deg |

## 11. Discrete stability of the sigmoid observer (Phase 03c Step 3)

Forward Euler on the linearised error dynamics gives
`S[k+1] = (1 - Ts (Rs + k a/2)/Ld) S[k]`, stable iff `Ts (Rs + k a/2)/Ld < 2`,
i.e. `Ts* = 2 Ld/(Rs + k a/2)`. With `k = 179.63 V`, `a = 0.5389 1/A`,
`Ld = 1.47 mH`, `Rs = 0.505 ohm`: **`Ts* = 60.1 us`**.

Sweep at 1000 rpm (sigmoid, post-fix; `experiments/output/03c_stability/`):

| Ts (us) | Ts*lam/2 | gain | phase (deg) | ripple (V) | period-2 | limit cycle |
| --- | --- | --- | --- | --- | --- | --- |
| 100 | 3.33 | 0.962 | -1.26 | 194.7 | 0.90 | yes |
| 80 | 2.66 | 0.974 | -0.67 | 164.5 | 0.91 | yes |
| 65 | 2.16 | 0.983 | -0.32 | 70.6 | 0.68 | yes |
| 60 | 2.00 | 0.983 | -0.31 | 0.018 | 0.00 | no |
| 55 | 1.83 | 0.984 | -0.34 | 0.018 | 0.00 | no |
| 50 | 1.66 | 0.984 | -0.38 | 0.018 | 0.00 | no |
| 40 | 1.33 | 0.985 | -0.45 | 0.018 | 0.00 | no |

The observed boundary falls exactly at `Ts*lam/2 = 2`, matching `Ts*`. **At the
paper's `Ts = 100 us` the sigmoid observer is unstable** (period-2 limit cycle) with
these SI gains. Largest stable slope at `Ts = 100 us`: `a <= 0.322 1/A`, implying a
per-unit current base `I_base >= 12.4 A` (we use 7.42 A, D-015). The analytical
small-signal test is `tests/test_smo_discrete_stability.py`. D-016 (forward Euler) is
unchanged; an implicit/exact injection discretization is noted as a possible *option*
only (it would remove the `Ts` bound) and is not implemented.

## 12. Phase 03d — filter isolation and paper check

Script: `experiments/03d_filter_isolation.py`; output
`experiments/output/03d_filter_isolation/isolation.json`. No code/defaults changed.

### 12.1 Filter isolation (1000 / 500 rpm; window = 8 whole electrical cycles)

Window: 8 cycles → 120 ms at 1000 rpm (66.67 Hz), 240 ms at 500 rpm (33.33 Hz).
Raw-observer ripple RMS used for the synthetic cases: 248.4 V (1000 rpm), 252.7 V
(500 rpm). Each case is run with the Phase 03 option (error LPF on) and with the
library default (LPF off).

| case | 1000 rpm LPF on | 1000 rpm LPF off | 500 rpm LPF on | 500 rpm LPF off |
| --- | --- | --- | --- | --- |
| (a) ideal `E_true` | −7.55 Hz, g 0.821 | **−0.003 Hz, g 1.000** | −29.9 Hz, g 0.075 | **−0.001 Hz, g 1.000** |
| (b1) + period-2 ripple | −7.38 Hz, g 0.827 | −64.7 Hz, g 0.696 | −6.28 Hz, g 0.774 | +99.7 Hz, g 1.253 |
| (b2) + white ripple | −64.7 Hz, g 0.353 | −64.7 Hz, g 2.260 | −31.3 Hz, g 0.930 | −30.8 Hz, g 5.144 |
| (c) SMO sign @100 µs (bang-bang) | −6.13 Hz, g 0.811 | **+9.02 Hz, g 1.004** | −2.38 Hz, g 0.837 | **+12.4 Hz, g 1.010** |
| (c) SMO sigmoid @100 µs (limit cycle) | −6.45 Hz, g 0.808 | +4.13 Hz, g 0.951 | −2.68 Hz, g 0.837 | −31.0 Hz, g 0.506 |
| (c) SMO sigmoid @50 µs (stable) | −7.06 Hz, g 0.834 | **+0.002 Hz, g 0.984** | −6.99 Hz, g 0.699 | **+0.005 Hz, g 0.987** |

**Conclusion:** the Phase 03 ~−5 Hz bias and ~0.85 gain ratio come from the **filter
loop configuration — the error low-pass `epsilon_filter_tau = 10 ms` that the Phase 03
experiment enabled** — not from the input ripple and not from the observer. With the
library default (no error LPF) the loop locks a clean input to −0.001 Hz / gain 1.000
and the post-fix stable SMO to ~0 Hz / 0.98. The LPF adds too much lag and makes the
loop oscillatory (16 Hz jitter at 1000 rpm; diverges at 500 rpm). The LPF is **off by
default** in `DSOGIFLLParameters`; the experiment enabled it. Reported in D-028.

The observer's own contribution (LPF off): sign (bang-bang) → gain ~1.00 but a
frequency bias/jitter of +9 Hz (1000 rpm) / +12 Hz (500 rpm); sigmoid limit cycle at
100 µs → larger errors (−31 Hz at 500 rpm); stable sigmoid at 50 µs → ~0 Hz, gain 0.98.

### 12.2 Amplitude dependence (1000 rpm, +10 % frequency step, library default LPF off)

| amplitude | default settling | default steady jitter | normalization ON settling | normalization ON jitter |
| --- | --- | --- | --- | --- |
| 0.1× | 0.64 s | 0.156 Hz | not settled in 1.2 s | 0.058 Hz |
| 1× | 0.0051 s | ~0 Hz | not settled in 1.2 s | 0.058 Hz |
| 10× | did not settle (diverged) | 10.2 Hz | not settled in 1.2 s | 0.058 Hz |

Settling scales as ~1/A² (loop gain ∝ ΓA²), as expected for the un-normalized loop;
at 10× it is unstable. Normalization removes the amplitude dependence but `Γ = 50`
then gives a very long time constant (>1.2 s). Defaults unchanged.

### 12.3 Post-fix filtered metrics — **RETRACTED** (Phase 03 run; error LPF on, D-028)

> **RETRACTED — non-paper option (D-028).** These were computed with
> `epsilon_filter_tau = 10 ms`. Kept for the record; the library-default numbers are
> in section 13.

Steady window `[1.75, 2.0] s`. "limit cycle" = D-027 (`Ts ≥ 65 µs`, sigmoid). The
sigmoid's raw ripple is already ~0.02 V at 50/25 µs, so the "reduction %" there is
meaningless (the FLL itself adds ripple via the LPF).

| Ts | observer | regime | freq err (Hz) | raw gain | filtered gain / phase | raw → filtered ripple (V) |
| --- | --- | --- | --- | --- | --- | --- |
| 100 µs | sign | bang-bang | −5.21 | 0.958 | 0.811 / −9.45° | 248.4 → 22.5 |
| 100 µs | sigmoid | **limit cycle** | −6.36 | 0.963 | 0.821 / −9.82° | 194.7 → 23.2 |
| 50 µs | sign | bang-bang | −5.84 | 0.980 | 0.816 / — | 248.1 → 23.5 |
| 50 µs | sigmoid | stable | −6.26 | 0.984 | 0.814 / — | 0.02 → 23.8 |
| 25 µs | sign | bang-bang | −7.20 | 0.989 | 0.804 / — | 248.0 → 24.6 |
| 25 µs | sigmoid | stable | −6.75 | 0.987 | 0.809 / — | 0.02 → 24.1 |

All of these frequency errors and gain droops are dominated by the error LPF (12.1).

### 12.4 Low-speed fit (sign observer)

| rpm | gain @ 1 / 2 / 4 / 8 cycles | phase (deg) | half-window gain std |
| --- | --- | --- | --- |
| 100 | 0.7578 / 0.7578 / 0.7578 / 0.7578 | +0.97 | ≤ 0.014 |
| 200 | 0.9263 / 0.9263 / 0.9263 / 0.9263 | +0.14 | ≤ 0.059 |

The 100 rpm gain (0.76 here; 0.78 in the earlier run) is **bias, not fit noise**: it is
independent of the window length (1–8 cycles) and the half-window scatter is ~0.

### 12.5 Paper check (rendered pages)

- **(a) Observer discretization/solver:** the paper specifies **no solver** for the
  observer and no simulation step separate from the sampling time. The only sample
  time is `Ts = 100 us` (Eq.(20), p.5) / 10 kHz (Table III, p.6). Only the SOGI
  integrator is discretized (Tustin with prewarping).
- **(b) Per-unit base values:** the only one stated is the back-EMF reference
  `311/sqrt(3)` V (p.5). No current base is given.
- **(c) FLC:** inputs `s` and `s_dot`, fuzzy sets {NB,NS,Z,PS,PB} over
  {−1,−0.5,0,0.5,1}; outputs `aF` and `kF`, sets {Z,PS,PM,PB} over [0,1]; triangular
  MFs; Mamdani inference; centre-of-gravity defuzzification. The outputs **multiply
  both** fixed parameters: `k = kF·k0`, `a = aF·a0` with `kF, aF ∈ [0,1]`.
  Under D-015 (`k0 = 179.63 V`, `a0 = 0.539 1/A`) the adapted ranges are
  `k ∈ [0, 179.63] V`, `a ∈ [0, 0.539] 1/A`. Stability `Ts(Rs + k a/2)/Ld < 2` at
  `Ts = 100 us` requires `k·a < 57.8`; i.e. `kF·aF < 0.60`, or at full `k`,
  `aF ≤ 0.597` (`a ≤ 0.322 1/A`, matching D-027). So the FLC *can* stabilise the
  100 µs loop by reducing `aF` (and/or `kF`), but only in the lower ~60 % of its
  output range. Not implemented.

### 12.6 Stable-regime comparison at Ts = 50 µs (post-fix)

At `Ts = 50 µs` the sigmoid is stable (D-027), so this is the fair "improved vs
conventional" comparison. Same ramp scenario, steady window `[1.75, 2.0] s`:

| observer | RMSE (A) | mean \|S\| (A) | max \|S\| (A) | raw gain / phase | raw ripple (V) |
| --- | --- | --- | --- | --- | --- |
| sign (bang-bang) | 5.10 | 4.97 | 10.07 | 0.980 / −0.82° | 248.1 |
| sigmoid (stable) | **0.922** | **1.157** | **1.167** | 0.984 / −0.38° | **0.02** |

The sigmoid is dramatically better in the stable regime (RMSE −82 %, ripple ~0).
The 100 µs comparison in sections 4–8 is therefore **"limit-cycle vs bang-bang"**, not
a fair improved-vs-conventional comparison.

## 13. Phase 03e — library-default re-base (Step 3)

Settings: **`epsilon_filter_tau=OFF` (library default), no clamps, no normalization**.
Initial `omega'` = true electrical frequency at the ramp start (500 rpm) =
209.44 rad/s (the paper states no initial frequency). Steady window =
**8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s**; ramp window =
**[0, 1.500] s** (500 -> 1000 rpm). Script: `experiments/03e_filter_rebase.py`.

| Ts | obs | window | regime | freq err (Hz) | ω′ std (Hz) | raw gain/phase | filt gain/phase | raw → filt ripple (V) | reduction | pred / meas phase (deg) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 100 µs | sign | steady | bang-bang | +7.91 | 6.85 | 0.958 / −1.58° | 0.977 / −7.44° | 248.40 → 8.07 | 96.7 % | +9.03 / −5.86 |
| 100 µs | sign | ramp | bang-bang | +9.60 | 10.41 | 0.961 / −1.65° | 0.992 / −8.57° | 250.85 → 9.75 | 96.1 % | +14.02 / −6.91 |
| 100 µs | sigmoid | steady | **limit cycle** | — | — | **FLL guard-fail: ω′<0 after D-027 limit cycle** | — | — | — | — |
| 100 µs | sigmoid | ramp | **limit cycle** | — | — | **FLL guard-fail: ω′<0 after D-027 limit cycle** | — | — | — | — |
| 50 µs | sign | steady | bang-bang | +4.97 | 2.78 | 0.981 / −0.86° | 1.000 / −3.92° | 248.13 → 2.40 | 99.0 % | +5.82 / −3.06 |
| 50 µs | sign | ramp | bang-bang | +5.09 | 9.91 | 0.981 / −0.86° | 1.013 / −4.89° | 250.72 → 8.40 | 96.7 % | +7.82 / −4.03 |
| 50 µs | sigmoid | steady | stable | +0.000 | 0.003 | 0.984 / −0.38° | 0.984 / −0.38° | 0.02 → 0.00 | (raw ~0) | 0.00 / 0.00 |
| 50 µs | sigmoid | ramp | stable | −0.123 | 9.76 | 0.986 / −0.34° | 0.978 / −0.39° | 7.87 → 8.31 | −5.6 % | −0.20 / −0.04 |
| 25 µs | sign | steady | bang-bang | +2.20 | 1.17 | 0.989 / −0.39° | 1.000 / −1.71° | 248.03 → 0.75 | 99.7 % | +2.63 / −1.32 |
| 25 µs | sign | ramp | bang-bang | +2.16 | 9.78 | 0.989 / −0.34° | 1.001 / −2.17° | 250.67 → 8.42 | 96.6 % | +3.43 / −1.83 |
| 25 µs | sigmoid | steady | stable | +0.000 | 0.003 | 0.987 / −0.56° | 0.987 / −0.56° | 0.02 → 0.00 | (raw ~0) | 0.00 / 0.00 |
| 25 µs | sigmoid | ramp | stable | −0.123 | 9.76 | 0.987 / −0.46° | 0.980 / −0.50° | 7.90 → 8.34 | −5.6 % | −0.20 / −0.04 |

Findings:

- With the library default the stable sigmoid locks **exactly** in the steady window
  (0.000 Hz, filtered gain 0.984, ripple 0.00 V). The sign observer keeps a **real
  +2 to +8 Hz bias** (decreasing with `Ts`) and its bang-bang ripple.
- The sigmoid at **100 µs** fails with the **FLL guard-fail: ω′<0 after D-027 limit
  cycle** (no usable filter output). **Correction (Phase 04 Step 0c):**
  the observed mechanism is **not** capture at an "absorbing ω′=0". Unclamped, ω′
  **overshoots through zero** and the non-negativity guard in `sogi_coefficients`
  raises (`ValueError`/NaN). The ≈2 Hz "pinning" seen in 03d/03e came from
  `min_omega = 12.566 rad/s`, not from an absorbing zero. This is the D-027
  instability, not a filter bug.
- Predicted vs measured phase: the given formula `atan((r²−1)/(k_s r))` has the
  **opposite sign** to the measured `filtered − raw` phase for the sign observer.
  Reported side by side; the sign convention is flagged (resolved in §14.2).
- Ramp jitter (~9.8 Hz) is the FLL tracking the 500→1000 rpm ramp; the steady window
  is the clean metric.
- **Ramp values invalid (Phase 03f):** all ramp-window ripple, ripple-reduction and
  ω′-std numbers in this section (and `phase02.md`) come from a *stationary* phasor
  fit and are **invalid for a ramp**. See §14.6 for the windowed fit.

## 14. Phase 03f — verification & disclosure

Script: `experiments/03f_verification.py`; output
`experiments/output/03f_verification/verification.json`. All blocks state their
options; the settings here are the **library default** (`epsilon_filter_tau=OFF`,
`min_omega=None`, no normalization).

### 14.1 Option disclosure (Step 1)

Non-default options actually used, from the `config_diff` blocks:

| script | `epsilon_filter_tau` | `min_omega` | notes |
| --- | --- | --- | --- |
| 03 | 0.01 s | 12.566 rad/s (2 Hz) | both enabled (D-028) |
| 03b | — | — | no DSOGI-FLL used |
| 03c | — | — | no DSOGI-FLL used |
| 03d | 0.01 s (LPF_on rows) / OFF (LPF_off rows) | **12.566 rad/s (2 Hz) on ALL rows** | so "LPF off" = "LPF off, min_omega=12.566 rad/s ON" |

03d rows whose ω′ was **pinned at the floor** (final ≈ 2 Hz; `ferr` ≈ −64.7 Hz at
1000 rpm, ≈ −31 Hz at 500 rpm): 1000 rpm `(b1) LPF_off`, `(b2) LPF_on`, `(b2) LPF_off`;
500 rpm `(b2) LPF_on`, `(b2) LPF_off` (≈2.6 Hz), and `(c) sigmoid@100 µs LPF_off`
(≈2.4 Hz). All other 03d rows are **not** floor-pinned. See D-028/D-029.

### 14.2 Off-resonance check and the phase-sign verdict (Step 2)

Permanent tests: `tests/test_sogi_off_resonance.py`. With ω′ frozen and
`z = e^{jωt}`, the implemented SOGI matches `D(jω)`, `Q(jω)` to **≤2×10⁻³**
relative at `ω/ω′ = 0.8, 0.9, 1.0, 1.1, 1.25`, and the positive-sequence output
matches **`½(D+jQ) = ½D(1+ω′/ω)`** to ≤3×10⁻³ (the prewarped biquad is exact at
`ω′` and the off-resonance warping is <1e-4; the bound is generous).

Phase comparison for the 03e sign rows (positive-sequence output):

| Ts | time-avg `atan((r²−1)/(k_s r))` using actual ω′(t), r=ω′/ω | measured closed-loop filtered−raw |
| --- | --- | --- |
| 100 µs | **+8.59°** | **−5.86°** |
| 50 µs | +5.74° | −3.06° |
| 25 µs | +2.62° | −1.32° |

**Verdict (corrected, Phase 04 Step 0b/0c): not a filter bug.** The independent
`scipy.signal.freqs` check (`tests/test_sogi_phase_independent.py`) proves the
implemented `D` phase is **`+atan((r²−1)/(k_s r))` with `r = ω′/ω`** (lead for
`ω′>ω`; +8.5° at `ω′=1, ω=0.9`), matching the supplied formula. A frozen-`ω′`
check of the raw→PNS transfer matches `½(D+jQ)` (r=1.104: +8.09° measured vs +7.98°
predicted; r=1.25: +17.68° vs +17.65°). **The earlier "−atan / inverted formula"
claim is retracted.** The static transfer is confirmed; the *closed-loop* measured
filtered−raw phase is negative. **HYPOTHESIS — untested:** it is explained by
`ω′(t)` being time-varying and correlated with the signal, so the static transfer
does not apply. This is carried to the Phase 05 prelude; it is not a filter bug and
has not been independently verified (e.g. by an `ω′(t)` replay).

### 14.3 Paper-literal output `v′` vs positive sequence (Step 3)

**Eq.(12) re-checked on the rendered page:** `Ê_f(s) = Ê(s)·D(s)` — the in-phase
band-pass, with **no** positive/negative-sequence combination printed. So the
paper-literal output is **`v′`**; the PNS output is the D-020 "non-paper option".

**API gap:** `DSOGIFLLOutput.v_alpha/v_beta` are the PNS output when
`positive_sequence=True` and the per-axis `v′` when it is `False`; the API does
**not** expose both simultaneously. To report both we run the same case twice
(same settings, flag toggled). A read-only accessor (e.g. `v_alpha_prime`,
`v_beta_prime`) is proposed as a **separate, approved step**.

Steady-window comparison (library default):

| Ts | observer | output | gain | phase | ripple (V) | FLL status |
| --- | --- | --- | --- | --- | --- | --- |
| 100 µs | sign | `v′` (paper-literal) | 0.922 | −2.04° | 7.46 | locked (bias +7.9 Hz) |
| 100 µs | sign | positive seq. | 0.977 | −7.44° | 8.07 | locked |
| 100 µs | sigmoid | both | — | — | — | **NaN** (ω′<0; see 14.4) |
| 50 µs | sign | `v′` | 0.965 | −0.83° | 2.89 | locked (+5.0 Hz) |
| 50 µs | sign | positive seq. | 1.000 | −3.92° | 2.40 | locked |
| 50 µs | sigmoid | `v′` | 0.984 | −0.38° | 0.01 | locked |
| 50 µs | sigmoid | positive seq. | 0.984 | −0.38° | 0.00 | locked |
| 25 µs | sign | `v′` | 0.985 | −0.37° | 1.24 | locked (+2.2 Hz) |
| 25 µs | sign | positive seq. | 1.000 | −1.71° | 0.75 | locked |
| 25 µs | sigmoid | `v′` | 0.987 | −0.56° | 0.01 | locked |
| 25 µs | sigmoid | positive seq. | 0.987 | −0.56° | 0.00 | locked |

The paper-literal `v′` has the lower gain for the detuned sign case (0.922 vs
0.977): `D` attenuates off resonance, while the PNS factor `½(1+ω′/ω)` offsets
the gain but adds the `∠D` phase.

### 14.4 Sigmoid @100 µs status (Step 4)

Raw `E_hat` metrics (independent of the filter): **gain 0.960, phase −1.31°,
ripple 203.7 V** (matches 03c's 0.963/−1.31°). FLL status = **NaN**, first NaN at
**0.0757 s**; ω′ just before = **−1.668 Hz**; the error is
`omega_prime must be non-negative` from `SOGIQSG` when ω′ crosses below zero.
**Not a `tan(ω′Ts/2)/ω′` singularity** — the biquad uses `sin/cos` and is finite at
ω′→0. It is the intended non-negativity guard rejecting a non-physical ω′ driven by
the D-027 limit cycle. Reported as a robustness note; no src change.

### 14.5 Collapse cases with no clamp (Step 5)

Rerunning the 03e collapse cases with `min_omega=None`: **ω′ does not decay
asymptotically — it crosses zero to negative and the SOGI raises (NaN).**

| rpm | case | result (no clamp) |
| --- | --- | --- |
| 1000 | ideal | locks (66.66 Hz) |
| 1000 | period-2 | NaN at 0.033 s (ω′ → −91.5) |
| 1000 | white | NaN at 0.021 s (ω′ → −11.4) |
| 1000 | sigmoid @100 µs (constant speed) | locks at 80.7 Hz (bias +14 Hz) |
| 500 | ideal | locks (33.35 Hz) |
| 500 | period-2 | NaN at 0.035 s (ω′ → −94.6) |
| 500 | white | NaN at 0.018 s (ω′ → −760) |
| 500 | sigmoid @100 µs | NaN at 0.078 s (ω′ → −24.1) |

So the observed collapse is a **zero-crossing to negative ω′**, after which the
non-negativity guard raises; there is **no observed capture at ω′=0**. The ≈2 Hz
"pinning" in 03d/03e came from `min_omega = 12.566 rad/s`, not from an absorbing
zero (Phase 04 Step 0c correction).

ε_f budget at frozen ω′=ω_true:
- synthetic white (32 seeds): mean ε_f = **+11.67 ± 448.44** — not significant
  (std ≫ mean); period-2 mean ε_f = **−3.6e−10 ≈ 0**.
- real sign signal: predicted offset `−mean ε_f,ripple / slope`:
  **+0.22 Hz (100 µs), +0.06 Hz (50 µs), +0.015 Hz (25 µs)** vs observed
  **+7.91 / +4.97 / +2.20 Hz**.

**Verdict: the open-loop ε_f budget does NOT explain the sign-observer bias.**
The sign agrees but the magnitude is **36× / 83× / 148×** too small (≫2×), and the
white-ripple mean is seed-dominated. **HYPOTHESIS — untested:** the bias is a
discrete/aliasing and nonlinear-loop effect rather than the steady-state ripple
ε_f; the budget is disproven but the mechanism is unproven and is carried to the
Phase 05 prelude.

### 14.6 Ramp metrics with the windowed fit (Step 6)

`experiments/validation_metrics.py` now provides `sliding_window_ramp_metrics`
(per 60 ms window, complex LS `z ≈ (G0 + G1(t−t_c))·z_true`) and
`summarize_ramp`. Permanent test: `tests/test_ramp_metrics.py`. Ramp = [0, 1.5] s.

Mean / worst-window (max ripple) over 25 windows:

| Ts | observer | output | mean gain | mean phase | mean ripple (V) | worst ripple (V) | ω′ err mean ± std (Hz) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 100 | sign | `v′` | 0.890 | −1.12° | 5.06 | 8.16 | +9.60 ± 4.55 |
| 100 | sign | positive seq. | 0.992 | −8.76° | 4.90 | 10.31 | +9.60 ± 4.55 |
| 100 | sigmoid | — | **NaN** | — | — | — | — |
| 50 | sign | `v′` | 0.956 | −0.93° | 2.09 | 3.92 | +5.09 ± 2.07 |
| 50 | sign | positive seq. | 1.013 | −5.02° | 1.56 | 5.73 | +5.09 ± 2.07 |
| 50 | sigmoid | `v′` | 0.978 | −0.59° | 0.28 | 4.45 | −0.12 ± 0.19 |
| 50 | sigmoid | positive seq. | 0.976 | −0.40° | 0.34 | 5.36 | −0.12 ± 0.19 |
| 25 | sign | `v′` | 0.977 | −0.54° | 1.15 | 3.95 | +2.16 ± 1.03 |
| 25 | sign | positive seq. | 0.999 | −2.24° | 0.89 | 5.49 | +2.16 ± 1.03 |
| 25 | sigmoid | `v′` | 0.980 | −0.69° | 0.28 | 4.45 | −0.12 ± 0.19 |
| 25 | sigmoid | positive seq. | 0.978 | −0.51° | 0.34 | 5.36 | −0.12 ± 0.19 |

**Invalid marking:** every earlier *ramp* ripple, ripple-reduction and ω′-std value
(sections 4–8, 10, 12, 13 and `phase02.md`) used a stationary phasor fit and is
**invalid for the ramp**; use this table instead.
