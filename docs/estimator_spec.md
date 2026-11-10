# Estimator & controller spec sheet (Phase 05a Step 4)

Extracted from **rendered pages** of `docs/act.pdf` (not only the text layer): pages
825, 826, 827, 828 (printed numbering; PDF pages 3–6). Every item is marked
**PRINTED** (with page/equation/figure) or **UNPRINTED** (→ `null` in
`parameter_extraction.md`). **No estimator or controller is implemented.**

Scope: the rotor position/speed estimator (Q-PLL) that Phase 05b will build, the
paper's own test scenarios and reported error numbers (Phase 05b targets), and the
controller details a later closed-loop phase would need.

---

## (a) Position / speed estimation

| Item | Status | Source / value |
| --- | --- | --- |
| Position from back EMF (direct) | **PRINTED** | Eq.(8), p.825: `θ̂_e = −arctan(Ê_α / Ê_β)` — one-argument `arctan`, a single minus sign **outside** the arctan, argument `Ê_α/Ê_β` |
| Q-PLL **structure** | **PRINTED** | Fig.1, p.825 |
| Q-PLL inputs | **PRINTED** | `Ê_α`, `Ê_β` (Fig.1) — i.e. the DSOGI-FLL output `Ê_f` (Eq.(12); Fig.7, p.828) |
| Q-PLL error `Δe` algebra | **UNPRINTED** | Fig.1 shows two multipliers (with `cos θ̂`, `sin θ̂`), a `−` combining node and a signal labelled `Δ_e`; no closed-form equation is printed |
| Q-PLL loop filter | **PRINTED (block only)** | Fig.1: a `PI` block whose output is `ω̂` |
| Q-PLL PI gains (`kp`, `ki`) | **UNPRINTED** | none printed (Fig.1 has no numbers) |
| Q-PLL normalization | **PRINTED (label)** | the text calls it a "normalized" Q-PLL; the normalization algebra is not printed |
| Q-PLL VCO / integrator | **PRINTED (block only)** | Fig.1: `1/s` integrating `ω̂` → `θ̂` |
| Speed estimate | **PRINTED (block only)** | `ω̂` is the Q-PLL PI output (Fig.1); **no speed equation** is printed |
| Electrical↔mechanical speed conversion (`ω_m = ω̂/p` or `ω̂·60/2πp`) | **UNPRINTED** | not printed anywhere |
| Angle wrapping / normalization convention (e.g. `(−π, π]`) | **UNPRINTED** | not printed |
| Phase compensation on `Ê_f` before the PLL | **UNPRINTED (none for the proposed method)** | the paper states the DSOGI-FLL keeps fundamental amplitude/phase unchanged (Eqs.(13)–(16)); the LPF phase-compensation discussion applies only to the *conventional* SMO it replaces |
| Which signal feeds the estimator | **PRINTED** | the **filtered** `Ê_f` from the DSOGI-FLL (Fig.7, p.828; Eq.(12)) |
| Startup / low-speed handling | **PRINTED (qualitative)** | Section IV-E / V-E: the method needs another strategy for start-up and low speed |

**Discrepancy to note:** Eq.(8) is printed with the minus sign *outside* the
arctangent and the argument `Ê_α/Ê_β`. With the paper's back-EMF convention
`E = ω_e ψ_f[−sin θ_e, cos θ_e]` (Eq.(2)), this yields `θ̂_e = θ_e + π/2` (a
constant quadrature offset); the paper prints Eq.(8) as a *direct* estimate but
also recommends the Q-PLL (Fig.1) "to improve the accuracy", which is what its own
block diagram (Fig.7) actually uses. **UNRESOLVED** whether Eq.(8) carries an
implicit quadrant/offset convention; flagged for Phase 05b.

## (b) The paper's own scenarios and reported error numbers (Phase 05b targets)

Scenario definitions (PRINTED):

| Scenario | Definition | Source |
| --- | --- | --- |
| Sim — no-load variable speed | 500 r/min no-load, ramp to 1000 r/min at 1.5 s, steady at 2 s | Section IV-B/C, pp.826–827; Fig.9/11/12 |
| Sim — constant speed + load step | 1000 r/min, load steps to 50 % rated at 1.5 s | Section IV-D, p.827; Fig.13 |
| Exp — no-load variable speed | reference 500 → 1000 r/min | Section V-C, p.829; Fig.17/18 |
| Exp — constant speed + load step | 1000 r/min, 20 % rated load disturbance | Section V-D, p.829; Fig.19/20 |

Reported numbers (improved SMO unless stated; PRINTED):

| Scenario | Metric | Value | Source |
| --- | --- | --- | --- |
| Sim, no-load | max speed error | **≈ 5 r/min** | Section IV-C, p.827 |
| Sim, no-load | steady position error | **≈ 0.07 rad** | Section IV-C, p.827 |
| Sim, no-load | max dynamic position error | **≈ 0.2 rad** (STASMO-CVGI 0.3 rad) | Section IV-C, p.827 |
| Sim, no-load | STASMO-CVGI position error @500/1000 r/min | 0.12 / 0.15 rad | Section IV-C, p.827 |
| Sim, 50 % load @1000 r/min | dynamic max speed error | **23 r/min** | Section IV-D, p.827 |
| Sim, 50 % load | steady mean speed error | **5 r/min** (STASMO-CVGI 8 r/min) | Section IV-D, p.827 |
| Sim, 50 % load | dynamic max position error | **0.22 rad** (STASMO-CVGI 0.3 rad) | Section IV-D, p.827 |
| Sim, 50 % load | steady position error | **0.08–0.1 rad** (STASMO-CVGI −0.15…−0.12 rad) | Section IV-D, p.827 |
| Exp, no-load @500 r/min | max speed / position error | **15 r/min / 0.1256 rad** (STASMO: 25 r/min / 0.32 rad) | Section V-C, p.829 |
| Exp, no-load @1000 r/min | max speed / position error | **10 r/min / 0.08 rad** (STASMO: 20 r/min / 0.2 rad) | Section V-C, p.829 |
| Exp, 20 % load @1000 r/min | speed error | **≈ 10 r/min** (STASMO max 20 r/min) | Section V-D, p.829 |
| Exp, 20 % load @1000 r/min | position error | **0.152 rad** (STASMO 0.184 rad) | Section V-D, p.829 |
| Table IV @1000 r/min | summary | **0.08 rad, 10 rpm**; low harmonics, no amplitude attenuation, no phase delay | Table IV, p.830 |

These are the Phase 05b acceptance targets. Baselines to reproduce: STASMO-CVGI,
SMO+LPF, STASMO+CVGI (see `configs/experiments.yaml`).

## (c) Controller details for a later closed-loop phase

| Item | Status | Source / value |
| --- | --- | --- |
| Control topology | **PRINTED** | Fig.7, p.828: cascade — speed PI → `i_q*`; `i_d* = 0`; two current PIs → `dq/αβ` → SVPWM → inverter → PMSM |
| `i_d` reference | **PRINTED** | `i_d* = 0` (Fig.7) |
| Speed loop | **PRINTED (block only)** | one `PI` on `ω_ref − ω̂`; output `i_q*` (Fig.7) |
| Current loops | **PRINTED (block only)** | two `PI` blocks on `i_d*−i_d`, `i_q*−i_q` (Fig.7) |
| Speed-loop frequency | **PRINTED** | **1 kHz** (Table III, p.828) |
| Current-loop / sampling frequency | **PRINTED** | **10 kHz** (Table III, p.828) |
| Switching frequency | **PRINTED** | **10 kHz** (Section V, p.829) |
| Inverter dead time | **PRINTED** | **2 µs** (Section V, p.829) |
| PWM | **PRINTED (block)** | `SVPWM` (Fig.7) |
| DC-bus voltage `V_DC` | **UNPRINTED (value)** | symbol shown (Fig.7); no number → `null` |
| PI gains (`kp`, `ki`) speed & current | **UNPRINTED** | none printed |
| Inverter model (switching/dead-time compensation block) | **UNPRINTED** | no model beyond "SVPWM + inverter + 2 µs dead time" |
| Current sensing | **PRINTED (block)** | 16-bit ADC (Fig.7) |
| Load profile | **PRINTED (partial)** | rated load 7.7 N·m (Table III); load steps = 50 % rated (sim) / 20 % rated (exp) (Sections IV-D, V-D). The full torque-vs-time shape is not printed |
| Mechanical model (J, friction) | **PRINTED (J only)** | `J = 0.00153 kg·m²` (Table III); friction **UNPRINTED** |
| Speed reference profile | **PRINTED** | 500 → 1000 r/min (Sections IV-C, V-C) |

## UNPRINTED summary (→ `null` in `parameter_extraction.md`)

- Q-PLL: error algebra, `kp`/`ki`, normalization, VCO/VCO gain.
- Speed equation and electrical↔mechanical conversion.
- Angle wrapping convention.
- Controller PI gains (speed and current), DC-bus voltage, PWM/dead-time model.
- Viscous/static friction.
- Simulation solver, timestep (beyond 10 kHz), injected-noise levels.

**No estimator or controller code is written from this spec until Phase 05b/06 is
approved.**

---

## Phase 06b-2 Step 0b — rendered-page re-check of the load-step quantities

Re-read from the **rendered pages** (PyMuPDF) of `docs/act.pdf`, pages 5–10
(printed 827–832). Question: do the quoted load-step numbers denote the estimation
error `(ω−ω̂)`, the reference error `(ω_ref−ω̂)`, or the actual speed deviation?

**Sim, 50 % load (Fig. 13, printed p.830).** Fig. 13 is a three-panel plot with
y-axes **"Speed(r/min)"** (top), **"Speed error(r/min)"** (middle, ticks 0/15/30) and
**"Rotor position error(rad)"** (bottom); caption verbatim *"Speed and position
estimation errors under 1000 r/min with 50 % rated load disturbance."* Section IV-D
text: *"the maximum error of the speed in the dynamic process is 23 r/min. After
entering the steady state … the average error of the improved SMO becomes 5 r/min."*
⟹ the **23 rpm / 5 rpm are speed *estimation* errors** `|ω−ω̂|` (the encoder gives the
actual speed; the actual speed is the separate top "Speed" panel), **not** an actual
speed deviation.

**Exp, 20 % load (Fig. 19/20, printed p.832).** Fig. 19 y-axis **"Rotor
Speed(120 rpm/div)"** with in-plot traces **`N_ref`, `N_actual`, `N_err(10 rpm/div)`**;
the *actual* speed dip during the step is annotated **ΔN = 45 / 48 r/min**
(STASMO-CVGI / improved). Section V-D text: *"During the load process, the estimation
error of the improved SMO remains around 10 r/min."* ⟹ the quoted **~10 rpm is the
speed *estimation* error** (`N_err`), while **ΔN ≈ 45 r/min is the actual dip** — the
figure shows both.

**Load amplitudes (confirmed printed).** Sim = **50 % of rated** at 1.5 s, 1000 rpm
(Section IV-D text + Fig. 13 caption); exp = **20 % of rated**, 1000 rpm
(Fig. 19/20 captions + Section V-D).

**Controller / start-up printed items.** Table III (printed p.828) lists **speed-loop
control frequency 1 kHz**, sampling and current-loop 10 kHz — **no DC-bus row**;
Fig. 7 prints **no PI gains or bandwidth**; **no minimum speed** is printed, and
Section V-E states only *"other control methods need to be used for start-up and
low-speed range"* (qualitative). Table IV's `N_err` (10 r/min for FSMO+DSOGI) is likewise
an estimation error.

**If a *dynamic* figure were an actual dip** (it is not — Fig. 13's is an estimation
error), `dip ∝ 1/α_ω` would apply. For reference, our **encoder** baseline gives a
**37 rpm** dip at 50 % load for `α_ω = 50 Hz` (14.9 rpm at 20 % load). The exp Fig. 19
**actual** dip ΔN ≈ 45 rpm at **20 %** load is ~3× ours, implying a hardware
speed-loop bandwidth ≈ 17 Hz — indicative only (hardware includes dead time/PWM).

---

## Phase 06b-3 — operating-point decoupling and low-speed margin relaxation

**Decision (D-059 in `implementation_decisions.md`; numbered D-059 because Phase 06b-2
already used D-056–D-058).** Justification for operating-point decoupling and the relaxed
low-speed phase margin:

- The Phase 06b-2 measurement shows `T_est(jω)` is **speed-dependent**: the DSOGI-FLL
  envelope corner is ≈ 41 Hz at 500 rpm vs ≈ 83 Hz at 1000 rpm (chain B, f_n = 20 Hz),
  so the estimator phase lag at a given frequency is much larger at low speed. A single
  global `α_ω` therefore cannot meet one margin target at both speeds.
- **Low-speed relaxation (`PM ≥ 10°` for ≤ 500 rpm):** at 500 rpm even `α_ω = 5 Hz`
  gives PM 10.8° (D-058); the loop is *stable* but lightly damped. This is accepted for
  the low-speed operating point because (i) the plant is still stable (PM > 0, GM 33 dB),
  (ii) the paper gives no low-speed requirement, and (iii) the paper itself prints that
  start-up/low speed need another strategy. It is a stated, labelled relaxation.
- **Operating-point decoupling:** the sensorless speed loop uses `α_ω = 5 Hz` at 500 rpm
  and `α_ω = 10 Hz` at 1000 rpm (per-scenario, from `configs/drive_assumptions.yaml`
  `speed_loop.alpha_w_hz_at_{500,1000}rpm`). The encoder-only Phase 06a value (50 Hz) is
  unchanged for the encoder baseline.
- **Verification caveat (Step 1):** against the measured `L = L_enc × T_est`, the
  500 rpm point meets the relaxed target (PM 10.8° ≥ 10°, GM 33 dB), but the **1000 rpm
  point at 10 Hz gives PM 14.6° < 45°** (GM 56.7 dB) — stable but below the 45° design
  target; the 45°-meeting bandwidth at 1000 rpm is 5 Hz (PM 54.1°). Step 2's time-domain
  gates are the arbiter; the margin model is ~one grid step conservative vs the 06b
  time-domain boundary (stable ≤ 15 Hz).

**Status:** decoupling and the low-speed relaxation are ASSUMPTION — not from paper.

---

## Phase 06b-4 — softening the 500 rpm bandwidth to 2.5 Hz (D-061)

**Justification (D-061).** Phase 06b-3 passed all 1000 rpm gates but failed Gate 2b at
500 rpm at `α_ω = 5 Hz` (PM 10.8°): a 20 % load step excited the lightly damped loop past
synchronism. Softening the low-speed bandwidth increases the phase margin and damping:
against the 06b-2 measured `L = L_enc × T_est` (chain B, `f_n = 20 Hz`), `α_ω = 2.5 Hz`
at 500 rpm gives **PM 65.5°, GM 165 dB, crossover 6.6 Hz** (target PM ≥ 30°), up from
10.8° at 5 Hz. The 1000 rpm bandwidth is retained at 10 Hz (PM 14.6°). Frozen before the
time-domain run (`configs/drive_assumptions.yaml`
`speed_loop.alpha_w_hz_at_500rpm = 2.5`).

**Result (Step 2 gate).** The softened loop passes the **steady** 500 rpm gate (no-load:
peak 0.3°, steady 0.3°) and all 1000 rpm regression gates (2a 0.6°, 2b 7.5°, 2c 21.7°),
but the 500 rpm **load transient** still fails: the **unclamped DSOGI-FLL loses lock**
(guards `ω′ < 0`) 36 ms after a 20 % step (peak `|θ̂−θ|` 35.9°, steady 7.3°), and a
start-up-under-load trips it at 60 ms. That is an **estimator-amplitude** limitation at
low speed (|E| = 27.8 V at 500 rpm vs 55.5 V at 1000 rpm), **not** the speed-loop
bandwidth. `min_omega` clamping (a non-default FLL option) would be the next lever.

**Status:** softened bandwidth is ASSUMPTION — not from paper; Gate 2 still not met at
500 rpm (load transients).
