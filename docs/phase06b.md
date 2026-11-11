# Phase 06b — Sensorless closed loop: one narrative (06b → 06b-6)

The estimator chain (**SMO `E_hat` → DSOGI-FLL `v'` → Q-PLL**) replaces the Phase 06a
encoder source. What follows is what was tried, what failed, and what was kept, with the
decision IDs. Chain composition lives only in `experiments/chain.py`; no `src` change
beyond the 06a `angle_advance` flag.

## What was built

- `experiments/chain.py`: `EstimatorSource` (chain behind the 06a `AngleSpeedSource`),
  `HandoverSource` (encoder start-up → chain-internal lock → sensorless), `build_fll`.
- Config (`configs/drive_assumptions.yaml`): handover (`ω_h`, tol, `t_lock`, initial
  `ω'`), `angle_advance`, decoupled `α_ω` mapping, FLL `min_omega` floor.

## What was tried and what happened

| Phase | Tried | Result | Decision |
| --- | --- | --- | --- |
| 06b | Chain in the loop at the encoder design `α_ω = 50 Hz` | **Gate 1 (voltage alignment) PASS** (Δgain −0.0002, Δphase +0.013° vs 04c); **Gate 2 FAIL** — the *estimated speed* in the speed loop destabilises it ~48 ms after handover | D-052, D-054 |
| 06b-2 | Measure `T_est(jω)` and derive margins | H1 (DSOGI envelope caps `T_est`, speed-dependent) **supported**; H2 (raw-`E_hat` chain wider) **refuted**. No global `α_ω` met PM≥45° / GM≥6 dB at both speeds → `α_ω* = None` | D-056–D-058 |
| 06b-3 | Operating-point decoupling (5 Hz @500 / 10 Hz @1000) + low-speed PM relaxation (≥10°) | 500 rpm @5 Hz PM 10.8°. Gate 2a all pass; 2b@1000, 2c pass; **2b@500 fails** (lightly damped) | D-059, D-060 |
| 06b-4 | Soften 500 rpm to 2.5 Hz | PM 65.5°. 500 rpm steady + all 1000 rpm gates pass; **2b@500 still fails** — the *unclamped FLL* loses lock on the step | D-061 |
| 06b-5 | FLL `min_omega` floor (100 rpm) | Guard trip gone; **start-up-under-load fixed** (2a passes with 20 % load from rest); 1000 rpm 2b/2c pass; **500 rpm loaded step still fails** (floor sweep: none restores tracking) | D-062 |
| 06b-6 | Re-scope to the paper's actual scenarios | **Gate P PASS**; paper suite passes; 500 rpm loaded + 1000 rpm rated steps fail (documented) | D-063, D-064 |

**Key result:** the speed loop was never the whole story — after 06b-4 the 1000 rpm loop is
well-damped, and the remaining low-speed limitation is the **FLL's amplitude-driven gain**
at 500 rpm (|E| = 27.8 V), which a bandwidth change or a frequency floor cannot fix.

## What was kept (final configuration)

`α_c = 500 Hz`; `α_ω = 2.5 Hz @500 / 10 Hz @1000` (decoupled); Q-PLL `f_n = 20 Hz`
(ζ 0.707); FLL `min_omega` floor 100 rpm; handover `ω_h = 400 rpm`, `tol = 2 rpm`,
`t_lock = 50 ms`, initial `ω' = p·ω_h`; decoupling ON, back-emf comp OFF; `V_dc = 311 V`;
1-sample + ½ ZOH delay; angle advance OFF by default. Full register:
[`closed_loop_assumptions.md`](closed_loop_assumptions.md).

## Phase 06b-6 — Gate P and the paper-scenario suite

**Step 0.** 06b-5 committed `5e9e9bc`, tagged `phase-06b5-gate2-fail`. Repo
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`, branch `master`, **no
remotes**. Non-paper elements registered in `closed_loop_assumptions.md`. The 06b-5 claim
"all paper scenarios pass" is **corrected**: those were gate sub-cases, not the suite (now
run). Exact printed scenarios listed in `estimator_spec.md` §06b-6 Step 0d (S1–S5; all
quoted numbers are **estimation errors**).

### Gate P (chain B, matched cell, clamp ON, advance off; on reported)

| scenario | handover | peak `|θ̂−θ|` | steady `|θ̂−θ|` max | sat. after | advance on | FLL | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 no-load 500→1000 ramp | yes | 2.2° | 0.61° | 0 | 2.2° | ok | **PASS** |
| S2 sim 50 % step @1000 | yes | 21.7° | 0.56° | 0 | 21.5° | ok | **PASS** |
| S3 exp 20 % step @1000 | yes | 7.5° | 0.59° | 0 | 7.4° | ok | **PASS** |
| S4 steady 1000 rpm | yes | 0.7° | 0.61° | 0 | 0.6° | ok | **PASS** |

**Gate P = PASS** (advance off and on). **Clamp ablation:** clamp-OFF is identical on all
four (the floor never engages) — so the clamp is not needed for the paper scenarios, only
for start-up-under-load.

S1 bandwidth: the row above uses `α_ω = 2.5 Hz` — a **scenario-specific bandwidth chosen
after a failure**. At the nominal `α_ω* = 10 Hz` S1 gives peak **180° (advance OFF) / 18.5°
(advance ON)**: advance-off fails, because a fixed bandwidth for a 500→1000 ramp must be
safe at the low-speed end (10 Hz is marginal at 500 rpm, PM 14.6°). *(Phase 06d, Step 0c.)*

### Scenario suite (`experiments/06b6_scenarios.py`)

Estimation errors are pointwise over the stated windows; **matching basis = estimation
error** (Step 0d).

| scenario | cell | steady `|θ̂−θ|` mean/max | steady `|ω̂−ω|` max | dynamic `|θ̂−θ|` max | dynamic `|ω̂−ω|` max | true dip | paper (est. err) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 ramp | matched | 0.61° | 0.15 rpm | 2.24° | 2.2 rpm | −12.4 rpm | ≈5 rpm, 0.07 rad (4°), 0.2 rad dyn |
| S2 sim 50 % | matched | 0.56° | 0.14 rpm | 21.7° | 111.7 rpm | −210 rpm | 23 rpm / 0.22 rad (12.6°) dyn; 5 rpm steady |
| S2 sim 50 % | FLC nominal | 1.92° | 4.73 rpm | 21.3° | 116.2 rpm | −212 rpm | (as above) |
| S3 exp 20 % | matched | 0.59° | 0.14 rpm | 7.5° | 40.4 rpm | −79.6 rpm | ~10 rpm; 0.152 rad (8.7°) |
| S3 exp 20 % | FLC nominal | 1.96° | 4.86 rpm | 6.8° | 42.6 rpm | −80.6 rpm | (as above) |
| S4 steady | matched | 0.61° | 0.14 rpm | 0.61° | 0.1 rpm | — | 0.08 rad, 10 rpm (Table IV) |

**Sensorless cost** (two bases, corrected in Phase 06d Step 0d):
- *equal `α_ω` (10 Hz):* encoder baseline dips −140.3 / −56.1 rpm (S2/S3) vs sensorless
  −210.0 / −79.6 → cost **−69.6 / −23.4 rpm** (the estimated-speed feedback deepens the
  dip; recovery time is nearly the same).
- *each config's own design `α_ω`:* encoder at 50 Hz −37.2 / −14.9 rpm (06a) vs sensorless
  at 10 Hz −210 / −79.6 → cost −172.8 / −64.7 rpm.

So the earlier "≈0" was wrong: the sensorless loop pays a real dip penalty at equal
bandwidth, and a much larger one at its stable bandwidth.

**No reproduction claim.** Our *steady* estimation errors are far below the paper's
(0.6° vs 4°; 0.14 vs 5 rpm), but our *dynamic* transient estimation errors during the load
steps are **larger** than the paper's (111.7 vs 23 rpm; 40.4 vs 10 rpm) because our speed
loop is slow (10 Hz → a large true-speed dip). The setups differ on every axis: encoder
start-up + handover (paper: sensorless start-up), ideal voltage (no inverter/dead time),
assumed PLL/FLL gains, our slow speed loop.

### Stress table (informational — **beyond paper**, not gates)

| case | peak `|θ̂−θ|` | FLL | sat. | failure mode | verdict |
| --- | --- | --- | --- | --- | --- |
| 500 rpm 20 % step | 180° | ok | 1.00 | synchronism loss | FAIL |
| 500 rpm 50 % step | 180° | ok | 1.00 | synchronism loss | FAIL |
| 1000 rpm **rated (100 %)** step | 180° | ok | 0.00 | synchronism loss | FAIL |
| 500 rpm start-up under 20 % load | 0.4° | ok | 0.00 | none | PASS |

The 1000 rpm **50 %** step (paper) passes; **100 %** does not — so the load-step limit is
between 50 % and 100 % at 1000 rpm. The 500 rpm loaded limit is documented from 06b-5
(D-062): floor sweep none→guard trip, 100/200/400 rpm→divergence, 300 rpm→pinned 36°.

### Sensitivity

At the 50 % step (S2): `α_ω × {0.5, 1, 2} × advance {off, on}` and `f_n × {0.5, 2}`.

| cell | peak `|θ̂−θ|` | dip | verdict |
| --- | --- | --- | --- |
| α_ω 0.5× (5 Hz), adv off / on | 25.7° / 25.4° | −309.6 / −309.0 | PASS |
| α_ω 1× (10 Hz), adv off / on | 21.7° / 21.4° | −210.0 / −207.6 | PASS |
| α_ω 2× (20 Hz), adv off / on | **180° / 180°** | −4568 / −4892 | **FAIL** (unstable) |
| f_n 0.5× (10 Hz), adv off | **invalid** | — | handover at 0.70 s is *after* the 0.5 s step → the step was encoder-driven, not a sensorless transient |
| f_n 2× (40 Hz), adv off | 17.0° | −172.2 | PASS |

*Correction (Phase 06d Step 0e):* the f_n×0.5 row's 0.6° is the **steady** error — the slow
PLL delays the lock criterion, so handover (0.70 s) falls after the load step (0.50 s) and
the transient was encoder-driven. f_n = 5 Hz never hands over at all. Only f_n ≥ 20 Hz
(handover ≈ 0.20 s) measures a sensorless transient.

`α_ω` 2× is unstable (confirms the measured-margin picture); the f_n×0.5 "improvement" is
the handover artefact above, not a real one.

## Limitations

- No dead time / PWM / current ripple; no conventional-SMO baseline (06c).
- Low speed under **load** is not met: 500 rpm loaded steps and the 1000 rpm rated step
  lose synchronism (D-064). Root cause: FLL amplitude-driven gain at low |E|.
- No speed scheduling of `α_ω` (would need a `src` change); a fixed bandwidth is used per
  scenario (S1 at 2.5 Hz).
- No reproduction claim; setup differences listed above.