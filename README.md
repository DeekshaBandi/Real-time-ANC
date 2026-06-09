# Active Noise Cancellation with FxLMS

A simulation-first implementation of the **Filtered-x Least Mean Squares (FxLMS)**
adaptive filtering algorithm for active noise cancellation (ANC) of periodic noise
(engine hum / fan noise). Written in MATLAB and kept **GNU Octave-compatible**
(no paid-toolbox-only functions).

The project is built single-channel first, but structured so the **MIMO** extension
drops in cleanly.

## Why this project

This is an end-to-end ANC pipeline that exercises the core skills of an automotive
NVH / DSP role:

| Skill | Where it lives |
|---|---|
| Adaptive filtering (FxLMS) | `src/fxlms.m` |
| Acoustic transfer-function measurement | `src/secondary_path_id.m` |
| FIR filter design | `src/fir_lowpass.m` |
| DSP math → code | `docs/derivation.md` + all sources |
| Validation under varying conditions | `experiments/` |
| MIMO systems | `src/fxlms_mimo.m` (planned extension) |
| Feasibility analysis | `docs/feasibility_report.md` (planned) |

## Pipeline

```
reference noise x(n)
      │
      ├──► Primary path P(z) ──────────────► d(n)  (noise at error mic)
      │                                        │
      └──► Control filter W(z) ──► Secondary ──┤
               (adaptive)          path S(z)   ▼
                   ▲                          e(n) = d(n) − y'(n)   (residual)
                   │                            │
                   └──── FxLMS update ◄─────────┘
                     w += μ · e(n) · x'(n)
                     x'(n) = Ŝ(z) * x(n)   (filtered reference)
```

## Layout

```
src/          core algorithm + signal/system models
experiments/  runnable end-to-end demos and parameter studies
results/      saved plots (.png)
docs/         math derivation and feasibility report
```

## Running (Octave)

```sh
cd anc-fxlms
octave --no-gui experiments/run_basic.m
```

## Build phases

- [x] Phase 0 — repo setup
- [x] Phase 1 — signal & system models
- [x] Phase 2 — FIR filter design
- [x] Phase 3 — secondary-path identification
- [x] Phase 4 — core FxLMS  (**43.5 dB** steady-state reduction)
- [x] Phase 5 — validation experiments
- [x] Phase 6 — FxLMS vs LMS + frequency analysis
- [x] Phase 7 — MIMO extension  (2×2, **~44 dB** both mics)
- [x] Phase 8 — feasibility report (`docs/feasibility_report.md`, `docs/derivation.md`)
- [x] Phase 9 — C port of the inner loop (`c/fxlms_rt.c`, matches MATLAB to 1e-8)

## Headline results

| Experiment | Result |
|---|---|
| Single-channel FxLMS (periodic noise) | **~43 dB** reduction |
| Secondary-path identification | 0.01% coef error, −59 dB MSE |
| Stable step-size range | μ ≈ 0.005–0.07 (diverges ≥ 0.1) |
| Robustness to delay error | holds until ±90° phase at top harmonic |
| FxLMS vs plain LMS (same μ) | FxLMS +43 dB; LMS **diverges** |
| 2×2 MIMO FxLMS (cross-coupled) | **~44 dB** at both error mics |

Run any experiment with `octave --no-gui experiments/<name>.m`; plots land in `results/`.

### C port (real-time inner loop)

```sh
octave --no-gui experiments/export_cref.m   # export test vectors + MATLAB ref
make -C c                                    # build
./c/fxlms_rt c/data                          # run; matches MATLAB to ~1e-8
```

## References

- S. M. Kuo & D. R. Morgan, *Active Noise Control Systems* — canonical FxLMS text.
- B. Widrow & S. D. Stearns, *Adaptive Signal Processing* — LMS foundations.
- S. Haykin, *Adaptive Filter Theory* — convergence, step-size bounds, misadjustment.
