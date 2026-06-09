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
- [ ] Phase 1 — signal & system models
- [ ] Phase 2 — FIR filter design
- [ ] Phase 3 — secondary-path identification
- [ ] Phase 4 — core FxLMS
- [ ] Phase 5 — validation experiments
- [ ] Phase 6 — FxLMS vs LMS + frequency analysis
- [ ] Phase 7 — MIMO extension *(future)*
- [ ] Phase 8 — feasibility report *(future)*
- [ ] Phase 9 — C port of the inner loop *(future)*

## References

- S. M. Kuo & D. R. Morgan, *Active Noise Control Systems* — canonical FxLMS text.
- B. Widrow & S. D. Stearns, *Adaptive Signal Processing* — LMS foundations.
- S. Haykin, *Adaptive Filter Theory* — convergence, step-size bounds, misadjustment.
