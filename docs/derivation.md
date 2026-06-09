# FxLMS: derivation from first principles

This note derives the Filtered-x LMS update from steepest descent, shows why
the "filtered-x" step is necessary, states the stability bound, and gives the
normalized and multichannel forms used in the code.

## 1. Notation

- `x(n)` — reference signal (correlated with the noise to be cancelled).
- `P(z)` — primary path: noise source → error mic. Disturbance `d(n) = P(z) x(n)`.
- `S(z)` — secondary path: control speaker → error mic. Length `M`.
- `w(n) = [w_0 … w_{L-1}]ᵀ` — adaptive FIR control filter.
- `x(n) = [x(n) … x(n-L+1)]ᵀ` — reference regressor.
- Control output `y(n) = wᵀ x(n)`.
- Residual at the error mic: `e(n) = d(n) − S(z) y(n)`.

## 2. Standard LMS recap (no secondary path)

If the control output reached the error mic directly (`S(z)=1`), the cost is the
instantaneous squared error `J(n) = e(n)²` with `e(n) = d(n) − wᵀx(n)`. Steepest
descent moves opposite the gradient:

```
∂J/∂w = 2 e(n) · ∂e/∂w = −2 e(n) x(n)
w(n+1) = w(n) + μ e(n) x(n)              (LMS)
```

## 3. Why standard LMS fails for ANC

With a real secondary path the residual is

```
e(n) = d(n) − S(z)[ wᵀ x(n) ].
```

Now the gradient picks up the secondary path:

```
∂e(n)/∂w = − S(z) x(n)  ≡  − x'(n),
```

i.e. the *reference must be filtered by S(z)* before it forms the gradient.
Using the raw `x(n)` (standard LMS) applies a gradient that is wrong in phase;
once `S(z)` contributes more than ~90° of phase the sign of the correction
flips and the loop **diverges**. (Confirmed empirically in Phase 6:
`experiments/compare_lms_fxlms.m`.)

## 4. The filtered-x update

We do not know `S(z)` exactly, so we use an offline estimate `Ŝ(z)`
(Phase 3 / `secondary_path_id.m`). Define the **filtered reference**

```
x'(n) = Ŝ(z) x(n),    x'(n) = [x'(n) … x'(n-L+1)]ᵀ.
```

Substituting into steepest descent gives the **FxLMS** update:

```
w(n+1) = w(n) + μ e(n) x'(n)            (FxLMS)
```

That single change — filter the reference through `Ŝ(z)` — is the whole
algorithm.

## 5. Stability / step-size bound

For LMS-type updates with input correlation matrix `R = E[x' x'ᵀ]`, convergence
in the mean requires

```
0 < μ < 2 / λ_max(R)   ≲   2 / ( L · E[x'(n)²] ).
```

Two practical consequences, both seen in Phase 5:

1. **Delay shrinks the bound.** The secondary path inserts a delay Δ into the
   feedback loop; the effective limit drops roughly like `μ_max ∝ 1/(L+Δ)`.
   This is why the stable range collapses well below the nominal value.
2. **Phase margin.** Cancellation holds only while the phase error between
   `S` and `Ŝ` stays within ±90° *at every frequency present*. For a
   delay mismatch the phase error grows with frequency, so the **highest tone**
   hits the limit first (Phase 5b shows the collapse at the 3rd harmonic).

## 6. Normalized FxLMS (FxNLMS) — used in the code

To make the step size scale-independent, normalize by the filtered-reference
energy:

```
w(n+1) = w(n) + ( μ / (ε + ‖x'(n)‖²) ) · e(n) x'(n),   0 < μ < 2.
```

This is what `src/fxlms.m` implements; it removes the dependence on signal/path
gain and is why the Phase 5 sweep is interpretable on a fixed 0–2 scale.
Optional **leakage** `(1 − μγ)w` bounds the coefficients and adds robustness.

## 7. Multichannel (MIMO) extension

For `J` error mics and `K` actuators (one reference), each actuator `k` reaches
mic `j` through `S_{jk}(z)`. The filtered reference becomes a `J×K` set
`x'_{jk}(n) = Ŝ_{jk}(z) x(n)`, and the filter for actuator `k` accumulates the
error over **all** mics:

```
w_k(n+1) = w_k(n) + ( μ / norm_k ) · Σ_j e_j(n) x'_{jk}(n).
```

This is Elliott's multichannel FxLMS, implemented in `src/fxlms_mimo.m`. It
reduces exactly to the single-channel update for `J=K=1`.

## References

- S. M. Kuo and D. R. Morgan, *Active Noise Control Systems*, Wiley, 1996.
- S. J. Elliott, *Signal Processing for Active Control*, Academic Press, 2001.
- B. Widrow and S. D. Stearns, *Adaptive Signal Processing*, Prentice Hall, 1985.
- S. Haykin, *Adaptive Filter Theory*, Prentice Hall.
