# Innovation gate (#84)

Reusable Mahalanobis innovation gate only. No sensor residual, Kalman gain,
Joseph form, state or covariance write, ZUPT/DVL/pressure model, automatic chi2
choice, ICON, safety authority, or real hardware. Contract: parent WP #30, issue
#84. The #81 `EskfCovariance` and `kCovDim = 15` (`eskf_state.*`) are reused
unchanged; #82 and #83 are not touched. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The gate is opt-in:
nothing calls it.

Files: `innovation_gate.{h,cc}` (C++), `innovation_gate.py` (Python mirror),
`innovation_gate_test.{cc,py}` (paired tests).

## API

```
GateInnovation(nu, H, P, R, chi2_threshold) -> GateResult         // C++
gate_innovation(nu, h, p, r, chi2_threshold) -> GateResult        # Python
```

| Argument | Shape | Notes |
| --- | --- | --- |
| `nu` | `m`, `m >= 1` | Innovation. `m` is the degrees of freedom. |
| `H` | row-major `m x 15` | Jacobian w.r.t. the 15-element error state. |
| `P` | `EskfCovariance` (15x15) | Prior covariance (#81). Need not be symmetric. |
| `R` | row-major `m x m` | Measurement noise. Symmetric within `1e-12`. |
| `chi2_threshold` | scalar | Finite and `> 0`. Always caller supplied. |

C++ takes `std::vector<double>` for `nu`, `H`, `R`. Python takes sequences of
floats. Inputs are never modified.

## Result

`GateResult` has four fields.

| Field | Meaning |
| --- | --- |
| `ok` | True for `OK_ACCEPT` and `OK_REJECT`. Always equal to `evaluated`. |
| `evaluated` | True when the gate ran to a decision. |
| `accepted` | True only for `OK_ACCEPT`. |
| `status` | Typed status, below. |
| `diagnostics` | Set only when `evaluated`. `nullopt` / `None` on every error. |

`ok` means the gate evaluated, not that the measurement was accepted. Use
`accepted` for the decision, or switch on `status`.

`GateDiagnostics`: `mahalanobis_sq` (`d^2`), `threshold` (echo),
`innovation_norm` (`||nu||_2`), `dof` (`m`), and `S` (the symmetrized `S_s`,
row-major `m x m`).

| Status (C++ / Python) | Meaning |
| --- | --- |
| `kOkAccept` / `OK_ACCEPT` | `d^2 <= chi2_threshold`. |
| `kOkReject` / `OK_REJECT` | Evaluated, `d^2 > chi2_threshold` (outlier). |
| `kInvalidDim` / `INVALID_DIM` | `m < 1`, `H` size is not `m * 15` (this covers any column count other than 15), or `R` size is not `m * m`. |
| `kInvalidP` / `INVALID_P` | `P` is unknown (no storage) or has a non-finite entry. |
| `kInvalidR` / `INVALID_R` | `R` has a non-finite entry, or `|R_ij - R_ji| > 1e-12` for some `i, j`. |
| `kInvalidNu` / `INVALID_NU` | `nu` has a non-finite entry. |
| `kInvalidThreshold` / `INVALID_THRESHOLD` | Threshold is non-finite or `<= 0`. |
| `kSingularS` / `SINGULAR_S` | Cholesky of `S_s` failed: a pivot is `<= 1e-12`. |
| `kNonFinite` / `NON_FINITE` | `H` has a non-finite entry, or `S`, a Cholesky pivot, `d^2`, or `||nu||` is non-finite, or `d^2 < -1e-9`. |

Checks run in this order and the first defect wins: dimensions, `P`, `R`, `nu`,
threshold, `H` finite, then the numerics below. No error status carries a
partial accept or reject.

## Numerics (locked)

1. `P_s = 0.5 (P + P^T)`.
2. `S = H P_s H^T + R`, then `S_s = 0.5 (S + S^T)`. Computed as `(H P_s) H^T`.
3. Lower Cholesky `S_s = L L^T`. The pivot is the value under the square root,
   `S_s[j,j] - sum_k L[j,k]^2`. If any pivot is `<= 1e-12`, return
   `SINGULAR_S`. A non-finite pivot returns `NON_FINITE`.
4. Solve `S_s y = nu` by forward then back substitution, and `d^2 = nu^T y`.
   `d^2` must be finite and `>= -1e-9`. A `d^2` in `[-1e-9, 0)` is stored as
   `0.0` (clamped). A `d^2 < -1e-9` returns `NON_FINITE`.
5. Accept iff `d^2 <= chi2_threshold`. The comparison uses the stored
   (clamped) `d^2`, so `d^2 == threshold` accepts.

`S_s` is never explicitly inverted. The C++ and Python loops use the same
summation order, so results match bit for bit on the shared fixture in practice
and are pinned at `1e-9`.

## Choosing a threshold (examples only)

The code has no chi2 table. The caller passes the threshold for its own `m` and
confidence level. Common 95% values, for reference only and not used anywhere:

| `m` | chi2 (95%) |
| --- | --- |
| 1 | 3.841 |
| 2 | 5.991 |
| 3 | 7.815 |
| 6 | 12.592 |

## Fixtures

| Case | Expectation |
| --- | --- |
| Exact boundary: `S = diag(4, 1)`, `nu = (2, 1)`, threshold `2.0` | `d^2 == 2.0` exactly, `OK_ACCEPT`. |
| Just over: same inputs, threshold `2 - 1e-9`; and `nu` scaled by `1 + 1e-6` | `OK_REJECT`. |
| Deep accept and deep reject | `OK_ACCEPT` with `d^2 < 1e-3`; `OK_REJECT` with `d^2 > 1e3`. |
| Singular: `H = 0`, `R = 0` (`m = 1, 2`); indefinite `R`; pivot floor at `1e-12` vs `2e-12` | `SINGULAR_S`, except `2e-12` which evaluates. |
| Non-finite `nu`, `P`, `R`, threshold; unknown `P`; non-finite `H` | `INVALID_NU`, `INVALID_P`, `INVALID_R`, `INVALID_THRESHOLD`, `INVALID_P`, `NON_FINITE`. |
| Overflow (`1e308` entries, `nu = 1e200`) | `NON_FINITE`. |
| Dim mismatch: empty `nu`, `H` short/long/14 cols/16 cols, `R` short/long/1x1 | `INVALID_DIM`. |
| Asymmetric `R`: `1e-11` and `+/-0.25` off; `5e-13` off | `INVALID_R`; the `5e-13` case evaluates. |
| Precedence | dim, `P`, `R`, `nu`, threshold, in that order. |
| Diagnostics | `d^2`, threshold, `||nu||`, `dof`, `S` present on accept and reject, `nullopt` / `None` on errors. |
| Asymmetric `P` | Same `d^2` and `S` as its symmetrization. |
| No writes | `nu`, `H`, `R`, `P` are unchanged after the call. |
| Determinism and golden | Identical inputs give identical outputs. One shared `m = 3` fixture (asymmetric `P`, non-diagonal `R`) pins `d^2 = 0.3613017414836702` and four `S` entries in C++ and Python at `1e-9`. |

The golden value was computed independently with numpy
(`nu @ solve(S_s, nu)`), not with this code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.innovation_gate_test
g++ -std=c++17 -I. intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/innovation_gate_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/innovation_gate_test \
  && /tmp/innovation_gate_test
```

If gtest is not installed system-wide, build it from a googletest checkout
(for example `git clone --branch v1.14.0 https://github.com/google/googletest`)
and compile `googletest/src/gtest-all.cc` and `googletest/src/gtest_main.cc`
alongside, with `-I googletest/include -I googletest`.

Bazel targets: `//intrinsic/estimation:innovation_gate`,
`:innovation_gate_test`, `:innovation_gate_py`, `:innovation_gate_test_py`.
