# ESKF covariance propagation (#83)

Covariance transition and process noise only. No measurement update, gain,
Joseph form, van Loan or expm, Earth-rate or transport-rate, lever arm, water
current, ICON, safety authority, or real hardware. Contract: parent WP #30,
issue #83. The #81 layouts (`eskf_state.*`) and the #82 `ImuSample` and
rotation helpers (`eskf_propagate.*`) are reused unchanged. `PropagateNominal`
is not modified: the caller still runs it separately. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged.

Files: `eskf_cov_propagate.{h,cc}` (C++), `eskf_cov_propagate.py` (Python
mirror), `eskf_cov_propagate_test.{cc,py}` (paired tests).

## API

```
PropagateCovariance(x, imu, dt_s, P, noise) -> CovPropagateResult   // C++
propagate_covariance(x, imu, dt_s, P, noise) -> CovPropagateResult  # Python
```

`x` is the **pre-update** nominal, at the same instant as the #82 step inputs.
`P` is a 15x15 `EskfCovariance`. `ProcessNoiseConfig` holds four sqrt-PSD
scalars, each finite and `>= 0`, applied isotropically per triad:

| Field | Unit |
| --- | --- |
| `sigma_accel` | (m/s^2)/sqrt(Hz) |
| `sigma_gyro` | (rad/s)/sqrt(Hz) |
| `sigma_accel_bias_rw` | (m/s^2)/sqrt(s) |
| `sigma_gyro_bias_rw` | (rad/s)/sqrt(s) |

`CovPropagateResult` has `ok`, a typed `status`, and `P`. `P` is set only when
`ok`. A rejected call returns no matrix and never changes its input.

| Status (C++ / Python) | Meaning |
| --- | --- |
| `kOk` / `OK` | Propagated. |
| `kInvalidDt` / `INVALID_DT` | `dt_s` is not finite, or not in `0 < dt_s <= 1.0`. |
| `kInvalidImu` / `INVALID_IMU` | Any IMU component is non-finite. |
| `kInvalidQuaternion` / `INVALID_QUATERNION` | `||q||` is non-finite or `< 1e-12`. |
| `kInvalidP` / `INVALID_P` | `P` is unknown (no storage) or has a non-finite entry. |
| `kInvalidNoise` / `INVALID_NOISE` | Any sigma is non-finite or negative. |
| `kNonFinite` / `NON_FINITE` | Any input nominal component is non-finite (as in #82), or any output entry is. |

Checks run in this order and the first defect wins: dt, IMU, nominal finite,
quaternion norm, `P`, noise, then output finiteness.

## Error state and conventions

`dx = [dp(3), dtheta(3), dv(3), dba(3), dbg(3)]`, indices 0-14, as in #81.
`dtheta` is the local (right) attitude error, `R_true ~ R(q) Exp(dtheta)`.

Let `omega = gyro - b_g`, `R = R_body_to_world(q)` (the #82 helper, Hamilton
wxyz, body to world), and `g_b = R^T g_enu` with `g_enu = (0, 0, -9.80665)`.
`[u x]` is the skew-symmetric matrix of `u`. The accelerometer reading and
`b_a` do not enter `F`.

## Transition (locked)

3x3 blocks of the continuous `F`. Omitted blocks are zero.

| row \ col | dp | dtheta | dv | dba | dbg |
| --- | --- | --- | --- | --- | --- |
| dp | 0 | `-R [v x]` | `R` | 0 | 0 |
| dtheta | 0 | `-[omega x]` | 0 | 0 | `-I` |
| dv | 0 | `[g_b x]` | `-[omega x]` | `-I` | `-[v x]` |
| dba | 0 | 0 | 0 | 0 | 0 |
| dbg | 0 | 0 | 0 | 0 | 0 |

`Phi = I_15 + F * dt_s`. First order only.

## Process noise (locked)

`n = [n_a, n_g, n_ba, n_bg]`, `Qc = diag(sa^2 I, sg^2 I, sba^2 I, sbg^2 I)`.
`G` (15x12) has identity blocks mapping `n_a -> dv`, `n_g -> dtheta`,
`n_ba -> dba`, `n_bg -> dbg`. `Qd = G Qc G^T * dt_s` (Euler). `G Qc G^T` is
diagonal, so `Qd` is diagonal with `sigma^2 * dt_s` on the matching triad.

## Update (locked)

1. `Phi` and `Qd` from the pre-update `x`, `imu`, `dt_s`, `noise`.
2. `P_tmp = Phi P Phi^T + Qd`.
3. `P_out = 0.5 * (P_tmp + P_tmp^T)`.

Eigenvalues are not clamped. There is no Joseph form and no square root filter.
The C++ and Python loops use the same summation order.

## Test helpers

`BuildPhi(x, imu, dt_s)` / `build_phi` and `BuildQd(noise, dt_s)` / `build_qd`
return the row-major 15x15 matrices, or nullopt / `None` on the same rejects.
They exist so C++ and Python tests share one `Phi`. They are not a product API.

## Fixtures

| Case | Expectation |
| --- | --- |
| Zero noise, `P = I` | `P_out = sym(Phi Phi^T)`, abs tol `1e-9`. |
| `Phi` blocks | Every 3x3 block matches the table at `1e-15`. |
| `Qd` | Diagonal `sigma^2 dt` per triad; `P = 0` returns `Qd`. |
| Stationary level hover, `P0 = diag(1, 2, 3, 4, 5)` per triad, `dt = 0.1`, zero noise | `P_out` matches the closed form in the test (for example `P[0,0] = 1.03`, `P[6,6] = 3 + 0.01 (2 g^2 + 4)`, `P[6,4] = 0.2 g`, `P[3,12] = -0.5`). |
| Asymmetric random `P`, positive sigma | `max |P_out - P_out^T| <= 1e-12`. |
| PSD sanity | Hover with `P0 = 1e-6 I` and zero noise, and a positive-sigma fixture: min eigenvalue `>= -1e-9`. |
| Rejects | Each status above, precedence order, no partial result, overflow gives `NON_FINITE`. |
| Determinism | Same inputs give identical outputs. A golden case (30 entries) is pinned to the same values in C++ and Python at `1e-9`. |

## Local runs without Bazel

From the repo root:

```
python3 -m intrinsic.estimation.eskf_cov_propagate_test
g++ -std=c++20 -I. intrinsic/estimation/eskf_cov_propagate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/eskf_cov_propagate_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/eskf_cov_propagate_test \
  && /tmp/eskf_cov_propagate_test
```

Bazel targets: `//intrinsic/estimation:eskf_cov_propagate`,
`:eskf_cov_propagate_test`, `:eskf_cov_propagate_py`,
`:eskf_cov_propagate_test_py`.
