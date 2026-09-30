# Depth update (#87)

Depth pressure / depth-sensor measurement update only. No altitude, barometric,
or air model, no DVL call, lever arm, proto adapter, ICON, safety authority, or
real hardware. Contract: parent WP #30, issue #87. It reuses the #81 types
(`EskfNominal`, `EskfCovariance`, `EskfError`, `kCovDim = 15`), the #82
`QuaternionMultiply` and `kMinQuaternionNorm`, and the #84 `GateInnovation`. The
Joseph and right-error inject pattern follows #85 and #86, but this module does
not call either. #81 to #86 are not modified. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The update is opt-in:
nothing calls it.

Files: `depth_update.{h,cc}` (C++), `depth_update.py` (Python mirror),
`depth_update_test.{cc,py}` (paired tests).

## Frame convention (locked)

The nominal position `p_enu` is ENU (`+Up`). Depth is positive down from a free
surface, and the surface is an ENU Up coordinate:

```
depth_true = free_surface_up_m - p_enu[2]
```

With the default `free_surface_up_m = 0` this is `-p_enu[2]`.

ENU to NED uses the standard axis swap `(n, e, d) = (y_enu, x_enu, -z_enu)`. The
same physical point therefore has `depth = -p_enu_z = +p_ned_down` (for example
`p_enu = (3, -4, -10)` gives `p_ned = (-4, 3, 10)` and depth `10`). The tests
assert this equality and feed `p_ned_down` as the depth sample. A point above
the surface has negative depth. That is finite and accepted as a value.

## API

```
UpdateDepth(x, P, z, chi2_threshold) -> DepthUpdateResult      // C++
update_depth(x, p, z, chi2_threshold) -> DepthUpdateResult     # Python
```

| Argument | Notes |
| --- | --- |
| `x` | `EskfNominal` (#81). Finite, `‖q‖ >= 1e-12`. `q` need not be unit. |
| `P` | `EskfCovariance` (#81). Known and finite. May be asymmetric. |
| `z` | `DepthSample`, below. |
| `chi2_threshold` | Finite and `> 0`. Always caller supplied. Fixtures use `3.841` (1 dof, 95%). |

| `DepthSample` field | Meaning |
| --- | --- |
| `depth_m` | Measured depth, positive down, SI meters. Finite. |
| `R` | Scalar variance, m². Finite and `> 1e-12` (the 1x1 Cholesky pivot floor). |
| `valid` | Must be true (stand-in for MeasurementHealth VALID). Defaults to false. |
| `free_surface_up_m` | ENU Up coordinate of the free surface. Finite. Fixtures use `0.0`. |

## Result

`DepthUpdateResult`: `evaluated`, `accepted`, `status`, `diagnostics`,
`nominal`, `P`, `delta_x` (`nullopt` / `None` when unset).

| Status (C++ / Python) | Meaning | `evaluated` |
| --- | --- | --- |
| `kOkAccept` / `OK_ACCEPT` | Gated in. `nominal`, `P`, and `delta_x` are set. | true |
| `kOkReject` / `OK_REJECT` | Gate outlier (`d² > threshold`). | true |
| `kSkippedInvalid` / `SKIPPED_INVALID` | `valid` is false, or bad threshold, `x`, `q`, `P`, `depth_m`, `free_surface_up_m`, or `R` (non-finite). | false |
| `kSingular` / `SINGULAR` | `R <= 1e-12` (zero, negative, or below the pivot floor), or the gate reported a singular `S`. | false |
| `kNonFinite` / `NON_FINITE` | `h(x)` or `nu` overflowed, the gate overflowed, the update produced a non-finite value, or `‖q ⊗ Exp(dtheta)‖ < 1e-12`. | false for a pre-gate or gate overflow, true for a post-gate failure |

Only `kOkAccept` sets `nominal`, `P`, and `delta_x`, and `accepted` is true only
there. `diagnostics` (the #84 `d²`, `threshold`, `‖nu‖`, `dof = 1`, and `S`) is
set exactly when `evaluated`. Inputs are never modified. The update runs on
temporaries and commits only when everything is finite, so every non-accept
status leaves the caller's `x` and `P` bit-identical.

Checks run in this order and the first defect wins: `valid`, threshold, `x`
finite, `q` norm, `P`, `depth_m` / `free_surface_up_m` / `R` finite, `R` pivot
floor, then the gate. An invalid sample is skipped even if its contents are
garbage.

## Model (locked)

- `h(x) = free_surface_up_m - p_enu[2]`, `nu = z.depth_m - h(x)` (scalar).
- `H` is 1x15: zeros except the `dp` block column for z (column 2) `= -1`,
  because `dh/d(dp_z) = -1`. `R = z.R` (1x1).
- `GateInnovation(nu, H, P, R, chi2_threshold)` with `dof = 1`. Not evaluated
  maps to `SKIPPED_INVALID`, `SINGULAR` (singular `S`), or `NON_FINITE`. Reject
  returns with no write.

On accept, with `P_s = 0.5 (P + P^T)`:

1. `S` is the gate's symmetrized scalar `S_s = P_s[2,2] + R`.
2. `K = P_s H^T / S`, so `K[i] = -P_s[i,2] / S` (15x1).
3. `dx = K nu` (15x1).
4. Inject into the nominal (right / local attitude error, as in #83, #85,
   #86): `p += dp`, `v += dv`, `b_a += dba`, `b_g += dbg`, and
   `q <- normalize(q ⊗ [1, 0.5 dtheta])` (first-order Hamilton, wxyz). Full
   `dx` is injected, so correlated `dtheta` noise is applied too. A norm
   `< 1e-12` before dividing is `NON_FINITE`.
5. Joseph: `P_tmp = (I - K H) P_s (I - K H)^T + K R K^T`, then
   `P_out = 0.5 (P_tmp + P_tmp^T)`.

There is no error-state reset Jacobian on `P` after injection. The contract does
not lock one, and it is not applied.

A deeper reading than predicted (`nu > 0`) moves `p_enu[2]` down (more
negative), because `K[2] = -P_s[2,2] / S < 0`.

## Fixtures

| Case | Expectation |
| --- | --- |
| Accepted hover: `p = (0, 0, -10)`, surface `0`, `z.depth = 10`, `R = 0.01`, `P = 0.04 I`, `chi2 = 3.841` | `OK_ACCEPT`. `dx = 0` and the nominal is identical. `S = 0.05`, `K[2] = -0.8`, `dp_z` variance `0.04 -> 0.008`, every other diagonal stays `0.04`. `d² = 0`, `dof = 1`. |
| Deeper by `0.1` (`depth = 10.1`), same `P`, `R` | `d² = 0.2`, `‖nu‖ = 0.1`, `p_z = -10.08`. Shallower by `0.1` gives `p_z = -9.92`. |
| Surface offset: surface `+2`, `p_z = -8`, `depth = 10` | Same as the hover. `depth = 10.1` gives `p_z = -8.08`. |
| ENU/NED sign: `p_enu = (3, -4, -10)`, `p_ned_down = 10` | `-p_enu_z == p_ned_down == 10`. Feeding `p_ned_down` as the depth gives `nu = 0`. |
| Correlated `P` (`P[0,2]=0.02`, `P[4,2]=0.01`, `P[9,2]=0.005`, `P[13,2]=0.0025`), `depth = 10.1` | `dp_x = -0.04`, `dtheta_y = -0.02` so `q = normalize(1, 0, -0.01, 0)`, `dba_x = -0.01`, `dbg_y = -0.005`. |
| Outlier: `depth = 11` | `OK_REJECT`, `d² = 20`, no outputs, inputs bit-identical. |
| Threshold `0.2 +/- 1e-6` for `d² = 0.2` | Accept, reject. |
| `valid = false`, even with NaN contents | `SKIPPED_INVALID`. |
| Non-finite depth, surface, or `R`; bad threshold, `P`, `x`, zero or `1e-13` `q`; unknown `P` | `SKIPPED_INVALID`. |
| `R = 0`, negative, `1e-12`, `1e-13` | `SINGULAR`. `R = 2e-12` evaluates. |
| `P[0,2] = 1e200`, `P_zz = 1`, `R = 1`, `nu = 2` (gate passes, Joseph overflows) | `NON_FINITE`, `evaluated`, inputs unchanged. |
| `P_zz = 1.5e308`, `R = 1.5e308` (gate overflows), or `depth = -1.7e308` with surface `1.7e308` | `NON_FINITE`, not evaluated. |
| Asymmetric `P` | Same output as its symmetrization. |
| Symmetric output, diagonal never grows | Checked on the golden case. |
| Determinism and no input writes | Identical inputs give identical outputs. |
| No altitude, baro, or DVL | The Python test checks that no module, sample, or status name contains `altitude`, `baro`, `asl`, `dvl`, `water`, or `bottom`. |
| Golden (C++ and Python) | One case: `p = (1, 2, -3)`, nonunit `q = (0.9, 0.1, -0.2, 0.3)`, surface `0.5`, `depth = 3.6` (so `nu = 0.1`), `R = 0.02`, asymmetric correlated `P`. Pins `d² = 0.1111111111111113`, all 16 nominal entries, and the 15 diagonal `P` entries within `1e-9`, in both languages. |

The golden values were computed independently with numpy (`K`, Joseph, and the
right-error injection written out with matrices), not with this code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.depth_update_test
g++ -std=c++17 -I. intrinsic/estimation/depth_update.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/depth_update_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/depth_update_test \
  && /tmp/depth_update_test
```

If gtest is not installed system-wide, build it from a googletest checkout as
described in `INNOVATION_GATE.md`.

Bazel targets: `//intrinsic/estimation:depth_update`,
`:depth_update_test`, `:depth_update_py`, `:depth_update_test_py`.
