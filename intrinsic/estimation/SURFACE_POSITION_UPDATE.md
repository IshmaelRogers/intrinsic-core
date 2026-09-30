# Surface position update (#89)

Surfaced horizontal ENU (East, North) GNSS / position fix measurement update
only. The caller supplies a `SurfaceFixPolicy` (`is_surfaced`, `quality_ok`)
that says whether the fix may be used. This leaf does not decide that and does
not keep, read, or write any navigation-mode state: there is no mode machine and
no transition here. The measurement is horizontal only. Up / depth is not part
of this update, and it does not call or duplicate the depth (#87), altitude
(#88), or DVL (#85, #86) updates. No lever arm, proto adapter, ICON, safety
authority, or real hardware. Contract: parent WP #30, issue #89. It reuses the
#81 types (`EskfNominal`, `EskfCovariance`, `EskfError`, `kCovDim = 15`), the
#82 `QuaternionMultiply` and `kMinQuaternionNorm`, and the #84 `GateInnovation`.
The Joseph and right-error inject pattern follows #85 to #88, but this module
calls none of them. #81 to #88 are not modified. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The update is opt-in:
nothing calls it.

Files: `surface_position_update.{h,cc}` (C++), `surface_position_update.py`
(Python mirror), `surface_position_update_test.{cc,py}` (paired tests).

## Frame convention (locked)

The nominal position `p_enu` is ENU. Only East (`p_enu[0]`) and North
(`p_enu[1]`) are measured, in the local ENU tangent frame, in meters:

```
h(x) = [p_enu[0], p_enu[1]]
```

## API

```
UpdateSurfacePosition(x, P, z, policy, chi2_threshold) -> SurfacePositionUpdateResult        // C++
update_surface_position(x, p, z, policy, chi2_threshold) -> SurfacePositionUpdateResult      # Python
```

| Argument | Notes |
| --- | --- |
| `x` | `EskfNominal` (#81). Finite, `‖q‖ >= 1e-12`. `q` need not be unit. |
| `P` | `EskfCovariance` (#81). Known and finite. May be asymmetric. |
| `z` | `SurfacePositionSample`, below. |
| `policy` | `SurfaceFixPolicy`, below. Supplied by the caller. |
| `chi2_threshold` | Finite and `> 0`. Always caller supplied. Fixtures use `5.991` (2 dof, 95%). |

| `SurfaceFixPolicy` field | Meaning |
| --- | --- |
| `is_surfaced` | Must be true. Defaults to false. False (for example submerged) skips the update. |
| `quality_ok` | Must be true. Defaults to false. Stand-in for an SNR / HDOP / fix-type gate already decided upstream. |

| `SurfacePositionSample` field | Meaning |
| --- | --- |
| `position_en_m` | Measured `{East, North}` in the local ENU tangent frame, m. Both finite. |
| `R_en` | Row-major 2x2 covariance, m². Finite, symmetric within `1e-12`, Cholesky pivots `> 1e-12`. |
| `valid` | Must be true (stand-in for MeasurementHealth VALID). Defaults to false. |

## Result

`SurfacePositionUpdateResult`: `evaluated`, `accepted`, `status`, `diagnostics`,
`nominal`, `P`, `delta_x` (`nullopt` / `None` when unset).

| Status (C++ / Python) | Meaning | `evaluated` |
| --- | --- | --- |
| `kOkAccept` / `OK_ACCEPT` | Gated in. `nominal`, `P`, and `delta_x` are set. | true |
| `kOkReject` / `OK_REJECT` | Gate outlier (`d² > threshold`). | true |
| `kSkippedPolicy` / `SKIPPED_POLICY` | `policy.is_surfaced` or `policy.quality_ok` is false. | false |
| `kSkippedInvalid` / `SKIPPED_INVALID` | `valid` is false, or bad threshold, `x`, `q`, `P`, `position_en_m`, or `R_en` (non-finite, wrong size, or asymmetric beyond `1e-12`). | false |
| `kSingular` / `SINGULAR` | `R_en` does not factor (a Cholesky pivot `<= 1e-12`), or the gate reported a singular `S`. | false |
| `kNonFinite` / `NON_FINITE` | `nu` overflowed, the gate overflowed, the update produced a non-finite value, or `‖q ⊗ Exp(dtheta)‖ < 1e-12`. | false for a pre-gate or gate overflow, true for a post-gate failure |

Only `kOkAccept` sets `nominal`, `P`, and `delta_x`, and `accepted` is true only
there. `diagnostics` (the #84 `d²`, `threshold`, `‖nu‖`, `dof = 2`, and the
row-major 2x2 `S`) is set exactly when `evaluated`. Inputs are never modified.
The update runs on temporaries and commits only when everything is finite, so
every non-accept status leaves the caller's `x` and `P` bit-identical.

The policy is the first check. A policy that forbids the fix returns
`SKIPPED_POLICY` without reading the sample, threshold, `x`, or `P`, so a
submerged vehicle can pass whatever sample it has. After that the order is:
`valid`, threshold, `x` finite, `q` norm, `P`, `position_en_m` / `R_en` finite
and sized, `R_en` symmetry, `R_en` Cholesky, `nu` finite, then the gate. The
first defect wins.

There is no navigation-mode output. The statuses above are the whole outcome
surface, and a caller that wants to react to a rejected or skipped fix does so
on its side.

## Model (locked)

- `h(x) = [p_enu[0], p_enu[1]]`, `nu = z.position_en_m - h(x)` (2x1).
- `H` is 2x15: zeros except the `dp` block columns 0 and 1, which form `I2`
  (`dh/d(dp_e) = 1`, `dh/d(dp_n) = 1`). Every other block is 0. `R = z.R_en`.
- `GateInnovation(nu, H, P, R, chi2_threshold)` with `dof = 2`. Not evaluated
  maps to `SKIPPED_INVALID`, `SINGULAR` (singular `S`), or `NON_FINITE`. Reject
  returns with no write.

On accept, with `P_s = 0.5 (P + P^T)`:

1. `S` is the gate's symmetrized 2x2 `S_s = H P_s H^T + R`.
2. `K = P_s H^T S^-1` (15x2), by Cholesky solve of `S`.
3. `dx = K nu` (15x1).
4. Inject into the nominal (right / local attitude error, as in #83, #85 to
   #88): `p += dp`, `v += dv`, `b_a += dba`, `b_g += dbg`, and
   `q <- normalize(q ⊗ [1, 0.5 dtheta])` (first-order Hamilton, wxyz). Full
   `dx` is injected, so correlated `dtheta` noise is applied too. A norm
   `< 1e-12` before dividing is `NON_FINITE`.
5. Joseph: `P_tmp = (I - K H) P_s (I - K H)^T + K R K^T`, then
   `P_out = 0.5 (P_tmp + P_tmp^T)`.

There is no error-state reset Jacobian on `P` after injection. The contract does
not lock one, and it is not applied.

## Fixtures

| Case | Expectation |
| --- | --- |
| Valid surfaced accept: hover at `(e, n) = (3, 4)`, `z = (3, 4)`, `R = 0.01 I2`, `P = 0.04 I`, `chi2 = 5.991` | `OK_ACCEPT`. `dx = 0` and the nominal is identical. `S = 0.05 I2`, `K = 0.8` on each of `dp_e`, `dp_n`, variance `0.04 -> 0.008` for both, every other diagonal stays `0.04`. `d² = 0`, `dof = 2`. |
| Offset fix `z = (3.1, 3.95)`, same `P`, `R` | `nu = (0.1, -0.05)`, `d² = 0.25`, `p = (3.08, 3.96)`, Up unchanged. |
| East only `z = (3.1, 4)` | `p_e = 3.08`, `p_n = 4` (no swap). |
| Correlated `P` (`P[0,3]=0.02`, `P[9,0]=0.005`, `P[13,1]=0.0025`), `z = (3.1, 4)` | `dp_e = +0.08`, `dtheta_x = +0.04` so `q = normalize(1, 0.02, 0, 0)`, `dba_x = +0.01`, `dbg_x = 0`. |
| Submerged policy `is_surfaced = false` | `SKIPPED_POLICY`, not evaluated, `x` and `P` bit-identical. |
| Low-quality policy `quality_ok = false`, default policy, both false | `SKIPPED_POLICY`. |
| Policy forbids the fix with NaN sample, threshold, `x`, `P` | `SKIPPED_POLICY` (policy is checked first). |
| Outlier `z = (4, 4)` | `OK_REJECT`, `d² = 20`, no outputs, inputs bit-identical. |
| Threshold `0.2 +/- 1e-6` for `d² = 0.2` | Accept, reject. |
| `valid = false`, even with NaN contents | `SKIPPED_INVALID`. |
| Non-finite position or `R_en`; wrong sizes; bad threshold, `P`, `x`, zero or `1e-13` `q`; unknown `P` | `SKIPPED_INVALID`. |
| `R_en` asymmetric by more than `1e-12` | `SKIPPED_INVALID`. By `1e-13` it evaluates. |
| `R_en` zero, negative diagonal, pivot `1e-12` / `1e-13`, indefinite (`[0.01 0.02; 0.02 0.01]`), or rank one (`[0.01 0.01; 0.01 0.01]`) | `SINGULAR`. `R = 2e-12 I` evaluates. |
| `P[0,2] = 1e200`, `P = I`, `R = I`, `nu = (2, 0)` (gate passes, Joseph overflows) | `NON_FINITE`, `evaluated`, inputs unchanged. |
| `P_ee = 1.5e308`, `R_ee = 1.5e308` (gate overflows), or `p_e = 1.7e308` with `z_e = -1.7e308` | `NON_FINITE`, not evaluated. |
| Asymmetric `P` | Same output as its symmetrization. |
| Symmetric output, diagonal never grows | Checked on the golden case. |
| Determinism and no input writes | Identical inputs give identical outputs. |
| No navigation-mode transition | The Python test checks that no module, sample, policy, result, or status name contains `mode`, `transition`, `depth`, `altitude`, `dvl`, `baro`, `water`, or `bottom`, that the status set is exactly the six above, and that the three source files never reference another update or a navigation-mode symbol. The C++ test pins the six status values. |
| Golden (C++ and Python) | One case: `p = (1, 2, -3)`, nonunit `q = (0.9, 0.1, -0.2, 0.3)`, `z = (1.1, 1.95)` (so `nu = (0.1, -0.05)`), `R = [0.02 0.004; 0.004 0.03]`, asymmetric correlated `P`. Pins `d² = 0.17928286852589673`, `S = [0.07 0.005; 0.005 0.09]`, all 16 nominal entries, and the 15 diagonal `P` entries within `1e-9`, in both languages. |

The golden values were computed independently with numpy (`K`, Joseph, and the
right-error injection written out with matrices), not with this code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.surface_position_update_test
g++ -std=c++17 -I. intrinsic/estimation/surface_position_update.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/surface_position_update_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/surface_position_update_test \
  && /tmp/surface_position_update_test
```

If gtest is not installed system-wide, build it from a googletest checkout as
described in `INNOVATION_GATE.md`.

Bazel targets: `//intrinsic/estimation:surface_position_update`,
`:surface_position_update_test`, `:surface_position_update_py`,
`:surface_position_update_test_py`.
