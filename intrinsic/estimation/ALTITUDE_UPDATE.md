# Altitude update (#88)

Terrain / seafloor altimeter clearance measurement update only. The measurement
is the vehicle's clearance above the seafloor, positive up. It is not a
barometric above-sea-level altitude, and it does not call or duplicate the depth
update (#87). No DVL call, lever arm, proto adapter, ICON, safety authority, or
real hardware. Contract: parent WP #30, issue #88. It reuses the #81 types
(`EskfNominal`, `EskfCovariance`, `EskfError`, `kCovDim = 15`), the #82
`QuaternionMultiply` and `kMinQuaternionNorm`, and the #84 `GateInnovation`. The
Joseph and right-error inject pattern follows #85 to #87, but this module calls
none of them. #81 to #87 are not modified. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The update is opt-in:
nothing calls it.

Files: `altitude_update.{h,cc}` (C++), `altitude_update.py` (Python mirror),
`altitude_update_test.{cc,py}` (paired tests).

## Frame convention (locked)

The nominal position `p_enu` is ENU (`+Up`). The seafloor is an external ENU Up
coordinate (typically negative). Altitude is positive up above it:

```
altitude_true = p_enu[2] - seafloor_up_m
```

For example `seafloor_up_m = -20` and `p_enu = (0, 0, -10)` give an altitude of
`10`. A vehicle below the supplied seafloor has negative clearance. That is
finite and accepted as a value.

## API

```
UpdateAltitude(x, P, z, seafloor, chi2_threshold) -> AltitudeUpdateResult      // C++
update_altitude(x, p, z, seafloor, chi2_threshold) -> AltitudeUpdateResult     # Python
```

| Argument | Notes |
| --- | --- |
| `x` | `EskfNominal` (#81). Finite, `‖q‖ >= 1e-12`. `q` need not be unit. |
| `P` | `EskfCovariance` (#81). Known and finite. May be asymmetric. |
| `z` | `AltitudeSample`, below. |
| `seafloor` | `SeafloorContext`, below. External, supplied by the caller. |
| `chi2_threshold` | Finite and `> 0`. Always caller supplied. Fixtures use `3.841` (1 dof, 95%). |

| `SeafloorContext` field | Meaning |
| --- | --- |
| `present` | Must be true. Defaults to false (no seafloor known). |
| `seafloor_up_m` | ENU Up coordinate of the seafloor under the vehicle, m. Finite when present. |

| `AltitudeSample` field | Meaning |
| --- | --- |
| `altitude_m` | Measured clearance above the seafloor, positive up, SI meters. Finite. |
| `R` | Scalar variance, m². Finite and `> 1e-12` (the 1x1 Cholesky pivot floor). |
| `valid` | Must be true (stand-in for MeasurementHealth VALID). Defaults to false. |

## Result

`AltitudeUpdateResult`: `evaluated`, `accepted`, `status`, `diagnostics`,
`nominal`, `P`, `delta_x` (`nullopt` / `None` when unset).

| Status (C++ / Python) | Meaning | `evaluated` |
| --- | --- | --- |
| `kOkAccept` / `OK_ACCEPT` | Gated in. `nominal`, `P`, and `delta_x` are set. | true |
| `kOkReject` / `OK_REJECT` | Gate outlier (`d² > threshold`). | true |
| `kSkippedMissingSeafloor` / `SKIPPED_MISSING_SEAFLOOR` | `seafloor.present` is false, or `seafloor_up_m` is non-finite. | false |
| `kSkippedInvalid` / `SKIPPED_INVALID` | `valid` is false, or bad threshold, `x`, `q`, `P`, `altitude_m`, or `R` (non-finite). | false |
| `kSingular` / `SINGULAR` | `R <= 1e-12` (zero, negative, or below the pivot floor), or the gate reported a singular `S`. | false |
| `kNonFinite` / `NON_FINITE` | `h(x)` or `nu` overflowed, the gate overflowed, the update produced a non-finite value, or `‖q ⊗ Exp(dtheta)‖ < 1e-12`. | false for a pre-gate or gate overflow, true for a post-gate failure |

Only `kOkAccept` sets `nominal`, `P`, and `delta_x`, and `accepted` is true only
there. `diagnostics` (the #84 `d²`, `threshold`, `‖nu‖`, `dof = 1`, and `S`) is
set exactly when `evaluated`. Inputs are never modified. The update runs on
temporaries and commits only when everything is finite, so every non-accept
status leaves the caller's `x` and `P` bit-identical.

Checks run in this order and the first defect wins: `valid`, threshold, `x`
finite, `q` norm, `P`, seafloor present and finite, `altitude_m` / `R` finite,
`R` pivot floor, then the gate. An invalid sample is skipped even if its
contents (or the seafloor) are garbage.

## Model (locked)

- `h(x) = p_enu[2] - seafloor_up_m`, `nu = z.altitude_m - h(x)` (scalar).
- `H` is 1x15: zeros except the `dp` block column for z (column 2) `= +1`,
  because `dh/d(dp_z) = +1`. Every other block is 0. `R = z.R` (1x1).
- `GateInnovation(nu, H, P, R, chi2_threshold)` with `dof = 1`. Not evaluated
  maps to `SKIPPED_INVALID`, `SINGULAR` (singular `S`), or `NON_FINITE`. Reject
  returns with no write.

On accept, with `P_s = 0.5 (P + P^T)`:

1. `S` is the gate's symmetrized scalar `S_s = P_s[2,2] + R`.
2. `K = P_s H^T / S`, so `K[i] = P_s[i,2] / S` (15x1).
3. `dx = K nu` (15x1).
4. Inject into the nominal (right / local attitude error, as in #83, #85 to
   #87): `p += dp`, `v += dv`, `b_a += dba`, `b_g += dbg`, and
   `q <- normalize(q ⊗ [1, 0.5 dtheta])` (first-order Hamilton, wxyz). Full
   `dx` is injected, so correlated `dtheta` noise is applied too. A norm
   `< 1e-12` before dividing is `NON_FINITE`.
5. Joseph: `P_tmp = (I - K H) P_s (I - K H)^T + K R K^T`, then
   `P_out = 0.5 (P_tmp + P_tmp^T)`.

There is no error-state reset Jacobian on `P` after injection. The contract does
not lock one, and it is not applied.

A larger reading than predicted (`nu > 0`) moves `p_enu[2]` up, because
`K[2] = P_s[2,2] / S > 0`. The sign is the opposite of the depth update, since
altitude increases with `p_z`.

## Fixtures

| Case | Expectation |
| --- | --- |
| Flat seafloor accept: seafloor `-20`, `p = (0, 0, -10)`, `z.altitude = 10`, `R = 0.01`, `P = 0.04 I`, `chi2 = 3.841` | `OK_ACCEPT`. `dx = 0` and the nominal is identical. `S = 0.05`, `K[2] = +0.8`, `dp_z` variance `0.04 -> 0.008`, every other diagonal stays `0.04`. `d² = 0`, `dof = 1`. |
| Higher by `0.1` (`altitude = 10.1`), same `P`, `R` | `d² = 0.2`, `‖nu‖ = 0.1`, `p_z = -9.92`. Lower by `0.1` gives `p_z = -10.08`. |
| Seafloor offset: seafloor `-18`, `p_z = -8`, `altitude = 10` | Same as the hover. `altitude = 10.1` gives `p_z = -7.92`. |
| Negative clearance: seafloor `-5`, `p_z = -10`, `altitude = -5` | Accepted with `nu = 0`. |
| Correlated `P` (`P[0,2]=0.02`, `P[4,2]=0.01`, `P[9,2]=0.005`, `P[13,2]=0.0025`), `altitude = 10.1` | `dp_x = +0.04`, `dtheta_y = +0.02` so `q = normalize(1, 0, 0.01, 0)`, `dba_x = +0.01`, `dbg_y = +0.005`. |
| Missing seafloor: `present = false`, default context, NaN / `+-inf` `seafloor_up_m` | `SKIPPED_MISSING_SEAFLOOR`, not evaluated, `x` and `P` bit-identical. |
| Invalid sample with a missing seafloor | `SKIPPED_INVALID` (the health flag is checked first). |
| Outlier: `altitude = 11` | `OK_REJECT`, `d² = 20`, no outputs, inputs bit-identical. |
| Threshold `0.2 +/- 1e-6` for `d² = 0.2` | Accept, reject. |
| `valid = false`, even with NaN contents | `SKIPPED_INVALID`. |
| Non-finite altitude or `R`; bad threshold, `P`, `x`, zero or `1e-13` `q`; unknown `P` | `SKIPPED_INVALID`. |
| `R = 0`, negative, `1e-12`, `1e-13` | `SINGULAR`. `R = 2e-12` evaluates. |
| `P[0,2] = 1e200`, `P_zz = 1`, `R = 1`, `nu = 2` (gate passes, Joseph overflows) | `NON_FINITE`, `evaluated`, inputs unchanged. |
| `P_zz = 1.5e308`, `R = 1.5e308` (gate overflows), or `p_z = 1.7e308` with seafloor `-1.7e308` | `NON_FINITE`, not evaluated. |
| Asymmetric `P` | Same output as its symmetrization. |
| Symmetric output, diagonal never grows | Checked on the golden case. |
| Determinism and no input writes | Identical inputs give identical outputs. |
| No depth, baro, or DVL | The Python test checks that no module, sample, context, or status name contains `depth`, `baro`, `dvl`, `water`, `bottom`, or `surface`, and that the three source files never reference the depth update. |
| Golden (C++ and Python) | One case: `p = (1, 2, -3)`, nonunit `q = (0.9, 0.1, -0.2, 0.3)`, seafloor `-5` (so `h = 2`), `altitude = 2.1` (so `nu = 0.1`), `R = 0.02`, asymmetric correlated `P`. Pins `d² = 0.11111111111111112`, all 16 nominal entries, and the 15 diagonal `P` entries within `1e-9`, in both languages. |

The golden values were computed independently with numpy (`K`, Joseph, and the
right-error injection written out with matrices), not with this code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.altitude_update_test
g++ -std=c++17 -I. intrinsic/estimation/altitude_update.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/altitude_update_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/altitude_update_test \
  && /tmp/altitude_update_test
```

If gtest is not installed system-wide, build it from a googletest checkout as
described in `INNOVATION_GATE.md`.

Bazel targets: `//intrinsic/estimation:altitude_update`,
`:altitude_update_test`, `:altitude_update_py`, `:altitude_update_test_py`.
