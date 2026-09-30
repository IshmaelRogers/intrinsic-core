# DVL water-track update (#86)

Water-track body-velocity measurement update only. The water current is an
external input (`WaterCurrentEstimate`), never an ESKF state. No estimation of
the current inside the filter, no lever arm, Earth rate, ZUPT, pressure or
magnetometer model, proto adapter, ICON, safety authority, or real hardware.
Contract: parent WP #30, issue #86. It reuses the #81 types (`EskfNominal`,
`EskfCovariance`, `EskfError`, `kCovDim = 15`), the #82 `QuaternionMultiply`,
`RotationBodyToWorld` and `kMinQuaternionNorm`, and the #84 `GateInnovation`.
#81 to #85 are not modified. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The update is opt-in:
nothing calls it.

This is a separate entrypoint from the ground-referenced (bottom-track) update
of #85. It does not call, wrap, or include `UpdateDvlBottomTrack`, and it has no
lock field. The Cholesky, Joseph, and inject steps are private to
`dvl_water_track_update.cc` / `.py` (same numerics as #85). Result types are
twins with a water-track name (`DvlWaterTrackStatus`, `DvlWaterTrackResult`)
because #85 does not expose a shared result header and both headers live in
`intrinsic::estimation`.

Files: `dvl_water_track_update.{h,cc}` (C++), `dvl_water_track_update.py`
(Python mirror), `dvl_water_track_update_test.{cc,py}` (paired tests).

## API

```
UpdateDvlWaterTrack(x, P, z, current, chi2_threshold) -> DvlWaterTrackResult      // C++
update_dvl_water_track(x, p, z, current, chi2_threshold) -> DvlWaterTrackResult   # Python
```

| Argument | Notes |
| --- | --- |
| `x` | `EskfNominal` (#81). Finite, `‖q‖ >= 1e-12`. `q` need not be unit. |
| `P` | `EskfCovariance` (#81). Known and finite. May be asymmetric. |
| `z` | `DvlWaterTrackSample`, below. |
| `current` | `WaterCurrentEstimate`, below. |
| `chi2_threshold` | Finite and `> 0`. Always caller supplied. Fixtures use `7.815` (3 dof, 95%). |

`DvlWaterTrackSample` is a plain value (no marine proto link). The Python
fields are spelled the same way.

| Field | Meaning |
| --- | --- |
| `velocity_body_m_s[3]` | Water-track velocity (relative to the water) in the body frame (REP-103), m/s. |
| `R_body[9]` | Row-major 3x3 covariance. Finite, symmetric within `1e-12`, Cholesky pivots `> 1e-12`. The caller folds current uncertainty into it if desired. |
| `valid` | Must be true (stand-in for MeasurementHealth VALID). Defaults to false. |

`WaterCurrentEstimate` is a plain value.

| Field | Meaning |
| --- | --- |
| `present` | If false the update is skipped. Defaults to false. |
| `v_enu_m_s[3]` | Water current in world ENU, m/s. Finite when present. |
| `R_current_enu[9]` | Unused by this update. Never read, even when non-finite. |

A later adapter from the #76 `DvlMeasurement` is out of scope.

## Result

`DvlWaterTrackResult`: `evaluated`, `accepted`, `status`, `diagnostics`,
`nominal`, `P`, `delta_x` (`nullopt` / `None` when unset).

| Status (C++ / Python) | Meaning | `evaluated` |
| --- | --- | --- |
| `kOkAccept` / `OK_ACCEPT` | Gated in. `nominal`, `P`, and `delta_x` are set. | true |
| `kOkReject` / `OK_REJECT` | Gate outlier (`d^2 > threshold`). | true |
| `kSkippedMissingCurrent` / `SKIPPED_MISSING_CURRENT` | `current.present` is false, or `current.v_enu_m_s` is not finite. | false |
| `kSkippedInvalid` / `SKIPPED_INVALID` | `z.valid` is false, or a bad threshold, `x`, `q`, `P`, velocity, or `R` (non-finite or asymmetric). | false |
| `kSingular` / `SINGULAR` | `R` Cholesky failed (a pivot `<= 1e-12`), or the gate reported a singular `S`. | false |
| `kNonFinite` / `NON_FINITE` | `R(q)^T c` overflowed, or the gate overflowed, or the update produced a non-finite value, or `‖q ⊗ Exp(dtheta)‖ < 1e-12`. | false for pre-gate and gate overflow, true for a post-gate failure |

Only `kOkAccept` sets `nominal`, `P`, and `delta_x`, and `accepted` is true only
there. `diagnostics` (the #84 `d^2`, `threshold`, `‖nu‖`, `dof = 3`, and `S`) is
set exactly when `evaluated`. Inputs are never modified. The update runs on
temporaries and commits only when everything is finite, so every non-accept
status leaves the caller's `x` and `P` bit-identical.

Checks run in this order and the first defect wins: `z.valid`, current present
and finite, threshold, `x` finite, `q` norm, `P`, velocity finite, `R` finite,
`R` symmetric, `R` Cholesky, `R(q)^T c` finite, then the gate. A missing
current is skipped as such even if the rest of the sample is garbage.

## Model (locked)

Water-relative body velocity. The lever arm is zero. With `R(q)` the body to
world rotation of `q / ‖q‖` and `c_enu` the external current:

- `c_b = R(q)^T c_enu`.
- `h(x) = v_body - c_b`, `nu = z.velocity_body_m_s - h(x)`.
- `H` is 3x15, zeros except:
  - `dtheta` block (columns 3 to 5) `= -[c_b x]`, i.e.
    `[[0, c_bz, -c_by], [-c_bz, 0, c_bx], [c_by, -c_bx, 0]]`.
  - `dv` block (columns 6 to 8) `= I3`.
- `R = R_body`.
- `GateInnovation(nu, H, P, R, chi2_threshold)`. Not evaluated maps to
  `SKIPPED_INVALID`, `SINGULAR` (singular `S`), or `NON_FINITE`. Reject returns
  with no write.

The `dtheta` block follows from the right (local) attitude error
`R(q ⊗ dq) = R(q) Exp(dtheta)`: `(R Exp(dtheta))^T c = (I - [dtheta x]) c_b`, so
`h` changes by `-[c_b x] dtheta`. The numpy script behind the golden case checks
this block against a central finite difference of `h` (max difference `3e-11`).

On accept, with `P_s = 0.5 (P + P^T)`:

1. `S` is the gate's symmetrized `S_s`.
2. `K = P_s H^T S^-1` by Cholesky solves, never an explicit inverse.
3. `dx = K nu` (15x1).
4. Inject into the nominal (right / local attitude error, as in #83 and #85):
   `p += dp`, `v += dv`, `b_a += dba`, `b_g += dbg`, and
   `q <- normalize(q ⊗ [1, 0.5 dtheta])` (first-order Hamilton, wxyz). A norm
   `< 1e-12` before dividing is `NON_FINITE`.
5. Joseph: `P_tmp = (I - K H) P_s (I - K H)^T + K R K^T`, then
   `P_out = 0.5 (P_tmp + P_tmp^T)`.

There is no error-state reset Jacobian on `P` after injection. The contract does
not lock one, and it is not applied.

## Fixtures

| Case | Expectation |
| --- | --- |
| Hover accept: `x` default, `current = 0`, `z.v = 0`, `P = 0.04 I`, `R = 0.01 I`, `chi2 = 7.815` | `OK_ACCEPT`. `dx = 0` and the nominal is identical. `S = 0.05`, `K = 0.8`, dv variance `0.04 -> 0.008`, every other diagonal stays `0.04`. `d^2 = 0`. |
| Nonzero current: identity `q`, `c_enu = (0.5, 0, 0)`, `v_body = 0`, `z = (-0.5, 0, 0)`, same `P`, `R` | `nu = 0`, `OK_ACCEPT`, nominal identical. `S = diag(0.05, 0.06, 0.06)`. Diagonal `P`: `dv_x = 0.008`, `dv_y = dv_z = 0.04 - 0.04^2/0.06`, `dtheta_y = dtheta_z = 0.04 - 0.02^2/0.06`, `dtheta_x` and all others stay `0.04`. |
| Attitude coupling: same `x`, `P`, `c_enu`, `z = (-0.5, 0.06, 0)` | `dv_y = 0.04`, `dtheta_z = 0.02`, so `q = normalize(1, 0, 0, 0.01)`. |
| `q` = +90 deg about z, `c_enu = (0, 1, 0)`, `z = (-1, 0, 0)` | `c_b = (1, 0, 0)`, `nu = 0`. |
| `present = false`, default `WaterCurrentEstimate`, or non-finite `v_enu_m_s`, even with NaN sample contents and threshold | `SKIPPED_MISSING_CURRENT`, `x` and `P` bit-identical. |
| Deep reject: identity `q`, `c_enu = (0.5, 0, 0)`, `z.v = (1, 0, 0)` | `nu = (1.5, 0, 0)`, `OK_REJECT`, `d^2 = 45`, no outputs, inputs bit-identical. |
| Threshold `0.2 +/- 1e-6` for `d^2 = 0.2` | Accept, reject. |
| `valid = false`; non-finite velocity, `R`, threshold, `P`, `x`, zero or `1e-13` `q`; unknown `P`; `R` off by `1e-11` or `+/-0.25` | `SKIPPED_INVALID`. `R` off by `5e-13` evaluates. |
| `R = 0`, indefinite `R`, `R = 1e-12 I` | `SINGULAR`. `R = 2e-12 I` evaluates. |
| `P[0,6] = 1e200`, `P_vv = 1`, `R = I`, `nu = 2` (gate passes, Joseph overflows) | `NON_FINITE`, `evaluated`, inputs unchanged. |
| `P_vv = 1.5e308`, `R = 1.5e308 I` (gate overflows); `c_enu = 1.7e308` on each axis with non-identity `q` | `NON_FINITE`, not evaluated. |
| Asymmetric `P` | Same output as its symmetrization. |
| `R_current_enu` set to NaN | Same output as the default. |
| Symmetric output, diagonal never grows | Checked on the golden case. |
| Determinism and no input writes | Identical inputs give identical outputs. |
| No ground-referenced velocity API | The Python test checks that neither the module nor the sample has a name containing `bottom` or `lock`, and that the sources do not reference the #85 module or symbols. |
| Golden (C++ and Python) | One case: nonunit `q = (0.9, 0.1, -0.2, 0.3)`, `c_enu = (0.4, -0.3, 0.1)`, `z = (0.15, 0.25, -0.05)`, asymmetric correlated `P`, non-diagonal `R`. Pins `d^2 = 0.4719881583806437`, the 9 entries of `S`, all 16 nominal entries, and the 15 diagonal `P` entries within `1e-9`, in both languages. It exercises the attitude coupling path (`‖c_b‖ > 0`, `q` not identity, `H` dtheta block nonzero). |

The golden values were computed independently with numpy (`c_b`, `H`, `K`,
Joseph, and the right-error injection written out with matrices), not with this
code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.dvl_water_track_update_test
g++ -std=c++17 -I. intrinsic/estimation/dvl_water_track_update.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/dvl_water_track_update_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/dvl_water_track_update_test \
  && /tmp/dvl_water_track_update_test
```

If gtest is not installed system-wide, build it from a googletest checkout as
described in `INNOVATION_GATE.md` (on Debian or Ubuntu, `apt-get install
libgtest-dev` provides the static libraries).

Bazel targets: `//intrinsic/estimation:dvl_water_track_update`,
`:dvl_water_track_update_test`, `:dvl_water_track_update_py`,
`:dvl_water_track_update_test_py`.
