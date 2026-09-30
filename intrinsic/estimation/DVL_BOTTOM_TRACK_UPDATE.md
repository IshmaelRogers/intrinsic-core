# DVL bottom-track update (#85)

Bottom-track body-velocity measurement update only. No water-track, lever arm,
Earth rate, ZUPT, pressure or magnetometer model, proto adapter, ICON, safety
authority, or real hardware. Contract: parent WP #30, issue #85. It reuses the
#81 types (`EskfNominal`, `EskfCovariance`, `EskfError`, `kCovDim = 15`), the
#82 `QuaternionMultiply` and `kMinQuaternionNorm`, and the #84
`GateInnovation`. #81 to #84 are not modified. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged. The update is opt-in:
nothing calls it.

Files: `dvl_bottom_track_update.{h,cc}` (C++), `dvl_bottom_track_update.py`
(Python mirror), `dvl_bottom_track_update_test.{cc,py}` (paired tests).

## API

```
UpdateDvlBottomTrack(x, P, z, chi2_threshold) -> DvlUpdateResult        // C++
update_dvl_bottom_track(x, p, z, chi2_threshold) -> DvlUpdateResult     # Python
```

| Argument | Notes |
| --- | --- |
| `x` | `EskfNominal` (#81). Finite, `‖q‖ >= 1e-12`. `q` need not be unit. |
| `P` | `EskfCovariance` (#81). Known and finite. May be asymmetric. |
| `z` | `DvlBottomTrackSample`, below. |
| `chi2_threshold` | Finite and `> 0`. Always caller supplied. Fixtures use `7.815` (3 dof, 95%). |

`DvlBottomTrackSample` is a plain value (no marine proto link). The Python
fields are spelled the same way.

| Field | Meaning |
| --- | --- |
| `velocity_body_m_s[3]` | Bottom-track velocity in the body frame (REP-103), m/s. |
| `R_body[9]` | Row-major 3x3 covariance. Finite, symmetric within `1e-12`, Cholesky pivots `> 1e-12`. |
| `bottom_lock` | Must be true to attempt an update. Defaults to false. |
| `valid` | Must be true (stand-in for MeasurementHealth VALID). Defaults to false. |

A later adapter from the #76 `DvlMeasurement` is out of scope.

## Result

`DvlUpdateResult`: `evaluated`, `accepted`, `status`, `diagnostics`,
`nominal`, `P`, `delta_x` (`nullopt` / `None` when unset).

| Status (C++ / Python) | Meaning | `evaluated` |
| --- | --- | --- |
| `kOkAccept` / `OK_ACCEPT` | Gated in. `nominal`, `P`, and `delta_x` are set. | true |
| `kOkReject` / `OK_REJECT` | Gate outlier (`d^2 > threshold`). | true |
| `kSkippedLockLoss` / `SKIPPED_LOCK_LOSS` | `bottom_lock` or `valid` is false. | false |
| `kSkippedInvalid` / `SKIPPED_INVALID` | Bad threshold, `x`, `q`, `P`, velocity, or `R` (non-finite or asymmetric). | false |
| `kSingular` / `SINGULAR` | `R` Cholesky failed (a pivot `<= 1e-12`), or the gate reported a singular `S`. | false |
| `kNonFinite` / `NON_FINITE` | The gate overflowed, or the update produced a non-finite value, or `‖q ⊗ Exp(dtheta)‖ < 1e-12`. | false for a gate overflow, true for a post-gate failure |

Only `kOkAccept` sets `nominal`, `P`, and `delta_x`, and `accepted` is true only
there. `diagnostics` (the #84 `d^2`, `threshold`, `‖nu‖`, `dof = 3`, and `S`) is
set exactly when `evaluated`. Inputs are never modified. The update runs on
temporaries and commits only when everything is finite, so every non-accept
status leaves the caller's `x` and `P` bit-identical.

Checks run in this order and the first defect wins: lock and valid, threshold,
`x` finite, `q` norm, `P`, velocity finite, `R` finite, `R` symmetric, `R`
Cholesky, then the gate. A sample without lock is skipped as lock loss even if
its contents are garbage.

## Model (locked)

Body velocity only. The lever arm is zero and there is no attitude coupling.

- `h(x) = v_body`, `nu = z.velocity_body_m_s - x.v_body`.
- `H` is 3x15: zeros except the dv block (columns 6 to 8) `= I3`. `R = R_body`.
- `GateInnovation(nu, H, P, R, chi2_threshold)`. Not evaluated maps to
  `SKIPPED_INVALID`, `SINGULAR` (singular `S`), or `NON_FINITE`. Reject returns
  with no write.

On accept, with `P_s = 0.5 (P + P^T)`:

1. `S` is the gate's symmetrized `S_s`.
2. `K = P_s H^T S^-1` by Cholesky solves, never an explicit inverse.
3. `dx = K nu` (15x1).
4. Inject into the nominal (right / local attitude error, as in #83):
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
| Hover accept: `x` default, `z.v = 0`, `P = 0.04 I`, `R = 0.01 I`, `chi2 = 7.815` | `OK_ACCEPT`. `dx = 0` and the nominal is identical. `S = 0.05`, `K = 0.8`, dv variance `0.04 -> 0.008`, every other diagonal stays `0.04`. `d^2 = 0`. |
| Accept with `nu = (0.1, 0, 0)`, same `P`, `R` | `d^2 = 0.2`, `‖nu‖ = 0.1`, `v_body = (0.08, 0, 0)`. |
| Correlated `P` (`P[0,6]=0.02`, `P[4,6]=0.01`, `P[9,6]=0.005`, `P[13,6]=0.0025`), `nu = (0.1, 0, 0)` | `dp_x = 0.04`, `dtheta_y = 0.02` so `q = normalize(1, 0, 0.01, 0)`, `dba_x = 0.01`, `dbg_y = 0.005`. |
| Deep reject: `z.v = (1, 0, 0)` | `OK_REJECT`, `d^2 = 20`, no outputs, inputs bit-identical. |
| Threshold `0.2 +/- 1e-6` for `d^2 = 0.2` | Accept, reject. |
| `bottom_lock = false`, `valid = false`, even with NaN contents | `SKIPPED_LOCK_LOSS`. |
| Non-finite velocity, `R`, threshold, `P`, `x`, zero or `1e-13` `q`; unknown `P`; `R` off by `1e-11` or `+/-0.25` | `SKIPPED_INVALID`. `R` off by `5e-13` evaluates. |
| `R = 0`, indefinite `R`, `R = 1e-12 I` | `SINGULAR`. `R = 2e-12 I` evaluates. |
| `P[0,6] = 1e200`, `P_vv = 1`, `R = I`, `nu = 2` (gate passes, Joseph overflows) | `NON_FINITE`, `evaluated`, inputs unchanged. |
| `P_vv = 1.5e308`, `R = 1.5e308 I` (gate overflows) | `NON_FINITE`, not evaluated. |
| Asymmetric `P` | Same output as its symmetrization. |
| Symmetric output, diagonal never grows | Checked on the golden case. |
| Determinism and no input writes | Identical inputs give identical outputs. |
| No water-track | The Python test checks that neither the module nor the sample has a name containing `water`. |
| Golden (C++ and Python) | One case: nonunit `q = (0.9, 0.1, -0.2, 0.3)`, asymmetric correlated `P`, non-diagonal `R`. Pins `d^2 = 0.03860645238264895`, all 16 nominal entries, and the 15 diagonal `P` entries within `1e-9`, in both languages. |

The golden values were computed independently with numpy (`K`, Joseph, and the
right-error injection written out with matrices), not with this code.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.dvl_bottom_track_update_test
g++ -std=c++17 -I. intrinsic/estimation/dvl_bottom_track_update.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/dvl_bottom_track_update_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/dvl_bottom_track_update_test \
  && /tmp/dvl_bottom_track_update_test
```

If gtest is not installed system-wide, build it from a googletest checkout as
described in `INNOVATION_GATE.md`.

Bazel targets: `//intrinsic/estimation:dvl_bottom_track_update`,
`:dvl_bottom_track_update_test`, `:dvl_bottom_track_update_py`,
`:dvl_bottom_track_update_test_py`.
