# ESKF nominal propagation (#82)

IMU nominal-state propagation only. No covariance, no Phi, no Q, no
measurement update (`#83+`), no bias random walk, no Earth-rate or
transport-rate, no lever arms, no water current, no ICON, no safety authority,
no real hardware. Contract: parent WP #30, issue #82. The #81 layouts
(`eskf_state.*`, `ESKF_LAYOUT.md`) are reused unchanged. Manipulator contracts
and `.github/baseline/manipulator_targets.tsv` are unchanged.

Files: `eskf_propagate.{h,cc}` (C++), `eskf_propagate.py` (Python mirror),
`eskf_propagate_test.{cc,py}` (paired tests).

## API

```
PropagateNominal(const EskfNominal& x, const ImuSample& imu, double dt_s)
    -> PropagateResult                                   // C++
propagate_nominal(x, imu, dt_s) -> PropagateResult       # Python
```

`ImuSample` is a plain value, in the **body** frame, SI:

| Field | Meaning |
| --- | --- |
| `accel_m_s2[3]` | Specific force as measured by the accelerometer (includes the gravity effect). |
| `gyro_rad_s[3]` | Angular rate. |

`PropagateResult` has `ok`, a typed `status`, and `nominal`. `nominal` is set
only when `ok`. A rejected call returns no state and never changes its input.

| Status (C++ / Python) | Meaning |
| --- | --- |
| `kOk` / `OK` | Propagated. |
| `kInvalidDt` / `INVALID_DT` | `dt_s` is not finite, or not in `0 < dt_s <= 1.0`. |
| `kInvalidImu` / `INVALID_IMU` | Any IMU component is non-finite. |
| `kInvalidQuaternion` / `INVALID_QUATERNION` | `||q||` is non-finite or `< 1e-12`, on input or after the attitude step. |
| `kNonFiniteState` / `NON_FINITE_STATE` | Any input nominal component is non-finite, or any output component is. |

## Conventions

World ENU, body REP-103. Hamilton quaternion in **wxyz** order, body to world.
`g_enu = (0, 0, -9.80665)` m/s^2 (`kGravityEnu` / `GRAVITY_ENU`). Biases are
held constant: `b_a` and `b_g` are copied through unchanged.

Let `omega = gyro - b_g`, `a = accel - b_a`, `R = R_body_to_world(q)`.

## Order of operations (locked)

1. `dt_s` must be finite with `0 < dt_s <= 1.0`, else `INVALID_DT`.
2. IMU finite, else `INVALID_IMU`. Nominal finite, else `NON_FINITE_STATE`.
   `||q||` finite and `>= 1e-12`, else `INVALID_QUATERNION`. Checks run in this
   order and the first defect wins. No state change on reject.
3. Attitude, first-order Hamilton:
   `q_dot = 0.5 * q (x) [0, omega]`, `q_tmp = q + dt * q_dot`. If
   `||q_tmp|| < 1e-12` (or is non-finite) return `INVALID_QUATERNION`, else
   `q_new = q_tmp / ||q_tmp||`.
4. Velocity, using the **pre-update** `q` and `v`:
   `v_new = v + dt * (a - omega x v + R(q)^T * g_enu)`.
5. Position, using the **pre-update** `q` and `v`:
   `p_new = p + dt * R(q) * v`.
6. Assign `q_new`, `v_new`, `p_new`. Biases are copied unchanged.
7. Any non-finite output component returns `NON_FINITE_STATE`, never a partial
   state.

## Helpers (same formulas in C++ and Python)

Cross product `a x b = (a1 b2 - a2 b1, a2 b0 - a0 b2, a0 b1 - a1 b0)`.

Hamilton product `a (x) b` for `a = (aw, ax, ay, az)`, `b = (bw, bx, by, bz)`:

```
w = aw bw - ax bx - ay by - az bz
x = aw bx + ax bw + ay bz - az by
y = aw by - ax bz + ay bw + az bx
z = aw bz + ax by - ay bx + az bw
```

`R_body_to_world(q)` is the standard rotation matrix of `q / ||q||`, row-major.
For a unit `q` this is the contract `R(q)`. Normalizing inside the helper keeps
`R` a rotation if a caller passes a slightly non-unit `q`, and the norm is
already checked to be `>= 1e-12` before it is used.

## Fixtures

| Case | Expectation |
| --- | --- |
| Stationary, level and tilted, no bias, `a = -R^T g`, `omega = 0` | `p`, `v`, `q` unchanged for any valid `dt` (abs tol `1e-9`). |
| Constant rate about body z, hover, `v = 0` | Position stays. `q = normalize(1, 0, 0, 0.5 wz dt)`. Yaw is `wz * dt` to `1e-7` for one step, and integrates to `wz * T` over 1000 steps. |
| `dt` of 0, negative, NaN, +/-inf, `> 1.0` | `INVALID_DT`. `dt = 1.0` is accepted. |
| Non-finite IMU component | `INVALID_IMU`. |
| Near-zero quaternion | `INVALID_QUATERNION`. |
| Non-finite nominal, or overflowing output | `NON_FINITE_STATE`. |
| `v = (1, 0, 0)`, `omega = (0, 0, 1)`, hover | `v_new = (1, -dt, 0)`, `p_new = (dt, 0, 0)`: proves the pre-update `q` and `v` ordering. |
| Biases | Subtracted before use, copied through unchanged. |
| Determinism | Same inputs give identical outputs. A golden case is pinned to the same values in the C++ and Python tests. |

## Local runs without Bazel

Bazel is not required to read or check this leaf. From the repo root:

```
python3 -m intrinsic.estimation.eskf_propagate_test
g++ -std=c++20 -I. intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/eskf_propagate_test.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/eskf_propagate_test \
  && /tmp/eskf_propagate_test
```

Bazel targets: `//intrinsic/estimation:eskf_propagate`,
`:eskf_propagate_test`, `:eskf_propagate_py`, `:eskf_propagate_test_py`.
