# Dive, DVL lock loss, recovery, resurface replay (#90)

Test data and a replay test only. One deterministic episode drives the already
merged estimation APIs end to end and pins the result with assertions and a
digest. Parent WP #30, issue #90, the last leaf of that work package.

This change adds no production logic. It calls only these public APIs, through
their existing headers and Python modules, and modifies none of them:

| API | Issue |
| --- | --- |
| `PropagateNominal` / `propagate_nominal` | #82 |
| `PropagateCovariance` / `propagate_covariance` | #83 |
| `UpdateDvlBottomTrack` / `update_dvl_bottom_track` | #85 |
| `UpdateDvlWaterTrack` / `update_dvl_water_track` | #86 |
| `UpdateDepth` / `update_depth` | #87 |
| `UpdateAltitude` / `update_altitude` | #88 |
| `UpdateSurfacePosition` / `update_surface_position` with a caller `SurfaceFixPolicy` | #89 |

`GateInnovation` (#84) is reached only through those updates. There is no
navigation-mode enum, state machine, or transition anywhere in this change. The
phase names below are labels on fixture lines. The test never reads them to make
a decision. ICON, hardware, and `.github/baseline/manipulator_targets.tsv` are
untouched.

Files: `testdata/dive_lock_loss_resurface.txt` (fixture),
`dive_lock_loss_resurface_replay_test.{cc,py}` (paired replayer and tests),
this document.

## Fixture format

`testdata/dive_lock_loss_resurface.txt`. One event per line, whitespace
separated, SI units. Lines starting with `#` are comments. Times are integer
milliseconds. Flags are `0` or `1`. There is no randomness: every number is
literal text, and both languages parse it with a correctly rounded decimal to
double conversion (`float` / `std::strtod`), so they start from the same bits.

| Line | Fields |
| --- | --- |
| `CONFIG` | `chi2_dof1 chi2_dof2 chi2_dof3 sigma_accel sigma_gyro sigma_accel_bias_rw sigma_gyro_bias_rw` |
| `INIT` | `t_ms px py pz qw qx qy qz vx vy vz` (biases zero) |
| `P0_DIAG` | 15 variances in error-state order (dp, dtheta, dv, dba, dbg) |
| `IMU` | `t_ms phase ax ay az gx gy gz` (specific force, rad/s) |
| `SURFACE` | `t_ms phase is_surfaced quality_ok valid e n r expect` (`R = r I2`) |
| `DEPTH` | `t_ms phase depth r valid free_surface_up expect` |
| `DVL_BT` | `t_ms phase vx vy vz r bottom_lock valid expect` (`R = r I3`) |
| `DVL_WT` | `t_ms phase vx vy vz r valid current_present cx cy cz expect` (`R = r I3`) |
| `ALT` | `t_ms phase altitude r valid seafloor_present seafloor_up expect` |

`expect` is the status name the public update must return, for example
`OK_ACCEPT` or `SKIPPED_LOCK_LOSS`. The same names are used in C++ and Python
(`kOkAccept` is written `OK_ACCEPT`).

### Replay rules

- The state starts at `INIT` with `P = diag(P0_DIAG)`.
- Event times must not decrease. An earlier time is a typed `ReplayError`
  (Python) or a failed replay with `error` set (C++).
- `IMU` at time `t` propagates with `dt = (t - previous IMU time) / 1000`, the
  first one from the `INIT` time. `PropagateCovariance` gets the pre-update
  nominal, the same IMU sample, and the same `dt` as `PropagateNominal`, as
  #83 requires. Both results are committed together.
- A measurement is applied to the current state. Only `OK_ACCEPT` commits the
  returned nominal and `P`. A skip or reject returns no state, and the replayer
  checks that, so `x` and `P` stay bit-identical.
- The surface update gets the `SurfaceFixPolicy` from the line. The caller, here
  the fixture, decides `is_surfaced` and `quality_ok`. The replayer does not.
- Unknown event kinds, unknown phases, wrong token counts, and non-numeric
  fields are typed `FixtureError` (Python) or a failed parse (C++).

## Locked constants

| Constant | Value |
| --- | --- |
| chi2 thresholds (1, 2, 3 dof) | `3.841`, `5.991`, `7.815` (95%) |
| IMU rate | 1 Hz, `t = 1 s ... 120 s` (`dt = 1`, the #82 maximum) |
| Process noise | `sigma_accel = 0.01`, `sigma_gyro = 1e-4`, `sigma_accel_bias_rw = 1e-5`, `sigma_gyro_bias_rw = 1e-7` |
| Initial nominal | `p = (10, 20, -0.2)`, `q = (1, 0, 0, 0)`, `v_body = (0.5, 0, 0)`, biases 0 |
| Initial `P` diagonal | dp `4`, dtheta `1e-8`, dv `0.04`, dba `1e-8`, dbg `1e-10` |
| Free surface | ENU Up `0` |
| Seafloor | ENU Up `-20` |
| Water current | ENU `(0.1, 0.05, 0)` when present |

The initial `P` keeps attitude and accelerometer bias very tight. Gravity turns
attitude error into velocity error, and nothing in this episode observes
attitude, so a loose attitude variance would make the unaided velocity variance
grow to tens of (m/s)^2 before the first DVL sample. That would hide the
lock-loss behaviour this fixture is meant to show.

## Fixture truth

The fixture measurements come from a reference trajectory, not from the filter.
It starts at `p = (10.6, 19.5, -0.25)`, `v_body = (0.52, 0.01, 0)` with identity
attitude, and is integrated with `PropagateNominal` using the same IMU lines.
The filter starts with a deliberately different position and velocity, so the
first updates have nonzero innovations. Measurements are the truth plus a fixed
offset (position `+0.05 / -0.04 m`, depth `+0.02 m`, bottom track
`(0.01, -0.01, 0.005) m/s`, water track `(-0.01, 0.01, 0.005) m/s`, altitude
`+0.03 m`), so no noise generator is involved.

IMU vertical specific force is `9.80665 + a` with `a` the commanded vertical
acceleration. Every IMU line not listed here is hover: `(0, 0, 9.80665)` with
zero gyro.

| Samples (s) | `az` | Effect |
| --- | --- | --- |
| 11 to 14 | `9.70665` | `a = -0.1`, descends to `vz = -0.4 m/s` |
| 41 to 44 | `9.90665` | `a = +0.1`, stops the descent at about 12.27 m depth |
| 93 to 96 | `10.00665` | `a = +0.2`, ascends at `vz = +0.8 m/s` |
| 108 to 111 | `9.60665` | `a = -0.2`, stops the ascent at about 0.27 m depth |

## Phases (fixture tags)

Phases are contiguous by time. The test checks that they appear in this order
and that all seven are present.

| Phase | IMU time (s) | Purpose |
| --- | --- | --- |
| `SURFACE_FIX` | 1 to 10 | Surfaced and high quality fixes accepted. Depth near surface. |
| `DIVE` | 11 to 44 | Descent. Surface policy says submerged, so fixes are skipped. |
| `BOTTOM_LOCK` | 45 to 54 | Bottom track with lock accepted. One outlier rejected. |
| `LOCK_LOSS` | 55 to 74 | Bottom track lock lost. Typed skips. IMU and depth continue. |
| `WATER_TRACK` | 75 to 84 | Recovery through water track and an external current. |
| `ALTITUDE` | 85 to 91 | Altimeter clearance against a known seafloor. |
| `RESURFACE` | 92 to 120 | Ascent, then surfaced fixes accepted again. |

## Measurement events and expected statuses

Times are seconds. Every measurement follows the IMU sample at the same time.

| t | Phase | Event | Input | Status |
| --- | --- | --- | --- | --- |
| 2 | SURFACE_FIX | SURFACE | surfaced, quality ok, `(11.69, 19.48)` | `OK_ACCEPT` |
| 3 | SURFACE_FIX | DEPTH | `0.27` | `OK_ACCEPT` |
| 4 | SURFACE_FIX | SURFACE | surfaced, `quality_ok = 0` | `SKIPPED_POLICY` |
| 6 | SURFACE_FIX | SURFACE | surfaced, quality ok, `(13.77, 19.52)` | `OK_ACCEPT` |
| 8 | SURFACE_FIX | DEPTH | `0.27` | `OK_ACCEPT` |
| 16 | DIVE | SURFACE | `is_surfaced = 0`, quality ok | `SKIPPED_POLICY` |
| 20 | DIVE | DEPTH | `3.27` | `OK_ACCEPT` |
| 28 | DIVE | SURFACE | `is_surfaced = 0`, `quality_ok = 0` | `SKIPPED_POLICY` |
| 32 | DIVE | DEPTH | `8.07` | `OK_ACCEPT` |
| 40 | DIVE | SURFACE | `is_surfaced = 0`, `valid = 0` | `SKIPPED_POLICY` |
| 42 | DIVE | DEPTH | `valid = 0` | `SKIPPED_INVALID` |
| 47 | BOTTOM_LOCK | DVL_BT | lock, `(0.53, 0, 0.005)` | `OK_ACCEPT` |
| 49 | BOTTOM_LOCK | DVL_BT | lock, `(3, 0, 0)` outlier | `OK_REJECT` |
| 51 | BOTTOM_LOCK | DEPTH | `12.27` | `OK_ACCEPT` |
| 53 | BOTTOM_LOCK | DVL_BT | lock, `(0.53, 0, 0.005)` | `OK_ACCEPT` |
| 56 | LOCK_LOSS | DVL_BT | `bottom_lock = 0` | `SKIPPED_LOCK_LOSS` |
| 60 | LOCK_LOSS | DEPTH | `12.27` | `OK_ACCEPT` |
| 62 | LOCK_LOSS | DVL_BT | `bottom_lock = 0` | `SKIPPED_LOCK_LOSS` |
| 66 | LOCK_LOSS | DVL_BT | lock, `valid = 0` | `SKIPPED_LOCK_LOSS` |
| 70 | LOCK_LOSS | DVL_BT | `bottom_lock = 0` | `SKIPPED_LOCK_LOSS` |
| 73 | LOCK_LOSS | DEPTH | `12.27` | `OK_ACCEPT` |
| 77 | WATER_TRACK | DVL_WT | `current_present = 0` | `SKIPPED_MISSING_CURRENT` |
| 79 | WATER_TRACK | DVL_WT | current `(0.1, 0.05, 0)`, `(0.41, -0.03, 0.005)` | `OK_ACCEPT` |
| 82 | WATER_TRACK | DVL_WT | same | `OK_ACCEPT` |
| 84 | WATER_TRACK | DVL_WT | `(2.5, 0, 0)` outlier | `OK_REJECT` |
| 86 | ALTITUDE | ALT | `seafloor_present = 0` | `SKIPPED_MISSING_SEAFLOOR` |
| 88 | ALTITUDE | ALT | `7.78`, seafloor `-20` | `OK_ACCEPT` |
| 90 | ALTITUDE | ALT | `30` outlier | `OK_REJECT` |
| 91 | ALTITUDE | ALT | `7.78` | `OK_ACCEPT` |
| 100 | RESURFACE | DEPTH | `7.87` | `OK_ACCEPT` |
| 106 | RESURFACE | SURFACE | `is_surfaced = 0`, quality ok | `SKIPPED_POLICY` |
| 116 | RESURFACE | SURFACE | surfaced, `quality_ok = 0` | `SKIPPED_POLICY` |
| 117 | RESURFACE | SURFACE | surfaced, quality ok, `(71.49, 20.63)` | `OK_ACCEPT` |
| 118 | RESURFACE | DEPTH | `0.27` | `OK_ACCEPT` |
| 119 | RESURFACE | SURFACE | surfaced, quality ok, `(72.53, 20.65)` | `OK_ACCEPT` |

Totals: 155 events (120 IMU, 35 measurements), 19 `OK_ACCEPT`.

Documented gate results and policy precedence, all observed from the existing
APIs and not changed here:

- The bottom-track update reports `SKIPPED_LOCK_LOSS` for both `bottom_lock = 0`
  and `valid = 0` (#85 checks them together), so the `valid = 0` line at 66 s
  expects `SKIPPED_LOCK_LOSS`, not `SKIPPED_INVALID`.
- The surface update checks the policy first (#89). The line at 40 s has
  `valid = 0` and is still `SKIPPED_POLICY`.
- The depth update has no lock or policy input, so its typed skip is
  `SKIPPED_INVALID`.
- Water track and altitude accept with their context present. Their outlier
  lines are documented `OK_REJECT` gate results.

## Assertions

1. The fixture is timestamped, non-decreasing in time, IMU at exactly 1 s
   spacing, phases in order, and all seven phases present.
2. After each event the status equals the fixture `expect`, every `expect` is
   in the closed set allowed for that sensor, and every IMU step is `OK`.
3. Per-phase status lists are pinned (see the table above).
4. Every skip and reject leaves `x` and `P` bit-identical (exact array
   equality). Every accept changes `P`.
5. For each accept, the observed diagonal entries do not grow and stay under a
   pinned ceiling that is the sensor's own measurement variance plus a margin:

   | Update | Observed `P` entries | Ceiling |
   | --- | --- | --- |
   | `SURFACE` | dp East, North | `0.04` |
   | `DEPTH`, `ALT` | dp Up | `0.01` |
   | `DVL_BT` | dv (3) | `0.0004` |
   | `DVL_WT` | dv (3) | `0.0009 + 1e-4` (the current couples attitude into `h(x)`) |

   A slack of `1e-12` covers floating-point noise.
6. After every step `P` is symmetric within `1e-9` with a strictly positive
   diagonal.
7. Lock loss: through `LOCK_LOSS` the dv variance stays at or below the pinned
   ceiling `0.03 (m/s)^2` (observed maximum about `0.0159`), ends higher than it
   was after the last bottom-lock accept (it grows from `Qd` while DVL is
   skipped), and the first water-track accept after recovery brings it back
   down. The 20 IMU steps inside `LOCK_LOSS` all run and all change `P`.
8. Final nominal (at 120 s) stays within pinned tolerances of the fixture truth
   `p = (73.0, 20.7, -0.25)` (`0.05 m` per axis) and `v_body = (0.52, 0.01, 0)`
   (`0.02 m/s` per axis), with a unit quaternion within `1e-12`.
9. Golden values. Final nominal (all 16 entries) and final `P` diagonal (all 15
   entries) match the pinned floats to `1e-9`, in both languages.

   | Final nominal | Value |
   | --- | --- |
   | `p_enu` | `(73.02814147347405, 20.684643554279752, -0.2714838825824188)` |
   | `q_wxyz` | `(0.9999999999994098, -8.693742969871378e-07, 6.50407971566527e-07, 3.9770524709302564e-08)` |
   | `v_body` | `(0.5091360476894995, 0.022370733591540008, 0.0002562895657194696)` |
   | `b_a` | `(-1.1704345690193751e-07, 7.988003429889057e-08, -5.6663669078122895e-08)` |
   | `b_g` | `(9.967584453900154e-09, -1.6367422251000055e-09, -5.227877923050971e-10)` |

   The 15 `P` diagonal values are listed in the test files. They are a
   regression pin from one replay, not an independent derivation. The C++ and
   Python implementations being separate code that agree is the cross-check.
10. Edge cases, in the same test files: empty episode (state is the initial
    state, no steps), one IMU event, one measurement event, end of stream
    (replaying any prefix equals that step of the full replay, and asking for
    more events than exist is the full replay), unknown or truncated events,
    an out-of-order timestamp, and a replay that never mutates the fixture.

## Digest

The digest is FNV-1a 64 over these bytes, in order:

1. final `x` as 16 little-endian IEEE-754 doubles,
2. final `P` as 225 little-endian doubles, row-major,
3. for each event in order, the ASCII line `<kind>:<status>\n`, for example
   `DVL_BT:SKIPPED_LOCK_LOSS`. IMU steps use `IMU:OK`.

Status names, not enum integers, are hashed because the enum values differ
between sensors.

| Check | Policy |
| --- | --- |
| Two replays in one process | Full digest, final `x`, and final `P` are exactly equal. Enforced in both languages. The second replay re-parses the fixture text. |
| Status sequence across languages | A second digest covers only step 3. It is exactly equal in C++ and Python and is pinned: `0xe487ef7cdb87a065`. |
| Final floats across languages | Pinned golden floats, compared to `1e-9`. |
| Full digest across languages | Not pinned. C++ (GCC, x86-64, `-O2`) and Python produced the same full digest `0xec35407f19869175` when this fixture was written, but a platform with fused multiply-add or a different `libm` may differ in the last bit, so the tests do not pin it. |

If the fixture changes, regenerate the golden floats and the status digest from
one replay and update both test files together, as with the other golden tests
in this package.

## Local runs without Bazel

From the repo root:

```
python3 -m unittest intrinsic.estimation.dive_lock_loss_resurface_replay_test

g++ -std=c++17 -I. \
    intrinsic/estimation/dive_lock_loss_resurface_replay_test.cc \
    intrinsic/estimation/altitude_update.cc \
    intrinsic/estimation/depth_update.cc \
    intrinsic/estimation/dvl_bottom_track_update.cc \
    intrinsic/estimation/dvl_water_track_update.cc \
    intrinsic/estimation/eskf_cov_propagate.cc \
    intrinsic/estimation/eskf_propagate.cc \
    intrinsic/estimation/eskf_state.cc \
    intrinsic/estimation/innovation_gate.cc \
    intrinsic/estimation/surface_position_update.cc \
    -lgtest -lgtest_main -lpthread -o /tmp/dive_lock_loss_resurface_replay_test \
  && /tmp/dive_lock_loss_resurface_replay_test
```

Run the C++ test from the repo root so it finds
`intrinsic/estimation/testdata/dive_lock_loss_resurface.txt`. Under Bazel the
fixture comes from the test data runfiles. If gtest is not installed
system-wide, build it from a googletest checkout as described in
`INNOVATION_GATE.md`.

Bazel targets: `//intrinsic/estimation:dive_lock_loss_resurface_replay_test`,
`//intrinsic/estimation:dive_lock_loss_resurface_replay_test_py`. They are
opt-in estimation targets and are not in
`.github/baseline/manipulator_targets.tsv`.
