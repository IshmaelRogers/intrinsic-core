# Fixed-seed dive and resurface scenario

Issue #134. Lock id `uuv-dive-resurface-scenario-v1`. Sim and test only, C++
only, host only. No hardware.

A scripted vehicle dives from the surface to 5.25 m, holds, and resurfaces. The
Depth, SurfaceFix, Dvl and Altimeter plugins observe it with seed 42, and the
samples feed `FakeEstimatorService`. The test checks the sensor outputs, the
surface-fix gate boundary, the fake's mode rule across one dropout window, and
that a second run on fresh instances is bit-identical.

This scenario proves wiring, gating, the fake's mode rule and determinism. It
does not prove dynamics or navigation accuracy.

## Picks (bit cloud, 2026-10-20)

| Id | Pick |
| --- | --- |
| D1-A | Kinematic scripted profile. A test-local `Mover` system sets the model `Pose` and the link `WorldLinearVelocity` in `PreUpdate`. No Physics system, no hydrodynamics, no controller. |
| D2-A | Depth, SurfaceFix, Dvl (`bottom_track`) and Altimeter plugins. No IMU (#238 blocks estimator use) and no Hydrodynamics. |
| D3-A | `FakeEstimatorService` (#29). "Estimator mode" is `GetState().state().mode()`, set by the fake's README mode rule. Depth is asserted from the depth sensor against the truth profile, not from the estimator pose (the fake holds its kinematics). |
| D4-A | A nominal run plus one dropout window `[180,200)` covering all aiding sources during HOLD, which must produce `DEAD_RECKONING` for exactly steps 180-199. |
| D5-A | New package `intrinsic/simulation/gazebo/scenarios/dive_resurface/`. The only edits outside it are additive visibility lines. |
| D6-A | Two in-process runs, each on a fresh `Server`, are bit-identical (tolerance 0). Physical checks use explicit tolerances. No new golden file. |
| D7-A | No episode recording in this leaf. |
| D8-A | In scope: nominal phases, sensor dropout, the surface-fix gate boundary, the fixed-seed repeat and isolation. N/A items are listed below. |

## Files

| File | Content |
| --- | --- |
| `BUILD` | Private visibility, a `filegroup` of the three asset files, and the `dive_resurface_scenario_test` `cc_test`. |
| `dive_resurface.world.sdf` | The world. One model `uuv` with one link `hull`. No `<plugin>`, no `<include>`, no URI. |
| `dive_resurface_sensors.sdf` | Exactly four `<plugin>` elements in the order Depth, SurfaceFix, Dvl, Altimeter. Each has `link_name hull`, `seed 42` and `latency_steps 0`. |
| `dive_resurface_profile.txt` | The truth profile. |
| `dive_resurface_scenario_test.cc` | Tests T1-T9, the profile parser tests and the malformed-profile cases. |
| `README.md` | This file. |

Outside the package, `plugins/{depth,dvl,altimeter,surface_fix}/BUILD` each add
`//intrinsic/simulation/gazebo/scenarios/dive_resurface:__pkg__` to the
visibility of the `*_plugin` target. Nothing else changes.

## How it runs

- The world is loaded with `SetSdfFile` into an empty `ServerConfig`, so there
  is no default Physics, UserCommands or SceneBroadcaster. The update period is
  zero. The test `Environment` sets `HOME`, as in `depth_plugin_test.cc`.
- Each plugin is attached the way `AddDepth` does it: the `<plugin>` element is
  parsed from `dive_resurface_sensors.sdf` with `sdf::Root` and passed to
  `server->AddSystem(plugin, model "uuv", element)`. The Mover and all four
  plugins are added before the first `Run`.
- The server is driven with `Run(true, 1, false)`. A call that does not advance
  the depth adapter's step index is skipped. At most 420 calls are made to
  collect 400 steps. Results are read only after `Run` returns.
- The Mover derives `k = simTime_ns / 125000000 - 1`. The division must be
  exact. It does not keep a private counter.
- The estimator is initialized once per run with `EstimatorConfig{seed: 42}`.
  At each step the samples are ingested in the order Depth, SurfaceFix, Dvl,
  Altimeter, then `Predict(t_k)` with `t_k = (k+1) * 0.125 s`. The envelope is
  built in the test: `header = sample.health().header()`, `source_id` one of
  `depth`, `surface_fix`, `dvl`, `altimeter`, `kind` one of `PRESSURE`,
  `SURFACE_FIX`, `DVL`, `ALTIMETER`, `payload_type` the full proto name, and
  `payload = SerializeAsString()`. This glue is test code, not a production
  bridge.

## Profile format

`dive_resurface_profile.txt`:

```
CONFIG dt_s surface_z_m seabed_z_m steps
SEGMENT label start_step end_step(exclusive) depth_start_m depth_rate_m_per_step
```

- Tokens are whitespace separated. `#` starts a comment line. Numbers are
  parsed with `std::from_chars`.
- Exactly one `CONFIG` line. Segments must be contiguous and cover
  `[0, steps)`.
- Any other line, a wrong token count, a gap or an overlap is a test failure.
  There is no recovery. Labels are descriptive only and are not a mode enum.

At step `k`, with `s` the segment containing `k`:

- depth `d(k) = depth_start + rate * (k - start)`
- truth position `z = surface_z - d`, `x = y = 0`, identity orientation
- world velocity `v_w = (0, 0, -rate / dt)`, ENU, +z up

| Segment | Steps | Depth at start (m) | Rate (m/step) | v_w z (m/s) |
| --- | --- | --- | --- | --- |
| SURFACE_HOLD | [0, 80) | 0.25 | 0 | 0 |
| DIVE | [80, 160) | 0.3125 | 0.0625 | -0.5 |
| HOLD | [160, 240) | 5.25 | 0 | 0 |
| ASCEND | [240, 320) | 5.1875 | -0.0625 | +0.5 |
| SURFACED | [320, 400) | 0.25 | 0 | 0 |

Every value is dyadic, so `d`, `z` and the gate boundary are exact in double.

## Expected outputs (C6)

Tolerances are the noise amplitude plus 1e-9. Noise is uniform on
`[-amp, amp)`. The same values are test constants in
`dive_resurface_scenario_test.cc`.

| # | Check | Expected |
| --- | --- | --- |
| T1 | Alignment | For every k in [0,400): all four adapters have the same step index. Each `result.step` equals the Mover's k. Each emitted sample's `source_time` equals `(k+1) * 0.125 s`. The link world pose equals `(0, 0, z(k))` exactly. Asserted first. |
| T2 | Depth | Present for every k except [180,200). `abs(depth_m - d(k)) <= 0.01 + 1e-9`. `AssessPressureDepth`: `kNone`, not accepted, `DEGRADED`, frame `body`. In HOLD, depth is `5.25 +- 0.01` for k in [160,240) minus the dropout window. |
| T3 | Surface fix and gate boundary | Present for k in [0,84) and [315,400). Absent for k in [84,315). k=83 and k=315 (d = 0.5 exactly) emit. k=84 and k=314 (d = 0.5625) do not. Present samples have `abs(position_x_m)`, `abs(position_y_m) <= 0.1 + 1e-9`, header frame `world_enu`, `AssessSurfaceFix` `kNone`, not accepted, `DEGRADED`. |
| T4 | DVL | Present except [180,200). `mode == MODE_BOTTOM_TRACK`, `bottom_lock == true`, `abs(altitude_m - (30 - d(k))) <= 0.005 + 1e-9`. `velocity_z_m_s` is -0.5 in DIVE, +0.5 in ASCEND and 0 elsewhere, within `0.005 + 1e-9`. `velocity_x_m_s` and `velocity_y_m_s` are 0 within the same tolerance. `AssessDvl` gives `kNone`, `DEGRADED`, not accepted. |
| T5 | Altimeter | Present except [180,200). `has_return == true`, `abs(range_m - (30 - d(k))) <= 0.01 + 1e-9`. `AssessAltimeter` gives `kNone`, `DEGRADED`, not accepted. |
| T6 | Ingest | Every `Ingest` returns `REJECT_REASON_NONE`. Per step, the ingest count equals the number of samples present. |
| T7 | Mode | `INITIALIZING` before step 0, with `Initialize` returning `REJECT_REASON_NONE`. After each `Predict`: `AIDED` for k in [0,180), `DEAD_RECKONING` for k in [180,200), `AIDED` for k in [200,400). Never `FAULTED`. Every `Predict` returns `REJECT_REASON_NONE`. In [180,200) no source emits (SurfaceFix is gated because d = 5.25 > 0.5). |
| T8 | Final surfaced state, k = 399 | Truth `z = -0.25`. Depth `0.25 +- 0.01`. Surface fix present (`kNone`, `DEGRADED`). Mode `AIDED`. Profile label `SURFACED`. |
| T9 | Repeat | Run 2, on a fresh `Server`, fresh plugin instances and a fresh `FakeEstimatorService`, is bit-identical to run 1 in every per-step record: each emitted sample's `SerializeAsString()` (or "absent"), each `Ingest` and `Predict` reject reason, the mode, and the truth pose bits. |

Malformed-profile checks are unit cases on in-memory strings in the same
`cc_test`. Each of these makes the parser fail: a missing CONFIG line, a gap, an
overlap, a wrong token count and a non-numeric token. The test also covers a
second CONFIG, segments that do not cover `[0, steps)`, a first segment that
does not start at 0, an unknown line and a blank line.

Every emitted sample has HAL oracle result `kNone`, health `DEGRADED`, not
accepted. With noise on, the HAL READMEs say "Bias or noise -> `DEGRADED`,
`kNone`, not accepted". The estimator still ingests them because it gates on
header validity only. It never reads `MeasurementHealth.state`.

## Mode is the fake's rule, not a navigation filter

The estimator mode in this scenario is the rule of `FakeEstimatorService`:
`AIDED` when an aiding sample was accepted since the last `Predict`, and
`DEAD_RECKONING` otherwise. It is not the output of a navigation filter. The
fake holds its kinematics, so depth, position and velocity are asserted from the
sensors against the truth profile, never from the estimator pose.

## Not applicable in this leaf

| Id | Item | Reason |
| --- | --- | --- |
| D7 | Episode recording | Locked out by D7-A. No episode schema, replay or recording code is touched. |
| D8 | Policy absent, failed or late | There is no policy or controller in this scenario (D1-A). |
| D8 | Actuator derating | There are no actuators or thrust input. The motion is scripted. |
| D8 | Stale snapshot | There is no World snapshot consumer in the loop. |
| D8 | Planner no-solution | There is no planner in the loop. |
| D8 | Decision event log | There are no decisions to log without a policy or planner. |

## Commands

```
bazel test //intrinsic/simulation/gazebo/scenarios/dive_resurface:dive_resurface_scenario_test --runs_per_test=2
bazel test //intrinsic/simulation/gazebo/plugins/depth:all //intrinsic/simulation/gazebo/plugins/dvl:all //intrinsic/simulation/gazebo/plugins/altimeter:all //intrinsic/simulation/gazebo/plugins/surface_fix:all
bazel test //intrinsic/estimation:fake_estimator_service_test
python3 tools/build_protected_manipulator_targets.py --manifest .github/baseline/manipulator_targets.tsv
```
