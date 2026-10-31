# UUV reference-solution simulation leaves

Parent #43. Composition decision: ADR 0002
(`docs/adr/0002-uuv-reference-solution-composition.md`). Sim and test only,
C++ only, host only. No hardware. The targets here are private and are not in
`.github/baseline/manipulator_targets.tsv`.

## Trajectory playback (L43-3, #263)

Lock id `l43-3-trajectory-playback-v1`. `TrajectoryPlayback` is a Gazebo
`ISystemPreUpdate` system that replays a `VehiclePlanResult` kinematically. In
each exact step it sets the model `components::Pose` and the link
`components::WorldLinearVelocity`. It is the reusable form of the test-local
`Mover` in `scenarios/dive_resurface/`. There is no Physics system, no
controller, no clock read and no thread. Simulation time comes only from
`UpdateInfo.simTime`.

### Step rule

All times are integer nanoseconds. `dt_ns` is the playback period (default
`125000000`).

1. A paused update does nothing.
2. If `simTime_ns % dt_ns != 0` the update is skipped. Otherwise
   `k = simTime_ns / dt_ns - 1`. A step with `k < 0` or `k <=` the last applied
   `k` is skipped. There is no private step counter.
3. A pending install with `install_step_k < k` becomes current, the trajectory
   serial increases by one, and `t_start_ns = (install_step_k + 1) * dt_ns`.
   The initial trajectory counts as installed at `k = -1`, so `t_start_ns = 0`.
4. The played time is `t_ns = (k + 1) * dt_ns - t_start_ns`. It is at least
   `dt_ns`, so the first played time is `dt`, never `t = 0`. Install at step 7
   plays step 8 at `t = 0.125 s`.
5. Sample times are `seconds * 1e9 + nanos`. If `t_ns` is at or past the last
   sample, the last sample is copied exactly. If it equals a sample time, that
   sample is copied exactly. Otherwise
   `u = double(t_ns - s_i) / double(s_{i+1} - s_i)` and the state is
   `motion_planning::vehicle::Interpolate(sample_i, sample_{i+1}, u)`.
6. The model and link are looked up by name (`uuv` and `hull` by default) in
   every update. A missing entity counts in `missing_entity_steps()` and
   nothing is applied or recorded.
7. The pose is built as
   `Pose3d(Vector3d(x, y, z), Quaterniond(w, x, y, z))`. The Hamilton
   `(x, y, z, w)` order of the plan maps to the gz `(w, x, y, z)` constructor,
   and no normalisation or Euler conversion happens. The link
   `WorldLinearVelocity` is `embodiment::RotateVector(q, twist.linear)`. The
   angular velocity is not set.
8. Every applied step is appended to `steps()` as
   `{k, sim_time_ns, state, trajectory_serial}`. `last_step()` returns the
   newest one.

Read `steps()` and the counters only between or after `Server::Run` calls.

### Installing a trajectory

`Create` and `InstallTrajectory` validate the plan and return a typed
`PlaybackInstallError`. On any defect the current trajectory is kept.

| Order | Defect | Error |
| --- | --- | --- |
| 1 | `status != kOk` | `kPlanNotOk` |
| 2 | no samples | `kEmptyTrajectory` |
| 3 | `AssessVehicleTrajectory(...).accepted == false` | `kTrajectoryRejected` |
| 4 | `frame_id != world_enu` | `kUnsupportedFrame` |
| 5 | a sample without time or without twist | `kMissingTimeOrTwist` |
| 6 | first sample time is not 0 | `kNonZeroStartTime` |
| 7 | `install_step_k <` the last applied `k` | `kStaleInstallStep` |

Continuity on a swap is not checked. The playback jumps to the new
trajectory's sample at `t = dt`. Continuity of the new start state with the
current truth belongs to the callers (L43-5 and L43-6).

Call `InstallTrajectory` only between `Server::Run` calls. A later install
replaces a pending one that has not been applied yet.

### Files

| File | Content |
| --- | --- |
| `BUILD` | Private visibility, the `playback_test_assets` filegroup, the `trajectory_playback` library and the `trajectory_playback_test` `cc_test`. |
| `trajectory_playback.{h,cc}` | The playback system. |
| `trajectory_playback_test.cc` | Tests T1-T7 plus an orientation-order check. |
| `testdata/playback_test.world.sdf` | World `uuv_playback_test`: step 0.125 s, one model `uuv` with one link `hull`, no plugins. |

The test builds its trajectories with the kinodynamic baseline planner and a
private copy of the replan-fixture binding from
`intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md`: epoch
1 gives 5 samples over `t` in `[0, 2]`, and epoch 2 with
`max_linear_speed = 0.6` gives 9 samples over `t` in `[0, 4]`.

### Build and test

```
bazel test //intrinsic/simulation/gazebo/scenarios/uuv_reference:all
```

## Later leaves append

L43-5, L43-6, L44-2, L44-4 and L44-9 append their targets and sections below
the existing ones in `BUILD` and in this file. They do not reorder or edit the
playback entries.

## Replan trigger (L43-5, #265)

Lock id `l43-5-replan-trigger-v1`. `EvaluateReplan` decides what to do when the
validity binding changes. It is a plain C++ function with no Gazebo, HAL,
driver, socket or `intrinsic_control` dependency, and it reads no clock, starts
no thread and does no I/O. It never sets `deadline_present` and never registers
a planner in a registry. Only a `cancel` flag can stop a replan.

### Rule

First match wins.

1. `new_binding.snapshot_id == current_binding.snapshot_id` (byte equality):
   `kUnchanged`. No check runs and no precondition is read.
2. Preconditions: `kError` with a typed `input_error` and nothing run:
   `active.status != kOk`, empty `active`, a sample without time or twist (or
   with nanos out of range), a first sample time that is not 0 or times that are
   not strictly increasing, `t_now_ns < 0`, a non-finite or non-unit
   `truth_now`, or `goal_request.states_present == false`.
3. `BuildRemainingTrajectory(active, truth_now, t_now_ns)`. `t_now_ns` is
   playback time on `active` in integer nanoseconds. The result is `truth_now`
   at time 0 followed by every sample with `sample_ns > t_now_ns`, rebased to
   `double(sample_ns - t_now_ns) / 1e9`. At or past the last sample only
   `truth_now` remains.
4. Recheck with `CheckTrajectoryValidity` against the whole new binding
   (`snapshot_id`, `descriptor`, `assess_skew`, `timings`, `query_time`,
   `skew_policy`, `bounds`, `fence`, `pose_frame`, `min_clearance_m`) and one
   template clearance sample per sample (replicated privately from the
   baseline planner).
   - `kOk`: `kKept`.
   - `kBounds`, `kGeofence`, `kClearance`: replan.
   - `kBadRequest`, `kStaleSnapshot`, `kFrameError`: `kError`, no replan. The
     binding itself is malformed or stale, and the planner's start probe
     ignores `kFrameError`, so a replan could report a false no-solution.
5. Replan with a fresh `MakeKinodynamicBaselinePlanner` built from a copy of
   `new_binding` whose `run_options` carry only `cancel` (`deadline_present`
   stays false; any `run_options` left in `new_binding` is overwritten). The
   request is `goal_request` unchanged except `start_state = truth_now`.
   `kOk`: `kReplanned` with the trajectory. `kNoSolution`: `kNoSolution`.
   Anything else (`kInvalidRequest`, `kCancelled`, `kDeadlineExceeded`):
   `kError`. A null `dynamics` is `kInvalidRequest`.

`ReplanStepResult` carries `input_error`, `recheck_ran`, `recheck`,
`replan_ran` and `plan_status`, so a real `kInvalidRequest` replan can be told
apart from the default value.

### Binding ownership

This leaf never mutates or stores a binding. After `kKept` or `kReplanned` the
caller (L43-7) adopts `new_binding` as current. After `kNoSolution` or `kError`
the L43-6 and L43-7 locks decide. Views borrowed from `new_binding`
(`pose_frame`, `descriptor`) must outlive the call.

### Mid-trajectory replans (for L43-6, L43-7, L43-8)

With the replan fixture and `max_linear_speed = 0.6`, a replan at 0.5 s is
`kNoSolution`. Once the vehicle is faster than 0.6 m/s (from 0.75 s on) the
replan starts outside the new bounds, so the planner returns `kInvalidRequest`
and the outcome is `kError`, not `kNoSolution`. A perception event at or after
0.75 s therefore does not produce a no-solution outcome. The L43-4 `k_c` choice
and the L43-6 and L43-7 locks must account for this.

### Files

| File | Content |
| --- | --- |
| `replan_trigger.{h,cc}` | `EvaluateReplan`, `BuildRemainingTrajectory` and the result types. |
| `replan_trigger_test.cc` | Tests P1-P12 with a private copy of the replan-fixture binding. |

### Build and test

```
bazel test //intrinsic/simulation/gazebo/scenarios/uuv_reference:replan_trigger_test
```
