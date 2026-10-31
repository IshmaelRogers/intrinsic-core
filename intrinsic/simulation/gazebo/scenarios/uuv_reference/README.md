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

## Shadow policy runner (L44-2, #266)

`ShadowPolicyRunner` (`shadow_policy_runner.{h,cc}`) runs a policy worker in
shadow once per sim step through the L44-1 `InferenceClient`, and classifies
every request as absent, failed, late or active. Contract: LOCK
`l44-2-shadow-policy-runner-v1` on #266, on top of S44-1 (#260) D44-4 A and
D44-1 A. Plain C++, sim and test only: no Gazebo, HAL, driver, socket, OIP or
Triton, no clock read, no sleep, no thread. `:shadow_policy_runner` is
`testonly` because `:policy_worker` is.

### Decisions

| # | Rule |
| --- | --- |
| P1 | The runner owns the sim clock. Its client clock returns `now_`, and `Step` sets `now_` from `sim_time_ns` before any queue call. The client never has an empty clock. |
| P2 | `dt_ns > 0` and `sim_time_ns >= 0` are integers. The clock is `clock_origin + sim_time_ns` in integer arithmetic (`clock_origin` defaults to `{0, 0}`). `deadline_ns = sim_time_ns + 2 * dt_ns` (N = 2, `kShadowDeadlineSteps`), and a request is late when `sim_time_ns > deadline_ns`. The recorded L44-1 fixtures need `clock_origin = {1700000000, 0}`. |
| P3 | `StepResult{error, submitted_request_id, resolutions}`. One resolution per request that reached a terminal class this step, oldest `submit_k` first, then a rejected submit of this step. Absent: exactly one `kAbsent`. |
| P4 | Step order: validate and set the clock; no worker is `kAbsent`; `Tick(ticks_per_step)` when > 0; poll every pending request oldest first (a terminal one is reaped and classified, an in-progress one past its deadline is cancelled and is `kLate`); submit this step's envelope (rejected is `kFailed`, accepted is polled and reaped if already terminal). In tick mode a request is never completed in the step that submitted it. |
| P5 | `complete_on_submit = true` and `ticks_per_step = 0` by default, passed through to the queue. |
| P6 | Envelope: header and validity present, `STATE_VALID`, `frame_id = world_enu`, `source_time = now`, `deadline = origin + deadline_ns`, `validity_horizon = 2 * dt`, OIP request `{model_name, model_version, "shadow-<k>"}`, provenance `model_id`. `input_digest`, `state_epoch` and `world_snapshot_id` come verbatim from the caller (P7). No confidence, uncertainty or metadata. Defaults are the L44-1 fixtures' `uuv_shadow_policy` / `1`. |
| P9 | `ClassifySubmitStatus` and `ClassifyTerminal` are public pure functions, tested over every `InferenceQueueStatus`. |
| P10 | `StepResult.error = true`, no queue call and no state change when `dt_ns <= 0`, a worker has no store, `clock_origin` is negative or has nanos out of range, `k` or `sim_time_ns` does not increase, or `sim_time_ns < 0`. The runner also rejects values whose deadline or clock would overflow `int64`. |
| P11 | A cancel race is not handled beyond the `kAlreadyComplete` branch: the runner is single threaded and the only caller of its client. |

### Classes

| Class | Source |
| --- | --- |
| `kFailed` | Submit `kQueueFull`, `kInvalidEnvelope`, `kDeadlineExpiredAtSubmit`, `kShutdown`. Terminal `kReplayMiss`, `kCorruptFixture`. `kComplete` whose `output_digest` is not in the store (`digest_mismatch = true`). Any other status. |
| `kLate` | Terminal `kReplayExpired` (result shell kept, no motion), or `kCancelled` from the runner's own cancel. |
| `kActive` | `kComplete` with a motion found in the store (copied into the resolution). |
| `kAbsent` | No worker configured. |

### Corrections to the issue text

- F5: with one-dt step spacing and N = 2, runner-driven cancels keep at most 3
  requests outstanding, so "9 submits with no completion" never fills the
  queue. The queue-full test uses 9 steps inside one deadline window
  (`sim_time_ns = k` ns, tick mode, 0 ticks).
- F6: `kDeadlineExpiredAtSubmit` is unreachable through `Step` (the deadline is
  never before the submit clock). It is covered on the pure classifier.
- F7: `receive_time` stamping and the wire `ShadowPolicyOutcome` record belong
  to S44-2 and L44-6, not to this leaf. `InferenceEnvelopeView` has no receive
  time.

### Build and test

```
bazel test //intrinsic/simulation/gazebo/scenarios/uuv_reference:shadow_policy_runner_test
```
