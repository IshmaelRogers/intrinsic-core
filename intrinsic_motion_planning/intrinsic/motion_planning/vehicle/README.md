# Vehicle planning

Opt-in `Interpolate`, `Distance`, and `Validate` for a free-body vehicle
planning sample, plus an exact-match planner registry, a deterministic
fake planner, and a deterministic UUV motion-primitive generator. Plain
values only: no protobuf parsing, no World access, no collision checking, no
kinodynamic search. Manipulator joint StateSpace
consumers and the services under `motion_planner/` are unchanged.

[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
records the additive capability decision. The `VehicleState` wire message and
its frame, quaternion, and finiteness rules stay in
[`intrinsic/vehicle`](../../../../intrinsic/vehicle/vehicle_contract_policy.h).
This package does not change them.

| Language | Target | Header or module |
| --- | --- | --- |
| C++ | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_state_space` | `intrinsic/motion_planning/vehicle/vehicle_state_space.h` |
| Python | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_state_space_py` | `intrinsic.motion_planning.vehicle.vehicle_state_space` |
| C++ | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_planner_registry` | `intrinsic/motion_planning/vehicle/vehicle_planner_registry.h` |
| Python | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_planner_registry_py` | `intrinsic.motion_planning.vehicle.vehicle_planner_registry` |
| C++ | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:fake_vehicle_planner` | `intrinsic/motion_planning/vehicle/fake_vehicle_planner.h` |
| Python | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:fake_vehicle_planner_py` | `intrinsic.motion_planning.vehicle.fake_vehicle_planner` |
| C++ | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_planner_deadline` | `intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h` |
| Python | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_planner_deadline_py` | `intrinsic.motion_planning.vehicle.vehicle_planner_deadline` |
| C++ | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_motion_primitives` | `intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h` |
| Python | `//intrinsic_motion_planning/intrinsic/motion_planning/vehicle:vehicle_motion_primitives_py` | `intrinsic.motion_planning.vehicle.vehicle_motion_primitives` |

C++ names live in `intrinsic::motion_planning::vehicle`. The Python mirror uses
snake_case functions (`interpolate`, `distance`, `validate`), tuples for
vectors and quaternions, and an `OK`, `MIX_PARAMETER`, `NON_FINITE`,
`QUATERNION`, `BOUNDS`, `BAD_BOUNDS` enum.

## State layout

`VehiclePlanningState` has three fields. There is no covariance, mode,
sources, epoch, header, or acceleration.

| Field | Layout | Notes |
| --- | --- | --- |
| `position` | x, y, z in meters | World frame. |
| `orientation` | Hamilton x, y, z, w | Identity is `w = 1`. World-from-body. |
| `twist` | linear x, y, z, then angular x, y, z | Body frame, REP-103, same order as `BodyTwist`. Use zeros when there is no velocity. |

ENU and NED are never converted here. Callers keep one world frame per call.

## Mix parameter

`u` is in `[0, 1]` inclusive. Anything else, including NaN, is
`kMixParameter`. There is no extrapolation.

## Interpolate

Check order, first defect wins: `u`, finiteness of both states, unit
orientation of both states. Orientation is unit when
`|norm(q) - 1| <= 1e-9`, the same tolerance as
`embodiment::IsNormalized`. Inputs are rejected, never renormalized.

| Component | Rule |
| --- | --- |
| position | `(1 - u) * a + u * b` |
| twist | componentwise `(1 - u) * a + u * b` |
| orientation | shortest-path slerp |

C++ calls `intrinsic::eigenmath::Interpolate` for orientation. That helper
negates the mix when `dot(a, b) < 0`, so the acute arc is taken. The Python
mirror implements the same formula. Consequences:

* `u == 0` returns `a` and `u == 1` returns `b` exactly, including `b`'s sign.
  Interior samples lie in `a`'s quaternion hemisphere.
* Equal orientations, and an antipodal pair `q`, `-q`, fold to a zero-angle
  delta, so every sample has `a`'s orientation.
* A 180 degree relative rotation has `dot == 0` and is not flipped. For `a`
  the identity and `b = (1, 0, 0, 0)`, `u = 0.5` is `(sqrt(2)/2, 0, 0,
  sqrt(2)/2)`. For `b = (-1, 0, 0, 0)` it is `(-sqrt(2)/2, 0, 0, sqrt(2)/2)`.
  Tests lock both branches.
* The output orientation is normalized to unit length. An input that is
  already unit to within `1e-15` is copied verbatim.
* If the blend overflows to a non-finite value the result is `kNonFinite`.

Documented absolute tolerance for fixtures: position `1e-9` m, quaternion
coefficients `1e-9`, twist `1e-9`.

## Distance

```
sqrt(|dp|^2 + theta^2 + |dv_lin|^2 + |dw|^2)
```

| Term | Meaning |
| --- | --- |
| `dp` | Euclidean position difference, meters. |
| `theta` | Geodesic angle in `[0, pi]` of the shortest rotation from `a` to `b`, radians. |
| `dv_lin`, `dw` | Linear and angular twist differences. |

All weights are 1.0. There are no configuration knobs. `theta` is computed as
`2 * atan2(|v|, |dot|)` with `v` the vector part of `conj(a) * b`. This stays
accurate for small angles and makes the result exactly symmetric.

If either state is non-finite or has a non-unit orientation, the result is
`+infinity` (C++ `std::numeric_limits<double>::infinity()`, Python
`math.inf`). Nothing throws.

## Validate

`VehicleStateBounds` has three independent limits. Each is inert unless its
`*_present` flag is set.

| Limit | Fields | Rule |
| --- | --- | --- |
| position box | `position_min`, `position_max` | Each component in `[min, max]`, inclusive. |
| linear speed | `max_linear_speed_m_s` | `norm(v_lin) <= max`. |
| angular speed | `max_angular_speed_rad_s` | `norm(w) <= max`. |

Check order, first defect wins:

1. `kBadBounds`: an engaged box has `min > max` on any axis or a NaN
   coefficient, or an engaged speed limit is non-finite or negative.
   Disengaged fields are ignored.
2. `kNonFinite`: the state has a non-finite coefficient.
3. `kQuaternion`: the orientation is not unit.
4. `kBounds`: the position box fails.
5. `kBounds`: the linear speed limit fails.
6. `kBounds`: the angular speed limit fails.
7. `kOk`.

Default bounds engage nothing, so only steps 2 and 3 apply.

Interpolation does not clamp. For a convex limit set, a blend of two states
inside it is inside it up to rounding; callers that sit on a limit should
expect `kBounds` from an ulp of rounding.

## Planner registry

Opt-in map from a planner implementation id to a `VehiclePlanner`. Keys are
exact strings. The embodiment category `ai.intrinsic.capability.planning` is
a descriptor advertisement only and is not a registry key. Lookup of an
unknown id returns not-found. The registry does not construct a planner.

| Id | Meaning |
| --- | --- |
| `ai.intrinsic.vehicle_planner.fake` | Deterministic fake. Implemented here. |
| `ai.intrinsic.vehicle_planner.kinodynamic_baseline` | Reserved. Not implemented. |

C++ names live in `intrinsic::motion_planning::vehicle`. The Python mirror
uses `PlannerRegistryError.OK`, `EMPTY_ID`, `DUPLICATE_ID`, `NOT_FOUND`, and
`NULL_PLANNER`. Plan statuses are `OK`, `NO_SOLUTION`,
`INVALID_REQUEST`, `CANCELLED`, and `DEADLINE_EXCEEDED`. The last two are
produced by the deadline harness below; the fake planner never returns them
unless configured to.

`Register` checks, first defect wins:

1. A null planner returns `kNullPlanner`.
2. An empty `id()` returns `kEmptyId`.
3. An id that is already present returns `kDuplicateId`. The first
   registration stays.
4. Otherwise the planner is stored and the result is `kOk`.

`Lookup` checks:

1. An empty id returns `kEmptyId`. The output is unchanged.
2. An unknown id returns `kNotFound`. The output is unchanged.
3. Otherwise the result is `kOk` and the output is the registered planner.

There is no gRPC service and no robot-type enum.

## Fake planner

`FakeVehiclePlanner` reports `ai.intrinsic.vehicle_planner.fake`.
`FakeVehiclePlannerConfig` holds a `VehiclePlanStatus` (default `kOk`) and
`fail_when_start_equals_goal` (default false). There is no sleep, thread,
random source, World access, or StateSpace search.

`Plan` checks, first match wins:

1. An empty `start_label` or `goal_label` returns `kInvalidRequest`. The
   trajectory is absent.
2. `fail_when_start_equals_goal` and equal labels return `kNoSolution`.
   The trajectory is absent.
3. Otherwise the configured status is returned. A trajectory is present
   only for `kOk`.

A success trajectory is the two-sample `world_enu` fixture from
`vehicle_trajectory_two_sample.textproto`: monotonic times, finite pose,
identity orientation, and provenance model id `uuv_planner`. Equal labels
with the failure switch off use that same fixture. An empty request
`trajectory_id` is stored as `fake-trajectory`. The same request produces
the same status, id, and sample times and poses. `AssessVehicleTrajectory`
accepts the success view.

## Deadline and cancellation

`RunWithDeadline` (C++) and `run_with_deadline` (Python) wrap an existing
`VehiclePlanner` with a deadline and a cooperative cancel flag. The wrapper
adds no search and creates no thread.

`VehiclePlanStatus` gains two values appended after the existing ones. Existing
values keep their numbers.

| Status | Value |
| --- | --- |
| `kOk` | 0 |
| `kNoSolution` | 1 |
| `kInvalidRequest` | 2 |
| `kCancelled` | 3 |
| `kDeadlineExceeded` | 4 |

`VehiclePlanRunOptions` holds:

| Field | C++ | Python | Meaning |
| --- | --- | --- | --- |
| deadline | `deadline_present`, `std::chrono::steady_clock::time_point deadline` | `deadline_present`, `deadline` in `time.monotonic()` seconds | Absolute deadline. Ignored unless present. |
| cancel | `std::atomic<bool>* cancel` | `threading.Event` | Not owned. Null or `None` means not cancelable. |

First applicable wins:

1. Cancel set before start returns `kCancelled`. The inner planner is not
   called.
2. Deadline present and `now >= deadline` before start returns
   `kDeadlineExceeded`. The inner planner is not called.
3. Otherwise the inner `Plan` result is returned unchanged. A non-ok status
   such as `kNoSolution` or `kInvalidRequest` is forwarded as is, and a `kOk`
   result keeps its trajectory.

Cancelled, deadline exceeded, and no solution are three different outcomes.
A cancelled or expired result never carries a trajectory;
`AsVehicleTrajectoryView` yields an empty view for it.

Mid-run cancel and timeout are cooperative. The harness itself only checks
before the inner call. A planner that is given the same cancel flag and
deadline can poll them while it works and return `kCancelled` or
`kDeadlineExceeded`, and the harness forwards that. The tests use a test-only
`CooperativeVehiclePlanner` that polls between steps. Its clock and step hook
are injectable, so the mid-run timeout and cancel cases do not depend on wall
time. It is not registered under any id and is not part of the fake planner.

The call is synchronous and bounded by the options. It does not
busy-spin and does not sleep. Tests assert that the cancel and deadline paths
return without the caller waiting on the inner planner.

## Motion primitives

`GenerateUuvMotionPrimitives` (C++) and `generate_uuv_motion_primitives`
(Python) build a deterministic list of constant body-frame wrench controls.
A primitive is a control plus a shared duration. It is not propagated: no
dynamics, currents, or integrators run here.

Controls are `BodyVector` in the `intrinsic/vehicle` layout: linear x, y, z
are force in N, then angular x, y, z are torque in N*m, body frame.

`VehicleMotionPrimitiveConfig`:

| Field | Rule |
| --- | --- |
| `mode` | `kAxisAligned` (default) or `kCustom`. Python: `AXIS_ALIGNED`, `CUSTOM`. |
| `max_force_torque` | Inclusive per-axis magnitude limit. Every component finite and `>= 0`. |
| `duration_s` | Shared by every primitive. Finite and `> 0`. |
| `custom_controls` | Read only for `kCustom`. Each entry finite with `abs(u) <= max` per axis. |

`PrimitiveSetError` is `kOk`, `kBadConfig`, or `kEmpty` (Python `OK`,
`BAD_CONFIG`, `EMPTY`). `primitives` is empty unless the error is `kOk`.

Check order, first defect wins:

1. A `max_force_torque` component is non-finite or negative: `kBadConfig`.
2. `duration_s` is non-finite or `<= 0`: `kBadConfig`.
3. `kCustom` and a control is non-finite or exceeds the max on any axis
   (`abs(u) > max`, exact comparison): `kBadConfig`. Controls are rejected,
   never clamped.
4. `kCustom` with no controls: `kEmpty`.

Axis-aligned set, 13 primitives, positive before negative in each pair:

| Index | Control |
| --- | --- |
| 0 | all zero |
| 1, 2 | `+max.linear_x`, `-max.linear_x` on Fx only |
| 3, 4 | `+max.linear_y`, `-max.linear_y` on Fy only |
| 5, 6 | `+max.linear_z`, `-max.linear_z` on Fz only |
| 7, 8 | `+max.angular_x`, `-max.angular_x` on Tx only |
| 9, 10 | `+max.angular_y`, `-max.angular_y` on Ty only |
| 11, 12 | `+max.angular_z`, `-max.angular_z` on Tz only |

Other components are 0. An axis with a zero bound still emits both entries
of its pair, as all-zero controls (positive zero, so they compare equal bit
for bit); the set stays 13 long. `kCustom` keeps the order of
`custom_controls`.

Ids are `uuv-prim-` plus a zero-padded index in generation order
(`uuv-prim-000`, `uuv-prim-001`, ...). The same config always gives the same
error, ids, controls, and durations.

## Out of scope

* Search, sampling, RRT, A*, kinodynamic planning, TOPP. The kinodynamic
  baseline planner id stays reserved and unregistered.
* Propagating motion primitives through dynamics, currents, or integrators
  (#106).
* Collision checking and World snapshots.
* Asynchronous planning, worker pools, and preemption of a planner that does
  not poll the options.
* Dynamics coupling, ICON, HAL, Gazebo, safety rules, `DesiredMotion`
  assembly.
* ENU and NED conversion.
* Changes to the trajectory wire, the StateSpace API, or manipulator
  `motion_planner` services.

## Build and test

These targets are opt-in. They are not in
`.github/baseline/manipulator_targets.tsv`.

```
bazel test //intrinsic_motion_planning/intrinsic/motion_planning/vehicle:all
```

Property tests are in the same test targets. They use a fixed seed.
