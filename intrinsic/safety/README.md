# Safety rules

Opt-in pure input gates and envelope rules for a safety filter. They return a
`SafetyRuleResult`. They do not build a `SafetyDecision` and do not aggregate
findings. `state.age` and `state.non_finite` only recommend `REJECT`. The
envelope rules (`depth.max`, `altitude.min`, and the operating-envelope rules
below) may recommend `PROJECT` with a clamped scalar. World, ICON, HAL, and
Gazebo are not read.

Host code lives in this directory. The protobuf contracts in
`intrinsic_apis/intrinsic/safety/proto` are unchanged.

## `SafetyRuleResult`

| Field | Compliant | Violated |
| --- | --- | --- |
| `violated` | false | true |
| `rule_id` | empty | locked id below |
| `severity` | 0 | wire severity below |
| `summary` | empty | short text |
| `recommended_kind` | `UNSPECIFIED` (0) | `REJECT` (3), or `PROJECT` (2) for envelope clamps |
| `has_projected_value` | false | true only when `recommended_kind` is `PROJECT` |
| `projected_value` | 0.0 | clamped scalar in the rule's unit (meters, m/s, radians, or rad/s) when projected, else 0.0 |

## Rule ids and severities

| Rule | `rule_id` | Severity | Recommends |
| --- | --- | --- | --- |
| State age | `state.age` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `REJECT` (3) |
| Non-finite input | `state.non_finite` | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) |
| Maximum depth, clamp | `depth.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Minimum altitude, clamp | `altitude.min` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Linear speed, clamp | `speed.linear.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Pitch, clamp | `attitude.pitch.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Roll, clamp | `attitude.roll.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Angular rate, clamp | `rate.angular.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Descent rate, clamp | `rate.descent.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Ascent rate, clamp | `rate.ascent.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| AABB geofence, hard fence | `geofence.aabb` | `SAFETY_FINDING_SEVERITY_ERROR` (3) outside, `SAFETY_FINDING_SEVERITY_CRITICAL` (4) unusable input | `REJECT` (3), never `PROJECT` |
| Envelope unknown or unusable input | every envelope rule above | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) |

## `state.age`

`EvaluateStateAgeRule` takes an injected `query_time`, `observation_time`,
and `max_age`. Each time is `(seconds, nanos)` with `nanos` in
`[0, 1000000000)`. The fixture default `max_age` is 2 seconds.

| Input | Result |
| --- | --- |
| `nanos` out of range, or `max_age` negative or with `nanos` out of range | violated |
| `observation_time` strictly after `query_time` | violated |
| `query_time - observation_time` `<=` `max_age` | compliant, including equality |
| age strictly greater than `max_age`, including exact + 1 ns | violated |

## `state.non_finite`

`EvaluateNonFiniteRule` checks a span of doubles with `embodiment::IsFinite`.
An empty span is compliant. NaN, +Inf, and -Inf violate. ±0.0 is finite.

Thin wrappers flatten a `BodyVector` (linear, then angular), the numeric
fields of a `DesiredMotionView` (position, orientation, twist, confidence),
and the numeric fields of a `VehicleStateView` (pose, twist, acceleration,
and both covariance spans) into that same check.

## `depth.max` and `altitude.min`

Depth is meters positive down from the surface. Altitude is meters positive
up from the seafloor. Fixture defaults are `max_depth = 100.0` and
`min_altitude = 2.0`. Equality with a limit is compliant (closed interval).
A finite value just outside the limit is clamped: the result is `PROJECT`
at ERROR with `has_projected_value` true and `projected_value` equal to the
limit. Only the scalar is clamped. No `DesiredMotion` or `applied_intent` is
built, and no trajectory is planned. Assembling a `SafetyDecision` is left to
the aggregator.

| Rule | Input | Result |
| --- | --- | --- |
| `depth.max` | `depth` or `max_depth` non-finite, or `max_depth < 0` | CRITICAL `REJECT`, no projection |
| `depth.max` | `depth <= max_depth` | compliant |
| `depth.max` | `depth > max_depth` | ERROR `PROJECT`, `projected_value = max_depth` |
| `altitude.min` | `altitude_known` false | CRITICAL `REJECT`, no projection (unknown altitude fails closed) |
| `altitude.min` | `altitude` or `min_altitude` non-finite, or `min_altitude < 0` | CRITICAL `REJECT`, no projection |
| `altitude.min` | `altitude >= min_altitude` | compliant |
| `altitude.min` | `altitude < min_altitude` | ERROR `PROJECT`, `projected_value = min_altitude` |

## Operating-envelope rules: speed, attitude, angular rate, heave

Pure scalar gates in `speed_max_rule`, `attitude_bound_rule`,
`angular_rate_max_rule`, and `heave_rate_bound_rule` (C++ and Python, with
paired tests). Angles are radians, rates are per second, and depth is positive
down. Equality with a limit is compliant. A finite value just outside a limit
is `PROJECT` at ERROR with `has_projected_value` true. A non-finite input or an
unusable limit (non-finite, or negative) is CRITICAL `REJECT` with no
projection. Only the scalar is clamped. No `DesiredMotion` or `applied_intent`
is built, and the results are not aggregated into a `SafetyDecision`.

| Rule id | Entry point | Fixture default | Compliant when | Over limit: `projected_value` |
| --- | --- | --- | --- | --- |
| `speed.linear.max` | `EvaluateSpeedMaxRule(speed, max_linear_speed)` | 1.5 m/s | `0 <= speed <= max` | `max_linear_speed` |
| `attitude.pitch.max` | `EvaluateAttitudePitchMaxRule(pitch_rad, max_pitch)` | pi/6 rad | `abs(WrapToPi(pitch)) <= max` | `copysign(max_pitch, wrapped)` |
| `attitude.roll.max` | `EvaluateAttitudeRollMaxRule(roll_rad, max_roll)` | pi/6 rad | `abs(WrapToPi(roll)) <= max` | `copysign(max_roll, wrapped)` |
| `rate.angular.max` | `EvaluateAngularRateMaxRule(angular_rate_mag, max_angular_rate)` | 0.5 rad/s | `0 <= rate <= max` | `max_angular_rate` |
| `rate.descent.max` | `EvaluateHeaveRateBoundRule(depth_rate, max_descent_rate, max_ascent_rate)` | 0.5 m/s | `depth_rate <= max_descent_rate` | `max_descent_rate` |
| `rate.ascent.max` | same call | 0.5 m/s (magnitude) | `depth_rate >= -max_ascent_rate` | `-max_ascent_rate` |

- Speed and angular rate are Euclidean magnitudes supplied by the caller. A
  negative magnitude is bad input and is rejected.
- `WrapToPi` returns an angle in `(-pi, pi]`. An angle already in that interval
  is returned unchanged. Otherwise `atan2(sin, cos)` is used, with `-pi`
  mapped to `+pi`. Limits are absolute, so the clamp keeps the sign of the
  wrapped angle.
- Heave returns exactly one result per call, the first applicable case: bad
  input, then descent over limit, then ascent over limit. Ascent is a negative
  depth rate. For bad input the `rule_id` is `rate.ascent.max` only when the
  ascent maximum is the sole unusable input. Otherwise it is
  `rate.descent.max`, including a non-finite `depth_rate`.

## `geofence.aabb`

`EvaluateGeofenceAabbRule(fence, pose)` and
`EvaluateGeofenceAabbSegmentRule(fence, start, end)` in `geofence_rule` (C++
and Python, with paired tests) check an injected pose against a closed
axis-aligned box. The inputs are plain structs: `AabbGeofence` (`frame_id`,
`region_id`, `min_x/y/z`, `max_x/y/z`) and `GeofencePose` (`frame_id`, `x`,
`y`, `z`). No World read, collision map, proto, or aggregation is involved.
This rule is a hard fence, not an envelope clamp: an outside pose is `REJECT`
and is never projected or clamped to the nearest point, so
`has_projected_value` is always false.

- Boundary is inclusive: `min_i <= p_i <= max_i` on every axis is compliant,
  including faces, edges, and corners.
- `pose.frame_id` must match `fence.frame_id` exactly (case-sensitive). A
  mismatch is CRITICAL `REJECT` and is never treated as inside.
- Empty `frame_id`, `region_id`, or pose `frame_id`, a non-finite bound or
  coordinate, or any `min_i > max_i` is CRITICAL `REJECT`.
- Outside is ERROR `REJECT` with summary `geofence outside region=<region_id>`.
  For a segment the summary is
  `geofence segment outside region=<region_id> end=<start|end|both>`.
- The segment check tests the two endpoints only. The box is convex, so a
  segment is inside exactly when both endpoints are. A segment that crosses
  the box but starts and ends outside is rejected. There is no edge clipping.
- In C++ the outside summary contains the caller's region id, so it is kept in
  a thread-local ring buffer rather than a string literal. It stays valid
  until at least seven further outside results on the same thread. Copy it to
  keep it longer.
