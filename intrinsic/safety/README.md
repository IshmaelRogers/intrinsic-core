# Safety rules

Opt-in pure input gates and envelope rules for a safety filter. They return a
`SafetyRuleResult`. They do not build a `SafetyDecision` and do not aggregate
findings. `state.age` and `state.non_finite` only recommend `REJECT`. The
envelope rules `depth.max` and `altitude.min` may recommend `PROJECT` with a
clamped scalar. World, ICON, HAL, and Gazebo are not read.

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
| `projected_value` | 0.0 | clamped depth or altitude in meters when projected, else 0.0 |

## Rule ids and severities

| Rule | `rule_id` | Severity | Recommends |
| --- | --- | --- | --- |
| State age | `state.age` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `REJECT` (3) |
| Non-finite input | `state.non_finite` | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) |
| Maximum depth, clamp | `depth.max` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Minimum altitude, clamp | `altitude.min` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `PROJECT` (2) |
| Envelope unknown or unusable input | `depth.max`, `altitude.min` | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) |

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
