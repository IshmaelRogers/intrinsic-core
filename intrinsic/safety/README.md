# Safety rules

Opt-in pure input gates for a safety filter. They return a
`SafetyRuleResult`. They do not build a `SafetyDecision`, do not aggregate
findings, and do not recommend `PROJECT`. World, ICON, HAL, and Gazebo are
not read.

Host code lives in this directory. The protobuf contracts in
`intrinsic_apis/intrinsic/safety/proto` are unchanged.

## `SafetyRuleResult`

| Field | Compliant | Violated |
| --- | --- | --- |
| `violated` | false | true |
| `rule_id` | empty | locked id below |
| `severity` | 0 | wire severity below |
| `summary` | empty | short text |
| `recommended_kind` | `UNSPECIFIED` (0) | `REJECT` (3) |

## Rule ids and severities

| Rule | `rule_id` | Severity | Recommends |
| --- | --- | --- | --- |
| State age | `state.age` | `SAFETY_FINDING_SEVERITY_ERROR` (3) | `REJECT` (3) |
| Non-finite input | `state.non_finite` | `SAFETY_FINDING_SEVERITY_CRITICAL` (4) | `REJECT` (3) |

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
