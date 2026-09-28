# Stamped header and frame policy

Opt-in non-real-time contract for embodiment data. It implements the
`StampedHeader` sketch and the frame, unit, time, and validity rules in
[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
and PDR §5. It does not add `VehicleState`, `DesiredMotion`, vehicle packages,
ICON features, or Gazebo plugins.

Manipulator joint, Cartesian, kinematics, motion-planning, World, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. Callers that never set the new fields keep the previous
wire image: a default `StampedHeader` serializes to zero bytes.

## Message

`stamped_header.proto` defines two messages.

`StampedHeader` fields 1–6 match the PDR sketch:

| Field | Tag | Meaning |
| --- | --- | --- |
| `sequence` | 1 | Producer sequence (`uint64`). |
| `source_time` | 2 | Measurement or model time. |
| `receive_time` | 3 | Ingestion time. |
| `source_id` | 4 | Stable producer id. Not a frame name. |
| `frame_id` | 5 | Explicit frame of the payload. |
| `clock_domain` | 6 | Clock that produced the two timestamps. |

`Validity` is a **companion message** in the same file, not a nested type.
`StampedHeader.validity` is tag 7, appended after the PDR sketch. Later
messages can reuse `Validity` without embedding `StampedHeader`.

### Validity: absent and invalid are distinct

| Observation | Meaning |
| --- | --- |
| `validity` field missing on the wire | Absent. No judgment was supplied. |
| `Validity` present, `STATE_UNSPECIFIED` (0) | Judgment present, unclassified. |
| `Validity` present, `STATE_VALID` (1) | Explicitly usable, subject to the finite-value rule below. |
| `Validity` present, `STATE_INVALID` (2) | Explicit rejection. |

Proto3 message presence is the absence signal. Do not treat a missing field as
`STATE_INVALID`, and do not treat `STATE_UNSPECIFIED` as a missing field.

Non-finite values are invalid samples. A stamp of `STATE_VALID` does not make
NaN or infinity usable. Host helpers report that separately so an absent stamp
is not rewritten into `STATE_INVALID`. An unknown future `State` value stays
on the wire and is not accepted as valid.

## Time

`source_time` is measurement or model time. `receive_time` is ingestion time.
Both are `google.protobuf.Timestamp` (seconds and nanos). `clock_domain` says
how to read them. The frame is never inferred from the message type, and the
clock is never inferred from it either.

| `clock_domain` | Meaning |
| --- | --- |
| `utc` | Unix time. Use it for correlation and logs. |
| `monotonic` | Seconds and nanos from a monotonic epoch. Comparable only with other monotonic readings from that epoch. |
| any other string, including empty | Preserved and not interpreted. |

ICON age and watchdogs use the monotonic clock. Wall clock, including `utc`,
never drives a watchdog. `MonotonicAgeSeconds` returns a value only when
`clock_domain` is exactly `monotonic`, both timestamps are present, nanos are
in `[0, 1000000000)`, and receive time is at or after source time. `utc` and
every other domain return no age. This helper is not an ICON watchdog and does
not command hardware.

`sequence` advances when the new value is strictly greater than the previous
value from the same source. A wrap from `2^64-1` to `0` does not count as an
advance. Proto3 omits a zero `sequence` on the wire; parsers then observe `0`.

## Units

SI only. Positions and translations are meters. Angles are radians. Time
offsets are seconds. The ENU↔NED helper changes axes only; it does not scale
units. When a later field would be ambiguous, its name carries a unit suffix
such as `depth_m` or `pressure_pa`. This package does not add those fields.

## Frames

World at platform boundaries is ENU. Marine NED is an explicit adapter. Body
axes follow [REP-103](https://www.ros.org/reps/rep-0103.html). A frame is a
`frame_id` string. Nothing in this package selects a frame from a message
type.

| Frame id | Axes |
| --- | --- |
| `world_enu` | x east, y north, z up. Platform-boundary world frame. |
| `world_ned` | x north, y east, z down. Marine world frame. |
| other, including `world`, `map`, `odom`, `base_link` | Not a well-known world id. Do not convert them with the ENU↔NED helper. |

The two world origins coincide. For a free vector or a position relative to
that origin:

```text
ned_x = enu_y
ned_y = enu_x
ned_z = -enu_z
```

The same map is its own inverse, so NED→ENU uses the same component rule.
Existing manipulator frame names are not reused and are not reinterpreted.

Body axes stay REP-103 on both sides of the adapter:

| Body axis | Direction |
| --- | --- |
| x | forward |
| y | left |
| z | up |

The adapter does not convert a body to marine FRD (x forward, y right, z
down). Passing a body-frame vector to the world helper is a caller error.

Host helpers live in `intrinsic/embodiment` (`frame_policy.h`,
`frame_policy.py`). `FrameIdMatches` checks an explicit id. `WorldVectorEnuToNed`
does not read a message.

## Quaternion order

Rotations use the Hamilton convention. Storage order is **x, y, z, w**, the
same component order as `intrinsic_proto.Quaternion` in
`intrinsic/math/proto/quaternion.proto`. `w` is the scalar part.

Composition is `q_a_from_c = q_a_from_b * q_b_from_c` with the Hamilton
product. The right-hand quaternion applies first. An active rotation of a
vector is `q * v * q_conjugate` with `v = (x, y, z, 0)`.

A world-from-body orientation (body axes REP-103) converts between world
frames by

```text
q_ned_from_body = q_ned_from_enu * q_enu_from_body
```

A body-frame vector rotated into ENU, then mapped with the world adapter,
matches the same body vector rotated by `q_ned_from_body`. The adapter does
not conjugate an ENU operator into an NED operator; it changes the world
frame the body is expressed in.

`q_ned_from_enu` is `(x, y, z, w) = (√2/2, √2/2, 0, 0)`. That quaternion is
its own inverse up to sign, so the NED→ENU orientation map multiplies by the
same quaternion. Round-trip may flip the overall sign; both signs are the
same rotation. Helpers do not renormalize. A non-unit or non-finite
quaternion is not a valid rotation.

## Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields on parse and reserialize. Golden bytes for one fully populated
header are fixed in the C++ and Python serialization tests. A message that
omits `validity` still parses, and those bytes are a prefix of the golden
that includes `STATE_INVALID`.

## Targets

These targets are separate from the protected manipulator baseline:

- `@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_proto`
- `@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_cc_proto`
- `@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_py_pb2`
- `@intrinsic_apis//intrinsic/embodiment/proto:stamped_header_go_proto`
- `//intrinsic/embodiment:frame_policy`
- `//intrinsic/embodiment:frame_policy_py`
- `//intrinsic/embodiment:frame_policy_test`
- `//intrinsic/embodiment:frame_policy_test_py`
- `//intrinsic/embodiment:stamped_header_serialization_test`
- `//intrinsic/embodiment:stamped_header_serialization_test_py`
