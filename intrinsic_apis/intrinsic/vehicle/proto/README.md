# Vehicle state and command contracts

Opt-in non-real-time protobuf contracts for a free-body vehicle. The first
consumer is a UUV. They implement the `VehicleState` and `DesiredMotion`
sketches in
[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
and PDR §5, plus `BodyTwist`, `BodyAcceleration`, `BodyWrench`, covariance,
and source health.

PDR §15 places these messages in this package. Common validity stays on
`StampedHeader`. Common provenance is `ModelProvenance` in the embodiment
proto package. This package does not add a robot-type enum, a platform-wide
embodiment switch, ICON feature interfaces, FlatBuffers, Gazebo plugins,
marine world components, inference envelopes, dynamics, or allocation.

Manipulator joint, Cartesian, kinematics, motion-planning, World, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes. Callers that
never set these messages keep the previous manipulator behavior.

Host checks live in `intrinsic/vehicle` (`vehicle_contract_policy.h`,
`vehicle_contract_policy.py`). They do not parse protobuf and they do not
convert frames.

## Messages

| Message | Role |
| --- | --- |
| `VehicleState` | Estimator output. PDR field numbers 1–9. |
| `BodyTwist` | Body-frame velocity, m/s and rad/s. |
| `BodyAcceleration` | Body-frame acceleration, m/s^2 and rad/s^2. |
| `Matrix6` | Row-major 6x6 covariance values. |
| `SourceHealth` | Health of one estimation source. |
| `NavigationMode` | Estimator mode. Not a robot class. |
| `BodyWrench` | Body-frame force and torque. Not a thruster command. |
| `DesiredMotion` | Timestamped intent. Not an actuator command. |

`DesiredMotion.objective` is a oneof: `PoseTarget` (tag 2), `TwistTarget`
(tag 3), `TrajectoryReference` (tag 4). `TrajectoryReference` stores an
opaque id only. It does not contain samples or a planner state space.

## Units

SI only. Field names carry a unit suffix when the quantity would otherwise
be ambiguous.

| Field | Unit |
| --- | --- |
| `Pose.position` | meters |
| `Pose.orientation` | Hamilton quaternion, stored x, y, z, w. Radians are not a field. |
| `BodyTwist.linear_*_m_s` | meters per second |
| `BodyTwist.angular_*_rad_s` | radians per second |
| `BodyAcceleration.linear_*_m_s2` | meters per second squared |
| `BodyAcceleration.angular_*_rad_s2` | radians per second squared |
| `BodyWrench.force_*_n` | newtons |
| `BodyWrench.torque_*_n_m` | newton-meters |
| `DesiredMotion.confidence` | dimensionless, closed interval [0, 1] |
| `DesiredMotion.horizon` | non-negative `google.protobuf.Duration` |

`intrinsic_proto.Pose` is reused and is not widened. Identity orientation
is `w = 1`. The zero quaternion `(0, 0, 0, 0)` is not a rotation. Helpers
do not renormalize.

## Frames

World at platform boundaries is ENU. Marine NED is an explicit adapter
owned by the stamped-header policy. A frame is never inferred from a
message type. This package does not call the ENU↔NED helper.

| Field | Frame |
| --- | --- |
| `VehicleState.header.frame_id` | Frame of `pose_world_from_body` only. Required and non-empty when that pose is present. `world_enu` and `world_ned` are well-known world ids. Any other non-empty id is kept and not converted. |
| `body_twist`, `body_acceleration` | Body frame, REP-103: x forward, y left, z up. Not `header.frame_id`. |
| `twist_covariance` | Same body frame as `body_twist`. |
| `pose_covariance` | Same frame as the pose (`header.frame_id`). |
| `BodyWrench.header.frame_id` | Must be `body` when the wrench message is present. |
| `DesiredMotion` pose objective | `header.frame_id` is the pose frame and must be non-empty. |
| `DesiredMotion` twist objective | `header.frame_id` must be `body`. |
| `DesiredMotion` trajectory objective | Frame is not interpreted here. |

`body` is the explicit body frame id for these contracts. It is not a
manipulator base frame, and it is not a world id. The ENU↔NED helper does
not accept it. Body axes are not converted to marine FRD (x forward, y
right, z down).

## Time

`StampedHeader` rules apply unchanged. `source_time` is measurement or
model time. `receive_time` is ingestion time. `clock_domain` of `monotonic`
is the only domain for which the existing age helper returns a value.
`utc` and every other domain do not drive a watchdog. This package does
not implement an ICON watchdog and does not command hardware.

`VehicleState.estimator_epoch` uses the same advance rule as `sequence`:
the new value must be strictly greater. A wrap from `2^64-1` to `0` does
not advance. Proto3 omits a zero epoch. Parsers then observe `0`.

## Validity, NaN, and infinity

Invalid and absent are distinct. `Validity` is the embodiment companion.
These checks do not rewrite a stamp.

| Observation | Meaning |
| --- | --- |
| Validity field missing | Absent. No judgment was supplied. |
| `STATE_UNSPECIFIED` | Judgment present, unclassified. |
| `STATE_VALID` | Explicitly usable, still subject to the finite-value rule. |
| `STATE_INVALID` | Explicit rejection. |
| Unknown state number | Kept on the wire. Not accepted as valid. Not rewritten to invalid. |

Any NaN or infinity in a present numeric field is an invalid sample.
`STATE_VALID` does not make that sample usable. An absent submessage is
not the same failure: a missing twist is "no twist," not "invalid zero."

A present `BodyTwist`, `BodyAcceleration`, or `BodyWrench` whose
components are all zero is a real zero sample. For the wrench, that zero
sample is the neutral command. The empty message (zero bytes) is not a
command. Producers that want a neutral wrench set `header.frame_id` to
`body` so the message is present even when every component is zero.

`DesiredMotion.confidence` is `optional`. Omitted means not supplied.
A present `0` is a supplied confidence. Values outside `[0, 1]`, and
non-finite values, are rejected. A present `horizon` must have `seconds
>= 0` and `nanos` in `[0, 1000000000)`. A zero duration is supplied. An
unset duration is not.

## Covariance

`Matrix6.values` is row-major and packed. Element `(row, col)` is at index
`row * 6 + col`, with `row` and `col` in `[0, 5]`.

Pose covariance is expressed in `header.frame_id`:

| Index | Component | Unit of a standard deviation |
| --- | --- | --- |
| 0 | x | meter |
| 1 | y | meter |
| 2 | z | meter |
| 3 | rotation about x | radian |
| 4 | rotation about y | radian |
| 5 | rotation about z | radian |

Twist covariance is expressed in the body frame, in `BodyTwist` order:

| Index | Component | Unit of a standard deviation |
| --- | --- | --- |
| 0 | linear x | meter/second |
| 1 | linear y | meter/second |
| 2 | linear z | meter/second |
| 3 | angular x | radian/second |
| 4 | angular y | radian/second |
| 5 | angular z | radian/second |

A matrix element has the product of the two component units.

| Wire | Meaning |
| --- | --- |
| Parent field unset | Unknown covariance. This is the only unknown encoding. |
| Present, length not 36 | Invalid shape. Not unknown. |
| Present, any NaN or infinity | Invalid. Not unknown. |
| Present, asymmetric beyond `1e-9` | Invalid. A covariance is symmetric. |
| Present, 36 zeros | Specified zero matrix. Not unknown. |

Positive-semidefinite checks stay with the estimator. This package checks
shape, finiteness, and symmetry. An unknown covariance does not by itself
fail the structural sample check. A later safety rule may still reject it.
This package does not implement that rule.

## Health

`SourceHealth` is estimation-source health. Actuator health is a later HAL
contract and is not this message.

| Observation | Meaning |
| --- | --- |
| `SourceHealth` entry absent from `sources` | That source was not reported. |
| Entry present, `validity` unset | No health judgment for that source. |
| Entry present, `STATE_INVALID` | Source explicitly unhealthy. |
| Entry present, empty `source_id` | Rejected. |

Duplicate source ids are preserved. Fusion policy is not this package.
`NavigationMode` values outside 0–4 are preserved and classified as
unknown. Unknown is not faulted and not aided.

## Intent and wrench

`DesiredMotion` is timestamped intent. `BodyWrench` is the body-frame
force and torque a later allocator consumes. Neither message is an
actuator command. Neither message is the ICON `BodyWrenchCommand`
feature. This package has no dependency on ICON, HAL, or actuator APIs.

A usable sample, `accepted` on the host assessment, requires
`STATE_VALID` and no structural defect in the fields that are present.
Absence of pose, twist, acceleration, covariance, confidence, horizon, or
provenance is not a structural defect. An empty message is not accepted
and is not an error. A present pose requires a non-empty frame id, a
finite position, and a normalized quaternion. A present wrench or twist
objective requires frame id `body`.

## Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields and unknown enum numbers. Golden bytes for one populated
`VehicleState`, `DesiredMotion`, and `BodyWrench` are fixed in the C++
and Python serialization tests. Clearing `estimator_epoch` leaves a
prefix of the vehicle-state golden. Field 100 is preserved.

## Examples

Text format examples:

- [`examples/vehicle_state.textproto`](examples/vehicle_state.textproto)
  specifies pose covariance and leaves twist covariance unknown.
- [`examples/vehicle_state_unknown_covariance.textproto`](examples/vehicle_state_unknown_covariance.textproto)
  omits both covariance fields.
- [`examples/desired_motion_pose.textproto`](examples/desired_motion_pose.textproto)
- [`examples/desired_motion_twist.textproto`](examples/desired_motion_twist.textproto)
- [`examples/body_wrench.textproto`](examples/body_wrench.textproto)

## Targets

These targets are separate from the protected manipulator baseline:

- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_cc_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_py_pb2`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_state_go_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_cc_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_py_pb2`
- `@intrinsic_apis//intrinsic/vehicle/proto:vehicle_command_go_proto`
- `@intrinsic_apis//intrinsic/vehicle/proto:examples`
- `//intrinsic/vehicle:vehicle_contract_policy`
- `//intrinsic/vehicle:vehicle_contract_policy_py`
- `//intrinsic/vehicle:vehicle_contract_test`
- `//intrinsic/vehicle:vehicle_contract_test_py`
- `//intrinsic/vehicle:vehicle_contract_serialization_test`
- `//intrinsic/vehicle:vehicle_contract_serialization_test_py`
