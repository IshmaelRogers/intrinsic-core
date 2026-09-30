# Current field component

Opt-in World component for a spatially uniform linear current.
`CurrentFieldComponent` stores a frame id, a linear velocity in meters per
second, and a `MarineComponentValidity` record.

The field is constant in space. There is no shear map and no grid. Angular
current is zero by convention and is not a field. This message does not
convert `world_enu`, `world_ned`, and `body`. It does not call vehicle
dynamics. It does not build a world snapshot and it does not assign a
snapshot id. Existing World component protos are unchanged. Geometry,
physics, and kinematics components are unchanged. Manipulator joint,
Cartesian, motion-planning, and Gazebo contracts are unchanged.
`.github/baseline/manipulator_targets.tsv` does not list these targets. A
default message serializes to zero bytes.

## Message

`current_field_component.proto` defines one message.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity` | 1 | `MarineComponentValidity`. Required when this component is present. |
| `frame_id` | 2 | Exactly `world_enu`, `world_ned`, or `body`. |
| `velocity_m_s` | 3 | `intrinsic_proto.Vector3`, meters/second, linear, in `frame_id`. |

## Frames and units

`frame_id` matches vehicle `Environment.current_frame_id`:

| `frame_id` | Velocity axes |
| --- | --- |
| `world_enu` | Meters/second in world ENU. +Z up. |
| `world_ned` | Meters/second in world NED. +Z down. |
| `body` | Meters/second in the body frame. |

The same numeric vector is stored for each id. This package does not rotate
it. Empty and any other id are rejected.

`velocity_m_s` is required when the component is present. Each component
must be finite. A zero vector is a supplied current. Angular velocity is
identically zero and has no tag.

## Validity

Embedded `validity` is checked by the marine component validity helpers.
Unknown and expired stay on that assessment. This package does not add
`STATE_EXPIRED` or `STATE_DEGRADED`, and it does not map those host results
onto a new error code.

Acceptance requires no structural defect and an accepted validity assessment
(`STATE_VALID` and fresh). The first structural defect wins:

1. `validity` missing, or a structural marine-validity error (empty source
   id, bad observation nanos, bad horizon, bad confidence).
2. `frame_id` outside `world_enu`, `world_ned`, and `body`.
3. `velocity_m_s` missing, or any component non-finite.

An empty message is not a present component. It is not an error and it is
not accepted.

## Out of scope

Shear maps, occupancy, semantic contact, world snapshot identity, skew
policy, World entity wiring, Gazebo or other simulator conversion, and
vehicle-dynamics evaluation.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. Golden bytes for one constant-current example are
fixed in the C++ and Python serialization tests.
