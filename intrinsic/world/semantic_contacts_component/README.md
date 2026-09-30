# Semantic contacts component

Opt-in World component that holds a value list of tracked semantic contacts.
`SemanticContactsComponent` stores a frame id, zero or more `SemanticContact`
entries, and a `MarineComponentValidity` record.

This is not a perception pipeline. There is no detector, tracker, embedding,
or image payload. It does not build a world snapshot and it does not assign a
snapshot id. It does not convert `world_enu` and `world_ned`. Existing World
component protos are unchanged. Geometry, physics, and kinematics components
are unchanged. Manipulator joint, Cartesian, motion-planning, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes.

## Messages

`semantic_contacts_component.proto` defines `SemanticContactsComponent` and
the nested `SemanticContactsComponent.SemanticContact`.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity` | 1 | `MarineComponentValidity`. Required when this component is present. |
| `frame_id` | 2 | Exactly `world_enu` or `world_ned`. |
| `contacts` | 3 | Zero or more `SemanticContact`. An empty list is allowed and clears the set. |

| `SemanticContact` field | Tag | Meaning |
| --- | --- | --- |
| `contact_id` | 1 | Stable identity within this component. Empty and repeated ids are rejected. |
| `pose` | 2 | `intrinsic_proto.Pose` in `frame_id`. Required. Position and quaternion components must be finite. |
| `velocity` | 3 | `intrinsic_proto.Twist` in `frame_id`: linear m/s and angular rad/s. Required. All six components must be finite. |
| `classification` | 4 | Opaque semantic class label, for example `buoy`, `dock`, or `unknown`. Empty is rejected. |
| `confidence` | 5 | Optional, dimensionless, finite, in `[0, 1]` (both ends inclusive). Unset means absent and is not zero. |
| `age` | 6 | `google.protobuf.Duration`. Required. Non-negative age of the track at the observation. |

## Frames and units

| `frame_id` | Vertical sense |
| --- | --- |
| `world_enu` | +Z up. |
| `world_ned` | +Z down. |

Pose and twist of every contact are expressed in `frame_id`. The same
numbers are stored for either id. This package does not rotate them. `body`
and any other id are rejected.

The quaternion is Hamilton `(x, y, z, w)`. It is checked for finite
components only. It is not renormalized and unit length is not required. A
present pose or twist with all-zero values is a supplied value. A missing
message is a defect. Likewise a zero `age` is supplied, and a missing `age`
is rejected. A negative `age`, or nanos outside `[0, 999999999]`, is
rejected.

`contact_id` and `classification` are compared as strings. Helpers do not
trim, case-fold, or parse them.

## Validity

Embedded `validity` is checked by the marine component validity helpers.
Unknown and expired stay on that assessment. This package does not add
`STATE_EXPIRED` or `STATE_DEGRADED`, and it does not map those host results
onto a new error code.

Acceptance requires no structural defect and an accepted validity assessment
(`STATE_VALID` and fresh). The first structural defect wins:

1. `validity` missing, or a structural marine-validity error (empty source
   id, bad observation nanos, bad horizon, bad confidence).
2. `frame_id` outside `world_enu` and `world_ned`.
3. The first defective contact, in list order. The assessment reports the
   contact index and a per-contact error. Within one contact the order is:
   empty `contact_id`, repeated `contact_id`, `pose`, `velocity`,
   `classification`, `confidence`, `age`.

A repeated `contact_id` is reported on the later contact, so the first
duplicate wins as the defect. A defect in an earlier contact wins over a
duplicate in a later one.

An empty message is not a present component. It is not an error and it is
not accepted. A present component with an empty contact list is accepted when
validity is accepted.

## Out of scope

Perception models, detectors, trackers, inline occupancy, world snapshot
identity, skew policy, World entity wiring, Gazebo or other simulator
conversion, and vehicle-dynamics evaluation.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize, including inside a contact. Golden bytes for one
multi-contact example are fixed in the C++ and Python serialization tests.
