# Occupancy reference component

Opt-in World component that names an external occupancy or free-space map.
`OccupancyReferenceComponent` stores a frame id, an opaque asset reference,
and a `MarineComponentValidity` record.

This message is not an inline voxel grid, point cloud, mesh, or occupancy
sample list. It does not build a world snapshot and it does not assign a
snapshot id. It does not convert `world_enu` and `world_ned`. Existing World
component protos are unchanged. Geometry, physics, and kinematics components
are unchanged. Manipulator joint, Cartesian, motion-planning, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes.

## Message

`occupancy_reference_component.proto` defines one message.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity` | 1 | `MarineComponentValidity`. Required when this component is present. |
| `frame_id` | 2 | Exactly `world_enu` or `world_ned`. |
| `asset_reference` | 3 | Opaque URI, digest, or key. Empty is rejected when present. |

## Frames

| `frame_id` | Vertical sense |
| --- | --- |
| `world_enu` | +Z up. |
| `world_ned` | +Z down. |

The referenced artifact is interpreted in `frame_id`. This package stores
the id. It does not rotate anything into the other world frame. `body` and
any other id are rejected.

`asset_reference` is compared as a string. Helpers do not parse it and do not
fetch the artifact.

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
3. Empty `asset_reference`.

An empty message is not a present component. It is not an error and it is
not accepted.

## Out of scope

Inline occupancy voxels, point clouds, or meshes, semantic contact, world
snapshot identity, skew policy, World entity wiring, perception models,
Gazebo or other simulator conversion, and vehicle-dynamics evaluation.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. Golden bytes for one referenced-occupancy example are
fixed in the C++ and Python serialization tests.
