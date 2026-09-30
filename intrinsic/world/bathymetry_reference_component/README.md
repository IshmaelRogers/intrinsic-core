# Bathymetry reference component

Opt-in World component that names an external bathymetry artifact.
`BathymetryReferenceComponent` stores a frame id, an opaque asset reference,
an optional vertical bias in meters, and a `MarineComponentValidity` record.

This message is not an inline height grid, mesh, or occupancy map. It does
not build a world snapshot and it does not assign a snapshot id. It does not
convert `world_enu` and `world_ned`. Existing World component protos are
unchanged. Geometry, physics, and kinematics components are unchanged.
Manipulator joint, Cartesian, motion-planning, and Gazebo contracts are
unchanged. `.github/baseline/manipulator_targets.tsv` does not list these
targets. A default message serializes to zero bytes.

## Message

`bathymetry_reference_component.proto` defines one message.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity` | 1 | `MarineComponentValidity`. Required when this component is present. |
| `frame_id` | 2 | Exactly `world_enu` or `world_ned`. |
| `asset_reference` | 3 | Opaque URI, digest, or key. Empty is rejected when present. |
| `vertical_bias_m` | 4 | Optional meters along `frame_id` +Z. Unset means absent. |

## Frames and units

| `frame_id` | Vertical sense |
| --- | --- |
| `world_enu` | +Z up. |
| `world_ned` | +Z down. |

`vertical_bias_m` is meters in that sense. A positive bias moves the
referenced surface toward +Z of `frame_id`. This package stores the number
and the id. It does not rotate the bias into the other world frame. `body`
and any other id are rejected.

`asset_reference` is compared as a string. Helpers do not parse it and do
not fetch the artifact.

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
4. Present `vertical_bias_m` that is not finite.

An empty message is not a present component. It is not an error and it is
not accepted. A supplied bias of `0` stays distinct from an omitted bias.

## Out of scope

Inline bathymetry samples, occupancy, semantic contact, world snapshot
identity, skew policy, World entity wiring, Gazebo or other simulator
conversion, and vehicle-dynamics evaluation.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. Golden bytes for one referenced-bathymetry example
are fixed in the C++ and Python serialization tests.
