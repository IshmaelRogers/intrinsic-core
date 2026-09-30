# World snapshot identity

Opt-in descriptor for one World snapshot. `WorldSnapshotDescriptor` carries a
content digest (`snapshot_id`), the creation time, the vehicle-state epoch,
and the revision of each World component kind the snapshot covers.
`WorldSnapshotBuilder` reads each source once and returns an immutable
descriptor. Consumers that hold the same descriptor share the same
`snapshot_id`.

The descriptor holds revisions and identity only. It does not embed
bathymetry, current, occupancy, or contact payloads. It has no skew, age,
freshness, or partial-status fields, and the builder never withholds a
well-formed snapshot. It does not call the marine validity helpers. It is not
wired into the World service. Existing World component protos, `world.h`,
`entity.h`, and WorldEntity tags are unchanged. Geometry, physics, and
kinematics components are unchanged. Manipulator joint, Cartesian,
motion-planning, and Gazebo contracts are unchanged.
`.github/baseline/manipulator_targets.tsv` does not list these targets. A
default message serializes to zero bytes.

## Messages

`world_snapshot_descriptor.proto` defines two messages in
`intrinsic_proto.world`.

`WorldComponentRevision`:

| Field | Tag | Meaning |
| --- | --- | --- |
| `component_kind` | 1 | Non-empty kind key. Empty is rejected. A repeated kind is rejected. |
| `revision` | 2 | Monotonic revision of that kind in the source store. Zero is a valid revision. |

`WorldSnapshotDescriptor`:

| Field | Tag | Meaning |
| --- | --- | --- |
| `snapshot_id` | 1 | Lowercase hex SHA-256 digest, 64 characters. Empty is rejected. |
| `creation_time` | 2 | When the builder formed the snapshot. Required. Nanos in `[0, 1e9)`. Not part of the digest. |
| `state_epoch` | 3 | `VehicleState.estimator_epoch`. Zero is allowed and is part of the digest. |
| `components` | 4 | Zero or more revisions, sorted by `component_kind` ascending. |

Known kinds: `bathymetry_reference`, `current_field`, `occupancy_reference`,
`semantic_contacts`. Any other non-empty kind is accepted, sorted, and hashed
the same way. A kind that is absent from the snapshot is omitted from
`components`. Revision zero is not absence. An empty `components` list is an
epoch-only snapshot.

## Digest

`snapshot_id` is `hex_lower(SHA256(bytes))`, where `bytes` is the UTF-8 text:

```text
v1\n
state_epoch=<decimal unsigned>\n
<kind>=<decimal unsigned revision>\n
...
```

One line per component, kinds sorted ascending byte-wise. No spaces around
`=`. No `creation_time` line. Equal `(state_epoch, components)` give an
identical `snapshot_id` for any `creation_time` and any input order.

Example: epoch 42 with `current_field=7` and `bathymetry_reference=3` hashes
`v1\nstate_epoch=42\nbathymetry_reference=3\ncurrent_field=7\n` and gives
`b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e`.

The digest input is line based and the contract does not restrict kind
characters. Kinds that contain `=` or a newline can collide with other
inputs. Use plain identifier kinds.

## Builder

| Step | Behavior |
| --- | --- |
| Configure | `SetStateEpoch` or `SetStateEpochSource`; `AddComponentRevision` or `AddComponentRevisionSource` per kind; `SetCreationTime` or `SetClock`. |
| Reject | Empty kind, repeated kind, null component source, missing creation time, creation time nanos outside `[0, 1e9)`. Kind defects are found before any source is read. |
| Read | The epoch source, each kind source, and the clock are each read at most once per `Build()`. Kinds are read in sorted order. |
| Normalize | Components are sorted by kind and `snapshot_id` is computed. |
| Return | An owned descriptor. It does not alias the sources, so later source changes do not alter it. |

An unset epoch is zero. Tests pass a fixed creation time.

C++ uses `absl::StatusOr` and returns `InvalidArgument`. Python raises
`WorldSnapshotBuildError`, which carries the `SnapshotError`.

## Validation

`AssessWorldSnapshot` (plain view) and `AssessWorldSnapshotDescriptor`
(proto) report the first structural defect in field order: `snapshot_id`,
then `creation_time`, then component kinds. They do not recompute
`snapshot_id`. An empty message is not a present descriptor. It is not an
error and it is not accepted.

## Out of scope

Skew, age, freshness, and partial-status policy, withholding a snapshot,
component payloads, World service wiring, perception, Gazebo or other
simulator conversion, ICON, and safety wiring.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. Golden bytes for one epoch-and-two-kinds descriptor
are fixed in the C++ and Python serialization tests.
