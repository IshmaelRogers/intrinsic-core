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
well-formed snapshot. It does not call the marine validity helpers. Skew and
age are a separate, opt-in assessment (see Skew policy). It is not
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

## Skew policy

Opt-in and separate from the descriptor. `AssessWorldSnapshotSkew` (C++,
plain view), `AssessWorldSnapshotDescriptorSkew` (C++, proto), and
`assess_world_snapshot_skew` / `assess_world_snapshot_descriptor_skew`
(Python) are pure functions of four latched inputs: the descriptor, per-kind
timings, `query_time`, and a `WorldSnapshotSkewPolicy`. They do not read the
World, so a later source change cannot alter an assessment. Reassess a new
descriptor and new timings to see new values. The descriptor, its fields, and
its digest are unchanged. Files: `world_snapshot_skew_policy.{h,py}` and
`world_snapshot_skew_assessor.{h,cc,py}`.

| Input | Meaning |
| --- | --- |
| Timings | One entry per kind: `observation_time`, `validity_horizon`, optional `source_id` (diagnostics). Empty or repeated kinds are rejected. |
| `query_time` | Assessment clock. Tests pass a fixed time. It need not equal `creation_time`. Nanos in `[0, 1e9)`. |
| `max_skew` | Non-negative. Default 200 ms. Skew equal to it is within the limit. |
| `max_age` | Non-negative. Default 2 s. Bounds `query_time - observation_time` per kind. Age equal to it is within the limit. |
| `required_kinds` | Must be in the descriptor. Empty means none required. |
| `optional_kinds` | May be absent. A kind in both lists is rejected. |

Freshness reuses the marine validity boundary: a query at
`observation_time + validity_horizon` is fresh and the next instant is
expired. `max_age` uses the same comparison. A required or optional kind that
is in the descriptor is stale if its horizon or `max_age` has passed, or if it
has no timing or an unusable timing. A negative age (observation after `query_time`) is not stale.

The skew set is every kind that is in the descriptor, is listed in
`required_kinds` or `optional_kinds`, and has a usable timing.
`measured_skew` is `max(observation_time) - min(observation_time)` over it,
with `measured_skew_present` false for fewer than two members. Kinds that are
neither required nor optional, and timings for kinds absent from the
descriptor, are ignored.

`WorldSnapshotPolicyAssessment` (alias `SnapshotSkewAssessment`):

| `status` | Value | When | `accepted` | `withhold` |
| --- | --- | --- | --- | --- |
| `UNSPECIFIED` | 0 | Not used when the policy ran. | no | see below |
| `FRESH` | 1 | All required kinds present and fresh, skew within `max_skew`, no optional gap. | yes | no |
| `PARTIAL` | 2 | As `FRESH`, but at least one optional kind is absent. | yes | no |
| `STALE` | 3 | A required or optional kind in the descriptor is expired, too old, or untimed. | no | yes |
| `EXCESSIVE_SKEW` | 4 | `measured_skew` is strictly greater than `max_skew`. | no | yes |
| `INCOMPLETE` | 5 | A required kind is absent. Not the same as `PARTIAL`. | no | yes |

`withhold` means do not publish the snapshot to safety or planning
consumers. It is set for every result that is not accepted.

Precedence, first decisive wins: structural defect, `INCOMPLETE`, `STALE`,
`EXCESSIVE_SKEW`, `PARTIAL`, `FRESH`. A structural defect is an absent or
malformed descriptor (the existing `AssessWorldSnapshot` result is carried in
`descriptor_error`), an invalid policy, invalid timings, or an invalid
`query_time`. It sets `error`, leaves `status` `UNSPECIFIED`, and the policy
does not run.

Diagnostics: `offending_kinds` is sorted and unique. It lists the missing
required kinds for `INCOMPLETE`, every stale kind for `STALE`, and the
earliest and latest kinds for `EXCESSIVE_SKEW`. It is empty for `FRESH` and
`PARTIAL`. `missing_optional_kinds` lists absent optional kinds.
`earliest_kind` and `latest_kind` give the minimum and maximum observation
kinds, with ties going to the smallest kind.

Nothing here implements a SafetyRule or wires the World service. A safety
consumer can call the validator later.

## Out of scope

Component payloads, World service wiring, perception, Gazebo or other
simulator conversion, ICON, and safety wiring.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. Golden bytes for one epoch-and-two-kinds descriptor
are fixed in the C++ and Python serialization tests.
