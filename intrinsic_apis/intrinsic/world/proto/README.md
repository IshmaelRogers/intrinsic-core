# Marine component validity

Opt-in metadata for an additive World component. `MarineComponentValidity`
records who produced an observation, when it was formed, how long it stays
fresh, an optional confidence, and an opaque pointer to uncertainty. It
implements the validity-horizon and confidence sketch in PDR §8 and reuses
the `Validity` companion from
[Stamped header and frame policy](../../embodiment/proto/README.md).

This message is not a bathymetry, current-field, occupancy, or covariance
payload. It does not build a world snapshot and it does not assign a
snapshot id. Existing World component protos are unchanged. Manipulator
joint, Cartesian, kinematics, motion-planning, and Gazebo contracts are
unchanged. `.github/baseline/manipulator_targets.tsv` does not list these
targets. A default `MarineComponentValidity` serializes to zero bytes.

## Message

`marine_component_validity.proto` defines one message.

| Field | Tag | Meaning |
| --- | --- | --- |
| `source_id` | 1 | Stable producer id. Empty is rejected when the message is present. |
| `observation_time` | 2 | `google.protobuf.Timestamp` when the observation was formed. |
| `validity_horizon` | 3 | Non-negative `google.protobuf.Duration` after `observation_time`. |
| `confidence` | 4 | Optional dimensionless value in `[0, 1]`. Unset means absent. |
| `uncertainty_reference` | 5 | Opaque URI, digest, or key. Empty means no reference. |
| `validity` | 6 | Optional `intrinsic_proto.embodiment.Validity`. |

`uncertainty_reference` names an artifact that lives somewhere else. The
covariance values are not a field of this message.

`confidence` uses proto3 `optional` so a supplied `0` stays distinct from
an omitted value.

## Unknown and expired

Embodiment `Validity.State` is `STATE_UNSPECIFIED`, `STATE_VALID`, and
`STATE_INVALID`. This package does not add `STATE_EXPIRED` or
`STATE_DEGRADED`.

| Observation | Host result |
| --- | --- |
| `validity` unset | Unknown. No judgment was supplied. |
| `observation_time` or `validity_horizon` missing | Unknown. Freshness cannot be evaluated. |
| Both times present and usable, `query_time` equal to `observation_time + validity_horizon` | Fresh. |
| Both times present and usable, `query_time` strictly after that deadline | Expired. |

Expired is the host helper's result. The serialized `Validity.state` is left
as the producer wrote it. A component can be `STATE_VALID` and expired, or
`STATE_UNSPECIFIED` and expired. An unset `validity` stays unknown even when
the timestamps are past the deadline.

A negative `validity_horizon`, or nanos outside `[0, 1000000000)`, is
rejected. That sample is not expired. Freshness stays unknown because the
deadline cannot be formed. `observation_time` may fall before the Unix epoch.
Its nanos use the same range. A query at the deadline is fresh. One nanosecond
later is expired.

## Validation order

Host checks are plain values. They do not parse protobuf and they do not
mutate a World entity. The first defect wins:

1. Empty `source_id` on a present message.
2. Present `observation_time` with nanos outside range.
3. Present `validity_horizon` that is negative or has nanos outside range.
4. Present `confidence` that is non-finite or outside `[0, 1]`.

An empty message is not a present component. It is not an error and it is
not accepted. A missing horizon or observation time is unknown, not a
structural defect, and it is not accepted. `uncertainty_reference` is stored
and compared as a string. Helpers do not parse it.

Acceptance requires no defect, `STATE_VALID`, and a fresh assessment.
`STATE_INVALID` remains an explicit rejection. An unknown future `State`
number stays on the wire, is not accepted as valid, and is not rewritten to
invalid or expired.

## Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields on
parse and reserialize. An unknown `Validity` enum number round-trips.
Golden bytes for one populated message are fixed in the C++ and Python
serialization tests. Bytes that omit `validity` are a prefix of that golden.

## Referenced bathymetry

`bathymetry_reference_component.proto` names an external bathymetry asset.
It does not store a height grid, a mesh, occupancy, or contacts.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity_meta` | 1 | `MarineComponentValidity`. Required when the component is present. |
| `frame_id` | 2 | ENU world frame (PDR §5): +X east, +Y north, +Z up. Empty is rejected. |
| `bathymetry_asset_ref` | 3 | Opaque CAS URI, digest, or key. Empty is rejected. Not inline cells. |
| `reference_z_m` | 4 | Optional ENU +Z offset in meters. Unset means no offset. |

`frame_id` is stored as given. This message does not define a NED mode and
does not convert axes. The example uses `world_enu`. A non-empty id is not
rewritten.

`reference_z_m` uses proto3 `optional`. A supplied `0` stays distinct from
an omitted offset. A set value must be finite. Units are meters.

Host checks live in `intrinsic/world/bathymetry_reference_component`. They
call `AssessMarineComponentValidity` for `validity_meta`. They do not encode
a second expiry rule. Unknown, fresh, and expired stay the results from that
helper. Acceptance requires no component defect and an accepted validity
assessment (`STATE_VALID`, fresh, no validity defect).

Check order, first defect wins:

1. Missing `validity_meta`, or a structural defect from the validity helper.
2. Empty `frame_id`.
3. Empty `bathymetry_asset_ref`.
4. Present `reference_z_m` that is not finite.

An empty message serializes to zero bytes and is not an error. The textproto
example is
`intrinsic/world/bathymetry_reference_component/testdata/referenced_bathymetry.textproto`.
It validates at `observation_time + validity_horizon` when `validity.state`
is `STATE_VALID`.

## Constant current

`current_field_component.proto` is a spatially uniform water current. Later
field models append another arm of `representation`. They do not reuse these
tags.

| Field | Tag | Meaning |
| --- | --- | --- |
| `validity_meta` | 1 | `MarineComponentValidity`. Same rules as bathymetry. |
| `frame_id` | 2 | ENU world frame for the velocity. Empty is rejected. |
| `constant` | 3 | `ConstantCurrent` arm of `oneof representation`. |

`ConstantCurrent.velocity_m_s` is `intrinsic_proto.Vector3`. It is linear
water velocity relative to `frame_id`, in meters per second, on ENU axes.
Omitted axes are zero. Every component must be finite when the arm is
present. This message does not convert frames.

Host checks live in `intrinsic/world/current_field_component` and call the
same validity helper. A present component with no `constant` arm is rejected.
Check order: validity, `frame_id`, missing constant arm, non-finite velocity.

An empty message serializes to zero bytes. The textproto example is
`intrinsic/world/current_field_component/testdata/constant_current.textproto`.
It validates under the same fresh `STATE_VALID` rule.

These targets are not listed in `.github/baseline/manipulator_targets.tsv`.
Existing World component protos are unchanged.
