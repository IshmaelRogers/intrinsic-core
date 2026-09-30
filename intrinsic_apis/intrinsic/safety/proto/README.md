# Safety decision contract

Opt-in non-real-time protobuf contract for the committed output of a safety
filter: one `SafetyDecision` with zero or more `SafetyFinding` entries. It is
the first leaf of the Phase 3 safety representation in
[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md).

This package records what a filter decided. It does not evaluate any safety
rule, does not define an authority or mode state machine, and does not call
ICON, a HAL, Gazebo, or perception. It is not the ICON `safety_status`
message: ICON industrial safety stays separate and is not touched here.

Manipulator joint, Cartesian, kinematics, motion-planning, World, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes. Callers that
never set these messages keep the previous manipulator behavior.

Host checks live in `intrinsic/safety` (`safety_decision_policy.h`,
`safety_decision_policy.py`, `safety_decision_assessor.h`,
`safety_decision_assessor.py`). The policy files work on plain values and do
not parse protobuf. The assessor files map a `SafetyDecision` onto those
values.

## Messages

| Message | Role |
| --- | --- |
| `SafetyDecision` | One committed filter outcome for one evaluated intent. |
| `SafetyFinding` | One rule observation: rule id, severity, text. |
| `SafetyDecisionKind` | ACCEPT, PROJECT, REJECT, ABORT, SURFACE. |
| `SafetyFindingSeverity` | INFO, WARNING, ERROR, CRITICAL. |

### `SafetyDecisionKind` (numbers are locked)

| Value | Number | Meaning |
| --- | --- | --- |
| `SAFETY_DECISION_KIND_UNSPECIFIED` | 0 | Not used on a committed decision. |
| `SAFETY_DECISION_KIND_ACCEPT` | 1 | Pass the original intent through unchanged. |
| `SAFETY_DECISION_KIND_PROJECT` | 2 | Replace the intent with `applied_intent`. |
| `SAFETY_DECISION_KIND_REJECT` | 3 | Refuse the intent. No applied motion. |
| `SAFETY_DECISION_KIND_ABORT` | 4 | Hard stop. Abandon the active intent. |
| `SAFETY_DECISION_KIND_SURFACE` | 5 | Marine fail-closed: command a surface or safe-ascent intent. |

### `SafetyFindingSeverity` (numbers are locked)

| Value | Number |
| --- | --- |
| `SAFETY_FINDING_SEVERITY_UNSPECIFIED` | 0 |
| `SAFETY_FINDING_SEVERITY_INFO` | 1 |
| `SAFETY_FINDING_SEVERITY_WARNING` | 2 |
| `SAFETY_FINDING_SEVERITY_ERROR` | 3 |
| `SAFETY_FINDING_SEVERITY_CRITICAL` | 4 |

### `SafetyFinding` (tags are append-only)

| Tag | Field | Rule |
| --- | --- | --- |
| 1 | `string rule_id` | Non-empty when the finding is present in a validated decision. |
| 2 | `SafetyFindingSeverity severity` | Unknown numbers are kept. |
| 3 | `string summary` | Human-readable. May be empty. |
| 4 | `optional string detail` | Optional longer text. |

### `SafetyDecision` (tags are append-only)

| Tag | Field | Rule |
| --- | --- | --- |
| 1 | `StampedHeader header` | Decision time and source. Required when engaged. |
| 2 | `SafetyDecisionKind kind` | See the kind table. |
| 3 | `string original_intent_digest` | Lowercase hex SHA-256, 64 characters. Required when engaged. |
| 4 | `DesiredMotion applied_intent` | Presence rules below. |
| 5 | `string snapshot_id` | 64 lowercase hex characters. Required when engaged. |
| 6 | `repeated SafetyFinding findings` | Zero or more. Duplicate `rule_id` is kept. |
| 7 | `optional uint64 decision_epoch` | Monotonic filter epoch. Unset is observed as 0. |

An empty `SafetyDecision` serializes to zero bytes and is not an engaged
decision. The same empty-versus-engaged pattern applies to `DesiredMotion`.
`decision_epoch` is `optional`, so an explicit zero is written to the wire and
makes the message engaged. Host checks then report the missing header.

## Identity and reuse

- `applied_intent` reuses `intrinsic_proto.vehicle.DesiredMotion`. No new
  intent message is defined.
- `snapshot_id` is exactly the `WorldSnapshotDescriptor.snapshot_id` string
  from `intrinsic/world/proto/world_snapshot_descriptor.proto`: 64 lowercase
  hex characters. The descriptor and any World component payload are not
  embedded, and this package does not import the World protos.
- `original_intent_digest` is the lowercase hex SHA-256 of the serialized
  original `DesiredMotion` that the filter evaluated. The host helper
  `ComputeOriginalIntentDigest` (C++) and `compute_original_intent_digest`
  (Python) serialize the message with no map fields, so the bytes are the
  canonical proto3 wire form. Whether a digest matches a given original is a
  producer and consumer concern. The decision does not carry the original.

## Applied intent

| Kind | `applied_intent` |
| --- | --- |
| ACCEPT | Must be absent. Accept means the original is unchanged. Do not echo a copy. |
| PROJECT | Must be present. |
| REJECT | Must be absent. |
| ABORT | Must be absent. |
| SURFACE | Must be present. It carries the surface or ascent intent. |

Presence is message presence. A present but empty `DesiredMotion` counts as
present. For PROJECT and SURFACE it then fails the delegated assessment.

When present and required, `applied_intent` must be accepted by the vehicle
host helper `AssessDesiredMotion` (`intrinsic/vehicle`). That includes
`STATE_VALID` on its own header and the NaN and infinity checks.

## Unknown enum numbers

Proto3 enums are open. Unknown `kind` and `severity` numbers survive parse
and serialize unchanged.

- Host classifiers map an unknown `kind` to `kUnknown` and an unknown
  severity to `kUnknown`.
- An unknown `kind` is not a structural defect. It is not accepted, so a
  consumer fails closed for actuation. The host never rewrites it to REJECT,
  ABORT, or any other kind, and never changes the message.
- The applied-intent presence rules do not apply to an unknown kind, because
  the meaning of that kind is not known. Other structural defects are still
  reported.
- An unknown severity is never a defect.

## Host assessment

`AssessSafetyDecision` returns `engaged`, the first `error`, the classified
`kind`, the header validity, and `accepted`. The first defect wins, in this
order:

1. Empty or unengaged: not engaged, no error, not accepted.
2. Header: missing header, a stamp `nanos` outside `[0, 1000000000)` on
   `source_time` or `receive_time`, or an explicit `STATE_INVALID`.
3. `kind` is `UNSPECIFIED`.
4. Unknown `kind`: classified as unknown, not an error, not accepted.
5. `original_intent_digest` does not match `^[0-9a-f]{64}$`.
6. `snapshot_id` does not match `^[0-9a-f]{64}$`.
7. Applied-intent presence per kind, then the `DesiredMotion` assessment.
8. A finding with an empty `rule_id`. The error reports the first index.
9. NaN and infinity in the nested `DesiredMotion` are covered by step 7.

`accepted` is true only when the decision is engaged, no structural defect
was found, the header is `STATE_VALID`, and the kind is one of ACCEPT,
PROJECT, REJECT, ABORT, or SURFACE. Header validity follows the embodiment
rules: an absent, unspecified, or unknown-number validity is not accepted and
is not rewritten to invalid.

`accepted` is structural. An accepted REJECT or ABORT means the message is
well formed, not that any motion is allowed. No rule is evaluated here.

## Out of scope

- Authority, shadow, recommend, constrained, revoked, and emergency modes,
  and their transition table.
- Concrete safety rules and filters.
- ICON `safety_status`, HAL, actuators, Gazebo, perception, and World
  entities.
- Embedding World component payloads or a full `WorldSnapshotDescriptor`.

## Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields and unknown enum numbers. Golden bytes for one populated
`SafetyDecision` are fixed in the C++ and Python serialization tests.
Clearing `decision_epoch` leaves a prefix of that golden. Field 100 is
preserved.

## Examples

Text format examples:

- [`examples/accept.textproto`](examples/accept.textproto)
- [`examples/project.textproto`](examples/project.textproto)
- [`examples/reject.textproto`](examples/reject.textproto)
- [`examples/abort.textproto`](examples/abort.textproto)
- [`examples/surface.textproto`](examples/surface.textproto)

## Targets

These targets are separate from the protected manipulator baseline:

- `@intrinsic_apis//intrinsic/safety/proto:safety_decision_proto`
- `@intrinsic_apis//intrinsic/safety/proto:safety_decision_cc_proto`
- `@intrinsic_apis//intrinsic/safety/proto:safety_decision_py_pb2`
- `@intrinsic_apis//intrinsic/safety/proto:safety_decision_go_proto`
- `@intrinsic_apis//intrinsic/safety/proto:examples`
- `//intrinsic/safety:safety_decision_policy`
- `//intrinsic/safety:safety_decision_policy_py`
- `//intrinsic/safety:safety_decision_assessor`
- `//intrinsic/safety:safety_decision_assessor_py`
- `//intrinsic/safety:safety_decision_policy_test`
- `//intrinsic/safety:safety_decision_policy_test_py`
- `//intrinsic/safety:safety_decision_assessor_test`
- `//intrinsic/safety:safety_decision_assessor_test_py`
- `//intrinsic/safety:safety_decision_serialization_test`
- `//intrinsic/safety:safety_decision_serialization_test_py`
