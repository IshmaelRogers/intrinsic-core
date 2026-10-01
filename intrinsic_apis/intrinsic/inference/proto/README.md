# Physical AI inference envelope

Opt-in, non-real-time protobuf contract that wraps an existing Open
Inference Protocol (OIP) request or result with the context Physical AI
needs: epoch, World snapshot, deadline, validity horizon, provenance,
confidence, uncertainty, and an input digest. It implements the "Inference
envelope" rule of PDR §9 and follows
[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md).

Package: `intrinsic_proto.inference`. Imports `google.protobuf.Duration`,
`google.protobuf.Timestamp`, `StampedHeader`
(`intrinsic/embodiment/proto/stamped_header.proto`), and `ModelProvenance`
(`intrinsic/embodiment/proto/model_provenance.proto`).

The envelope wraps OIP **identifiers only**. It does not import or embed
`ModelInferRequest`, `ModelInferResponse`, tensors, or raw contents, and it
does not change the OIP payload or wire, `ml_model.proto`,
`inference_service_config.proto`, or the Triton service.

## Out of scope

- `InferenceClient` Submit, Poll, Cancel, the async queue, and the replay
  backend. Those are #119 and #120.
- Expiry consumption. Expired results are logged and ignored later (PDR §11).
  This contract rejects structurally invalid envelopes only.
- ICON, HAL FlatBuffers, Gazebo, real hardware, and the `DesiredMotion`
  actuator path.
- Robot-type enums and covariance-matrix uncertainty. Uncertainty is one
  scalar.
- Changes to manipulator joint, Cartesian, kinematics, motion-planning, or
  World contracts. `.github/baseline/manipulator_targets.tsv` does not list
  these targets.

## Messages

`inference_envelope.proto` defines five messages. Field numbers and names are
locked. Evolution is append-only: add fields, and reserve removed tags and
names.

| Message | Fields |
| --- | --- |
| `OipModelIdentifier` | `model_name` 1, `model_version` 2 |
| `OipRequestIdentifier` | `model` 1, `request_id` 2 |
| `OipResultIdentifier` | `model` 1, `request_id` 2 (echo of the request id) |
| `InferenceEnvelope` | `header` 1, `oip_request` 2, `state_epoch` 3, `world_snapshot_id` 4, `deadline` 5, `validity_horizon` 6, `confidence` 7, `uncertainty` 8, `input_digest` 9, `provenance` 10, `metadata` 100 |
| `InferenceResult` | `header` 1, `oip_result` 2, `state_epoch` 3, `world_snapshot_id` 4, `deadline` 5, `validity_horizon` 6, `confidence` 7, `uncertainty` 8, `input_digest` 9, `output_digest` 10, `provenance` 11, `metadata` 100 |

`confidence` and `uncertainty` are `optional double`. Unset is not zero.

## Rules

| Field | Rule when the message is engaged |
| --- | --- |
| `header.frame_id` | Non-empty. |
| `header.source_time` | Present, nanos in range. This is the creation time. |
| `deadline` | Present, nanos in range, and not before `header.source_time`. Equal is allowed. |
| `validity_horizon` | Present, seconds `>= 0`, nanos in range. |
| OIP ids | `model_name` and `request_id` non-empty. `model_version` may be empty. |
| `provenance` | Present, `model_id` non-empty. |
| `confidence` | If set: finite and in `[0, 1]`. |
| `uncertainty` | If set: finite and `>= 0`. |
| `input_digest` | Non-empty. A result also needs `output_digest`. |
| `world_snapshot_id` | Empty is allowed. Otherwise exactly 64 lowercase hex characters (`WorldSnapshotDescriptor.snapshot_id`). |
| `state_epoch` | Any value. Zero is allowed. It is the estimator epoch (`VehicleState.estimator_epoch`). |
| `metadata` | Always tolerated and never inspected. |

An empty message is not engaged. Unknown protobuf fields round-trip and are
not a defect.

## Host validator

The validator lives in
[`intrinsic_inference/envelope`](../../../../intrinsic_inference/envelope/README.md).
It takes plain values and does not parse protobuf. A default message
serializes to zero bytes.

## Examples

`examples/` holds text-format fixtures that the paired C++ and Python tests
parse and assess.

| Fixture | Result |
| --- | --- |
| `inference_envelope_nominal.textproto` | Accepted. Its wire bytes are the golden in the serialization tests. |
| `inference_result_nominal.textproto` | Accepted. |
| `inference_envelope_deadline_equal_creation.textproto` | Accepted. `deadline == header.source_time`. |
| `inference_envelope_deadline_before_creation.textproto` | `kDeadline` |
| `inference_envelope_bad_confidence.textproto` | `kConfidence` |
| `inference_envelope_bad_uncertainty.textproto` | `kUncertainty` |
| `inference_envelope_bad_snapshot_id.textproto` | `kSnapshotId` |
| `inference_envelope_empty_oip_request_id.textproto` | `kOipIdentifier` |

Every invalid fixture keeps `STATE_VALID` on the stamp to show that validity
does not repair a structural defect.
