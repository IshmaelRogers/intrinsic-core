# Inference envelope host validator

Opt-in plain-value checks for `InferenceEnvelope` and `InferenceResult`
(`intrinsic_apis/intrinsic/inference/proto`). This package is not wired into
the Triton inference service, does not parse protobuf, and does not call OIP,
World, ICON, or a HAL. There is no queue, client, or replay backend here
(#119 and #120). Message and field rules are in the
[proto README](../../intrinsic_apis/intrinsic/inference/proto/README.md).

## API

- C++, namespace `intrinsic::inference`:
  `AssessInferenceEnvelope(const InferenceEnvelopeView&)` and
  `AssessInferenceResult(const InferenceResultView&)` in
  `inference_envelope_contract_policy.h`.
- Python: `assess_inference_envelope` and `assess_inference_result` in
  `inference_envelope_contract_policy.py`.

Both return an error, the header validity kind, and `accepted`. `accepted` is
true only when the error is `kNone` and the header validity is `STATE_VALID`.
An empty view is not engaged: `kNone`, validity absent, `accepted = false`.

Any signal engages a view: header, OIP ids (including an empty but set OIP
message), non-zero `state_epoch`, snapshot id, deadline, horizon, confidence,
uncertainty, digests, provenance, or metadata. A set zero confidence,
uncertainty, or horizon engages.

## Check order

The first defect wins.

| Order | Check | Error |
| --- | --- | --- |
| 1 | `header.frame_id` non-empty | `kMissingFrame` (1) |
| 2 | `header.source_time` present, nanos in range | `kCreationTime` (2) |
| 3 | `deadline` present, nanos in range, `deadline >= source_time` | `kDeadline` (3) |
| 4 | `validity_horizon` present, `>= 0`, nanos in range | `kValidityHorizon` (4) |
| 5 | OIP `model_name` and `request_id` non-empty | `kOipIdentifier` (5) |
| 6 | provenance present with non-empty `model_id` | `kProvenance` (6) |
| 7 | confidence, if set: finite in `[0, 1]` | `kConfidence` (7) |
| 8 | uncertainty, if set: finite and `>= 0` | `kUncertainty` (8) |
| 9 | `input_digest` non-empty; a result also needs `output_digest` | `kDigest` (9) |
| 10 | `world_snapshot_id`, if non-empty: 64 lowercase hex | `kSnapshotId` (10) |
| 11 | metadata | always tolerated |

## Targets

All targets are opt-in. None is listed in
`.github/baseline/manipulator_targets.tsv`.

| Target | Purpose |
| --- | --- |
| `inference_envelope_contract_policy`, `inference_envelope_contract_policy_py` | Policy libraries. |
| `inference_envelope_contract_test`, `inference_envelope_contract_test_py` | Plain-value unit tests. |
| `inference_envelope_contract_serialization_test`, `inference_envelope_contract_serialization_test_py` | Textproto fixtures, golden bytes, unknown-field round trip. |

```
bazel test //intrinsic_inference/envelope/...
```
