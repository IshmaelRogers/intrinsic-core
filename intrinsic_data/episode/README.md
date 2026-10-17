# Episode event host validator

Opt-in plain-value checks for `EpisodeEvent`
(`intrinsic_apis/intrinsic/episode/proto`). The package layout mirrors
[`intrinsic_inference/envelope`](../../intrinsic_inference/envelope/README.md).
Message and field rules are in the
[proto README](../../intrinsic_apis/intrinsic/episode/proto/README.md).

This is a single-event validator. It does not write, read, or replay a stream,
does not check sequence advance, does not order times, does not convert
frames, and does not re-validate inner payloads. Consumers use the existing
`SafetyDecision`, `VehicleTrajectory`, and Inference policies for those.

## API

- C++, namespace `intrinsic::episode`: `AssessEpisodeEvent(const
  EpisodeEventView&)` and `ComputeConfigDigest(std::string_view)` in
  `episode_event_policy.h`.
- Python: `assess_episode_event` and `compute_config_digest` in
  `episode_event_policy.py`.

The serialization tests fill an `EpisodeEventView` from the proto: header
presence and fields, `schema_version`, both strings, whether `provenance` is
set, `payload_case` (the oneof field number, 0 when unset),
`has_unknown_fields`, and the metadata text when the case is 14.

An assessment has an `error`, a `payload_kind`, `skip`, and `engaged`. An
empty message is not engaged: `kNone`, `engaged = false`, not an error. Header
validity is not judged here. Consumers classify it with
`stamped_header_policy`.

## Check order

The first defect wins. The error numbers are append-only.

| Order | Check | Error |
| --- | --- | --- |
| 1 | `header` set | `kMissingHeader` (1) |
| 2 | `header.source_id` non-empty | `kSourceId` (2) |
| 3 | `header.frame_id` non-empty | `kMissingFrame` (3) |
| 4 | `source_time` set, nanos in range | `kSourceTime` (4) |
| 5 | `receive_time` set, nanos in range | `kReceiveTime` (5) |
| 6 | `schema_version >= 1` | `kSchemaVersion` (6) |
| 7 | `snapshot_id` empty or 64 lowercase hex | `kSnapshotId` (7) |
| 8 | `config_digest` empty or 64 lowercase hex | `kConfigDigest` (8) |
| 9 | case 14: `config_digest == SHA256Hex(text)`, both empty allowed | `kConfigDigestMismatch` (9) |
| 10 | case 10 to 14 known. Case not set with unknown fields is skippable (`kNone`, `payload_kind = kUnknown`, `skip = true`). Otherwise not set. | `kMissingPayload` (10) |

A defective event is never skipped, even when its payload is unknown.

## Known limitations

- Single event only. There is no cross-event check: sequence advance, or the
  same `config_digest` across an episode. That is #128.
- The digest covers only the `FormatMarineMetadata` text. It does not cover
  SDF, plugin configuration, scenario, or code version.
- Python upb does not implement `UnknownFields()`, so the Python test builds
  `has_unknown_fields` with a top-level wire scan. C++ uses the
  `UnknownFieldSet`. A future Python protobuf release may change this.
- The C++ and Python views count only top-level unknown fields. Unknown
  fields inside a payload message are preserved and never inspected here.
- Unknown-payload skipping relies on the payload oneof being unset. An event
  that carries both a known payload and an unknown payload field is treated as
  its known payload.

## Fixtures and tests

`testdata/` is committed and never rewritten by tests.

| Fixture | Content |
| --- | --- |
| `episode_event_v1_<payload>.binpb` and `.textproto` | One event per payload case, built from the #91, #101, #118 examples and the #123 seed-42 text. |
| `episode_event_v2_additive_unknown_payload.binpb` | The v1 metadata event envelope without its payload, plus an unknown length-delimited payload field at tag 536870000 and an unknown varint envelope field at tag 536870001. |
| `episode_event_schema_freeze.txt` | `<message>.<field> <tag> <type> <label>` for every field, sorted. |

| Test | Covers |
| --- | --- |
| E1 | Round trip per payload: parse the golden, `kNone`, re-serialize byte-equal. C++ and Python assert the same bytes. |
| E2 | Header fields, `schema_version`, `snapshot_id`, `config_digest`, and `provenance` survive a round trip. |
| E3 | Empty message is not engaged. A single event is `kNone`. |
| E4 | One mutation per error row 1 to 10, exact error. Uppercase, 63, 65, and non-hex fail rows 7 and 8. |
| E5 | Truncated bytes report a parse failure and do not crash. |
| E6 | The v2 golden through the v1 reader: parses, `skip = true`, `kNone`, bytes preserved. |
| E7 | v1 goldens parse identically from text and bytes. `schema_version = 2` on a v1 payload is `kNone`. |
| E8 | SHA-256 of the seed-42 text equals the pinned hex in both languages. A one-byte change is `kConfigDigestMismatch`. |
| E9 | The descriptor equals `episode_event_schema_freeze.txt`. |

## Targets

All targets are opt-in. None is listed in
`.github/baseline/manipulator_targets.tsv`.

| Target | Purpose |
| --- | --- |
| `episode_event_policy`, `episode_event_policy_py` | Policy libraries. |
| `episode_event_policy_test`, `episode_event_policy_test_py` | Plain-value unit tests. |
| `episode_event_serialization_test`, `episode_event_serialization_test_py` | E1 to E8 against the committed goldens. |
| `episode_event_schema_freeze_test`, `episode_event_schema_freeze_test_py` | E9. |

```
bazel test //intrinsic_data/episode/...
```
