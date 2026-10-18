# Physical AI episode event envelope

Opt-in, non-real-time protobuf contract for one recorded episode event: an
append-only envelope plus a typed payload union. It carries the existing
`SafetyDecision`, `VehicleTrajectory`, `InferenceEnvelope`, and
`InferenceResult` messages unchanged, and the realized randomization text of
the marine parameter sampler. It follows
[ADR 0001](../../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
(`intrinsic_data` owns the episode schema).

Package: `intrinsic_proto.episode`. Imports `StampedHeader`, `ModelProvenance`,
`SafetyDecision`, `VehicleTrajectory`, `InferenceEnvelope`, `InferenceResult`,
`DesiredMotion`, `WorldSnapshotDescriptor`, `Timestamp`, and `Duration`. No
other proto imports `episode_event.proto`.

Lock id: `episode-event-envelope-v1`. This is **serialization and validation
only**. There is no reader, replay clock, digest equality, or digest over
state, plan, or commands (#129 to #132). The bounded writer, its file format,
sequence ordering, overflow handling and cancellation (#128) live in
[`intrinsic_data/episode`](../../../../intrinsic_data/episode/README.md). The
writer needed one additive payload, `EpisodeLoss` at tag 15, which amends the
lock as described under Messages.

## Messages

Every tag, type, and name below is **permanent**. Evolution is append-only. A
change after merge is breaking and needs senior review.
`episode_event_schema_freeze_test` compares the generated descriptor with
`intrinsic_data/episode/testdata/episode_event_schema_freeze.txt`, so any
tag, type, name, or oneof change fails CI.

| Message | Fields |
| --- | --- |
| `EpisodeMetadata` | `marine_randomization_metadata` 1 (`string`) |
| `EpisodeLoss` | `dropped_count` 1 (`uint64`), `first_dropped_writer_index` 2 (`uint64`), `last_dropped_writer_index` 3 (`uint64`), `reason` 4 (`LossReason`) |
| `EpisodeEvent` | `header` 1 (`StampedHeader`), `schema_version` 2 (`uint32`), `snapshot_id` 3 (`string`), `config_digest` 4 (`string`), `provenance` 5 (`ModelProvenance`) |
| `RecordedMessage` | `message_type` 1 (`string`), `message_bytes` 2 (`bytes`) |
| `RecordedComponentTiming` | `component_kind` 1 (`string`), `observation_time` 2 (`Timestamp`), `validity_horizon` 3 (`Duration`), `source_id` 4 (`string`) |
| `WorldSnapshotRecord` | `snapshot` 1 (`WorldSnapshotDescriptor`), `timings` 2 (repeated `RecordedComponentTiming`) |
| `EpisodeEvent.payload` oneof | `safety_decision` 10, `vehicle_trajectory` 11, `inference_envelope` 12, `inference_result` 13, `episode_metadata` 14, `episode_loss` 15, `recorded_message` 16, `world_snapshot_record` 17, `desired_motion` 18 |

The payload type is the oneof case. There is no separate enum, so the type and
the payload can never disagree. Every oneof member is a message type, so a
future member can be added without changing existing wire bytes. There is no
`Any` and no map field, so serialization order is deterministic.

## Tag plan

Except for the allocated tags 15 to 18 below, this is documentation only:
nothing here is declared in the `.proto`, and nothing is `reserved`:
`reserved` forbids future use, and the repository policy is to use it only for
removed fields.

| Range | Plan |
| --- | --- |
| Envelope tags 6 to 9 | The next envelope fields. |
| Payload tag 15 | **Allocated** to `episode_loss` (`EpisodeLoss`), #128, approved 2026-10-17. |
| Payload tags 16 to 18 | **Allocated** to #244 (`episode-replay-payloads-v1`, approved 2026-10-18): `recorded_message` 16, `world_snapshot_record` 17, `desired_motion` 18. |
| Payload tags 19 to 99 | Future payload categories. Candidates, not commitments: marine HAL measurements (#73 to #80), thruster commands, episode start and end lifecycle markers. |

### EpisodeLoss

`EpisodeLoss` is written only by the episode writer
([`intrinsic_data/episode`](../../../../intrinsic_data/episode/README.md)). It
records one contiguous range of dropped events: `dropped_count` equals
`last_dropped_writer_index - first_dropped_writer_index + 1`, both indices are
inclusive, and `reason` is `LOSS_REASON_OVERFLOW` (1, the writer had no room)
or `LOSS_REASON_TOO_LARGE` (2, the serialized event exceeded the size limit).
The enum is nested in the message and `LOSS_REASON_UNSPECIFIED` (0) is not
written. `schema_version` stays 1. This was an explicit, one-time, additive
exception to the #128 scope, and it is the only schema change #128 makes.

A reader built before tag 15 existed sees the payload as unknown fields, so
the record is skippable and its bytes are kept (see Unknown payloads). Producers
cannot send tag 15: the writer rejects it as reserved.

### Replay payloads (#244)

Lock id `episode-replay-payloads-v1`. Three additive payloads let the replay
adapters (#131, #132) read recorded estimator measurements, World snapshots and
intent from an episode. `schema_version` stays 1. A reader built before #244
sees them as unknown payloads: skippable, bytes kept (see Unknown payloads).

- `recorded_message` (16): `RecordedMessage` is an episode-local opaque
  wrapper. `message_type` is the bare fully qualified proto name, for example
  `intrinsic_proto.estimation.MeasurementEnvelope`, with no
  `type.googleapis.com/` prefix, as in `MeasurementEnvelope.payload_type`.
  `message_bytes` are the serialized message, kept exact. There is no `Any`.
  It carries messages that `@intrinsic_apis` cannot import. The adapter parses
  and validates the bytes.
- `world_snapshot_record` (17): `WorldSnapshotRecord` is the
  `WorldSnapshotDescriptor` as served plus one `RecordedComponentTiming` per
  component kind. `RecordedComponentTiming` matches
  `world_snapshot::ComponentTiming` field for field. There is no query-time
  field: the assessment query time is the event's `header.receive_time`. The
  field is named `snapshot`, not `descriptor`, because `descriptor` collides
  with the generated C++ `Message::descriptor()` accessor.
- `desired_motion` (18): `intrinsic_proto.vehicle.DesiredMotion`, recorded
  verbatim. Its inner `header` is not cross-checked against the envelope
  header.

No new envelope field, enum, or `reserved` statement is added, and
`AssessEpisodeEvent` does no inner validation of these payloads.

## Rules

| Field | Rule |
| --- | --- |
| `header` | Set, with non-empty `source_id` and `frame_id`, and `source_time` and `receive_time` set with nanos in range. |
| `schema_version` | Writers set 1. Readers accept any value `>= 1`: newer versions are additive. Zero is unset. |
| `snapshot_id` | Empty, or exactly 64 lowercase hex characters. Same rule as `InferenceEnvelope.world_snapshot_id` and the `world_snapshot_policy` digest. The envelope does not recompute it. |
| `config_digest` | Empty, or exactly 64 lowercase hex characters. See below. |
| `provenance` | Optional. Unset when not applicable, and not validated here. |
| payload | One of the nine cases (`episode_loss` is writer-only), or unknown fields only (see Unknown payloads). |

### Config digest

`config_digest` is the lowercase hex SHA-256 over exactly the bytes of
`FormatMarineMetadata(realized)`. Those are the bytes carried in
`EpisodeMetadata.marine_randomization_metadata`, with no prefix, suffix, or
normalization. For a `episode_metadata` payload the validator recomputes the
digest and rejects a mismatch. An empty text goes with an empty digest, and a
non-empty text needs the matching digest.

Pinned: the seed-42 golden
(`intrinsic/simulation/randomization/testdata/marine_parameter_sampler_seed_42.golden.txt`,
1530 bytes) hashes to
`93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a`.

**Scope limit.** The digest covers only the realized sampler parameters. It
does **not** cover the SDF, plugin configuration, scenario, or code version.
Widening it is a later senior decision (ADR and PDR update). For events with
any other payload the validator only checks that `config_digest` is empty or
well formed. It does not check that two events of one episode agree.

### Unknown payloads

A reader that does not know a payload case sees the oneof as not set and the
bytes as unknown fields. A structurally valid event in that state is
**skippable**: the validator returns no error, payload kind unknown, and
`skip = true`. Re-serialization preserves the unknown bytes. C++ keeps them in
the `UnknownFieldSet`. Python upb keeps them on the wire, but its
`UnknownFields()` accessor is not implemented, so the Python tests detect them
by scanning the serialized top-level fields.

An unset payload with no unknown fields is `kMissingPayload`. An empty message
(zero bytes) is not engaged: absent, not an error.

## Host validator

The validator lives in
[`intrinsic_data/episode`](../../../../intrinsic_data/episode/README.md). It
takes plain values and does not parse protobuf.

## Examples

`examples/` holds text-format fixtures. They are byte-identical to the
`.textproto` files in `intrinsic_data/episode/testdata/`, and the paired tests
assert that. The `.binpb` goldens are committed and never rewritten by tests.

| Fixture | Payload |
| --- | --- |
| `episode_event_v1_safety_decision.textproto` | `SafetyDecision` ACCEPT example (#91). |
| `episode_event_v1_vehicle_trajectory.textproto` | Two-sample `VehicleTrajectory` example (#101). |
| `episode_event_v1_inference_envelope.textproto` | Nominal `InferenceEnvelope` example (#118). |
| `episode_event_v1_inference_result.textproto` | Nominal `InferenceResult` example (#118). |
| `episode_event_v1_metadata.textproto` | `EpisodeMetadata` with the seed-42 text verbatim and the pinned digest. |
| `episode_event_v1_recorded_message.textproto` | `RecordedMessage` wrapping a fixed hand-encoded `MeasurementEnvelope` (#244). |
| `episode_event_v1_world_snapshot_record.textproto` | `WorldSnapshotRecord` with a two-component descriptor and matching timings (#244). |
| `episode_event_v1_desired_motion.textproto` | `DesiredMotion` with a twist objective and a horizon (#244). |

## Protected surface

`episode/` is not in the manipulator schema inventory allowlist. The manipulator
public schema inventory JSON and `.github/baseline/manipulator_targets.tsv` are
unchanged. The freeze test guards permanence for this proto instead.
