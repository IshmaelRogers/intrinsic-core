# Episode event host validator

Opt-in plain-value checks for `EpisodeEvent`
(`intrinsic_apis/intrinsic/episode/proto`). The package layout mirrors
[`intrinsic_inference/envelope`](../../intrinsic_inference/envelope/README.md).
Message and field rules are in the
[proto README](../../intrinsic_apis/intrinsic/episode/proto/README.md).

The validator (`episode_event_policy`) checks a single event. It does not
check sequence advance, does not order times, does not convert frames, and does
not re-validate inner payloads. Consumers use the existing `SafetyDecision`,
`VehicleTrajectory`, and Inference policies for those. The bounded writer
(`episode_writer`, see [Bounded episode writer](#bounded-episode-writer))
writes a stream and the episode reader (`episode_reader`, see
[Episode reader](#episode-reader)) reads it back and detects corruption. The
replay clock (`episode_replay`, see [Replay clock](#replay-clock)) hands the
events back in file order with deterministic timing. There is still no
subsystem or replay adapter and no digest equality (#131, #132).

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
| 10 | case 10 to 18 known. Case not set with an unknown payload candidate is skippable (`kNone`, `payload_kind = kUnknown`, `skip = true`). A candidate is a top-level unknown length-delimited field (wire type 2) in the future-payload tag range below. Otherwise not set, including an event whose only unknown fields are envelope fields or wrong-wire occurrences. | `kMissingPayload` (10) |

A defective event is never skipped, even when its payload is unknown.

## Tag allocation (#240)

Lock id `episode-unknown-payload-classification-v1`. The allocation is
append-only. The policy helper `IsUnknownPayloadCandidate(field_number,
wire_type)` (C++) and `is_unknown_payload_candidate(field_number, wire_type)`
(Python) is true only for `wire_type == 2` with a number in the future-payload
range. Groups (wire types 3 and 4) and every other wire type are false. The
view builders set `unknown_payload_candidate` when any top-level unknown field
passes it. `has_unknown_fields` is kept for engagement and preservation only.

| Tag numbers | Use | Constant |
| --- | --- | --- |
| 1 to 9 | Envelope. 1 to 5 are allocated, 6 to 9 are unallocated. | |
| 10 to 18 | Known payloads. | `kPayloadCaseSafetyDecision` to `kPayloadCaseDesiredMotion` |
| 19 to 536870911, except 19000 to 19999 | Future-payload candidates (length-delimited only). | `kFirstFuturePayloadTag` = 19, `kMaxFieldNumber` = 536870911 |
| 19000 to 19999 | Excluded. The protobuf implementation reserves these numbers. | `kExcludedFieldNumberFirst`, `kExcludedFieldNumberLast` |

A correctly parsed known payload stays known, even next to wrong-wire
occurrences of a known tag or unknown candidates. No malformed-wire rejection
was added.

## Known limitations

- The validator checks a single event. Sequence advance is checked by the
  writer, per `source_id`. There is no check that the same `config_digest`
  holds across an episode.
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

## Judgement calls

- `config_digest` covers only the `FormatMarineMetadata` text.
- Empty `snapshot_id` and `config_digest` mean absent, not an error.
- Empty-digest rule (row 9, payload 14 only): empty metadata text requires an
  empty digest, and non-empty text requires the matching digest. An empty
  digest with non-empty text is `kConfigDigestMismatch`, and a non-empty digest
  with empty text is `kConfigDigestMismatch`. For other payloads an empty
  digest is always accepted.
- `schema_version >= 1` is accepted. Newer versions are additive.
- Envelope `provenance` is optional and not validated.
- The v2 golden is the v1 metadata envelope without payload 14, plus unknown
  fields at tags 536870000 and 536870001, so the payload is unknown and
  skippable.
- The Python digest hashes UTF-8, which equals ASCII for the sampler output.
- The freeze file also records presence and oneof membership.

## Fixtures and tests

`testdata/` is committed and never rewritten by tests.

| Fixture | Content |
| --- | --- |
| `episode_event_v1_<payload>.binpb` and `.textproto` | One event per payload case, built from the #91, #101, #118 examples and the #123 seed-42 text. |
| `episode_event_v1_recorded_message.binpb` and `.textproto` | #244. `RecordedMessage` (case 16) carrying the fixed 113-byte `MeasurementEnvelope` listed in [Replay payload fixtures](#replay-payload-fixtures). Sequence 6. |
| `episode_event_v1_world_snapshot_record.binpb` and `.textproto` | #244. `WorldSnapshotRecord` (case 17): a two-component descriptor (`bathymetry_reference` revision 3, `current_field` revision 5) whose `snapshot_id` equals the envelope's, and one timing per component. Sequence 7. |
| `episode_event_v1_desired_motion.binpb` and `.textproto` | #244. `DesiredMotion` (case 18) with a body `twist` objective and a 2 s `horizon`. Sequence 8. |
| `episode_event_v2_additive_unknown_payload.binpb` | The v1 metadata event envelope without its payload, plus an unknown length-delimited payload field at tag 536870000 and an unknown varint envelope field at tag 536870001. |
| `episode_writer_v1_nominal.iepisod` | The five v1 goldens written in sequence order 1 to 5 by one source, then `Close`. 4069 bytes. |
| `episode_writer_v1_overflow.iepisod` | The same five events with `capacity = 4`: three records, then the `OVERFLOW` loss record. 1607 bytes. |
| `episode_reader_v1_*.iepisod` | Thirteen corruption and edge fixtures for the reader, see [Reader fixtures](#reader-fixtures). |
| `episode_event_schema_freeze.txt` | `<message>.<field> <tag> <type> <label>` for every field, sorted. The type is the scalar name or `message:<full name>`. The label is `implicit`, `explicit`, `oneof:<name>`, or `repeated`, computed the same way in C++ and Python (`real_containing_oneof` semantics). |

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
| S1 to S5 | The #244 replay payloads, see [Replay payload fixtures](#replay-payload-fixtures). |
| W1 to W14 | The bounded writer, see below. |
| RT1 to RT14 | The episode reader, see [Reader tests](#reader-tests). |

## Bounded episode writer

`episode_writer.h` (lock id `episode-writer-bounded-v1`, #128) writes accepted
events to an `EpisodeSink` through a fixed-size ring. It is **host-only**: never
call it from an ICON cycle, never link it into a real-time part. It reads no
clock, starts no thread, and has no sleep, timer, or retry. It is C++ only.

### API

- `EpisodeWriter::Create(config, sink, &status)` returns `nullptr` and
  `kInvalidConfig` for a null sink, `capacity` outside `[2, 65536]`, or
  `max_event_bytes` outside `[1, 16 MiB]`. It never clamps. It also returns
  `kInvalidConfig` if the preallocation itself cannot be made. The sink is not
  owned and must outlive the writer.
- `Append(event)` returns an `EpisodeAppendResult` (`status`, `event_error`,
  `writer_index`). It may run from any thread.
- `Flush()`, `Close()`, and `Cancel()` come from one owner thread, not
  concurrently with each other.
- `Stats()` is a plain value snapshot. `FirstSinkError()` is `OkStatus` until a
  sink error latches.
- `EpisodeSink::Write(bytes)` writes all of the bytes or fails. `Sync()` is
  called only by the first `Close`. `FileEpisodeSink::Open(path)` uses
  `O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC` with mode 0644, so it never
  overwrites a file. `Write` loops over partial writes and `EINTR`, and `Sync`
  is `fsync`.

Status numbers are append-only: `kOk` 0, `kInvalidConfig` 1, `kInvalidEvent` 2,
`kReservedEvent` 3, `kOutOfOrder` 4, `kTooManySources` 5, `kBufferFull` 6,
`kEventTooLarge` 7, `kWriteFailed` 8, `kClosed` 9, `kCancelled` 10.

### File format (version 1)

Fixed width, little endian, written with explicit byte shifts. There are no
varints and no trailer.

| Part | Bytes | Content |
| --- | --- | --- |
| File header | 12, once | `49 45 50 49 53 4F 44 31` (`IEPISOD1`), then `01 00 00 00`. Written by the first `Flush` or `Close` that reaches the sink, never by `Create`. |
| Record | 16 + N | `u32 N` (event length, 1 to `max_event_bytes`, or the loss record length), `u32 crc32c`, `u64 writer_index`, then the N `EpisodeEvent` bytes. |

The CRC is CRC32C (`absl::ComputeCrc32c`) over the N event bytes only. `N` and
`writer_index` are not covered. Each record is exactly one `Sink::Write` call
made from the slot buffer that already holds the header and the bytes. The
event bytes are `SerializeToArray` of the accepted event, so a parsed #127
golden comes out byte-identical to its `.binpb`, and an unknown payload keeps
its bytes.

### Append order

The first match wins.

| Step | Check | Result |
| --- | --- | --- |
| 1 | cancelled, closed, or a latched sink failure | `kCancelled`, `kClosed`, `kWriteFailed` (`rejected_after_stop`) |
| 2 | empty message, or `AssessEpisodeEvent` error | `kInvalidEvent`, `event_error` echoed (`rejected_invalid`). A valid unknown payload is accepted. |
| 3 | `source_id == "episode_writer"` or payload case 15 | `kReservedEvent` (`rejected_reserved`) |
| 4 | known source: sequence does not advance. Unknown source while 64 are tracked. | `kOutOfOrder`, `kTooManySources` |
| 5 | `queued + (pending loss ? 1 : 0) + 1 > capacity - 1` | `kBufferFull`, drop with reason `OVERFLOW` |
| 6 | `ByteSizeLong() > max_event_bytes` | `kEventTooLarge`, drop with reason `TOO_LARGE` |
| 7 | accept | `kOk`. A pending loss record is written to the ring first. |

Steps 2 to 4 consume no `writer_index`. Accepted events and the drops of steps 5
and 6 each consume the next one, so every gap in the `writer_index` sequence is
covered by exactly one `EpisodeLoss` range. The first event of a source is
accepted with any sequence, including 0. The producer `sequence` is never
rewritten, and the source table is updated only on accept.

### Loss records

At most one loss is pending. It covers one contiguous index range with one
reason. A drop with the same reason extends it. A drop with the other reason
first writes the pending record to the ring (at the next index), then starts a
new one. The record's own index is `last_dropped_writer_index + 1`.

A loss record is an `EpisodeEvent` with `source_id` and `frame_id`
`episode_writer`, `schema_version` 1, and the `episode_loss` payload. Its
`header.sequence` counts loss records 1, 2, 3, and both times equal the
`receive_time` of the last dropped event in the range. `clock_domain`,
`validity`, `snapshot_id`, `config_digest`, and `provenance` are unset. It
passes `AssessEpisodeEvent` with kind `kEpisodeLoss`. Loss records are never
dropped: `queued <= capacity - 1` after every accept, and the ring reaches
`capacity` only through a reason switch, after which every drop is `kBufferFull`
until a `Flush`.

### Flush, Close, Cancel

- `Flush` writes the header (first time only), the queued records in FIFO
  order, then the pending loss record once the ring has drained. It takes the
  mutex to snapshot the queued range and to release written slots, and holds no
  lock during a sink call. Records appended during a `Flush` stay queued for the
  next call, and a pending loss waits behind them. With nothing left to write
  it makes zero sink calls. It never syncs. The first `Flush` writes the header
  even when the episode is empty.
- The first sink error latches `kWriteFailed`. The failed record and everything
  after it stay queued and are never retried. `FirstSinkError()` keeps the first
  status. Later `Append`, `Flush`, and `Close` return `kWriteFailed`, with no
  sink call, unless the writer is cancelled or already closed (those are
  reported first).
- `Close` flushes until empty, then calls `Sync` once if the flush succeeded. A
  `Sync` error also latches. The writer ends closed in every case and returns
  `kOk` or `kWriteFailed`. A second `Close` makes no sink calls and returns the
  first result. An empty episode closes as the 12 byte header plus one `Sync`.
- `Cancel` makes no sink call. It discards the records in the ring (including
  loss records already written to the ring) and the pending loss, adds the
  discarded record count to `dropped_on_cancel`, and returns it. A pending loss
  that was never written to the ring is not a record and is not counted. A
  second `Cancel` returns 0, and afterwards every call returns `kCancelled`.

### Memory and allocation

Everything is preallocated in `Create`: `capacity` slots of
`max(max_event_bytes, largest loss record) + 16` bytes, the loss template, and
64 source entries whose `std::string` has `max_event_bytes` reserved. That is
about 64 MiB of slots plus about 4 MiB for the source table at the defaults.
`Append` performs no I/O, no blocking wait, and no heap allocation. The one
documented exception is a case-14 `EpisodeMetadata` event with non-empty text:
`ComputeConfigDigest` allocates a 64 character hex string once. That is accepted
for host-only use. One `std::mutex` guards the state. `Append` holds it for the
O(64) source lookup plus serializing one slot.

### Writer tests

| Test | Covers |
| --- | --- |
| W1 | The five v1 goldens in order equal `episode_writer_v1_nominal.iepisod`. Decoded indices 0 to 4, every CRC matches, record bytes equal the `.binpb` bytes. |
| W2, W3 | Empty episode is the 12 header bytes plus one `Sync`. A single event is the header plus one record at index 0. |
| W4 | An event of exactly `max_event_bytes` is accepted, +1 byte is `kEventTooLarge` with a `TOO_LARGE` loss record. `Create` rejects every out-of-range config and a null sink. |
| W5 | `capacity = 4` equals `episode_writer_v1_overflow.iepisod`. Accept after a drop, the `TOO_LARGE` to `OVERFLOW` switch, both reasons in sequence, and the stats. |
| W6, W7 | Out-of-order and interleaved sources, first sequence 0, dropped and rejected events leave the table alone. 64 sources, the 65th is `kTooManySources`. |
| W8, W9 | Every `EpisodeEventError` 1 to 10, the empty message, and reserved events are rejected and consume no index. The v2 unknown payload is accepted byte-exact. |
| W10, W11, W12 | Write failure on the header, on a record, and on `Sync` latches with no more calls. `Flush` and `Close` idempotence. `Cancel` discards without sink calls. |
| W13 | Two runs are byte-identical. `FileEpisodeSink` writes the same bytes as the fake, and `Open` on an existing path fails. |
| W14 | A test-binary `operator new` replacement counts allocations armed around `Append` only: zero for accept, every reject, overflow, too large, loss materialization, the reason switch, and new-source insertion. |

The two `.iepisod` goldens are committed and never rewritten by tests. They
were built outside the writer, from the committed `.binpb` goldens, a CRC32C
reference, and a hand-encoded loss record, and the writer output is compared
with them byte for byte.

## Episode reader

`episode_reader.h` (lock id `episode-reader-v1`, #129) reads a whole
`IEPISOD1` version 1 buffer, validates every record, and reports the longest
structurally valid prefix. It is **host-only**: never call it from an ICON
cycle, never link it into a real-time part. It reads no clock, starts no
thread, and has no sleep, timer, or retry. It is C++ only. It reads the format
exactly as the writer produces it. It changes no schema, proto, or file format.

### API

- `ReadEpisode(bytes, config, filter)` is pure. The returned
  `EpisodeRecordEntry::event_bytes` spans point into `bytes`, which must
  outlive the result. Reading the same input twice gives identical results.
- `EpisodeReaderConfig`: `max_event_bytes` in `[1, 16 MiB]` (default 16 MiB),
  `max_records` in `[1, 16777216]` (default 1,000,000), `max_file_bytes` in
  `[12, 4 GiB]` (default 1 GiB). An out-of-range value gives
  `status = kInvalidConfig` and nothing else is filled. It never clamps. A
  buffer larger than `max_file_bytes` gives `kInputTooLarge`. In both cases no
  record is read and `defect` stays `kNone`.
- `EpisodeReadFilter` selects which valid records appear in `records`: a set of
  `source_id`s, a set of `PayloadKind`s, and an inclusive `writer_index` range.
  The parts are AND-ed, and an empty part matches all. A name that matches
  nothing is not an error. An index range with `first > last` is
  `kInvalidConfig`. The filter never hides a defect: every record is validated,
  and the defect, the last-good fields, `stats`, and `losses` always describe
  the whole valid prefix. Loss records are always in `losses`, and appear in
  `records` only if they pass the filter (kind `kEpisodeLoss`, source
  `episode_writer`).
- `LoadEpisodeFile(path, max_file_bytes)` does `open(O_RDONLY | O_CLOEXEC)`,
  `fstat`, then reads the whole file, looping over partial reads and `EINTR`. A
  size above `max_file_bytes` is `FailedPreconditionError` and nothing is read.
  An invalid `max_file_bytes` is `InvalidArgumentError`, checked first. I/O
  errors map like `FileEpisodeSink`: `ENOENT` is `NotFound`, `EACCES` and
  `EPERM` are `PermissionDenied`, `EEXIST` is `AlreadyExists`, anything else is
  `Internal`.
- `MaxEpisodeLossRecordBytes()` is the serialized size of the largest possible
  writer loss record (122 bytes), computed exactly as `EpisodeWriter::Create`
  computes its `largest` template. A record length limit is
  `max(max_event_bytes, MaxEpisodeLossRecordBytes())`, so a loss record is
  readable even with a tiny `max_event_bytes`.

`EpisodeReaderStatus` and `EpisodeReadDefect` numbers are append-only.

### Result fields

- `good_records`: valid records before the defect, loss records included.
- `good_end_offset`: the byte just past the last valid record. 12 for a
  header-only file, 0 when the header itself is bad.
- `has_last_good_index` and `last_good_writer_index`: the index of the last
  valid record, if any. `next_expected_writer_index` is the index the next
  record would need.
- `defect_offset`: where the defective header or record starts. It equals
  `good_end_offset`, and is 0 when there is no defect.
- `event_error`: the `AssessEpisodeEvent` error, set for `kInvalidEvent`.
- `stats` covers the whole valid prefix and ignores the filter. `records` is
  every valid record, so it equals `good_records`. `loss_records`,
  `unknown_skipped`, and `producer_sources` are subsets or counts within it.
  The three `dropped_*` counters sum `dropped_count` by loss reason (reasons
  above 2 go to `dropped_other_reason`).
- The loss source `episode_writer` is not counted in `producer_sources`.

### Check order

The first failure wins and reading stops. Everything before the defect is
returned and trusted, nothing after it is. There is no resync and no repair.

Header:

| Order | Check | Defect |
| --- | --- | --- |
| 1 | fewer than 12 bytes, including 0 bytes | `kTruncatedHeader` |
| 2 | magic is not `IEPISOD1` | `kBadMagic` |
| 3 | version is not 1 (including 0) | `kUnsupportedVersion` |

A header-only file is a valid empty episode.

Each record, from `good_end_offset` while bytes remain:

| Order | Check | Defect |
| --- | --- | --- |
| 1 | fewer than 16 bytes left | `kTruncatedRecord` |
| 2 | `N == 0` or `N > max(max_event_bytes, MaxEpisodeLossRecordBytes())` | `kLengthOutOfRange` |
| 3 | fewer than `16 + N` bytes left | `kTruncatedRecord` |
| 4 | CRC32C of the N bytes differs from the stored CRC | `kCrcMismatch` |
| 5 | `ParseFromArray` fails | `kParseFailed` |
| 6 | `AssessEpisodeEvent` is not engaged or has an error (echoed in `event_error`) | `kInvalidEvent` |
| 7 | case 15 from a source other than `episode_writer`, or `episode_writer` with any other payload (an unknown payload too) | `kReservedEvent` |
| 7 | a loss record fails a loss rule below | `kLossRecordInvalid` |
| 8 | `writer_index < expected` / `> expected` | `kIndexNotIncreasing` / `kIndexGap` |
| 9 | a new producer source while 64 are tracked / a known source whose sequence does not advance | `kTooManySources` / `kOrderViolation` |
| 10 | this would be record number `max_records + 1` | `kTooManyRecords` |

A valid record updates the counters, the source table, and `expected`, then the
filter decides whether it appears in `records`. The source table is updated
only for valid records. The first sequence of a source may be any value,
including 0. A valid unknown payload (`skip = true`) is a valid record: it
takes its index and passes the source and sequence checks, and its entry keeps
the exact bytes. The reader fills its `EpisodeEventView` with the same helper
as the writer (`episode_event_view.h`, `EpisodeEventViewOf`).

Loss records (case 15 from `episode_writer`) must satisfy all of these, or the
record is `kLossRecordInvalid`:

- `first_dropped_writer_index == expected`
- `last_dropped_writer_index >= first_dropped_writer_index`, and `last` is not
  the largest `uint64`
- `writer_index == last + 1`
- `dropped_count == last - first + 1`
- `reason` is not `LOSS_REASON_UNSPECIFIED`. Reasons above 2 are accepted.
- `header.sequence` is the previous loss `header.sequence` plus one, and the
  first loss has 1
- `source_time == receive_time`, seconds and nanos

Then `expected = writer_index + 1`. The loss checks run before the index check,
so a damaged loss record reports `kLossRecordInvalid`. Every index computation
is overflow-checked. A record whose next expected index would wrap is a
defect: a loss record with `writer_index` equal to the largest `uint64` is
`kLossRecordInvalid`, and an ordinary record with that index is
`kIndexNotIncreasing`.

### Reading the result

- `kNone` means a **structurally intact prefix**, not a cleanly closed
  episode. v1 has no trailer and no sync marker, so the reader cannot tell a
  file that ended on a record boundary from a file that was cut on one. A
  truncation exactly on a record boundary is `kNone` with fewer records.
- The writer legitimately leaves a **0-byte file** after `Cancel` before any
  flush, or after a failed header write. The reader reports it as
  `kTruncatedHeader` with `good_end_offset = 0`. That is a missing episode, not
  a corrupt record.
- The CRC covers only the N event bytes, not `N` or `writer_index`. Damage to
  those fields shows up as `kLengthOutOfRange`, `kTruncatedRecord`, or (with
  overwhelming probability) `kCrcMismatch` or `kParseFailed` of the misframed
  bytes, and a damaged `writer_index` as `kIndexGap`, `kIndexNotIncreasing`, or
  `kLossRecordInvalid`. There is a 2^-32 chance that a misframed `N` over
  garbage still passes the CRC. The parse, assessment, and index checks then
  catch it with high probability, but not with certainty.
- After a later change to how `AssessEpisodeEvent` classifies unknown payloads
  (#240), a file written before it that holds a payload-less event with only
  unknown envelope fields stops at that record with `kInvalidEvent` and
  `kMissingPayload`.
- The whole file is read into memory. At the default `max_file_bytes` that is
  up to 1 GiB resident.

### Reader fixtures

The `episode_reader_v1_*.iepisod` files are committed and never rewritten by
tests. They were built outside the reader and the writer by a throwaway Python
script with its own CRC32C and protobuf wire walker. Inputs are the two
committed writer goldens and the committed v2 unknown-payload `.binpb`. The
script checks its CRC32C against the check value `0xE3069283` and against every
record of both writer goldens before it writes anything. Tests assert the
expected results below. Tests that need other damage (for example `N` changes,
duplicate indices, each loss-field corruption, the 65th source) cut or build
buffers in memory and never write fixtures.

| Fixture | Built from | Defect | `good_records` | `good_end_offset` | Last index |
| --- | --- | --- | --- | --- | --- |
| `header_only` | nominal, first 12 bytes | `kNone` | 0 | 12 | none |
| `bad_magic` | nominal, last magic byte `1` changed to `X` | `kBadMagic` | 0 | 0 | none |
| `version_0` | nominal, version 0 | `kUnsupportedVersion` | 0 | 0 | none |
| `version_2` | nominal, version 2 | `kUnsupportedVersion` | 0 | 0 | none |
| `crc_flip` | nominal, one bit of record 2's CRC flipped | `kCrcMismatch` | 2 | 903 | 1 |
| `index_gap_uncovered` | nominal with record 2 removed | `kIndexGap` | 2 | 903 | 1 |
| `sequence_regression` | nominal, record 2 replaced by record 1's event at index 2 | `kOrderViolation` | 2 | 903 | 1 |
| `forged_loss` | overflow golden, loss record's `source_id` changed to `episode_forger`, CRC recomputed | `kReservedEvent` | 3 | 1517 | 2 |
| `bad_loss_count` | overflow golden, loss `dropped_count` 2 changed to 3, CRC recomputed | `kLossRecordInvalid` | 3 | 1517 | 2 |
| `unknown_payload_record` | header, then the v2 golden at index 0 | `kNone` | 1 | 367 | 0 |
| `index_bit_flip_gap` | nominal, record 2 `writer_index` 2 xor 4 (6), CRC valid | `kIndexGap` | 2 | 903 | 1 |
| `index_bit_flip_decrease` | nominal, record 2 `writer_index` 2 xor 2 (0), CRC valid | `kIndexNotIncreasing` | 2 | 903 | 1 |
| `index_bit_flip_loss` | overflow golden, loss `writer_index` 5 xor 1 (4), CRC valid | `kLossRecordInvalid` | 3 | 1517 | 2 |

### Reader tests

| Test | Covers |
| --- | --- |
| RT1 | The nominal golden: `kNone`, indices 0 to 4, each `event_bytes` equals its `.binpb`, `good_end_offset = 4069`. |
| RT2 | The overflow golden: indices 0, 1, 2, 5, one loss `{5, 3, 4, 2, OVERFLOW, seq 1}`, `dropped_overflow = 2`, `good_end_offset = 1607`. |
| RT3 | Header-only is a valid empty episode. 0 and 11 bytes (and every length below 12) are `kTruncatedHeader`. |
| RT4 | Bad magic, version 0, and version 2. |
| RT5 | The nominal golden cut at every length 0 to 4068. A cut inside a record is `kTruncatedRecord`, a cut on a record boundary is `kNone` with fewer records. No crash. |
| RT6 | CRC flip, event byte flip, `N = 0`, `N` above the limit, a tiny `max_event_bytes`, `N + 1` and `N - 1`, a trailing partial header. |
| RT7 | Duplicate, decreasing, and uncovered-gap indices. Each loss-record rule, the loss sequence, and the index overflow cases. |
| RT8 | Sequence regression, the 65th producer source, a forged case 15, and `episode_writer` with another or an unknown payload. |
| RT9 | A v2 unknown-payload record: `skip`, `kUnknown`, exact bytes, and its part in the index and sequence checks. |
| RT10 | Filters by source, kind, index range, and combined. Defects, stats, and `losses` do not change. An invalid range is `kInvalidConfig`. |
| RT11 | Config bounds, `kInputTooLarge`, `max_records`, the pinned `MaxEpisodeLossRecordBytes()`, and a loss record read with `max_event_bytes = 1`. |
| RT12 | Two reads of the same buffer are field-for-field equal. |
| RT13 | Writer round trip with an in-memory sink: the `TOO_LARGE` to `OVERFLOW` switch, an accept after a drop, cancel and empty close. `LoadEpisodeFile` equals the in-memory bytes, with the size limit, missing path, and invalid limit errors. |
| RT14 | A flipped `writer_index` bit with a valid CRC: the three fixtures, and all 64 bits of a record index and of a loss index. |

## Replay clock

Deterministic episode replay (#130, lock id `episode-replay-clock-v1`). Host
only, sim and test only. Never call it from an ICON cycle and never link it
into a real-time part. It adds no format, schema, reader, or writer change.

The replayer is passive. It starts no thread, never sleeps, and reads no clock.
The caller owns the clock and passes `now` (an `embodiment::ClockReading`) to
`Poll`, so a test with a fake clock is fully deterministic and a real-time
caller decides how to wait.

### API

C++, namespace `intrinsic::episode`, in `episode_replay.h`.

```cpp
std::unique_ptr<EpisodeReplayer> EpisodeReplayer::Create(
    absl::Span<const uint8_t> bytes, const EpisodeReaderConfig& reader_config,
    const EpisodeReadFilter& filter, const EpisodeReplayConfig& config,
    EpisodeReplayStatus* status);
EpisodeReplayResult Step();                              // kStep only
EpisodeReplayStatus Start(embodiment::ClockReading now);  // kTimed only, once
EpisodeReplayResult Poll(embodiment::ClockReading now);   // kTimed only
uint64_t Cancel();
EpisodeReplayStats Stats() const;
```

`Create` calls `ReadEpisode` twice on the same buffer with the same reader
config: once with the filter (the replayed records and the defect) and once
without a filter (the offset of every record, so a loss record that the filter
dropped can still be timed). The replayer keeps both results and does not copy
or own the bytes. The caller keeps the buffer alive, as with the reader's
spans. `Create` returns `nullptr` and sets `*status` on any rejection, and
never clamps. It also parses every emitted header once and computes every due
offset, so `Step`, `Start`, `Poll`, and `Cancel` do not allocate.

An `EpisodeReplayEvent` has `record` (an entry of the filtered read, for a loss
record only when the filtered read contains it), `loss` (set for loss records,
always an entry of the unfiltered read), `writer_index`, `recorded_offset_ns`,
`due_offset_ns`, and `ordinal` (0, 1, 2, ... in emission order). The pointers
point into the replayer's own read results, which are never modified after
`Create`, so they stay valid for the replayer's lifetime.

`EpisodeReplayStatus` numbers are append-only: `kOk` 0, `kInvalidConfig` 1,
`kDefectiveStream` 2, `kMixedClockDomain` 3, `kTimeOutOfRange` 4, `kWrongMode`
5, `kNotStarted` 6, `kNotDue` 7, `kEndOfStream` 8, `kEndOfTrustedPrefix` 9,
`kCancelled` 10, `kInvalidTime` 11.

### Which records are emitted

- The reader's `records` for the filter. Stream filtering is the reader's
  filter, not a second one.
- Plus every loss record, even when the filter excludes it, merged by
  `writer_index`. A gap therefore stays visible.
- A valid unknown-payload record (`skip = true`) is not emitted and takes no
  timeline slot. `stats.skipped_unknown` counts the ones that pass the filter.
- Always in file order (`writer_index`). Time never reorders events.

### Timestamp policy

- **T1, order.** File order only. Time spaces events out and never reorders
  them. Recorded timestamps and bytes are never modified.
- **T2, timeline.** `header.receive_time`, as an offset from the first emitted
  event (after the unknown-payload skip and the loss merge). The arithmetic is
  exact integer nanoseconds in `__int128`, with no floating point.
  `recorded_offset_ns = receive_time - first receive_time`. An offset outside
  `int64` nanoseconds gives `kTimeOutOfRange` at `Create`.
- **T3, receive time that goes backwards.** `scaled_k = recorded_offset_ns *
  speed_den / speed_num`, truncated toward zero. `due_0 = 0` and
  `due_k = max(due_(k-1), scaled_k)`, so a regression is due immediately after
  its predecessor. A regression is `recorded_offset_k < recorded_offset_(k-1)`,
  strictly less than the previous event, and is counted in
  `stats.time_regressions` (at `Create`, over the whole timeline). Both
  `recorded_offset_ns` and `due_offset_ns` are reported. A scaled `due` that
  does not fit `int64` nanoseconds also gives `kTimeOutOfRange`.
- **T4, clock domains.** All emitted non-loss records must share one non-empty
  `clock_domain` (`utc`, `monotonic`, or any other non-empty string, compared
  exactly). Otherwise, including an empty domain on such a record, `Create`
  returns `kMixedClockDomain`. A stream of only loss records has no domain and
  is accepted. Loss records have an empty domain and take their time from the
  last dropped event. The domain of a loss record is not checked. No wall
  clock is ever read.

### Modes

| Mode | Calls | Behavior |
| --- | --- | --- |
| `kStep` | `Step()` | Emits exactly the next event and ignores time. `Poll` and `Start` return `kWrongMode`. The same offsets are computed and reported. |
| `kTimed` | `Start(now)` once, then `Poll(now)` | `Start` anchors replay time, `due_time = start + due_offset`, and an event is due when `now >= due_time`. `Poll` returns at most one due event, or `kNotDue` with `next_due`. `Step` returns `kWrongMode`. |

Speed is an integer ratio `speed_num / speed_den`, each in [1, 1000]. Real time
is 1/1, 10/1 is ten times faster, and 1/2 is slow motion. Anything outside
[1, 1000] is `kInvalidConfig` and is checked in both modes. An unknown mode
number is `kInvalidConfig`. A reader config that the reader rejects (reader
status other than `kOk`) is `kInvalidConfig` too, whatever
`allow_defective_prefix` says.

`Start(now)` rejects a `start + last due offset` that overflows `ClockReading`
with `kTimeOutOfRange` and does not start. A `now` with nanos outside
[0, 1e9) is `kInvalidTime`. A `now` earlier than the previous `now` is allowed:
nothing becomes due early, and it is not an error. `next_due` is normalized
(nanos in [0, 1e9)).

### Defective input

By default `Create` rejects a read with `defect != kNone` as
`kDefectiveStream`. With `allow_defective_prefix = true` it replays the trusted
prefix, the end status is `kEndOfTrustedPrefix`, and `reader_defect` carries
the reader's defect. A clean file ends with `kEndOfStream`. The defect is the
filtered read's defect.

### Call precedence, end, and cancel rules

The first match wins.

| Call | Order |
| --- | --- |
| `Step` | cancelled: `kCancelled`. Mode is not `kStep`: `kWrongMode`. No events left: the end status. Otherwise `kOk`. |
| `Start` | cancelled: `kCancelled`. Mode is not `kTimed` or already started: `kWrongMode`. Bad nanos: `kInvalidTime`. Overflow: `kTimeOutOfRange`. Otherwise `kOk`. |
| `Poll` | cancelled: `kCancelled`. Mode is not `kTimed`: `kWrongMode`. Not started: `kNotStarted`. Bad nanos: `kInvalidTime`. No events left: the end status. Next event not due: `kNotDue` with `next_due`. Otherwise `kOk`. |

- End of stream is sticky: `kEndOfStream` or `kEndOfTrustedPrefix` on every
  later call. An empty stream (header only, everything filtered out or
  skipped) creates fine and ends on the first call.
- `Cancel()` is idempotent. The first call returns the number of events not yet
  emitted (also in `stats.cancelled_remaining`), later calls return 0. After
  it, `Step`, `Poll`, and `Start` return `kCancelled`, even at the end of the
  stream.
- No rewind, no pause, and no `Reset()`. A repeated replay is a new replayer
  over the same buffer.

### Limitations

- `receive_time` is producer-filled. Replay timing is only as faithful as the
  producers' clocks, and T3 hides small regressions behind a count.
- Episodes that mix `utc` and `monotonic` producers are rejected.
- Integer speed ratios cap acceleration at 1000x.
- A loss record takes the last dropped event's time, so a long loss burst shows
  up as one late event.
- The passive model puts the waiting loop in the caller. Real-time pacing
  quality is the caller's to own.
- Two reads cost two passes and up to twice the entry memory.
- No time-range filtering beyond the reader's filter, no subsystem or replay
  adapter, and no digest equality (#131, #132). C++ only, no Python replayer.

### Replay tests

`episode_replay_test` builds its extra streams in memory (framing with a
recomputed CRC32C). The replayer never writes a stream and no new fixture is
committed.

| Test | Covers |
| --- | --- |
| RC1 | `Step` on the nominal golden: indices 0 to 4, one per call, then sticky `kEndOfStream`. |
| RC2 | `kTimed` 1/1 with a fake clock: due exactly at the recorded offsets, 1 ns early is `kNotDue` with the exact `next_due`. |
| RC3 | Speeds 10/1, 1000/1, 1/2: same order, exact truncated offsets. |
| RC4 | The overflow golden: loss record (index 5) in file order, with the last dropped event's time. |
| RC5 | A loss record survives a filter that excludes it. An unknown-payload record is skipped and counted. |
| RC6 | A decreasing `receive_time`: monotone due offsets, kept recorded offsets, `time_regressions`. |
| RC7 | `kMixedClockDomain`. A loss record with an empty domain is accepted. |
| RC8 | Defective input: `kDefectiveStream` by default, the trusted prefix and `kEndOfTrustedPrefix` with the opt-in. |
| RC9 | Cancel mid-stream, idempotent, remaining counts. Wrong-mode, not-started, and invalid-time calls. |
| RC10 | Config bounds, reader config, empty stream, `kTimeOutOfRange` at `Create` and at `Start`. |
| RC11 | Two replays give equal `(writer_index, recorded_offset_ns, due_offset_ns)` sequences. |
| RC12 | An allocation guard that replaces the throwing and the nothrow `operator new` forms counts 0 allocations in `Step`, `Start`, `Poll`, and `Cancel`. |

### Expected results for the goldens

Each row is `(ordinal, writer_index, recorded_offset_ns, due_offset_ns)`. The
recorded offsets do not depend on the mode. `kStep` reports the due offsets of
its configured speed (1/1 by default), the same numbers as `kTimed` at that
speed. The goldens are spaced one second apart (receive times 1700000101.251 s
to 1700000105.251 s). The numbers come from an independent throwaway script
that reads the golden bytes, not from the replayer.

| Golden | `kStep` and `kTimed` 1/1 | `kTimed` 10/1 | `kTimed` 1000/1 | `kTimed` 1/2 |
| --- | --- | --- | --- | --- |
| `episode_writer_v1_nominal` | (0, 0, 0, 0)<br>(1, 1, 1000000000, 1000000000)<br>(2, 2, 2000000000, 2000000000)<br>(3, 3, 3000000000, 3000000000)<br>(4, 4, 4000000000, 4000000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 100000000)<br>(2, 2, 2000000000, 200000000)<br>(3, 3, 3000000000, 300000000)<br>(4, 4, 4000000000, 400000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 1000000)<br>(2, 2, 2000000000, 2000000)<br>(3, 3, 3000000000, 3000000)<br>(4, 4, 4000000000, 4000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 2000000000)<br>(2, 2, 2000000000, 4000000000)<br>(3, 3, 3000000000, 6000000000)<br>(4, 4, 4000000000, 8000000000) |
| `episode_writer_v1_overflow` | (0, 0, 0, 0)<br>(1, 1, 1000000000, 1000000000)<br>(2, 2, 2000000000, 2000000000)<br>(3, 5, 4000000000, 4000000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 100000000)<br>(2, 2, 2000000000, 200000000)<br>(3, 5, 4000000000, 400000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 1000000)<br>(2, 2, 2000000000, 2000000)<br>(3, 5, 4000000000, 4000000) | (0, 0, 0, 0)<br>(1, 1, 1000000000, 2000000000)<br>(2, 2, 2000000000, 4000000000)<br>(3, 5, 4000000000, 8000000000) |

The overflow golden's loss record (index 5, dropped 3 and 4) carries the
receive time of dropped event 4, which is 4 s after event 0.

## Targets

All targets are opt-in. None is listed in
`.github/baseline/manipulator_targets.tsv`.

| Target | Purpose |
| --- | --- |
| `episode_event_policy`, `episode_event_policy_py` | Policy libraries. |
| `episode_event_policy_test`, `episode_event_policy_test_py` | Plain-value unit tests. |
| `episode_event_serialization_test`, `episode_event_serialization_test_py` | E1 to E8 against the committed goldens. |
| `episode_event_schema_freeze_test`, `episode_event_schema_freeze_test_py` | E9. |
| `episode_writer` | Bounded writer, `EpisodeSink`, `FileEpisodeSink`. Visibility is private until a later issue widens it with review. |
| `episode_writer_test` | W1 to W14. |
| `episode_event_view` | Header-only helper that fills `EpisodeEventView` from an `EpisodeEvent`, shared by the writer and the reader. Private. |
| `episode_reader` | Whole-buffer reader, filters, and `LoadEpisodeFile`. Visibility is private until a later issue widens it with review. |
| `episode_reader_test` | RT1 to RT14. |
| `episode_replay` | Deterministic replay clock. Visibility is private until a later issue widens it with review. |
| `episode_replay_test` | RC1 to RC12. |

```
bazel test //intrinsic_data/episode/...
```

## Replay payload fixtures

Lock id `episode-replay-payloads-v1` (#244). The schema gains three oneof
cases, all additive and all still `schema_version` 1:

| Case | Arm | `PayloadKind` |
| --- | --- | --- |
| 16 | `recorded_message` (`RecordedMessage`) | `kRecordedMessage` = 8 |
| 17 | `world_snapshot_record` (`WorldSnapshotRecord`) | `kWorldSnapshotRecord` = 9 |
| 18 | `desired_motion` (`intrinsic_proto.vehicle.DesiredMotion`) | `kDesiredMotion` = 10 |

`AssessEpisodeEvent` adds no error code and no inner validation: a new case
is a payload (never `kMissingPayload`), and the adapters (#131, #132) check the
wrapper, the record, and the intent. `DesiredMotion.header` is recorded
verbatim and not cross-checked against the envelope header. The writer, reader,
and replayer need no change: the new kinds are known, so they are emitted and
not counted in `skipped_unknown`. A reader built before #244 sees these records
as unknown payloads and skips them with the bytes kept (#127 E6).

The three goldens share the v1 metadata envelope identity: source
`episode_recorder`, frame `world_enu`, clock `monotonic`, `schema_version` 1,
the same `snapshot_id`, `config_digest` and `provenance`. Only the sequence
(6, 7, 8) and the times (1700000106 to 1700000108 s) differ. The `.binpb` files
were produced by an independent throwaway script that encodes the wire bytes by
hand, never by code under test, and are never rewritten by tests.

The fixed `MeasurementEnvelope` inside `episode_event_v1_recorded_message`
(`message_bytes`, 113 bytes). It has header (sequence 1, source `dvl_0`, frame
`body`, clock `monotonic`, valid), `source_id` `dvl_0`, `kind` DVL (3),
`payload_type` `intrinsic_proto.vehicle.BodyTwist`, and a `payload` of one
`BodyTwist` with `linear_x_m_s = 0.5`:

```
0a 38 08 01 12 0b 08 e4 e2 cf aa 06 10 80 e5 9a
77 1a 0b 08 e4 e2 cf aa 06 10 c0 e9 d7 77 22 05
64 76 6c 5f 30 2a 04 62 6f 64 79 32 09 6d 6f 6e
6f 74 6f 6e 69 63 3a 02 08 01 12 05 64 76 6c 5f
30 18 03 22 21 69 6e 74 72 69 6e 73 69 63 5f 70
72 6f 74 6f 2e 76 65 68 69 63 6c 65 2e 42 6f 64
79 54 77 69 73 74 2a 09 09 00 00 00 00 00 00 e0
3f
```

| Test | Covers |
| --- | --- |
| S1 | Each new golden parses and re-serializes byte-exactly, and the textproto equals the binpb. C++ and Python. |
| S2 | Cases 16, 17, 18 classify to kinds 8, 9, 10, are engaged, `kNone`, and not `kMissingPayload`. The kind numbers are pinned. |
| S3 | Each golden's payload is one top-level field with the allocated tag and wire type 2. A view with `payload_case = 0` and `has_unknown_fields = true` (the pre-#244 view) is `kUnknown` and skipped. |
| S4 | The freeze file and freeze tests (message set and the 11 new lines). |
| S5 | C++ only, `episode_replay_payloads_test`: write the three goldens with `EpisodeWriter`, read them with `EpisodeReader` (kinds 8, 9, 10, known), and emit them with `EpisodeReplayer` in `kStep` with `skipped_unknown == 0`. |

## Replay adapters (#131)

Lock id `episode-replay-adapters-estimator-world-v1`. Two passive, host-only
adapters turn replayed episode events into estimator and World calls. C++ only,
namespace `intrinsic::episode`, `//visibility:private`, sim and test only. The
adapters add no schema, writer, reader, replay, estimator or World change, and
`schema_version` stays 1. They start no thread, never sleep, and read no clock
and no randomness. They never modify recorded bytes or timestamps, never reorder
events, and never synthesize, interpolate or hold-last.

| Target | Contents |
| --- | --- |
| `episode_replay_adapter_common` | Header-only. `ReplayAdapterStatus`, `ReplayCompatibilityConfig`, `ParseReplayRecord`, `CheckCompatibility`. |
| `estimator_replay_adapter` | `EstimatorReplayAdapter`, fed with `EpisodeReplayEvent`s. |
| `world_replay_adapter` | `WorldReplayAdapter`, fed with `EpisodeReplayEvent`s. |
| `estimator_replay_adapter_test` | RA1 to RA6, RA8 to RA13 (estimator side). |
| `world_replay_adapter_test` | RA2 to RA5, RA7 to RA10, RA12 (World side). |

```cpp
// Both adapters.
static std::unique_ptr<Adapter> Create(..., ReplayAdapterStatus* status);
StepType Feed(const EpisodeReplayEvent& event);  // from EpisodeReplayer::Step()
SummaryType Finish();                            // idempotent
```

`EstimatorReplayAdapter::Create(EstimatorService*, EstimatorReplayConfig,
status, EstimatorResult* init_result = nullptr)` and
`WorldReplayAdapter::Create(WorldReplayConfig, status)`. The adapters own no
replayer, buffer or clock. A test drives `EpisodeReplayer` in `kStep` and passes
each `Step()` event to `Feed`.

`ReplayAdapterStatus` numbers are append-only: `kOk` 0, `kInvalidConfig` 1,
`kIgnoredPayload` 2, `kIncompatibleSchema` 3, `kConfigDigestMismatch` 4,
`kSnapshotIdMismatch` 5, `kInvalidRecord` 6, `kMissingRequiredStream` 7,
`kLatched` 8, `kLossRecord` 9, `kFinished` 10.

### Per-`Feed` order

Identical in both adapters. The first match wins.

| Step | Rule | Result |
| --- | --- | --- |
| 1 | After `Finish()`. Nothing is counted or traced. | `kFinished`, `writer_index` 0 |
| 2 | The adapter is latched. Counted in `stats.after_latch`. | `kLatched` |
| 3 | `event.loss != nullptr`. `gaps += 1`, `gap_dropped += dropped_count`, `degraded = true`. No synthesis, no `Reset`, replay continues. | `kLossRecord` |
| 4 | `ParseReplayRecord` fails. `invalid_records += 1`, `degraded = true`. | `kInvalidRecord` |
| 5 | `CheckCompatibility` on every parsed record, ignored kinds and `EpisodeMetadata` (case 14) included. `schema_version != 1`, or a non-empty event `config_digest` that differs from a non-empty `expected_config_digest`. | `kIncompatibleSchema` or `kConfigDigestMismatch`, and the adapter latches |
| 6 | Dispatch on `record->payload_kind`. A kind the adapter does not own (also kind 10, `desired_motion`, and `EpisodeMetadata`) is counted in `stats.ignored`. | `kIgnoredPayload` |

Only the two compatibility statuses latch. Every other status is per record. An
empty event `config_digest` is always accepted (#127 "absent"). An empty
`expected_config_digest` disables the digest check. A non-empty
`expected_config_digest` that is not 64 lowercase hex characters is
`kInvalidConfig`, never clamped.

### Estimator adapter

- `Create` calls `EstimatorService::Initialize(estimator_config)` exactly once
  and never calls `Predict`, `Reset` or `Initialize` again. A null service, an
  invalid compatibility config, an empty or duplicate `required_sources` entry,
  or an `Initialize` result with `ok == false` gives `nullptr` and
  `kInvalidConfig`. The `Initialize` result is returned in `*init_result`. The
  other checks run before `Initialize`, so a config the adapter rejects never
  touches the service.
- It owns only `PayloadKind::kRecordedMessage` (8). A `RecordedMessage` whose
  `message_type` is not `intrinsic_proto.estimation.MeasurementEnvelope` is
  `kIgnoredPayload` and counted in `stats.ignored_message_type`. Bytes that do
  not parse as a `MeasurementEnvelope` are `kInvalidRecord`.
- Each `MeasurementEnvelope` gets exactly one `Ingest`, with the parsed envelope
  unmodified. There is no restamp and no cross-check against the `EpisodeEvent`
  header. The estimator validates its own envelope. Estimator rejects are data:
  a `kOk` step with `result.ok == false`, counted in `stats.rejected`, never an
  adapter error and never `degraded`.
- `envelope.source_id()` counts as seen even when the estimator rejects it.
  `Finish` reports `kMissingRequiredStream` with the sorted unique list in
  `missing_required_sources` when a `required_sources` entry never appeared. A
  latched status takes precedence for `summary.status` (the list is still
  filled). Absent sources that are not required are fine, and the estimator's
  own source-health policy applies.
- Each `Ingest` appends an `EstimatorTraceEntry` `(writer_index, ok, reason,
  epoch, state_digest)`.

### World adapter

- `Create` validates the policy with `world::snapshot_skew_internal::PolicyValid`
  (the #114 rules), and an invalid compatibility config. Either is
  `kInvalidConfig` and `nullptr`.
- It owns only `PayloadKind::kWorldSnapshotRecord` (9). With `d = record.snapshot()`:
  1. No `snapshot`, or `AssessWorldSnapshotDescriptor(d)` does not accept:
     `kInvalidRecord`, with `descriptor_error` set, `invalid_records += 1`,
     `degraded = true`.
  2. A non-empty envelope `snapshot_id` that differs from `d.snapshot_id()`:
     `kSnapshotIdMismatch`.
  3. `ComputeSnapshotId(d.state_epoch(), WorldSnapshotViewFromProto(d, true).components)`
     differs from `d.snapshot_id()`: `kSnapshotIdMismatch`. The assessors do
     not recompute the id, so this is an explicit comparison.

  Steps 2 and 3 increment `snapshot_id_mismatches`, set `degraded`, do not latch,
  and the policy does not run.
  4. `timings` convert one to one into `world::ComponentTimings`, in record
     order. An unset timestamp or duration is `{0, 0}`. Nothing is dropped,
     deduplicated or reordered. The assessor judges duplicates
     (`kInvalidTimings`).
  5. `query_time` is the event's `header.receive_time`. No wall clock.
  6. `AssessWorldSnapshotDescriptorSkew(d, timings, query_time, policy)`. The step
     is `kOk` with `assessed == true`, and a non-`kNone` `assessment.error` is
     still `kOk` at adapter level (status `kUnspecified`, `withhold` true in the
     trace). Missing, stale and partial kinds follow #114 exactly.
- Each assessed record appends a `WorldTraceEntry` `(writer_index, snapshot_id,
  status, withhold)`. There is no stream-level requirement beyond #114, and zero
  World records is `kOk`.

### Digests

- **Estimator state digest (R6).** `EstimatorStateDigest` in
  `estimator_replay_adapter.cc` reads only `service->GetState()`. It is 0 when
  `state` is unset. Otherwise it is FNV-1a 64 (offset `0xCBF29CE484222325`, prime
  `0x100000001B3`) over `state()` serialized with
  `SetSerializationDeterministic(true)`. This is byte for byte
  `FakeEstimatorService::StateDigest()` and does not downcast to the fake. RA3
  asserts `step.state_digest == fake.StateDigest()` after every `Ingest`. The
  digest follows `VehicleState.sources`, so it moves with per-source health even
  though `Ingest` never changes the published pose, mode or epoch.
- **World chain digest (R7).** Lowercase hex SHA-256 (`internal::EpisodeSha256Hex`)
  over `"v1\n"` followed by `snapshot_id + "\n"` for each assessed record in feed
  order. Withheld snapshots are included. Invalid and mismatched records are
  excluded. With no assessed record it is the SHA-256 of `"v1\n"`
  (`2d27fbdf4e8ca207afbfa388ca9172fbcc6c70e534af2476b3b704f87debadcf`).

### Fixtures and tests

No file is added under `testdata/`. The tests build every episode in memory with
`EpisodeWriter` and an in-memory `EpisodeSink`, read it with `EpisodeReader` and
replay it with `EpisodeReplayer` in `kStep`. Event envelopes copy the v1
metadata golden identity (`source_id`, `clock_domain "monotonic"`, the pinned
seed-42 `config_digest`, provenance, `schema_version` 1). Only the sequence, the
times and the payload vary. Estimator measurements are the #244 113-byte
`MeasurementEnvelope` with other `source_id` and times. Positive World records
recompute their `snapshot_id`. The `episode_event_v1_world_snapshot_record`
golden is read only as the `kSnapshotIdMismatch` negative case (see
Limitations) and is not modified.

| Test | Covers |
| --- | --- |
| RA1 | Estimator nominal: metadata, 3 measurements from 2 sources and a World event (ignored). The trace equals the committed table. A call-counting wrapper sees 1 `Initialize`, 3 `Ingest`, 0 `Predict` and 0 `Reset`. |
| RA2 | World nominal: one FRESH and one PARTIAL snapshot, the trace and chain digest equal the committed table, a measurement event is ignored. |
| RA3 | Determinism: the same bytes replayed twice into fresh adapters give identical steps, traces, stats and digests. `state_digest == fake.StateDigest()` after every `Ingest`, also after the test faults the fake between two events. |
| RA4 | Compatibility: `schema_version 2` and a wrong non-empty `config_digest`, each on a measurement or World event, an ignored kind and a case-14 metadata event, give their status and latch. An empty event digest is accepted, and an empty `expected_config_digest` disables the check. |
| RA5 | World integrity: envelope id mismatch, the #244 golden, a wrong `state_epoch` and the structural defects (duplicate or empty kind, no creation time, no id, no snapshot). None latch, and replay continues. |
| RA6 | Required sources: a missing source, a source seen only as rejects, and absent sources that are not required. |
| RA7 | #114 statuses: INCOMPLETE, STALE (horizon expired, no timing), EXCESSIVE_SKEW, duplicate timings (`kInvalidTimings`) and an unset horizon equal a direct `AssessWorldSnapshotDescriptorSkew` call with `query_time = receive_time`. |
| RA8 | Loss: a writer overflow gives a loss record, `kLossRecord`, `gaps == 1`, `gap_dropped == dropped_count`, `degraded`, and later events are consumed. |
| RA9 | Ignored: other kinds (safety, trajectory, inference, `desired_motion` 18) and another `RecordedMessage` type are counted with no `Ingest` and no assessment. |
| RA10 | Boundaries: an empty episode, a single event, `kFinished` after `Finish()`, and an idempotent `Finish()`. |
| RA11 | Estimator rejects: out-of-order, duplicate-timestamp and clock-domain-mismatch inner envelopes are `kOk` steps with the matching `RejectReason`. |
| RA12 | Config: a null service, a bad `expected_config_digest`, empty or duplicate `required_sources`, an `Initialize` the estimator rejects, and an invalid #114 policy. |
| RA13 | Unparsable `MeasurementEnvelope` bytes give `kInvalidRecord` and `degraded`, with no `Ingest`. |

### Limitations

- Loss records cannot attribute the lost source, so `degraded` is episode-wide.
- `query_time` is `header.receive_time`. A replayed #114 status equals the live
  status only if live assessed at `receive_time`.
- The state digests are fake-estimator digests. Real-estimator replay digests
  are a later concern.
- The #244 golden `episode_event_v1_world_snapshot_record` does not recompute
  its `snapshot_id` (`6ceb1af0...33de`, the recomputed id for `state_epoch 7`,
  `bathymetry_reference=3`, `current_field=5` is
  `02ed8cd9476beeb4395573244f1744c052d41a16387c5fc1c1375319836a9900`). Under
  this adapter it is a `kSnapshotIdMismatch` case, not a valid World snapshot.
- Out of scope: the #132 adapters (planner, safety, inference, `desired_motion`;
  kind 10 is ignored), Python, `.gitattributes`, and any schema, writer, reader,
  replay, estimator or World change.

## Replay adapters (#132)

Lock id `episode-replay-adapters-planning-safety-inference-v1`. Three more
passive, host-only adapters verify replayed planning, safety and inference
records. They follow the #131 shape and reuse `episode_replay_adapter_common.h`
unchanged (the R2 `Feed` order, `ReplayAdapterStatus`, `ReplayCompatibilityConfig`,
`ParseReplayRecord`, `CheckCompatibility`). C++ only, namespace
`intrinsic::episode`, `//visibility:private`, sim and test only. They add no
schema, proto, freeze-file, writer, reader, replay, view-helper or policy
change, no #131 adapter change, and `schema_version` stays 1. They start no
thread, never sleep, and read no clock and no randomness. They never modify
recorded bytes or timestamps, never reorder events, and never synthesize.

No planner, safety supervisor, rule aggregation, `InferenceClient`, queue or
`ReplayWorker` is linked or called. Each config has one field, the
compatibility rules, so there is nothing to inject.

| Target | Contents |
| --- | --- |
| `planning_replay_adapter` | `PlanningReplayAdapter`. Owns kinds 2 (`vehicle_trajectory`) and 10 (`desired_motion`). |
| `safety_replay_adapter` | `SafetyReplayAdapter`. Owns kinds 1 (`safety_decision`) and 10. |
| `inference_replay_adapter` | `InferenceReplayAdapter`. Owns kinds 3 (`inference_envelope`) and 4 (`inference_result`). |
| `planning_replay_adapter_test`, `safety_replay_adapter_test`, `inference_replay_adapter_test` | RB1 to RB14. |

### API

```cpp
// All three adapters (X is Planning, Safety or Inference).
static std::unique_ptr<XReplayAdapter> Create(const XReplayConfig& config,
                                              ReplayAdapterStatus* status);
XReplayStep Feed(const EpisodeReplayEvent& event);  // from EpisodeReplayer::Step()
XReplaySummary Finish();                            // idempotent
```

`XReplayConfig` holds only `ReplayCompatibilityConfig compatibility`. A
non-empty `expected_config_digest` that is not 64 lowercase hex characters gives
`nullptr` and `kInvalidConfig`, never clamped. `Feed` follows the #131 order
(finished, latched, loss, parse, compatibility, dispatch). Only
`kIncompatibleSchema` and `kConfigDigestMismatch` latch. A kind the adapter does
not own, including `recorded_message`, `world_snapshot_record` and
`EpisodeMetadata`, gives `kIgnoredPayload`. A loss record gives `kLossRecord`
(`gaps`, `gap_dropped`, `degraded`).

Domain outcomes live in step and trace fields. The status is `kOk` for every
owned, parsed and compatible record, even when its assessment fails (the same
pattern as the #131 World and estimator rejects). A rejected record never
enters a chain or the pending list.

### Digests and chains

`payload_sha256` is the lowercase hex SHA-256 (`internal::EpisodeSha256Hex`) of
the payload message's deterministic serialization
(`SetSerializationDeterministic(true)`). For a `DesiredMotion` it equals
`safety::ComputeOriginalIntentDigest` (a test asserts it). A chain is the
SHA-256 of `"v1\n"` followed by one `payload_sha256 + "\n"` line per record
that enters the chain, in feed order. The empty chain is the SHA-256 of `"v1\n"`
(`2d27fbdf4e8ca207afbfa388ca9172fbcc6c70e534af2476b3b704f87debadcf`).

| Chain | Adapter | Records that enter it |
| --- | --- | --- |
| `plan_chain` | planning | Accepted trajectories. |
| `intent_chain` | planning | Accepted motions. |
| `decision_chain` | safety | Accepted decisions, linked or not. |
| `inference_chain` | inference | `kComplete` and `kReplayExpired` results. |

### Planning adapter (P1-A)

Verify and digest. There is no planner call.

- `desired_motion`: `vehicle::AssessDesiredMotion(safety::DesiredMotionViewFromProto(m))`.
- `vehicle_trajectory`: an adapter-private `VehicleTrajectoryViewFromProto`
  (field for field, presence flags from proto `has_*`, a copy of the view in
  `trajectory_contract_serialization_test.cc`), then
  `vehicle::AssessVehicleTrajectory`.
- Step and trace entry: `(writer_index, payload_kind, error, accepted,
  payload_sha256)`. `error` is the `TrajectoryContractError` or
  `vehicle::ContractError` number.
- Stats: `fed`, `trajectories`, `trajectories_accepted`, `motions`,
  `motions_accepted`, `ignored`, `invalid_records`, `gaps`, `gap_dropped`,
  `after_latch`. `degraded` is set by a loss record or an unparsable record.

### Safety adapter (P2-A)

Verify, and link each decision to an earlier motion. There is no
`AggregateSafetyRuleResults` call and no supervisor.

- `desired_motion`: assessed as in the planning adapter. Every motion, accepted
  or not, adds `ComputeOriginalIntentDigest(motion)` to the seen set (the
  recorded intent is what the filter digested). A motion trace entry leaves
  `kind`, `original_intent_digest`, `intent_linked` and `snapshot_id` at their
  defaults.
- `safety_decision`: `AssessSafetyDecisionProto`. `intent_linked` is true when
  the decision's `original_intent_digest` is in the seen set. The set is checked
  only against earlier records, so a decision recorded before its motion is
  unlinked. An unlinked decision sets `degraded`, counts in
  `unlinked_decisions`, and never latches.
- Trace entry: `(writer_index, payload_kind, error, accepted, kind,
  original_intent_digest, intent_linked, snapshot_id, payload_sha256)`.
  `snapshot_id` is trace only, never compared (P5-A).
- Stats: the planning stats without `trajectories*`, plus `decisions`,
  `decisions_accepted` and `unlinked_decisions`.

### Inference adapter (P3-A, P4-A, P6-A)

Direct evaluation with the #118 assessors and a mirror of the #120 key and
expiry predicate. There is no `InferenceClient`, no `ReplayWorker`, no
`//intrinsic_inference/queue` dependency, and no #119 or #120 change.

- Views: adapter-private `CommonViewOf` overloads, identical field for field to
  `inference_envelope_contract_serialization_test.cc`. The OIP views come from
  `oip_request` and `oip_result`. Then `AssessInferenceEnvelope` or
  `AssessInferenceResult`.
- Pairing key: `(provenance.model_id, input_digest, state_epoch)`. Pairing takes
  the oldest pending entry with an equal key.
- Clock: `now` is the inner `header.receive_time` of the inference record being
  processed (P4-A), never the episode envelope time.
- Accepted envelope: run the missing sweep over the existing pending entries
  with `now` = its inner `receive_time`, then append it to `pending` with its
  key, `deadline`, `provenance.digest`, `world_snapshot_id` and `writer_index`.
  The new entry is never judged against its own `receive_time`.
- Accepted result, `now` = its inner `receive_time`:
  1. Run the missing sweep, excluding pending entries whose key equals the
     result's key, so a late result is `kReplayExpired` and not `kMissing` plus
     an orphan.
  2. No pending entry with an equal key: `kReplayMiss` (orphan, not chained).
     Otherwise remove the oldest equal-key entry. The outcome is
     `TimeBefore(result.deadline, now) ? kReplayExpired : kComplete`. The test
     is strict, so an equal deadline is complete.
  3. When the envelope `provenance.digest` is non-empty and differs from the
     result's, set `model_digest_mismatch` and `degraded`. This never latches.
  The result is never modified.
- Missing sweep: every pending entry with `TimeBefore(deadline, now)` is removed
  in pending order and reported as `kMissing` with `expired = true`. Each sets
  `degraded` and counts in `missing_expired`. A record whose `receive_time` is
  absent or has nanos out of range skips the sweep. `Finish()` reports every
  remaining pending entry, in pending order, as `kMissing` with
  `expired = false` (`missing_unresolved`, `degraded`).
- `InferenceOutcome` (append-only): `kNone` 0, `kComplete` 1, `kReplayExpired` 2,
  `kReplayMiss` 3, `kMissing` 4.
- Trace entries: `trace` holds `(writer_index, payload_kind, error, accepted)`
  for each owned record. `outcomes` holds `(writer_index, outcome, expired,
  model_digest_mismatch, envelope_writer_index, world_snapshot_id)` for each
  pairing and each missing entry, in order of occurrence. `writer_index` is the
  result's, or the envelope's for `kMissing`. `envelope_writer_index` is 0 for
  an orphan, which `outcome == kReplayMiss` tells apart from a paired envelope
  whose own writer index is 0. `world_snapshot_id` is trace only (P5-A).
  `InferenceReplayStep::outcomes` lists, in order, the `kMissing` entries the
  record swept, then the result's pairing entry.
- Stats: `fed`, `envelopes`, `envelopes_accepted`, `results`,
  `results_accepted`, `complete`, `expired`, `orphans`, `missing_expired`,
  `missing_unresolved`, `model_digest_mismatches`, `ignored`, `invalid_records`,
  `gaps`, `gap_dropped`, `after_latch`.

### Why the inner `receive_time` is the clock (G4)

In the #127 goldens the episode envelope `receive_time` is `1700000103/104.251 s`
(the recorder), while the inner inference header `receive_time` is
`1700000000.001 s` with `deadline 1700000000.5 s` (the producer). Both say
`clock_domain "monotonic"` but they are different timelines, so judging the
inner deadline against the envelope time would make the golden pair expired.
The golden pair is `kComplete` under P4-A. The inner `world_snapshot_id`
(`0123...cdef`) also differs from the envelope `snapshot_id` (`6ceb...33de`),
which is why snapshot ids are trace only.

### R5 erratum: an accepted result with an unusable `receive_time` (#248)

Lock id `episode-replay-inference-receive-time-v1` (amends R1 and R5 of the #132
lock). `AssessInferenceResult` does not look at the inner `receive_time`, so an
accepted result can carry one that is absent or has nanos out of range. Such a
result still pairs with the oldest equal-key pending envelope and removes it, but
its deadline is not judged and `TimeBefore` is not called. The outcome is
`kComplete` with `InferenceReplayStep::receive_time_unusable` set, which means
"paired; deadline not judged". The sweep is still skipped for it. With no
matching envelope the outcome is `kReplayMiss` (an orphan), also with the flag
set. Every such result, paired or orphaned, increments
`stats.unusable_receive_time` and sets `degraded`. `stats.complete` counts
judged completions only, so an unjudged pair is not in it, and
`complete + expired + (unusable_receive_time - orphans with an unusable time)`
equals the number of paired results. D2 chain rule: `inference_chain` holds only
the `kComplete` and `kReplayExpired` results whose deadline was judged, so an
unjudged pair never enters it. `InferenceOutcomeEntry` and `InferenceTraceEntry`
are unchanged. The tests are RB15a to RB15f. Envelopes with an unusable
`receive_time` are unchanged.

### Fixtures and tests

No file is added under `testdata/`. The tests build every episode in memory with
`EpisodeWriter` and an in-memory `EpisodeSink`, read it with `EpisodeReader` and
replay it with `EpisodeReplayer` in `kStep`. Envelopes copy the v1 metadata
golden identity (the pinned seed-42 `config_digest`, `schema_version` 1). The
#127/#244 goldens `episode_event_v1_{safety_decision,vehicle_trajectory,
inference_envelope,inference_result,desired_motion}` are read unmodified. Other
records are built from them with proto setters. The expected digests and chains
are committed as test constants and were recomputed independently of the C++
code.

| Test | Covers |
| --- | --- |
| RB1 | Planning nominal: the golden trajectory, the golden motion and a built trajectory. Trace and both chains equal the committed table. |
| RB2 | Safety nominal: the golden decision (before any motion) is unlinked, then the golden motion, then a decision with `original_intent_digest = ComputeOriginalIntentDigest(motion)` is linked. |
| RB3 | Inference nominal: the golden envelope and result give `kComplete`. |
| RB4 | Determinism: two replays into fresh adapters give identical steps, traces, stats and chains. A motion's `payload_sha256` equals `ComputeOriginalIntentDigest`. |
| RB5 | Compatibility (all three): `schema_version 2` and a wrong `config_digest`, each on an owned kind, an ignored kind and a metadata event, latch. Then `kLatched`, and `Finish` reports the latched status. |
| RB6 | Invalid payloads (all three): an invalid trajectory, motion, decision, envelope and result, and empty messages, give `kOk` with `accepted == false` and the policy error number. None enters a chain or pending. |
| RB7 | Intent linkage: a decision before its motion is unlinked and degraded. A decision for an earlier rejected motion is linked. |
| RB8 | Late: a result deadline strictly before its inner `receive_time` is `kReplayExpired`, unchanged and chained. An equal deadline is `kComplete`. |
| RB9 | Missing: a pending envelope whose deadline passes before a later record's inner `receive_time` (an envelope, or a result with another key) is `kMissing` with `expired`, swept before that record. Still pending at `Finish` is `expired == false`. |
| RB10 | Orphan and FIFO: a result with no pending envelope is `kReplayMiss` and not chained. Two same-key envelopes pair with the results oldest first. |
| RB11 | Model digest: differing non-empty digests set `model_digest_mismatch` and `degraded`. An empty envelope digest is not flagged. |
| RB12 | Ignored kinds and loss (all three): unowned kinds are ignored, and a writer overflow gives `kLossRecord`, `degraded`, and replay continues. |
| RB13 | Boundaries (all three): an empty episode, a single event, `kFinished` after `Finish()`, and an idempotent `Finish()`. |
| RB14 | Config and isolation (all three): a bad `expected_config_digest` is `kInvalidConfig`. Each config is the size of `ReplayCompatibilityConfig`, so there is nothing to inject, and the test reads `BUILD` to assert the adapter's `deps` have no queue, planner, worker, `motion_planning` or `rule_aggregation` label. |

### Limitations

- Verify only. There is no re-planning and no re-aggregation. The episode
  schema holds no planner start state, bounds, fence or dynamics, and no
  per-rule `SafetyRuleResult`.
- Linkage proves reference, not safety correctness. A digest in the seen set
  shows that a motion with that digest was replayed earlier, not that the
  decision is right.
- The inference pairing and expiry mirror the #120 predicate
  (`intrinsic_inference/queue`: exact `(model_id, input_digest, state_epoch)`
  key, `kReplayExpired` when the recorded deadline is strictly before `now`).
  Any #120 change needs a matching change here.
- The producer's inner `receive_time` is trusted (P4-A). A result whose
  `receive_time` is absent is judged with `now` read as stored (the epoch), so
  it skips the sweep and is `kComplete`.
- Loss records cannot attribute the lost stream, so `degraded` is episode-wide.
- Out of scope: schema changes, re-planning, re-running safety aggregation, the
  inference queue or worker, Python, #240, `.gitattributes`, and manipulator
  targets.
