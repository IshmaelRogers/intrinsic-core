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
[Episode reader](#episode-reader)) reads it back and detects corruption. There
is still no replay clock or digest equality (#130 to #132).

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
| 10 | case 10 to 15 known. Case not set with unknown fields is skippable (`kNone`, `payload_kind = kUnknown`, `skip = true`). Otherwise not set. | `kMissingPayload` (10) |

A defective event is never skipped, even when its payload is unknown.

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

```
bazel test //intrinsic_data/episode/...
```
