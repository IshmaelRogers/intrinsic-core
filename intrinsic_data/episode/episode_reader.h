// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_READER_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_READER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic_data/episode/episode_event_policy.h"

namespace intrinsic::episode {

// Whole-buffer episode reader with corruption detection. Contract: lock id
// episode-reader-v1 (#129). File format and semantics:
// intrinsic_data/episode/README.md.
//
// Host-only. Never call this from an ICON cycle, never link it into a
// real-time part. It reads no clock, starts no thread, and does not retry.

inline constexpr size_t kDefaultEpisodeReaderMaxEventBytes = 16 * 1024 * 1024;
inline constexpr uint64_t kDefaultEpisodeReaderMaxRecords = 1'000'000;
inline constexpr uint64_t kMaxEpisodeReaderMaxRecords = 16'777'216;
inline constexpr uint64_t kDefaultEpisodeReaderMaxFileBytes = uint64_t{1} << 30;
inline constexpr uint64_t kMaxEpisodeReaderMaxFileBytes = uint64_t{4} << 30;

// Serialized size of the largest possible writer loss record, computed exactly
// as EpisodeWriter::Create's `largest` template. A test pins its value.
size_t MaxEpisodeLossRecordBytes();

// Append-only. Numbers are permanent.
enum class EpisodeReaderStatus {
  kOk = 0,
  kInvalidConfig = 1,
  kInputTooLarge = 2,
};

// Append-only. Numbers are permanent.
enum class EpisodeReadDefect {
  kNone = 0,
  kTruncatedHeader = 1,
  kBadMagic = 2,
  kUnsupportedVersion = 3,
  kTruncatedRecord = 4,
  kLengthOutOfRange = 5,
  kCrcMismatch = 6,
  kParseFailed = 7,
  kInvalidEvent = 8,
  kReservedEvent = 9,
  kLossRecordInvalid = 10,
  kIndexNotIncreasing = 11,
  kIndexGap = 12,
  kOrderViolation = 13,
  kTooManySources = 14,
  kTooManyRecords = 15,
};

struct EpisodeReaderConfig {
  // [1, 16 MiB]
  size_t max_event_bytes = kDefaultEpisodeReaderMaxEventBytes;
  // [1, 16777216]
  uint64_t max_records = kDefaultEpisodeReaderMaxRecords;
  // [12, 4 GiB]
  uint64_t max_file_bytes = kDefaultEpisodeReaderMaxFileBytes;
};

// Each part is optional. The parts are AND-ed.
struct EpisodeReadFilter {
  // Empty means all.
  std::vector<std::string> source_ids;
  // Empty means all.
  std::vector<PayloadKind> payload_kinds;
  bool has_index_range = false;
  // Inclusive.
  uint64_t first_writer_index = 0;
  // Inclusive.
  uint64_t last_writer_index = 0;
};

// A lightweight view. The list of entries is the index.
struct EpisodeRecordEntry {
  // Byte offset of the record header in the file.
  uint64_t offset = 0;
  uint64_t writer_index = 0;
  uint32_t crc32c = 0;
  // Into the caller's buffer, the exact bytes.
  absl::Span<const uint8_t> event_bytes;
  PayloadKind payload_kind = PayloadKind::kNone;
  // A valid unknown payload (#127 rule).
  bool skip = false;
  std::string source_id;
};

struct EpisodeLossEntry {
  // The loss record's own index (== last + 1).
  uint64_t writer_index = 0;
  uint64_t first_dropped_writer_index = 0;
  uint64_t last_dropped_writer_index = 0;
  uint64_t dropped_count = 0;
  // An EpisodeLoss::LossReason number, kept open.
  int reason = 0;
  uint64_t sequence = 0;
};

// Covers the whole valid prefix and ignores the filter. `records` counts every
// valid record, loss records and skipped unknown payloads included, so it
// equals EpisodeReadResult::good_records.
struct EpisodeReadStats {
  uint64_t records = 0;
  uint64_t loss_records = 0;
  uint64_t unknown_skipped = 0;
  uint64_t dropped_overflow = 0;
  uint64_t dropped_too_large = 0;
  uint64_t dropped_other_reason = 0;
  uint64_t producer_sources = 0;
};

struct EpisodeReadResult {
  EpisodeReaderStatus status = EpisodeReaderStatus::kOk;
  EpisodeReadDefect defect = EpisodeReadDefect::kNone;
  // Set for kInvalidEvent.
  EpisodeEventError event_error = EpisodeEventError::kNone;
  // Start of the defective header or record. 0 if none.
  uint64_t defect_offset = 0;
  // Valid records before the defect, loss records included.
  uint64_t good_records = 0;
  // Just past the last valid record. 12 for a header-only file, 0 for a bad
  // header.
  uint64_t good_end_offset = 0;
  bool has_last_good_index = false;
  uint64_t last_good_writer_index = 0;
  uint64_t next_expected_writer_index = 0;
  // Valid records that pass the filter, in file order.
  std::vector<EpisodeRecordEntry> records;
  // Every valid loss record, whatever the filter.
  std::vector<EpisodeLossEntry> losses;
  EpisodeReadStats stats;
};

// Pure. The returned spans point into `bytes`, which must outlive the result.
//
// An invalid config gives kInvalidConfig and nothing else is filled. A buffer
// larger than config.max_file_bytes gives kInputTooLarge. In both cases no
// record is read and `defect` stays kNone. An index range with
// first_writer_index > last_writer_index is kInvalidConfig. A filter that
// names an unknown source or kind matches nothing and is not an error.
//
// v1 has no trailer: kNone means "structurally intact prefix", not "cleanly
// closed episode".
EpisodeReadResult ReadEpisode(absl::Span<const uint8_t> bytes,
                              const EpisodeReaderConfig& config,
                              const EpisodeReadFilter& filter = {});

// fstat first. A size above max_file_bytes is FailedPreconditionError with no
// read. An invalid max_file_bytes is InvalidArgumentError. I/O errors map like
// FileEpisodeSink. Read loops over partial reads and EINTR. O_RDONLY |
// O_CLOEXEC.
absl::StatusOr<std::string> LoadEpisodeFile(const std::string& path,
                                            uint64_t max_file_bytes);

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_READER_H_
