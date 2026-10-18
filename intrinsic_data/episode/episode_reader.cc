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

#include "intrinsic_data/episode/episode_reader.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "absl/crc/crc32c.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_event_view.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::EpisodeLoss;

static_assert(static_cast<int>(EpisodeEvent::kEpisodeLoss) ==
              kPayloadCaseEpisodeLoss);

// The merged IEPISOD1 v1 format (#128), as the writer defines it. The reader
// does not link the writer, so the values are repeated here and the reader
// test checks them against the writer's constants.
constexpr uint8_t kMagic[8] = {'I', 'E', 'P', 'I', 'S', 'O', 'D', '1'};
constexpr size_t kFileHeaderBytes = 12;
constexpr size_t kRecordHeaderBytes = 16;
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kMaxProducerSources = 64;
constexpr std::string_view kLossSourceId = "episode_writer";
constexpr std::string_view kLossFrameId = "episode_writer";

uint32_t GetU32(const uint8_t* in) {
  uint32_t value = 0;
  for (int i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(in[i]) << (8 * i);
  }
  return value;
}

uint64_t GetU64(const uint8_t* in) {
  uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value |= static_cast<uint64_t>(in[i]) << (8 * i);
  }
  return value;
}

absl::Status ErrnoStatus(int error, const std::string& what) {
  const std::string message =
      what + ": " + std::error_code(error, std::generic_category()).message();
  switch (error) {
    case EEXIST:
      return absl::AlreadyExistsError(message);
    case ENOENT:
      return absl::NotFoundError(message);
    case EACCES:
    case EPERM:
      return absl::PermissionDeniedError(message);
    default:
      return absl::InternalError(message);
  }
}

bool ConfigValid(const EpisodeReaderConfig& config) {
  return config.max_event_bytes >= 1 &&
         config.max_event_bytes <= kDefaultEpisodeReaderMaxEventBytes &&
         config.max_records >= 1 &&
         config.max_records <= kMaxEpisodeReaderMaxRecords &&
         config.max_file_bytes >= kFileHeaderBytes &&
         config.max_file_bytes <= kMaxEpisodeReaderMaxFileBytes;
}

bool FilterMatches(const EpisodeReadFilter& filter, std::string_view source_id,
                   PayloadKind kind, uint64_t writer_index) {
  if (!filter.source_ids.empty() &&
      std::find(filter.source_ids.begin(), filter.source_ids.end(),
                source_id) == filter.source_ids.end()) {
    return false;
  }
  if (!filter.payload_kinds.empty() &&
      std::find(filter.payload_kinds.begin(), filter.payload_kinds.end(),
                kind) == filter.payload_kinds.end()) {
    return false;
  }
  if (filter.has_index_range && (writer_index < filter.first_writer_index ||
                                 writer_index > filter.last_writer_index)) {
    return false;
  }
  return true;
}

struct ProducerSource {
  std::string id;
  uint64_t last_sequence = 0;
};

// Returns kNone when the loss record is valid. Updates nothing.
EpisodeReadDefect CheckLossRecord(const EpisodeEvent& event,
                                  uint64_t writer_index, uint64_t expected,
                                  uint64_t previous_loss_sequence) {
  const EpisodeLoss& loss = event.episode_loss();
  const uint64_t first = loss.first_dropped_writer_index();
  const uint64_t last = loss.last_dropped_writer_index();
  if (first != expected || last < first ||
      last == std::numeric_limits<uint64_t>::max() ||
      writer_index != last + 1 || loss.dropped_count() != last - first + 1 ||
      loss.reason() == EpisodeLoss::LOSS_REASON_UNSPECIFIED ||
      previous_loss_sequence == std::numeric_limits<uint64_t>::max() ||
      event.header().sequence() != previous_loss_sequence + 1 ||
      event.header().source_time().seconds() !=
          event.header().receive_time().seconds() ||
      event.header().source_time().nanos() !=
          event.header().receive_time().nanos()) {
    return EpisodeReadDefect::kLossRecordInvalid;
  }
  // The next expected index must not wrap.
  if (writer_index == std::numeric_limits<uint64_t>::max()) {
    return EpisodeReadDefect::kLossRecordInvalid;
  }
  return EpisodeReadDefect::kNone;
}

}  // namespace

size_t MaxEpisodeLossRecordBytes() {
  static const size_t kBytes = [] {
    EpisodeEvent loss_template;
    auto* header = loss_template.mutable_header();
    header->set_source_id(std::string(kLossSourceId));
    header->set_frame_id(std::string(kLossFrameId));
    header->mutable_source_time();
    header->mutable_receive_time();
    loss_template.set_schema_version(1);
    loss_template.mutable_episode_loss();

    EpisodeEvent largest = loss_template;
    auto* largest_header = largest.mutable_header();
    largest_header->set_sequence(std::numeric_limits<uint64_t>::max());
    for (auto* time : {largest_header->mutable_source_time(),
                       largest_header->mutable_receive_time()}) {
      time->set_seconds(std::numeric_limits<int64_t>::min());
      time->set_nanos(std::numeric_limits<int32_t>::max());
    }
    auto* largest_loss = largest.mutable_episode_loss();
    largest_loss->set_dropped_count(std::numeric_limits<uint64_t>::max());
    largest_loss->set_first_dropped_writer_index(
        std::numeric_limits<uint64_t>::max());
    largest_loss->set_last_dropped_writer_index(
        std::numeric_limits<uint64_t>::max());
    largest_loss->set_reason(EpisodeLoss::LOSS_REASON_TOO_LARGE);
    return static_cast<size_t>(largest.ByteSizeLong());
  }();
  return kBytes;
}

EpisodeReadResult ReadEpisode(absl::Span<const uint8_t> bytes,
                              const EpisodeReaderConfig& config,
                              const EpisodeReadFilter& filter) {
  EpisodeReadResult result;
  if (!ConfigValid(config) ||
      (filter.has_index_range &&
       filter.first_writer_index > filter.last_writer_index)) {
    result.status = EpisodeReaderStatus::kInvalidConfig;
    return result;
  }
  if (bytes.size() > config.max_file_bytes) {
    result.status = EpisodeReaderStatus::kInputTooLarge;
    return result;
  }

  if (bytes.size() < kFileHeaderBytes) {
    result.defect = EpisodeReadDefect::kTruncatedHeader;
    return result;
  }
  if (!std::equal(std::begin(kMagic), std::end(kMagic), bytes.begin())) {
    result.defect = EpisodeReadDefect::kBadMagic;
    return result;
  }
  if (GetU32(bytes.data() + sizeof(kMagic)) != kFormatVersion) {
    result.defect = EpisodeReadDefect::kUnsupportedVersion;
    return result;
  }
  result.good_end_offset = kFileHeaderBytes;

  const uint64_t max_length =
      std::max<uint64_t>(config.max_event_bytes, MaxEpisodeLossRecordBytes());
  const uint64_t total = bytes.size();

  EpisodeEvent scratch;
  std::vector<ProducerSource> sources;
  uint64_t expected = 0;
  uint64_t loss_ordinal = 0;

  while (result.good_end_offset < total) {
    const uint64_t offset = result.good_end_offset;
    const uint64_t remaining = total - offset;
    const uint8_t* record = bytes.data() + offset;

    auto fail = [&](EpisodeReadDefect defect) -> EpisodeReadResult {
      result.defect = defect;
      result.defect_offset = offset;
      return std::move(result);
    };

    if (remaining < kRecordHeaderBytes) {
      return fail(EpisodeReadDefect::kTruncatedRecord);
    }
    const uint32_t length = GetU32(record);
    const uint32_t stored_crc = GetU32(record + 4);
    const uint64_t writer_index = GetU64(record + 8);
    if (length == 0 || length > max_length) {
      return fail(EpisodeReadDefect::kLengthOutOfRange);
    }
    if (remaining < kRecordHeaderBytes + length) {
      return fail(EpisodeReadDefect::kTruncatedRecord);
    }
    const uint8_t* event_data = record + kRecordHeaderBytes;
    const uint32_t actual_crc = static_cast<uint32_t>(absl::ComputeCrc32c(
        std::string_view(reinterpret_cast<const char*>(event_data), length)));
    if (actual_crc != stored_crc) {
      return fail(EpisodeReadDefect::kCrcMismatch);
    }
    if (!scratch.ParseFromArray(event_data, static_cast<int>(length))) {
      return fail(EpisodeReadDefect::kParseFailed);
    }

    const EpisodeEventView view = EpisodeEventViewOf(scratch);
    const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
    if (!assessment.engaged || assessment.error != EpisodeEventError::kNone) {
      result.event_error = assessment.error;
      return fail(EpisodeReadDefect::kInvalidEvent);
    }

    const bool loss_source = view.source_id == kLossSourceId;
    const bool loss_case = view.payload_case == kPayloadCaseEpisodeLoss;
    if (loss_source != loss_case) {
      return fail(EpisodeReadDefect::kReservedEvent);
    }

    ProducerSource* source = nullptr;
    uint64_t next_expected = 0;
    if (loss_case) {
      const EpisodeReadDefect defect =
          CheckLossRecord(scratch, writer_index, expected, loss_ordinal);
      if (defect != EpisodeReadDefect::kNone) {
        return fail(defect);
      }
      next_expected = writer_index + 1;
    } else {
      if (writer_index < expected) {
        return fail(EpisodeReadDefect::kIndexNotIncreasing);
      }
      if (writer_index > expected) {
        return fail(EpisodeReadDefect::kIndexGap);
      }
      // No index can follow the largest one without wrapping.
      if (writer_index == std::numeric_limits<uint64_t>::max()) {
        return fail(EpisodeReadDefect::kIndexNotIncreasing);
      }
      next_expected = writer_index + 1;

      const uint64_t sequence = scratch.header().sequence();
      for (ProducerSource& candidate : sources) {
        if (candidate.id == view.source_id) {
          source = &candidate;
          break;
        }
      }
      if (source != nullptr) {
        if (!embodiment::SequenceAdvances(source->last_sequence, sequence)) {
          return fail(EpisodeReadDefect::kOrderViolation);
        }
      } else if (sources.size() >= kMaxProducerSources) {
        return fail(EpisodeReadDefect::kTooManySources);
      }
    }

    if (result.good_records >= config.max_records) {
      return fail(EpisodeReadDefect::kTooManyRecords);
    }

    if (loss_case) {
      const EpisodeLoss& loss = scratch.episode_loss();
      EpisodeLossEntry entry;
      entry.writer_index = writer_index;
      entry.first_dropped_writer_index = loss.first_dropped_writer_index();
      entry.last_dropped_writer_index = loss.last_dropped_writer_index();
      entry.dropped_count = loss.dropped_count();
      entry.reason = static_cast<int>(loss.reason());
      entry.sequence = scratch.header().sequence();
      result.losses.push_back(entry);
      ++loss_ordinal;
      ++result.stats.loss_records;
      switch (loss.reason()) {
        case EpisodeLoss::LOSS_REASON_OVERFLOW:
          result.stats.dropped_overflow += entry.dropped_count;
          break;
        case EpisodeLoss::LOSS_REASON_TOO_LARGE:
          result.stats.dropped_too_large += entry.dropped_count;
          break;
        default:
          result.stats.dropped_other_reason += entry.dropped_count;
          break;
      }
    } else {
      if (source == nullptr) {
        sources.push_back({std::string(view.source_id), 0});
        source = &sources.back();
      }
      source->last_sequence = scratch.header().sequence();
      result.stats.producer_sources = sources.size();
    }
    if (assessment.skip) {
      ++result.stats.unknown_skipped;
    }
    ++result.stats.records;

    ++result.good_records;
    result.good_end_offset = offset + kRecordHeaderBytes + length;
    result.has_last_good_index = true;
    result.last_good_writer_index = writer_index;
    expected = next_expected;
    result.next_expected_writer_index = expected;

    if (FilterMatches(filter, view.source_id, assessment.payload_kind,
                      writer_index)) {
      EpisodeRecordEntry entry;
      entry.offset = offset;
      entry.writer_index = writer_index;
      entry.crc32c = stored_crc;
      entry.event_bytes = absl::MakeConstSpan(event_data, length);
      entry.payload_kind = assessment.payload_kind;
      entry.skip = assessment.skip;
      entry.source_id = std::string(view.source_id);
      result.records.push_back(std::move(entry));
    }
  }
  return result;
}

absl::StatusOr<std::string> LoadEpisodeFile(const std::string& path,
                                            uint64_t max_file_bytes) {
  if (max_file_bytes < kFileHeaderBytes ||
      max_file_bytes > kMaxEpisodeReaderMaxFileBytes) {
    return absl::InvalidArgumentError("max_file_bytes must be in [12, 4 GiB]");
  }
  int fd;
  do {
    fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  } while (fd < 0 && errno == EINTR);
  if (fd < 0) {
    return ErrnoStatus(errno, "open " + path);
  }

  struct FdCloser {
    int fd;
    ~FdCloser() { ::close(fd); }
  } closer{fd};

  struct stat info;
  while (::fstat(fd, &info) != 0) {
    if (errno != EINTR) {
      return ErrnoStatus(errno, "fstat " + path);
    }
  }
  const uint64_t size = static_cast<uint64_t>(info.st_size);
  if (size > max_file_bytes) {
    return absl::FailedPreconditionError(
        "episode file " + path + " has " + std::to_string(size) +
        " bytes, above max_file_bytes " + std::to_string(max_file_bytes));
  }

  std::string contents;
  try {
    contents.resize(static_cast<size_t>(size));
  } catch (const std::bad_alloc&) {
    return absl::ResourceExhaustedError("cannot allocate " +
                                        std::to_string(size) + " bytes");
  }
  size_t read_total = 0;
  while (read_total < contents.size()) {
    const ssize_t n =
        ::read(fd, contents.data() + read_total, contents.size() - read_total);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return ErrnoStatus(errno, "read " + path);
    }
    if (n == 0) {
      break;
    }
    read_total += static_cast<size_t>(n);
  }
  contents.resize(read_total);
  return contents;
}

}  // namespace intrinsic::episode
