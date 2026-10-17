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

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_WRITER_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_WRITER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"

namespace intrinsic::episode {

// Bounded, host-only episode writer. Contract: lock id
// episode-writer-bounded-v1 (#128). File format and semantics:
// intrinsic_data/episode/README.md.
//
// Host-only. Never call this from an ICON cycle, never link it into a
// real-time part. It reads no clock, starts no thread, and does not retry.

inline constexpr char kEpisodeFileMagic[8] = {'I', 'E', 'P', 'I',
                                              'S', 'O', 'D', '1'};
inline constexpr uint32_t kEpisodeFormatVersion = 1;
// magic + u32 version.
inline constexpr size_t kEpisodeFileHeaderBytes = 12;
// u32 len, u32 crc, u64 index.
inline constexpr size_t kEpisodeRecordHeaderBytes = 16;
// Ring slots, including the reserved slot.
inline constexpr size_t kDefaultEpisodeWriterCapacity = 1024;
inline constexpr size_t kDefaultEpisodeMaxEventBytes = 64 * 1024;
// 1 producer slot + 1 reserved.
inline constexpr size_t kMinEpisodeWriterCapacity = 2;
inline constexpr size_t kMaxEpisodeWriterCapacity = 65536;
inline constexpr size_t kMinEpisodeMaxEventBytes = 1;
inline constexpr size_t kMaxEpisodeMaxEventBytes = 16 * 1024 * 1024;
inline constexpr size_t kMaxEpisodeSources = 64;
// Reserved: producers cannot use this source_id.
inline constexpr std::string_view kEpisodeWriterSourceId = "episode_writer";
inline constexpr std::string_view kEpisodeWriterFrameId = "episode_writer";

// Append-only. Numbers are permanent.
enum class EpisodeWriterStatus {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidEvent = 2,
  kReservedEvent = 3,
  kOutOfOrder = 4,
  kTooManySources = 5,
  kBufferFull = 6,
  kEventTooLarge = 7,
  kWriteFailed = 8,
  kClosed = 9,
  kCancelled = 10,
};

struct EpisodeWriterConfig {
  size_t capacity = kDefaultEpisodeWriterCapacity;
  size_t max_event_bytes = kDefaultEpisodeMaxEventBytes;
};

struct EpisodeAppendResult {
  EpisodeWriterStatus status = EpisodeWriterStatus::kInvalidEvent;
  // Set for kInvalidEvent.
  EpisodeEventError event_error = EpisodeEventError::kNone;
  // Valid for kOk, kBufferFull, and kEventTooLarge.
  uint64_t writer_index = 0;
};

// Plain value snapshot.
struct EpisodeWriterStats {
  uint64_t accepted = 0;
  uint64_t loss_records = 0;
  uint64_t written_records = 0;
  uint64_t dropped_overflow = 0;
  uint64_t dropped_too_large = 0;
  uint64_t dropped_on_cancel = 0;
  uint64_t rejected_invalid = 0;
  uint64_t rejected_reserved = 0;
  uint64_t rejected_out_of_order = 0;
  uint64_t rejected_too_many_sources = 0;
  uint64_t rejected_after_stop = 0;
  // Records in the ring, not yet written.
  uint64_t queued_records = 0;
  bool pending_loss = false;
  uint64_t next_writer_index = 0;
};

class EpisodeSink {
 public:
  virtual ~EpisodeSink() = default;
  // Writes all the bytes or returns an error.
  virtual absl::Status Write(absl::Span<const uint8_t> bytes) = 0;
  virtual absl::Status Sync() = 0;
};

// POSIX open, write, and fsync.
class FileEpisodeSink final : public EpisodeSink {
 public:
  // O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, mode 0644. Never overwrites an
  // existing file.
  static absl::StatusOr<std::unique_ptr<FileEpisodeSink>> Open(
      const std::string& path);

  FileEpisodeSink(const FileEpisodeSink&) = delete;
  FileEpisodeSink& operator=(const FileEpisodeSink&) = delete;
  ~FileEpisodeSink() override;

  // Loops over partial writes and EINTR.
  absl::Status Write(absl::Span<const uint8_t> bytes) override;
  // fsync.
  absl::Status Sync() override;

 private:
  explicit FileEpisodeSink(int fd) : fd_(fd) {}

  int fd_;
};

class EpisodeWriter {
 public:
  // Returns nullptr and sets *status to kInvalidConfig for a null sink or an
  // out-of-range config. Never clamps. *status is kOk on success. The sink is
  // not owned and must outlive the writer.
  static std::unique_ptr<EpisodeWriter> Create(
      const EpisodeWriterConfig& config, EpisodeSink* sink,
      EpisodeWriterStatus* status);

  EpisodeWriter(const EpisodeWriter&) = delete;
  EpisodeWriter& operator=(const EpisodeWriter&) = delete;
  ~EpisodeWriter();

  // May run from any thread. Performs no I/O, no blocking wait, and no heap
  // allocation, except for a case-14 EpisodeMetadata event with non-empty text,
  // which allocates once inside ComputeConfigDigest.
  EpisodeAppendResult Append(
      const intrinsic_proto::episode::EpisodeEvent& event);

  // Flush, Close, and Cancel must come from one owner thread, and not
  // concurrently with each other.
  EpisodeWriterStatus Flush();
  EpisodeWriterStatus Close();
  // Returns the number of records discarded by this call.
  uint64_t Cancel();

  EpisodeWriterStats Stats() const;
  // OkStatus until a sink error latches.
  absl::Status FirstSinkError() const;

 private:
  struct Impl;
  explicit EpisodeWriter(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_WRITER_H_
