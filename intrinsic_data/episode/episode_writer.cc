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

#include "intrinsic_data/episode/episode_writer.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "absl/crc/crc32c.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "google/protobuf/message.h"
#include "google/protobuf/unknown_field_set.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::EpisodeLoss;

static_assert(static_cast<int>(EpisodeEvent::kEpisodeLoss) ==
              kPayloadCaseEpisodeLoss);

void PutU32(uint32_t value, uint8_t* out) {
  for (int i = 0; i < 4; ++i) {
    out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xff);
  }
}

void PutU64(uint64_t value, uint8_t* out) {
  for (int i = 0; i < 8; ++i) {
    out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xff);
  }
}

// Same fields as the #127 serialization tests fill.
EpisodeEventView ViewOf(const EpisodeEvent& message) {
  const auto& header = message.header();
  EpisodeEventView view;
  view.header_present = message.has_header();
  view.source_id = header.source_id();
  view.frame_id = header.frame_id();
  view.source_time_present = header.has_source_time();
  view.source_time = {header.source_time().seconds(),
                      header.source_time().nanos()};
  view.receive_time_present = header.has_receive_time();
  view.receive_time = {header.receive_time().seconds(),
                       header.receive_time().nanos()};
  view.schema_version = message.schema_version();
  view.snapshot_id = message.snapshot_id();
  view.config_digest = message.config_digest();
  view.provenance_present = message.has_provenance();
  view.payload_case = static_cast<int>(message.payload_case());
  view.has_unknown_fields =
      !message.GetReflection()->GetUnknownFields(message).empty();
  if (message.has_episode_metadata()) {
    view.marine_randomization_metadata =
        message.episode_metadata().marine_randomization_metadata();
  }
  return view;
}

absl::Status ErrnoStatus(int error, std::string_view what) {
  const std::string message = absl::StrCat(
      what, ": ", std::error_code(error, std::generic_category()).message());
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

}  // namespace

absl::StatusOr<std::unique_ptr<FileEpisodeSink>> FileEpisodeSink::Open(
    const std::string& path) {
  int fd;
  do {
    fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  } while (fd < 0 && errno == EINTR);
  if (fd < 0) {
    return ErrnoStatus(errno, absl::StrCat("open ", path));
  }
  return std::unique_ptr<FileEpisodeSink>(new FileEpisodeSink(fd));
}

FileEpisodeSink::~FileEpisodeSink() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

absl::Status FileEpisodeSink::Write(absl::Span<const uint8_t> bytes) {
  while (!bytes.empty()) {
    const ssize_t written = ::write(fd_, bytes.data(), bytes.size());
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      return ErrnoStatus(errno, "write");
    }
    if (written == 0) {
      return absl::InternalError("write: returned zero bytes");
    }
    bytes.remove_prefix(static_cast<size_t>(written));
  }
  return absl::OkStatus();
}

absl::Status FileEpisodeSink::Sync() {
  while (::fsync(fd_) != 0) {
    if (errno != EINTR) {
      return ErrnoStatus(errno, "fsync");
    }
  }
  return absl::OkStatus();
}

struct EpisodeWriter::Impl {
  struct Source {
    std::string id;
    uint64_t last_sequence = 0;
  };

  struct PendingLoss {
    uint64_t first_index = 0;
    uint64_t last_index = 0;
    uint64_t count = 0;
    EpisodeLoss::LossReason reason = EpisodeLoss::LOSS_REASON_UNSPECIFIED;
    int64_t receive_seconds = 0;
    int32_t receive_nanos = 0;
  };

  Impl(EpisodeSink* sink_in, size_t capacity_in, size_t max_event_bytes_in,
       size_t slot_bytes_in)
      : sink(sink_in),
        capacity(capacity_in),
        max_event_bytes(max_event_bytes_in),
        slot_bytes(slot_bytes_in) {}

  uint8_t* SlotAt(size_t slot) { return storage.get() + slot * slot_bytes; }

  // Stores one serialized event as the newest queued record. The caller holds
  // the mutex, has checked there is room, and has checked n against the
  // slot size.
  void StoreRecordLocked(const EpisodeEvent& event, size_t n, uint64_t index) {
    const size_t slot = (head + queued) % capacity;
    uint8_t* bytes = SlotAt(slot);
    event.SerializeToArray(bytes + kEpisodeRecordHeaderBytes,
                           static_cast<int>(n));
    const uint32_t crc =
        static_cast<uint32_t>(absl::ComputeCrc32c(std::string_view(
            reinterpret_cast<const char*>(bytes + kEpisodeRecordHeaderBytes),
            n)));
    PutU32(static_cast<uint32_t>(n), bytes);
    PutU32(crc, bytes + 4);
    PutU64(index, bytes + 8);
    slot_length[slot] = kEpisodeRecordHeaderBytes + n;
    ++queued;
  }

  // Moves the pending loss into the ring at the next writer index. The caller
  // holds the mutex. The ring always has a free slot here (see README.md).
  void MaterializeLossLocked() {
    ++loss_ordinal;
    auto* header = loss_template.mutable_header();
    header->set_sequence(loss_ordinal);
    for (auto* time :
         {header->mutable_source_time(), header->mutable_receive_time()}) {
      time->set_seconds(pending_loss.receive_seconds);
      time->set_nanos(pending_loss.receive_nanos);
    }
    auto* loss = loss_template.mutable_episode_loss();
    loss->set_dropped_count(pending_loss.count);
    loss->set_first_dropped_writer_index(pending_loss.first_index);
    loss->set_last_dropped_writer_index(pending_loss.last_index);
    loss->set_reason(pending_loss.reason);
    StoreRecordLocked(loss_template, loss_template.ByteSizeLong(),
                      next_index++);
    pending = false;
    ++stats.loss_records;
  }

  // Consumes the next writer index for a dropped event and extends or starts
  // the pending loss. The caller holds the mutex.
  uint64_t DropLocked(EpisodeLoss::LossReason reason,
                      const EpisodeEventView& view) {
    if (pending && pending_loss.reason != reason) {
      MaterializeLossLocked();
    }
    const uint64_t index = next_index++;
    if (!pending) {
      pending = true;
      pending_loss.first_index = index;
      pending_loss.count = 0;
      pending_loss.reason = reason;
    }
    pending_loss.last_index = index;
    ++pending_loss.count;
    pending_loss.receive_seconds = view.receive_time.seconds;
    pending_loss.receive_nanos = view.receive_time.nanos;
    return index;
  }

  Source* FindSourceLocked(std::string_view id) {
    for (size_t i = 0; i < source_count; ++i) {
      if (sources[i].id == id) {
        return &sources[i];
      }
    }
    return nullptr;
  }

  void LatchLocked(const absl::Status& status) {
    if (!latched) {
      latched = true;
      first_error = status;
    }
  }

  void Latch(const absl::Status& status) {
    std::lock_guard<std::mutex> lock(mutex);
    LatchLocked(status);
  }

  EpisodeAppendResult Append(const EpisodeEvent& event) {
    std::lock_guard<std::mutex> lock(mutex);
    EpisodeAppendResult result;
    if (cancelled || closed || latched) {
      result.status = cancelled ? EpisodeWriterStatus::kCancelled
                      : closed  ? EpisodeWriterStatus::kClosed
                                : EpisodeWriterStatus::kWriteFailed;
      ++stats.rejected_after_stop;
      return result;
    }

    const EpisodeEventView view = ViewOf(event);
    const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
    if (!assessment.engaged || assessment.error != EpisodeEventError::kNone) {
      result.status = EpisodeWriterStatus::kInvalidEvent;
      result.event_error = assessment.error;
      ++stats.rejected_invalid;
      return result;
    }

    if (view.source_id == kEpisodeWriterSourceId ||
        view.payload_case == kPayloadCaseEpisodeLoss) {
      result.status = EpisodeWriterStatus::kReservedEvent;
      ++stats.rejected_reserved;
      return result;
    }

    const uint64_t sequence = event.header().sequence();
    Source* source = FindSourceLocked(view.source_id);
    if (source != nullptr) {
      if (!embodiment::SequenceAdvances(source->last_sequence, sequence)) {
        result.status = EpisodeWriterStatus::kOutOfOrder;
        ++stats.rejected_out_of_order;
        return result;
      }
    } else if (source_count >= kMaxEpisodeSources) {
      result.status = EpisodeWriterStatus::kTooManySources;
      ++stats.rejected_too_many_sources;
      return result;
    }

    if (queued + (pending ? 1 : 0) + 1 > capacity - 1) {
      result.status = EpisodeWriterStatus::kBufferFull;
      result.writer_index = DropLocked(EpisodeLoss::LOSS_REASON_OVERFLOW, view);
      ++stats.dropped_overflow;
      return result;
    }

    const size_t size = event.ByteSizeLong();
    if (size > max_event_bytes) {
      result.status = EpisodeWriterStatus::kEventTooLarge;
      result.writer_index =
          DropLocked(EpisodeLoss::LOSS_REASON_TOO_LARGE, view);
      ++stats.dropped_too_large;
      return result;
    }

    if (pending) {
      MaterializeLossLocked();
    }
    const uint64_t index = next_index++;
    StoreRecordLocked(event, size, index);
    if (source == nullptr) {
      source = &sources[source_count++];
      source->id.assign(view.source_id);
    }
    source->last_sequence = sequence;
    ++stats.accepted;
    result.status = EpisodeWriterStatus::kOk;
    result.writer_index = index;
    return result;
  }

  // Writes the oldest n queued records, one sink call each. The sink is called
  // without the mutex. Slots being written stay counted as queued, so Append
  // never overwrites them. Returns false once a sink error is latched.
  bool WriteQueued(size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* bytes;
      size_t length;
      {
        std::lock_guard<std::mutex> lock(mutex);
        bytes = SlotAt(head);
        length = slot_length[head];
      }
      const absl::Status status =
          sink->Write(absl::MakeConstSpan(bytes, length));
      if (!status.ok()) {
        Latch(status);
        return false;
      }
      std::lock_guard<std::mutex> lock(mutex);
      head = (head + 1) % capacity;
      --queued;
      ++stats.written_records;
    }
    return true;
  }

  // Writes the file header once, then the queued records and then the pending
  // loss. With until_empty, repeats until nothing is left. Owner thread only.
  EpisodeWriterStatus Drain(bool until_empty) {
    if (!header_written) {
      uint8_t header[kEpisodeFileHeaderBytes];
      std::copy(std::begin(kEpisodeFileMagic), std::end(kEpisodeFileMagic),
                header);
      PutU32(kEpisodeFormatVersion, header + sizeof(kEpisodeFileMagic));
      const absl::Status status = sink->Write(absl::MakeConstSpan(header));
      if (!status.ok()) {
        Latch(status);
        return EpisodeWriterStatus::kWriteFailed;
      }
      header_written = true;
    }
    for (;;) {
      size_t snapshot;
      {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot = queued;
      }
      if (!WriteQueued(snapshot)) {
        return EpisodeWriterStatus::kWriteFailed;
      }
      size_t loss_records = 0;
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (queued == 0 && pending) {
          MaterializeLossLocked();
          loss_records = 1;
        }
      }
      if (!WriteQueued(loss_records)) {
        return EpisodeWriterStatus::kWriteFailed;
      }
      if (!until_empty) {
        return EpisodeWriterStatus::kOk;
      }
      std::lock_guard<std::mutex> lock(mutex);
      if (queued == 0 && !pending) {
        return EpisodeWriterStatus::kOk;
      }
    }
  }

  EpisodeWriterStatus Flush() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (cancelled) return EpisodeWriterStatus::kCancelled;
      if (closed) return EpisodeWriterStatus::kClosed;
      if (latched) return EpisodeWriterStatus::kWriteFailed;
    }
    return Drain(/*until_empty=*/false);
  }

  EpisodeWriterStatus Close() {
    bool already_latched;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (cancelled) return EpisodeWriterStatus::kCancelled;
      if (closed) return close_result;
      closed = true;
      already_latched = latched;
    }
    EpisodeWriterStatus result = EpisodeWriterStatus::kWriteFailed;
    if (!already_latched) {
      result = Drain(/*until_empty=*/true);
      if (result == EpisodeWriterStatus::kOk) {
        const absl::Status status = sink->Sync();
        if (!status.ok()) {
          Latch(status);
          result = EpisodeWriterStatus::kWriteFailed;
        }
      }
    }
    std::lock_guard<std::mutex> lock(mutex);
    close_result = result;
    return result;
  }

  uint64_t Cancel() {
    std::lock_guard<std::mutex> lock(mutex);
    if (cancelled) {
      return 0;
    }
    cancelled = true;
    const uint64_t discarded = queued;
    stats.dropped_on_cancel += discarded;
    queued = 0;
    pending = false;
    return discarded;
  }

  EpisodeWriterStats Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex);
    EpisodeWriterStats snapshot = stats;
    snapshot.queued_records = queued;
    snapshot.pending_loss = pending;
    snapshot.next_writer_index = next_index;
    return snapshot;
  }

  absl::Status FirstError() const {
    std::lock_guard<std::mutex> lock(mutex);
    return first_error;
  }

  mutable std::mutex mutex;
  EpisodeSink* const sink;
  const size_t capacity;
  const size_t max_event_bytes;
  const size_t slot_bytes;

  std::unique_ptr<uint8_t[]> storage;
  std::vector<size_t> slot_length;
  size_t head = 0;
  size_t queued = 0;

  std::vector<Source> sources;
  size_t source_count = 0;

  EpisodeEvent loss_template;
  uint64_t loss_ordinal = 0;
  bool pending = false;
  PendingLoss pending_loss;
  uint64_t next_index = 0;

  bool header_written = false;
  bool closed = false;
  bool cancelled = false;
  bool latched = false;
  absl::Status first_error;
  EpisodeWriterStatus close_result = EpisodeWriterStatus::kOk;

  EpisodeWriterStats stats;
};

EpisodeWriter::EpisodeWriter(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

EpisodeWriter::~EpisodeWriter() = default;

std::unique_ptr<EpisodeWriter> EpisodeWriter::Create(
    const EpisodeWriterConfig& config, EpisodeSink* sink,
    EpisodeWriterStatus* status) {
  EpisodeWriterStatus unused;
  EpisodeWriterStatus& out = status != nullptr ? *status : unused;
  out = EpisodeWriterStatus::kInvalidConfig;
  if (sink == nullptr || config.capacity < kMinEpisodeWriterCapacity ||
      config.capacity > kMaxEpisodeWriterCapacity ||
      config.max_event_bytes < kMinEpisodeMaxEventBytes ||
      config.max_event_bytes > kMaxEpisodeMaxEventBytes) {
    return nullptr;
  }

  EpisodeEvent loss_template;
  auto* header = loss_template.mutable_header();
  header->set_source_id(std::string(kEpisodeWriterSourceId));
  header->set_frame_id(std::string(kEpisodeWriterFrameId));
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

  const size_t slot_bytes =
      std::max(config.max_event_bytes, largest.ByteSizeLong()) +
      kEpisodeRecordHeaderBytes;

  auto impl = std::make_unique<Impl>(sink, config.capacity,
                                     config.max_event_bytes, slot_bytes);
  impl->storage.reset(new (std::nothrow) uint8_t[config.capacity * slot_bytes]);
  if (impl->storage == nullptr) {
    return nullptr;
  }
  try {
    impl->slot_length.assign(config.capacity, 0);
    impl->sources.resize(kMaxEpisodeSources);
    for (Impl::Source& source : impl->sources) {
      source.id.reserve(config.max_event_bytes);
    }
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
  impl->loss_template = std::move(loss_template);

  out = EpisodeWriterStatus::kOk;
  return std::unique_ptr<EpisodeWriter>(new EpisodeWriter(std::move(impl)));
}

EpisodeAppendResult EpisodeWriter::Append(const EpisodeEvent& event) {
  return impl_->Append(event);
}

EpisodeWriterStatus EpisodeWriter::Flush() { return impl_->Flush(); }

EpisodeWriterStatus EpisodeWriter::Close() { return impl_->Close(); }

uint64_t EpisodeWriter::Cancel() { return impl_->Cancel(); }

EpisodeWriterStats EpisodeWriter::Stats() const { return impl_->Snapshot(); }

absl::Status EpisodeWriter::FirstSinkError() const {
  return impl_->FirstError();
}

}  // namespace intrinsic::episode
