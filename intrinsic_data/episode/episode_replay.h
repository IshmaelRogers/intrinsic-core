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

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "absl/types/span.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_data/episode/episode_reader.h"

namespace intrinsic::episode {

// Deterministic episode replay clock. Contract: lock id
// episode-replay-clock-v1 (#130). Timestamp policy and semantics:
// intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Never call this from an ICON cycle, never link it
// into a real-time part. The replayer is passive: it starts no thread, never
// sleeps, and reads no clock. The caller passes `now` to Poll(). Events come
// out in file order (writer_index) and recorded timestamps and bytes are never
// modified.

enum class EpisodeReplayMode {
  // Step() emits exactly the next event and ignores time.
  kStep = 0,
  // Start(now) once, then Poll(now). Speed is an integer ratio.
  kTimed = 1,
};

// Append-only. Numbers are permanent.
enum class EpisodeReplayStatus {
  kOk = 0,
  kInvalidConfig = 1,
  kDefectiveStream = 2,
  kMixedClockDomain = 3,
  kTimeOutOfRange = 4,
  kWrongMode = 5,
  kNotStarted = 6,
  kNotDue = 7,
  kEndOfStream = 8,
  kEndOfTrustedPrefix = 9,
  kCancelled = 10,
  kInvalidTime = 11,
};

struct EpisodeReplayConfig {
  EpisodeReplayMode mode = EpisodeReplayMode::kStep;
  // Replay speed is speed_num / speed_den, each in [1, 1000]. Real time is
  // 1/1, num > den is faster, num < den is slow motion. Rejected, never
  // clamped.
  uint32_t speed_num = 1;
  uint32_t speed_den = 1;
  // Replay the trusted prefix of a defective stream. The end status is then
  // kEndOfTrustedPrefix instead of the Create rejection kDefectiveStream.
  bool allow_defective_prefix = false;
};

struct EpisodeReplayEvent {
  // For a loss record, the entry in the filtered read if present.
  const EpisodeRecordEntry* record = nullptr;
  // Set for loss records.
  const EpisodeLossEntry* loss = nullptr;
  uint64_t writer_index = 0;
  // receive_time minus the first emitted event's receive_time.
  int64_t recorded_offset_ns = 0;
  // After speed scaling and the monotone rule.
  int64_t due_offset_ns = 0;
  // 0, 1, 2, ... in emission order.
  uint64_t ordinal = 0;
};

struct EpisodeReplayResult {
  EpisodeReplayStatus status = EpisodeReplayStatus::kOk;
  // Valid for kOk.
  EpisodeReplayEvent event;
  // Valid for kNotDue.
  embodiment::ClockReading next_due{};
  // Set for kEndOfTrustedPrefix.
  EpisodeReadDefect reader_defect = EpisodeReadDefect::kNone;
};

// Plain value snapshot.
struct EpisodeReplayStats {
  // Events to emit, after the unknown-payload skip and the loss merge.
  uint64_t total = 0;
  uint64_t emitted = 0;
  uint64_t loss_emitted = 0;
  uint64_t skipped_unknown = 0;
  uint64_t time_regressions = 0;
  uint64_t cancelled_remaining = 0;
};

class EpisodeReplayer {
 public:
  // Calls ReadEpisode(bytes, reader_config, filter) and
  // ReadEpisode(bytes, reader_config, {}) and keeps both results. Returns
  // nullptr and sets *status on any rejection. Never clamps. *status is kOk on
  // success. Every due offset is computed here, so Step, Start, Poll and
  // Cancel do not allocate.
  //
  // `bytes` is not copied and must outlive the replayer.
  static std::unique_ptr<EpisodeReplayer> Create(
      absl::Span<const uint8_t> bytes, const EpisodeReaderConfig& reader_config,
      const EpisodeReadFilter& filter, const EpisodeReplayConfig& config,
      EpisodeReplayStatus* status);

  EpisodeReplayer(const EpisodeReplayer&) = delete;
  EpisodeReplayer& operator=(const EpisodeReplayer&) = delete;
  ~EpisodeReplayer();

  // kStep only.
  EpisodeReplayResult Step();
  // kTimed only, once.
  EpisodeReplayStatus Start(embodiment::ClockReading now);
  // kTimed only. Returns at most one due event.
  EpisodeReplayResult Poll(embodiment::ClockReading now);
  // Idempotent. The first call returns the number of events not yet emitted.
  uint64_t Cancel();

  EpisodeReplayStats Stats() const;

 private:
  struct Slot {
    const EpisodeRecordEntry* record = nullptr;
    const EpisodeLossEntry* loss = nullptr;
    uint64_t writer_index = 0;
    int64_t recorded_offset_ns = 0;
    int64_t due_offset_ns = 0;
  };

  EpisodeReplayer(EpisodeReplayConfig config, EpisodeReadResult filtered,
                  EpisodeReadResult unfiltered);

  EpisodeReplayResult EndResult() const;
  void Emit(EpisodeReplayResult* result);

  EpisodeReplayConfig config_;
  // Never mutated after Create: the slots and the emitted events point into
  // these.
  EpisodeReadResult filtered_;
  EpisodeReadResult unfiltered_;
  std::vector<Slot> slots_;
  size_t next_ = 0;
  bool cancelled_ = false;
  bool started_ = false;
  __int128 start_ns_ = 0;
  EpisodeReplayStats stats_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_H_
