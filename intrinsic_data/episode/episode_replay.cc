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

#include "intrinsic_data/episode/episode_replay.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_event_view.h"
#include "intrinsic_data/episode/episode_reader.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;

constexpr uint32_t kMinSpeed = 1;
constexpr uint32_t kMaxSpeed = 1000;
constexpr __int128 kNanosPerSecond = embodiment::kNanosPerSecond;
constexpr __int128 kInt64Min = std::numeric_limits<int64_t>::min();
constexpr __int128 kInt64Max = std::numeric_limits<int64_t>::max();

bool ConfigValid(const EpisodeReplayConfig& config) {
  return (config.mode == EpisodeReplayMode::kStep ||
          config.mode == EpisodeReplayMode::kTimed) &&
         config.speed_num >= kMinSpeed && config.speed_num <= kMaxSpeed &&
         config.speed_den >= kMinSpeed && config.speed_den <= kMaxSpeed;
}

bool FitsInt64(__int128 value) {
  return value >= kInt64Min && value <= kInt64Max;
}

__int128 ToNanos(embodiment::ClockReading reading) {
  return static_cast<__int128>(reading.seconds) * kNanosPerSecond +
         reading.nanos;
}

// Floor division, so nanos stays in [0, 1e9) for negative values too.
__int128 FloorDivNanosPerSecond(__int128 nanos) {
  __int128 seconds = nanos / kNanosPerSecond;
  if (nanos % kNanosPerSecond < 0) {
    --seconds;
  }
  return seconds;
}

// Caller guarantees that the seconds fit in int64.
embodiment::ClockReading FromNanos(__int128 nanos) {
  const __int128 seconds = FloorDivNanosPerSecond(nanos);
  embodiment::ClockReading reading;
  reading.seconds = static_cast<int64_t>(seconds);
  reading.nanos = static_cast<int32_t>(nanos - seconds * kNanosPerSecond);
  return reading;
}

bool SecondsFitInt64(__int128 nanos) {
  return FitsInt64(FloorDivNanosPerSecond(nanos));
}

}  // namespace

EpisodeReplayer::EpisodeReplayer(EpisodeReplayConfig config,
                                 EpisodeReadResult filtered,
                                 EpisodeReadResult unfiltered)
    : config_(config),
      filtered_(std::move(filtered)),
      unfiltered_(std::move(unfiltered)) {}

EpisodeReplayer::~EpisodeReplayer() = default;

std::unique_ptr<EpisodeReplayer> EpisodeReplayer::Create(
    absl::Span<const uint8_t> bytes, const EpisodeReaderConfig& reader_config,
    const EpisodeReadFilter& filter, const EpisodeReplayConfig& config,
    EpisodeReplayStatus* status) {
  EpisodeReplayStatus ignored = EpisodeReplayStatus::kOk;
  EpisodeReplayStatus& out = status != nullptr ? *status : ignored;
  auto reject = [&out](EpisodeReplayStatus reason) {
    out = reason;
    return std::unique_ptr<EpisodeReplayer>();
  };

  if (!ConfigValid(config)) {
    return reject(EpisodeReplayStatus::kInvalidConfig);
  }
  EpisodeReadResult filtered = ReadEpisode(bytes, reader_config, filter);
  if (filtered.status != EpisodeReaderStatus::kOk) {
    return reject(EpisodeReplayStatus::kInvalidConfig);
  }
  EpisodeReadResult unfiltered = ReadEpisode(bytes, reader_config, {});
  if (unfiltered.status != EpisodeReaderStatus::kOk) {
    return reject(EpisodeReplayStatus::kInvalidConfig);
  }
  if (filtered.defect != EpisodeReadDefect::kNone &&
      !config.allow_defective_prefix) {
    return reject(EpisodeReplayStatus::kDefectiveStream);
  }

  std::unique_ptr<EpisodeReplayer> replayer(
      new EpisodeReplayer(config, std::move(filtered), std::move(unfiltered)));
  const std::vector<EpisodeRecordEntry>& records = replayer->filtered_.records;
  const std::vector<EpisodeLossEntry>& losses = replayer->unfiltered_.losses;
  const std::vector<EpisodeRecordEntry>& all = replayer->unfiltered_.records;

  // The reader's records plus every loss record, merged by writer_index. Both
  // lists are in file order, and a loss record that passes the filter is in
  // both.
  std::vector<Slot>& slots = replayer->slots_;
  slots.reserve(records.size() + losses.size());
  size_t next_record = 0;
  size_t next_loss = 0;
  while (next_record < records.size() || next_loss < losses.size()) {
    const bool take_record =
        next_record < records.size() &&
        (next_loss >= losses.size() ||
         records[next_record].writer_index <= losses[next_loss].writer_index);
    Slot slot;
    if (take_record) {
      const EpisodeRecordEntry& record = records[next_record++];
      if (record.skip) {
        ++replayer->stats_.skipped_unknown;
        continue;
      }
      slot.record = &record;
      slot.writer_index = record.writer_index;
      if (record.payload_kind == PayloadKind::kEpisodeLoss) {
        if (next_loss >= losses.size() ||
            losses[next_loss].writer_index != record.writer_index) {
          // The reader gives every loss record an entry in `losses`.
          return reject(EpisodeReplayStatus::kDefectiveStream);
        }
        slot.loss = &losses[next_loss++];
      }
    } else {
      slot.loss = &losses[next_loss++];
      slot.writer_index = slot.loss->writer_index;
    }
    slots.push_back(slot);
  }
  replayer->stats_.total = slots.size();

  // Parse each header once. A loss record that the filter dropped is found in
  // the unfiltered read.
  std::vector<__int128> receive_ns;
  receive_ns.reserve(slots.size());
  EpisodeEvent message;
  std::string clock_domain;
  bool has_clock_domain = false;
  for (const Slot& slot : slots) {
    absl::Span<const uint8_t> event_bytes;
    if (slot.record != nullptr) {
      event_bytes = slot.record->event_bytes;
    } else {
      const auto it = std::lower_bound(
          all.begin(), all.end(), slot.writer_index,
          [](const EpisodeRecordEntry& entry, uint64_t writer_index) {
            return entry.writer_index < writer_index;
          });
      if (it == all.end() || it->writer_index != slot.writer_index) {
        return reject(EpisodeReplayStatus::kDefectiveStream);
      }
      event_bytes = it->event_bytes;
    }
    if (!message.ParseFromArray(event_bytes.data(),
                                static_cast<int>(event_bytes.size()))) {
      return reject(EpisodeReplayStatus::kDefectiveStream);
    }
    const EpisodeEventView view = EpisodeEventViewOf(message);
    receive_ns.push_back(ToNanos(view.receive_time));

    if (slot.loss == nullptr) {
      const std::string& domain = message.header().clock_domain();
      if (domain.empty() || (has_clock_domain && domain != clock_domain)) {
        return reject(EpisodeReplayStatus::kMixedClockDomain);
      }
      clock_domain = domain;
      has_clock_domain = true;
    }
  }

  const __int128 num = config.speed_num;
  const __int128 den = config.speed_den;
  int64_t previous_recorded = 0;
  int64_t previous_due = 0;
  for (size_t k = 0; k < slots.size(); ++k) {
    const __int128 recorded = receive_ns[k] - receive_ns[0];
    if (!FitsInt64(recorded)) {
      return reject(EpisodeReplayStatus::kTimeOutOfRange);
    }
    // Truncation toward zero.
    const __int128 scaled = recorded * den / num;
    const __int128 due = k == 0 ? 0 : std::max<__int128>(previous_due, scaled);
    if (!FitsInt64(due)) {
      return reject(EpisodeReplayStatus::kTimeOutOfRange);
    }
    if (k > 0 && recorded < previous_recorded) {
      ++replayer->stats_.time_regressions;
    }
    slots[k].recorded_offset_ns = static_cast<int64_t>(recorded);
    slots[k].due_offset_ns = static_cast<int64_t>(due);
    previous_recorded = slots[k].recorded_offset_ns;
    previous_due = slots[k].due_offset_ns;
  }

  out = EpisodeReplayStatus::kOk;
  return replayer;
}

EpisodeReplayResult EpisodeReplayer::EndResult() const {
  EpisodeReplayResult result;
  if (config_.allow_defective_prefix &&
      filtered_.defect != EpisodeReadDefect::kNone) {
    result.status = EpisodeReplayStatus::kEndOfTrustedPrefix;
    result.reader_defect = filtered_.defect;
  } else {
    result.status = EpisodeReplayStatus::kEndOfStream;
  }
  return result;
}

void EpisodeReplayer::Emit(EpisodeReplayResult* result) {
  const Slot& slot = slots_[next_];
  result->status = EpisodeReplayStatus::kOk;
  result->event.record = slot.record;
  result->event.loss = slot.loss;
  result->event.writer_index = slot.writer_index;
  result->event.recorded_offset_ns = slot.recorded_offset_ns;
  result->event.due_offset_ns = slot.due_offset_ns;
  result->event.ordinal = next_;
  ++next_;
  ++stats_.emitted;
  if (slot.loss != nullptr) {
    ++stats_.loss_emitted;
  }
}

EpisodeReplayResult EpisodeReplayer::Step() {
  EpisodeReplayResult result;
  if (cancelled_) {
    result.status = EpisodeReplayStatus::kCancelled;
    return result;
  }
  if (config_.mode != EpisodeReplayMode::kStep) {
    result.status = EpisodeReplayStatus::kWrongMode;
    return result;
  }
  if (next_ >= slots_.size()) {
    return EndResult();
  }
  Emit(&result);
  return result;
}

EpisodeReplayStatus EpisodeReplayer::Start(embodiment::ClockReading now) {
  if (cancelled_) {
    return EpisodeReplayStatus::kCancelled;
  }
  if (config_.mode != EpisodeReplayMode::kTimed || started_) {
    return EpisodeReplayStatus::kWrongMode;
  }
  if (!embodiment::NanosInRange(now.nanos)) {
    return EpisodeReplayStatus::kInvalidTime;
  }
  // Due offsets never decrease, so the last one is the largest.
  const int64_t max_due = slots_.empty() ? 0 : slots_.back().due_offset_ns;
  const __int128 start_ns = ToNanos(now);
  if (!SecondsFitInt64(start_ns + max_due)) {
    return EpisodeReplayStatus::kTimeOutOfRange;
  }
  start_ns_ = start_ns;
  started_ = true;
  return EpisodeReplayStatus::kOk;
}

EpisodeReplayResult EpisodeReplayer::Poll(embodiment::ClockReading now) {
  EpisodeReplayResult result;
  if (cancelled_) {
    result.status = EpisodeReplayStatus::kCancelled;
    return result;
  }
  if (config_.mode != EpisodeReplayMode::kTimed) {
    result.status = EpisodeReplayStatus::kWrongMode;
    return result;
  }
  if (!started_) {
    result.status = EpisodeReplayStatus::kNotStarted;
    return result;
  }
  if (!embodiment::NanosInRange(now.nanos)) {
    result.status = EpisodeReplayStatus::kInvalidTime;
    return result;
  }
  if (next_ >= slots_.size()) {
    return EndResult();
  }
  const __int128 due_ns = start_ns_ + slots_[next_].due_offset_ns;
  if (ToNanos(now) < due_ns) {
    result.status = EpisodeReplayStatus::kNotDue;
    result.next_due = FromNanos(due_ns);
    return result;
  }
  Emit(&result);
  return result;
}

uint64_t EpisodeReplayer::Cancel() {
  if (cancelled_) {
    return 0;
  }
  cancelled_ = true;
  const uint64_t remaining = slots_.size() - next_;
  stats_.cancelled_remaining = remaining;
  return remaining;
}

EpisodeReplayStats EpisodeReplayer::Stats() const { return stats_; }

}  // namespace intrinsic::episode
