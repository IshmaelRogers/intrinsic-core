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

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_ADAPTER_COMMON_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_ADAPTER_COMMON_H_

#include <string>

#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"

namespace intrinsic::episode {

// Shared by the estimator and World replay adapters. Contract: lock id
// episode-replay-adapters-estimator-world-v1 (#131). Semantics:
// intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Never call this from an ICON cycle, never link it
// into a real-time part. No thread, no sleep, no clock, no randomness.

// Append-only. Numbers are permanent.
enum class ReplayAdapterStatus {
  kOk = 0,
  kInvalidConfig = 1,
  // Another payload kind, or another RecordedMessage type.
  kIgnoredPayload = 2,
  // schema_version != 1. Latches.
  kIncompatibleSchema = 3,
  // A non-empty event config_digest differs from the expected one. Latches.
  kConfigDigestMismatch = 4,
  // World: the envelope, descriptor and recomputed ids differ.
  kSnapshotIdMismatch = 5,
  // Unparsable bytes or a structural defect.
  kInvalidRecord = 6,
  // Finish() only.
  kMissingRequiredStream = 7,
  // Every Feed after a latching status.
  kLatched = 8,
  // A loss entry, counted as a gap.
  kLossRecord = 9,
  // Feed after Finish().
  kFinished = 10,
};

inline const char* ReplayAdapterStatusName(ReplayAdapterStatus status) {
  switch (status) {
    case ReplayAdapterStatus::kOk:
      return "ok";
    case ReplayAdapterStatus::kInvalidConfig:
      return "invalid_config";
    case ReplayAdapterStatus::kIgnoredPayload:
      return "ignored_payload";
    case ReplayAdapterStatus::kIncompatibleSchema:
      return "incompatible_schema";
    case ReplayAdapterStatus::kConfigDigestMismatch:
      return "config_digest_mismatch";
    case ReplayAdapterStatus::kSnapshotIdMismatch:
      return "snapshot_id_mismatch";
    case ReplayAdapterStatus::kInvalidRecord:
      return "invalid_record";
    case ReplayAdapterStatus::kMissingRequiredStream:
      return "missing_required_stream";
    case ReplayAdapterStatus::kLatched:
      return "latched";
    case ReplayAdapterStatus::kLossRecord:
      return "loss_record";
    case ReplayAdapterStatus::kFinished:
      return "finished";
  }
  return "unknown";
}

struct ReplayCompatibilityConfig {
  // Empty, or 64 lowercase hex (IsLowercaseSha256Hex). Empty disables the
  // digest check (an empty event digest is always accepted: #127 "absent").
  std::string expected_config_digest;
};

// Parses record->event_bytes into *event_out (ParseFromArray, exact bytes).
// Returns false on a parse failure or when event.record is null.
inline bool ParseReplayRecord(
    const EpisodeReplayEvent& event,
    intrinsic_proto::episode::EpisodeEvent* event_out) {
  if (event.record == nullptr) {
    return false;
  }
  return event_out->ParseFromArray(
      event.record->event_bytes.data(),
      static_cast<int>(event.record->event_bytes.size()));
}

// kOk, kIncompatibleSchema, or kConfigDigestMismatch, checked in that order.
inline ReplayAdapterStatus CheckCompatibility(
    const intrinsic_proto::episode::EpisodeEvent& event,
    const ReplayCompatibilityConfig& config) {
  if (event.schema_version() != 1) {
    return ReplayAdapterStatus::kIncompatibleSchema;
  }
  if (!config.expected_config_digest.empty() &&
      !event.config_digest().empty() &&
      event.config_digest() != config.expected_config_digest) {
    return ReplayAdapterStatus::kConfigDigestMismatch;
  }
  return ReplayAdapterStatus::kOk;
}

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_REPLAY_ADAPTER_COMMON_H_
