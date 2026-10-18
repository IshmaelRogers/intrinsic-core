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

#ifndef INTRINSIC_DATA_EPISODE_WORLD_REPLAY_ADAPTER_H_
#define INTRINSIC_DATA_EPISODE_WORLD_REPLAY_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {

// Passive World snapshot replay adapter. Contract: lock id
// episode-replay-adapters-estimator-world-v1 (#131). Semantics:
// intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Feed() takes each EpisodeReplayer event. The
// #114 query time is the event's recorded header.receive_time, never a wall
// clock. The adapter never synthesizes, interpolates or holds a snapshot.

struct WorldReplayConfig {
  ReplayCompatibilityConfig compatibility;
  // #114 policy. An invalid policy gives kInvalidConfig.
  world::WorldSnapshotSkewPolicy policy;
};

struct WorldReplayStep {
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // 0 for kFinished.
  uint64_t writer_index = 0;
  // True when the policy ran (status kOk).
  bool assessed = false;
  // Set when assessed.
  world::WorldSnapshotPolicyAssessment assessment;
  // Set for kInvalidRecord.
  world::SnapshotError descriptor_error = world::SnapshotError::kNone;
};

// One per assessed record, in feed order.
struct WorldTraceEntry {
  uint64_t writer_index = 0;
  std::string snapshot_id;
  world::SnapshotPolicyStatus status =
      world::SnapshotPolicyStatus::kUnspecified;
  bool withhold = false;
};

struct WorldReplayStats {
  uint64_t fed = 0;
  uint64_t assessed = 0;
  uint64_t accepted = 0;
  uint64_t withheld = 0;
  uint64_t ignored = 0;
  uint64_t invalid_records = 0;
  uint64_t snapshot_id_mismatches = 0;
  uint64_t gaps = 0;
  uint64_t gap_dropped = 0;
  uint64_t after_latch = 0;
};

struct WorldReplaySummary {
  // kOk or the latched status.
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  bool degraded = false;
  // Lowercase hex SHA-256 over "v1\n" and "<snapshot_id>\n" for each assessed
  // record, in feed order.
  std::string chain_digest;
  std::vector<WorldTraceEntry> trace;
  WorldReplayStats stats;
};

class WorldReplayAdapter {
 public:
  // An invalid policy (world::snapshot_skew_internal::PolicyValid) or an
  // invalid compatibility config gives nullptr and kInvalidConfig.
  static std::unique_ptr<WorldReplayAdapter> Create(
      const WorldReplayConfig& config, ReplayAdapterStatus* status);

  WorldReplayAdapter(const WorldReplayAdapter&) = delete;
  WorldReplayAdapter& operator=(const WorldReplayAdapter&) = delete;

  WorldReplayStep Feed(const EpisodeReplayEvent& event);

  // Idempotent: a second call returns the same summary.
  WorldReplaySummary Finish();

 private:
  explicit WorldReplayAdapter(WorldReplayConfig config);

  WorldReplayConfig config_;
  bool latched_ = false;
  ReplayAdapterStatus latched_status_ = ReplayAdapterStatus::kOk;
  bool finished_ = false;
  bool degraded_ = false;
  std::string chain_input_;
  std::vector<WorldTraceEntry> trace_;
  WorldReplayStats stats_;
  WorldReplaySummary summary_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_WORLD_REPLAY_ADAPTER_H_
