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

#ifndef INTRINSIC_DATA_EPISODE_PLANNING_REPLAY_ADAPTER_H_
#define INTRINSIC_DATA_EPISODE_PLANNING_REPLAY_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {

// Passive planning replay adapter. Contract: lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), decision P1-A.
// Semantics: intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Feed() takes each EpisodeReplayer event. The
// adapter verifies and digests the recorded vehicle_trajectory (kind 2) and
// desired_motion (kind 10) payloads. It never calls a VehiclePlanner, never
// re-plans, and never rewrites a record. No thread, no sleep, no clock, no
// randomness.

struct PlanningReplayConfig {
  ReplayCompatibilityConfig compatibility;
};

struct PlanningReplayStep {
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // 0 for kFinished.
  uint64_t writer_index = 0;
  // kVehicleTrajectory or kDesiredMotion for an owned, parsed, compatible
  // record. kNone otherwise.
  PayloadKind payload_kind = PayloadKind::kNone;
  // TrajectoryContractError (kVehicleTrajectory) or vehicle::ContractError
  // (kDesiredMotion), as an int.
  int error = 0;
  bool accepted = false;
  // Lowercase hex SHA-256 of the payload's deterministic serialization. Empty
  // when no owned record was parsed.
  std::string payload_sha256;
};

// One per owned, parsed, compatible record, in feed order.
struct PlanningTraceEntry {
  uint64_t writer_index = 0;
  PayloadKind payload_kind = PayloadKind::kNone;
  int error = 0;
  bool accepted = false;
  std::string payload_sha256;
};

struct PlanningReplayStats {
  uint64_t fed = 0;
  uint64_t trajectories = 0;
  uint64_t trajectories_accepted = 0;
  uint64_t motions = 0;
  uint64_t motions_accepted = 0;
  uint64_t ignored = 0;
  uint64_t invalid_records = 0;
  uint64_t gaps = 0;
  uint64_t gap_dropped = 0;
  uint64_t after_latch = 0;
};

struct PlanningReplaySummary {
  // kOk or the latched status.
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // Set by a loss record or an unparsable record.
  bool degraded = false;
  // Lowercase hex SHA-256 over "v1\n" and "<payload_sha256>\n" for each
  // accepted trajectory, in feed order.
  std::string plan_chain;
  // The same for each accepted desired motion.
  std::string intent_chain;
  std::vector<PlanningTraceEntry> trace;
  PlanningReplayStats stats;
};

class PlanningReplayAdapter {
 public:
  // An invalid compatibility config gives nullptr and kInvalidConfig.
  static std::unique_ptr<PlanningReplayAdapter> Create(
      const PlanningReplayConfig& config, ReplayAdapterStatus* status);

  PlanningReplayAdapter(const PlanningReplayAdapter&) = delete;
  PlanningReplayAdapter& operator=(const PlanningReplayAdapter&) = delete;

  PlanningReplayStep Feed(const EpisodeReplayEvent& event);

  // Idempotent: a second call returns the same summary.
  PlanningReplaySummary Finish();

 private:
  explicit PlanningReplayAdapter(PlanningReplayConfig config);

  PlanningReplayConfig config_;
  bool latched_ = false;
  ReplayAdapterStatus latched_status_ = ReplayAdapterStatus::kOk;
  bool finished_ = false;
  bool degraded_ = false;
  std::string plan_chain_input_;
  std::string intent_chain_input_;
  std::vector<PlanningTraceEntry> trace_;
  PlanningReplayStats stats_;
  PlanningReplaySummary summary_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_PLANNING_REPLAY_ADAPTER_H_
