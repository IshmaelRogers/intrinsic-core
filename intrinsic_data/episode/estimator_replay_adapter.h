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

#ifndef INTRINSIC_DATA_EPISODE_ESTIMATOR_REPLAY_ADAPTER_H_
#define INTRINSIC_DATA_EPISODE_ESTIMATOR_REPLAY_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/estimation/estimator_service.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {

// Passive estimator replay adapter. Contract: lock id
// episode-replay-adapters-estimator-world-v1 (#131). Semantics:
// intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Feed() takes each EpisodeReplayer event. The
// adapter owns no replayer, buffer or clock. It calls Initialize once in
// Create and then exactly one Ingest per MeasurementEnvelope. It never calls
// Predict, Reset or Initialize again, and it never synthesizes data.

struct EstimatorReplayConfig {
  ReplayCompatibilityConfig compatibility;
  intrinsic_proto::estimation::EstimatorConfig estimator_config;
  // Each non-empty and unique, else kInvalidConfig. May be empty.
  std::vector<std::string> required_sources;
};

struct EstimatorReplayStep {
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // 0 for kFinished.
  uint64_t writer_index = 0;
  // Set only when Ingest was called (status kOk).
  bool ingested = false;
  intrinsic_proto::estimation::EstimatorResult result;
  // After the Ingest.
  uint64_t state_digest = 0;
};

// One per Ingest, in feed order.
struct EstimatorTraceEntry {
  uint64_t writer_index = 0;
  bool ok = false;
  int reason = 0;
  uint64_t epoch = 0;
  uint64_t state_digest = 0;
};

struct EstimatorReplayStats {
  uint64_t fed = 0;
  uint64_t ingested = 0;
  uint64_t accepted = 0;
  uint64_t rejected = 0;
  uint64_t ignored = 0;
  uint64_t ignored_message_type = 0;
  uint64_t invalid_records = 0;
  uint64_t gaps = 0;
  uint64_t gap_dropped = 0;
  uint64_t after_latch = 0;
};

struct EstimatorReplaySummary {
  // kOk, kMissingRequiredStream, or the latched status.
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // Sorted unique.
  std::vector<std::string> missing_required_sources;
  bool degraded = false;
  uint64_t final_state_digest = 0;
  std::vector<EstimatorTraceEntry> trace;
  EstimatorReplayStats stats;
};

class EstimatorReplayAdapter {
 public:
  // `service` is not owned and must outlive the adapter. A null service, an
  // invalid compatibility or required_sources config, or an Initialize result
  // with ok == false gives nullptr and kInvalidConfig. The Initialize result
  // is returned in *init_result when non-null.
  static std::unique_ptr<EstimatorReplayAdapter> Create(
      estimation::EstimatorService* service,
      const EstimatorReplayConfig& config, ReplayAdapterStatus* status,
      intrinsic_proto::estimation::EstimatorResult* init_result = nullptr);

  EstimatorReplayAdapter(const EstimatorReplayAdapter&) = delete;
  EstimatorReplayAdapter& operator=(const EstimatorReplayAdapter&) = delete;

  EstimatorReplayStep Feed(const EpisodeReplayEvent& event);

  // Idempotent: a second call returns the same summary.
  EstimatorReplaySummary Finish();

 private:
  EstimatorReplayAdapter(estimation::EstimatorService* service,
                         EstimatorReplayConfig config);

  estimation::EstimatorService* service_;
  EstimatorReplayConfig config_;
  bool latched_ = false;
  ReplayAdapterStatus latched_status_ = ReplayAdapterStatus::kOk;
  bool finished_ = false;
  bool degraded_ = false;
  std::set<std::string> seen_sources_;
  std::vector<EstimatorTraceEntry> trace_;
  EstimatorReplayStats stats_;
  EstimatorReplaySummary summary_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_ESTIMATOR_REPLAY_ADAPTER_H_
