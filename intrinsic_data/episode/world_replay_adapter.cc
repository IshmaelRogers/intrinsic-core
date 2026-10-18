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

#include "intrinsic_data/episode/world_replay_adapter.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_builder.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_assessor.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;

bool CompatibilityValid(const ReplayCompatibilityConfig& config) {
  return config.expected_config_digest.empty() ||
         IsLowercaseSha256Hex(config.expected_config_digest);
}

}  // namespace

WorldReplayAdapter::WorldReplayAdapter(WorldReplayConfig config)
    : config_(std::move(config)) {}

std::unique_ptr<WorldReplayAdapter> WorldReplayAdapter::Create(
    const WorldReplayConfig& config, ReplayAdapterStatus* status) {
  ReplayAdapterStatus local_status = ReplayAdapterStatus::kInvalidConfig;
  ReplayAdapterStatus* out = status != nullptr ? status : &local_status;
  *out = ReplayAdapterStatus::kInvalidConfig;
  if (!CompatibilityValid(config.compatibility) ||
      !world::snapshot_skew_internal::PolicyValid(config.policy)) {
    return nullptr;
  }
  *out = ReplayAdapterStatus::kOk;
  return std::unique_ptr<WorldReplayAdapter>(new WorldReplayAdapter(config));
}

WorldReplayStep WorldReplayAdapter::Feed(const EpisodeReplayEvent& event) {
  WorldReplayStep step;
  if (finished_) {
    step.status = ReplayAdapterStatus::kFinished;
    return step;
  }
  step.writer_index = event.writer_index;
  ++stats_.fed;
  if (latched_) {
    ++stats_.after_latch;
    step.status = ReplayAdapterStatus::kLatched;
    return step;
  }
  if (event.loss != nullptr) {
    ++stats_.gaps;
    stats_.gap_dropped += event.loss->dropped_count;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kLossRecord;
    return step;
  }
  EpisodeEvent parsed;
  if (!ParseReplayRecord(event, &parsed)) {
    ++stats_.invalid_records;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kInvalidRecord;
    return step;
  }
  const ReplayAdapterStatus compatibility =
      CheckCompatibility(parsed, config_.compatibility);
  if (compatibility != ReplayAdapterStatus::kOk) {
    latched_ = true;
    latched_status_ = compatibility;
    step.status = compatibility;
    return step;
  }
  if (event.record->payload_kind != PayloadKind::kWorldSnapshotRecord) {
    ++stats_.ignored;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }

  const auto& record = parsed.world_snapshot_record();
  if (!record.has_snapshot()) {
    ++stats_.invalid_records;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kInvalidRecord;
    return step;
  }
  const auto& descriptor = record.snapshot();
  const world::SnapshotAssessment structural =
      world::AssessWorldSnapshotDescriptor(descriptor);
  if (!structural.accepted) {
    ++stats_.invalid_records;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kInvalidRecord;
    step.descriptor_error = structural.error;
    return step;
  }
  const bool envelope_mismatch =
      !parsed.snapshot_id().empty() &&
      parsed.snapshot_id() != descriptor.snapshot_id();
  if (envelope_mismatch ||
      world::ComputeSnapshotId(
          descriptor.state_epoch(),
          world::WorldSnapshotViewFromProto(descriptor, true).components) !=
          descriptor.snapshot_id()) {
    ++stats_.snapshot_id_mismatches;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kSnapshotIdMismatch;
    return step;
  }

  world::ComponentTimings timings;
  timings.reserve(record.timings_size());
  for (const auto& timing : record.timings()) {
    world::ComponentTiming converted;
    converted.component_kind = timing.component_kind();
    converted.observation_time = {timing.observation_time().seconds(),
                                  timing.observation_time().nanos()};
    converted.validity_horizon = {timing.validity_horizon().seconds(),
                                  timing.validity_horizon().nanos()};
    converted.source_id = timing.source_id();
    timings.push_back(std::move(converted));
  }
  const world::TimeParts query_time = {parsed.header().receive_time().seconds(),
                                       parsed.header().receive_time().nanos()};
  step.assessment = world::AssessWorldSnapshotDescriptorSkew(
      descriptor, timings, query_time, config_.policy);
  step.assessed = true;
  step.status = ReplayAdapterStatus::kOk;
  ++stats_.assessed;
  if (step.assessment.accepted) {
    ++stats_.accepted;
  } else {
    ++stats_.withheld;
  }
  trace_.push_back(WorldTraceEntry{event.writer_index, descriptor.snapshot_id(),
                                   step.assessment.status,
                                   step.assessment.withhold});
  chain_input_ += descriptor.snapshot_id();
  chain_input_ += '\n';
  return step;
}

WorldReplaySummary WorldReplayAdapter::Finish() {
  if (finished_) {
    return summary_;
  }
  finished_ = true;
  summary_.status = latched_ ? latched_status_ : ReplayAdapterStatus::kOk;
  summary_.degraded = degraded_;
  summary_.chain_digest =
      internal::EpisodeSha256Hex(std::string("v1\n") + chain_input_);
  summary_.trace = trace_;
  summary_.stats = stats_;
  return summary_;
}

}  // namespace intrinsic::episode
