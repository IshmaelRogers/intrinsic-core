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

#include "intrinsic_data/episode/safety_replay_adapter.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message_lite.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
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

std::string DeterministicSha256Hex(
    const google::protobuf::MessageLite& message) {
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    message.SerializeToCodedStream(&coded);
  }
  return internal::EpisodeSha256Hex(bytes);
}

}  // namespace

SafetyReplayAdapter::SafetyReplayAdapter(SafetyReplayConfig config)
    : config_(std::move(config)) {}

std::unique_ptr<SafetyReplayAdapter> SafetyReplayAdapter::Create(
    const SafetyReplayConfig& config, ReplayAdapterStatus* status) {
  ReplayAdapterStatus local_status = ReplayAdapterStatus::kInvalidConfig;
  ReplayAdapterStatus* out = status != nullptr ? status : &local_status;
  *out = ReplayAdapterStatus::kInvalidConfig;
  if (!CompatibilityValid(config.compatibility)) {
    return nullptr;
  }
  *out = ReplayAdapterStatus::kOk;
  return std::unique_ptr<SafetyReplayAdapter>(new SafetyReplayAdapter(config));
}

SafetyReplayStep SafetyReplayAdapter::Feed(const EpisodeReplayEvent& event) {
  SafetyReplayStep step;
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
  const PayloadKind kind = event.record->payload_kind;
  if (kind == PayloadKind::kDesiredMotion) {
    const auto& motion = parsed.desired_motion();
    const vehicle::ContractAssessment assessment = vehicle::AssessDesiredMotion(
        safety::DesiredMotionViewFromProto(motion));
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    step.payload_sha256 = DeterministicSha256Hex(motion);
    ++stats_.motions;
    seen_intents_.insert(safety::ComputeOriginalIntentDigest(motion));
    if (step.accepted) {
      ++stats_.motions_accepted;
    }
  } else if (kind == PayloadKind::kSafetyDecision) {
    const auto& decision = parsed.safety_decision();
    const safety::SafetyDecisionAssessment assessment =
        safety::AssessSafetyDecisionProto(decision);
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    step.payload_sha256 = DeterministicSha256Hex(decision);
    step.kind = static_cast<int>(assessment.kind);
    step.original_intent_digest = decision.original_intent_digest();
    step.intent_linked = seen_intents_.contains(step.original_intent_digest);
    step.snapshot_id = decision.snapshot_id();
    ++stats_.decisions;
    if (!step.intent_linked) {
      ++stats_.unlinked_decisions;
      degraded_ = true;
    }
    if (step.accepted) {
      ++stats_.decisions_accepted;
      decision_chain_input_ += step.payload_sha256;
      decision_chain_input_ += '\n';
    }
  } else {
    ++stats_.ignored;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }
  step.status = ReplayAdapterStatus::kOk;
  trace_.push_back(SafetyTraceEntry{
      event.writer_index, step.payload_kind, step.error, step.accepted,
      step.kind, step.original_intent_digest, step.intent_linked,
      step.snapshot_id, step.payload_sha256});
  return step;
}

SafetyReplaySummary SafetyReplayAdapter::Finish() {
  if (finished_) {
    return summary_;
  }
  finished_ = true;
  summary_.status = latched_ ? latched_status_ : ReplayAdapterStatus::kOk;
  summary_.degraded = degraded_;
  summary_.decision_chain =
      internal::EpisodeSha256Hex(std::string("v1\n") + decision_chain_input_);
  summary_.trace = trace_;
  summary_.stats = stats_;
  return summary_;
}

}  // namespace intrinsic::episode
