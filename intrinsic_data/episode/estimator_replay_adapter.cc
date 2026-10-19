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

#include "intrinsic_data/episode/estimator_replay_adapter.h"

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/estimation/estimator_service.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::estimation::EstimatorResult;
using intrinsic_proto::estimation::GetStateResponse;
using intrinsic_proto::estimation::MeasurementEnvelope;

constexpr char kMeasurementEnvelopeType[] =
    "intrinsic_proto.estimation.MeasurementEnvelope";

// FNV-1a 64 over the deterministic serialization of state(), 0 when unset.
// Byte for byte the FakeEstimatorService::StateDigest definition, computed
// through the EstimatorService interface only.
uint64_t EstimatorStateDigest(const GetStateResponse& response) {
  if (!response.has_state()) {
    return 0;
  }
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    response.state().SerializeToCodedStream(&coded);
  }
  uint64_t hash = 0xCBF29CE484222325ULL;
  for (unsigned char byte : bytes) {
    hash = (hash ^ byte) * 0x100000001B3ULL;
  }
  return hash;
}

bool CompatibilityValid(const ReplayCompatibilityConfig& config) {
  return config.expected_config_digest.empty() ||
         IsLowercaseSha256Hex(config.expected_config_digest);
}

bool RequiredSourcesValid(const std::vector<std::string>& sources) {
  std::set<std::string> seen;
  for (const std::string& source : sources) {
    if (source.empty() || !seen.insert(source).second) {
      return false;
    }
  }
  return true;
}

}  // namespace

EstimatorReplayAdapter::EstimatorReplayAdapter(
    estimation::EstimatorService* service, EstimatorReplayConfig config)
    : service_(service), config_(std::move(config)) {}

std::unique_ptr<EstimatorReplayAdapter> EstimatorReplayAdapter::Create(
    estimation::EstimatorService* service, const EstimatorReplayConfig& config,
    ReplayAdapterStatus* status, EstimatorResult* init_result) {
  ReplayAdapterStatus local_status = ReplayAdapterStatus::kInvalidConfig;
  ReplayAdapterStatus* out = status != nullptr ? status : &local_status;
  *out = ReplayAdapterStatus::kInvalidConfig;
  if (service == nullptr || !CompatibilityValid(config.compatibility) ||
      !RequiredSourcesValid(config.required_sources)) {
    return nullptr;
  }
  const EstimatorResult initialized =
      service->Initialize(config.estimator_config);
  if (init_result != nullptr) {
    *init_result = initialized;
  }
  if (!initialized.ok()) {
    return nullptr;
  }
  *out = ReplayAdapterStatus::kOk;
  return std::unique_ptr<EstimatorReplayAdapter>(
      new EstimatorReplayAdapter(service, config));
}

EstimatorReplayStep EstimatorReplayAdapter::Feed(
    const EpisodeReplayEvent& event) {
  EstimatorReplayStep step;
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
  if (event.record->payload_kind != PayloadKind::kRecordedMessage) {
    ++stats_.ignored;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }
  const auto& recorded = parsed.recorded_message();
  if (recorded.message_type() != kMeasurementEnvelopeType) {
    ++stats_.ignored_message_type;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }
  MeasurementEnvelope envelope;
  if (!envelope.ParseFromString(recorded.message_bytes())) {
    ++stats_.invalid_records;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kInvalidRecord;
    return step;
  }

  step.result = service_->Ingest(envelope);
  step.ingested = true;
  step.state_digest = EstimatorStateDigest(service_->GetState());
  step.status = ReplayAdapterStatus::kOk;
  seen_sources_.insert(envelope.source_id());
  ++stats_.ingested;
  if (step.result.ok()) {
    ++stats_.accepted;
  } else {
    ++stats_.rejected;
  }
  trace_.push_back(EstimatorTraceEntry{event.writer_index, step.result.ok(),
                                       static_cast<int>(step.result.reason()),
                                       step.result.estimator_epoch(),
                                       step.state_digest});
  return step;
}

EstimatorReplaySummary EstimatorReplayAdapter::Finish() {
  if (finished_) {
    return summary_;
  }
  finished_ = true;
  std::set<std::string> missing;
  for (const std::string& source : config_.required_sources) {
    if (seen_sources_.count(source) == 0) {
      missing.insert(source);
    }
  }
  summary_.missing_required_sources.assign(missing.begin(), missing.end());
  if (latched_) {
    summary_.status = latched_status_;
  } else if (!missing.empty()) {
    summary_.status = ReplayAdapterStatus::kMissingRequiredStream;
  } else {
    summary_.status = ReplayAdapterStatus::kOk;
  }
  summary_.degraded = degraded_;
  summary_.final_state_digest = EstimatorStateDigest(service_->GetState());
  summary_.trace = trace_;
  summary_.stats = stats_;
  return summary_;
}

}  // namespace intrinsic::episode
