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

#include "intrinsic_data/episode/inference_replay_adapter.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message_lite.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/inference/proto/inference_envelope.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::inference::InferenceEnvelope;
using intrinsic_proto::inference::InferenceResult;

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

// Identical field for field to CommonViewOf in
// inference_envelope_contract_serialization_test.cc.
inference::InferenceCommonView CommonViewOf(const InferenceEnvelope& message) {
  const auto& header = message.header();
  inference::InferenceCommonView common;
  common.header_present = message.has_header();
  common.validity_present = header.has_validity();
  common.validity_state = static_cast<int>(header.validity().state());
  common.frame_id = header.frame_id();
  common.source_time_present = header.has_source_time();
  common.source_time = {header.source_time().seconds(),
                        header.source_time().nanos()};
  common.state_epoch = message.state_epoch();
  common.world_snapshot_id = message.world_snapshot_id();
  common.deadline_present = message.has_deadline();
  common.deadline = {message.deadline().seconds(), message.deadline().nanos()};
  common.validity_horizon_present = message.has_validity_horizon();
  common.validity_horizon = {message.validity_horizon().seconds(),
                             message.validity_horizon().nanos()};
  common.confidence_present = message.has_confidence();
  common.confidence = message.confidence();
  common.uncertainty_present = message.has_uncertainty();
  common.uncertainty = message.uncertainty();
  common.input_digest = message.input_digest();
  common.provenance_present = message.has_provenance();
  common.provenance_model_id = message.provenance().model_id();
  common.metadata_present = !message.metadata().empty();
  return common;
}

inference::InferenceCommonView CommonViewOf(const InferenceResult& message) {
  const auto& header = message.header();
  inference::InferenceCommonView common;
  common.header_present = message.has_header();
  common.validity_present = header.has_validity();
  common.validity_state = static_cast<int>(header.validity().state());
  common.frame_id = header.frame_id();
  common.source_time_present = header.has_source_time();
  common.source_time = {header.source_time().seconds(),
                        header.source_time().nanos()};
  common.state_epoch = message.state_epoch();
  common.world_snapshot_id = message.world_snapshot_id();
  common.deadline_present = message.has_deadline();
  common.deadline = {message.deadline().seconds(), message.deadline().nanos()};
  common.validity_horizon_present = message.has_validity_horizon();
  common.validity_horizon = {message.validity_horizon().seconds(),
                             message.validity_horizon().nanos()};
  common.confidence_present = message.has_confidence();
  common.confidence = message.confidence();
  common.uncertainty_present = message.has_uncertainty();
  common.uncertainty = message.uncertainty();
  common.input_digest = message.input_digest();
  common.provenance_present = message.has_provenance();
  common.provenance_model_id = message.provenance().model_id();
  common.metadata_present = !message.metadata().empty();
  return common;
}

inference::InferenceEnvelopeContractAssessment Assess(
    const InferenceEnvelope& message) {
  inference::InferenceEnvelopeView view;
  view.common = CommonViewOf(message);
  view.oip_request.message_set = message.has_oip_request();
  view.oip_request.model_name = message.oip_request().model().model_name();
  view.oip_request.model_version =
      message.oip_request().model().model_version();
  view.oip_request.request_id = message.oip_request().request_id();
  return inference::AssessInferenceEnvelope(view);
}

inference::InferenceEnvelopeContractAssessment Assess(
    const InferenceResult& message) {
  inference::InferenceResultView view;
  view.common = CommonViewOf(message);
  view.oip_result.message_set = message.has_oip_result();
  view.oip_result.model_name = message.oip_result().model().model_name();
  view.oip_result.model_version = message.oip_result().model().model_version();
  view.oip_result.request_id = message.oip_result().request_id();
  view.output_digest = message.output_digest();
  return inference::AssessInferenceResult(view);
}

// The inner header.receive_time is the replay clock. An absent time or one with
// nanos out of range cannot judge a deadline.
bool ReceiveTimeUsable(
    const intrinsic_proto::embodiment::StampedHeader& header) {
  return header.has_receive_time() &&
         embodiment::NanosInRange(header.receive_time().nanos());
}

embodiment::ClockReading ReceiveTimeOf(
    const intrinsic_proto::embodiment::StampedHeader& header) {
  return {header.receive_time().seconds(), header.receive_time().nanos()};
}

}  // namespace

InferenceReplayAdapter::InferenceReplayAdapter(InferenceReplayConfig config)
    : config_(std::move(config)) {}

std::unique_ptr<InferenceReplayAdapter> InferenceReplayAdapter::Create(
    const InferenceReplayConfig& config, ReplayAdapterStatus* status) {
  ReplayAdapterStatus local_status = ReplayAdapterStatus::kInvalidConfig;
  ReplayAdapterStatus* out = status != nullptr ? status : &local_status;
  *out = ReplayAdapterStatus::kInvalidConfig;
  if (!CompatibilityValid(config.compatibility)) {
    return nullptr;
  }
  *out = ReplayAdapterStatus::kOk;
  return std::unique_ptr<InferenceReplayAdapter>(
      new InferenceReplayAdapter(config));
}

void InferenceReplayAdapter::SweepMissing(embodiment::ClockReading now,
                                          const Pending* exclude_key,
                                          InferenceReplayStep* step) {
  std::vector<Pending> kept;
  kept.reserve(pending_.size());
  for (Pending& pending : pending_) {
    const bool excluded = exclude_key != nullptr &&
                          pending.model_id == exclude_key->model_id &&
                          pending.input_digest == exclude_key->input_digest &&
                          pending.state_epoch == exclude_key->state_epoch;
    if (excluded || !inference::TimeBefore(pending.deadline, now)) {
      kept.push_back(std::move(pending));
      continue;
    }
    InferenceOutcomeEntry entry;
    entry.writer_index = pending.writer_index;
    entry.outcome = InferenceOutcome::kMissing;
    entry.expired = true;
    entry.envelope_writer_index = pending.writer_index;
    entry.world_snapshot_id = std::move(pending.world_snapshot_id);
    ++stats_.missing_expired;
    degraded_ = true;
    outcomes_.push_back(entry);
    step->outcomes.push_back(std::move(entry));
  }
  pending_ = std::move(kept);
}

InferenceReplayStep InferenceReplayAdapter::Feed(
    const EpisodeReplayEvent& event) {
  InferenceReplayStep step;
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
  if (kind == PayloadKind::kInferenceEnvelope) {
    const InferenceEnvelope& envelope = parsed.inference_envelope();
    const inference::InferenceEnvelopeContractAssessment assessment =
        Assess(envelope);
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    ++stats_.envelopes;
    if (step.accepted) {
      ++stats_.envelopes_accepted;
      if (ReceiveTimeUsable(envelope.header())) {
        SweepMissing(ReceiveTimeOf(envelope.header()), nullptr, &step);
      }
      Pending pending;
      pending.model_id = envelope.provenance().model_id();
      pending.input_digest = envelope.input_digest();
      pending.state_epoch = envelope.state_epoch();
      pending.deadline = {envelope.deadline().seconds(),
                          envelope.deadline().nanos()};
      pending.provenance_digest = envelope.provenance().digest();
      pending.writer_index = event.writer_index;
      pending.world_snapshot_id = envelope.world_snapshot_id();
      pending_.push_back(std::move(pending));
    }
  } else if (kind == PayloadKind::kInferenceResult) {
    const InferenceResult& result = parsed.inference_result();
    const inference::InferenceEnvelopeContractAssessment assessment =
        Assess(result);
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    ++stats_.results;
    if (step.accepted) {
      ++stats_.results_accepted;
      const embodiment::ClockReading now = ReceiveTimeOf(result.header());
      const bool time_usable = ReceiveTimeUsable(result.header());
      Pending key;
      key.model_id = result.provenance().model_id();
      key.input_digest = result.input_digest();
      key.state_epoch = result.state_epoch();
      if (time_usable) {
        SweepMissing(now, &key, &step);
      } else {
        ++stats_.unusable_receive_time;
        step.receive_time_unusable = true;
        degraded_ = true;
      }
      InferenceOutcomeEntry entry;
      entry.writer_index = event.writer_index;
      entry.world_snapshot_id = result.world_snapshot_id();
      size_t match = pending_.size();
      for (size_t i = 0; i < pending_.size(); ++i) {
        if (pending_[i].model_id == key.model_id &&
            pending_[i].input_digest == key.input_digest &&
            pending_[i].state_epoch == key.state_epoch) {
          match = i;
          break;
        }
      }
      if (match == pending_.size()) {
        entry.outcome = InferenceOutcome::kReplayMiss;
        ++stats_.orphans;
      } else {
        const Pending paired = pending_[match];
        pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(match));
        const embodiment::ClockReading deadline = {result.deadline().seconds(),
                                                   result.deadline().nanos()};
        if (!time_usable) {
          entry.outcome = InferenceOutcome::kComplete;
        } else {
          entry.outcome = inference::TimeBefore(deadline, now)
                              ? InferenceOutcome::kReplayExpired
                              : InferenceOutcome::kComplete;
        }
        entry.envelope_writer_index = paired.writer_index;
        entry.model_digest_mismatch =
            !paired.provenance_digest.empty() &&
            paired.provenance_digest != result.provenance().digest();
        if (entry.model_digest_mismatch) {
          ++stats_.model_digest_mismatches;
          degraded_ = true;
        }
        if (time_usable) {
          if (entry.outcome == InferenceOutcome::kComplete) {
            ++stats_.complete;
          } else {
            ++stats_.expired;
          }
          chain_input_ += DeterministicSha256Hex(result);
          chain_input_ += '\n';
        }
      }
      outcomes_.push_back(entry);
      step.outcomes.push_back(std::move(entry));
    }
  } else {
    ++stats_.ignored;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }
  step.status = ReplayAdapterStatus::kOk;
  trace_.push_back(InferenceTraceEntry{event.writer_index, step.payload_kind,
                                       step.error, step.accepted});
  return step;
}

InferenceReplaySummary InferenceReplayAdapter::Finish() {
  if (finished_) {
    return summary_;
  }
  finished_ = true;
  for (Pending& pending : pending_) {
    InferenceOutcomeEntry entry;
    entry.writer_index = pending.writer_index;
    entry.outcome = InferenceOutcome::kMissing;
    entry.expired = false;
    entry.envelope_writer_index = pending.writer_index;
    entry.world_snapshot_id = std::move(pending.world_snapshot_id);
    ++stats_.missing_unresolved;
    degraded_ = true;
    outcomes_.push_back(std::move(entry));
  }
  pending_.clear();
  summary_.status = latched_ ? latched_status_ : ReplayAdapterStatus::kOk;
  summary_.degraded = degraded_;
  summary_.inference_chain =
      internal::EpisodeSha256Hex(std::string("v1\n") + chain_input_);
  summary_.trace = trace_;
  summary_.outcomes = outcomes_;
  summary_.stats = stats_;
  return summary_;
}

}  // namespace intrinsic::episode
