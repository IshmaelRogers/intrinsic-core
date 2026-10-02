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

#ifndef INTRINSIC_INFERENCE_ENVELOPE_INFERENCE_ENVELOPE_CONTRACT_POLICY_H_
#define INTRINSIC_INFERENCE_ENVELOPE_INFERENCE_ENVELOPE_CONTRACT_POLICY_H_

#include <cmath>
#include <cstdint>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::inference {

// Policy: intrinsic_apis/intrinsic/inference/proto/README.md.
//
// Plain-value checks for InferenceEnvelope and InferenceResult. These
// functions do not parse protobuf, do not call OIP or Triton, do not queue,
// submit, poll, or cancel, and do not call World, ICON, or a HAL. Expiry
// consumption is a later leaf. This policy rejects structurally invalid
// envelopes only.

enum class InferenceEnvelopeContractError {
  kNone = 0,
  kMissingFrame = 1,
  kCreationTime = 2,
  kDeadline = 3,
  kValidityHorizon = 4,
  kOipIdentifier = 5,
  kProvenance = 6,
  kConfidence = 7,
  kUncertainty = 8,
  kDigest = 9,
  kSnapshotId = 10,
};

struct InferenceEnvelopeContractAssessment {
  InferenceEnvelopeContractError error = InferenceEnvelopeContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only when error == kNone and the header validity is STATE_VALID. An
  // empty message is not engaged, is not accepted, and is not an error.
  bool accepted = false;
};

// Seconds and nanos, matching google.protobuf.Duration on the wire.
struct DurationParts {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

struct OipIdentifierView {
  // Wire presence of the OipRequestIdentifier or OipResultIdentifier message.
  bool message_set = false;
  std::string_view model_name;
  std::string_view model_version;
  std::string_view request_id;
};

// Fields shared by InferenceEnvelope and InferenceResult.
struct InferenceCommonView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  // Header source_time is the creation time the deadline is checked against.
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  uint64_t state_epoch = 0;
  std::string_view world_snapshot_id;
  bool deadline_present = false;
  embodiment::ClockReading deadline;
  bool validity_horizon_present = false;
  DurationParts validity_horizon;
  bool confidence_present = false;
  double confidence = 0.0;
  bool uncertainty_present = false;
  double uncertainty = 0.0;
  std::string_view input_digest;
  bool provenance_present = false;
  std::string_view provenance_model_id;
  // True when the metadata map has any entry. Values are not inspected.
  bool metadata_present = false;
};

struct InferenceEnvelopeView {
  InferenceCommonView common;
  OipIdentifierView oip_request;
};

struct InferenceResultView {
  InferenceCommonView common;
  OipIdentifierView oip_result;
  std::string_view output_digest;
};

inline bool OipIdentifierSet(const OipIdentifierView& oip) {
  return oip.message_set || !oip.model_name.empty() ||
         !oip.model_version.empty() || !oip.request_id.empty();
}

// Engaged when any field carries a signal: header, OIP ids, a non-zero
// state_epoch, snapshot id, deadline, horizon, confidence, uncertainty,
// digests, provenance, or metadata. A present zero confidence, uncertainty,
// or horizon engages. state_epoch 0 alone does not.
inline bool InferenceCommonEngaged(const InferenceCommonView& common,
                                   const OipIdentifierView& oip,
                                   std::string_view output_digest) {
  return common.header_present || common.validity_present ||
         !common.frame_id.empty() || common.source_time_present ||
         OipIdentifierSet(oip) || common.state_epoch != 0 ||
         !common.world_snapshot_id.empty() || common.deadline_present ||
         common.validity_horizon_present || common.confidence_present ||
         common.uncertainty_present || !common.input_digest.empty() ||
         !output_digest.empty() || common.provenance_present ||
         !common.provenance_model_id.empty() || common.metadata_present;
}

inline bool IsLowercaseSha256Hex(std::string_view value) {
  if (value.size() != 64) {
    return false;
  }
  for (const char c : value) {
    const bool digit = c >= '0' && c <= '9';
    const bool hex = c >= 'a' && c <= 'f';
    if (!digit && !hex) {
      return false;
    }
  }
  return true;
}

inline bool ConfidenceInRange(double confidence) {
  return embodiment::IsFinite(confidence) && confidence >= 0.0 &&
         confidence <= 1.0;
}

inline bool UncertaintyInRange(double uncertainty) {
  return embodiment::IsFinite(uncertainty) && uncertainty >= 0.0;
}

inline bool DurationNonNegative(DurationParts duration) {
  return duration.seconds >= 0 && embodiment::NanosInRange(duration.nanos);
}

inline bool TimeBefore(embodiment::ClockReading a, embodiment::ClockReading b) {
  return a.seconds < b.seconds || (a.seconds == b.seconds && a.nanos < b.nanos);
}

inline InferenceEnvelopeContractAssessment MakeInferenceAssessment(
    InferenceEnvelopeContractError error, embodiment::ValidityKind validity) {
  InferenceEnvelopeContractAssessment assessment;
  assessment.error = error;
  assessment.validity = validity;
  assessment.accepted = error == InferenceEnvelopeContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  return assessment;
}

// Check order, first defect wins: frame, creation time, deadline, validity
// horizon, OIP ids, provenance, confidence, uncertainty, digests, snapshot id.
// Metadata is always tolerated. The digest step needs input_digest, and also
// output_digest when require_output_digest is true (results).
inline InferenceEnvelopeContractError AssessInferenceCommon(
    const InferenceCommonView& common, const OipIdentifierView& oip,
    std::string_view output_digest, bool require_output_digest) {
  using Error = InferenceEnvelopeContractError;
  if (common.frame_id.empty()) {
    return Error::kMissingFrame;
  }
  if (!common.source_time_present ||
      !embodiment::NanosInRange(common.source_time.nanos)) {
    return Error::kCreationTime;
  }
  if (!common.deadline_present ||
      !embodiment::NanosInRange(common.deadline.nanos) ||
      TimeBefore(common.deadline, common.source_time)) {
    return Error::kDeadline;
  }
  if (!common.validity_horizon_present ||
      !DurationNonNegative(common.validity_horizon)) {
    return Error::kValidityHorizon;
  }
  if (oip.model_name.empty() || oip.request_id.empty()) {
    return Error::kOipIdentifier;
  }
  if (!common.provenance_present || common.provenance_model_id.empty()) {
    return Error::kProvenance;
  }
  if (common.confidence_present && !ConfidenceInRange(common.confidence)) {
    return Error::kConfidence;
  }
  if (common.uncertainty_present && !UncertaintyInRange(common.uncertainty)) {
    return Error::kUncertainty;
  }
  if (common.input_digest.empty() ||
      (require_output_digest && output_digest.empty())) {
    return Error::kDigest;
  }
  if (!common.world_snapshot_id.empty() &&
      !IsLowercaseSha256Hex(common.world_snapshot_id)) {
    return Error::kSnapshotId;
  }
  return Error::kNone;
}

inline InferenceEnvelopeContractAssessment AssessInferenceEnvelope(
    const InferenceEnvelopeView& envelope) {
  if (!InferenceCommonEngaged(envelope.common, envelope.oip_request, {})) {
    return InferenceEnvelopeContractAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      envelope.common.validity_present, envelope.common.validity_state);
  return MakeInferenceAssessment(
      AssessInferenceCommon(envelope.common, envelope.oip_request, {}, false),
      validity);
}

inline InferenceEnvelopeContractAssessment AssessInferenceResult(
    const InferenceResultView& result) {
  if (!InferenceCommonEngaged(result.common, result.oip_result,
                              result.output_digest)) {
    return InferenceEnvelopeContractAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      result.common.validity_present, result.common.validity_state);
  return MakeInferenceAssessment(
      AssessInferenceCommon(result.common, result.oip_result,
                            result.output_digest, true),
      validity);
}

}  // namespace intrinsic::inference

#endif  // INTRINSIC_INFERENCE_ENVELOPE_INFERENCE_ENVELOPE_CONTRACT_POLICY_H_
