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

#ifndef INTRINSIC_SAFETY_SAFETY_DECISION_POLICY_H_
#define INTRINSIC_SAFETY_SAFETY_DECISION_POLICY_H_

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::safety {

// Policy: intrinsic_apis/intrinsic/safety/proto/README.md.
//
// Plain-value checks for the SafetyDecision proto. These functions do not
// parse protobuf, do not evaluate any safety rule, and do not call ICON, a
// HAL, or an actuator API.

inline constexpr size_t kSha256HexLength = 64;

enum class SafetyDecisionKind {
  kUnspecified = 0,
  kAccept = 1,
  kProject = 2,
  kReject = 3,
  kAbort = 4,
  kSurface = 5,
  // Any other wire number. Never rewritten to kReject or kAbort.
  kUnknown = 6,
};

// Wire values other than 0..5 are kUnknown.
inline constexpr SafetyDecisionKind ClassifySafetyDecisionKind(int kind) {
  switch (kind) {
    case 0:
      return SafetyDecisionKind::kUnspecified;
    case 1:
      return SafetyDecisionKind::kAccept;
    case 2:
      return SafetyDecisionKind::kProject;
    case 3:
      return SafetyDecisionKind::kReject;
    case 4:
      return SafetyDecisionKind::kAbort;
    case 5:
      return SafetyDecisionKind::kSurface;
    default:
      return SafetyDecisionKind::kUnknown;
  }
}

// True for ACCEPT, PROJECT, REJECT, ABORT, and SURFACE.
inline constexpr bool IsKnownCommittedKind(SafetyDecisionKind kind) {
  return kind != SafetyDecisionKind::kUnspecified &&
         kind != SafetyDecisionKind::kUnknown;
}

// Kinds whose applied_intent must be present.
inline constexpr bool KindRequiresAppliedIntent(SafetyDecisionKind kind) {
  return kind == SafetyDecisionKind::kProject ||
         kind == SafetyDecisionKind::kSurface;
}

// Kinds whose applied_intent must be absent.
inline constexpr bool KindForbidsAppliedIntent(SafetyDecisionKind kind) {
  return kind == SafetyDecisionKind::kAccept ||
         kind == SafetyDecisionKind::kReject ||
         kind == SafetyDecisionKind::kAbort;
}

enum class SafetyFindingSeverityKind {
  kUnspecified = 0,
  kInfo = 1,
  kWarning = 2,
  kError = 3,
  kCritical = 4,
  // Any other wire number. Kept as is. Not an error.
  kUnknown = 5,
};

inline constexpr SafetyFindingSeverityKind ClassifySafetyFindingSeverity(
    int severity) {
  switch (severity) {
    case 0:
      return SafetyFindingSeverityKind::kUnspecified;
    case 1:
      return SafetyFindingSeverityKind::kInfo;
    case 2:
      return SafetyFindingSeverityKind::kWarning;
    case 3:
      return SafetyFindingSeverityKind::kError;
    case 4:
      return SafetyFindingSeverityKind::kCritical;
    default:
      return SafetyFindingSeverityKind::kUnknown;
  }
}

// Exactly 64 characters, each in [0-9a-f]. Uppercase is rejected. This is
// the form of WorldSnapshotDescriptor.snapshot_id and of a SHA-256 digest.
inline bool IsLowercaseSha256Hex(std::string_view value) {
  if (value.size() != kSha256HexLength) {
    return false;
  }
  for (char c : value) {
    const bool digit = c >= '0' && c <= '9';
    const bool lower = c >= 'a' && c <= 'f';
    if (!digit && !lower) {
      return false;
    }
  }
  return true;
}

enum class SafetyDecisionError {
  kNone = 0,
  kMissingHeader = 1,
  kHeaderTime = 2,
  kHeaderInvalid = 3,
  kUnspecifiedKind = 4,
  kOriginalIntentDigest = 5,
  kSnapshotId = 6,
  kAppliedIntentMissing = 7,
  kAppliedIntentUnexpected = 8,
  kAppliedIntentInvalid = 9,
  kFindingRuleId = 10,
};

struct SafetyFindingView {
  std::string_view rule_id;
  // Raw wire number. Classify with ClassifySafetyFindingSeverity.
  int severity = 0;
};

struct SafetyDecisionView {
  bool header_present = false;
  bool header_validity_present = false;
  int header_validity_state = 0;
  bool header_source_time_present = false;
  int32_t header_source_time_nanos = 0;
  bool header_receive_time_present = false;
  int32_t header_receive_time_nanos = 0;
  // Raw wire number. Classify with ClassifySafetyDecisionKind.
  int kind = 0;
  std::string_view original_intent_digest;
  bool applied_intent_present = false;
  vehicle::DesiredMotionView applied_intent;
  std::string_view snapshot_id;
  // Spans are valid only while the caller's storage is alive.
  std::span<const SafetyFindingView> findings;
  bool decision_epoch_present = false;
  uint64_t decision_epoch = 0;
};

struct SafetyDecisionAssessment {
  // False for an empty decision. An unengaged decision is not an error and
  // is not accepted.
  bool engaged = false;
  SafetyDecisionError error = SafetyDecisionError::kNone;
  // Index of the first finding with an empty rule_id. -1 otherwise.
  int finding_index = -1;
  // Delegated DesiredMotion defect. kNone unless error is
  // kAppliedIntentInvalid.
  vehicle::ContractError applied_intent_error = vehicle::ContractError::kNone;
  // Classification of the wire kind. Unknown stays kUnknown.
  SafetyDecisionKind kind = SafetyDecisionKind::kUnspecified;
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only when the decision is engaged, has no structural defect, the
  // header is STATE_VALID, and kind is ACCEPT, PROJECT, REJECT, ABORT, or
  // SURFACE. An unknown kind is never accepted. The message is not changed.
  bool accepted = false;
};

inline bool DecisionEngaged(const SafetyDecisionView& decision) {
  return decision.header_present || decision.kind != 0 ||
         !decision.original_intent_digest.empty() ||
         decision.applied_intent_present || !decision.snapshot_id.empty() ||
         !decision.findings.empty() || decision.decision_epoch_present ||
         decision.decision_epoch != 0;
}

// Check order, first defect wins: header presence and stamp, unspecified
// kind, original_intent_digest, snapshot_id, applied_intent presence and
// DesiredMotion assessment, finding rule ids. An unknown kind is not a
// structural defect: it is classified kUnknown, the applied_intent rules are
// not applied to it, and it is never accepted. Severity is never a defect.
inline SafetyDecisionAssessment AssessSafetyDecision(
    const SafetyDecisionView& decision) {
  SafetyDecisionAssessment result;
  if (!DecisionEngaged(decision)) {
    return result;
  }
  result.engaged = true;
  result.kind = ClassifySafetyDecisionKind(decision.kind);
  result.header_validity = embodiment::ClassifyValidity(
      decision.header_validity_present, decision.header_validity_state);

  if (!decision.header_present) {
    result.error = SafetyDecisionError::kMissingHeader;
  } else if ((decision.header_source_time_present &&
              !embodiment::NanosInRange(decision.header_source_time_nanos)) ||
             (decision.header_receive_time_present &&
              !embodiment::NanosInRange(decision.header_receive_time_nanos))) {
    result.error = SafetyDecisionError::kHeaderTime;
  } else if (result.header_validity == embodiment::ValidityKind::kInvalid) {
    result.error = SafetyDecisionError::kHeaderInvalid;
  } else if (result.kind == SafetyDecisionKind::kUnspecified) {
    result.error = SafetyDecisionError::kUnspecifiedKind;
  } else if (!IsLowercaseSha256Hex(decision.original_intent_digest)) {
    result.error = SafetyDecisionError::kOriginalIntentDigest;
  } else if (!IsLowercaseSha256Hex(decision.snapshot_id)) {
    result.error = SafetyDecisionError::kSnapshotId;
  } else if (KindRequiresAppliedIntent(result.kind) &&
             !decision.applied_intent_present) {
    result.error = SafetyDecisionError::kAppliedIntentMissing;
  } else if (KindForbidsAppliedIntent(result.kind) &&
             decision.applied_intent_present) {
    result.error = SafetyDecisionError::kAppliedIntentUnexpected;
  } else if (KindRequiresAppliedIntent(result.kind)) {
    const vehicle::ContractAssessment applied =
        vehicle::AssessDesiredMotion(decision.applied_intent);
    if (!applied.accepted) {
      result.error = SafetyDecisionError::kAppliedIntentInvalid;
      result.applied_intent_error = applied.error;
    }
  }
  if (result.error == SafetyDecisionError::kNone) {
    for (size_t i = 0; i < decision.findings.size(); ++i) {
      if (decision.findings[i].rule_id.empty()) {
        result.error = SafetyDecisionError::kFindingRuleId;
        result.finding_index = static_cast<int>(i);
        break;
      }
    }
  }
  result.accepted = result.error == SafetyDecisionError::kNone &&
                    IsKnownCommittedKind(result.kind) &&
                    embodiment::SampleAccepted(result.header_validity, true);
  return result;
}

inline const char* SafetyDecisionErrorName(SafetyDecisionError error) {
  switch (error) {
    case SafetyDecisionError::kNone:
      return "none";
    case SafetyDecisionError::kMissingHeader:
      return "missing_header";
    case SafetyDecisionError::kHeaderTime:
      return "header_time";
    case SafetyDecisionError::kHeaderInvalid:
      return "header_invalid";
    case SafetyDecisionError::kUnspecifiedKind:
      return "unspecified_kind";
    case SafetyDecisionError::kOriginalIntentDigest:
      return "original_intent_digest";
    case SafetyDecisionError::kSnapshotId:
      return "snapshot_id";
    case SafetyDecisionError::kAppliedIntentMissing:
      return "applied_intent_missing";
    case SafetyDecisionError::kAppliedIntentUnexpected:
      return "applied_intent_unexpected";
    case SafetyDecisionError::kAppliedIntentInvalid:
      return "applied_intent_invalid";
    case SafetyDecisionError::kFindingRuleId:
      return "finding_rule_id";
  }
  return "unknown";
}

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_SAFETY_DECISION_POLICY_H_
