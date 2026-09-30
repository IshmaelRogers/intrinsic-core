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

#ifndef INTRINSIC_HARDWARE_MARINE_MEASUREMENT_HEALTH_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_MEASUREMENT_HEALTH_POLICY_H_

#include <cstdint>
#include <span>
#include <string_view>

#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for MeasurementHealth. These functions do not parse
// protobuf, do not convert ENU and NED, and do not command hardware.
//
// `state` on the view is the measurement-local enum (UNKNOWN=0, VALID=1,
// DEGRADED=2, INVALID=3). It is not embodiment Validity. Unset state
// (`state_present == false`) is an absent judgment. A present 0 is UNKNOWN.
// Numbers outside 0..3 stay unrecognized: they are not VALID and they are
// not rewritten to INVALID.
//
// Embodiment Validity on the stamped header is classified separately and
// does not select `accepted`. Covariance shape is #17 AssessCovariance.

enum class MeasurementStateKind {
  kAbsent = 0,
  kUnknown = 1,
  kValid = 2,
  kDegraded = 3,
  kInvalid = 4,
  kUnrecognized = 5,
};

// field_present is MeasurementHealth.state presence, not the enum value.
inline constexpr MeasurementStateKind ClassifyMeasurementState(
    bool field_present, int state) {
  if (!field_present) {
    return MeasurementStateKind::kAbsent;
  }
  switch (state) {
    case 0:
      return MeasurementStateKind::kUnknown;
    case 1:
      return MeasurementStateKind::kValid;
    case 2:
      return MeasurementStateKind::kDegraded;
    case 3:
      return MeasurementStateKind::kInvalid;
    default:
      return MeasurementStateKind::kUnrecognized;
  }
}

enum class MeasurementError {
  kNone = 0,
  kMissingFrame = 1,
  kWrongFrame = 2,
  kTimeReversal = 3,
  kNonFinite = 4,
  kQuality = 5,
  kCovariance = 6,
  kSourceId = 7,
};

struct MeasurementAssessment {
  MeasurementError error = MeasurementError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for measurement state VALID with no structural defect.
  // An empty message is not accepted and is not an error. DEGRADED,
  // UNKNOWN, INVALID, absent, and unrecognized are not accepted.
  bool accepted = false;
};

struct SourceHealthView {
  std::string_view source_id;
  bool validity_present = false;
  int validity_state = 0;
};

struct MeasurementHealthView {
  bool header_present = false;
  std::string_view frame_id;
  // When set, frame_id must match exactly. Empty means the caller did not
  // name a frame. The message type never supplies one.
  bool expected_frame_present = false;
  std::string_view expected_frame;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  bool header_validity_present = false;
  int header_validity_state = 0;
  bool state_present = false;
  int state = 0;
  bool quality_present = false;
  double quality = 0;
  // Span is valid only while the caller's storage is alive.
  bool covariance_present = false;
  std::span<const double> covariance;
  std::span<const SourceHealthView> sources;
};

inline bool MeasurementEngaged(const MeasurementHealthView& sample) {
  return sample.header_present || sample.state_present ||
         sample.quality_present || sample.covariance_present ||
         !sample.sources.empty() || sample.source_time_present ||
         sample.receive_time_present;
}

// True when `lhs` is strictly earlier than `rhs` in (seconds, nanos) order.
inline bool TimestampPrecedes(embodiment::ClockReading lhs,
                              embodiment::ClockReading rhs) {
  if (lhs.seconds != rhs.seconds) {
    return lhs.seconds < rhs.seconds;
  }
  return lhs.nanos < rhs.nanos;
}

inline bool QualityInRange(double quality) {
  return embodiment::IsFinite(quality) && quality >= 0.0 && quality <= 1.0;
}

// Check order: frame, caller frame, source-time reversal, quality finiteness,
// quality range, covariance shape, source ids. The first defect wins.
// Absent optional fields are not defects. Header Validity is not a defect
// and is not rewritten from `state`.
inline MeasurementAssessment AssessMeasurementHealth(
    const MeasurementHealthView& sample) {
  const embodiment::ValidityKind header_validity = embodiment::ClassifyValidity(
      sample.header_validity_present, sample.header_validity_state);
  if (!MeasurementEngaged(sample)) {
    return MeasurementAssessment{MeasurementError::kNone,
                                 MeasurementStateKind::kAbsent, header_validity,
                                 false};
  }
  const MeasurementStateKind state =
      ClassifyMeasurementState(sample.state_present, sample.state);
  MeasurementError error = MeasurementError::kNone;
  if (sample.frame_id.empty()) {
    error = MeasurementError::kMissingFrame;
  } else if (sample.expected_frame_present &&
             sample.frame_id != sample.expected_frame) {
    error = MeasurementError::kWrongFrame;
  } else if (sample.source_time_present && sample.receive_time_present &&
             TimestampPrecedes(sample.receive_time, sample.source_time)) {
    error = MeasurementError::kTimeReversal;
  } else if (sample.quality_present && !embodiment::IsFinite(sample.quality)) {
    error = MeasurementError::kNonFinite;
  } else if (sample.quality_present && !QualityInRange(sample.quality)) {
    error = MeasurementError::kQuality;
  } else {
    const vehicle::CovarianceError covariance =
        vehicle::AssessCovariance(sample.covariance_present, sample.covariance);
    if (covariance != vehicle::CovarianceError::kNone &&
        covariance != vehicle::CovarianceError::kAbsent) {
      error = MeasurementError::kCovariance;
    }
  }
  if (error == MeasurementError::kNone) {
    for (const SourceHealthView& source : sample.sources) {
      if (source.source_id.empty()) {
        error = MeasurementError::kSourceId;
        break;
      }
    }
  }
  const bool accepted =
      error == MeasurementError::kNone && state == MeasurementStateKind::kValid;
  return MeasurementAssessment{error, state, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_MEASUREMENT_HEALTH_POLICY_H_
