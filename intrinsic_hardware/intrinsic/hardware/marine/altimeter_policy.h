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

#ifndef INTRINSIC_HARDWARE_MARINE_ALTIMETER_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_ALTIMETER_POLICY_H_

#include <span>
#include <string_view>

#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for AltimeterMeasurement. These functions do not parse
// protobuf, do not convert ENU and NED, do not fuse bathymetry, and do not
// command hardware.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.
//
// Explicit no-return is `has_return_present && !has_return`. Unset
// `has_return` is absent and is not no-return. A present range with unset
// `has_return` is a range-only sample.

enum class AltimeterError {
  kNone = 0,
  kMissingHealth = 1,
  kMissingFrame = 2,
  kWrongFrame = 3,
  kTimeReversal = 4,
  kNonFinite = 5,
  kQuality = 6,
  kCovariance = 7,
  kSourceId = 8,
  kCovarianceSlots = 9,
  kPayload = 10,
  kRange = 11,
  kBounds = 12,
  kNoReturn = 13,
  kBeamId = 14,
};

// Row-major Matrix6 diagonal. Index 0 is range variance (m^2). The other
// 35 entries stay exactly 0 when covariance is present.
inline constexpr int kRangeVarianceSlot = vehicle::CovarianceIndex(0, 0);

struct AltimeterAssessment {
  AltimeterError error = AltimeterError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID and no structural defect. An empty
  // message is not accepted and is not an error.
  bool accepted = false;
};

struct AltimeterMeasurementView {
  // Span inside `health.covariance` and `beam_id` are valid only while
  // caller storage lives.
  MeasurementHealthView health;
  bool range_present = false;
  double range_m = 0;
  bool beam_present = false;
  std::string_view beam_id;
  bool min_present = false;
  double min_range_m = 0;
  bool max_present = false;
  double max_range_m = 0;
  bool has_return_present = false;
  bool has_return = false;
};

inline bool ExplicitNoReturn(const AltimeterMeasurementView& sample) {
  return sample.has_return_present && !sample.has_return;
}

inline bool AltimeterEngaged(const AltimeterMeasurementView& sample) {
  return MeasurementEngaged(sample.health) || sample.range_present ||
         sample.beam_present || sample.min_present || sample.max_present ||
         sample.has_return_present;
}

inline bool AllowedRangeCovarianceOnly(std::span<const double> values) {
  if (static_cast<int>(values.size()) != vehicle::kCovarianceValues) {
    return false;
  }
  for (int index = 0; index < vehicle::kCovarianceValues; ++index) {
    if (index == kRangeVarianceSlot) {
      continue;
    }
    if (values[index] != 0.0) {
      return false;
    }
  }
  return true;
}

inline AltimeterError ToAltimeterError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return AltimeterError::kNone;
    case MeasurementError::kMissingFrame:
      return AltimeterError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return AltimeterError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return AltimeterError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return AltimeterError::kNonFinite;
    case MeasurementError::kQuality:
      return AltimeterError::kQuality;
    case MeasurementError::kCovariance:
      return AltimeterError::kCovariance;
    case MeasurementError::kSourceId:
      return AltimeterError::kSourceId;
  }
  return AltimeterError::kNone;
}

// Check order: missing health, then #75 health order, then covariance
// slots, payload presence, range finiteness, range sign, bound finiteness
// and sign, min <= max, range inside present bounds, no-return consistency,
// empty beam id. The first defect wins. Absent optional fields are not
// defects. Health state is not rewritten.
inline AltimeterAssessment AssessAltimeter(
    const AltimeterMeasurementView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  if (!AltimeterEngaged(sample)) {
    return AltimeterAssessment{AltimeterError::kNone,
                               MeasurementStateKind::kAbsent, header_validity,
                               false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  AltimeterError error = AltimeterError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = AltimeterError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = ToAltimeterError(health.error);
    if (error == AltimeterError::kNone && sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AllowedRangeCovarianceOnly(sample.health.covariance)) {
      error = AltimeterError::kCovarianceSlots;
    }
  }
  if (error == AltimeterError::kNone && !sample.range_present &&
      !ExplicitNoReturn(sample)) {
    error = AltimeterError::kPayload;
  }
  if (error == AltimeterError::kNone && sample.range_present) {
    if (!embodiment::IsFinite(sample.range_m)) {
      error = AltimeterError::kNonFinite;
    } else if (sample.range_m < 0.0) {
      error = AltimeterError::kRange;
    }
  }
  if (error == AltimeterError::kNone && sample.min_present) {
    if (!embodiment::IsFinite(sample.min_range_m)) {
      error = AltimeterError::kNonFinite;
    } else if (sample.min_range_m < 0.0) {
      error = AltimeterError::kBounds;
    }
  }
  if (error == AltimeterError::kNone && sample.max_present) {
    if (!embodiment::IsFinite(sample.max_range_m)) {
      error = AltimeterError::kNonFinite;
    } else if (sample.max_range_m < 0.0) {
      error = AltimeterError::kBounds;
    }
  }
  if (error == AltimeterError::kNone && sample.min_present &&
      sample.max_present && sample.min_range_m > sample.max_range_m) {
    error = AltimeterError::kBounds;
  }
  if (error == AltimeterError::kNone && sample.range_present &&
      sample.min_present && sample.range_m < sample.min_range_m) {
    error = AltimeterError::kBounds;
  }
  if (error == AltimeterError::kNone && sample.range_present &&
      sample.max_present && sample.range_m > sample.max_range_m) {
    error = AltimeterError::kBounds;
  }
  if (error == AltimeterError::kNone && ExplicitNoReturn(sample) &&
      state == MeasurementStateKind::kValid) {
    error = AltimeterError::kNoReturn;
  }
  if (error == AltimeterError::kNone && ExplicitNoReturn(sample) &&
      sample.range_present) {
    error = AltimeterError::kNoReturn;
  }
  if (error == AltimeterError::kNone && sample.beam_present &&
      sample.beam_id.empty()) {
    error = AltimeterError::kBeamId;
  }
  const bool accepted =
      error == AltimeterError::kNone && state == MeasurementStateKind::kValid;
  return AltimeterAssessment{error, state, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_ALTIMETER_POLICY_H_
