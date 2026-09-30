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

#ifndef INTRINSIC_HARDWARE_MARINE_DVL_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_DVL_POLICY_H_

#include <span>

#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for DvlMeasurement. These functions do not parse
// protobuf, do not convert ENU and NED, and do not command hardware.
//
// `mode` is the measurement-local enum (MODE_UNSPECIFIED=0,
// MODE_BOTTOM_TRACK=1, MODE_WATER_TRACK=2). Unset mode
// (`mode_present == false`) is absent. A present 0 is MODE_UNSPECIFIED.
// Numbers other than 0, 1, and 2 stay unrecognized: they are not a track
// mode and they are not rewritten.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.

enum class DvlModeKind {
  kAbsent = 0,
  kUnspecified = 1,
  kBottomTrack = 2,
  kWaterTrack = 3,
  kUnrecognized = 4,
};

// field_present is DvlMeasurement.mode presence, not the enum value.
inline constexpr DvlModeKind ClassifyDvlMode(bool field_present, int mode) {
  if (!field_present) {
    return DvlModeKind::kAbsent;
  }
  switch (mode) {
    case 0:
      return DvlModeKind::kUnspecified;
    case 1:
      return DvlModeKind::kBottomTrack;
    case 2:
      return DvlModeKind::kWaterTrack;
    default:
      return DvlModeKind::kUnrecognized;
  }
}

enum class DvlError {
  kNone = 0,
  kMissingHealth = 1,
  kMissingFrame = 2,
  kWrongFrame = 3,
  kTimeReversal = 4,
  kNonFinite = 5,
  kQuality = 6,
  kCovariance = 7,
  kSourceId = 8,
  kMode = 9,
  kVelocity = 10,
  kLock = 11,
  kAltitude = 12,
  kAngularCovariance = 13,
};

// Row-major angular variance slots. Linear x, y, z occupy the leading 3x3.
// DVL measures linear velocity only, so these three diagonals are exactly 0
// when covariance is present. Off-diagonal angular entries stay on #17.
inline constexpr int kDvlAngularVarianceSlots[3] = {
    vehicle::CovarianceIndex(3, 3),
    vehicle::CovarianceIndex(4, 4),
    vehicle::CovarianceIndex(5, 5),
};

struct DvlAssessment {
  DvlError error = DvlError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  DvlModeKind mode = DvlModeKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID, a bottom or water track mode, and no
  // structural defect. An empty message is not accepted and is not an error.
  bool accepted = false;
};

struct DvlMeasurementView {
  // Span inside `health.covariance` is valid only while caller storage lives.
  MeasurementHealthView health;
  bool mode_present = false;
  int mode = 0;
  bool velocity_x_present = false;
  double velocity_x_m_s = 0;
  bool velocity_y_present = false;
  double velocity_y_m_s = 0;
  bool velocity_z_present = false;
  double velocity_z_m_s = 0;
  bool bottom_lock_present = false;
  bool bottom_lock = false;
  bool altitude_present = false;
  double altitude_m = 0;
};

inline bool DvlEngaged(const DvlMeasurementView& sample) {
  return MeasurementEngaged(sample.health) || sample.mode_present ||
         sample.velocity_x_present || sample.velocity_y_present ||
         sample.velocity_z_present || sample.bottom_lock_present ||
         sample.altitude_present;
}

inline bool DvlTrackMode(DvlModeKind mode) {
  return mode == DvlModeKind::kBottomTrack || mode == DvlModeKind::kWaterTrack;
}

inline bool AngularVariancesAreZero(std::span<const double> values) {
  if (static_cast<int>(values.size()) <= kDvlAngularVarianceSlots[2]) {
    return false;
  }
  for (int index : kDvlAngularVarianceSlots) {
    if (values[index] != 0.0) {
      return false;
    }
  }
  return true;
}

inline DvlError FromMeasurementError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return DvlError::kNone;
    case MeasurementError::kMissingFrame:
      return DvlError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return DvlError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return DvlError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return DvlError::kNonFinite;
    case MeasurementError::kQuality:
      return DvlError::kQuality;
    case MeasurementError::kCovariance:
      return DvlError::kCovariance;
    case MeasurementError::kSourceId:
      return DvlError::kSourceId;
  }
  return DvlError::kNone;
}

// Check order: missing health, then #75 health order, then angular variance
// slots, mode, velocity presence, velocity finiteness, bottom-lock
// inconsistency, altitude finiteness, altitude sign. The first defect wins.
// Absent optional fields are not defects. Health state is not rewritten.
inline DvlAssessment AssessDvl(const DvlMeasurementView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  const DvlModeKind mode = ClassifyDvlMode(sample.mode_present, sample.mode);
  if (!DvlEngaged(sample)) {
    return DvlAssessment{DvlError::kNone, MeasurementStateKind::kAbsent,
                         DvlModeKind::kAbsent, header_validity, false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  DvlError error = DvlError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = DvlError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = FromMeasurementError(health.error);
    if (error == DvlError::kNone && sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AngularVariancesAreZero(sample.health.covariance)) {
      error = DvlError::kAngularCovariance;
    }
  }
  if (error == DvlError::kNone && !DvlTrackMode(mode)) {
    error = DvlError::kMode;
  }
  if (error == DvlError::kNone) {
    if (!sample.velocity_x_present || !sample.velocity_y_present ||
        !sample.velocity_z_present) {
      error = DvlError::kVelocity;
    } else if (!embodiment::IsFinite(sample.velocity_x_m_s) ||
               !embodiment::IsFinite(sample.velocity_y_m_s) ||
               !embodiment::IsFinite(sample.velocity_z_m_s)) {
      error = DvlError::kNonFinite;
    }
  }
  if (error == DvlError::kNone && mode == DvlModeKind::kBottomTrack &&
      sample.bottom_lock_present && !sample.bottom_lock &&
      state == MeasurementStateKind::kValid) {
    error = DvlError::kLock;
  }
  if (error == DvlError::kNone && sample.altitude_present) {
    if (!embodiment::IsFinite(sample.altitude_m)) {
      error = DvlError::kNonFinite;
    } else if (sample.altitude_m < 0.0) {
      error = DvlError::kAltitude;
    }
  }
  const bool accepted = error == DvlError::kNone &&
                        state == MeasurementStateKind::kValid &&
                        DvlTrackMode(mode);
  return DvlAssessment{error, state, mode, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_DVL_POLICY_H_
