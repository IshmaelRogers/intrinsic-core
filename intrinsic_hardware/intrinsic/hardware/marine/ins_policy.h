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

#ifndef INTRINSIC_HARDWARE_MARINE_INS_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_INS_POLICY_H_

#include <cmath>
#include <span>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for InsSolution. These functions do not parse protobuf,
// do not convert ENU and NED, do not renormalize quaternions, do not read
// ImuMeasurement, do not propagate a filter, and do not command hardware.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.
//
// `source` is the measurement-local enum (SOURCE_UNSPECIFIED=0,
// SOURCE_VENDOR_INS=1, SOURCE_EXTERNAL_NAV=2). Unset source
// (`source_present == false`) is absent. A present 0 is SOURCE_UNSPECIFIED.
// Numbers other than 0, 1, and 2 stay unrecognized: they are not accepted
// and they are not rewritten.
//
// Unit norm uses the same absolute tolerance as ImuMeasurement: 1e-6.
// Attitude variance slots are placeholders. They are not a quaternion
// covariance. This header does not include the IMU policy.

inline constexpr double kInsUnitQuaternionTolerance = 1e-6;

inline constexpr int kPositionXVarianceSlot = vehicle::CovarianceIndex(0, 0);
inline constexpr int kPositionYVarianceSlot = vehicle::CovarianceIndex(1, 1);
inline constexpr int kPositionZVarianceSlot = vehicle::CovarianceIndex(2, 2);
inline constexpr int kAttitudeXVarianceSlot = vehicle::CovarianceIndex(3, 3);
inline constexpr int kAttitudeYVarianceSlot = vehicle::CovarianceIndex(4, 4);
inline constexpr int kAttitudeZVarianceSlot = vehicle::CovarianceIndex(5, 5);

inline constexpr int kInsVarianceSlots[] = {
    kPositionXVarianceSlot, kPositionYVarianceSlot, kPositionZVarianceSlot,
    kAttitudeXVarianceSlot, kAttitudeYVarianceSlot, kAttitudeZVarianceSlot,
};

enum class InsSourceKind {
  kAbsent = 0,
  kUnspecified = 1,
  kVendorIns = 2,
  kExternalNav = 3,
  kUnrecognized = 4,
};

// field_present is InsSolution.source presence, not the enum value.
inline constexpr InsSourceKind ClassifyInsSource(bool field_present,
                                                 int source) {
  if (!field_present) {
    return InsSourceKind::kAbsent;
  }
  switch (source) {
    case 0:
      return InsSourceKind::kUnspecified;
    case 1:
      return InsSourceKind::kVendorIns;
    case 2:
      return InsSourceKind::kExternalNav;
    default:
      return InsSourceKind::kUnrecognized;
  }
}

enum class InsError {
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
  kPosition = 10,
  kOrientation = 11,
  kLinearVelocity = 12,
  kAngularVelocity = 13,
  kSource = 14,
};

struct InsAssessment {
  InsError error = InsError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  InsSourceKind source = InsSourceKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID and no structural defect. An empty
  // message is not accepted and is not an error. Absent source is allowed.
  bool accepted = false;
};

struct InsSolutionView {
  // Span inside `health.covariance` is valid only while caller storage lives.
  MeasurementHealthView health;
  bool position_x_present = false;
  double position_x_m = 0;
  bool position_y_present = false;
  double position_y_m = 0;
  bool position_z_present = false;
  double position_z_m = 0;
  bool orientation_present = false;
  double orientation_x = 0;
  double orientation_y = 0;
  double orientation_z = 0;
  double orientation_w = 0;
  bool linear_velocity_x_present = false;
  double linear_velocity_x_m_s = 0;
  bool linear_velocity_y_present = false;
  double linear_velocity_y_m_s = 0;
  bool linear_velocity_z_present = false;
  double linear_velocity_z_m_s = 0;
  bool angular_velocity_x_present = false;
  double angular_velocity_x_rad_s = 0;
  bool angular_velocity_y_present = false;
  double angular_velocity_y_rad_s = 0;
  bool angular_velocity_z_present = false;
  double angular_velocity_z_rad_s = 0;
  bool source_present = false;
  int source = 0;
};

inline bool InsEngaged(const InsSolutionView& sample) {
  return MeasurementEngaged(sample.health) || sample.position_x_present ||
         sample.position_y_present || sample.position_z_present ||
         sample.orientation_present || sample.linear_velocity_x_present ||
         sample.linear_velocity_y_present || sample.linear_velocity_z_present ||
         sample.angular_velocity_x_present ||
         sample.angular_velocity_y_present ||
         sample.angular_velocity_z_present || sample.source_present;
}

inline bool AllowedInsCovarianceOnly(std::span<const double> values) {
  if (static_cast<int>(values.size()) != vehicle::kCovarianceValues) {
    return false;
  }
  for (int index = 0; index < vehicle::kCovarianceValues; ++index) {
    bool allowed = false;
    for (int slot : kInsVarianceSlots) {
      if (index == slot) {
        allowed = true;
        break;
      }
    }
    if (!allowed && values[index] != 0.0) {
      return false;
    }
  }
  return true;
}

// Finite components only. |‖q‖ − 1| <= 1e-6. Does not renormalize.
inline bool InsUnitQuaternion(double x, double y, double z, double w) {
  const embodiment::Quaternion value{x, y, z, w};
  if (!embodiment::IsFinite(value)) {
    return false;
  }
  return std::abs(embodiment::QuaternionNorm(value) - 1.0) <=
         kInsUnitQuaternionTolerance;
}

inline bool KnownInsSource(InsSourceKind source) {
  return source == InsSourceKind::kVendorIns ||
         source == InsSourceKind::kExternalNav;
}

inline InsError ToInsError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return InsError::kNone;
    case MeasurementError::kMissingFrame:
      return InsError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return InsError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return InsError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return InsError::kNonFinite;
    case MeasurementError::kQuality:
      return InsError::kQuality;
    case MeasurementError::kCovariance:
      return InsError::kCovariance;
    case MeasurementError::kSourceId:
      return InsError::kSourceId;
  }
  return InsError::kNone;
}

inline int PresentCount(bool x, bool y, bool z) {
  return static_cast<int>(x) + static_cast<int>(y) + static_cast<int>(z);
}

// Check order: missing health, then #75 health order, then covariance
// slots, position presence, position finiteness, orientation presence,
// orientation finiteness, orientation unit norm, linear-velocity partial
// triples, linear-velocity finiteness, angular-velocity partial triples,
// angular-velocity finiteness, source kind. The first defect wins. Absent
// twist and absent source are not defects. Health state is not rewritten.
inline InsAssessment AssessIns(const InsSolutionView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  const InsSourceKind source =
      ClassifyInsSource(sample.source_present, sample.source);
  if (!InsEngaged(sample)) {
    return InsAssessment{InsError::kNone, MeasurementStateKind::kAbsent,
                         InsSourceKind::kAbsent, header_validity, false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  InsError error = InsError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = InsError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = ToInsError(health.error);
    if (error == InsError::kNone && sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AllowedInsCovarianceOnly(sample.health.covariance)) {
      error = InsError::kCovarianceSlots;
    }
  }
  if (error == InsError::kNone &&
      !(sample.position_x_present && sample.position_y_present &&
        sample.position_z_present)) {
    error = InsError::kPosition;
  }
  if (error == InsError::kNone &&
      (!embodiment::IsFinite(sample.position_x_m) ||
       !embodiment::IsFinite(sample.position_y_m) ||
       !embodiment::IsFinite(sample.position_z_m))) {
    error = InsError::kNonFinite;
  }
  if (error == InsError::kNone && !sample.orientation_present) {
    error = InsError::kOrientation;
  }
  if (error == InsError::kNone) {
    const embodiment::Quaternion orientation{
        sample.orientation_x, sample.orientation_y, sample.orientation_z,
        sample.orientation_w};
    if (!embodiment::IsFinite(orientation)) {
      error = InsError::kNonFinite;
    } else if (!InsUnitQuaternion(sample.orientation_x, sample.orientation_y,
                                  sample.orientation_z, sample.orientation_w)) {
      error = InsError::kOrientation;
    }
  }
  const int linear = PresentCount(sample.linear_velocity_x_present,
                                  sample.linear_velocity_y_present,
                                  sample.linear_velocity_z_present);
  if (error == InsError::kNone && linear != 0 && linear != 3) {
    error = InsError::kLinearVelocity;
  }
  if (error == InsError::kNone && linear == 3 &&
      (!embodiment::IsFinite(sample.linear_velocity_x_m_s) ||
       !embodiment::IsFinite(sample.linear_velocity_y_m_s) ||
       !embodiment::IsFinite(sample.linear_velocity_z_m_s))) {
    error = InsError::kNonFinite;
  }
  const int angular = PresentCount(sample.angular_velocity_x_present,
                                   sample.angular_velocity_y_present,
                                   sample.angular_velocity_z_present);
  if (error == InsError::kNone && angular != 0 && angular != 3) {
    error = InsError::kAngularVelocity;
  }
  if (error == InsError::kNone && angular == 3 &&
      (!embodiment::IsFinite(sample.angular_velocity_x_rad_s) ||
       !embodiment::IsFinite(sample.angular_velocity_y_rad_s) ||
       !embodiment::IsFinite(sample.angular_velocity_z_rad_s))) {
    error = InsError::kNonFinite;
  }
  if (error == InsError::kNone && sample.source_present &&
      !KnownInsSource(source)) {
    error = InsError::kSource;
  }
  const bool accepted =
      error == InsError::kNone && state == MeasurementStateKind::kValid;
  return InsAssessment{error, state, source, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_INS_POLICY_H_
