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

#ifndef INTRINSIC_HARDWARE_MARINE_IMU_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_IMU_POLICY_H_

#include <cmath>
#include <span>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for ImuMeasurement. These functions do not parse
// protobuf, do not convert ENU and NED, do not renormalize quaternions, do
// not propagate a filter, and do not command hardware.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.
//
// An engaged sample requires all three angular-velocity components and all
// three linear-acceleration components. Orientation is optional. A present
// orientation is a Hamilton quaternion (x, y, z, w). Unit norm is an
// absolute tolerance of 1e-6 on |‖q‖ − 1|.

inline constexpr double kImuUnitQuaternionTolerance = 1e-6;

// Row-major Matrix6 diagonals. Off-diagonal entries stay exactly 0 when
// covariance is present. Orientation uncertainty is not a slot.
inline constexpr int kAngularVelocityXVarianceSlot =
    vehicle::CovarianceIndex(0, 0);
inline constexpr int kAngularVelocityYVarianceSlot =
    vehicle::CovarianceIndex(1, 1);
inline constexpr int kAngularVelocityZVarianceSlot =
    vehicle::CovarianceIndex(2, 2);
inline constexpr int kLinearAccelerationXVarianceSlot =
    vehicle::CovarianceIndex(3, 3);
inline constexpr int kLinearAccelerationYVarianceSlot =
    vehicle::CovarianceIndex(4, 4);
inline constexpr int kLinearAccelerationZVarianceSlot =
    vehicle::CovarianceIndex(5, 5);

inline constexpr int kImuVarianceSlots[] = {
    kAngularVelocityXVarianceSlot,    kAngularVelocityYVarianceSlot,
    kAngularVelocityZVarianceSlot,    kLinearAccelerationXVarianceSlot,
    kLinearAccelerationYVarianceSlot, kLinearAccelerationZVarianceSlot,
};

enum class ImuError {
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
  kAngularVelocity = 10,
  kLinearAcceleration = 11,
  kOrientation = 12,
};

struct ImuAssessment {
  ImuError error = ImuError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID and no structural defect. An empty
  // message is not accepted and is not an error.
  bool accepted = false;
};

struct ImuMeasurementView {
  // Span inside `health.covariance` is valid only while caller storage lives.
  MeasurementHealthView health;
  bool angular_velocity_x_present = false;
  double angular_velocity_x_rad_s = 0;
  bool angular_velocity_y_present = false;
  double angular_velocity_y_rad_s = 0;
  bool angular_velocity_z_present = false;
  double angular_velocity_z_rad_s = 0;
  bool linear_acceleration_x_present = false;
  double linear_acceleration_x_m_s2 = 0;
  bool linear_acceleration_y_present = false;
  double linear_acceleration_y_m_s2 = 0;
  bool linear_acceleration_z_present = false;
  double linear_acceleration_z_m_s2 = 0;
  bool orientation_present = false;
  double orientation_x = 0;
  double orientation_y = 0;
  double orientation_z = 0;
  double orientation_w = 0;
};

inline bool ImuEngaged(const ImuMeasurementView& sample) {
  return MeasurementEngaged(sample.health) ||
         sample.angular_velocity_x_present ||
         sample.angular_velocity_y_present ||
         sample.angular_velocity_z_present ||
         sample.linear_acceleration_x_present ||
         sample.linear_acceleration_y_present ||
         sample.linear_acceleration_z_present || sample.orientation_present;
}

inline bool AllowedImuCovarianceOnly(std::span<const double> values) {
  if (static_cast<int>(values.size()) != vehicle::kCovarianceValues) {
    return false;
  }
  for (int index = 0; index < vehicle::kCovarianceValues; ++index) {
    bool allowed = false;
    for (int slot : kImuVarianceSlots) {
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
inline bool ImuUnitQuaternion(double x, double y, double z, double w) {
  const embodiment::Quaternion value{x, y, z, w};
  if (!embodiment::IsFinite(value)) {
    return false;
  }
  return std::abs(embodiment::QuaternionNorm(value) - 1.0) <=
         kImuUnitQuaternionTolerance;
}

inline ImuError ToImuError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return ImuError::kNone;
    case MeasurementError::kMissingFrame:
      return ImuError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return ImuError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return ImuError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return ImuError::kNonFinite;
    case MeasurementError::kQuality:
      return ImuError::kQuality;
    case MeasurementError::kCovariance:
      return ImuError::kCovariance;
    case MeasurementError::kSourceId:
      return ImuError::kSourceId;
  }
  return ImuError::kNone;
}

// Check order: missing health, then #75 health order, then covariance
// slots, angular-velocity presence, angular-velocity finiteness,
// linear-acceleration presence, linear-acceleration finiteness, orientation
// finiteness, orientation unit norm. The first defect wins. Absent optional
// orientation is not a defect. Health state is not rewritten.
inline ImuAssessment AssessImu(const ImuMeasurementView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  if (!ImuEngaged(sample)) {
    return ImuAssessment{ImuError::kNone, MeasurementStateKind::kAbsent,
                         header_validity, false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  ImuError error = ImuError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = ImuError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = ToImuError(health.error);
    if (error == ImuError::kNone && sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AllowedImuCovarianceOnly(sample.health.covariance)) {
      error = ImuError::kCovarianceSlots;
    }
  }
  if (error == ImuError::kNone && !(sample.angular_velocity_x_present &&
                                    sample.angular_velocity_y_present &&
                                    sample.angular_velocity_z_present)) {
    error = ImuError::kAngularVelocity;
  }
  if (error == ImuError::kNone &&
      (!embodiment::IsFinite(sample.angular_velocity_x_rad_s) ||
       !embodiment::IsFinite(sample.angular_velocity_y_rad_s) ||
       !embodiment::IsFinite(sample.angular_velocity_z_rad_s))) {
    error = ImuError::kNonFinite;
  }
  if (error == ImuError::kNone && !(sample.linear_acceleration_x_present &&
                                    sample.linear_acceleration_y_present &&
                                    sample.linear_acceleration_z_present)) {
    error = ImuError::kLinearAcceleration;
  }
  if (error == ImuError::kNone &&
      (!embodiment::IsFinite(sample.linear_acceleration_x_m_s2) ||
       !embodiment::IsFinite(sample.linear_acceleration_y_m_s2) ||
       !embodiment::IsFinite(sample.linear_acceleration_z_m_s2))) {
    error = ImuError::kNonFinite;
  }
  if (error == ImuError::kNone && sample.orientation_present) {
    const embodiment::Quaternion orientation{
        sample.orientation_x, sample.orientation_y, sample.orientation_z,
        sample.orientation_w};
    if (!embodiment::IsFinite(orientation)) {
      error = ImuError::kNonFinite;
    } else if (!ImuUnitQuaternion(sample.orientation_x, sample.orientation_y,
                                  sample.orientation_z, sample.orientation_w)) {
      error = ImuError::kOrientation;
    }
  }
  const bool accepted =
      error == ImuError::kNone && state == MeasurementStateKind::kValid;
  return ImuAssessment{error, state, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_IMU_POLICY_H_
