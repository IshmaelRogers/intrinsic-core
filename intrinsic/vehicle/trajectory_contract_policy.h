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

#ifndef INTRINSIC_VEHICLE_TRAJECTORY_CONTRACT_POLICY_H_
#define INTRINSIC_VEHICLE_TRAJECTORY_CONTRACT_POLICY_H_

#include <cstdint>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::vehicle {

// Policy: intrinsic_apis/intrinsic/vehicle/proto/README.md.
//
// Plain-value checks for VehicleTrajectory. These functions do not parse
// protobuf, do not interpolate, do not convert ENU and NED, and do not
// call World, ICON, or a HAL.

enum class TrajectoryContractError {
  kNone = 0,
  kTrajectoryId = 1,
  kEmptySamples = 2,
  kNonMonotonicTime = 3,
  kMissingFrame = 4,
  kNonFinite = 5,
  kQuaternion = 6,
  kTolerance = 7,
  kCost = 8,
  kRisk = 9,
  kUncertainty = 10,
  kProvenance = 11,
  kSampleTime = 12,
};

struct TrajectoryContractAssessment {
  TrajectoryContractError error = TrajectoryContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only when error == kNone and the header validity is STATE_VALID.
  // An empty message is not accepted and is not an error.
  bool accepted = false;
};

// One timed sample. Pose is always supplied by the caller. An absent wire
// pose is the zero quaternion, which fails the normalized-quaternion rule.
struct TrajectorySampleView {
  bool time_present = false;
  int64_t seconds = 0;
  int32_t nanos = 0;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
  bool twist_present = false;
  BodyVector twist;
  bool acceleration_present = false;
  BodyVector acceleration;
};

struct VehicleTrajectoryView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  std::string_view trajectory_id;
  // Valid only while the caller's storage is alive.
  std::span<const TrajectorySampleView> samples;
  bool tolerances_present = false;
  double position_tolerance_m = 0;
  double orientation_tolerance_rad = 0;
  double linear_velocity_tolerance_m_s = 0;
  double angular_velocity_tolerance_rad_s = 0;
  bool cost_present = false;
  double cost = 0;
  bool risk_present = false;
  double risk = 0;
  // Valid only while the caller's storage is alive.
  bool uncertainty_present = false;
  std::span<const double> uncertainty;
  bool provenance_present = false;
  std::string_view model_id;
  // True when the metadata map has any entry. Values are not inspected.
  bool metadata_present = false;
};

inline bool TrajectoryEngaged(const VehicleTrajectoryView& trajectory) {
  return trajectory.header_present || !trajectory.trajectory_id.empty() ||
         !trajectory.samples.empty() || trajectory.tolerances_present ||
         trajectory.cost_present || trajectory.risk_present ||
         trajectory.uncertainty_present || trajectory.provenance_present ||
         trajectory.metadata_present;
}

inline bool SampleTimeOk(const TrajectorySampleView& sample) {
  return sample.time_present && embodiment::NanosInRange(sample.nanos);
}

// Strictly earlier, compared as (seconds, nanos). Equal times are not
// earlier.
inline bool SampleTimeBefore(const TrajectorySampleView& earlier,
                             const TrajectorySampleView& later) {
  if (earlier.seconds != later.seconds) {
    return earlier.seconds < later.seconds;
  }
  return earlier.nanos < later.nanos;
}

inline bool ToleranceOk(double value) {
  return embodiment::IsFinite(value) && value >= 0.0;
}

inline bool CostOk(double cost) { return embodiment::IsFinite(cost); }

inline bool RiskOk(double risk) {
  return embodiment::IsFinite(risk) && risk >= 0.0 && risk <= 1.0;
}

inline TrajectoryContractAssessment MakeTrajectoryAssessment(
    TrajectoryContractError error, embodiment::ValidityKind validity) {
  const bool accepted = error == TrajectoryContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  return TrajectoryContractAssessment{error, validity, accepted};
}

// First defect wins. Order: trajectory id, sample count, each sample time
// in index order, strict increase, frame id, then each sample's pose,
// twist, and acceleration, then tolerances, cost, risk, uncertainty, and
// provenance. Metadata is never a defect. An empty view is not engaged.
inline TrajectoryContractAssessment AssessVehicleTrajectory(
    const VehicleTrajectoryView& trajectory) {
  if (!TrajectoryEngaged(trajectory)) {
    return TrajectoryContractAssessment{TrajectoryContractError::kNone,
                                        embodiment::ValidityKind::kAbsent,
                                        false};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      trajectory.validity_present, trajectory.validity_state);
  TrajectoryContractError error = TrajectoryContractError::kNone;
  if (trajectory.trajectory_id.empty()) {
    error = TrajectoryContractError::kTrajectoryId;
  } else if (trajectory.samples.empty()) {
    error = TrajectoryContractError::kEmptySamples;
  } else {
    for (const TrajectorySampleView& sample : trajectory.samples) {
      if (!SampleTimeOk(sample)) {
        error = TrajectoryContractError::kSampleTime;
        break;
      }
    }
    if (error == TrajectoryContractError::kNone) {
      for (size_t index = 1; index < trajectory.samples.size(); ++index) {
        if (!SampleTimeBefore(trajectory.samples[index - 1],
                              trajectory.samples[index])) {
          error = TrajectoryContractError::kNonMonotonicTime;
          break;
        }
      }
    }
    if (error == TrajectoryContractError::kNone &&
        trajectory.frame_id.empty()) {
      error = TrajectoryContractError::kMissingFrame;
    }
    if (error == TrajectoryContractError::kNone) {
      for (const TrajectorySampleView& sample : trajectory.samples) {
        if (!embodiment::IsFinite(sample.position)) {
          error = TrajectoryContractError::kNonFinite;
        } else if (!embodiment::IsFinite(sample.orientation)) {
          error = TrajectoryContractError::kNonFinite;
        } else if (!embodiment::IsNormalized(sample.orientation)) {
          error = TrajectoryContractError::kQuaternion;
        } else if (sample.twist_present && !IsFinite(sample.twist)) {
          error = TrajectoryContractError::kNonFinite;
        } else if (sample.acceleration_present &&
                   !IsFinite(sample.acceleration)) {
          error = TrajectoryContractError::kNonFinite;
        }
        if (error != TrajectoryContractError::kNone) {
          break;
        }
      }
    }
  }
  if (error == TrajectoryContractError::kNone &&
      trajectory.tolerances_present &&
      !(ToleranceOk(trajectory.position_tolerance_m) &&
        ToleranceOk(trajectory.orientation_tolerance_rad) &&
        ToleranceOk(trajectory.linear_velocity_tolerance_m_s) &&
        ToleranceOk(trajectory.angular_velocity_tolerance_rad_s))) {
    error = TrajectoryContractError::kTolerance;
  }
  if (error == TrajectoryContractError::kNone && trajectory.cost_present &&
      !CostOk(trajectory.cost)) {
    error = TrajectoryContractError::kCost;
  }
  if (error == TrajectoryContractError::kNone && trajectory.risk_present &&
      !RiskOk(trajectory.risk)) {
    error = TrajectoryContractError::kRisk;
  }
  if (error == TrajectoryContractError::kNone &&
      trajectory.uncertainty_present) {
    const CovarianceError covariance =
        AssessCovariance(true, trajectory.uncertainty);
    if (covariance != CovarianceError::kNone) {
      error = TrajectoryContractError::kUncertainty;
    }
  }
  if (error == TrajectoryContractError::kNone &&
      trajectory.provenance_present && trajectory.model_id.empty()) {
    error = TrajectoryContractError::kProvenance;
  }
  return MakeTrajectoryAssessment(error, validity);
}

}  // namespace intrinsic::vehicle

#endif  // INTRINSIC_VEHICLE_TRAJECTORY_CONTRACT_POLICY_H_
