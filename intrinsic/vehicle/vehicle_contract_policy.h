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

#ifndef INTRINSIC_VEHICLE_VEHICLE_CONTRACT_POLICY_H_
#define INTRINSIC_VEHICLE_VEHICLE_CONTRACT_POLICY_H_

#include <cstdint>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::vehicle {

// Policy: intrinsic_apis/intrinsic/vehicle/proto/README.md.
//
// Plain-value checks for the vehicle protos. These functions do not parse
// protobuf, do not convert ENU and NED, and do not command actuators.

inline constexpr std::string_view kBodyFrameId = "body";

inline constexpr int kSpatialDof = 6;
inline constexpr int kCovarianceValues = kSpatialDof * kSpatialDof;
inline constexpr double kCovarianceSymmetryTolerance = 1e-9;

// Row-major index. row and col are in [0, 5].
inline constexpr int CovarianceIndex(int row, int col) {
  return row * kSpatialDof + col;
}

// Linear components first, then angular, matching BodyTwist field order.
// Units come from the proto field that supplied the vector.
struct BodyVector {
  double linear_x = 0;
  double linear_y = 0;
  double linear_z = 0;
  double angular_x = 0;
  double angular_y = 0;
  double angular_z = 0;
};

inline bool IsFinite(BodyVector value) {
  return embodiment::IsFinite(value.linear_x) &&
         embodiment::IsFinite(value.linear_y) &&
         embodiment::IsFinite(value.linear_z) &&
         embodiment::IsFinite(value.angular_x) &&
         embodiment::IsFinite(value.angular_y) &&
         embodiment::IsFinite(value.angular_z);
}

enum class CovarianceError {
  kNone = 0,
  kAbsent = 1,
  kWrongLength = 2,
  kNonFinite = 3,
  kAsymmetric = 4,
};

// present false is unknown covariance. A present span is specified data,
// including 36 zeros. Unknown is never the zero matrix.
inline CovarianceError AssessCovariance(bool present,
                                        std::span<const double> values) {
  if (!present) {
    return CovarianceError::kAbsent;
  }
  if (static_cast<int>(values.size()) != kCovarianceValues) {
    return CovarianceError::kWrongLength;
  }
  for (double value : values) {
    if (!embodiment::IsFinite(value)) {
      return CovarianceError::kNonFinite;
    }
  }
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = row + 1; col < kSpatialDof; ++col) {
      const double upper = values[CovarianceIndex(row, col)];
      const double lower = values[CovarianceIndex(col, row)];
      if ((upper > lower ? upper - lower : lower - upper) >
          kCovarianceSymmetryTolerance) {
        return CovarianceError::kAsymmetric;
      }
    }
  }
  return CovarianceError::kNone;
}

inline constexpr bool CovarianceIsUnknown(CovarianceError error) {
  return error == CovarianceError::kAbsent;
}

inline bool IsAllZeroCovariance(std::span<const double> values) {
  if (static_cast<int>(values.size()) != kCovarianceValues) {
    return false;
  }
  for (double value : values) {
    if (value != 0.0) {
      return false;
    }
  }
  return true;
}

enum class NavigationModeKind {
  kUnspecified = 0,
  kInitializing = 1,
  kDeadReckoning = 2,
  kAided = 3,
  kFaulted = 4,
  kUnknown = 5,
};

// Wire values other than 0..4 are kUnknown. kUnknown is not faulted.
inline constexpr NavigationModeKind ClassifyNavigationMode(int mode) {
  switch (mode) {
    case 0:
      return NavigationModeKind::kUnspecified;
    case 1:
      return NavigationModeKind::kInitializing;
    case 2:
      return NavigationModeKind::kDeadReckoning;
    case 3:
      return NavigationModeKind::kAided;
    case 4:
      return NavigationModeKind::kFaulted;
    default:
      return NavigationModeKind::kUnknown;
  }
}

enum class ContractError {
  kNone = 0,
  kNonFinite = 1,
  kQuaternion = 2,
  kMissingFrame = 3,
  kBodyFrame = 4,
  kPoseCovariance = 5,
  kTwistCovariance = 6,
  kConfidence = 7,
  kHorizon = 8,
  kObjective = 9,
  kTrajectoryId = 10,
  kProvenance = 11,
  kSourceId = 12,
};

struct ContractAssessment {
  ContractError error = ContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only for STATE_VALID with no structural defect. An empty message
  // is not accepted and is not an error.
  bool accepted = false;
};

struct SourceView {
  std::string_view source_id;
  bool validity_present = false;
  int validity_state = 0;
};

struct VehicleStateView {
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  bool pose_present = false;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
  bool twist_present = false;
  BodyVector twist;
  bool acceleration_present = false;
  BodyVector acceleration;
  // Spans are valid only while the caller's storage is alive.
  bool pose_covariance_present = false;
  std::span<const double> pose_covariance;
  bool twist_covariance_present = false;
  std::span<const double> twist_covariance;
  std::span<const SourceView> sources;
};

inline bool ConfidenceInRange(double confidence) {
  return embodiment::IsFinite(confidence) && confidence >= 0.0 &&
         confidence <= 1.0;
}

inline bool DurationNonNegative(int64_t seconds, int32_t nanos) {
  return seconds >= 0 && nanos >= 0 && nanos < embodiment::kNanosPerSecond;
}

inline ContractAssessment MakeAssessment(ContractError error,
                                         embodiment::ValidityKind validity) {
  const bool accepted = error == ContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  return ContractAssessment{error, validity, accepted};
}

// Check order: pose, twist, acceleration, pose covariance, twist
// covariance, source ids. The first defect wins. Absent optional fields
// are not defects. Mode is not a structural defect; classify it separately.
inline ContractAssessment AssessVehicleState(const VehicleStateView& state) {
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      state.validity_present, state.validity_state);
  ContractError error = ContractError::kNone;
  if (state.pose_present) {
    if (!embodiment::IsFinite(state.position)) {
      error = ContractError::kNonFinite;
    } else if (!embodiment::IsFinite(state.orientation)) {
      error = ContractError::kNonFinite;
    } else if (!embodiment::IsNormalized(state.orientation)) {
      error = ContractError::kQuaternion;
    } else if (state.frame_id.empty()) {
      error = ContractError::kMissingFrame;
    }
  }
  if (error == ContractError::kNone && state.twist_present &&
      !IsFinite(state.twist)) {
    error = ContractError::kNonFinite;
  }
  if (error == ContractError::kNone && state.acceleration_present &&
      !IsFinite(state.acceleration)) {
    error = ContractError::kNonFinite;
  }
  if (error == ContractError::kNone) {
    const CovarianceError pose =
        AssessCovariance(state.pose_covariance_present, state.pose_covariance);
    if (pose != CovarianceError::kNone && pose != CovarianceError::kAbsent) {
      error = ContractError::kPoseCovariance;
    }
  }
  if (error == ContractError::kNone) {
    const CovarianceError twist = AssessCovariance(
        state.twist_covariance_present, state.twist_covariance);
    if (twist != CovarianceError::kNone && twist != CovarianceError::kAbsent) {
      error = ContractError::kTwistCovariance;
    }
  }
  if (error == ContractError::kNone) {
    for (const SourceView& source : state.sources) {
      if (source.source_id.empty()) {
        error = ContractError::kSourceId;
        break;
      }
    }
  }
  return MakeAssessment(error, validity);
}

enum class ObjectiveKind {
  kAbsent = 0,
  kPose = 1,
  kTwist = 2,
  kTrajectory = 3,
};

struct DesiredMotionView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  ObjectiveKind objective = ObjectiveKind::kAbsent;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
  BodyVector twist;
  std::string_view trajectory_id;
  bool confidence_present = false;
  double confidence = 0;
  bool horizon_present = false;
  int64_t horizon_seconds = 0;
  int32_t horizon_nanos = 0;
  bool provenance_present = false;
  std::string_view model_id;
};

struct BodyWrenchView {
  // False when the protobuf message is empty. A zero wrench with frame id
  // "body" is engaged.
  bool engaged = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  // linear_* is force in newtons. angular_* is torque in newton-meters.
  BodyVector force_torque;
};

inline bool MotionEngaged(const DesiredMotionView& motion) {
  return motion.header_present || motion.objective != ObjectiveKind::kAbsent ||
         motion.confidence_present || motion.horizon_present ||
         motion.provenance_present;
}

// Check order: objective presence, pose or twist or trajectory, confidence,
// horizon, provenance. An empty view is not an intent.
inline ContractAssessment AssessDesiredMotion(const DesiredMotionView& motion) {
  if (!MotionEngaged(motion)) {
    return ContractAssessment{ContractError::kNone,
                              embodiment::ValidityKind::kAbsent, false};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      motion.validity_present, motion.validity_state);
  ContractError error = ContractError::kNone;
  if (motion.objective == ObjectiveKind::kAbsent) {
    error = ContractError::kObjective;
  } else if (motion.objective == ObjectiveKind::kPose) {
    if (!embodiment::IsFinite(motion.position)) {
      error = ContractError::kNonFinite;
    } else if (!embodiment::IsFinite(motion.orientation)) {
      error = ContractError::kNonFinite;
    } else if (!embodiment::IsNormalized(motion.orientation)) {
      error = ContractError::kQuaternion;
    } else if (motion.frame_id.empty()) {
      error = ContractError::kMissingFrame;
    }
  } else if (motion.objective == ObjectiveKind::kTwist) {
    if (!IsFinite(motion.twist)) {
      error = ContractError::kNonFinite;
    } else if (motion.frame_id != kBodyFrameId) {
      error = ContractError::kBodyFrame;
    }
  } else if (motion.trajectory_id.empty()) {
    error = ContractError::kTrajectoryId;
  }
  if (error == ContractError::kNone && motion.confidence_present &&
      !ConfidenceInRange(motion.confidence)) {
    error = ContractError::kConfidence;
  }
  if (error == ContractError::kNone && motion.horizon_present &&
      !DurationNonNegative(motion.horizon_seconds, motion.horizon_nanos)) {
    error = ContractError::kHorizon;
  }
  if (error == ContractError::kNone && motion.provenance_present &&
      motion.model_id.empty()) {
    error = ContractError::kProvenance;
  }
  return MakeAssessment(error, validity);
}

// An unengaged wrench is not a command. Frame id must be "body" when
// engaged. Components must be finite. Zero components are a neutral wrench.
inline ContractAssessment AssessBodyWrench(const BodyWrenchView& wrench) {
  if (!wrench.engaged) {
    return ContractAssessment{ContractError::kNone,
                              embodiment::ValidityKind::kAbsent, false};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      wrench.validity_present, wrench.validity_state);
  ContractError error = ContractError::kNone;
  if (!IsFinite(wrench.force_torque)) {
    error = ContractError::kNonFinite;
  } else if (wrench.frame_id != kBodyFrameId) {
    error = ContractError::kBodyFrame;
  }
  return MakeAssessment(error, validity);
}

}  // namespace intrinsic::vehicle

#endif  // INTRINSIC_VEHICLE_VEHICLE_CONTRACT_POLICY_H_
