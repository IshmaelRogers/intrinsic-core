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

#include "intrinsic/vehicle/guidance/guidance_step.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::guidance {
namespace {

constexpr std::string_view kUpdatePeriodMessage =
    "update period must be finite and greater than or equal to zero";
constexpr std::string_view kStateFrameMessage =
    "state pose frame must be world_enu or world_ned";
constexpr std::string_view kStateFiniteMessage = "state value must be finite";
constexpr std::string_view kStateQuaternionMessage =
    "state orientation quaternion must have unit norm";
constexpr std::string_view kConfidenceMessage =
    "confidence must be finite and in [0, 1]";
constexpr std::string_view kHorizonMessage =
    "horizon must be finite and greater than or equal to zero";
constexpr std::string_view kMissingObjectiveMessage = "objective is unset";
constexpr std::string_view kPoseFiniteMessage = "pose value must be finite";
constexpr std::string_view kPoseQuaternionMessage =
    "pose orientation quaternion must have unit norm";
constexpr std::string_view kPoseFrameMessage =
    "pose frame must be world_enu or world_ned";
constexpr std::string_view kTwistFiniteMessage = "twist value must be finite";
constexpr std::string_view kTwistFrameMessage = "twist frame must be body";
constexpr std::string_view kTrajectoryEmptyMessage = "trajectory id is empty";
constexpr std::string_view kTrajectoryLengthMessage =
    "trajectory id is longer than 64 bytes";
constexpr std::string_view kObjectiveMessage =
    "objective is not pose, twist, or trajectory";
constexpr std::string_view kSnapshotMessage =
    "snapshot_id does not match the vehicle state snapshot";
constexpr std::string_view kExpiredMessage =
    "horizon does not cover the update period";

template <std::size_t N>
bool AllFinite(const std::array<double, N>& values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool UnitQuaternion(const std::array<double, 4>& quaternion) {
  const double norm_sq = quaternion[kQuatX] * quaternion[kQuatX] +
                         quaternion[kQuatY] * quaternion[kQuatY] +
                         quaternion[kQuatZ] * quaternion[kQuatZ] +
                         quaternion[kQuatW] * quaternion[kQuatW];
  const double norm = std::sqrt(norm_sq);
  return std::isfinite(norm) &&
         std::abs(norm - 1.0) <= kUnitQuaternionTolerance;
}

bool WorldFrame(FrameId frame) {
  return frame == FrameId::kWorldEnu || frame == FrameId::kWorldNed;
}

GuidanceStatus ValidateState(const VehicleStateRt& state) {
  if (!WorldFrame(state.pose_frame)) {
    return GuidanceStatus::Invalid(kStateFrameMessage);
  }
  if (!AllFinite(state.position_m) || !AllFinite(state.orientation_xyzw) ||
      !AllFinite(state.body_twist)) {
    return GuidanceStatus::InvalidArgument(kStateFiniteMessage);
  }
  if (!UnitQuaternion(state.orientation_xyzw)) {
    return GuidanceStatus::Invalid(kStateQuaternionMessage);
  }
  return GuidanceStatus::Ok();
}

GuidanceStatus ValidateObjective(const DesiredMotionRt& intent) {
  switch (intent.objective) {
    case ObjectiveKind::kUnset:
      return GuidanceStatus::MissingObjective(kMissingObjectiveMessage);
    case ObjectiveKind::kPose:
      if (!AllFinite(intent.position_m) ||
          !AllFinite(intent.orientation_xyzw)) {
        return GuidanceStatus::InvalidArgument(kPoseFiniteMessage);
      }
      if (!UnitQuaternion(intent.orientation_xyzw)) {
        return GuidanceStatus::Invalid(kPoseQuaternionMessage);
      }
      if (!WorldFrame(intent.pose_frame)) {
        return GuidanceStatus::Invalid(kPoseFrameMessage);
      }
      return GuidanceStatus::Ok();
    case ObjectiveKind::kTwist:
      if (!AllFinite(intent.body_twist)) {
        return GuidanceStatus::InvalidArgument(kTwistFiniteMessage);
      }
      if (intent.twist_frame != FrameId::kBody) {
        return GuidanceStatus::Invalid(kTwistFrameMessage);
      }
      return GuidanceStatus::Ok();
    case ObjectiveKind::kTrajectory:
      if (intent.trajectory_id.length == 0) {
        return GuidanceStatus::Invalid(kTrajectoryEmptyMessage);
      }
      if (intent.trajectory_id.length > kTrajectoryIdCapacity) {
        return GuidanceStatus::InvalidArgument(kTrajectoryLengthMessage);
      }
      return GuidanceStatus::Ok();
  }
  return GuidanceStatus::Invalid(kObjectiveMessage);
}

}  // namespace

GuidanceStatus ValidateGuidanceInputs(const DesiredMotionRt& intent,
                                      const VehicleStateRt* state,
                                      Duration update_period) {
  if (!std::isfinite(update_period.seconds) ||
      !(update_period.seconds >= 0.0)) {
    return GuidanceStatus::InvalidArgument(kUpdatePeriodMessage);
  }
  if (state != nullptr) {
    const GuidanceStatus state_status = ValidateState(*state);
    if (!state_status.ok()) {
      return state_status;
    }
  }
  if (intent.has_confidence &&
      (!std::isfinite(intent.confidence) || !(intent.confidence >= 0.0) ||
       !(intent.confidence <= 1.0))) {
    return GuidanceStatus::InvalidArgument(kConfidenceMessage);
  }
  if (intent.has_horizon &&
      (!std::isfinite(intent.horizon_s) || !(intent.horizon_s >= 0.0))) {
    return GuidanceStatus::InvalidArgument(kHorizonMessage);
  }
  const GuidanceStatus objective = ValidateObjective(intent);
  if (!objective.ok()) {
    return objective;
  }
  if (state != nullptr && state->snapshot_id != intent.snapshot_id) {
    return GuidanceStatus::Stale(kSnapshotMessage);
  }
  if (intent.has_horizon && intent.horizon_s < update_period.seconds) {
    return GuidanceStatus::Stale(kExpiredMessage);
  }
  return GuidanceStatus::Ok();
}

}  // namespace intrinsic::vehicle::guidance
