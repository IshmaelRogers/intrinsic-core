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

#include "intrinsic/vehicle/control/reference_control.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::control {
namespace {

constexpr std::string_view kUpdatePeriodMessage =
    "update period must be finite and greater than or equal to zero";
constexpr std::string_view kStateFrameMessage =
    "state pose frame must be world_enu or world_ned";
constexpr std::string_view kStateFiniteMessage = "state value must be finite";
constexpr std::string_view kStateQuaternionMessage =
    "state orientation quaternion must have unit norm";
constexpr std::string_view kReferencePeriodMessage =
    "reference update period must be finite and greater than or equal to zero";
constexpr std::string_view kPoseFiniteMessage = "pose value must be finite";
constexpr std::string_view kPoseQuaternionMessage =
    "pose orientation quaternion must have unit norm";
constexpr std::string_view kPoseFrameMessage =
    "pose frame must be world_enu or world_ned";
constexpr std::string_view kTwistFiniteMessage = "twist value must be finite";
constexpr std::string_view kTwistFrameMessage = "twist frame must be body";
constexpr std::string_view kMissingMessage =
    "reference has no pose or twist objective";
constexpr std::string_view kSnapshotMessage =
    "snapshot_id does not match the vehicle state snapshot";

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
  const double norm_sq =
      quaternion[guidance::kQuatX] * quaternion[guidance::kQuatX] +
      quaternion[guidance::kQuatY] * quaternion[guidance::kQuatY] +
      quaternion[guidance::kQuatZ] * quaternion[guidance::kQuatZ] +
      quaternion[guidance::kQuatW] * quaternion[guidance::kQuatW];
  const double norm = std::sqrt(norm_sq);
  return std::isfinite(norm) &&
         std::abs(norm - 1.0) <= guidance::kUnitQuaternionTolerance;
}

bool WorldFrame(FrameId frame) {
  return frame == FrameId::kWorldEnu || frame == FrameId::kWorldNed;
}

bool ValidPeriod(Duration period) {
  return std::isfinite(period.seconds) && period.seconds >= 0.0;
}

}  // namespace

ControlStatus ValidateControlInputs(const MotionReferenceRt& reference,
                                    const VehicleStateRt& state,
                                    Duration update_period) {
  if (!ValidPeriod(update_period)) {
    return ControlStatus::InvalidArgument(kUpdatePeriodMessage);
  }
  if (!WorldFrame(state.pose_frame)) {
    return ControlStatus::Invalid(kStateFrameMessage);
  }
  if (!AllFinite(state.position_m) || !AllFinite(state.orientation_xyzw) ||
      !AllFinite(state.body_twist)) {
    return ControlStatus::InvalidArgument(kStateFiniteMessage);
  }
  if (!UnitQuaternion(state.orientation_xyzw)) {
    return ControlStatus::Invalid(kStateQuaternionMessage);
  }
  if (!ValidPeriod(reference.update_period)) {
    return ControlStatus::InvalidArgument(kReferencePeriodMessage);
  }
  if (reference.has_pose) {
    if (!AllFinite(reference.position_m) ||
        !AllFinite(reference.orientation_xyzw)) {
      return ControlStatus::InvalidArgument(kPoseFiniteMessage);
    }
    if (!UnitQuaternion(reference.orientation_xyzw)) {
      return ControlStatus::Invalid(kPoseQuaternionMessage);
    }
    if (!WorldFrame(reference.pose_frame)) {
      return ControlStatus::Invalid(kPoseFrameMessage);
    }
  }
  if (reference.has_twist) {
    if (!AllFinite(reference.body_twist)) {
      return ControlStatus::InvalidArgument(kTwistFiniteMessage);
    }
    if (reference.twist_frame != FrameId::kBody) {
      return ControlStatus::Invalid(kTwistFrameMessage);
    }
  }
  if (!reference.has_pose && !reference.has_twist) {
    return ControlStatus::MissingObjective(kMissingMessage);
  }
  if (state.snapshot_id != reference.snapshot_id) {
    return ControlStatus::Stale(kSnapshotMessage);
  }
  return ControlStatus::Ok();
}

}  // namespace intrinsic::vehicle::control
