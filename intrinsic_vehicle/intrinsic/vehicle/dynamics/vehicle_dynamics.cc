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

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

constexpr std::string_view kPoseFrameMessage =
    "state pose frame must be world_enu or world_ned";
constexpr std::string_view kStateFiniteMessage = "state value must be finite";
constexpr std::string_view kQuaternionMessage =
    "orientation quaternion must have unit norm";
constexpr std::string_view kWrenchFrameMessage = "wrench frame must be body";
constexpr std::string_view kWrenchFiniteMessage = "wrench value must be finite";
constexpr std::string_view kEnvironmentFiniteMessage =
    "environment value must be finite";
constexpr std::string_view kGravityMessage =
    "environment gravity must be greater than or equal to zero";
constexpr std::string_view kDensityMessage =
    "environment density must be greater than or equal to zero";
constexpr std::string_view kCurrentFrameMessage =
    "environment current frame must be world_enu, world_ned, or body";
constexpr std::string_view kTimeStepMessage =
    "time step must be finite and greater than or equal to zero";

template <std::size_t N>
bool AllFinite(const std::array<double, N>& values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool KnownWorldOrBody(FrameId frame) {
  return frame == FrameId::kWorldEnu || frame == FrameId::kWorldNed ||
         frame == FrameId::kBody;
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

DynamicsStatus Invalid(std::string_view message) {
  return DynamicsStatus::InvalidArgument(message);
}

}  // namespace

DynamicsStatus ValidateEvaluationInputs(const VehicleStateRt& state,
                                        const BodyWrenchRt& wrench,
                                        const EnvironmentRt& environment,
                                        Duration dt) {
  if (state.pose_frame != FrameId::kWorldEnu &&
      state.pose_frame != FrameId::kWorldNed) {
    return Invalid(kPoseFrameMessage);
  }
  if (!AllFinite(state.position_m) || !AllFinite(state.orientation_xyzw) ||
      !AllFinite(state.body_twist)) {
    return Invalid(kStateFiniteMessage);
  }
  if (!UnitQuaternion(state.orientation_xyzw)) {
    return Invalid(kQuaternionMessage);
  }
  if (wrench.frame != FrameId::kBody) {
    return Invalid(kWrenchFrameMessage);
  }
  if (!AllFinite(wrench.force_n) || !AllFinite(wrench.torque_n_m)) {
    return Invalid(kWrenchFiniteMessage);
  }
  if (!std::isfinite(environment.gravity_m_s2) ||
      !std::isfinite(environment.fluid_density_kg_m3)) {
    return Invalid(kEnvironmentFiniteMessage);
  }
  if (!(environment.gravity_m_s2 >= 0.0)) {
    return Invalid(kGravityMessage);
  }
  if (!(environment.fluid_density_kg_m3 >= 0.0)) {
    return Invalid(kDensityMessage);
  }
  if (!AllFinite(environment.current_velocity_m_s)) {
    return Invalid(kEnvironmentFiniteMessage);
  }
  if (!KnownWorldOrBody(environment.current_frame)) {
    return Invalid(kCurrentFrameMessage);
  }
  if (!std::isfinite(dt.seconds) || !(dt.seconds >= 0.0)) {
    return Invalid(kTimeStepMessage);
  }
  return DynamicsStatus::Ok();
}

}  // namespace intrinsic::vehicle::dynamics
