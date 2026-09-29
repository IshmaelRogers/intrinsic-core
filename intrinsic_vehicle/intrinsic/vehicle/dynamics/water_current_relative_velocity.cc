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

#include "intrinsic/vehicle/dynamics/water_current_relative_velocity.h"

#include <array>
#include <cmath>
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

static_assert(kSpatialDof == parameters::kSpatialDof);
static_assert(kSpatialDof == 6);

constexpr std::string_view kCurrentNonFiniteMessage =
    "environment.current_velocity_m_s must be finite";
constexpr std::string_view kCurrentFrameEmptyMessage =
    "environment.current_frame_id must be non-empty";
constexpr std::string_view kCurrentFrameMessage =
    "environment.current_frame_id must be world_enu, world_ned, or body";
constexpr std::string_view kPoseFrameMessage =
    "pose_frame must be world_enu or world_ned";
constexpr std::string_view kOrientationNonFiniteMessage =
    "orientation_xyzw must be finite";
constexpr std::string_view kOrientationUnitMessage =
    "orientation_xyzw must have unit norm";
constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kRelativeVelocityNonFiniteMessage =
    "relative velocity is not finite";

StatusOr<CurrentRelativeTwist> Reject(std::string_view message) {
  return StatusOr<CurrentRelativeTwist>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// -0 compares equal to 0 and is a different bit pattern. Stored zeros use
// +0 so a cancelling component is the zero component.
double PositiveZero(double value) { return value == 0.0 ? 0.0 : value; }

bool Finite(const parameters::Vec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool FiniteQuaternion(const std::array<double, 4> &quaternion) {
  return std::isfinite(quaternion[kQuatX]) &&
         std::isfinite(quaternion[kQuatY]) &&
         std::isfinite(quaternion[kQuatZ]) && std::isfinite(quaternion[kQuatW]);
}

bool FiniteTwist(const std::array<double, kSpatialDof> &twist) {
  for (double component : twist) {
    if (!std::isfinite(component)) {
      return false;
    }
  }
  return true;
}

// Absolute |norm - 1| <= 1e-9. Does not renormalize. Matches the dynamics
// pose contract and the embodiment frame policy.
bool UnitQuaternion(const std::array<double, 4> &quaternion) {
  const double norm_sq = quaternion[kQuatX] * quaternion[kQuatX] +
                         quaternion[kQuatY] * quaternion[kQuatY] +
                         quaternion[kQuatZ] * quaternion[kQuatZ] +
                         quaternion[kQuatW] * quaternion[kQuatW];
  const double norm = std::sqrt(norm_sq);
  return std::isfinite(norm) &&
         std::abs(norm - 1.0) <= kUnitQuaternionTolerance;
}

// Body-to-navigation rotation, then its transpose. Element formulas match
// RotateBodyToPose in zero_force_dynamics.cc and RotateNavToBody in the
// restoring wrench. The active Hamilton map is v_nav = R v_body.
std::array<double, 3> RotateNavToBody(const std::array<double, 4> &quaternion,
                                      const std::array<double, 3> &nav) {
  const double x = quaternion[kQuatX];
  const double y = quaternion[kQuatY];
  const double z = quaternion[kQuatZ];
  const double w = quaternion[kQuatW];
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  const double r00 = 1.0 - 2.0 * (yy + zz);
  const double r01 = 2.0 * (xy - wz);
  const double r02 = 2.0 * (xz + wy);
  const double r10 = 2.0 * (xy + wz);
  const double r11 = 1.0 - 2.0 * (xx + zz);
  const double r12 = 2.0 * (yz - wx);
  const double r20 = 2.0 * (xz - wy);
  const double r21 = 2.0 * (yz + wx);
  const double r22 = 1.0 - 2.0 * (xx + yy);
  return {
      r00 * nav[kX] + r10 * nav[kY] + r20 * nav[kZ],
      r01 * nav[kX] + r11 * nav[kY] + r21 * nav[kZ],
      r02 * nav[kX] + r12 * nav[kY] + r22 * nav[kZ],
  };
}

bool BodyCurrent(std::string_view frame_id) {
  return frame_id == parameters::kBodyFrameId;
}

bool WorldCurrent(std::string_view frame_id) {
  return frame_id == parameters::kWorldEnuFrameId ||
         frame_id == parameters::kWorldNedFrameId;
}

} // namespace

StatusOr<CurrentRelativeTwist> ComputeWaterCurrentRelativeVelocity(
    const parameters::Environment &environment, FrameId pose_frame,
    const std::array<double, 4> &orientation_xyzw,
    const std::array<double, kSpatialDof> &body_twist) {
  if (!Finite(environment.current_velocity_m_s)) {
    return Reject(kCurrentNonFiniteMessage);
  }
  const std::string_view current_frame = environment.current_frame_id;
  if (current_frame.empty()) {
    return Reject(kCurrentFrameEmptyMessage);
  }
  if (!BodyCurrent(current_frame) && !WorldCurrent(current_frame)) {
    return Reject(kCurrentFrameMessage);
  }
  if (pose_frame != FrameId::kWorldEnu && pose_frame != FrameId::kWorldNed) {
    return Reject(kPoseFrameMessage);
  }
  if (!FiniteQuaternion(orientation_xyzw)) {
    return Reject(kOrientationNonFiniteMessage);
  }
  if (!UnitQuaternion(orientation_xyzw)) {
    return Reject(kOrientationUnitMessage);
  }
  if (!FiniteTwist(body_twist)) {
    return Reject(kBodyTwistNonFiniteMessage);
  }

  // World frames share R^T. The frame id names the vector's basis. It does
  // not swap ENU and NED. A body-frame current is already v_c^b.
  const parameters::Vec3 &current = environment.current_velocity_m_s;
  const std::array<double, 3> current_nav = {current.x, current.y, current.z};
  const std::array<double, 3> current_body =
      BodyCurrent(current_frame)
          ? current_nav
          : RotateNavToBody(orientation_xyzw, current_nav);

  CurrentRelativeTwist relative;
  relative.components[kSurge] =
      PositiveZero(body_twist[kSurge] - current_body[kX]);
  relative.components[kSway] =
      PositiveZero(body_twist[kSway] - current_body[kY]);
  relative.components[kHeave] =
      PositiveZero(body_twist[kHeave] - current_body[kZ]);
  // Angular current is zero. ν2 is copied.
  relative.components[kRoll] = PositiveZero(body_twist[kRoll]);
  relative.components[kPitch] = PositiveZero(body_twist[kPitch]);
  relative.components[kYaw] = PositiveZero(body_twist[kYaw]);

  for (double component : relative.components) {
    if (!std::isfinite(component)) {
      return Reject(kRelativeVelocityNonFiniteMessage);
    }
  }
  return StatusOr<CurrentRelativeTwist>::Ok(relative);
}

} // namespace intrinsic::vehicle::dynamics
