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

#include "intrinsic/vehicle/dynamics/gravity_buoyancy_restoring_wrench.h"

#include <array>
#include <cmath>
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

static_assert(kSpatialDof == parameters::kSpatialDof);
static_assert(kSpatialDof == 6);

constexpr std::string_view kMassNonFiniteMessage =
    "mass_inertia.mass_kg must be finite";
constexpr std::string_view kMassNotPositiveMessage =
    "mass_inertia.mass_kg must be greater than zero";
constexpr std::string_view kCenterOfGravityNonFiniteMessage =
    "centers.center_of_gravity_m must be finite";
constexpr std::string_view kCenterOfBuoyancyNonFiniteMessage =
    "centers.center_of_buoyancy_m must be finite";
constexpr std::string_view kVolumeNonFiniteMessage =
    "buoyancy.displaced_volume_m3 must be finite";
constexpr std::string_view kVolumeNotPositiveMessage =
    "buoyancy.displaced_volume_m3 must be greater than zero";
constexpr std::string_view kGravityNonFiniteMessage =
    "environment.gravity_m_s2 must be finite";
constexpr std::string_view kGravityNotPositiveMessage =
    "environment.gravity_m_s2 must be greater than zero";
constexpr std::string_view kDensityNonFiniteMessage =
    "environment.fluid_density_kg_m3 must be finite";
constexpr std::string_view kDensityNotPositiveMessage =
    "environment.fluid_density_kg_m3 must be greater than zero";
constexpr std::string_view kPoseFrameMessage =
    "pose_frame must be world_enu or world_ned";
constexpr std::string_view kOrientationNonFiniteMessage =
    "orientation_xyzw must be finite";
constexpr std::string_view kOrientationUnitMessage =
    "orientation_xyzw must have unit norm";
constexpr std::string_view kRestoringWrenchNonFiniteMessage =
    "restoring wrench is not finite";

StatusOr<RestoringWrench> Reject(std::string_view message) {
  return StatusOr<RestoringWrench>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// -0 compares equal to 0 and is a different bit pattern. Stored zeros use
// +0 so a cancelling wrench is the zero wrench.
double PositiveZero(double value) { return value == 0.0 ? 0.0 : value; }

bool Finite(const parameters::Vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool FiniteQuaternion(const std::array<double, 4>& quaternion) {
  return std::isfinite(quaternion[kQuatX]) &&
         std::isfinite(quaternion[kQuatY]) &&
         std::isfinite(quaternion[kQuatZ]) && std::isfinite(quaternion[kQuatW]);
}

// Absolute |norm - 1| <= 1e-9. Does not renormalize. Matches the dynamics
// pose contract and the embodiment frame policy.
bool UnitQuaternion(const std::array<double, 4>& quaternion) {
  const double norm_sq = quaternion[kQuatX] * quaternion[kQuatX] +
                         quaternion[kQuatY] * quaternion[kQuatY] +
                         quaternion[kQuatZ] * quaternion[kQuatZ] +
                         quaternion[kQuatW] * quaternion[kQuatW];
  const double norm = std::sqrt(norm_sq);
  return std::isfinite(norm) &&
         std::abs(norm - 1.0) <= kUnitQuaternionTolerance;
}

// Body-to-navigation rotation, then its transpose. Element formulas match
// RotateBodyToPose in zero_force_dynamics.cc and the active Hamilton map
// in the embodiment frame policy (v_nav = R v_body).
std::array<double, 3> RotateNavToBody(const std::array<double, 4>& quaternion,
                                      const std::array<double, 3>& nav) {
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

std::array<double, 3> WorldDown(FrameId pose_frame) {
  if (pose_frame == FrameId::kWorldNed) {
    return {0.0, 0.0, 1.0};
  }
  return {0.0, 0.0, -1.0};
}

double CrossX(const parameters::Vec3& arm, const std::array<double, 3>& force) {
  return arm.y * force[kZ] - arm.z * force[kY];
}

double CrossY(const parameters::Vec3& arm, const std::array<double, 3>& force) {
  return arm.z * force[kX] - arm.x * force[kZ];
}

double CrossZ(const parameters::Vec3& arm, const std::array<double, 3>& force) {
  return arm.x * force[kY] - arm.y * force[kX];
}

}  // namespace

StatusOr<RestoringWrench> ComputeGravityBuoyancyRestoringWrench(
    const parameters::MassInertia& mass_inertia,
    const parameters::Buoyancy& buoyancy, const parameters::Centers& centers,
    const parameters::Environment& environment, FrameId pose_frame,
    const std::array<double, 4>& orientation_xyzw) {
  const double mass = mass_inertia.mass_kg;
  if (!std::isfinite(mass)) {
    return Reject(kMassNonFiniteMessage);
  }
  if (!(mass > 0.0)) {
    return Reject(kMassNotPositiveMessage);
  }
  if (!Finite(centers.center_of_gravity_m)) {
    return Reject(kCenterOfGravityNonFiniteMessage);
  }
  if (!Finite(centers.center_of_buoyancy_m)) {
    return Reject(kCenterOfBuoyancyNonFiniteMessage);
  }
  const double volume = buoyancy.displaced_volume_m3;
  if (!std::isfinite(volume)) {
    return Reject(kVolumeNonFiniteMessage);
  }
  if (!(volume > 0.0)) {
    return Reject(kVolumeNotPositiveMessage);
  }
  const double gravity = environment.gravity_m_s2;
  if (!std::isfinite(gravity)) {
    return Reject(kGravityNonFiniteMessage);
  }
  if (!(gravity > 0.0)) {
    return Reject(kGravityNotPositiveMessage);
  }
  const double density = environment.fluid_density_kg_m3;
  if (!std::isfinite(density)) {
    return Reject(kDensityNonFiniteMessage);
  }
  if (!(density > 0.0)) {
    return Reject(kDensityNotPositiveMessage);
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

  // f_W^n = W e_down, f_B^n = -B e_down, then R^T into the body frame.
  const double weight = mass * gravity;
  const double buoyancy_magnitude = density * gravity * volume;
  const std::array<double, 3> down = WorldDown(pose_frame);
  const std::array<double, 3> weight_nav = {
      weight * down[kX],
      weight * down[kY],
      weight * down[kZ],
  };
  const std::array<double, 3> buoyancy_nav = {
      -buoyancy_magnitude * down[kX],
      -buoyancy_magnitude * down[kY],
      -buoyancy_magnitude * down[kZ],
  };
  const std::array<double, 3> weight_body =
      RotateNavToBody(orientation_xyzw, weight_nav);
  const std::array<double, 3> buoyancy_body =
      RotateNavToBody(orientation_xyzw, buoyancy_nav);

  const parameters::Vec3& center_of_gravity = centers.center_of_gravity_m;
  const parameters::Vec3& center_of_buoyancy = centers.center_of_buoyancy_m;
  RestoringWrench wrench;
  wrench.components[kSurge] = PositiveZero(weight_body[kX] + buoyancy_body[kX]);
  wrench.components[kSway] = PositiveZero(weight_body[kY] + buoyancy_body[kY]);
  wrench.components[kHeave] = PositiveZero(weight_body[kZ] + buoyancy_body[kZ]);
  wrench.components[kRoll] =
      PositiveZero(CrossX(center_of_gravity, weight_body) +
                   CrossX(center_of_buoyancy, buoyancy_body));
  wrench.components[kPitch] =
      PositiveZero(CrossY(center_of_gravity, weight_body) +
                   CrossY(center_of_buoyancy, buoyancy_body));
  wrench.components[kYaw] =
      PositiveZero(CrossZ(center_of_gravity, weight_body) +
                   CrossZ(center_of_buoyancy, buoyancy_body));

  for (double component : wrench.components) {
    if (!std::isfinite(component)) {
      return Reject(kRestoringWrenchNonFiniteMessage);
    }
  }
  return StatusOr<RestoringWrench>::Ok(wrench);
}

}  // namespace intrinsic::vehicle::dynamics
