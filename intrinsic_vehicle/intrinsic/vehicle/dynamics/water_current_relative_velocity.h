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

// Water-current relative velocity in the body frame.
//
// Senior-approved kinematic helper. current_velocity_m_s is the linear
// fluid current in meters/second, expressed in Environment::current_frame_id.
// That id is exactly one of world_enu, world_ned, or body. ν = [ν1; ν2] is
// the body twist in surge, sway, heave, roll, pitch, yaw order. ν1 is linear
// meters/second. ν2 is angular radians/second.
//
// R is the body-to-navigation rotation of the supplied Hamilton quaternion,
// stored x, y, z, w. It is the same active map as the restoring wrench and
// the zero-force pose derivative: a body vector maps to navigation by R.
// Angular current is zero. Linear current in the body frame is
//
//   body:                 v_c^b = current_velocity_m_s
//   world_enu, world_ned: v_c^b = R^T current_velocity_m_s
//
// The frame id selects which basis the supplied current vector already
// uses. world_enu and world_ned take the same rotation. This function does
// not swap ENU and NED axes. The relative body twist is
//
//   ν_r = [ ν1 − v_c^b ; ν2 ]
//
// A zero current leaves ν_r = ν. components[0..2] are linear meters/second.
// components[3..5] are angular radians/second. This is not a wrench.
//
// Gravity, density, mass, inertia, damping, added mass, and thrusters are
// not read. The function does not compute a hydrodynamic force, a restoring
// wrench, Coriolis, damping, or a state derivative.
// MarineForceDynamics::Evaluate calls it and stores ν_r. Added-mass
// Coriolis and damping then read that twist. Rigid-body Coriolis reads ν.
//
// Checks follow ValidateMarineModel for the current, then the pose, then
// the twist. The first defect wins: current non-finite, current frame
// empty, current frame unknown, pose frame, orientation non-finite,
// orientation not a unit quaternion within 1e-9, twist non-finite, then a
// non-finite relative twist. A failed call returns kInvalidArgument and a
// zero relative twist. Outputs are finite. The same inputs produce the same
// twist. The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_WATER_CURRENT_RELATIVE_VELOCITY_H_
#define INTRINSIC_VEHICLE_DYNAMICS_WATER_CURRENT_RELATIVE_VELOCITY_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Body-frame twist relative to the fluid. components[0..2] are linear
// meters/second (surge, sway, heave). components[3..5] are angular
// radians/second (roll, pitch, yaw). Not a force or a wrench.
struct CurrentRelativeTwist {
  std::array<double, kSpatialDof> components = {};
};

// pose_frame is the navigation frame of orientation_xyzw. The quaternion
// is navigation-from-body, Hamilton, stored x, y, z, w. body_twist is ν.
[[nodiscard]] StatusOr<CurrentRelativeTwist>
ComputeWaterCurrentRelativeVelocity(
    const parameters::Environment &environment, FrameId pose_frame,
    const std::array<double, 4> &orientation_xyzw,
    const std::array<double, kSpatialDof> &body_twist);

} // namespace intrinsic::vehicle::dynamics

#endif // INTRINSIC_VEHICLE_DYNAMICS_WATER_CURRENT_RELATIVE_VELOCITY_H_
