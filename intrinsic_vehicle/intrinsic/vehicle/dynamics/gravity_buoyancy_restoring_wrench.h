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

// Gravity and buoyancy restoring wrench in the body frame.
//
// Senior-approved restoring term. m is MassInertia::mass_kg. g is
// Environment::gravity_m_s2, a positive magnitude. W = m g. V is
// Buoyancy::displaced_volume_m3. ρ is Environment::fluid_density_kg_m3.
// B = ρ g V. r_g and r_b are Centers::center_of_gravity_m and
// center_of_buoyancy_m, body-frame positions in meters.
//
// World-down depends on the navigation frame of the supplied pose:
//
//   NED: e_down = [0, 0, +1]
//   ENU: e_down = [0, 0, -1]
//
// World-frame forces are f_W^n = W e_down and f_B^n = -B e_down. R is the
// body-to-navigation rotation of the supplied Hamilton quaternion, stored
// x, y, z, w. It is the same active map as the zero-force pose derivative:
// a body vector maps to navigation by R. The body-frame wrench is
//
//   f_W^b = R^T f_W^n
//   f_B^b = R^T f_B^n
//   τ_g = [ f_W^b + f_B^b ; r_g × f_W^b + r_b × f_B^b ]
//
// components[0..2] are force in newtons (surge, sway, heave).
// components[3..5] are torque in newton-meters (roll, pitch, yaw).
//
// This term is only gravity and buoyancy. It does not read inertia,
// damping, added mass, current, thrusters, or a body twist. It does not
// compute drag, Coriolis, or an actuator wrench. VehicleDynamics::Evaluate
// does not call it: that contract returns a derivative and has no
// restoring-wrench output.
//
// Checks follow ValidateMarineModel for the fields this term reads, then
// the pose. The first defect wins: mass non-finite, mass not strictly
// positive, center of gravity non-finite, center of buoyancy non-finite,
// displaced volume non-finite, displaced volume not strictly positive,
// gravity non-finite, gravity not strictly positive, density non-finite,
// density not strictly positive, pose frame, orientation non-finite,
// orientation not a unit quaternion within 1e-9, then a non-finite wrench.
// A failed call returns kInvalidArgument and a zero wrench. Outputs are
// finite. The same inputs produce the same wrench. The function does not
// allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_GRAVITY_BUOYANCY_RESTORING_WRENCH_H_
#define INTRINSIC_VEHICLE_DYNAMICS_GRAVITY_BUOYANCY_RESTORING_WRENCH_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Body-frame generalized restoring force. components[0..2] are force in
// newtons (surge, sway, heave). components[3..5] are torque in
// newton-meters (roll, pitch, yaw).
struct RestoringWrench {
  std::array<double, kSpatialDof> components = {};
};

// pose_frame is the navigation frame of orientation_xyzw. The quaternion
// is navigation-from-body, Hamilton, stored x, y, z, w.
[[nodiscard]] StatusOr<RestoringWrench> ComputeGravityBuoyancyRestoringWrench(
    const parameters::MassInertia& mass_inertia,
    const parameters::Buoyancy& buoyancy, const parameters::Centers& centers,
    const parameters::Environment& environment, FrameId pose_frame,
    const std::array<double, 4>& orientation_xyzw);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_GRAVITY_BUOYANCY_RESTORING_WRENCH_H_
