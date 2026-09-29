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

// Rigid-body Coriolis and centripetal matrix in the body frame.
//
// Senior-approved Fossen matrix, consistent with the rigid-body mass
// matrix. Let ν = [ν1; ν2] with ν1 = [u, v, w] and ν2 = [p, q, r]. S(a)
// is skew-symmetric with S(a) b = a cross b, the same layout as
// ComputeRigidBodyMassMatrix. With that matrix M_RB,
//
//   p1 = M11 ν1 + M12 ν2 = m ν1 - m S(r_g) ν2
//   p2 = M21 ν1 + M22 ν2 = m S(r_g) ν1 + I_b ν2
//   C_RB(ν) = [ 0,       -S(p1) ]
//             [ -S(p1),  -S(p2) ]
//
// I_b is the inertia about the body origin from that mass matrix,
// I_g - m S(r_g) S(r_g). C_RB is 6x6, row-major, in surge, sway, heave,
// roll, pitch, yaw order. Units are SI. The matrix is body-frame. This
// function does not convert ENU and NED.
//
// This term is only the rigid-body Coriolis matrix. It does not compute
// added-mass Coriolis, damping, buoyancy, actuator commands, or a state
// derivative. VehicleDynamics::Evaluate does not call it: that contract
// returns a derivative and has no Coriolis output.
//
// Mass, inertia, and the center of gravity are checked by
// ComputeRigidBodyMassMatrix, in that function's order. A non-finite body
// twist is rejected after those checks. The first defect wins. A failed
// call returns kInvalidArgument and a zero matrix. Outputs are finite. The
// same inputs produce the same matrix. The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_CORIOLIS_MATRIX_H_
#define INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_CORIOLIS_MATRIX_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Row-major body-frame rigid-body Coriolis matrix.
// coefficients[row * 6 + col].
struct RigidBodyCoriolisMatrix {
  std::array<double, kSpatialDof * kSpatialDof> coefficients = {};
};

[[nodiscard]] StatusOr<RigidBodyCoriolisMatrix> ComputeRigidBodyCoriolisMatrix(
    const parameters::MassInertia& mass_inertia,
    const parameters::Vec3& center_of_gravity_m,
    const std::array<double, kSpatialDof>& body_twist);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_CORIOLIS_MATRIX_H_
