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

// Added-mass Coriolis and centripetal matrix in the body frame.
//
// Senior-approved Fossen matrix, same block layout as
// ComputeRigidBodyCoriolisMatrix. Let ν = [ν1; ν2] with ν1 = [u, v, w]
// and ν2 = [p, q, r]. S(a) is skew-symmetric with S(a) b = a cross b.
// M_A is the validated added-mass matrix from ComputeAddedMassMatrix,
// partitioned into 3x3 blocks [[M11, M12], [M21, M22]]. With
//
//   p1 = M11 ν1 + M12 ν2
//   p2 = M21 ν1 + M22 ν2
//   C_A(ν) = [ 0,       -S(p1) ]
//            [ -S(p1),  -S(p2) ]
//
// C_A is 6x6, row-major, in surge, sway, heave, roll, pitch, yaw order.
// Units are the SI units stored on AddedMass. The matrix is body-frame.
// This function does not convert ENU and NED and does not derive M_A
// from geometry.
//
// Coefficients come only from M_A. This term does not add the rigid-body
// Coriolis matrix, damping, buoyancy, actuator commands, or a state
// derivative. MarineForceDynamics::Evaluate calls it on the relative
// twist ν_r while composing the hydrodynamic wrench. The matrix is not a
// DynamicsResult field. The composed term is -C_A(ν_r) ν_r.
//
// Added mass is checked by ComputeAddedMassMatrix, in that function's
// order. A non-finite body twist is rejected after those checks. The
// first defect wins. A failed call returns kInvalidArgument and a zero
// matrix. Outputs are finite. The same inputs produce the same matrix.
// The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_CORIOLIS_MATRIX_H_
#define INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_CORIOLIS_MATRIX_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Row-major body-frame added-mass Coriolis matrix.
// coefficients[row * 6 + col].
struct AddedMassCoriolisMatrix {
  std::array<double, kSpatialDof * kSpatialDof> coefficients = {};
};

[[nodiscard]] StatusOr<AddedMassCoriolisMatrix> ComputeAddedMassCoriolisMatrix(
    const parameters::AddedMass& added_mass,
    const std::array<double, kSpatialDof>& body_twist);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_CORIOLIS_MATRIX_H_
