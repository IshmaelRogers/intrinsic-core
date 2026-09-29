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

// Rigid-body mass matrix in the body frame.
//
// Senior-approved Fossen equation. Let m be the mass, r_g the center of
// gravity in the body frame, and I_g the inertia about the center of
// gravity. S(a) is skew-symmetric with S(a) b = a cross b. Inertia about
// the body origin is I_b = I_g - m S(r_g) S(r_g).
//
//   M_RB = [ m I_3,    -m S(r_g) ]
//          [ m S(r_g),  I_b      ]
//
// M_RB is 6x6, row-major, in surge, sway, heave, roll, pitch, yaw order.
// Units are SI. The matrix is body-frame. This function does not convert
// ENU and NED.
//
// This term is only the rigid-body mass matrix. It does not compute
// Coriolis, damping, buoyancy, added mass, actuator commands, or a state
// derivative. VehicleDynamics::Evaluate does not call it: that contract
// returns a derivative and has no mass-matrix output.
//
// Inputs are the parameters-package mass/inertia and the body-frame center
// of gravity (Centers::center_of_gravity_m). Checks follow that package:
// mass finite and > 0, inertia entries finite, symmetric within 1e-9, and
// positive definite by unpivoted Cholesky with no pivot slack, then the
// center of gravity finite. The first defect wins. A failed call returns
// kInvalidArgument and a zero matrix. Outputs are finite. The same inputs
// produce the same matrix. The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_MASS_MATRIX_H_
#define INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_MASS_MATRIX_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Row-major body-frame rigid-body mass matrix. coefficients[row * 6 + col].
struct RigidBodyMassMatrix {
  std::array<double, kSpatialDof * kSpatialDof> coefficients = {};
};

[[nodiscard]] StatusOr<RigidBodyMassMatrix> ComputeRigidBodyMassMatrix(
    const parameters::MassInertia& mass_inertia,
    const parameters::Vec3& center_of_gravity_m);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_RIGID_BODY_MASS_MATRIX_H_
