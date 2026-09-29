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

// Hydrodynamic added-mass matrix in the body frame.
//
// Senior-approved mapping from the marine-model added-mass parameters.
// AddedMass::coefficients is the row-major 6x6 matrix M_A in body-twist
// order: surge, sway, heave, roll, pitch, yaw. That is the order used by
// ComputeRigidBodyMassMatrix and ValidateMarineModel. Units are the SI
// units stored on AddedMass.
//
// M_A is that stored matrix. This function copies the coefficients. It
// does not derive them from geometry, does not change signs, and does not
// convert ENU and NED. Added mass is already in the body frame.
//
// This term is only the added-mass matrix. It does not compute Coriolis,
// damping, buoyancy, the rigid-body mass matrix, actuator commands, or a
// state derivative. VehicleDynamics::Evaluate does not call it: that
// contract returns a derivative and has no mass-matrix output.
//
// Checks follow ValidateMarineModel for this field. Every entry must be
// finite. Symmetry uses the absolute tolerance 1e-9. Positive definiteness
// is unpivoted Cholesky with no pivot slack, on the upper triangle. The
// returned matrix uses the stored entries. The first defect wins. A failed
// call returns kInvalidArgument and a zero matrix. Outputs are finite. The
// same inputs produce the same matrix. The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_MATRIX_H_
#define INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_MATRIX_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Row-major body-frame added-mass matrix. coefficients[row * 6 + col].
struct AddedMassMatrix {
  std::array<double, kSpatialDof * kSpatialDof> coefficients = {};
};

[[nodiscard]] StatusOr<AddedMassMatrix> ComputeAddedMassMatrix(
    const parameters::AddedMass& added_mass);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_ADDED_MASS_MATRIX_H_
