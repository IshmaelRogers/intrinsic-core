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

// Linear and quadratic damping wrench in the body frame.
//
// Senior-approved damping term. D_L is Damping::linear_coefficients, the
// row-major 6x6 matrix in surge, sway, heave, roll, pitch, yaw order.
// d_q is Damping::quadratic_coefficients, the length-6 diagonal quadratic
// coefficients in that same order. ν is the supplied body twist, also in
// that order. D_Q(|ν|) = diag(d_q ∘ |ν|). The body-frame wrench is
//
//   τ_d = -D_L ν - diag(d_q_i |ν_i|) ν
//       = -(D_L + D_Q(|ν|)) ν
//
// Component i of the quadratic contribution is -d_q_i |ν_i| ν_i. Units are
// the SI units stored on Damping: newtons and newton-meters. The matrix
// product uses the stored linear entries. This function does not
// symmetrize them and does not derive them from geometry.
//
// ν is the relative velocity for this leaf. The function does not convert
// ENU and NED and does not subtract an environment current. Callers that
// need a current-relative twist supply that twist themselves.
//
// This term is only the linear and quadratic damping wrench. It does not
// compute Coriolis, restoring, buoyancy, actuator commands, or a state
// derivative. VehicleDynamics::Evaluate does not call it: that contract
// returns a derivative and has no damping-wrench output.
//
// Checks follow ValidateMarineModel for Damping, then the twist. Linear
// entries must be finite, symmetric within 1e-9, and positive definite by
// unpivoted Cholesky with no pivot slack, on the upper triangle. Each
// quadratic coefficient must be finite and >= 0. The body twist must be
// finite. The first defect wins: linear non-finite, linear asymmetry,
// linear definiteness, quadratic non-finite, quadratic sign, twist, then
// a non-finite wrench. A failed call returns kInvalidArgument and a zero
// wrench. Outputs are finite. The same inputs produce the same wrench.
// The function does not allocate.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_LINEAR_QUADRATIC_DAMPING_WRENCH_H_
#define INTRINSIC_VEHICLE_DYNAMICS_LINEAR_QUADRATIC_DAMPING_WRENCH_H_

#include <array>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Body-frame generalized damping force. components[0..2] are force in
// newtons (surge, sway, heave). components[3..5] are torque in
// newton-meters (roll, pitch, yaw).
struct DampingWrench {
  std::array<double, kSpatialDof> components = {};
};

[[nodiscard]] StatusOr<DampingWrench> ComputeLinearQuadraticDampingWrench(
    const parameters::Damping& damping,
    const std::array<double, kSpatialDof>& body_twist);

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_LINEAR_QUADRATIC_DAMPING_WRENCH_H_
