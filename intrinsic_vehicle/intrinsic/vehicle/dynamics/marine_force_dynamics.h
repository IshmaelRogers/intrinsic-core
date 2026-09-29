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

#ifndef INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_DYNAMICS_H_
#define INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_DYNAMICS_H_

#include <string_view>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::dynamics {

// Composes the marine force terms into DynamicsResult.
//
// Evaluate calls the merged helpers. It does not re-derive their
// equations, invert a mass matrix, integrate, allocate thrusters, or call
// Gazebo. DynamicsResult has no mass-matrix field, so M_RB and M_A are
// not formed and body_acceleration stays zero. total_wrench is the
// derivative input.
//
// Coefficient matrices, mass, centers, and displaced volume come from the
// stored model. Gravity, density, and current come from the environment
// argument. Stored environment and thruster fields are not read.
//
// The body-frame hydrodynamic wrench, excluding the input wrench, is
//
//   τ_hydro = -C_RB(ν) ν - C_A(ν_r) ν_r + τ_damp(ν_r) + τ_g
//
// C_RB uses the body twist ν. C_A and damping use ν_r from the
// water-current helper. τ_damp and τ_g are the helper wrenches, already
// the force on the body. Substituting τ_damp = -D(ν_r) ν_r and
// τ_g = -g(η) is the approved
//
//   τ_hydro = -C_RB(ν) ν - C_A(ν_r) ν_r - D(ν_r) ν_r - g(η)
//
// model_force_n and model_torque_n_m copy τ_hydro. The input wrench is
// passed through as
//
//   total_wrench = τ_input + τ_hydro
//
// The pose rate is the rigid-body kinematic map of ν, the same map as
// ZeroForceDynamics. dt is reported and is not an integration step.
//
// On success the four signed contributions, ν_r, τ_hydro, and
// total_wrench are written to diagnostics. Their sum identity is the
// DynamicsDiagnostics contract. input_wrench_used is true.
// allocation_invoked stays false.
//
// Order, first defect wins: ValidateEvaluationInputs, relative velocity,
// rigid-body Coriolis, added-mass Coriolis, damping, restoring, then a
// finite check of the products and sums. A failed helper returns that
// status unchanged. Every failure writes finite zeros and does not echo
// a non-finite time step. The same inputs produce the same outputs.
//
// Evaluate is const and does not mutate the stored model. Concurrent
// calls on one instance do not share mutable state.
inline constexpr std::string_view kMarineForceModelId = "marine_force";

class MarineForceDynamics final : public VehicleDynamics {
 public:
  explicit MarineForceDynamics(parameters::MarineModel model);

  [[nodiscard]] StatusOr<DynamicsResult> Evaluate(
      const VehicleStateRt& state, const BodyWrenchRt& wrench,
      const EnvironmentRt& environment, Duration dt) const override;

 private:
  parameters::MarineModel model_;
};

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_DYNAMICS_H_
