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

#include "intrinsic/vehicle/dynamics/marine_force_dynamics.h"

#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "intrinsic/vehicle/dynamics/added_mass_coriolis_matrix.h"
#include "intrinsic/vehicle/dynamics/gravity_buoyancy_restoring_wrench.h"
#include "intrinsic/vehicle/dynamics/linear_quadratic_damping_wrench.h"
#include "intrinsic/vehicle/dynamics/rigid_body_coriolis_matrix.h"
#include "intrinsic/vehicle/dynamics/water_current_relative_velocity.h"

namespace intrinsic::vehicle::dynamics {
namespace {

constexpr std::string_view kComposedWrenchNonFiniteMessage =
    "composed wrench is not finite";

using Wrench = std::array<double, kSpatialDof>;

// -0 compares equal to 0 and is a different bit pattern. Stored zeros use
// +0 so a cancelling component is the zero component.
double PositiveZero(double value) { return value == 0.0 ? 0.0 : value; }

bool FiniteWrench(const Wrench& wrench) {
  for (double component : wrench) {
    if (!std::isfinite(component)) {
      return false;
    }
  }
  return true;
}

Wrench Multiply(const std::array<double, kSpatialDof * kSpatialDof>& matrix,
                const Wrench& vector) {
  Wrench product = {};
  for (int row = 0; row < kSpatialDof; ++row) {
    double sum = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      sum += matrix[row * kSpatialDof + col] * vector[col];
    }
    product[row] = PositiveZero(sum);
  }
  return product;
}

Wrench Negate(const Wrench& value) {
  Wrench negated = {};
  for (int index = 0; index < kSpatialDof; ++index) {
    negated[index] = PositiveZero(-value[index]);
  }
  return negated;
}

void AddInto(Wrench& sum, const Wrench& term) {
  for (int index = 0; index < kSpatialDof; ++index) {
    sum[index] = PositiveZero(sum[index] + term[index]);
  }
}

std::string_view CurrentFrameId(FrameId frame) {
  if (frame == FrameId::kWorldEnu) {
    return parameters::kWorldEnuFrameId;
  }
  if (frame == FrameId::kWorldNed) {
    return parameters::kWorldNedFrameId;
  }
  if (frame == FrameId::kBody) {
    return parameters::kBodyFrameId;
  }
  return {};
}

parameters::Environment RuntimeEnvironment(const EnvironmentRt& environment) {
  parameters::Environment mapped;
  mapped.gravity_m_s2 = environment.gravity_m_s2;
  mapped.fluid_density_kg_m3 = environment.fluid_density_kg_m3;
  mapped.current_velocity_m_s = {
      environment.current_velocity_m_s[kX],
      environment.current_velocity_m_s[kY],
      environment.current_velocity_m_s[kZ],
  };
  mapped.current_frame_id =
      std::string(CurrentFrameId(environment.current_frame));
  return mapped;
}

Wrench InputWrench(const BodyWrenchRt& wrench) {
  return {
      wrench.force_n[kX],    wrench.force_n[kY],    wrench.force_n[kZ],
      wrench.torque_n_m[kX], wrench.torque_n_m[kY], wrench.torque_n_m[kZ],
  };
}

// Same active map as zero_force_dynamics.cc. Body velocity rotates into
// pose_frame. This is the kinematic pose rate, not an integration step.
std::array<double, 3> RotateBodyToPose(const std::array<double, 4>& quaternion,
                                       const std::array<double, 3>& body) {
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
      r00 * body[kX] + r01 * body[kY] + r02 * body[kZ],
      r10 * body[kX] + r11 * body[kY] + r12 * body[kZ],
      r20 * body[kX] + r21 * body[kY] + r22 * body[kZ],
  };
}

std::array<double, 4> QuaternionDerivative(
    const std::array<double, 4>& quaternion, const Wrench& twist) {
  const double x = quaternion[kQuatX];
  const double y = quaternion[kQuatY];
  const double z = quaternion[kQuatZ];
  const double w = quaternion[kQuatW];
  const double wx = twist[kRoll];
  const double wy = twist[kPitch];
  const double wz = twist[kYaw];
  // 0.5 * q ⊗ (wx, wy, wz, 0), Hamilton product, stored x, y, z, w.
  return {
      0.5 * (w * wx + y * wz - z * wy),
      0.5 * (w * wy - x * wz + z * wx),
      0.5 * (w * wz + x * wy - y * wx),
      0.5 * (-x * wx - y * wy - z * wz),
  };
}

DynamicsResult NamedZero() {
  DynamicsResult result;
  result.diagnostics.model_id = kMarineForceModelId;
  return result;
}

StatusOr<DynamicsResult> Reject(DynamicsStatus status) {
  return StatusOr<DynamicsResult>::Failure(status, NamedZero());
}

}  // namespace

MarineForceDynamics::MarineForceDynamics(parameters::MarineModel model)
    : model_(std::move(model)) {}

StatusOr<DynamicsResult> MarineForceDynamics::Evaluate(
    const VehicleStateRt& state, const BodyWrenchRt& wrench,
    const EnvironmentRt& environment, Duration dt) const {
  const DynamicsStatus inputs =
      ValidateEvaluationInputs(state, wrench, environment, dt);
  if (!inputs.ok()) {
    return Reject(inputs);
  }

  const parameters::Environment runtime = RuntimeEnvironment(environment);
  const StatusOr<CurrentRelativeTwist> relative =
      ComputeWaterCurrentRelativeVelocity(
          runtime, state.pose_frame, state.orientation_xyzw, state.body_twist);
  if (!relative.ok()) {
    return Reject(relative.status());
  }

  const StatusOr<RigidBodyCoriolisMatrix> rigid_body_coriolis =
      ComputeRigidBodyCoriolisMatrix(model_.mass_inertia,
                                     model_.centers.center_of_gravity_m,
                                     state.body_twist);
  if (!rigid_body_coriolis.ok()) {
    return Reject(rigid_body_coriolis.status());
  }

  const StatusOr<AddedMassCoriolisMatrix> added_mass_coriolis =
      ComputeAddedMassCoriolisMatrix(model_.added_mass,
                                     relative.value().components);
  if (!added_mass_coriolis.ok()) {
    return Reject(added_mass_coriolis.status());
  }

  const StatusOr<DampingWrench> damping = ComputeLinearQuadraticDampingWrench(
      model_.damping, relative.value().components);
  if (!damping.ok()) {
    return Reject(damping.status());
  }

  const StatusOr<RestoringWrench> restoring =
      ComputeGravityBuoyancyRestoringWrench(
          model_.mass_inertia, model_.buoyancy, model_.centers, runtime,
          state.pose_frame, state.orientation_xyzw);
  if (!restoring.ok()) {
    return Reject(restoring.status());
  }

  const Wrench rigid_body_wrench = Negate(
      Multiply(rigid_body_coriolis.value().coefficients, state.body_twist));
  const Wrench added_mass_wrench = Negate(Multiply(
      added_mass_coriolis.value().coefficients, relative.value().components));
  Wrench hydrodynamic = {};
  AddInto(hydrodynamic, rigid_body_wrench);
  AddInto(hydrodynamic, added_mass_wrench);
  AddInto(hydrodynamic, damping.value().components);
  AddInto(hydrodynamic, restoring.value().components);
  Wrench total = hydrodynamic;
  AddInto(total, InputWrench(wrench));

  if (!FiniteWrench(relative.value().components) ||
      !FiniteWrench(rigid_body_wrench) || !FiniteWrench(added_mass_wrench) ||
      !FiniteWrench(damping.value().components) ||
      !FiniteWrench(restoring.value().components) ||
      !FiniteWrench(hydrodynamic) || !FiniteWrench(total)) {
    return Reject(
        DynamicsStatus::InvalidArgument(kComposedWrenchNonFiniteMessage));
  }

  DynamicsResult result = NamedZero();
  result.diagnostics.dt_s = dt.seconds;
  result.diagnostics.input_wrench_used = true;
  result.diagnostics.relative_twist = relative.value().components;
  result.diagnostics.rigid_body_coriolis_wrench = rigid_body_wrench;
  result.diagnostics.added_mass_coriolis_wrench = added_mass_wrench;
  result.diagnostics.damping_wrench = damping.value().components;
  result.diagnostics.restoring_wrench = restoring.value().components;
  result.diagnostics.hydrodynamic_wrench = hydrodynamic;
  result.diagnostics.total_wrench = total;
  result.diagnostics.model_force_n = {
      hydrodynamic[kSurge],
      hydrodynamic[kSway],
      hydrodynamic[kHeave],
  };
  result.diagnostics.model_torque_n_m = {
      hydrodynamic[kRoll],
      hydrodynamic[kPitch],
      hydrodynamic[kYaw],
  };
  result.derivative.position_dot_m_s =
      RotateBodyToPose(state.orientation_xyzw,
                       {state.body_twist[kSurge], state.body_twist[kSway],
                        state.body_twist[kHeave]});
  result.derivative.orientation_dot_xyzw =
      QuaternionDerivative(state.orientation_xyzw, state.body_twist);
  return StatusOr<DynamicsResult>::Ok(result);
}

}  // namespace intrinsic::vehicle::dynamics
