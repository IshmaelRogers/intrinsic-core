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

#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"

#include <array>

namespace intrinsic::vehicle::dynamics {
namespace {

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
    const std::array<double, 4>& quaternion,
    const std::array<double, kSpatialDof>& twist) {
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

std::array<double, 3> LinearTwist(
    const std::array<double, kSpatialDof>& twist) {
  return {twist[kSurge], twist[kSway], twist[kHeave]};
}

DynamicsResult NamedZero() {
  DynamicsResult result;
  result.diagnostics.model_id = kZeroForceModelId;
  return result;
}

}  // namespace

StatusOr<DynamicsResult> ZeroForceDynamics::Evaluate(
    const VehicleStateRt& state, const BodyWrenchRt& wrench,
    const EnvironmentRt& environment, Duration dt) const {
  const DynamicsStatus status =
      ValidateEvaluationInputs(state, wrench, environment, dt);
  if (!status.ok()) {
    return StatusOr<DynamicsResult>::Failure(status, NamedZero());
  }
  DynamicsResult result = NamedZero();
  result.diagnostics.dt_s = dt.seconds;
  result.derivative.position_dot_m_s =
      RotateBodyToPose(state.orientation_xyzw, LinearTwist(state.body_twist));
  result.derivative.orientation_dot_xyzw =
      QuaternionDerivative(state.orientation_xyzw, state.body_twist);
  return StatusOr<DynamicsResult>::Ok(result);
}

}  // namespace intrinsic::vehicle::dynamics
