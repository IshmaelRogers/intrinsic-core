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

#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsResult;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::FrameId;
using ::intrinsic::vehicle::dynamics::StatusOr;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;

constexpr int kSpatialDof = 6;

PropagationResult Fail(PropagationError error) {
  PropagationResult result;
  result.error = error;
  return result;
}

bool FinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool ConfigOk(const PropagationConfig& config) {
  if (!FinitePositive(config.dt_s)) {
    return false;
  }
  if (config.max_steps < 1) {
    return false;
  }
  for (double mass : config.mass_diag) {
    if (!FinitePositive(mass)) {
      return false;
    }
  }
  if (!std::isfinite(config.gravity_m_s2) || config.gravity_m_s2 < 0.0) {
    return false;
  }
  if (!std::isfinite(config.fluid_density_kg_m3) ||
      config.fluid_density_kg_m3 < 0.0) {
    return false;
  }
  for (double current : config.current_world_enu_m_s) {
    if (!std::isfinite(current)) {
      return false;
    }
  }
  return true;
}

bool StartOk(const VehiclePlanningState& start) {
  if (!embodiment::IsFinite(start.position) ||
      !embodiment::IsFinite(start.orientation) ||
      !::intrinsic::vehicle::IsFinite(start.twist)) {
    return false;
  }
  return embodiment::IsNormalized(start.orientation, kQuaternionNormTolerance);
}

bool PrimitiveOk(const VehicleMotionPrimitive& primitive) {
  if (!std::isfinite(primitive.duration_s) || primitive.duration_s <= 0.0) {
    return false;
  }
  return ::intrinsic::vehicle::IsFinite(primitive.control);
}

// n = floor(duration/dt), rem = duration - n*dt,
// required = n + (rem > 0 ? 1 : 0). A non-finite or overflowing count is
// over budget.
bool StepCount(double duration_s, double dt_s, int max_steps, int* required) {
  const double ratio = duration_s / dt_s;
  if (!std::isfinite(ratio) || ratio < 0.0) {
    return false;
  }
  const double n_full = std::floor(ratio);
  if (n_full > static_cast<double>(std::numeric_limits<int>::max())) {
    return false;
  }
  const int n = static_cast<int>(n_full);
  const double rem = duration_s - static_cast<double>(n) * dt_s;
  int steps = n;
  if (rem > 0.0) {
    if (n == std::numeric_limits<int>::max()) {
      return false;
    }
    steps = n + 1;
  }
  *required = steps;
  return steps <= max_steps;
}

VehicleStateRt ToStateRt(const VehiclePlanningState& state) {
  VehicleStateRt rt;
  rt.pose_frame = FrameId::kWorldEnu;
  rt.position_m = {state.position.x, state.position.y, state.position.z};
  rt.orientation_xyzw = {state.orientation.x, state.orientation.y,
                         state.orientation.z, state.orientation.w};
  rt.body_twist = {state.twist.linear_x,  state.twist.linear_y,
                   state.twist.linear_z,  state.twist.angular_x,
                   state.twist.angular_y, state.twist.angular_z};
  return rt;
}

BodyWrenchRt ToWrench(const VehicleMotionPrimitive& primitive) {
  BodyWrenchRt wrench;
  wrench.frame = FrameId::kBody;
  wrench.force_n = {primitive.control.linear_x, primitive.control.linear_y,
                    primitive.control.linear_z};
  wrench.torque_n_m = {primitive.control.angular_x, primitive.control.angular_y,
                       primitive.control.angular_z};
  return wrench;
}

EnvironmentRt ToEnvironment(const PropagationConfig& config) {
  EnvironmentRt environment;
  environment.gravity_m_s2 = config.gravity_m_s2;
  environment.fluid_density_kg_m3 = config.fluid_density_kg_m3;
  environment.current_velocity_m_s = config.current_world_enu_m_s;
  environment.current_frame = FrameId::kWorldEnu;
  return environment;
}

std::array<double, kSpatialDof> PlanningWrench(const DynamicsResult& evaluated,
                                               const BodyWrenchRt& wrench) {
  if (evaluated.diagnostics.input_wrench_used) {
    return evaluated.diagnostics.total_wrench;
  }
  return {wrench.force_n[0],    wrench.force_n[1],    wrench.force_n[2],
          wrench.torque_n_m[0], wrench.torque_n_m[1], wrench.torque_n_m[2]};
}

::intrinsic::vehicle::BodyVector AddTwist(
    ::intrinsic::vehicle::BodyVector twist, double dt_i,
    const std::array<double, kSpatialDof>& wrench,
    const std::array<double, kSpatialDof>& mass_diag) {
  const double accel[kSpatialDof] = {
      wrench[0] / mass_diag[0], wrench[1] / mass_diag[1],
      wrench[2] / mass_diag[2], wrench[3] / mass_diag[3],
      wrench[4] / mass_diag[4], wrench[5] / mass_diag[5]};
  twist.linear_x += dt_i * accel[0];
  twist.linear_y += dt_i * accel[1];
  twist.linear_z += dt_i * accel[2];
  twist.angular_x += dt_i * accel[3];
  twist.angular_y += dt_i * accel[4];
  twist.angular_z += dt_i * accel[5];
  return twist;
}

bool Renormalize(embodiment::Quaternion* orientation) {
  const double norm_sq =
      (orientation->x * orientation->x) + (orientation->y * orientation->y) +
      (orientation->z * orientation->z) + (orientation->w * orientation->w);
  const double norm = std::sqrt(norm_sq);
  // A non-finite or zero norm cannot produce a finite unit quaternion.
  // Dividing a large finite value by an overflowed norm yields zeros.
  if (!std::isfinite(norm) || !(norm > 0.0)) {
    return false;
  }
  orientation->x /= norm;
  orientation->y /= norm;
  orientation->z /= norm;
  orientation->w /= norm;
  return embodiment::IsFinite(*orientation);
}

}  // namespace

PropagationResult PropagateUuvMotionPrimitive(
    const VehiclePlanningState& start, const VehicleMotionPrimitive& primitive,
    const PropagationConfig& config,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics& dynamics) {
  if (!ConfigOk(config)) {
    return Fail(PropagationError::kBadConfig);
  }
  if (!StartOk(start)) {
    return Fail(PropagationError::kBadStart);
  }
  if (!PrimitiveOk(primitive)) {
    return Fail(PropagationError::kBadPrimitive);
  }
  int required = 0;
  if (!StepCount(primitive.duration_s, config.dt_s, config.max_steps,
                 &required)) {
    return Fail(PropagationError::kStepBudget);
  }

  const BodyWrenchRt wrench = ToWrench(primitive);
  const EnvironmentRt environment = ToEnvironment(config);

  PropagationResult result;
  result.error = PropagationError::kOk;
  result.samples.push_back(PropagationSample{0.0, start});

  VehiclePlanningState state = start;
  double elapsed = 0.0;
  int steps_taken = 0;
  while (elapsed < primitive.duration_s) {
    if (steps_taken >= config.max_steps) {
      return Fail(PropagationError::kStepBudget);
    }
    const double dt_i = std::min(config.dt_s, primitive.duration_s - elapsed);
    const VehicleStateRt state_rt = ToStateRt(state);
    const StatusOr<DynamicsResult> first =
        dynamics.Evaluate(state_rt, wrench, environment, Duration{dt_i});
    if (!first.ok()) {
      return Fail(PropagationError::kDynamicsFailed);
    }

    const std::array<double, kSpatialDof> planning_wrench =
        PlanningWrench(first.value(), wrench);
    VehiclePlanningState state_mid = state;
    state_mid.twist =
        AddTwist(state.twist, dt_i, planning_wrench, config.mass_diag);

    const StatusOr<DynamicsResult> second = dynamics.Evaluate(
        ToStateRt(state_mid), wrench, environment, Duration{dt_i});
    if (!second.ok()) {
      return Fail(PropagationError::kDynamicsFailed);
    }

    const auto& position_dot = second.value().derivative.position_dot_m_s;
    const auto& orientation_dot =
        second.value().derivative.orientation_dot_xyzw;
    state.position.x +=
        dt_i * (position_dot[0] + config.current_world_enu_m_s[0]);
    state.position.y +=
        dt_i * (position_dot[1] + config.current_world_enu_m_s[1]);
    state.position.z +=
        dt_i * (position_dot[2] + config.current_world_enu_m_s[2]);
    state.orientation.x += dt_i * orientation_dot[0];
    state.orientation.y += dt_i * orientation_dot[1];
    state.orientation.z += dt_i * orientation_dot[2];
    state.orientation.w += dt_i * orientation_dot[3];
    if (!Renormalize(&state.orientation)) {
      return Fail(PropagationError::kDynamicsFailed);
    }
    state.twist = state_mid.twist;
    elapsed += dt_i;
    ++steps_taken;
    result.samples.push_back(PropagationSample{elapsed, state});
  }
  return result;
}

}  // namespace intrinsic::motion_planning::vehicle
