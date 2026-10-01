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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PRIMITIVE_PROPAGATION_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PRIMITIVE_PROPAGATION_H_

#include <array>
#include <vector>

#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Current-aware propagation of one UUV motion primitive. Planning integrates
// with semi-implicit Euler and a diagonal mass surrogate. VehicleDynamics
// is injected and is not reimplemented here. There is no search, collision
// checking, or World access.

enum class PropagationError {
  kOk = 0,
  kBadConfig = 1,
  kBadStart = 2,
  kBadPrimitive = 3,
  kDynamicsFailed = 4,
  kStepBudget = 5,
};

struct PropagationConfig {
  // Substep size. Must be finite and > 0.
  double dt_s = 0;
  // Hard cap on integration substeps (including a possible remainder step).
  // Must be >= 1. If the required step count exceeds max_steps, the result
  // is kStepBudget and samples stay empty.
  int max_steps = 0;
  // Planning diagonal mass surrogate (force to linear accel, torque to
  // angular accel). All six entries must be finite and > 0. Order: surge
  // through yaw. This is not a full mass-matrix inverse.
  std::array<double, 6> mass_diag{{1, 1, 1, 1, 1, 1}};

  // Environment passed to Evaluate. gravity >= 0 and finite; density >= 0
  // and finite. Zero gravity and density are allowed (ZeroForce path).
  double gravity_m_s2 = 0;
  double fluid_density_kg_m3 = 0;
  // World-ENU linear current (m/s). Added to position_dot after the
  // kinematic map. Angular current is always zero in this leaf.
  std::array<double, 3> current_world_enu_m_s{{0, 0, 0}};
};

struct PropagationSample {
  double time_s = 0;  // monotonic, starts at 0
  VehiclePlanningState state;
};

struct PropagationResult {
  PropagationError error = PropagationError::kBadConfig;
  // Includes the start sample at t = 0 when kOk. Empty unless kOk.
  std::vector<PropagationSample> samples;
};

// Pure adapter. `dynamics` is injected (tests use ZeroForceDynamics).
// Does not own or register a planner. No search, collision, or World.
//
// Check order, first defect wins: config, start, primitive, step budget,
// then dynamics. A failed Evaluate, or a quaternion renormalize that is
// non-finite, returns kDynamicsFailed and clears samples.
PropagationResult PropagateUuvMotionPrimitive(
    const VehiclePlanningState& start, const VehicleMotionPrimitive& primitive,
    const PropagationConfig& config,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics& dynamics);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PRIMITIVE_PROPAGATION_H_
