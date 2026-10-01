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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_MOTION_PRIMITIVES_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_MOTION_PRIMITIVES_H_

#include <string>
#include <vector>

#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Config-driven body-frame twist primitives for a UUV. This is a pure
// function of the config: no search, sampling, collision, World, dynamics
// integration, or planner registration.

// Linear x, y, z (m/s) then angular x, y, z (rad/s). Same layout as
// VehiclePlanningState::twist. Not a wrench.
using BodyVector = ::intrinsic::vehicle::BodyVector;

// Body-frame twist axis. The numeric value is the BodyVector component index.
enum class MotionPrimitiveAxis {
  kSurge = 0,  // linear x
  kSway = 1,   // linear y
  kHeave = 2,  // linear z
  kRoll = 3,   // angular x
  kPitch = 4,  // angular y
  kYaw = 5,    // angular z
};

// One bounded control held for a positive duration. Unused axes are zero.
struct VehicleMotionPrimitive {
  std::string id;         // Stable, deterministic label.
  BodyVector control;     // 6 doubles; zeros on unused axes.
  double duration_s = 0;  // > 0 and finite when the set is valid.
};

// Speed limits applied to every emitted control. An absent limit does not
// constrain that group. A present limit must be finite and >= 0.
struct VehicleMotionPrimitiveBounds {
  bool max_linear_speed_present = false;
  double max_linear_speed_m_s = 0;  // Applies to ||v_lin||.
  bool max_angular_speed_present = false;
  double max_angular_speed_rad_s = 0;  // Applies to ||w||.
};

// Generator input. An axis is emitted only when its *_present flag is true
// and its level is finite and > 0. A present flag with any other level is
// kBadConfig.
struct VehicleMotionPrimitiveSetConfig {
  double duration_s = 1.0;  // Shared by every emitted primitive.

  // When true, emit a zero-control "hover" primitive first.
  bool include_hover = true;

  bool surge_present = false;
  double surge_m_s = 0;
  bool sway_present = false;
  double sway_m_s = 0;
  bool heave_present = false;
  double heave_m_s = 0;
  bool roll_present = false;
  double roll_rad_s = 0;
  bool pitch_present = false;
  double pitch_rad_s = 0;
  bool yaw_present = false;
  double yaw_rad_s = 0;

  VehicleMotionPrimitiveBounds bounds;
};

enum class MotionPrimitiveSetError {
  kOk = 0,
  kBadConfig = 1,        // Duration, bounds, or levels are invalid.
  kBoundsViolation = 2,  // A would-be primitive fails an engaged bound.
};

struct GenerateMotionPrimitivesResult {
  MotionPrimitiveSetError error = MotionPrimitiveSetError::kOk;
  // Ordered and deterministic. Empty is legal when include_hover is false,
  // no axis is present, and the config is otherwise valid. Empty on every
  // non-ok error; a bounds failure does not return a partial set.
  std::vector<VehicleMotionPrimitive> primitives;
};

// Pure function. Check order, first defect wins, is kBadConfig:
//
//   1. duration_s non-finite or <= 0.
//   2. A present speed limit is non-finite or < 0. Linear, then angular.
//      Disengaged limit fields are ignored.
//   3. A *_present axis level is non-finite or <= 0, in axis order
//      surge, sway, heave, roll, pitch, yaw.
//
// A valid config emits, in order: hover (if requested), then for each
// engaged axis the +level primitive ("<axis>_pos") and the -level primitive
// ("<axis>_neg"). Other components stay zero. Every primitive uses
// duration_s.
//
// Every emitted control, including hover, must satisfy the engaged bounds
// (||v_lin|| <= max linear, ||w|| <= max angular, inclusive). If any fails,
// the result is kBoundsViolation and primitives is empty.
GenerateMotionPrimitivesResult GenerateMotionPrimitives(
    const VehicleMotionPrimitiveSetConfig& config);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_MOTION_PRIMITIVES_H_
