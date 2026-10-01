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
// Deterministic UUV motion-primitive generator. A primitive is a constant
// body-frame wrench (force in N, torque in N*m) held for a fixed duration.
// This is a pure function: no dynamics integration, no search, no collision
// checking, no World access.

enum class PrimitiveSetError {
  kOk = 0,
  kBadConfig = 1,
  kEmpty = 2,
};

enum class PrimitiveGenerationMode {
  // Zero wrench + +/-max on each of the 6 body axes alone -> 13 primitives.
  kAxisAligned = 0,
  // Use config.custom_controls as the control list (still validated).
  kCustom = 1,
};

struct VehicleMotionPrimitiveConfig {
  PrimitiveGenerationMode mode = PrimitiveGenerationMode::kAxisAligned;

  // Inclusive magnitude limits for body-frame wrench (force N, torque N*m).
  // Axis-aligned generation uses these magnitudes; every emitted control must
  // satisfy |component| <= corresponding max (and all finite, max >= 0).
  ::intrinsic::vehicle::BodyVector max_force_torque{};

  // Shared duration for every generated primitive. Must be finite and > 0.
  double duration_s = 0;

  // Used only when mode == kCustom. May be empty -> kEmpty after validation.
  // Each entry must be finite and within +/-max_force_torque per component.
  std::vector<::intrinsic::vehicle::BodyVector> custom_controls;
};

struct VehicleMotionPrimitive {
  // Deterministic id: "uuv-prim-" + zero-padded index ("uuv-prim-000", ...)
  // in generation order. Same config -> same ids and same control vectors.
  std::string id;
  ::intrinsic::vehicle::BodyVector control;  // body wrench (force/torque)
  double duration_s = 0;
};

struct PrimitiveSetResult {
  PrimitiveSetError error = PrimitiveSetError::kBadConfig;
  std::vector<VehicleMotionPrimitive> primitives;  // empty unless kOk
};

// Checks, first defect wins:
//   1. a max_force_torque component is non-finite or negative -> kBadConfig.
//   2. duration_s is non-finite or <= 0                       -> kBadConfig.
//   3. kCustom and a control is non-finite or has
//      |component| > max on any axis                          -> kBadConfig.
//      Illegal controls are rejected, never clamped.
//   4. kCustom with no controls                               -> kEmpty.
//
// kAxisAligned order (13 entries): zero; +/-max.linear_x; +/-max.linear_y;
// +/-max.linear_z; +/-max.angular_x; +/-max.angular_y; +/-max.angular_z.
// Within each pair the positive control comes first. Other components are 0.
// A zero bound still emits both entries of its pair, as all-zero controls.
// kCustom preserves custom_controls order. Ids are uuv-prim-000... in order.
PrimitiveSetResult GenerateUuvMotionPrimitives(
    const VehicleMotionPrimitiveConfig& config);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_MOTION_PRIMITIVES_H_
