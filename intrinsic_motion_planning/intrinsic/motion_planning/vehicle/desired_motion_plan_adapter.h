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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_DESIRED_MOTION_PLAN_ADAPTER_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_DESIRED_MOTION_PLAN_ADAPTER_H_

#include <string_view>

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Pure adapter from a DesiredMotionView pose intent plus a caller-supplied
// start state to a VehiclePlanRequest. There is no planner call, registry,
// World read, clock, or I/O here. Doubles are copied verbatim: no quaternion
// sign canonicalisation, no renormalisation, and -0.0 is preserved. There is
// no ENU and NED conversion.

enum class DesiredMotionPlanAdapterStatus {
  kOk = 0,
  // AssessDesiredMotion(motion).accepted == false.
  kInvalidMotion = 1,
  // ObjectiveKind::kTwist or kTrajectory.
  kUnsupportedObjective = 2,
  // Pose frame_id is not "world_enu".
  kUnsupportedFrame = 3,
  // Validate(start, VehicleStateBounds{}) != StateSpaceError::kOk.
  kInvalidStart = 4,
};

struct DesiredMotionPlanAdapterResult {
  DesiredMotionPlanAdapterStatus status =
      DesiredMotionPlanAdapterStatus::kInvalidMotion;
  // Default-constructed unless status == kOk.
  VehiclePlanRequest request;
};

// Check order: invalid motion, unsupported objective, unsupported frame,
// invalid start. The first defect wins.
//
// On kOk the request has empty labels, states_present == true, start_state
// equal to `start`, and a goal state taken from the motion pose with an
// all-zero twist (a pose target is a rest goal). confidence, horizon,
// provenance, and labels are not mapped. An empty `trajectory_id` is copied
// as is, so the planner chooses its own id.
DesiredMotionPlanAdapterResult AdaptDesiredMotionToPlanRequest(
    const ::intrinsic::vehicle::DesiredMotionView& motion,
    const VehiclePlanningState& start, std::string_view trajectory_id);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_DESIRED_MOTION_PLAN_ADAPTER_H_
