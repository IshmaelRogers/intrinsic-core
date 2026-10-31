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

#include "intrinsic/motion_planning/vehicle/desired_motion_plan_adapter.h"

#include <string>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {

DesiredMotionPlanAdapterResult AdaptDesiredMotionToPlanRequest(
    const ::intrinsic::vehicle::DesiredMotionView& motion,
    const VehiclePlanningState& start, std::string_view trajectory_id) {
  using ::intrinsic::vehicle::ObjectiveKind;

  DesiredMotionPlanAdapterResult result;
  if (!::intrinsic::vehicle::AssessDesiredMotion(motion).accepted) {
    result.status = DesiredMotionPlanAdapterStatus::kInvalidMotion;
    return result;
  }
  if (motion.objective != ObjectiveKind::kPose) {
    result.status = DesiredMotionPlanAdapterStatus::kUnsupportedObjective;
    return result;
  }
  if (motion.frame_id != ::intrinsic::embodiment::kWorldEnuFrameId) {
    result.status = DesiredMotionPlanAdapterStatus::kUnsupportedFrame;
    return result;
  }
  if (Validate(start, VehicleStateBounds{}) != StateSpaceError::kOk) {
    result.status = DesiredMotionPlanAdapterStatus::kInvalidStart;
    return result;
  }

  result.status = DesiredMotionPlanAdapterStatus::kOk;
  result.request.trajectory_id = std::string(trajectory_id);
  result.request.states_present = true;
  result.request.start_state = start;
  result.request.goal_state.position = motion.position;
  result.request.goal_state.orientation = motion.orientation;
  result.request.goal_state.twist = BodyVector{};
  return result;
}

}  // namespace intrinsic::motion_planning::vehicle
