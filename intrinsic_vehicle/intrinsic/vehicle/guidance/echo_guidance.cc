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

#include "intrinsic/vehicle/guidance/echo_guidance.h"

#include <string_view>

namespace intrinsic::vehicle::guidance {
namespace {

constexpr std::string_view kTrajectoryNotSampledMessage =
    "trajectory objective is not sampled";

}  // namespace

StatusOr<MotionReferenceRt> EchoGuidance::Evaluate(
    const DesiredMotionRt& intent, const VehicleStateRt* state,
    Duration update_period) const {
  const GuidanceStatus status =
      ValidateGuidanceInputs(intent, state, update_period);
  if (!status.ok()) {
    return StatusOr<MotionReferenceRt>::Failure(status);
  }
  if (intent.objective == ObjectiveKind::kTrajectory) {
    return StatusOr<MotionReferenceRt>::Failure(
        GuidanceStatus::Invalid(kTrajectoryNotSampledMessage));
  }
  MotionReferenceRt reference{};
  reference.snapshot_id = intent.snapshot_id;
  reference.update_period = update_period;
  if (intent.objective == ObjectiveKind::kPose) {
    reference.has_pose = true;
    reference.pose_frame = intent.pose_frame;
    reference.position_m = intent.position_m;
    reference.orientation_xyzw = intent.orientation_xyzw;
    return StatusOr<MotionReferenceRt>::Ok(reference);
  }
  reference.has_twist = true;
  reference.twist_frame = intent.twist_frame;
  reference.body_twist = intent.body_twist;
  return StatusOr<MotionReferenceRt>::Ok(reference);
}

}  // namespace intrinsic::vehicle::guidance
