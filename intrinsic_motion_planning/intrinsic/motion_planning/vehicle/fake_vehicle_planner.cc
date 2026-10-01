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

#include "intrinsic/motion_planning/vehicle/fake_vehicle_planner.h"

#include <cstdint>
#include <string>
#include <utility>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::BodyVector;
using ::intrinsic::vehicle::TrajectorySampleView;

// Shape of vehicle_trajectory_two_sample.textproto. Times increase, poses
// are finite, and the frame is world_enu. The second sample has no yaw rate.
TrajectorySampleView FixtureSample(int64_t seconds, int32_t nanos, double x,
                                   double angular_z) {
  TrajectorySampleView sample;
  sample.time_present = true;
  sample.seconds = seconds;
  sample.nanos = nanos;
  sample.position = embodiment::Vec3{x, 2.0, -3.0};
  sample.orientation = embodiment::Quaternion{0.0, 0.0, 0.0, 1.0};
  sample.twist_present = true;
  sample.twist = BodyVector{0.5, 0.0, 0.0, 0.0, 0.0, angular_z};
  return sample;
}

VehiclePlanResult FailureResult(VehiclePlanStatus status) {
  VehiclePlanResult result;
  result.status = status;
  return result;
}

VehiclePlanResult SuccessResult(const VehiclePlanRequest& request) {
  VehiclePlanResult result;
  result.status = VehiclePlanStatus::kOk;
  result.header_present = true;
  result.validity_present = true;
  result.validity_state = 1;
  result.frame_id = std::string(embodiment::kWorldEnuFrameId);
  if (request.trajectory_id.empty()) {
    result.trajectory_id = std::string(kFakeVehicleTrajectoryId);
  } else {
    result.trajectory_id = request.trajectory_id;
  }
  result.samples.push_back(FixtureSample(1700000010, 0, 1.0, 0.125));
  result.samples.push_back(FixtureSample(1700000011, 500000000, 1.5, 0.0));
  result.provenance_present = true;
  result.model_id = "uuv_planner";
  return result;
}

}  // namespace

FakeVehiclePlanner::FakeVehiclePlanner(FakeVehiclePlannerConfig config)
    : config_(std::move(config)) {}

std::string_view FakeVehiclePlanner::id() const { return kVehiclePlannerFake; }

VehiclePlanResult FakeVehiclePlanner::Plan(
    const VehiclePlanRequest& request) const {
  if (request.start_label.empty() || request.goal_label.empty()) {
    return FailureResult(VehiclePlanStatus::kInvalidRequest);
  }
  if (config_.fail_when_start_equals_goal &&
      request.start_label == request.goal_label) {
    return FailureResult(VehiclePlanStatus::kNoSolution);
  }
  if (config_.status != VehiclePlanStatus::kOk) {
    return FailureResult(config_.status);
  }
  return SuccessResult(request);
}

std::shared_ptr<FakeVehiclePlanner> MakeFakeVehiclePlanner(
    FakeVehiclePlannerConfig config) {
  return std::make_shared<FakeVehiclePlanner>(std::move(config));
}

}  // namespace intrinsic::motion_planning::vehicle
