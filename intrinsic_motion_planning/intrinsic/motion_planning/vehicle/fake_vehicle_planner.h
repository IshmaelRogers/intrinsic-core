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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_FAKE_VEHICLE_PLANNER_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_FAKE_VEHICLE_PLANNER_H_

#include <memory>
#include <string_view>

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"

namespace intrinsic::motion_planning::vehicle {

// Id stamped when a success request leaves trajectory_id empty.
inline constexpr std::string_view kFakeVehicleTrajectoryId = "fake-trajectory";

// Deterministic stand-in. No sleep, threads, randomness, World access, or
// StateSpace search. A kOk result is the fixed two-sample world_enu fixture
// from the vehicle trajectory examples.
struct FakeVehiclePlannerConfig {
  VehiclePlanStatus status = VehiclePlanStatus::kOk;
  // When start_label equals goal_label and this is false, Plan still
  // returns kOk with the two-sample fixture. When true, that case is
  // kNoSolution and the trajectory is absent.
  bool fail_when_start_equals_goal = false;
};

class FakeVehiclePlanner : public VehiclePlanner {
 public:
  explicit FakeVehiclePlanner(FakeVehiclePlannerConfig config = {});

  std::string_view id() const override;
  VehiclePlanResult Plan(const VehiclePlanRequest& request) const override;

 private:
  FakeVehiclePlannerConfig config_;
};

std::shared_ptr<FakeVehiclePlanner> MakeFakeVehiclePlanner(
    FakeVehiclePlannerConfig config = {});

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_FAKE_VEHICLE_PLANNER_H_
