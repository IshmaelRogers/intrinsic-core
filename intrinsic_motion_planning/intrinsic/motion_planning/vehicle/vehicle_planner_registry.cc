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

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"

#include <string>
#include <utility>

namespace intrinsic::motion_planning::vehicle {

PlannerRegistryError VehiclePlannerRegistry::Register(
    std::shared_ptr<VehiclePlanner> planner) {
  if (planner == nullptr) {
    return PlannerRegistryError::kNullPlanner;
  }
  // Copy the id before moving the planner. id() may alias planner storage.
  const std::string planner_id(planner->id());
  if (planner_id.empty()) {
    return PlannerRegistryError::kEmptyId;
  }
  if (planners_.contains(planner_id)) {
    return PlannerRegistryError::kDuplicateId;
  }
  planners_.emplace(planner_id, std::move(planner));
  return PlannerRegistryError::kOk;
}

PlannerRegistryError VehiclePlannerRegistry::Lookup(
    std::string_view planner_id, std::shared_ptr<VehiclePlanner>* out) const {
  if (planner_id.empty()) {
    return PlannerRegistryError::kEmptyId;
  }
  const auto found = planners_.find(planner_id);
  if (found == planners_.end()) {
    return PlannerRegistryError::kNotFound;
  }
  *out = found->second;
  return PlannerRegistryError::kOk;
}

std::size_t VehiclePlannerRegistry::size() const { return planners_.size(); }

}  // namespace intrinsic::motion_planning::vehicle
