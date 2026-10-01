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

#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"

#include <chrono>

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

VehiclePlanResult Failure(VehiclePlanStatus status) {
  VehiclePlanResult result;
  result.status = status;
  return result;
}

}  // namespace

VehiclePlanResult RunWithDeadline(const VehiclePlanner& planner,
                                  const VehiclePlanRequest& request,
                                  const VehiclePlanRunOptions& options) {
  if (options.cancel != nullptr && options.cancel->load()) {
    return Failure(VehiclePlanStatus::kCancelled);
  }
  if (options.deadline_present &&
      std::chrono::steady_clock::now() >= options.deadline) {
    return Failure(VehiclePlanStatus::kDeadlineExceeded);
  }
  return planner.Plan(request);
}

}  // namespace intrinsic::motion_planning::vehicle
