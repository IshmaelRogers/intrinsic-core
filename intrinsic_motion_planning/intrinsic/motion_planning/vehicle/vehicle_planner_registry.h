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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_REGISTRY_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_REGISTRY_H_

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Exact-match registry of vehicle planner implementations. This is not the
// embodiment category id ai.intrinsic.capability.planning. There is no
// search, no default planner, and no gRPC service.

// Well-known vehicle planner implementation ids (not the embodiment
// category id).
inline constexpr std::string_view kVehiclePlannerFake =
    "ai.intrinsic.vehicle_planner.fake";
// Reserved for later leaves (do not implement algorithms here):
// "ai.intrinsic.vehicle_planner.kinodynamic_baseline"  // #105+

enum class PlannerRegistryError {
  kOk = 0,
  kEmptyId = 1,
  kDuplicateId = 2,
  kNotFound = 3,
  kNullPlanner = 4,
};

enum class VehiclePlanStatus {
  kOk = 0,
  kNoSolution = 1,
  kInvalidRequest = 2,
  kCancelled = 3,
  kDeadlineExceeded = 4,
};

struct VehiclePlanRequest {
  // Opaque start/goal labels for the fake. Later planners will use
  // VehiclePlanningState and World snapshots. Keep plain values.
  std::string start_label;
  std::string goal_label;
  // Optional id stamped onto a success trajectory. Empty means the planner
  // chooses its own id. The fake uses "fake-trajectory".
  std::string trajectory_id;
};

// Owned plan. A trajectory is present only when status == kOk. Samples and
// strings are owned so a view built from this result stays valid for the
// result's lifetime.
struct VehiclePlanResult {
  VehiclePlanStatus status = VehiclePlanStatus::kInvalidRequest;
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string frame_id;
  std::string trajectory_id;
  std::vector<::intrinsic::vehicle::TrajectorySampleView> samples;
  bool provenance_present = false;
  std::string model_id;
};

// Aliases strings and samples inside `result`. Valid only while `result`
// is alive. A non-ok status yields an empty, unengaged view.
inline ::intrinsic::vehicle::VehicleTrajectoryView AsVehicleTrajectoryView(
    const VehiclePlanResult& result) {
  ::intrinsic::vehicle::VehicleTrajectoryView view;
  if (result.status != VehiclePlanStatus::kOk) {
    return view;
  }
  view.header_present = result.header_present;
  view.validity_present = result.validity_present;
  view.validity_state = result.validity_state;
  view.frame_id = result.frame_id;
  view.trajectory_id = result.trajectory_id;
  view.samples = result.samples;
  view.provenance_present = result.provenance_present;
  view.model_id = result.model_id;
  return view;
}

class VehiclePlanner {
 public:
  virtual ~VehiclePlanner() = default;
  virtual std::string_view id() const = 0;
  virtual VehiclePlanResult Plan(const VehiclePlanRequest& request) const = 0;
};

// Exact map of implementations. Not thread-safe. Does not construct a
// planner on lookup.
class VehiclePlannerRegistry {
 public:
  // First defect wins: null, empty id, duplicate id, then store.
  // A duplicate leaves the first registration in place.
  PlannerRegistryError Register(std::shared_ptr<VehiclePlanner> planner);

  // On kOk, sets *out to the registered planner. On every other error,
  // leaves *out unchanged. `out` must be non-null.
  PlannerRegistryError Lookup(std::string_view planner_id,
                              std::shared_ptr<VehiclePlanner>* out) const;

  std::size_t size() const;

 private:
  std::map<std::string, std::shared_ptr<VehiclePlanner>, std::less<>> planners_;
};

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_REGISTRY_H_
