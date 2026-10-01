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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_DEADLINE_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_DEADLINE_H_

#include <atomic>
#include <chrono>

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Deadline and cooperative cancellation around an existing VehiclePlanner.
// This is a wrapper only: no search, no sampling, no threads.

struct VehiclePlanRunOptions {
  // Absolute steady-clock deadline. If it has already passed before Plan
  // starts, the result is kDeadlineExceeded and the inner planner is not
  // called. When false, there is no time bound (cancel still applies).
  bool deadline_present = false;
  std::chrono::steady_clock::time_point deadline{};

  // Cooperative cancel flag. Not owned; must outlive the call. Null means
  // the run is not cancelable. A flag that is already set before Plan starts
  // yields kCancelled without calling the inner planner.
  std::atomic<bool>* cancel = nullptr;
};

// Runs `planner.Plan(request)` under the policy in `options`. First
// applicable wins:
//
//   1. cancel set before start      -> kCancelled, inner Plan not called.
//   2. deadline passed before start -> kDeadlineExceeded, inner Plan not
//                                      called.
//   3. otherwise the inner Plan result is returned unchanged, including
//      non-ok statuses such as kNoSolution and kInvalidRequest.
//
// A planner that polls the same cancel flag and deadline while it works can
// itself return kCancelled or kDeadlineExceeded; those are forwarded like
// any other status. A cancelled or expired result never carries a
// trajectory.
//
// The call is synchronous on the calling thread. It does not spin or sleep
// and creates no thread; how long it takes is bounded only by how well the
// inner planner honours `options`.
VehiclePlanResult RunWithDeadline(const VehiclePlanner& planner,
                                  const VehiclePlanRequest& request,
                                  const VehiclePlanRunOptions& options);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_PLANNER_DEADLINE_H_
