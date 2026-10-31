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

// Replan trigger for the UUV reference solution (#265, lock id
// l43-5-replan-trigger-v1). Sim and test only, plain C++.
//
// When the validity binding changes (a new snapshot_id), EvaluateReplan
// rechecks the remaining part of the active trajectory against the new
// binding. A route defect (bounds, geofence or clearance) triggers one fresh
// kinodynamic-baseline plan from the current truth state. The function is pure:
// it reads no clock, starts no thread, does no I/O, never mutates or stores a
// binding, and never sets a deadline. Only the cancel flag can stop a run.

#ifndef INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_REPLAN_TRIGGER_H_
#define INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_REPLAN_TRIGGER_H_

#include <atomic>
#include <cstdint>
#include <vector>

#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

namespace intrinsic::simulation::uuv_reference {

enum class ReplanOutcome {
  // The new binding has the same snapshot_id. No check ran.
  kUnchanged = 0,
  // The remaining trajectory is valid under the new binding.
  kKept = 1,
  // The recheck found a route defect and a fresh plan succeeded.
  kReplanned = 2,
  // The recheck found a route defect and the fresh plan found no solution.
  kNoSolution = 3,
  // A precondition, a malformed or stale binding, or a planner failure other
  // than kNoSolution. Nothing was replaced.
  kError = 4,
};

enum class ReplanInputError {
  kNone = 0,
  kActiveNotOk = 1,
  kActiveEmpty = 2,
  // A sample without time or twist, or with nanos out of range.
  kActiveSample = 3,
  // First sample time is not 0, or times are not strictly increasing.
  kActiveTimes = 4,
  kNegativeTime = 5,
  // truth_now is non-finite or has a non-unit orientation.
  kTruthState = 6,
  // goal_request.states_present is false.
  kGoalRequest = 7,
};

struct RemainingTrajectory {
  ReplanInputError error = ReplanInputError::kNone;
  // samples[0] is truth_now at time 0. Then every active sample strictly after
  // t_now, with time_s = (sample_ns - t_now_ns) / 1e9. Empty on error.
  std::vector<motion_planning::vehicle::PropagationSample> samples;
};

// `t_now_ns` is playback time on `active` in integer nanoseconds from its first
// sample. A sample time is seconds * 1e9 + nanos. Checks, in order:
// kActiveNotOk, kActiveEmpty, kActiveSample, kActiveTimes, kNegativeTime,
// kTruthState. If t_now_ns is at or past the last sample, only truth_now is
// returned.
RemainingTrajectory BuildRemainingTrajectory(
    const motion_planning::vehicle::VehiclePlanResult& active,
    const motion_planning::vehicle::VehiclePlanningState& truth_now,
    int64_t t_now_ns);

struct ReplanStepResult {
  ReplanOutcome outcome = ReplanOutcome::kError;
  ReplanInputError input_error = ReplanInputError::kNone;
  bool recheck_ran = false;
  // Valid when recheck_ran.
  motion_planning::vehicle::TrajectoryValidityResult recheck;
  bool replan_ran = false;
  // Valid when replan_ran.
  motion_planning::vehicle::VehiclePlanStatus plan_status =
      motion_planning::vehicle::VehiclePlanStatus::kInvalidRequest;
  // Only for kReplanned.
  motion_planning::vehicle::VehiclePlanResult trajectory;
};

// First match wins:
//  1. Equal snapshot_id (byte equality): kUnchanged.
//  2. Preconditions: kError with input_error set.
//  3. Remaining trajectory from `active` (BuildRemainingTrajectory).
//  4. Recheck against the whole new binding. kOk: kKept. kBounds, kGeofence or
//     kClearance: replan. kBadRequest, kStaleSnapshot or kFrameError: kError
//     (the binding itself is malformed or stale; no replan).
//  5. Fresh planner from a copy of new_binding whose run_options carry only
//     `cancel`; goal_request with start_state = truth_now. kOk: kReplanned.
//     kNoSolution: kNoSolution. Anything else: kError.
//
// The caller adopts new_binding as current after kKept or kReplanned; this
// function never stores a binding. Inputs are not retained. Views borrowed
// from new_binding (pose_frame, descriptor) must outlive the call. `cancel` may
// be null.
ReplanStepResult EvaluateReplan(
    const motion_planning::vehicle::KinodynamicBaselineConfig& current_binding,
    const motion_planning::vehicle::KinodynamicBaselineConfig& new_binding,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics,
    const motion_planning::vehicle::VehiclePlanResult& active,
    const motion_planning::vehicle::VehiclePlanningState& truth_now,
    int64_t t_now_ns,
    const motion_planning::vehicle::VehiclePlanRequest& goal_request,
    std::atomic<bool>* cancel);

}  // namespace intrinsic::simulation::uuv_reference

#endif  // INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_REPLAN_TRIGGER_H_
