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

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/replan_trigger.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"
#include "intrinsic/safety/clearance_rule.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using ::intrinsic::motion_planning::vehicle::CheckTrajectoryValidity;
using ::intrinsic::motion_planning::vehicle::KinodynamicBaselineConfig;
using ::intrinsic::motion_planning::vehicle::MakeKinodynamicBaselinePlanner;
using ::intrinsic::motion_planning::vehicle::PropagationSample;
using ::intrinsic::motion_planning::vehicle::RunWithDeadline;
using ::intrinsic::motion_planning::vehicle::StateSpaceError;
using ::intrinsic::motion_planning::vehicle::TrajectoryValidityError;
using ::intrinsic::motion_planning::vehicle::TrajectoryValidityRequest;
using ::intrinsic::motion_planning::vehicle::TrajectoryValidityResult;
using ::intrinsic::motion_planning::vehicle::Validate;
using ::intrinsic::motion_planning::vehicle::VehiclePlanningState;
using ::intrinsic::motion_planning::vehicle::VehiclePlanRequest;
using ::intrinsic::motion_planning::vehicle::VehiclePlanResult;
using ::intrinsic::motion_planning::vehicle::VehiclePlanRunOptions;
using ::intrinsic::motion_planning::vehicle::VehiclePlanStatus;
using ::intrinsic::motion_planning::vehicle::VehicleStateBounds;
using ::intrinsic::safety::ClearanceSample;
using ::intrinsic::vehicle::SampleTimeOk;
using ::intrinsic::vehicle::TrajectorySampleView;

constexpr int64_t kNanosPerSecond = 1000000000;
// Keeps seconds * 1e9 + nanos inside int64.
constexpr int64_t kMaxSampleSeconds = 9000000000;

bool SampleTimeRepresentable(const TrajectorySampleView& sample) {
  return sample.seconds >= -kMaxSampleSeconds &&
         sample.seconds <= kMaxSampleSeconds;
}

int64_t SampleNs(const TrajectorySampleView& sample) {
  return sample.seconds * kNanosPerSecond + sample.nanos;
}

VehiclePlanningState SampleState(const TrajectorySampleView& sample) {
  VehiclePlanningState state;
  state.position = sample.position;
  state.orientation = sample.orientation;
  state.twist = sample.twist;
  return state;
}

// Private copy of the planner's validity binding (E2): the whole binding, not
// only the snapshot id.
TrajectoryValidityRequest ValidityBinding(
    const KinodynamicBaselineConfig& config) {
  TrajectoryValidityRequest request;
  request.snapshot_id = config.snapshot_id;
  request.descriptor = config.descriptor;
  request.assess_skew = config.assess_skew;
  request.timings = config.timings;
  request.query_time = config.query_time;
  request.skew_policy = config.skew_policy;
  request.bounds = config.bounds;
  request.fence = config.fence;
  request.pose_frame = config.pose_frame;
  request.min_clearance_m = config.min_clearance_m;
  return request;
}

// Private copy of the planner's clearance template (E2).
std::vector<ClearanceSample> TemplateClearance(
    const KinodynamicBaselineConfig& config, std::size_t count) {
  const std::string_view frame = config.pose_frame.empty()
                                     ? std::string_view(config.fence.frame_id)
                                     : config.pose_frame;
  std::vector<ClearanceSample> samples(count);
  for (ClearanceSample& sample : samples) {
    sample.frame_id = frame;
    sample.source = config.clearance_source;
    sample.clearance_m = config.clearance_template_m;
    sample.snapshot_usable = true;
  }
  return samples;
}

ReplanStepResult InputError(ReplanInputError error) {
  ReplanStepResult result;
  result.outcome = ReplanOutcome::kError;
  result.input_error = error;
  return result;
}

bool RouteDefect(TrajectoryValidityError error) {
  return error == TrajectoryValidityError::kBounds ||
         error == TrajectoryValidityError::kGeofence ||
         error == TrajectoryValidityError::kClearance;
}

}  // namespace

RemainingTrajectory BuildRemainingTrajectory(
    const VehiclePlanResult& active, const VehiclePlanningState& truth_now,
    int64_t t_now_ns) {
  RemainingTrajectory remaining;
  if (active.status != VehiclePlanStatus::kOk) {
    remaining.error = ReplanInputError::kActiveNotOk;
    return remaining;
  }
  if (active.samples.empty()) {
    remaining.error = ReplanInputError::kActiveEmpty;
    return remaining;
  }
  for (const TrajectorySampleView& sample : active.samples) {
    if (!SampleTimeOk(sample) || !sample.twist_present ||
        !SampleTimeRepresentable(sample)) {
      remaining.error = ReplanInputError::kActiveSample;
      return remaining;
    }
  }
  if (SampleNs(active.samples.front()) != 0) {
    remaining.error = ReplanInputError::kActiveTimes;
    return remaining;
  }
  for (std::size_t i = 1; i < active.samples.size(); ++i) {
    if (SampleNs(active.samples[i]) <= SampleNs(active.samples[i - 1])) {
      remaining.error = ReplanInputError::kActiveTimes;
      return remaining;
    }
  }
  if (t_now_ns < 0) {
    remaining.error = ReplanInputError::kNegativeTime;
    return remaining;
  }
  if (Validate(truth_now, VehicleStateBounds{}) != StateSpaceError::kOk) {
    remaining.error = ReplanInputError::kTruthState;
    return remaining;
  }

  PropagationSample start;
  start.time_s = 0.0;
  start.state = truth_now;
  remaining.samples.push_back(start);
  for (const TrajectorySampleView& sample : active.samples) {
    const int64_t sample_ns = SampleNs(sample);
    if (sample_ns <= t_now_ns) {
      continue;
    }
    PropagationSample rebased;
    rebased.time_s = static_cast<double>(sample_ns - t_now_ns) /
                     static_cast<double>(kNanosPerSecond);
    rebased.state = SampleState(sample);
    remaining.samples.push_back(rebased);
  }
  return remaining;
}

ReplanStepResult EvaluateReplan(
    const KinodynamicBaselineConfig& current_binding,
    const KinodynamicBaselineConfig& new_binding,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics,
    const VehiclePlanResult& active, const VehiclePlanningState& truth_now,
    int64_t t_now_ns, const VehiclePlanRequest& goal_request,
    std::atomic<bool>* cancel) {
  ReplanStepResult result;
  if (new_binding.snapshot_id == current_binding.snapshot_id) {
    result.outcome = ReplanOutcome::kUnchanged;
    return result;
  }

  RemainingTrajectory remaining =
      BuildRemainingTrajectory(active, truth_now, t_now_ns);
  if (remaining.error != ReplanInputError::kNone) {
    return InputError(remaining.error);
  }
  if (!goal_request.states_present) {
    return InputError(ReplanInputError::kGoalRequest);
  }

  TrajectoryValidityRequest recheck = ValidityBinding(new_binding);
  recheck.samples = std::move(remaining.samples);
  recheck.clearance_samples =
      TemplateClearance(new_binding, recheck.samples.size());
  result.recheck_ran = true;
  result.recheck = CheckTrajectoryValidity(recheck);
  if (result.recheck.error == TrajectoryValidityError::kOk) {
    result.outcome = ReplanOutcome::kKept;
    return result;
  }
  if (!RouteDefect(result.recheck.error)) {
    result.outcome = ReplanOutcome::kError;
    return result;
  }

  VehiclePlanRunOptions options;
  options.cancel = cancel;
  KinodynamicBaselineConfig config = new_binding;
  config.run_options = &options;
  const auto planner = MakeKinodynamicBaselinePlanner(config, dynamics);
  VehiclePlanRequest request = goal_request;
  request.start_state = truth_now;

  VehiclePlanResult planned = RunWithDeadline(*planner, request, options);
  result.replan_ran = true;
  result.plan_status = planned.status;
  switch (planned.status) {
    case VehiclePlanStatus::kOk:
      result.outcome = ReplanOutcome::kReplanned;
      result.trajectory = std::move(planned);
      break;
    case VehiclePlanStatus::kNoSolution:
      result.outcome = ReplanOutcome::kNoSolution;
      break;
    default:
      result.outcome = ReplanOutcome::kError;
      break;
  }
  return result;
}

}  // namespace intrinsic::simulation::uuv_reference
