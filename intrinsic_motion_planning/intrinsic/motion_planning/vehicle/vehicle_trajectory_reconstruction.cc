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

#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_reconstruction.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::SampleTimeBefore;
using ::intrinsic::vehicle::TrajectorySampleView;

constexpr std::string_view kModelId = "uuv_planner";

TrajectoryReconstructionResult Failure(TrajectoryReconstructionError error,
                                       int failed_node = -1) {
  TrajectoryReconstructionResult result;
  result.error = error;
  result.failed_node = failed_node;
  return result;
}

bool SameState(const VehiclePlanningState& a, const VehiclePlanningState& b) {
  return a.position.x == b.position.x && a.position.y == b.position.y &&
         a.position.z == b.position.z && a.orientation.x == b.orientation.x &&
         a.orientation.y == b.orientation.y &&
         a.orientation.z == b.orientation.z &&
         a.orientation.w == b.orientation.w &&
         a.twist.linear_x == b.twist.linear_x &&
         a.twist.linear_y == b.twist.linear_y &&
         a.twist.linear_z == b.twist.linear_z &&
         a.twist.angular_x == b.twist.angular_x &&
         a.twist.angular_y == b.twist.angular_y &&
         a.twist.angular_z == b.twist.angular_z;
}

bool FiniteAndNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

void SplitTime(double time_s, int64_t* seconds, int32_t* nanos) {
  const double whole = std::floor(time_s);
  int64_t whole64 = static_cast<int64_t>(whole);
  const double nanos_scale = static_cast<double>(embodiment::kNanosPerSecond);
  int64_t nanos64 = std::llround((time_s - whole) * nanos_scale);
  if (nanos64 >= embodiment::kNanosPerSecond) {
    nanos64 -= embodiment::kNanosPerSecond;
    ++whole64;
  }
  if (nanos64 < 0) {
    nanos64 = 0;
  }
  *seconds = whole64;
  *nanos = static_cast<int32_t>(nanos64);
}

TrajectorySampleView ToTrajectorySample(const VehiclePlanningState& state,
                                        double time_s) {
  TrajectorySampleView sample;
  sample.time_present = true;
  SplitTime(time_s, &sample.seconds, &sample.nanos);
  sample.position = state.position;
  sample.orientation = state.orientation;
  sample.twist_present = true;
  sample.twist = state.twist;
  sample.acceleration_present = false;
  return sample;
}

void FillSuccessHeader(VehiclePlanResult* result,
                       const VehiclePlanRequest& request,
                       const ReconstructionOptions& options) {
  result->status = VehiclePlanStatus::kOk;
  result->header_present = true;
  result->validity_present = true;
  result->validity_state = 1;
  result->frame_id = std::string(embodiment::kWorldEnuFrameId);
  if (!request.trajectory_id.empty()) {
    result->trajectory_id = request.trajectory_id;
  } else if (!options.default_trajectory_id.empty()) {
    result->trajectory_id = std::string(options.default_trajectory_id);
  } else {
    result->trajectory_id = std::string(kReconstructedTrajectoryId);
  }
  result->provenance_present = true;
  result->model_id = std::string(kModelId);
}

// Chain indices root first. Empty on a bad link or a loop, with *failed_node
// set to the node where the walk stopped.
std::vector<int> WalkToRoot(std::span<const ReconstructionNodeView> nodes,
                            int goal_index, int* failed_node) {
  std::vector<int> chain;
  int index = goal_index;
  while (true) {
    if (chain.size() >= nodes.size()) {
      *failed_node = index;
      return {};
    }
    chain.push_back(index);
    const int parent = nodes[index].parent;
    if (parent == -1) {
      break;
    }
    if (parent < 0 || static_cast<std::size_t>(parent) >= nodes.size()) {
      *failed_node = index;
      return {};
    }
    index = parent;
  }
  std::reverse(chain.begin(), chain.end());
  return chain;
}

// Checks one non-root edge. kOk on success.
TrajectoryReconstructionError CheckEdge(const ReconstructionNodeView& parent,
                                        const ReconstructionNodeView& node) {
  const std::span<const PropagationSample> edge = node.edge_samples;
  if (edge.empty() || edge.front().time_s != 0.0 ||
      !SameState(edge.front().state, parent.state) ||
      !SameState(edge.back().state, node.state)) {
    return TrajectoryReconstructionError::kDisjointEdge;
  }
  if (!FiniteAndNonNegative(node.time_s) || node.time_s < parent.time_s) {
    return TrajectoryReconstructionError::kNonMonotonicTime;
  }
  for (std::size_t i = 0; i < edge.size(); ++i) {
    if (!FiniteAndNonNegative(edge[i].time_s) ||
        (i > 0 && !(edge[i - 1].time_s < edge[i].time_s))) {
      return TrajectoryReconstructionError::kNonMonotonicTime;
    }
  }
  return TrajectoryReconstructionError::kOk;
}

}  // namespace

TrajectoryReconstructionResult ReconstructVehicleTrajectory(
    std::span<const ReconstructionNodeView> nodes, int goal_index,
    const VehiclePlanRequest& request, const ReconstructionOptions& options) {
  if (!request.states_present || nodes.empty() || goal_index < 0 ||
      static_cast<std::size_t>(goal_index) >= nodes.size()) {
    return Failure(TrajectoryReconstructionError::kBadRequest);
  }
  int failed_node = -1;
  const std::vector<int> chain = WalkToRoot(nodes, goal_index, &failed_node);
  if (chain.empty()) {
    return Failure(TrajectoryReconstructionError::kBadChain, failed_node);
  }

  const ReconstructionNodeView& root = nodes[chain.front()];
  if (root.time_s != 0.0 || !SameState(root.state, request.start_state)) {
    return Failure(TrajectoryReconstructionError::kStartMismatch,
                   chain.front());
  }

  VehiclePlanResult trajectory;
  FillSuccessHeader(&trajectory, request, options);
  trajectory.samples.push_back(ToTrajectorySample(request.start_state, 0.0));
  for (std::size_t link = 1; link < chain.size(); ++link) {
    const ReconstructionNodeView& parent = nodes[chain[link - 1]];
    const ReconstructionNodeView& node = nodes[chain[link]];
    const TrajectoryReconstructionError edge_error = CheckEdge(parent, node);
    if (edge_error != TrajectoryReconstructionError::kOk) {
      return Failure(edge_error, chain[link]);
    }
    // Sample 0 repeats the parent state at the parent time and is dropped.
    for (std::size_t i = 1; i < node.edge_samples.size(); ++i) {
      const PropagationSample& sample = node.edge_samples[i];
      const TrajectorySampleView view =
          ToTrajectorySample(sample.state, parent.time_s + sample.time_s);
      if (!SampleTimeBefore(trajectory.samples.back(), view)) {
        return Failure(TrajectoryReconstructionError::kNonMonotonicTime,
                       chain[link]);
      }
      trajectory.samples.push_back(view);
    }
  }

  if (!::intrinsic::vehicle::AssessVehicleTrajectory(
           AsVehicleTrajectoryView(trajectory))
           .accepted) {
    return Failure(TrajectoryReconstructionError::kRejected);
  }
  TrajectoryReconstructionResult result;
  result.error = TrajectoryReconstructionError::kOk;
  result.trajectory = std::move(trajectory);
  return result;
}

}  // namespace intrinsic::motion_planning::vehicle
