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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_RECONSTRUCTION_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_RECONSTRUCTION_H_

#include <span>
#include <string_view>

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Turns a parent-linked chain of lattice-Dijkstra search nodes into one
// VehiclePlanResult trajectory. This is the concatenation step of
// SearchKinodynamicBaseline, extracted unchanged. It does not search,
// expand, smooth, retime, or read World, and it never edits a sample: a
// chain it cannot reconstruct as given is rejected, not repaired.

// Id stamped when neither the request nor the options name a trajectory.
inline constexpr std::string_view kReconstructedTrajectoryId =
    "reconstructed-trajectory";

// One search node. Views alias caller storage and must outlive the call.
struct ReconstructionNodeView {
  // Index of the parent in the node array. -1 marks the root.
  int parent = -1;
  // Absolute time of this node in seconds (the search's cumulative cost).
  // The root must be exactly 0.
  double time_s = 0;
  // State at the end of the edge. The root holds the request start state.
  VehiclePlanningState state;
  // Edge from the parent to this node: PropagationSample times are relative
  // to the parent, the first sample is the parent state at t = 0, and the
  // last sample is `state`. Ignored for the root.
  std::span<const PropagationSample> edge_samples;
};

struct ReconstructionOptions {
  // Used when the request leaves trajectory_id empty. Borrowed, and must
  // outlive the call. Empty means kReconstructedTrajectoryId.
  std::string_view default_trajectory_id;
};

// First defect wins: the chain is checked root first, and within one node in
// the order below. Python: the same names in upper snake case, numbered
// identically.
enum class TrajectoryReconstructionError {
  kOk = 0,
  // states_present is false, the node array is empty, or goal_index is out
  // of range.
  kBadRequest = 1,
  // A parent index is out of range, the chain loops, or it does not reach a
  // root (parent == -1).
  kBadChain = 2,
  // The root does not hold the request start state exactly at time 0.
  kStartMismatch = 3,
  // A non-root node has no edge samples, or its edge does not begin at the
  // parent state at relative t = 0, or does not end at the node state.
  kDisjointEdge = 4,
  // A time is non-finite or negative, edge times are not strictly
  // increasing, a node time precedes its parent, or the concatenated
  // trajectory is not strictly increasing at nanosecond resolution.
  kNonMonotonicTime = 5,
  // AssessVehicleTrajectory did not accept the reconstructed trajectory.
  kRejected = 6,
};

struct TrajectoryReconstructionResult {
  TrajectoryReconstructionError error =
      TrajectoryReconstructionError::kBadRequest;
  // Index into `nodes` of the node that carries the defect, or -1 when the
  // defect is not tied to one node (and for kOk).
  int failed_node = -1;
  // status == kOk and a full trajectory only when error == kOk. Otherwise
  // status is kInvalidRequest and no trajectory is carried.
  VehiclePlanResult trajectory;
};

// Walks parent links from `goal_index` to the root, then concatenates the
// edge samples root first.
//
// Each edge's first sample (the parent state at t = 0) repeats the previous
// sample and is dropped. Every other sample is kept at absolute time
// parent.time_s + sample.time_s, split into seconds and nanos. A chain whose
// goal is the root yields one sample at t = 0.
//
// The result has frame world_enu, model id uuv_planner, validity state 1,
// and trajectory_id from the request or the options. Its first sample is the
// request start state at t = 0, times are strictly increasing, and
// AssessVehicleTrajectory accepts it.
TrajectoryReconstructionResult ReconstructVehicleTrajectory(
    std::span<const ReconstructionNodeView> nodes, int goal_index,
    const VehiclePlanRequest& request,
    const ReconstructionOptions& options = {});

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_RECONSTRUCTION_H_
