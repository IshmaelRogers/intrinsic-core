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

#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_reconstruction.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::safety::ClearanceSample;

struct ClosedKey {
  int64_t bin_x = 0;
  int64_t bin_y = 0;
  int64_t bin_z = 0;
  uint64_t qx = 0;
  uint64_t qy = 0;
  uint64_t qz = 0;
  uint64_t qw = 0;
  uint64_t linear_x = 0;
  uint64_t linear_y = 0;
  uint64_t linear_z = 0;
  uint64_t angular_x = 0;
  uint64_t angular_y = 0;
  uint64_t angular_z = 0;

  bool operator<(const ClosedKey& other) const {
    return std::tie(bin_x, bin_y, bin_z, qx, qy, qz, qw, linear_x, linear_y,
                    linear_z, angular_x, angular_y, angular_z) <
           std::tie(other.bin_x, other.bin_y, other.bin_z, other.qx, other.qy,
                    other.qz, other.qw, other.linear_x, other.linear_y,
                    other.linear_z, other.angular_x, other.angular_y,
                    other.angular_z);
  }
};

struct SearchNode {
  VehiclePlanningState state;
  double g_cost = 0;
  int parent = -1;
  int primitive_index = -1;
  int parent_expand_seq = 0;
  int node_seq = 0;
  std::vector<PropagationSample> edge_samples;
};

// Min-heap: true when `left` should be popped after `right`.
struct OpenWorse {
  const std::vector<SearchNode>* nodes = nullptr;

  bool operator()(int left, int right) const {
    const SearchNode& a = (*nodes)[left];
    const SearchNode& b = (*nodes)[right];
    if (a.g_cost != b.g_cost) {
      return a.g_cost > b.g_cost;
    }
    if (a.primitive_index != b.primitive_index) {
      return a.primitive_index > b.primitive_index;
    }
    if (a.parent_expand_seq != b.parent_expand_seq) {
      return a.parent_expand_seq > b.parent_expand_seq;
    }
    return a.node_seq > b.node_seq;
  }
};

VehiclePlanResult Failure(VehiclePlanStatus status) {
  VehiclePlanResult result;
  result.status = status;
  return result;
}

// A reconstruction defect is unreachable for a chain this search builds. It
// fails closed as kNoSolution and never returns a trajectory.
VehiclePlanResult Reconstructed(TrajectoryReconstructionResult reconstructed) {
  if (reconstructed.error != TrajectoryReconstructionError::kOk) {
    return Failure(VehiclePlanStatus::kNoSolution);
  }
  return std::move(reconstructed.trajectory);
}

ReconstructionOptions ReconstructionBinding() {
  ReconstructionOptions options;
  options.default_trajectory_id = kKinodynamicBaselineTrajectoryId;
  return options;
}

VehiclePlanResult StartGoalSuccess(const VehiclePlanRequest& request) {
  ReconstructionNodeView root;
  root.state = request.start_state;
  return Reconstructed(ReconstructVehicleTrajectory(
      std::span<const ReconstructionNodeView>(&root, 1), 0, request,
      ReconstructionBinding()));
}

VehiclePlanResult SuccessFromNode(const std::vector<SearchNode>& nodes,
                                  int goal_index,
                                  const VehiclePlanRequest& request) {
  std::vector<ReconstructionNodeView> views;
  views.reserve(nodes.size());
  for (const SearchNode& node : nodes) {
    ReconstructionNodeView view;
    view.parent = node.parent;
    view.time_s = node.g_cost;
    view.state = node.state;
    view.edge_samples = node.edge_samples;
    views.push_back(view);
  }
  return Reconstructed(ReconstructVehicleTrajectory(views, goal_index, request,
                                                    ReconstructionBinding()));
}

std::string_view ClearanceFrame(const KinodynamicBaselineConfig& config) {
  if (!config.pose_frame.empty()) {
    return config.pose_frame;
  }
  return config.fence.frame_id;
}

std::vector<ClearanceSample> TemplateClearance(
    const KinodynamicBaselineConfig& config, std::size_t count) {
  std::vector<ClearanceSample> samples(count);
  const std::string_view frame = ClearanceFrame(config);
  for (ClearanceSample& sample : samples) {
    sample.frame_id = frame;
    sample.source = config.clearance_source;
    sample.clearance_m = config.clearance_template_m;
    sample.snapshot_usable = true;
  }
  return samples;
}

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

uint64_t DoubleBits(double value) {
  static_assert(sizeof(double) == sizeof(uint64_t));
  uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

int64_t PositionBin(double value, double bin_m) {
  const double scaled = std::floor(value / bin_m);
  if (scaled >= static_cast<double>(std::numeric_limits<int64_t>::max())) {
    return std::numeric_limits<int64_t>::max();
  }
  if (scaled <= static_cast<double>(std::numeric_limits<int64_t>::min())) {
    return std::numeric_limits<int64_t>::min();
  }
  return static_cast<int64_t>(scaled);
}

ClosedKey MakeClosedKey(const VehiclePlanningState& state, double bin_m) {
  ClosedKey key;
  key.bin_x = PositionBin(state.position.x, bin_m);
  key.bin_y = PositionBin(state.position.y, bin_m);
  key.bin_z = PositionBin(state.position.z, bin_m);
  key.qx = DoubleBits(state.orientation.x);
  key.qy = DoubleBits(state.orientation.y);
  key.qz = DoubleBits(state.orientation.z);
  key.qw = DoubleBits(state.orientation.w);
  key.linear_x = DoubleBits(state.twist.linear_x);
  key.linear_y = DoubleBits(state.twist.linear_y);
  key.linear_z = DoubleBits(state.twist.linear_z);
  key.angular_x = DoubleBits(state.twist.angular_x);
  key.angular_y = DoubleBits(state.twist.angular_y);
  key.angular_z = DoubleBits(state.twist.angular_z);
  return key;
}

std::optional<VehiclePlanStatus> Poll(const VehiclePlanRunOptions* options) {
  if (options == nullptr) {
    return std::nullopt;
  }
  if (options->cancel != nullptr && options->cancel->load()) {
    return VehiclePlanStatus::kCancelled;
  }
  if (options->deadline_present &&
      std::chrono::steady_clock::now() >= options->deadline) {
    return VehiclePlanStatus::kDeadlineExceeded;
  }
  return std::nullopt;
}

bool FiniteAndNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

VehiclePlanResult SearchKinodynamicBaseline(
    const KinodynamicBaselineConfig& config, const VehiclePlanRequest& request,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics& dynamics) {
  if (!request.states_present) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (Validate(request.start_state, config.bounds) != StateSpaceError::kOk ||
      Validate(request.goal_state, config.bounds) != StateSpaceError::kOk) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (!FiniteAndNonNegative(config.goal_tolerance)) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (!std::isfinite(config.position_bin_m) || !(config.position_bin_m > 0.0)) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (config.max_expansions < 1) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (!FiniteAndNonNegative(config.clearance_template_m)) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }

  const PrimitiveSetResult generated =
      GenerateUuvMotionPrimitives(config.primitives);
  if (generated.error != PrimitiveSetError::kOk ||
      generated.primitives.empty()) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }

  TrajectoryValidityRequest probe = ValidityBinding(config);
  PropagationSample probe_sample;
  probe_sample.time_s = 0.0;
  probe_sample.state = request.start_state;
  probe.samples.push_back(probe_sample);
  probe.clearance_samples = TemplateClearance(config, 1);
  const TrajectoryValidityResult probed = CheckTrajectoryValidity(probe);
  if (probed.error == TrajectoryValidityError::kBadRequest ||
      probed.error == TrajectoryValidityError::kStaleSnapshot) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (probed.error == TrajectoryValidityError::kOk &&
      Distance(request.start_state, request.goal_state) <=
          config.goal_tolerance) {
    return StartGoalSuccess(request);
  }

  std::vector<SearchNode> nodes;
  nodes.reserve(static_cast<std::size_t>(config.max_expansions) + 1);
  SearchNode start;
  start.state = request.start_state;
  start.g_cost = 0.0;
  start.parent = -1;
  start.primitive_index = -1;
  start.parent_expand_seq = 0;
  start.node_seq = 0;
  nodes.push_back(std::move(start));

  std::priority_queue<int, std::vector<int>, OpenWorse> open(OpenWorse{&nodes});
  open.push(0);
  int next_node_seq = 1;
  // First expanded parent stamps children with 1. Start itself stays 0.
  int next_expand_seq = 1;
  int expansions = 0;
  std::map<ClosedKey, double> settled;

  while (!open.empty() && expansions < config.max_expansions) {
    if (const std::optional<VehiclePlanStatus> polled =
            Poll(config.run_options)) {
      return Failure(*polled);
    }
    const int index = open.top();
    open.pop();
    const ClosedKey key =
        MakeClosedKey(nodes[index].state, config.position_bin_m);
    if (settled.contains(key)) {
      continue;
    }
    settled.emplace(key, nodes[index].g_cost);
    // Start==goal with a kOk probe already returned a one-sample trajectory.
    // A start node that reaches this loop failed that probe, so it is not a
    // success; its outgoing edges are checked like any other node.
    if (nodes[index].parent >= 0 &&
        Distance(nodes[index].state, request.goal_state) <=
            config.goal_tolerance) {
      return SuccessFromNode(nodes, index, request);
    }

    const int this_expand_seq = next_expand_seq++;
    const double parent_g = nodes[index].g_cost;
    const VehiclePlanningState parent_state = nodes[index].state;
    for (std::size_t primitive_index = 0;
         primitive_index < generated.primitives.size(); ++primitive_index) {
      if (const std::optional<VehiclePlanStatus> polled =
              Poll(config.run_options)) {
        return Failure(*polled);
      }
      const VehicleMotionPrimitive& primitive =
          generated.primitives[primitive_index];
      PropagationResult propagated = PropagateUuvMotionPrimitive(
          parent_state, primitive, config.propagation, dynamics);
      if (propagated.error != PropagationError::kOk ||
          propagated.samples.empty()) {
        continue;
      }
      TrajectoryValidityRequest edge = ValidityBinding(config);
      edge.samples = propagated.samples;
      edge.clearance_samples = TemplateClearance(config, edge.samples.size());
      if (CheckTrajectoryValidity(edge).error != TrajectoryValidityError::kOk) {
        continue;
      }
      const VehiclePlanningState child_state = propagated.samples.back().state;
      const ClosedKey child_key =
          MakeClosedKey(child_state, config.position_bin_m);
      if (settled.contains(child_key)) {
        continue;
      }
      SearchNode child;
      child.state = child_state;
      child.g_cost = parent_g + primitive.duration_s;
      child.parent = index;
      child.primitive_index = static_cast<int>(primitive_index);
      child.parent_expand_seq = this_expand_seq;
      child.node_seq = next_node_seq++;
      child.edge_samples = std::move(propagated.samples);
      nodes.push_back(std::move(child));
      open.push(static_cast<int>(nodes.size() - 1));
      ++expansions;
    }
  }
  return Failure(VehiclePlanStatus::kNoSolution);
}

KinodynamicBaselinePlanner::KinodynamicBaselinePlanner(
    KinodynamicBaselineConfig config,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics)
    : config_(std::move(config)), dynamics_(dynamics) {}

std::string_view KinodynamicBaselinePlanner::id() const {
  return kVehiclePlannerKinodynamicBaseline;
}

VehiclePlanResult KinodynamicBaselinePlanner::Plan(
    const VehiclePlanRequest& request) const {
  if (!request.states_present) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  if (dynamics_ == nullptr) {
    return Failure(VehiclePlanStatus::kInvalidRequest);
  }
  return SearchKinodynamicBaseline(config_, request, *dynamics_);
}

std::shared_ptr<KinodynamicBaselinePlanner> MakeKinodynamicBaselinePlanner(
    KinodynamicBaselineConfig config,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics) {
  return std::make_shared<KinodynamicBaselinePlanner>(std::move(config),
                                                      dynamics);
}

}  // namespace intrinsic::motion_planning::vehicle
