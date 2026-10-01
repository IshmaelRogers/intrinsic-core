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
#include <deque>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"
#include "intrinsic/safety/clearance_rule.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::BodyVector;
using ::intrinsic::vehicle::SampleTimeBefore;
using ::intrinsic::vehicle::TrajectoryEngaged;
using ::intrinsic::vehicle::TrajectorySampleView;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;
using Error = TrajectoryReconstructionError;

constexpr std::string_view kFrame = "world_enu";
constexpr std::string_view kRegion = "ops-box";
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

VehiclePlanningState State(double x = 0, double y = 0, double vx = 0) {
  VehiclePlanningState state;
  state.position.x = x;
  state.position.y = y;
  state.twist.linear_x = vx;
  return state;
}

PropagationSample Sample(double time_s, const VehiclePlanningState& state) {
  PropagationSample sample;
  sample.time_s = time_s;
  sample.state = state;
  return sample;
}

// Straight edge from `start` to `end` sampled at `times`.
std::vector<PropagationSample> Edge(const VehiclePlanningState& start,
                                    const VehiclePlanningState& end,
                                    std::vector<double> times = {0.0, 0.5,
                                                                 1.0}) {
  std::vector<PropagationSample> samples;
  for (double time_s : times) {
    const double u = time_s / times.back();
    if (u == 0.0) {
      samples.push_back(Sample(time_s, start));
    } else if (u == 1.0) {
      samples.push_back(Sample(time_s, end));
    } else {
      samples.push_back(Sample(
          time_s,
          State(start.position.x + u * (end.position.x - start.position.x),
                start.position.y + u * (end.position.y - start.position.y))));
    }
  }
  return samples;
}

// Owns edge storage so the node views stay valid.
class Chain {
 public:
  void AddRoot(const VehiclePlanningState& state) { Add(-1, 0.0, state, {}); }

  void Add(int parent, double time_s, const VehiclePlanningState& state,
           std::vector<PropagationSample> edge) {
    edges_.push_back(std::move(edge));
    ReconstructionNodeView node;
    node.parent = parent;
    node.time_s = time_s;
    node.state = state;
    node.edge_samples = edges_.back();
    nodes_.push_back(node);
  }

  void SetEdge(std::size_t index, std::vector<PropagationSample> edge) {
    edges_[index] = std::move(edge);
    nodes_[index].edge_samples = edges_[index];
  }

  std::vector<ReconstructionNodeView>& nodes() { return nodes_; }
  const std::vector<ReconstructionNodeView>& nodes() const { return nodes_; }

 private:
  std::deque<std::vector<PropagationSample>> edges_;
  std::vector<ReconstructionNodeView> nodes_;
};

// Root (x = 0) to A (x = 1) to B (x = 2), one second per edge.
Chain ThreeNodeChain() {
  const VehiclePlanningState s0 = State(0), s1 = State(1), s2 = State(2);
  Chain chain;
  chain.AddRoot(s0);
  chain.Add(0, 1.0, s1, Edge(s0, s1));
  chain.Add(1, 2.0, s2, Edge(s1, s2));
  return chain;
}

VehiclePlanRequest Request(const VehiclePlanningState& start = State(0),
                           const VehiclePlanningState& goal = State(2),
                           std::string trajectory_id = {}) {
  VehiclePlanRequest request;
  request.start_label = "start";
  request.goal_label = "goal";
  request.trajectory_id = std::move(trajectory_id);
  request.states_present = true;
  request.start_state = start;
  request.goal_state = goal;
  return request;
}

std::vector<std::pair<int64_t, int32_t>> Times(const VehiclePlanResult& plan) {
  std::vector<std::pair<int64_t, int32_t>> times;
  for (const TrajectorySampleView& sample : plan.samples) {
    times.emplace_back(sample.seconds, sample.nanos);
  }
  return times;
}

std::vector<double> PositionsX(const VehiclePlanResult& plan) {
  std::vector<double> positions;
  for (const TrajectorySampleView& sample : plan.samples) {
    positions.push_back(sample.position.x);
  }
  return positions;
}

bool Accepted(const VehiclePlanResult& plan) {
  return AssessVehicleTrajectory(AsVehicleTrajectoryView(plan)).accepted;
}

bool StrictlyIncreasing(const VehiclePlanResult& plan) {
  for (std::size_t i = 1; i < plan.samples.size(); ++i) {
    if (!SampleTimeBefore(plan.samples[i - 1], plan.samples[i])) {
      return false;
    }
  }
  return true;
}

bool SameSample(const TrajectorySampleView& a, const TrajectorySampleView& b) {
  return a.seconds == b.seconds && a.nanos == b.nanos &&
         a.position.x == b.position.x && a.position.y == b.position.y &&
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

bool SamePlan(const VehiclePlanResult& a, const VehiclePlanResult& b) {
  if (a.status != b.status || a.trajectory_id != b.trajectory_id ||
      a.frame_id != b.frame_id || a.model_id != b.model_id ||
      a.samples.size() != b.samples.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    if (!SameSample(a.samples[i], b.samples[i])) {
      return false;
    }
  }
  return true;
}

void ExpectFailed(const TrajectoryReconstructionResult& result, Error error,
                  int failed_node = -2) {
  EXPECT_EQ(result.error, error);
  if (failed_node != -2) {
    EXPECT_EQ(result.failed_node, failed_node);
  }
  const VehiclePlanResult& plan = result.trajectory;
  EXPECT_EQ(plan.status, VehiclePlanStatus::kInvalidRequest);
  EXPECT_FALSE(plan.header_present);
  EXPECT_TRUE(plan.samples.empty());
  EXPECT_TRUE(plan.trajectory_id.empty());
  EXPECT_FALSE(TrajectoryEngaged(AsVehicleTrajectoryView(plan)));
  EXPECT_FALSE(Accepted(plan));
}

TrajectoryReconstructionResult Reconstruct(
    const Chain& chain, int goal, const VehiclePlanRequest& request,
    const ReconstructionOptions& options = {}) {
  return ReconstructVehicleTrajectory(chain.nodes(), goal, request, options);
}

TEST(ReconstructionTest, ChainStartsAtRequestAndIsMonotonic) {
  const Chain chain = ThreeNodeChain();
  const VehiclePlanRequest request = Request();
  const TrajectoryReconstructionResult result = Reconstruct(chain, 2, request);
  ASSERT_EQ(result.error, Error::kOk);
  EXPECT_EQ(result.failed_node, -1);
  const VehiclePlanResult& plan = result.trajectory;
  EXPECT_EQ(plan.status, VehiclePlanStatus::kOk);
  EXPECT_TRUE(Accepted(plan));
  EXPECT_TRUE(StrictlyIncreasing(plan));
  const std::vector<std::pair<int64_t, int32_t>> expected_times = {
      {0, 0}, {0, 500000000}, {1, 0}, {1, 500000000}, {2, 0}};
  EXPECT_EQ(Times(plan), expected_times);
  EXPECT_EQ(PositionsX(plan), (std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0}));
  ASSERT_FALSE(plan.samples.empty());
  const TrajectorySampleView& first = plan.samples.front();
  EXPECT_EQ(first.seconds, 0);
  EXPECT_EQ(first.nanos, 0);
  EXPECT_EQ(first.position.x, request.start_state.position.x);
  EXPECT_EQ(first.position.y, request.start_state.position.y);
  EXPECT_EQ(first.position.z, request.start_state.position.z);
  EXPECT_EQ(first.orientation.w, request.start_state.orientation.w);
  EXPECT_EQ(first.twist.linear_x, request.start_state.twist.linear_x);
  EXPECT_EQ(plan.frame_id, kFrame);
  EXPECT_EQ(plan.model_id, "uuv_planner");
  EXPECT_EQ(plan.validity_state, 1);
  EXPECT_TRUE(plan.header_present);
  EXPECT_TRUE(plan.provenance_present);
  for (const TrajectorySampleView& sample : plan.samples) {
    EXPECT_TRUE(sample.time_present);
    EXPECT_TRUE(sample.twist_present);
    EXPECT_FALSE(sample.acceleration_present);
  }
}

TEST(ReconstructionTest, DuplicateJointSampleIsDropped) {
  const Chain chain = ThreeNodeChain();
  std::size_t raw = 0;
  for (const ReconstructionNodeView& node : chain.nodes()) {
    raw += node.edge_samples.size();
  }
  EXPECT_EQ(raw, 6);
  EXPECT_EQ(Reconstruct(chain, 2, Request()).trajectory.samples.size(), 5);
}

TEST(ReconstructionTest, GoalRootIsOneSample) {
  const Chain chain = ThreeNodeChain();
  const TrajectoryReconstructionResult result =
      Reconstruct(chain, 0, Request());
  ASSERT_EQ(result.error, Error::kOk);
  EXPECT_EQ(result.trajectory.samples.size(), 1);
  EXPECT_EQ(Times(result.trajectory),
            (std::vector<std::pair<int64_t, int32_t>>{{0, 0}}));
  EXPECT_TRUE(Accepted(result.trajectory));
}

TEST(ReconstructionTest, PartialChainStopsAtGoal) {
  const Chain chain = ThreeNodeChain();
  const TrajectoryReconstructionResult result =
      Reconstruct(chain, 1, Request());
  ASSERT_EQ(result.error, Error::kOk);
  EXPECT_EQ(PositionsX(result.trajectory),
            (std::vector<double>{0.0, 0.5, 1.0}));
}

TEST(ReconstructionTest, FollowsParentLinksNotArrayOrder) {
  const VehiclePlanningState s0 = State(0), s1 = State(1), s2 = State(2);
  Chain chain;
  chain.Add(2, 2.0, s2, Edge(s1, s2));
  chain.AddRoot(s0);
  chain.Add(1, 1.0, s1, Edge(s0, s1));
  const TrajectoryReconstructionResult result =
      Reconstruct(chain, 0, Request());
  ASSERT_EQ(result.error, Error::kOk);
  EXPECT_EQ(PositionsX(result.trajectory),
            (std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0}));
}

TEST(ReconstructionTest, SiblingBranchIsIgnored) {
  Chain chain = ThreeNodeChain();
  const VehiclePlanningState side = State(0, 1);
  chain.Add(0, 1.0, side, Edge(chain.nodes()[0].state, side));
  const TrajectoryReconstructionResult main = Reconstruct(chain, 2, Request());
  ASSERT_EQ(main.error, Error::kOk);
  EXPECT_EQ(PositionsX(main.trajectory),
            (std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0}));
  const TrajectoryReconstructionResult branch =
      Reconstruct(chain, 3, Request());
  ASSERT_EQ(branch.error, Error::kOk);
  std::vector<double> ys;
  for (const TrajectorySampleView& sample : branch.trajectory.samples) {
    ys.push_back(sample.position.y);
  }
  EXPECT_EQ(ys, (std::vector<double>{0.0, 0.5, 1.0}));
}

TEST(ReconstructionTest, TrajectoryIdPrecedence) {
  const Chain chain = ThreeNodeChain();
  EXPECT_EQ(Reconstruct(chain, 2, Request()).trajectory.trajectory_id,
            kReconstructedTrajectoryId);
  ReconstructionOptions options;
  options.default_trajectory_id = "from-options";
  EXPECT_EQ(Reconstruct(chain, 2, Request(), options).trajectory.trajectory_id,
            "from-options");
  EXPECT_EQ(Reconstruct(chain, 2, Request(State(0), State(2), "from-request"),
                        options)
                .trajectory.trajectory_id,
            "from-request");
}

TEST(ReconstructionTest, BadRequest) {
  const Chain chain = ThreeNodeChain();
  VehiclePlanRequest no_states = Request();
  no_states.states_present = false;
  ExpectFailed(Reconstruct(chain, 2, no_states), Error::kBadRequest, -1);
  ExpectFailed(ReconstructVehicleTrajectory({}, 0, Request()),
               Error::kBadRequest, -1);
  ExpectFailed(Reconstruct(chain, 3, Request()), Error::kBadRequest);
  ExpectFailed(Reconstruct(chain, -1, Request()), Error::kBadRequest);
}

TEST(ReconstructionTest, BadChain) {
  Chain out_of_range = ThreeNodeChain();
  out_of_range.nodes()[1].parent = 7;
  ExpectFailed(Reconstruct(out_of_range, 2, Request()), Error::kBadChain, 1);

  Chain negative = ThreeNodeChain();
  negative.nodes()[1].parent = -2;
  ExpectFailed(Reconstruct(negative, 2, Request()), Error::kBadChain, 1);

  Chain self_loop = ThreeNodeChain();
  self_loop.nodes()[1].parent = 1;
  ExpectFailed(Reconstruct(self_loop, 2, Request()), Error::kBadChain);

  Chain cycle = ThreeNodeChain();
  cycle.nodes()[0].parent = 2;
  ExpectFailed(Reconstruct(cycle, 2, Request()), Error::kBadChain);
}

TEST(ReconstructionTest, StartMismatch) {
  const Chain chain = ThreeNodeChain();
  ExpectFailed(Reconstruct(chain, 2, Request(State(0.5))),
               Error::kStartMismatch, 0);
  ExpectFailed(Reconstruct(chain, 0, Request(State(0, 0, 1))),
               Error::kStartMismatch, 0);
  Chain shifted = ThreeNodeChain();
  shifted.nodes()[0].time_s = 0.25;
  ExpectFailed(Reconstruct(shifted, 2, Request()), Error::kStartMismatch, 0);
}

TEST(ReconstructionTest, DisjointEdge) {
  const VehiclePlanningState s0 = State(0), s1 = State(1);

  Chain empty = ThreeNodeChain();
  empty.SetEdge(1, {});
  ExpectFailed(Reconstruct(empty, 2, Request()), Error::kDisjointEdge, 1);

  Chain late = ThreeNodeChain();
  late.SetEdge(1, Edge(s0, s1, {0.25, 0.5, 1.0}));
  ExpectFailed(Reconstruct(late, 2, Request()), Error::kDisjointEdge, 1);

  Chain wrong_start = ThreeNodeChain();
  wrong_start.SetEdge(2, Edge(State(1.5), State(2)));
  ExpectFailed(Reconstruct(wrong_start, 2, Request()), Error::kDisjointEdge, 2);

  Chain wrong_end = ThreeNodeChain();
  wrong_end.SetEdge(1, Edge(s0, State(0.75)));
  ExpectFailed(Reconstruct(wrong_end, 2, Request()), Error::kDisjointEdge, 1);
}

TEST(ReconstructionTest, NonMonotonicTime) {
  const VehiclePlanningState s0 = State(0), s1 = State(1);

  for (const std::vector<double>& times :
       {std::vector<double>{0.0, 0.5, 0.5, 1.0},
        std::vector<double>{0.0, 0.75, 0.5, 1.0}}) {
    Chain chain = ThreeNodeChain();
    chain.SetEdge(1, Edge(s0, s1, times));
    ExpectFailed(Reconstruct(chain, 2, Request()), Error::kNonMonotonicTime, 1);
  }
  for (double bad : {-0.5, kNan, kInf}) {
    Chain chain = ThreeNodeChain();
    chain.SetEdge(1,
                  {Sample(0.0, s0), Sample(bad, State(0.5)), Sample(1.0, s1)});
    ExpectFailed(Reconstruct(chain, 2, Request()), Error::kNonMonotonicTime, 1);
  }

  Chain backwards = ThreeNodeChain();
  backwards.nodes()[2].time_s = 0.5;
  ExpectFailed(Reconstruct(backwards, 2, Request()), Error::kNonMonotonicTime,
               2);

  Chain nan_time = ThreeNodeChain();
  nan_time.nodes()[1].time_s = kNan;
  ExpectFailed(Reconstruct(nan_time, 2, Request()), Error::kNonMonotonicTime,
               1);
}

TEST(ReconstructionTest, SubNanosecondStepIsNotSilentlyDropped) {
  const VehiclePlanningState s0 = State(0), s1 = State(1);
  Chain chain = ThreeNodeChain();
  chain.SetEdge(1,
                {Sample(0.0, s0), Sample(1e-12, State(0.5)), Sample(1.0, s1)});
  ExpectFailed(Reconstruct(chain, 2, Request()), Error::kNonMonotonicTime, 1);
}

TEST(ReconstructionTest, ContractRejection) {
  VehiclePlanningState skewed;
  skewed.orientation.w = 2;
  Chain chain;
  chain.AddRoot(skewed);
  ExpectFailed(Reconstruct(chain, 0, Request(skewed)), Error::kRejected, -1);
}

TEST(ReconstructionTest, FailureNeverLeaksPartialTrajectory) {
  Chain chain = ThreeNodeChain();
  chain.nodes()[2].time_s = kNan;
  ExpectFailed(Reconstruct(chain, 2, Request()), Error::kNonMonotonicTime, 2);
}

TEST(ReconstructionTest, Deterministic) {
  const Chain chain = ThreeNodeChain();
  const TrajectoryReconstructionResult first = Reconstruct(chain, 2, Request());
  const TrajectoryReconstructionResult second =
      Reconstruct(chain, 2, Request());
  EXPECT_EQ(first.error, second.error);
  EXPECT_TRUE(SamePlan(first.trajectory, second.trajectory));
}

// Search integration and the replan fixture.

std::vector<::intrinsic::world::ComponentRevisionView> Components() {
  return {{std::string(::intrinsic::world::kOccupancyReferenceKind), 1}};
}

::intrinsic::world::WorldSnapshotView Snapshot(int epoch) {
  ::intrinsic::world::WorldSnapshotView view;
  view.present = true;
  view.components = Components();
  view.state_epoch = epoch;
  view.snapshot_id =
      ::intrinsic::world::ComputeSnapshotId(epoch, view.components);
  view.creation_time_present = true;
  view.creation_time = {100, 0};
  return view;
}

PropagationConfig NominalPropagation() {
  PropagationConfig config;
  config.dt_s = 0.5;
  config.max_steps = 4;
  config.mass_diag = {10, 10, 10, 10, 10, 10};
  return config;
}

BodyVector Wrench(double force) {
  BodyVector control;
  control.linear_x = force;
  return control;
}

BodyVector MaxWrench() {
  BodyVector max;
  max.linear_x = max.linear_y = max.linear_z = 20;
  max.angular_x = max.angular_y = max.angular_z = 20;
  return max;
}

// The obstacle and clearance snapshot binding one plan runs under.
struct Binding {
  int epoch = 1;
  double clearance_m = 1.0;
  VehicleStateBounds bounds{};
};

::intrinsic::safety::AabbGeofence Fence(double max_x = 2) {
  ::intrinsic::safety::AabbGeofence fence;
  fence.frame_id = kFrame;
  fence.region_id = kRegion;
  fence.min_x = -2;
  fence.max_x = max_x;
  fence.min_y = -2;
  fence.max_y = 2;
  fence.min_z = -2;
  fence.max_z = 2;
  return fence;
}

KinodynamicBaselineConfig Config(const std::vector<double>& forces,
                                 const Binding& binding, double max_x = 2) {
  KinodynamicBaselineConfig config;
  config.primitives.mode = PrimitiveGenerationMode::kCustom;
  config.primitives.max_force_torque = MaxWrench();
  config.primitives.duration_s = 1;
  for (double force : forces) {
    config.primitives.custom_controls.push_back(Wrench(force));
  }
  config.propagation = NominalPropagation();
  config.descriptor = Snapshot(binding.epoch);
  config.snapshot_id = config.descriptor.snapshot_id;
  config.bounds = binding.bounds;
  config.fence = Fence(max_x);
  config.goal_tolerance = 1e-6;
  config.position_bin_m = 0.25;
  config.max_expansions = 64;
  config.clearance_template_m = binding.clearance_m;
  return config;
}

PropagationResult PropagateForce(const VehiclePlanningState& start,
                                 double force) {
  VehicleMotionPrimitive primitive;
  primitive.id = "uuv-prim-000";
  primitive.control = Wrench(force);
  primitive.duration_s = 1;
  const ZeroForceDynamics dynamics;
  return PropagateUuvMotionPrimitive(start, primitive, NominalPropagation(),
                                     dynamics);
}

VehiclePlanningState StateAfter(const std::vector<double>& forces) {
  VehiclePlanningState state;
  for (double force : forces) {
    const PropagationResult propagated = PropagateForce(state, force);
    EXPECT_EQ(propagated.error, PropagationError::kOk);
    state = propagated.samples.back().state;
  }
  return state;
}

// CheckTrajectoryValidity on a plan's samples under `binding`.
TrajectoryValidityError CheckUnder(const Binding& binding,
                                   const VehiclePlanResult& plan) {
  const ::intrinsic::world::WorldSnapshotView view = Snapshot(binding.epoch);
  TrajectoryValidityRequest request;
  request.snapshot_id = view.snapshot_id;
  request.descriptor = view;
  request.bounds = binding.bounds;
  request.fence = Fence();
  for (const TrajectorySampleView& sample : plan.samples) {
    PropagationSample propagated;
    propagated.time_s =
        static_cast<double>(sample.seconds) + sample.nanos * 1e-9;
    propagated.state.position = sample.position;
    propagated.state.orientation = sample.orientation;
    propagated.state.twist = sample.twist;
    request.samples.push_back(propagated);
    ::intrinsic::safety::ClearanceSample clearance;
    clearance.frame_id = kFrame;
    clearance.source = ::intrinsic::safety::ClearanceSource::kObstacle;
    clearance.clearance_m = binding.clearance_m;
    clearance.snapshot_usable = true;
    request.clearance_samples.push_back(clearance);
  }
  return CheckTrajectoryValidity(request).error;
}

double MaxSpeed(const VehiclePlanResult& plan) {
  double speed = 0;
  for (const TrajectorySampleView& sample : plan.samples) {
    speed = std::max(speed, sample.twist.linear_x);
  }
  return speed;
}

TEST(SearchUsesReconstructionTest, SearchMatchesManualChainReconstruction) {
  const VehiclePlanningState goal = StateAfter({10, 10});
  const VehiclePlanRequest request = Request(State(0), goal, "two-edge");
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult searched =
      SearchKinodynamicBaseline(Config({10}, Binding{}, 5), request, dynamics);
  ASSERT_EQ(searched.status, VehiclePlanStatus::kOk);

  const PropagationResult first = PropagateForce(State(0), 10);
  const PropagationResult second =
      PropagateForce(first.samples.back().state, 10);
  Chain chain;
  chain.AddRoot(State(0));
  chain.Add(0, 1.0, first.samples.back().state, first.samples);
  chain.Add(1, 2.0, second.samples.back().state, second.samples);
  const TrajectoryReconstructionResult manual = Reconstruct(chain, 2, request);
  ASSERT_EQ(manual.error, Error::kOk);
  EXPECT_TRUE(SamePlan(searched, manual.trajectory));
}

TEST(SearchUsesReconstructionTest, StartEqualsGoalUsesBaselineId) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult result = SearchKinodynamicBaseline(
      Config({10}, Binding{}), Request(State(0), State(0)), dynamics);
  ASSERT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(result.trajectory_id, kKinodynamicBaselineTrajectoryId);
  EXPECT_EQ(result.samples.size(), 1);
  EXPECT_TRUE(Accepted(result));
}

// Same start, goal, and dynamics. Only the snapshot binding changes.
class ReplanFixtureTest : public ::testing::Test {
 protected:
  static VehiclePlanResult Plan(const Binding& binding) {
    const VehiclePlanRequest request =
        Request(State(0), StateAfter({10, -10}), "replan");
    const ZeroForceDynamics dynamics;
    return SearchKinodynamicBaseline(Config({10, -10, 5, -5}, binding), request,
                                     dynamics);
  }

  static Binding SlowEnvelope() {
    Binding binding;
    binding.epoch = 2;
    binding.bounds.max_linear_speed_present = true;
    binding.bounds.max_linear_speed_m_s = 0.6;
    return binding;
  }

  static Binding BlockedObstacle() {
    Binding binding;
    binding.epoch = 2;
    binding.clearance_m = 0.2;
    return binding;
  }
};

TEST_F(ReplanFixtureTest, FirstPlanAdmitsTheFastRoute) {
  const Binding binding;
  const VehiclePlanResult first = Plan(binding);
  ASSERT_EQ(first.status, VehiclePlanStatus::kOk);
  EXPECT_TRUE(Accepted(first));
  EXPECT_TRUE(StrictlyIncreasing(first));
  EXPECT_EQ(first.samples.size(), 5);
  EXPECT_EQ(first.samples.front().position.x, 0.0);
  EXPECT_EQ(first.samples.back().position.x, 1.0);
  EXPECT_EQ(first.samples.back().seconds, 2);
  EXPECT_EQ(first.samples.back().nanos, 0);
  EXPECT_EQ(MaxSpeed(first), 1.0);
  EXPECT_EQ(CheckUnder(binding, first), TrajectoryValidityError::kOk);
}

TEST_F(ReplanFixtureTest, UpdatedSnapshotSelectsADifferentSafeRoute) {
  const Binding updated = SlowEnvelope();
  const VehiclePlanResult first = Plan(Binding{});
  const VehiclePlanResult second = Plan(updated);
  ASSERT_EQ(first.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(second.status, VehiclePlanStatus::kOk);

  EXPECT_FALSE(SamePlan(first, second));
  EXPECT_EQ(second.samples.size(), 9);
  EXPECT_TRUE(SameSample(second.samples.front(), first.samples.front()));
  EXPECT_EQ(second.samples.back().position.x, first.samples.back().position.x);
  EXPECT_EQ(second.samples.back().twist.linear_x,
            first.samples.back().twist.linear_x);
  EXPECT_EQ(second.samples.back().seconds, 4);
  EXPECT_EQ(second.samples.back().nanos, 0);
  EXPECT_LE(MaxSpeed(second), 0.5);

  EXPECT_TRUE(Accepted(second));
  EXPECT_TRUE(StrictlyIncreasing(second));
  EXPECT_EQ(CheckUnder(updated, second), TrajectoryValidityError::kOk);
  EXPECT_EQ(CheckUnder(updated, first), TrajectoryValidityError::kBounds);
}

TEST_F(ReplanFixtureTest, UpdatedObstacleIsTypedNoSolution) {
  const Binding blocked = BlockedObstacle();
  const VehiclePlanResult first = Plan(Binding{});
  ASSERT_EQ(first.status, VehiclePlanStatus::kOk);
  const VehiclePlanResult second = Plan(blocked);
  EXPECT_EQ(second.status, VehiclePlanStatus::kNoSolution);
  EXPECT_FALSE(second.header_present);
  EXPECT_TRUE(second.samples.empty());
  EXPECT_FALSE(Accepted(second));
  EXPECT_EQ(CheckUnder(blocked, first), TrajectoryValidityError::kClearance);
}

TEST_F(ReplanFixtureTest, ReplanIsDeterministic) {
  for (const Binding& binding :
       {Binding{}, SlowEnvelope(), BlockedObstacle()}) {
    EXPECT_TRUE(SamePlan(Plan(binding), Plan(binding)));
  }
}

TEST_F(ReplanFixtureTest, SnapshotIdBindsEachPlan) {
  const KinodynamicBaselineConfig first = Config({10, -10, 5, -5}, Binding{});
  Binding updated_binding;
  updated_binding.epoch = 2;
  KinodynamicBaselineConfig updated = Config({10, -10, 5, -5}, updated_binding);
  EXPECT_NE(first.snapshot_id, updated.snapshot_id);
  updated.snapshot_id = first.snapshot_id;
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult result = SearchKinodynamicBaseline(
      updated, Request(State(0), StateAfter({10, -10})), dynamics);
  EXPECT_EQ(result.status, VehiclePlanStatus::kInvalidRequest);
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
