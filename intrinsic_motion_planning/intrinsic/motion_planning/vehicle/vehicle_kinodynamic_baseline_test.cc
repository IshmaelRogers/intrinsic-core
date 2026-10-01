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

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/fake_vehicle_planner.h"
#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::BodyVector;
using ::intrinsic::vehicle::TrajectoryEngaged;
using ::intrinsic::vehicle::TrajectorySampleView;
using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsResult;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::StatusOr;
using ::intrinsic::vehicle::dynamics::VehicleDynamics;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;
using Clock = std::chrono::steady_clock;

constexpr std::string_view kFrame = "world_enu";
constexpr std::string_view kRegion = "ops-box";

std::vector<::intrinsic::world::ComponentRevisionView> Components() {
  return {{std::string(::intrinsic::world::kOccupancyReferenceKind), 1}};
}

::intrinsic::world::WorldSnapshotView Snapshot() {
  ::intrinsic::world::WorldSnapshotView view;
  view.present = true;
  view.components = Components();
  view.state_epoch = 1;
  view.snapshot_id = ::intrinsic::world::ComputeSnapshotId(1, view.components);
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

KinodynamicBaselineConfig Config(std::vector<double> forces, double min_x = -2,
                                 double max_x = 2, double min_y = -2,
                                 double max_y = 2) {
  KinodynamicBaselineConfig config;
  config.primitives.mode = PrimitiveGenerationMode::kCustom;
  config.primitives.max_force_torque = MaxWrench();
  config.primitives.duration_s = 1;
  for (double force : forces) {
    config.primitives.custom_controls.push_back(Wrench(force));
  }
  config.propagation = NominalPropagation();
  config.descriptor = Snapshot();
  config.snapshot_id = config.descriptor.snapshot_id;
  config.fence.frame_id = kFrame;
  config.fence.region_id = kRegion;
  config.fence.min_x = min_x;
  config.fence.max_x = max_x;
  config.fence.min_y = min_y;
  config.fence.max_y = max_y;
  config.fence.min_z = -2;
  config.fence.max_z = 2;
  config.goal_tolerance = 1e-6;
  config.position_bin_m = 0.25;
  config.max_expansions = 64;
  config.clearance_template_m = 1.0;
  return config;
}

VehiclePlanningState Origin() { return {}; }

VehiclePlanRequest Request(const VehiclePlanningState& goal,
                           const VehiclePlanningState& start = {},
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

void ExpectEmpty(const VehiclePlanResult& result, VehiclePlanStatus status) {
  EXPECT_EQ(result.status, status);
  EXPECT_FALSE(result.header_present);
  EXPECT_TRUE(result.frame_id.empty());
  EXPECT_TRUE(result.trajectory_id.empty());
  EXPECT_TRUE(result.samples.empty());
  EXPECT_FALSE(result.provenance_present);
  const auto view = AsVehicleTrajectoryView(result);
  EXPECT_FALSE(TrajectoryEngaged(view));
  EXPECT_FALSE(AssessVehicleTrajectory(view).accepted);
}

void ExpectSampleState(const TrajectorySampleView& sample,
                       const VehiclePlanningState& state) {
  EXPECT_DOUBLE_EQ(sample.position.x, state.position.x);
  EXPECT_DOUBLE_EQ(sample.position.y, state.position.y);
  EXPECT_DOUBLE_EQ(sample.position.z, state.position.z);
  EXPECT_DOUBLE_EQ(sample.orientation.x, state.orientation.x);
  EXPECT_DOUBLE_EQ(sample.orientation.y, state.orientation.y);
  EXPECT_DOUBLE_EQ(sample.orientation.z, state.orientation.z);
  EXPECT_DOUBLE_EQ(sample.orientation.w, state.orientation.w);
  EXPECT_DOUBLE_EQ(sample.twist.linear_x, state.twist.linear_x);
  EXPECT_DOUBLE_EQ(sample.twist.linear_y, state.twist.linear_y);
  EXPECT_DOUBLE_EQ(sample.twist.linear_z, state.twist.linear_z);
  EXPECT_DOUBLE_EQ(sample.twist.angular_x, state.twist.angular_x);
  EXPECT_DOUBLE_EQ(sample.twist.angular_y, state.twist.angular_y);
  EXPECT_DOUBLE_EQ(sample.twist.angular_z, state.twist.angular_z);
}

void ExpectAccepted(const VehiclePlanResult& result) {
  const auto assessment =
      AssessVehicleTrajectory(AsVehicleTrajectoryView(result));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(result.frame_id, kFrame);
  EXPECT_EQ(result.model_id, "uuv_planner");
  EXPECT_TRUE(result.header_present);
  EXPECT_EQ(result.validity_state, 1);
  for (const TrajectorySampleView& sample : result.samples) {
    EXPECT_TRUE(sample.time_present);
    EXPECT_TRUE(std::isfinite(sample.position.x));
    EXPECT_TRUE(std::isfinite(sample.position.y));
    EXPECT_TRUE(std::isfinite(sample.position.z));
    EXPECT_TRUE(sample.twist_present);
  }
  for (std::size_t i = 1; i < result.samples.size(); ++i) {
    EXPECT_TRUE(::intrinsic::vehicle::SampleTimeBefore(result.samples[i - 1],
                                                       result.samples[i]));
  }
}

class CancelAfterEvaluate : public VehicleDynamics {
 public:
  explicit CancelAfterEvaluate(std::atomic<bool>* cancel) : cancel_(cancel) {}

  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt& state,
                                    const BodyWrenchRt& wrench,
                                    const EnvironmentRt& environment,
                                    Duration dt) const override {
    ++calls_;
    if (cancel_ != nullptr) {
      cancel_->store(true);
    }
    return inner_.Evaluate(state, wrench, environment, dt);
  }

  int calls() const { return calls_.load(); }

 private:
  ZeroForceDynamics inner_;
  std::atomic<bool>* cancel_;
  mutable std::atomic<int> calls_{0};
};

class CountingDynamics : public VehicleDynamics {
 public:
  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt& state,
                                    const BodyWrenchRt& wrench,
                                    const EnvironmentRt& environment,
                                    Duration dt) const override {
    ++calls_;
    return inner_.Evaluate(state, wrench, environment, dt);
  }

  int calls() const { return calls_.load(); }

 private:
  ZeroForceDynamics inner_;
  mutable std::atomic<int> calls_{0};
};

TEST(KinodynamicBaselineTest, ReachableMatchesPropagation) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const VehiclePlanningState goal = propagated.samples.back().state;
  EXPECT_DOUBLE_EQ(goal.position.x, 0.75);
  EXPECT_DOUBLE_EQ(goal.twist.linear_x, 1.0);

  const ZeroForceDynamics dynamics;
  const VehiclePlanResult result = SearchKinodynamicBaseline(
      Config({10}), Request(goal, Origin(), "reach"), dynamics);
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(result.trajectory_id, "reach");
  ExpectAccepted(result);
  ASSERT_EQ(result.samples.size(), propagated.samples.size());
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    ExpectSampleState(result.samples[i], propagated.samples[i].state);
  }
  EXPECT_EQ(result.samples[0].seconds, 0);
  EXPECT_EQ(result.samples[0].nanos, 0);
  EXPECT_EQ(result.samples[1].seconds, 0);
  EXPECT_EQ(result.samples[1].nanos, 500000000);
  EXPECT_EQ(result.samples[2].seconds, 1);
  EXPECT_EQ(result.samples[2].nanos, 0);
}

TEST(KinodynamicBaselineTest, BlockedFenceHasNoSolution) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const ZeroForceDynamics dynamics;
  const VehiclePlanRequest request = Request(propagated.samples.back().state);
  ExpectEmpty(
      SearchKinodynamicBaseline(Config({10}, -0.5, 0.3), request, dynamics),
      VehiclePlanStatus::kNoSolution);
}

TEST(KinodynamicBaselineTest, EqualCostTieBreakPicksLowerIndex) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const VehiclePlanningState goal = propagated.samples.back().state;
  const ZeroForceDynamics dynamics;
  const VehiclePlanRequest request = Request(goal);
  const VehiclePlanResult first =
      SearchKinodynamicBaseline(Config({10, 10}), request, dynamics);
  const VehiclePlanResult second =
      SearchKinodynamicBaseline(Config({10, 10}), request, dynamics);
  EXPECT_EQ(first.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    EXPECT_EQ(first.samples[i].seconds, second.samples[i].seconds);
    EXPECT_EQ(first.samples[i].nanos, second.samples[i].nanos);
    ExpectSampleState(first.samples[i], propagated.samples[i].state);
    ExpectSampleState(second.samples[i], propagated.samples[i].state);
  }

  const PropagationResult near = PropagateForce(Origin(), 10.0 + 1e-6);
  ASSERT_EQ(near.error, PropagationError::kOk);
  EXPECT_LE(Distance(near.samples.back().state, goal), 1e-6);
  const VehiclePlanResult lower =
      SearchKinodynamicBaseline(Config({10, 10.0 + 1e-6}), request, dynamics);
  const VehiclePlanResult higher_first =
      SearchKinodynamicBaseline(Config({10.0 + 1e-6, 10}), request, dynamics);
  EXPECT_EQ(lower.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(higher_first.status, VehiclePlanStatus::kOk);
  ExpectSampleState(lower.samples.back(), propagated.samples.back().state);
  ExpectSampleState(higher_first.samples.back(), near.samples.back().state);
  EXPECT_NE(lower.samples.back().position.x,
            higher_first.samples.back().position.x);
}

TEST(KinodynamicBaselineTest, DeadlineAndCancelThroughHarness) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const VehiclePlanRequest request =
      Request(propagated.samples.back().state, Origin(), "run");

  std::atomic<bool> cancel{false};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  CancelAfterEvaluate dynamics(&cancel);
  KinodynamicBaselineConfig config = Config({10});
  config.run_options = &options;
  const auto planner = MakeKinodynamicBaselinePlanner(config, &dynamics);
  ExpectEmpty(RunWithDeadline(*planner, request, options),
              VehiclePlanStatus::kCancelled);
  EXPECT_GE(dynamics.calls(), 1);

  CountingDynamics counting;
  VehiclePlanRunOptions past;
  past.deadline_present = true;
  past.deadline = Clock::now() - std::chrono::seconds(1);
  KinodynamicBaselineConfig past_config = Config({10});
  past_config.run_options = &past;
  const auto past_planner =
      MakeKinodynamicBaselinePlanner(past_config, &counting);
  ExpectEmpty(past_planner->Plan(request),
              VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_EQ(counting.calls(), 0);

  std::atomic<bool> pre_cancel{true};
  VehiclePlanRunOptions pre_options;
  pre_options.cancel = &pre_cancel;
  CountingDynamics pre_dynamics;
  KinodynamicBaselineConfig pre_config = Config({10});
  pre_config.run_options = &pre_options;
  const auto pre_planner =
      MakeKinodynamicBaselinePlanner(pre_config, &pre_dynamics);
  ExpectEmpty(RunWithDeadline(*pre_planner, request, pre_options),
              VehiclePlanStatus::kCancelled);
  EXPECT_EQ(pre_dynamics.calls(), 0);

  VehiclePlanRunOptions pre_deadline;
  pre_deadline.deadline_present = true;
  pre_deadline.deadline = Clock::now() - std::chrono::seconds(5);
  CountingDynamics pre_deadline_dynamics;
  KinodynamicBaselineConfig pre_deadline_config = Config({10});
  pre_deadline_config.run_options = &pre_deadline;
  const auto pre_deadline_planner = MakeKinodynamicBaselinePlanner(
      pre_deadline_config, &pre_deadline_dynamics);
  ExpectEmpty(RunWithDeadline(*pre_deadline_planner, request, pre_deadline),
              VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_EQ(pre_deadline_dynamics.calls(), 0);

  VehiclePlanRunOptions forward;
  forward.deadline_present = true;
  forward.deadline = Clock::now() + std::chrono::hours(1);
  KinodynamicBaselineConfig blocked_config = Config({10}, -0.5, 0.3);
  blocked_config.run_options = &forward;
  const ZeroForceDynamics forward_dynamics;
  const auto blocked =
      MakeKinodynamicBaselinePlanner(blocked_config, &forward_dynamics);
  ExpectEmpty(RunWithDeadline(*blocked, request, forward),
              VehiclePlanStatus::kNoSolution);

  KinodynamicBaselineConfig open_config = Config({10});
  open_config.run_options = &forward;
  const auto reached =
      MakeKinodynamicBaselinePlanner(open_config, &forward_dynamics);
  const VehiclePlanResult ok = RunWithDeadline(*reached, request, forward);
  EXPECT_EQ(ok.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(ok.trajectory_id, "run");
  ExpectAccepted(ok);
}

TEST(KinodynamicBaselineTest, StartEqualsGoalIsOneSample) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanningState origin = Origin();
  const VehiclePlanResult result =
      SearchKinodynamicBaseline(Config({10}), Request(origin), dynamics);
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(result.trajectory_id, kKinodynamicBaselineTrajectoryId);
  ASSERT_EQ(result.samples.size(), 1);
  EXPECT_EQ(result.samples[0].seconds, 0);
  EXPECT_EQ(result.samples[0].nanos, 0);
  ExpectSampleState(result.samples[0], origin);
  ExpectAccepted(result);
}

TEST(KinodynamicBaselineTest, EmptyPrimitivesAreInvalid) {
  KinodynamicBaselineConfig config = Config({});
  config.primitives.custom_controls.clear();
  const ZeroForceDynamics dynamics;
  ExpectEmpty(SearchKinodynamicBaseline(config, Request(Origin()), dynamics),
              VehiclePlanStatus::kInvalidRequest);
}

TEST(KinodynamicBaselineTest, InvalidStartQuaternionIsInvalid) {
  VehiclePlanningState start;
  start.orientation.w = 2;
  const ZeroForceDynamics dynamics;
  const VehiclePlanRequest request = Request(Origin(), start);
  ExpectEmpty(SearchKinodynamicBaseline(Config({10}), request, dynamics),
              VehiclePlanStatus::kInvalidRequest);
}

TEST(KinodynamicBaselineTest, MissingStatesAndNullDynamicsAreInvalid) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  VehiclePlanRequest missing;
  missing.start_label = "start";
  missing.goal_label = "goal";
  const ZeroForceDynamics dynamics;
  ExpectEmpty(SearchKinodynamicBaseline(Config({10}), missing, dynamics),
              VehiclePlanStatus::kInvalidRequest);

  const auto planner = MakeKinodynamicBaselinePlanner(Config({10}), nullptr);
  ExpectEmpty(planner->Plan(Request(propagated.samples.back().state)),
              VehiclePlanStatus::kInvalidRequest);
  ExpectEmpty(planner->Plan(missing), VehiclePlanStatus::kInvalidRequest);
}

TEST(KinodynamicBaselineTest, StaleSnapshotProbeIsInvalid) {
  KinodynamicBaselineConfig config = Config({10});
  config.assess_skew = true;
  ::intrinsic::world::ComponentTiming timing;
  timing.component_kind =
      std::string(::intrinsic::world::kOccupancyReferenceKind);
  timing.observation_time = {0, 0};
  timing.validity_horizon = {1, 0};
  config.timings = {timing};
  config.query_time = {5, 0};
  config.skew_policy.required_kinds = {
      std::string(::intrinsic::world::kOccupancyReferenceKind)};
  const ZeroForceDynamics dynamics;
  ExpectEmpty(SearchKinodynamicBaseline(config, Request(Origin()), dynamics),
              VehiclePlanStatus::kInvalidRequest);
}

TEST(KinodynamicBaselineTest, BadSnapshotProbeIsInvalid) {
  KinodynamicBaselineConfig config = Config({10});
  config.snapshot_id = "not-the-digest";
  const ZeroForceDynamics dynamics;
  ExpectEmpty(SearchKinodynamicBaseline(config, Request(Origin()), dynamics),
              VehiclePlanStatus::kInvalidRequest);
}

TEST(KinodynamicBaselineTest, Determinism) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const VehiclePlanRequest request =
      Request(propagated.samples.back().state, Origin(), "same");
  const KinodynamicBaselineConfig config = Config({10});
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult first =
      SearchKinodynamicBaseline(config, request, dynamics);
  const VehiclePlanResult second =
      SearchKinodynamicBaseline(config, request, dynamics);
  EXPECT_EQ(first.status, second.status);
  EXPECT_EQ(first.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(first.trajectory_id, second.trajectory_id);
  EXPECT_EQ(first.trajectory_id, "same");
  ASSERT_EQ(first.samples.size(), second.samples.size());
  ASSERT_GT(first.samples.size(), 0);
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    EXPECT_EQ(first.samples[i].seconds, second.samples[i].seconds);
    EXPECT_EQ(first.samples[i].nanos, second.samples[i].nanos);
    ExpectSampleState(first.samples[i], propagated.samples[i].state);
    ExpectSampleState(second.samples[i], propagated.samples[i].state);
  }
}

TEST(KinodynamicBaselineTest, RegistryRegisterAndLookup) {
  const ZeroForceDynamics dynamics;
  const auto planner = MakeKinodynamicBaselinePlanner(Config({10}), &dynamics);
  EXPECT_EQ(planner->id(), kVehiclePlannerKinodynamicBaseline);
  VehiclePlannerRegistry book;
  EXPECT_EQ(book.Register(planner), PlannerRegistryError::kOk);
  std::shared_ptr<VehiclePlanner> found;
  EXPECT_EQ(book.Lookup(kVehiclePlannerKinodynamicBaseline, &found),
            PlannerRegistryError::kOk);
  EXPECT_EQ(found, planner);
  EXPECT_EQ(found->id(), kVehiclePlannerKinodynamicBaseline);
}

TEST(KinodynamicBaselineTest, FakeIgnoresPlanningStates) {
  VehiclePlanRequest request;
  request.start_label = "dock";
  request.goal_label = "sea";
  request.states_present = true;
  request.start_state.orientation.w = 2;
  const VehiclePlanResult result = FakeVehiclePlanner().Plan(request);
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(result.samples.size(), 2);
}

TEST(KinodynamicBaselineTest, TwoEdgePathDropsDuplicateJoint) {
  const PropagationResult first = PropagateForce(Origin(), 10);
  ASSERT_EQ(first.error, PropagationError::kOk);
  const PropagationResult second =
      PropagateForce(first.samples.back().state, 10);
  ASSERT_EQ(second.error, PropagationError::kOk);
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult result = SearchKinodynamicBaseline(
      Config({10}, -2, 5), Request(second.samples.back().state), dynamics);
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  ExpectAccepted(result);
  ASSERT_EQ(result.samples.size(), 5);
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    ExpectSampleState(result.samples[i], first.samples[i].state);
  }
  for (std::size_t i = 1; i < second.samples.size(); ++i) {
    ExpectSampleState(result.samples[first.samples.size() + i - 1],
                      second.samples[i].state);
  }
  EXPECT_EQ(result.samples[0].seconds, 0);
  EXPECT_EQ(result.samples[1].nanos, 500000000);
  EXPECT_EQ(result.samples[2].seconds, 1);
  EXPECT_EQ(result.samples[3].seconds, 1);
  EXPECT_EQ(result.samples[3].nanos, 500000000);
  EXPECT_EQ(result.samples[4].seconds, 2);
}

TEST(KinodynamicBaselineTest, ExpansionBudgetStopsBeforeUndequeuedGoal) {
  const PropagationResult propagated = PropagateForce(Origin(), 10);
  ASSERT_EQ(propagated.error, PropagationError::kOk);
  const VehiclePlanRequest request = Request(propagated.samples.back().state);
  const ZeroForceDynamics dynamics;
  KinodynamicBaselineConfig exhausted = Config({10});
  exhausted.max_expansions = 1;
  ExpectEmpty(SearchKinodynamicBaseline(exhausted, request, dynamics),
              VehiclePlanStatus::kNoSolution);
  KinodynamicBaselineConfig reached = Config({10});
  reached.max_expansions = 2;
  EXPECT_EQ(SearchKinodynamicBaseline(reached, request, dynamics).status,
            VehiclePlanStatus::kOk);
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
