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

#include "intrinsic/motion_planning/vehicle/desired_motion_plan_adapter.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::DesiredMotionView;
using ::intrinsic::vehicle::ObjectiveKind;
using ::intrinsic::vehicle::TrajectorySampleView;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;

constexpr int kStateValid = 1;
constexpr int kStateInvalid = 2;
// A wire value the contract policy does not classify as valid or invalid.
constexpr int kStateDegraded = 3;

constexpr std::string_view kEnu = "world_enu";
constexpr std::string_view kRegion = "ops-box";

uint64_t Bits(double value) { return std::bit_cast<uint64_t>(value); }

void ExpectBitEq(double actual, double expected) {
  EXPECT_EQ(Bits(actual), Bits(expected))
      << "actual=" << actual << " expected=" << expected;
}

void ExpectStateBitEq(const VehiclePlanningState& actual,
                      const VehiclePlanningState& expected) {
  ExpectBitEq(actual.position.x, expected.position.x);
  ExpectBitEq(actual.position.y, expected.position.y);
  ExpectBitEq(actual.position.z, expected.position.z);
  ExpectBitEq(actual.orientation.x, expected.orientation.x);
  ExpectBitEq(actual.orientation.y, expected.orientation.y);
  ExpectBitEq(actual.orientation.z, expected.orientation.z);
  ExpectBitEq(actual.orientation.w, expected.orientation.w);
  ExpectBitEq(actual.twist.linear_x, expected.twist.linear_x);
  ExpectBitEq(actual.twist.linear_y, expected.twist.linear_y);
  ExpectBitEq(actual.twist.linear_z, expected.twist.linear_z);
  ExpectBitEq(actual.twist.angular_x, expected.twist.angular_x);
  ExpectBitEq(actual.twist.angular_y, expected.twist.angular_y);
  ExpectBitEq(actual.twist.angular_z, expected.twist.angular_z);
}

void ExpectRequestBitEq(const VehiclePlanRequest& actual,
                        const VehiclePlanRequest& expected) {
  EXPECT_EQ(actual.start_label, expected.start_label);
  EXPECT_EQ(actual.goal_label, expected.goal_label);
  EXPECT_EQ(actual.trajectory_id, expected.trajectory_id);
  EXPECT_EQ(actual.states_present, expected.states_present);
  ExpectStateBitEq(actual.start_state, expected.start_state);
  ExpectStateBitEq(actual.goal_state, expected.goal_state);
}

void ExpectDefaultRequest(const VehiclePlanRequest& request) {
  EXPECT_TRUE(request.start_label.empty());
  EXPECT_TRUE(request.goal_label.empty());
  EXPECT_TRUE(request.trajectory_id.empty());
  EXPECT_FALSE(request.states_present);
  ExpectRequestBitEq(request, VehiclePlanRequest{});
}

DesiredMotionView PoseMotion(std::string_view frame_id = kEnu) {
  DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = kStateValid;
  motion.frame_id = frame_id;
  motion.objective = ObjectiveKind::kPose;
  motion.position = {1.0, 0.0, 0.0};
  motion.orientation = {0.0, 0.0, 0.0, 1.0};
  return motion;
}

DesiredMotionView TwistMotion(std::string_view frame_id) {
  DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = kStateValid;
  motion.frame_id = frame_id;
  motion.objective = ObjectiveKind::kTwist;
  motion.twist.linear_x = 0.5;
  return motion;
}

DesiredMotionView TrajectoryMotion() {
  DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = kStateValid;
  motion.frame_id = kEnu;
  motion.objective = ObjectiveKind::kTrajectory;
  motion.trajectory_id = "reference-trajectory";
  return motion;
}

VehiclePlanningState GoodStart() { return {}; }

VehiclePlanningState NanStart() {
  VehiclePlanningState start;
  start.position.x = std::numeric_limits<double>::quiet_NaN();
  return start;
}

VehiclePlanningState NormTwoStart() {
  VehiclePlanningState start;
  start.orientation = {0.0, 0.0, 0.0, 2.0};
  return start;
}

struct Row {
  const char* name;
  DesiredMotionView motion;
  VehiclePlanningState start;
  DesiredMotionPlanAdapterStatus expected;
};

TEST(DesiredMotionPlanAdapterTest, P1EveryStatus) {
  DesiredMotionView unnormalised = PoseMotion();
  unnormalised.orientation = {0.0, 0.0, 0.0, 1.1};

  DesiredMotionView nan_position = PoseMotion();
  nan_position.position.y = std::numeric_limits<double>::quiet_NaN();

  DesiredMotionView degraded = PoseMotion();
  degraded.validity_state = kStateDegraded;

  DesiredMotionView invalid = PoseMotion();
  invalid.validity_state = kStateInvalid;

  DesiredMotionView absent_validity = PoseMotion();
  absent_validity.validity_present = false;

  using Status = DesiredMotionPlanAdapterStatus;
  const std::vector<Row> rows = {
      {"ok", PoseMotion(), GoodStart(), Status::kOk},
      {"empty view", DesiredMotionView{}, GoodStart(), Status::kInvalidMotion},
      {"unnormalised quaternion", unnormalised, GoodStart(),
       Status::kInvalidMotion},
      {"nan position", nan_position, GoodStart(), Status::kInvalidMotion},
      {"degraded validity", degraded, GoodStart(), Status::kInvalidMotion},
      {"invalid validity", invalid, GoodStart(), Status::kInvalidMotion},
      {"absent validity", absent_validity, GoodStart(), Status::kInvalidMotion},
      {"twist in body", TwistMotion("body"), GoodStart(),
       Status::kUnsupportedObjective},
      {"twist in world_enu", TwistMotion("world_enu"), GoodStart(),
       Status::kInvalidMotion},
      {"trajectory reference", TrajectoryMotion(), GoodStart(),
       Status::kUnsupportedObjective},
      {"world_ned pose", PoseMotion("world_ned"), GoodStart(),
       Status::kUnsupportedFrame},
      {"map pose", PoseMotion("map"), GoodStart(), Status::kUnsupportedFrame},
      {"start with nan", PoseMotion(), NanStart(), Status::kInvalidStart},
      {"start with quaternion norm 2", PoseMotion(), NormTwoStart(),
       Status::kInvalidStart},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const DesiredMotionPlanAdapterResult result =
        AdaptDesiredMotionToPlanRequest(row.motion, row.start, "id");
    EXPECT_EQ(result.status, row.expected);
    if (row.expected != Status::kOk) {
      ExpectDefaultRequest(result.request);
    } else {
      EXPECT_TRUE(result.request.states_present);
    }
  }
}

TEST(DesiredMotionPlanAdapterTest, P2FirstDefectWins) {
  const DesiredMotionPlanAdapterResult twist_and_bad_start =
      AdaptDesiredMotionToPlanRequest(TwistMotion("body"), NanStart(), "id");
  EXPECT_EQ(twist_and_bad_start.status,
            DesiredMotionPlanAdapterStatus::kUnsupportedObjective);
  ExpectDefaultRequest(twist_and_bad_start.request);

  const DesiredMotionPlanAdapterResult ned_and_bad_start =
      AdaptDesiredMotionToPlanRequest(PoseMotion("world_ned"), NormTwoStart(),
                                      "id");
  EXPECT_EQ(ned_and_bad_start.status,
            DesiredMotionPlanAdapterStatus::kUnsupportedFrame);
  ExpectDefaultRequest(ned_and_bad_start.request);

  DesiredMotionView degraded_ned = PoseMotion("world_ned");
  degraded_ned.validity_state = kStateDegraded;
  const DesiredMotionPlanAdapterResult invalid_first =
      AdaptDesiredMotionToPlanRequest(degraded_ned, NanStart(), "id");
  EXPECT_EQ(invalid_first.status,
            DesiredMotionPlanAdapterStatus::kInvalidMotion);
  ExpectDefaultRequest(invalid_first.request);
}

TEST(DesiredMotionPlanAdapterTest, P3BitExactOk) {
  VehiclePlanningState start;
  start.position = {0.1, -0.0, -3.5};
  start.orientation = {0.0, 0.0, -0.7071067811865476, 0.7071067811865476};
  start.twist = {0.25, 0.0, -0.0, 0.0, 0.0, 0.125};

  DesiredMotionView motion = PoseMotion();
  motion.position = {1.5, -2.25, -4.0};
  motion.orientation = {0.0, 0.0, 0.0, -1.0};

  const DesiredMotionPlanAdapterResult result =
      AdaptDesiredMotionToPlanRequest(motion, start, "uuv-ref-p3");
  ASSERT_EQ(result.status, DesiredMotionPlanAdapterStatus::kOk);
  const VehiclePlanRequest& request = result.request;
  EXPECT_TRUE(request.states_present);
  EXPECT_TRUE(request.start_label.empty());
  EXPECT_TRUE(request.goal_label.empty());
  EXPECT_EQ(request.trajectory_id, "uuv-ref-p3");

  ExpectStateBitEq(request.start_state, start);
  ExpectBitEq(request.start_state.position.y, -0.0);
  ExpectBitEq(request.start_state.twist.linear_z, -0.0);

  ExpectBitEq(request.goal_state.position.x, 1.5);
  ExpectBitEq(request.goal_state.position.y, -2.25);
  ExpectBitEq(request.goal_state.position.z, -4.0);
  ExpectBitEq(request.goal_state.orientation.x, 0.0);
  ExpectBitEq(request.goal_state.orientation.y, 0.0);
  ExpectBitEq(request.goal_state.orientation.z, 0.0);
  ExpectBitEq(request.goal_state.orientation.w, -1.0);
  EXPECT_EQ(Bits(request.goal_state.twist.linear_x), 0u);
  EXPECT_EQ(Bits(request.goal_state.twist.linear_y), 0u);
  EXPECT_EQ(Bits(request.goal_state.twist.linear_z), 0u);
  EXPECT_EQ(Bits(request.goal_state.twist.angular_x), 0u);
  EXPECT_EQ(Bits(request.goal_state.twist.angular_y), 0u);
  EXPECT_EQ(Bits(request.goal_state.twist.angular_z), 0u);
}

TEST(DesiredMotionPlanAdapterTest, P3GoalTwistIgnoresMotionTwist) {
  DesiredMotionView motion = PoseMotion();
  motion.twist.linear_x = 3.0;
  motion.confidence_present = true;
  motion.confidence = 0.5;
  motion.horizon_present = true;
  motion.horizon_seconds = 5;
  motion.provenance_present = true;
  motion.model_id = "some-model";
  VehiclePlanningState start;
  start.twist.linear_x = 2.0;

  const DesiredMotionPlanAdapterResult result =
      AdaptDesiredMotionToPlanRequest(motion, start, "id");
  ASSERT_EQ(result.status, DesiredMotionPlanAdapterStatus::kOk);
  EXPECT_EQ(Bits(result.request.goal_state.twist.linear_x), 0u);
  ExpectBitEq(result.request.start_state.twist.linear_x, 2.0);
  EXPECT_TRUE(result.request.start_label.empty());
  EXPECT_TRUE(result.request.goal_label.empty());
}

TEST(DesiredMotionPlanAdapterTest, P4EmptyTrajectoryIdIsAllowed) {
  const DesiredMotionPlanAdapterResult result =
      AdaptDesiredMotionToPlanRequest(PoseMotion(), GoodStart(), "");
  EXPECT_EQ(result.status, DesiredMotionPlanAdapterStatus::kOk);
  EXPECT_TRUE(result.request.trajectory_id.empty());
  EXPECT_TRUE(result.request.states_present);
}

// Private copy of the README "Replan fixture" epoch-1 binding. Mirrors the
// Config() and Snapshot() helpers in vehicle_kinodynamic_baseline_test.cc.
::intrinsic::world::WorldSnapshotView Epoch1Snapshot() {
  ::intrinsic::world::WorldSnapshotView view;
  view.present = true;
  view.components = {
      {std::string(::intrinsic::world::kOccupancyReferenceKind), 1}};
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

KinodynamicBaselineConfig ReplanConfig(const std::vector<double>& forces) {
  KinodynamicBaselineConfig config;
  config.primitives.mode = PrimitiveGenerationMode::kCustom;
  config.primitives.max_force_torque = MaxWrench();
  config.primitives.duration_s = 1;
  for (double force : forces) {
    config.primitives.custom_controls.push_back(Wrench(force));
  }
  config.propagation = NominalPropagation();
  config.descriptor = Epoch1Snapshot();
  config.snapshot_id = config.descriptor.snapshot_id;
  config.fence.frame_id = kEnu;
  config.fence.region_id = kRegion;
  config.fence.min_x = -2;
  config.fence.max_x = 2;
  config.fence.min_y = -2;
  config.fence.max_y = 2;
  config.fence.min_z = -2;
  config.fence.max_z = 2;
  config.goal_tolerance = 1e-6;
  config.position_bin_m = 0.25;
  config.max_expansions = 64;
  config.clearance_template_m = 1.0;
  return config;
}

void ExpectSampleBitEq(const TrajectorySampleView& actual,
                       const TrajectorySampleView& expected) {
  EXPECT_EQ(actual.time_present, expected.time_present);
  EXPECT_EQ(actual.seconds, expected.seconds);
  EXPECT_EQ(actual.nanos, expected.nanos);
  ExpectBitEq(actual.position.x, expected.position.x);
  ExpectBitEq(actual.position.y, expected.position.y);
  ExpectBitEq(actual.position.z, expected.position.z);
  ExpectBitEq(actual.orientation.x, expected.orientation.x);
  ExpectBitEq(actual.orientation.y, expected.orientation.y);
  ExpectBitEq(actual.orientation.z, expected.orientation.z);
  ExpectBitEq(actual.orientation.w, expected.orientation.w);
  EXPECT_EQ(actual.twist_present, expected.twist_present);
  ExpectBitEq(actual.twist.linear_x, expected.twist.linear_x);
  ExpectBitEq(actual.twist.linear_y, expected.twist.linear_y);
  ExpectBitEq(actual.twist.linear_z, expected.twist.linear_z);
  ExpectBitEq(actual.twist.angular_x, expected.twist.angular_x);
  ExpectBitEq(actual.twist.angular_y, expected.twist.angular_y);
  ExpectBitEq(actual.twist.angular_z, expected.twist.angular_z);
}

TEST(DesiredMotionPlanAdapterTest, P5ReplanFixtureEpoch1) {
  DesiredMotionView motion = PoseMotion();
  motion.position = {1.0, 0.0, 0.0};
  motion.orientation = {0.0, 0.0, 0.0, 1.0};

  const DesiredMotionPlanAdapterResult adapted =
      AdaptDesiredMotionToPlanRequest(motion, VehiclePlanningState{},
                                      "uuv-ref-epoch-1");
  ASSERT_EQ(adapted.status, DesiredMotionPlanAdapterStatus::kOk);

  const ZeroForceDynamics dynamics;
  const auto planner =
      MakeKinodynamicBaselinePlanner(ReplanConfig({10, -10, 5, -5}), &dynamics);
  const VehiclePlanResult result = planner->Plan(adapted.request);
  ASSERT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(result.trajectory_id, "uuv-ref-epoch-1");
  ASSERT_EQ(result.samples.size(), 5u);

  const int64_t expected_seconds[] = {0, 0, 1, 1, 2};
  const int32_t expected_nanos[] = {0, 500000000, 0, 500000000, 0};
  double peak_linear_x = 0.0;
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    EXPECT_EQ(result.samples[i].seconds, expected_seconds[i]);
    EXPECT_EQ(result.samples[i].nanos, expected_nanos[i]);
    peak_linear_x = std::max(peak_linear_x, result.samples[i].twist.linear_x);
  }
  const TrajectorySampleView& last = result.samples.back();
  EXPECT_EQ(last.position.x, 1.0);
  EXPECT_EQ(last.twist.linear_x, 0.0);
  EXPECT_EQ(last.twist.linear_y, 0.0);
  EXPECT_EQ(last.twist.linear_z, 0.0);
  EXPECT_EQ(last.twist.angular_x, 0.0);
  EXPECT_EQ(last.twist.angular_y, 0.0);
  EXPECT_EQ(last.twist.angular_z, 0.0);
  EXPECT_EQ(peak_linear_x, 1.0);

  VehiclePlanRequest hand_built;
  hand_built.start_label = "start";
  hand_built.goal_label = "goal";
  hand_built.trajectory_id = "uuv-ref-epoch-1";
  hand_built.states_present = true;
  hand_built.goal_state.position = {1.0, 0.0, 0.0};
  const VehiclePlanResult expected = planner->Plan(hand_built);
  ASSERT_EQ(expected.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(result.samples.size(), expected.samples.size());
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    SCOPED_TRACE(i);
    ExpectSampleBitEq(result.samples[i], expected.samples[i]);
  }
  EXPECT_EQ(result.trajectory_id, expected.trajectory_id);
  EXPECT_EQ(result.frame_id, expected.frame_id);
  EXPECT_EQ(result.model_id, expected.model_id);

  EXPECT_TRUE(
      AssessVehicleTrajectory(AsVehicleTrajectoryView(result)).accepted);
}

TEST(DesiredMotionPlanAdapterTest, P6Deterministic) {
  VehiclePlanningState start;
  start.position = {0.1, -0.0, -3.5};
  start.orientation = {0.0, 0.0, -0.7071067811865476, 0.7071067811865476};
  start.twist = {0.25, 0.0, -0.0, 0.0, 0.0, 0.125};
  DesiredMotionView motion = PoseMotion();
  motion.position = {1.5, -2.25, -4.0};
  motion.orientation = {0.0, 0.0, 0.0, -1.0};

  const DesiredMotionPlanAdapterResult first =
      AdaptDesiredMotionToPlanRequest(motion, start, "same");
  const DesiredMotionPlanAdapterResult second =
      AdaptDesiredMotionToPlanRequest(motion, start, "same");
  ASSERT_EQ(first.status, DesiredMotionPlanAdapterStatus::kOk);
  EXPECT_EQ(first.status, second.status);
  ExpectRequestBitEq(first.request, second.request);
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
