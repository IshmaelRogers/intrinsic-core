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

#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/fake_vehicle_planner.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::TrajectoryContractError;
using ::intrinsic::vehicle::TrajectoryEngaged;
using ::intrinsic::vehicle::TrajectorySampleView;

class NamedPlanner : public VehiclePlanner {
 public:
  NamedPlanner(std::string id, VehiclePlanStatus status)
      : id_(std::move(id)), status_(status) {}

  std::string_view id() const override { return id_; }

  VehiclePlanResult Plan(const VehiclePlanRequest& /*request*/) const override {
    VehiclePlanResult result;
    result.status = status_;
    return result;
  }

 private:
  std::string id_;
  VehiclePlanStatus status_;
};

VehiclePlanRequest Request(std::string start, std::string goal,
                           std::string trajectory_id = "") {
  VehiclePlanRequest request;
  request.start_label = std::move(start);
  request.goal_label = std::move(goal);
  request.trajectory_id = std::move(trajectory_id);
  return request;
}

void ExpectFixture(const VehiclePlanResult& result, std::string_view id) {
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_TRUE(result.header_present);
  EXPECT_TRUE(result.validity_present);
  EXPECT_EQ(result.validity_state, 1);
  EXPECT_EQ(result.frame_id, "world_enu");
  EXPECT_EQ(result.trajectory_id, id);
  ASSERT_EQ(result.samples.size(), 2);

  const TrajectorySampleView& first = result.samples[0];
  EXPECT_TRUE(first.time_present);
  EXPECT_EQ(first.seconds, 1700000010);
  EXPECT_EQ(first.nanos, 0);
  EXPECT_EQ(first.position.x, 1.0);
  EXPECT_EQ(first.position.y, 2.0);
  EXPECT_EQ(first.position.z, -3.0);
  EXPECT_EQ(first.orientation.x, 0.0);
  EXPECT_EQ(first.orientation.y, 0.0);
  EXPECT_EQ(first.orientation.z, 0.0);
  EXPECT_EQ(first.orientation.w, 1.0);
  EXPECT_TRUE(first.twist_present);
  EXPECT_EQ(first.twist.linear_x, 0.5);
  EXPECT_EQ(first.twist.angular_z, 0.125);

  const TrajectorySampleView& second = result.samples[1];
  EXPECT_TRUE(second.time_present);
  EXPECT_EQ(second.seconds, 1700000011);
  EXPECT_EQ(second.nanos, 500000000);
  EXPECT_EQ(second.position.x, 1.5);
  EXPECT_EQ(second.position.y, 2.0);
  EXPECT_EQ(second.position.z, -3.0);
  EXPECT_EQ(second.orientation.w, 1.0);
  EXPECT_TRUE(second.twist_present);
  EXPECT_EQ(second.twist.linear_x, 0.5);
  EXPECT_EQ(second.twist.angular_z, 0.0);

  EXPECT_TRUE(result.provenance_present);
  EXPECT_EQ(result.model_id, "uuv_planner");

  const auto assessment =
      AssessVehicleTrajectory(AsVehicleTrajectoryView(result));
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

void ExpectAbsent(const VehiclePlanResult& result, VehiclePlanStatus status) {
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

TEST(VehiclePlannerRegistryTest, RegisterLookupPlanSuccess) {
  VehiclePlannerRegistry registry;
  auto planner = MakeFakeVehiclePlanner();
  EXPECT_EQ(planner->id(), kVehiclePlannerFake);
  EXPECT_EQ(registry.Register(planner), PlannerRegistryError::kOk);
  EXPECT_EQ(registry.size(), 1);

  std::shared_ptr<VehiclePlanner> found;
  EXPECT_EQ(registry.Lookup(kVehiclePlannerFake, &found),
            PlannerRegistryError::kOk);
  ASSERT_EQ(found, planner);
  ExpectFixture(found->Plan(Request("start", "goal")), "fake-trajectory");
}

TEST(VehiclePlannerRegistryTest, DuplicateIdKeepsFirst) {
  VehiclePlannerRegistry registry;
  auto first = MakeFakeVehiclePlanner();
  FakeVehiclePlannerConfig rejected;
  rejected.status = VehiclePlanStatus::kNoSolution;
  auto second = MakeFakeVehiclePlanner(rejected);

  EXPECT_EQ(registry.Register(first), PlannerRegistryError::kOk);
  EXPECT_EQ(registry.Register(second), PlannerRegistryError::kDuplicateId);
  EXPECT_EQ(registry.size(), 1);

  std::shared_ptr<VehiclePlanner> found;
  EXPECT_EQ(registry.Lookup(kVehiclePlannerFake, &found),
            PlannerRegistryError::kOk);
  EXPECT_EQ(found, first);
  ExpectFixture(found->Plan(Request("start", "goal")), "fake-trajectory");
}

TEST(VehiclePlannerRegistryTest, UnknownIdIsNotFound) {
  VehiclePlannerRegistry registry;
  EXPECT_EQ(registry.Register(MakeFakeVehiclePlanner()),
            PlannerRegistryError::kOk);

  std::shared_ptr<VehiclePlanner> sentinel = MakeFakeVehiclePlanner();
  std::shared_ptr<VehiclePlanner> found = sentinel;
  EXPECT_EQ(registry.Lookup("ai.intrinsic.vehicle_planner.kinodynamic_baseline",
                            &found),
            PlannerRegistryError::kNotFound);
  EXPECT_EQ(found, sentinel);
  EXPECT_EQ(registry.Lookup("ai.intrinsic.capability.planning", &found),
            PlannerRegistryError::kNotFound);
  EXPECT_EQ(found, sentinel);
  EXPECT_EQ(registry.size(), 1);

  VehiclePlannerRegistry empty;
  std::shared_ptr<VehiclePlanner> missing;
  EXPECT_EQ(empty.Lookup(kVehiclePlannerFake, &missing),
            PlannerRegistryError::kNotFound);
  EXPECT_EQ(missing, nullptr);
  EXPECT_EQ(empty.size(), 0);
}

TEST(VehiclePlannerRegistryTest, EmptyIdAndNullPlanner) {
  VehiclePlannerRegistry registry;
  auto empty = std::make_shared<NamedPlanner>("", VehiclePlanStatus::kOk);
  EXPECT_EQ(registry.Register(empty), PlannerRegistryError::kEmptyId);
  EXPECT_EQ(registry.Register(nullptr), PlannerRegistryError::kNullPlanner);
  EXPECT_EQ(registry.size(), 0);

  std::shared_ptr<VehiclePlanner> sentinel = MakeFakeVehiclePlanner();
  std::shared_ptr<VehiclePlanner> found = sentinel;
  EXPECT_EQ(registry.Lookup("", &found), PlannerRegistryError::kEmptyId);
  EXPECT_EQ(found, sentinel);

  EXPECT_EQ(registry.Register(MakeFakeVehiclePlanner()),
            PlannerRegistryError::kOk);
  EXPECT_EQ(registry.size(), 1);
  EXPECT_EQ(registry.Lookup("", &found), PlannerRegistryError::kEmptyId);
  EXPECT_EQ(found, sentinel);
}

TEST(FakeVehiclePlannerTest, ConfiguredFailureAndInvalidLabels) {
  FakeVehiclePlannerConfig no_solution;
  no_solution.status = VehiclePlanStatus::kNoSolution;
  FakeVehiclePlanner blocked(no_solution);
  ExpectAbsent(blocked.Plan(Request("start", "goal")),
               VehiclePlanStatus::kNoSolution);

  FakeVehiclePlannerConfig invalid;
  invalid.status = VehiclePlanStatus::kInvalidRequest;
  FakeVehiclePlanner configured_invalid(invalid);
  ExpectAbsent(configured_invalid.Plan(Request("start", "goal")),
               VehiclePlanStatus::kInvalidRequest);

  FakeVehiclePlanner ok;
  ExpectAbsent(ok.Plan(Request("", "goal")),
               VehiclePlanStatus::kInvalidRequest);
  ExpectAbsent(ok.Plan(Request("start", "")),
               VehiclePlanStatus::kInvalidRequest);
  ExpectAbsent(ok.Plan(Request("", "")), VehiclePlanStatus::kInvalidRequest);

  // Empty labels win over the equal-label failure switch.
  FakeVehiclePlannerConfig fail_equal;
  fail_equal.fail_when_start_equals_goal = true;
  FakeVehiclePlanner strict(fail_equal);
  ExpectAbsent(strict.Plan(Request("", "")),
               VehiclePlanStatus::kInvalidRequest);
}

TEST(FakeVehiclePlannerTest, StartEqualsGoal) {
  FakeVehiclePlanner ok;
  ExpectFixture(ok.Plan(Request("dock", "dock")), "fake-trajectory");

  FakeVehiclePlannerConfig config;
  config.fail_when_start_equals_goal = true;
  FakeVehiclePlanner strict(config);
  ExpectAbsent(strict.Plan(Request("dock", "dock")),
               VehiclePlanStatus::kNoSolution);
  ExpectFixture(strict.Plan(Request("dock", "sea")), "fake-trajectory");
}

TEST(FakeVehiclePlannerTest, DeterministicSuccessFixture) {
  FakeVehiclePlanner first;
  FakeVehiclePlanner second;
  const VehiclePlanRequest request = Request("start", "goal", "traj_alpha");
  const VehiclePlanResult a = first.Plan(request);
  const VehiclePlanResult b = first.Plan(request);
  const VehiclePlanResult c = second.Plan(request);
  ExpectFixture(a, "traj_alpha");
  ExpectFixture(b, "traj_alpha");
  ExpectFixture(c, "traj_alpha");
  EXPECT_EQ(a.trajectory_id, b.trajectory_id);
  EXPECT_EQ(a.trajectory_id, c.trajectory_id);
  ASSERT_EQ(a.samples.size(), b.samples.size());
  ASSERT_EQ(a.samples.size(), c.samples.size());
  for (size_t i = 0; i < a.samples.size(); ++i) {
    EXPECT_EQ(a.samples[i].seconds, b.samples[i].seconds);
    EXPECT_EQ(a.samples[i].nanos, b.samples[i].nanos);
    EXPECT_EQ(a.samples[i].position.x, b.samples[i].position.x);
    EXPECT_EQ(a.samples[i].position.y, b.samples[i].position.y);
    EXPECT_EQ(a.samples[i].position.z, b.samples[i].position.z);
    EXPECT_EQ(a.samples[i].seconds, c.samples[i].seconds);
    EXPECT_EQ(a.samples[i].nanos, c.samples[i].nanos);
    EXPECT_EQ(a.samples[i].position.x, c.samples[i].position.x);
  }
  EXPECT_EQ(kFakeVehicleTrajectoryId, "fake-trajectory");
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
