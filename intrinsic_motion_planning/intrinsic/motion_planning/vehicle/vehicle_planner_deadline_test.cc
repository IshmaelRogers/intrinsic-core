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

#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string_view>
#include <thread>
#include <utility>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/fake_vehicle_planner.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::TrajectoryEngaged;
using Clock = std::chrono::steady_clock;

VehiclePlanRequest Request() {
  VehiclePlanRequest request;
  request.start_label = "start";
  request.goal_label = "goal";
  return request;
}

void ExpectNoTrajectory(const VehiclePlanResult& result,
                        VehiclePlanStatus status) {
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

struct CooperativeConfig {
  // Number of simulated work steps before the inner result is returned.
  int steps = 3;
  // Called at the start of each step with the zero-based step index. Tests
  // use it to set the cancel flag or advance the clock deterministically.
  std::function<void(int)> on_step;
  // Time source used for the mid-run deadline check. Defaults to the steady
  // clock.
  std::function<Clock::time_point()> now = [] { return Clock::now(); };
  // Optional real sleep per step for tests that cancel from another thread.
  // Never the only timing mechanism: timeouts use `now`.
  std::chrono::milliseconds step_sleep{0};
};

// Test-only stand-in for a planner that polls cancel and deadline between
// work steps. Not registered under any production id. After the last step it
// returns whatever `inner` returns.
class CooperativeVehiclePlanner : public VehiclePlanner {
 public:
  CooperativeVehiclePlanner(std::shared_ptr<VehiclePlanner> inner,
                            const VehiclePlanRunOptions& options,
                            CooperativeConfig config)
      : inner_(std::move(inner)),
        options_(options),
        config_(std::move(config)) {}

  std::string_view id() const override { return "test.cooperative"; }

  VehiclePlanResult Plan(const VehiclePlanRequest& request) const override {
    plan_calls_.fetch_add(1);
    for (int step = 0; step < config_.steps; ++step) {
      if (config_.on_step) config_.on_step(step);
      steps_started_.fetch_add(1);
      if (options_.cancel != nullptr && options_.cancel->load()) {
        VehiclePlanResult result;
        result.status = VehiclePlanStatus::kCancelled;
        return result;
      }
      if (options_.deadline_present && config_.now() >= options_.deadline) {
        VehiclePlanResult result;
        result.status = VehiclePlanStatus::kDeadlineExceeded;
        return result;
      }
      if (config_.step_sleep.count() > 0) {
        std::this_thread::sleep_for(config_.step_sleep);
      }
    }
    return inner_->Plan(request);
  }

  int plan_calls() const { return plan_calls_.load(); }
  int steps_started() const { return steps_started_.load(); }

 private:
  std::shared_ptr<VehiclePlanner> inner_;
  VehiclePlanRunOptions options_;
  CooperativeConfig config_;
  mutable std::atomic<int> plan_calls_{0};
  mutable std::atomic<int> steps_started_{0};
};

class CountingPlanner : public VehiclePlanner {
 public:
  explicit CountingPlanner(std::shared_ptr<VehiclePlanner> inner)
      : inner_(std::move(inner)) {}

  std::string_view id() const override { return "test.counting"; }
  VehiclePlanResult Plan(const VehiclePlanRequest& request) const override {
    calls_.fetch_add(1);
    return inner_->Plan(request);
  }
  int calls() const { return calls_.load(); }

 private:
  std::shared_ptr<VehiclePlanner> inner_;
  mutable std::atomic<int> calls_{0};
};

TEST(VehiclePlannerDeadlineTest, StatusValuesAreStable) {
  EXPECT_EQ(static_cast<int>(VehiclePlanStatus::kOk), 0);
  EXPECT_EQ(static_cast<int>(VehiclePlanStatus::kNoSolution), 1);
  EXPECT_EQ(static_cast<int>(VehiclePlanStatus::kInvalidRequest), 2);
  EXPECT_EQ(static_cast<int>(VehiclePlanStatus::kCancelled), 3);
  EXPECT_EQ(static_cast<int>(VehiclePlanStatus::kDeadlineExceeded), 4);
}

TEST(VehiclePlannerDeadlineTest, AlreadyExpiredDeadline) {
  CountingPlanner planner(MakeFakeVehiclePlanner());
  VehiclePlanRunOptions options;
  options.deadline_present = true;
  options.deadline = Clock::now() - std::chrono::seconds(1);

  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_EQ(planner.calls(), 0);

  options.deadline = Clock::now();
  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_EQ(planner.calls(), 0);
}

TEST(VehiclePlannerDeadlineTest, PreCancelledSkipsPlanner) {
  CountingPlanner planner(MakeFakeVehiclePlanner());
  std::atomic<bool> cancel{true};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;

  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kCancelled);
  EXPECT_EQ(planner.calls(), 0);
}

TEST(VehiclePlannerDeadlineTest, CancelWinsOverExpiredDeadline) {
  CountingPlanner planner(MakeFakeVehiclePlanner());
  std::atomic<bool> cancel{true};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  options.deadline_present = true;
  options.deadline = Clock::now() - std::chrono::seconds(1);

  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kCancelled);
  EXPECT_EQ(planner.calls(), 0);
}

TEST(VehiclePlannerDeadlineTest, MidRunCancel) {
  std::atomic<bool> cancel{false};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  CooperativeConfig config;
  config.steps = 5;
  config.on_step = [&cancel](int step) {
    if (step == 2) cancel.store(true);
  };
  CooperativeVehiclePlanner planner(MakeFakeVehiclePlanner(), options, config);

  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kCancelled);
  EXPECT_EQ(planner.plan_calls(), 1);
  EXPECT_EQ(planner.steps_started(), 3);
}

TEST(VehiclePlannerDeadlineTest, MidRunTimeout) {
  const Clock::time_point start = Clock::now();
  Clock::time_point fake_now = start;
  VehiclePlanRunOptions options;
  options.deadline_present = true;
  options.deadline = start + std::chrono::seconds(10);
  CooperativeConfig config;
  config.steps = 5;
  config.now = [&fake_now] { return fake_now; };
  config.on_step = [&fake_now, start](int step) {
    fake_now = start + std::chrono::seconds(4 * (step + 1));
  };
  CooperativeVehiclePlanner planner(MakeFakeVehiclePlanner(), options, config);

  ExpectNoTrajectory(RunWithDeadline(planner, Request(), options),
                     VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_EQ(planner.plan_calls(), 1);
  // Steps 0 and 1 end at +4s and +8s; step 2 reaches +12s and times out.
  EXPECT_EQ(planner.steps_started(), 3);
}

TEST(VehiclePlannerDeadlineTest, CompletionWithinBudget) {
  std::atomic<bool> cancel{false};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  options.deadline_present = true;
  options.deadline = Clock::now() + std::chrono::hours(1);
  CooperativeConfig config;
  config.steps = 4;
  CooperativeVehiclePlanner cooperative(MakeFakeVehiclePlanner(), options,
                                        config);

  const auto fake = MakeFakeVehiclePlanner();
  VehiclePlanRequest request = Request();
  request.trajectory_id = "traj_deadline";
  for (const VehiclePlanner* planner :
       {static_cast<const VehiclePlanner*>(&cooperative),
        static_cast<const VehiclePlanner*>(fake.get())}) {
    const VehiclePlanResult result =
        RunWithDeadline(*planner, request, options);
    EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
    EXPECT_EQ(result.trajectory_id, "traj_deadline");
    EXPECT_EQ(result.frame_id, "world_enu");
    EXPECT_EQ(result.samples.size(), 2);
    EXPECT_TRUE(
        AssessVehicleTrajectory(AsVehicleTrajectoryView(result)).accepted);
  }
  EXPECT_EQ(cooperative.steps_started(), 4);
}

TEST(VehiclePlannerDeadlineTest, DefaultOptionsHaveNoBound) {
  CountingPlanner planner(MakeFakeVehiclePlanner());
  const VehiclePlanResult result =
      RunWithDeadline(planner, Request(), VehiclePlanRunOptions{});
  EXPECT_EQ(result.status, VehiclePlanStatus::kOk);
  EXPECT_EQ(planner.calls(), 1);
}

TEST(VehiclePlannerDeadlineTest, ForwardsNoSolutionAndInvalidRequest) {
  std::atomic<bool> cancel{false};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  options.deadline_present = true;
  options.deadline = Clock::now() + std::chrono::hours(1);

  FakeVehiclePlannerConfig no_solution;
  no_solution.status = VehiclePlanStatus::kNoSolution;
  CountingPlanner blocked(MakeFakeVehiclePlanner(no_solution));
  ExpectNoTrajectory(RunWithDeadline(blocked, Request(), options),
                     VehiclePlanStatus::kNoSolution);
  EXPECT_EQ(blocked.calls(), 1);

  CooperativeConfig config;
  config.steps = 2;
  CooperativeVehiclePlanner cooperative(MakeFakeVehiclePlanner(no_solution),
                                        options, config);
  ExpectNoTrajectory(RunWithDeadline(cooperative, Request(), options),
                     VehiclePlanStatus::kNoSolution);

  VehiclePlanRequest bad;
  ExpectNoTrajectory(RunWithDeadline(*MakeFakeVehiclePlanner(), bad, options),
                     VehiclePlanStatus::kInvalidRequest);
}

TEST(VehiclePlannerDeadlineTest, CancelFromAnotherThread) {
  std::atomic<bool> cancel{false};
  VehiclePlanRunOptions options;
  options.cancel = &cancel;
  CooperativeConfig config;
  // Far more work than the test waits for. Cancel is what ends the run.
  config.steps = 100000;
  config.step_sleep = std::chrono::milliseconds(1);
  CooperativeVehiclePlanner planner(MakeFakeVehiclePlanner(), options, config);

  VehiclePlanResult result;
  std::thread worker(
      [&] { result = RunWithDeadline(planner, Request(), options); });
  while (planner.steps_started() < 2) {
    std::this_thread::yield();
  }
  cancel.store(true);
  worker.join();

  ExpectNoTrajectory(result, VehiclePlanStatus::kCancelled);
  EXPECT_LT(planner.steps_started(), config.steps);
}

TEST(VehiclePlannerDeadlineTest, CancelAndDeadlineAreDistinct) {
  EXPECT_NE(VehiclePlanStatus::kCancelled,
            VehiclePlanStatus::kDeadlineExceeded);
  EXPECT_NE(VehiclePlanStatus::kCancelled, VehiclePlanStatus::kNoSolution);
  EXPECT_NE(VehiclePlanStatus::kDeadlineExceeded,
            VehiclePlanStatus::kNoSolution);
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
