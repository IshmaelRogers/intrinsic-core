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

// Tests P1-P12 for the replan trigger (#265, lock id
// l43-5-replan-trigger-v1). The fixture is a private copy of the replan-fixture
// binding from
// intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/replan_trigger.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using ::intrinsic::motion_planning::vehicle::BodyVector;
using ::intrinsic::motion_planning::vehicle::Interpolate;
using ::intrinsic::motion_planning::vehicle::InterpolateResult;
using ::intrinsic::motion_planning::vehicle::KinodynamicBaselineConfig;
using ::intrinsic::motion_planning::vehicle::MakeKinodynamicBaselinePlanner;
using ::intrinsic::motion_planning::vehicle::PrimitiveGenerationMode;
using ::intrinsic::motion_planning::vehicle::PropagateUuvMotionPrimitive;
using ::intrinsic::motion_planning::vehicle::PropagationConfig;
using ::intrinsic::motion_planning::vehicle::PropagationError;
using ::intrinsic::motion_planning::vehicle::PropagationResult;
using ::intrinsic::motion_planning::vehicle::SearchKinodynamicBaseline;
using ::intrinsic::motion_planning::vehicle::StateSpaceError;
using ::intrinsic::motion_planning::vehicle::TrajectoryValidityError;
using ::intrinsic::motion_planning::vehicle::TrajectoryValidityResult;
using ::intrinsic::motion_planning::vehicle::VehicleMotionPrimitive;
using ::intrinsic::motion_planning::vehicle::VehiclePlanningState;
using ::intrinsic::motion_planning::vehicle::VehiclePlanRequest;
using ::intrinsic::motion_planning::vehicle::VehiclePlanResult;
using ::intrinsic::motion_planning::vehicle::VehiclePlanRunOptions;
using ::intrinsic::motion_planning::vehicle::VehiclePlanStatus;
using ::intrinsic::motion_planning::vehicle::VehicleStateBounds;
using ::intrinsic::vehicle::TrajectorySampleView;
using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsResult;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::StatusOr;
using ::intrinsic::vehicle::dynamics::VehicleDynamics;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;

constexpr char kPackageDir[] =
    "intrinsic/simulation/gazebo/scenarios/uuv_reference/";
constexpr std::string_view kFrame = "world_enu";
constexpr std::string_view kRegion = "ops-box";
constexpr int64_t kSecondNs = 1000000000;

// ---- Fixture (private copy of the README replan-fixture binding) -----------

struct Binding {
  int epoch = 1;
  double clearance_m = 1.0;
  VehicleStateBounds bounds{};
  std::string_view pose_frame;
};

::intrinsic::world::WorldSnapshotView Snapshot(int epoch) {
  ::intrinsic::world::WorldSnapshotView view;
  view.present = true;
  view.components = {
      {std::string(::intrinsic::world::kOccupancyReferenceKind), 1}};
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

KinodynamicBaselineConfig Config(const Binding& binding) {
  KinodynamicBaselineConfig config;
  config.primitives.mode = PrimitiveGenerationMode::kCustom;
  config.primitives.max_force_torque = MaxWrench();
  config.primitives.duration_s = 1;
  for (double force : {10.0, -10.0, 5.0, -5.0}) {
    config.primitives.custom_controls.push_back(Wrench(force));
  }
  config.propagation = NominalPropagation();
  config.descriptor = Snapshot(binding.epoch);
  config.snapshot_id = config.descriptor.snapshot_id;
  config.bounds = binding.bounds;
  config.pose_frame = binding.pose_frame;
  config.fence.frame_id = kFrame;
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
  config.clearance_template_m = binding.clearance_m;
  return config;
}

Binding Epoch1() { return Binding{}; }

Binding Speed06() {
  Binding binding;
  binding.epoch = 2;
  binding.bounds.max_linear_speed_present = true;
  binding.bounds.max_linear_speed_m_s = 0.6;
  return binding;
}

Binding Clr02() {
  Binding binding;
  binding.epoch = 2;
  binding.clearance_m = 0.2;
  return binding;
}

Binding Keep() {
  Binding binding;
  binding.epoch = 2;
  return binding;
}

KinodynamicBaselineConfig Stale() {
  KinodynamicBaselineConfig config = Config(Keep());
  config.snapshot_id = Snapshot(1).snapshot_id;
  return config;
}

VehiclePlanningState Origin() { return VehiclePlanningState(); }

VehiclePlanningState StateAfter(const std::vector<double>& forces) {
  VehiclePlanningState state;
  for (double force : forces) {
    VehicleMotionPrimitive primitive;
    primitive.id = "uuv-prim-000";
    primitive.control = Wrench(force);
    primitive.duration_s = 1;
    const ZeroForceDynamics dynamics;
    const PropagationResult propagated = PropagateUuvMotionPrimitive(
        state, primitive, NominalPropagation(), dynamics);
    EXPECT_EQ(propagated.error, PropagationError::kOk);
    state = propagated.samples.back().state;
  }
  return state;
}

VehiclePlanRequest GoalRequest() {
  VehiclePlanRequest request;
  request.start_label = "start";
  request.goal_label = "goal";
  request.trajectory_id = "replan";
  request.states_present = true;
  request.start_state = Origin();
  request.goal_state = StateAfter({10, -10});
  return request;
}

VehiclePlanResult Epoch1Plan() {
  const ZeroForceDynamics dynamics;
  return SearchKinodynamicBaseline(Config(Epoch1()), GoalRequest(), dynamics);
}

// ---- Bit-exact comparison ---------------------------------------------------

uint64_t B(double v) { return std::bit_cast<uint64_t>(v); }

std::array<uint64_t, 13> Bits(const VehiclePlanningState& s) {
  return {B(s.position.x),     B(s.position.y),      B(s.position.z),
          B(s.orientation.x),  B(s.orientation.y),   B(s.orientation.z),
          B(s.orientation.w),  B(s.twist.linear_x),  B(s.twist.linear_y),
          B(s.twist.linear_z), B(s.twist.angular_x), B(s.twist.angular_y),
          B(s.twist.angular_z)};
}

bool SameState(const VehiclePlanningState& a, const VehiclePlanningState& b) {
  return Bits(a) == Bits(b);
}

bool SameView(const TrajectorySampleView& a, const TrajectorySampleView& b) {
  const auto vec = [](const BodyVector& v) {
    return std::array<uint64_t, 6>{B(v.linear_x),  B(v.linear_y),
                                   B(v.linear_z),  B(v.angular_x),
                                   B(v.angular_y), B(v.angular_z)};
  };
  return a.time_present == b.time_present && a.seconds == b.seconds &&
         a.nanos == b.nanos && B(a.position.x) == B(b.position.x) &&
         B(a.position.y) == B(b.position.y) &&
         B(a.position.z) == B(b.position.z) &&
         B(a.orientation.x) == B(b.orientation.x) &&
         B(a.orientation.y) == B(b.orientation.y) &&
         B(a.orientation.z) == B(b.orientation.z) &&
         B(a.orientation.w) == B(b.orientation.w) &&
         a.twist_present == b.twist_present && vec(a.twist) == vec(b.twist) &&
         a.acceleration_present == b.acceleration_present &&
         vec(a.acceleration) == vec(b.acceleration);
}

bool SamePlan(const VehiclePlanResult& a, const VehiclePlanResult& b) {
  if (a.status != b.status || a.header_present != b.header_present ||
      a.validity_present != b.validity_present ||
      a.validity_state != b.validity_state || a.frame_id != b.frame_id ||
      a.trajectory_id != b.trajectory_id ||
      a.provenance_present != b.provenance_present ||
      a.model_id != b.model_id || a.samples.size() != b.samples.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    if (!SameView(a.samples[i], b.samples[i])) {
      return false;
    }
  }
  return true;
}

bool SameRequest(const VehiclePlanRequest& a, const VehiclePlanRequest& b) {
  return a.start_label == b.start_label && a.goal_label == b.goal_label &&
         a.trajectory_id == b.trajectory_id &&
         a.states_present == b.states_present &&
         SameState(a.start_state, b.start_state) &&
         SameState(a.goal_state, b.goal_state);
}

bool SameRecheck(const TrajectoryValidityResult& a,
                 const TrajectoryValidityResult& b) {
  return a.error == b.error &&
         a.first_invalid_sample == b.first_invalid_sample &&
         a.failed_rule == b.failed_rule;
}

bool SameResult(const ReplanStepResult& a, const ReplanStepResult& b) {
  return a.outcome == b.outcome && a.input_error == b.input_error &&
         a.recheck_ran == b.recheck_ran && SameRecheck(a.recheck, b.recheck) &&
         a.replan_ran == b.replan_ran && a.plan_status == b.plan_status &&
         SamePlan(a.trajectory, b.trajectory);
}

// ---- Helpers ----------------------------------------------------------------

VehiclePlanningState SampleState(const TrajectorySampleView& sample) {
  VehiclePlanningState state;
  state.position = sample.position;
  state.orientation = sample.orientation;
  state.twist = sample.twist;
  return state;
}

int64_t SampleNs(const TrajectorySampleView& sample) {
  return sample.seconds * kSecondNs + sample.nanos;
}

// State of `plan` at trajectory time t_ns, by a linear scan.
VehiclePlanningState TruthAt(const VehiclePlanResult& plan, int64_t t_ns) {
  const std::vector<TrajectorySampleView>& samples = plan.samples;
  if (t_ns >= SampleNs(samples.back())) {
    return SampleState(samples.back());
  }
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    const int64_t a = SampleNs(samples[i]);
    const int64_t b = SampleNs(samples[i + 1]);
    if (t_ns == a) {
      return SampleState(samples[i]);
    }
    if (t_ns > a && t_ns < b) {
      const double u =
          static_cast<double>(t_ns - a) / static_cast<double>(b - a);
      const InterpolateResult result =
          Interpolate(SampleState(samples[i]), SampleState(samples[i + 1]), u);
      EXPECT_EQ(result.error, StateSpaceError::kOk);
      return result.state;
    }
  }
  ADD_FAILURE() << "t_ns " << t_ns << " is not bracketed";
  return VehiclePlanningState();
}

// Runs EvaluateReplan from epoch 1 against `next` on a fresh epoch-1 plan and
// the truth at t_ns.
ReplanStepResult RunFrom(const KinodynamicBaselineConfig& current,
                         const KinodynamicBaselineConfig& next,
                         const VehiclePlanResult& active, int64_t t_ns,
                         const VehicleDynamics* dynamics,
                         std::atomic<bool>* cancel) {
  return EvaluateReplan(current, next, dynamics, active, TruthAt(active, t_ns),
                        t_ns, GoalRequest(), cancel);
}

ReplanStepResult RunBinding(const Binding& next, int64_t t_ns) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult active = Epoch1Plan();
  return RunFrom(Config(Epoch1()), Config(next), active, t_ns, &dynamics,
                 nullptr);
}

void ExpectRecheck(const ReplanStepResult& result,
                   TrajectoryValidityError error, int first_invalid,
                   std::string_view rule) {
  EXPECT_TRUE(result.recheck_ran);
  EXPECT_EQ(result.recheck.error, error);
  EXPECT_EQ(result.recheck.first_invalid_sample, first_invalid);
  EXPECT_EQ(result.recheck.failed_rule, rule);
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

std::string ReadRunfile(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  const std::string suffix = std::string(kPackageDir) + name;
  std::vector<std::string> candidates;
  if (src != nullptr) {
    candidates.push_back(std::string(src) + "/_main/" + suffix);
    const char* workspace = std::getenv("TEST_WORKSPACE");
    if (workspace != nullptr) {
      candidates.push_back(std::string(src) + "/" + workspace + "/" + suffix);
    }
    candidates.push_back(std::string(src) + "/" + suffix);
  }
  for (const std::string& path : candidates) {
    std::ifstream file(path);
    if (file) {
      std::ostringstream contents;
      contents << file.rdbuf();
      return contents.str();
    }
  }
  return "";
}

// ---- Tests ------------------------------------------------------------------

TEST(ReplanTriggerTest, P1Unchanged) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult active = Epoch1Plan();
  const ReplanStepResult result = RunFrom(Config(Epoch1()), Config(Epoch1()),
                                          active, 0, &dynamics, nullptr);
  EXPECT_EQ(result.outcome, ReplanOutcome::kUnchanged);
  EXPECT_EQ(result.input_error, ReplanInputError::kNone);
  EXPECT_FALSE(result.recheck_ran);
  EXPECT_FALSE(result.replan_ran);

  const ReplanStepResult empty = EvaluateReplan(
      Config(Epoch1()), Config(Epoch1()), &dynamics, VehiclePlanResult(),
      Origin(), -5, VehiclePlanRequest(), nullptr);
  EXPECT_EQ(empty.outcome, ReplanOutcome::kUnchanged);
  EXPECT_EQ(empty.input_error, ReplanInputError::kNone);
  EXPECT_FALSE(empty.recheck_ran);
  EXPECT_FALSE(empty.replan_ran);
}

TEST(ReplanTriggerTest, P2Replanned) {
  const ReplanStepResult result = RunBinding(Speed06(), 0);
  EXPECT_EQ(result.outcome, ReplanOutcome::kReplanned);
  EXPECT_EQ(result.input_error, ReplanInputError::kNone);
  ExpectRecheck(result, TrajectoryValidityError::kBounds, 2, "bounds");
  EXPECT_TRUE(result.replan_ran);
  EXPECT_EQ(result.plan_status, VehiclePlanStatus::kOk);

  const ZeroForceDynamics dynamics;
  const KinodynamicBaselineConfig config = Config(Speed06());
  const auto planner = MakeKinodynamicBaselinePlanner(config, &dynamics);
  VehiclePlanRequest request = GoalRequest();
  request.start_state = Origin();
  EXPECT_TRUE(SamePlan(result.trajectory, planner->Plan(request)));

  const VehiclePlanResult& trajectory = result.trajectory;
  ASSERT_EQ(trajectory.samples.size(), 9u);
  EXPECT_EQ(trajectory.frame_id, kFrame);
  EXPECT_EQ(trajectory.trajectory_id, "replan");
  EXPECT_EQ(SampleNs(trajectory.samples.back()), 4 * kSecondNs);
  const std::array<double, 9> x = {0, .125, .375, .5, .5, .625, .875, 1, 1};
  double peak = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_NEAR(trajectory.samples[i].position.x, x[i], 1e-12) << i;
    peak = std::max(peak, std::abs(trajectory.samples[i].twist.linear_x));
  }
  EXPECT_NEAR(peak, 0.5, 1e-12);
}

TEST(ReplanTriggerTest, P3NoSolution) {
  const ReplanStepResult result = RunBinding(Clr02(), 0);
  EXPECT_EQ(result.outcome, ReplanOutcome::kNoSolution);
  ExpectRecheck(result, TrajectoryValidityError::kClearance, 0,
                "clearance.min");
  EXPECT_TRUE(result.replan_ran);
  EXPECT_EQ(result.plan_status, VehiclePlanStatus::kNoSolution);
  EXPECT_TRUE(result.trajectory.samples.empty());
  EXPECT_EQ(result.trajectory.status, VehiclePlanStatus::kInvalidRequest);
}

TEST(ReplanTriggerTest, P4Kept) {
  for (int64_t t_ns : {int64_t{0}, kSecondNs}) {
    const ReplanStepResult result = RunBinding(Keep(), t_ns);
    EXPECT_EQ(result.outcome, ReplanOutcome::kKept) << t_ns;
    ExpectRecheck(result, TrajectoryValidityError::kOk, -1, "");
    EXPECT_FALSE(result.replan_ran);
  }
  for (int64_t t_ns : {2 * kSecondNs, 3 * kSecondNs}) {
    const ReplanStepResult result = RunBinding(Speed06(), t_ns);
    EXPECT_EQ(result.outcome, ReplanOutcome::kKept) << t_ns;
    ExpectRecheck(result, TrajectoryValidityError::kOk, -1, "");
    EXPECT_FALSE(result.replan_ran);
  }
}

TEST(ReplanTriggerTest, P5RemainingByValue) {
  const VehiclePlanResult active = Epoch1Plan();
  ASSERT_EQ(active.samples.size(), 5u);
  std::array<VehiclePlanningState, 5> s;
  for (std::size_t i = 0; i < s.size(); ++i) {
    s[i] = SampleState(active.samples[i]);
  }

  const auto expect_remaining =
      [&](int64_t t_ns, const VehiclePlanningState& truth,
          const std::vector<double>& times,
          const std::vector<VehiclePlanningState>& states) {
        const RemainingTrajectory remaining =
            BuildRemainingTrajectory(active, truth, t_ns);
        EXPECT_EQ(remaining.error, ReplanInputError::kNone) << t_ns;
        ASSERT_EQ(remaining.samples.size(), times.size()) << t_ns;
        for (std::size_t i = 0; i < times.size(); ++i) {
          EXPECT_EQ(remaining.samples[i].time_s, times[i]) << t_ns << " " << i;
          EXPECT_TRUE(SameState(remaining.samples[i].state, states[i]))
              << t_ns << " " << i;
        }
      };

  expect_remaining(0, s[0], {0, 0.5, 1.0, 1.5, 2.0},
                   {s[0], s[1], s[2], s[3], s[4]});
  expect_remaining(kSecondNs, s[2], {0, 0.5, 1.0}, {s[2], s[3], s[4]});

  const InterpolateResult mid = Interpolate(s[1], s[2], 0.5);
  ASSERT_EQ(mid.error, StateSpaceError::kOk);
  EXPECT_NEAR(mid.state.position.x, 0.5, 1e-12);
  EXPECT_NEAR(mid.state.twist.linear_x, 0.75, 1e-12);
  expect_remaining(750000000, mid.state, {0, 0.25, 0.75, 1.25},
                   {mid.state, s[2], s[3], s[4]});
  EXPECT_TRUE(SameState(TruthAt(active, 750000000), mid.state));

  expect_remaining(2 * kSecondNs, s[4], {0}, {s[4]});
  expect_remaining(2500000000, s[4], {0}, {s[4]});
}

TEST(ReplanTriggerTest, P6MidTrajectory) {
  const ReplanStepResult half = RunBinding(Speed06(), 500000000);
  EXPECT_EQ(half.outcome, ReplanOutcome::kNoSolution);
  ExpectRecheck(half, TrajectoryValidityError::kBounds, 1, "bounds");
  EXPECT_TRUE(half.replan_ran);
  EXPECT_EQ(half.plan_status, VehiclePlanStatus::kNoSolution);

  for (int64_t t_ns : {int64_t{750000000}, kSecondNs}) {
    const ReplanStepResult late = RunBinding(Speed06(), t_ns);
    EXPECT_EQ(late.outcome, ReplanOutcome::kError) << t_ns;
    ExpectRecheck(late, TrajectoryValidityError::kBounds, 0, "bounds");
    EXPECT_TRUE(late.replan_ran);
    EXPECT_EQ(late.plan_status, VehiclePlanStatus::kInvalidRequest);
    EXPECT_TRUE(late.trajectory.samples.empty());
  }
}

TEST(ReplanTriggerTest, P7StaleAndFrameBinding) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult active = Epoch1Plan();
  Binding other;
  other.epoch = 3;
  const ReplanStepResult stale =
      RunFrom(Config(other), Stale(), active, 0, &dynamics, nullptr);
  EXPECT_EQ(stale.outcome, ReplanOutcome::kError);
  ExpectRecheck(stale, TrajectoryValidityError::kBadRequest, -1, "snapshot");
  EXPECT_FALSE(stale.replan_ran);

  Binding ned = Speed06();
  ned.pose_frame = "world_ned";
  const ReplanStepResult frame = RunBinding(ned, 0);
  EXPECT_EQ(frame.outcome, ReplanOutcome::kError);
  ExpectRecheck(frame, TrajectoryValidityError::kFrameError, 0,
                "geofence.aabb");
  EXPECT_FALSE(frame.replan_ran);
}

TEST(ReplanTriggerTest, P8Preconditions) {
  const ZeroForceDynamics dynamics;
  const KinodynamicBaselineConfig current = Config(Epoch1());
  const KinodynamicBaselineConfig next = Config(Speed06());
  const VehiclePlanResult good = Epoch1Plan();

  const auto expect_error = [&](const VehiclePlanResult& active,
                                const VehiclePlanningState& truth, int64_t t_ns,
                                const VehiclePlanRequest& goal,
                                ReplanInputError error, const char* label) {
    const ReplanStepResult result = EvaluateReplan(
        current, next, &dynamics, active, truth, t_ns, goal, nullptr);
    EXPECT_EQ(result.outcome, ReplanOutcome::kError) << label;
    EXPECT_EQ(result.input_error, error) << label;
    EXPECT_FALSE(result.recheck_ran) << label;
    EXPECT_FALSE(result.replan_ran) << label;
    if (error != ReplanInputError::kGoalRequest) {
      EXPECT_NE(BuildRemainingTrajectory(active, truth, t_ns).error,
                ReplanInputError::kNone)
          << label;
    }
  };

  VehiclePlanResult not_ok = good;
  not_ok.status = VehiclePlanStatus::kNoSolution;
  expect_error(not_ok, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveNotOk, "not ok");

  VehiclePlanResult empty = good;
  empty.samples.clear();
  expect_error(empty, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveEmpty, "empty");

  VehiclePlanResult no_time = good;
  no_time.samples[2].time_present = false;
  expect_error(no_time, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveSample, "no time");

  VehiclePlanResult no_twist = good;
  no_twist.samples[1].twist_present = false;
  expect_error(no_twist, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveSample, "no twist");

  VehiclePlanResult bad_nanos = good;
  bad_nanos.samples[3].nanos = 1000000000;
  expect_error(bad_nanos, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveSample, "nanos");

  VehiclePlanResult late_start = good;
  for (TrajectorySampleView& sample : late_start.samples) {
    sample.seconds += 1;
  }
  expect_error(late_start, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveTimes, "first time not 0");

  VehiclePlanResult repeated = good;
  repeated.samples[2].seconds = repeated.samples[1].seconds;
  repeated.samples[2].nanos = repeated.samples[1].nanos;
  expect_error(repeated, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveTimes, "repeated time");

  VehiclePlanResult reversed = good;
  std::swap(reversed.samples[3], reversed.samples[4]);
  expect_error(reversed, Origin(), 0, GoalRequest(),
               ReplanInputError::kActiveTimes, "reversed times");

  expect_error(good, Origin(), -1, GoalRequest(),
               ReplanInputError::kNegativeTime, "negative time");

  VehiclePlanningState nan_truth = Origin();
  nan_truth.position.x = std::numeric_limits<double>::quiet_NaN();
  expect_error(good, nan_truth, 0, GoalRequest(), ReplanInputError::kTruthState,
               "nan truth");

  VehiclePlanningState bad_quaternion = Origin();
  bad_quaternion.orientation.w = 2.0;
  expect_error(good, bad_quaternion, 0, GoalRequest(),
               ReplanInputError::kTruthState, "quaternion");

  VehiclePlanRequest no_states = GoalRequest();
  no_states.states_present = false;
  expect_error(good, Origin(), 0, no_states, ReplanInputError::kGoalRequest,
               "no goal states");
}

TEST(ReplanTriggerTest, P9CancelAndDynamics) {
  const VehiclePlanResult active = Epoch1Plan();
  const KinodynamicBaselineConfig current = Config(Epoch1());

  std::atomic<bool> pre_cancel{true};
  const ZeroForceDynamics zero;
  const ReplanStepResult pre =
      RunFrom(current, Config(Speed06()), active, 0, &zero, &pre_cancel);
  EXPECT_EQ(pre.outcome, ReplanOutcome::kError);
  EXPECT_TRUE(pre.replan_ran);
  EXPECT_EQ(pre.plan_status, VehiclePlanStatus::kCancelled);
  EXPECT_TRUE(pre.trajectory.samples.empty());

  // A run_options pointer left in new_binding is overwritten (R5).
  std::atomic<bool> cancel{false};
  CancelAfterEvaluate dynamics(&cancel);
  VehiclePlanRunOptions caller_options;
  KinodynamicBaselineConfig next = Config(Speed06());
  next.run_options = &caller_options;
  const ReplanStepResult mid =
      RunFrom(current, next, active, 0, &dynamics, &cancel);
  EXPECT_EQ(mid.outcome, ReplanOutcome::kError);
  EXPECT_TRUE(mid.replan_ran);
  EXPECT_EQ(mid.plan_status, VehiclePlanStatus::kCancelled);
  EXPECT_GE(dynamics.calls(), 1);
  EXPECT_FALSE(caller_options.deadline_present);
  EXPECT_EQ(caller_options.cancel, nullptr);

  const ReplanStepResult null_dynamics =
      RunFrom(current, Config(Speed06()), active, 0, nullptr, nullptr);
  EXPECT_EQ(null_dynamics.outcome, ReplanOutcome::kError);
  EXPECT_TRUE(null_dynamics.replan_ran);
  EXPECT_EQ(null_dynamics.plan_status, VehiclePlanStatus::kInvalidRequest);
}

TEST(ReplanTriggerTest, P10RepeatIsBitIdentical) {
  const std::vector<std::pair<Binding, int64_t>> cases = {
      {Speed06(), 0},
      {Clr02(), 0},
      {Speed06(), 500000000},
      {Speed06(), 750000000}};
  for (const auto& [binding, t_ns] : cases) {
    const ReplanStepResult first = RunBinding(binding, t_ns);
    const ReplanStepResult second = RunBinding(binding, t_ns);
    EXPECT_TRUE(SameResult(first, second))
        << binding.epoch << " " << binding.clearance_m << " " << t_ns;
  }
}

TEST(ReplanTriggerTest, P11SourceScan) {
  const std::array<std::string_view, 7> forbidden = {"steady_clock",
                                                     "system_clock",
                                                     "absl::Now",
                                                     "VehiclePlannerRegistry",
                                                     "deadline_present = true",
                                                     "sleep",
                                                     "std::thread"};
  for (const char* file : {"replan_trigger.h", "replan_trigger.cc"}) {
    const std::string source = ReadRunfile(file);
    ASSERT_FALSE(source.empty()) << file;
    for (std::string_view token : forbidden) {
      EXPECT_EQ(source.find(token), std::string::npos) << file << " " << token;
    }
  }
}

TEST(ReplanTriggerTest, P12NoMutation) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanResult active = Epoch1Plan();
  const VehiclePlanResult active_before = active;
  const VehiclePlanRequest goal = GoalRequest();
  const VehiclePlanRequest goal_before = goal;
  const KinodynamicBaselineConfig current = Config(Epoch1());
  for (const Binding& binding : {Speed06(), Clr02(), Keep()}) {
    for (int64_t t_ns : {int64_t{0}, int64_t{500000000}, 2 * kSecondNs}) {
      const ReplanStepResult result =
          EvaluateReplan(current, Config(binding), &dynamics, active,
                         TruthAt(active, t_ns), t_ns, goal, nullptr);
      (void)result;
      EXPECT_TRUE(SamePlan(active, active_before));
      EXPECT_TRUE(SameRequest(goal, goal_before));
    }
  }
}

}  // namespace
}  // namespace intrinsic::simulation::uuv_reference
