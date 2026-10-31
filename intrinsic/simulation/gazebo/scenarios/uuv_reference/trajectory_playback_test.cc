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

// Tests for the trajectory-playback mover (#263, lock id
// l43-3-trajectory-playback-v1). The trajectories come from the kinodynamic
// baseline planner using a private copy of the replan-fixture binding from
// intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md. A
// test-local probe system reads the model Pose and the link
// WorldLinearVelocity after each step.

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/trajectory_playback.h"

#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/motion_planning/vehicle/vehicle_kinodynamic_baseline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using ::intrinsic::motion_planning::vehicle::AsVehicleTrajectoryView;
using ::intrinsic::motion_planning::vehicle::Interpolate;
using ::intrinsic::motion_planning::vehicle::InterpolateResult;
using ::intrinsic::motion_planning::vehicle::KinodynamicBaselineConfig;
using ::intrinsic::motion_planning::vehicle::PrimitiveGenerationMode;
using ::intrinsic::motion_planning::vehicle::PropagateUuvMotionPrimitive;
using ::intrinsic::motion_planning::vehicle::PropagationConfig;
using ::intrinsic::motion_planning::vehicle::PropagationError;
using ::intrinsic::motion_planning::vehicle::PropagationResult;
using ::intrinsic::motion_planning::vehicle::SearchKinodynamicBaseline;
using ::intrinsic::motion_planning::vehicle::StateSpaceError;
using ::intrinsic::motion_planning::vehicle::VehicleMotionPrimitive;
using ::intrinsic::motion_planning::vehicle::VehiclePlanningState;
using ::intrinsic::motion_planning::vehicle::VehiclePlanRequest;
using ::intrinsic::motion_planning::vehicle::VehiclePlanResult;
using ::intrinsic::motion_planning::vehicle::VehiclePlanStatus;
using ::intrinsic::motion_planning::vehicle::VehicleStateBounds;
using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::BodyVector;
using ::intrinsic::vehicle::TrajectorySampleView;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;

constexpr char kPackageDir[] =
    "intrinsic/simulation/gazebo/scenarios/uuv_reference/";
constexpr char kWorldFile[] = "testdata/playback_test.world.sdf";

constexpr int64_t kStepNs = 125000000;
constexpr int kMaxRunCalls = 200;
constexpr std::string_view kFrame = "world_enu";
constexpr std::string_view kRegion = "ops-box";

// ---- Plans (private copy of the README replan-fixture binding) -------------

struct Binding {
  int epoch = 1;
  double clearance_m = 1.0;
  VehicleStateBounds bounds{};
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

KinodynamicBaselineConfig Config(const std::vector<double>& forces,
                                 const Binding& binding) {
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

VehiclePlanResult Plan(const Binding& binding) {
  VehiclePlanRequest request;
  request.start_label = "start";
  request.goal_label = "goal";
  request.trajectory_id = "replan";
  request.states_present = true;
  request.goal_state = StateAfter({10, -10});
  const ZeroForceDynamics dynamics;
  return SearchKinodynamicBaseline(Config({10, -10, 5, -5}, binding), request,
                                   dynamics);
}

// Epoch 1: 5 samples over t in [0, 2].
VehiclePlanResult Epoch1Plan() { return Plan(Binding{}); }

// Epoch 2, max_linear_speed 0.6: 9 samples over t in [0, 4].
VehiclePlanResult Epoch2Plan() {
  Binding binding;
  binding.epoch = 2;
  binding.bounds.max_linear_speed_present = true;
  binding.bounds.max_linear_speed_m_s = 0.6;
  return Plan(binding);
}

// ---- Bit comparison and the hand-computed oracle ----------------------------

std::array<uint64_t, 13> Bits(const VehiclePlanningState& s) {
  const auto b = [](double v) { return std::bit_cast<uint64_t>(v); };
  return {b(s.position.x),     b(s.position.y),      b(s.position.z),
          b(s.orientation.x),  b(s.orientation.y),   b(s.orientation.z),
          b(s.orientation.w),  b(s.twist.linear_x),  b(s.twist.linear_y),
          b(s.twist.linear_z), b(s.twist.angular_x), b(s.twist.angular_y),
          b(s.twist.angular_z)};
}

bool SameState(const VehiclePlanningState& a, const VehiclePlanningState& b) {
  return Bits(a) == Bits(b);
}

VehiclePlanningState SampleState(const TrajectorySampleView& sample) {
  VehiclePlanningState state;
  state.position = sample.position;
  state.orientation = sample.orientation;
  state.twist = sample.twist;
  return state;
}

int64_t SampleNs(const TrajectorySampleView& sample) {
  return sample.seconds * 1000000000LL + sample.nanos;
}

// State of `plan` at trajectory time t_ns, by a linear scan.
VehiclePlanningState StateAt(const VehiclePlanResult& plan, int64_t t_ns) {
  const std::vector<TrajectorySampleView>& samples = plan.samples;
  if (t_ns >= SampleNs(samples.back())) {
    return SampleState(samples.back());
  }
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
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

// ---- Runfiles and environment (as in dive_resurface_scenario_test.cc) ------

std::string RunfilesPath(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = std::string(kPackageDir) + name;
  const std::string primary = std::string(src) + "/_main/" + suffix;
  if (std::ifstream(primary)) {
    return primary;
  }
  const char* workspace = std::getenv("TEST_WORKSPACE");
  if (workspace != nullptr) {
    const std::string legacy =
        std::string(src) + "/" + workspace + "/" + suffix;
    if (std::ifstream(legacy)) {
      return legacy;
    }
  }
  return std::string(src) + "/" + suffix;
}

class PlaybackEnvironment : public testing::Environment {
 public:
  void SetUp() override {
    // Empty server config skips the default Physics, UserCommands and
    // SceneBroadcaster systems. HOME avoids a write to /.gz.
    setenv("GZ_SIM_SERVER_CONFIG_PATH", "", 1);
    setenv("HOME", "/tmp", 1);
    setenv("GZ_PARTITION", "intrinsic_uuv_playback_test", 1);
  }
};

testing::Environment* const kPartitionEnvironment =
    testing::AddGlobalTestEnvironment(new PlaybackEnvironment);

// ---- Probe
// -------------------------------------------------------------------

struct ProbeSample {
  int64_t sim_time_ns = 0;
  std::array<double, 7> pose = {};  // x, y, z, qx, qy, qz, qw.
  std::array<double, 3> velocity = {};
};

// Reads the model Pose and the link WorldLinearVelocity components directly,
// not worldPose, after every server step.
class Probe final : public gz::sim::System, public gz::sim::ISystemPostUpdate {
 public:
  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    const gz::sim::Entity model = ecm.EntityByComponents(
        gz::sim::components::Name("uuv"), gz::sim::components::Model());
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    if (model == gz::sim::kNullEntity || link == gz::sim::kNullEntity) {
      return;
    }
    ProbeSample sample;
    sample.sim_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    const auto* pose = ecm.Component<gz::sim::components::Pose>(model);
    if (pose != nullptr) {
      const gz::math::Pose3d& p = pose->Data();
      sample.pose = {p.Pos().X(), p.Pos().Y(), p.Pos().Z(), p.Rot().X(),
                     p.Rot().Y(), p.Rot().Z(), p.Rot().W()};
    }
    const auto* velocity =
        ecm.Component<gz::sim::components::WorldLinearVelocity>(link);
    if (velocity != nullptr) {
      const gz::math::Vector3d& v = velocity->Data();
      sample.velocity = {v.X(), v.Y(), v.Z()};
    }
    samples_.push_back(sample);
  }

  // Read only after Server::Run has returned.
  const std::vector<ProbeSample>& samples() const { return samples_; }

  const ProbeSample* At(int64_t sim_time_ns) const {
    for (const ProbeSample& sample : samples_) {
      if (sample.sim_time_ns == sim_time_ns) {
        return &sample;
      }
    }
    return nullptr;
  }

 private:
  std::vector<ProbeSample> samples_;
};

std::array<uint64_t, 7> PoseBits(const std::array<double, 7>& pose) {
  std::array<uint64_t, 7> bits;
  for (size_t i = 0; i < pose.size(); ++i) {
    bits[i] = std::bit_cast<uint64_t>(pose[i]);
  }
  return bits;
}

std::array<double, 7> PoseOf(const VehiclePlanningState& s) {
  return {s.position.x,    s.position.y,    s.position.z,   s.orientation.x,
          s.orientation.y, s.orientation.z, s.orientation.w};
}

// ---- Harness
// -----------------------------------------------------------------

class Harness {
 public:
  explicit Harness(VehiclePlanResult initial_plan,
                   TrajectoryPlaybackOptions options = {}) {
    gz::sim::ServerConfig config;
    config.SetHeadlessRendering(true);
    EXPECT_TRUE(config.SetSdfFile(RunfilesPath(kWorldFile)));
    server_ = std::make_unique<gz::sim::Server>(config);
    server_->SetUpdatePeriod(std::chrono::steady_clock::duration::zero());

    PlaybackInstallError error = PlaybackInstallError::kOk;
    playback_ =
        TrajectoryPlayback::Create(options, std::move(initial_plan), &error);
    EXPECT_EQ(error, PlaybackInstallError::kOk);
    EXPECT_TRUE(playback_ != nullptr);
    probe_ = std::make_shared<Probe>();
    if (playback_ != nullptr) {
      EXPECT_TRUE(server_->AddSystem(playback_).value_or(false));
    }
    EXPECT_TRUE(server_->AddSystem(probe_).value_or(false));
  }

  TrajectoryPlayback& playback() { return *playback_; }
  const Probe& probe() const { return *probe_; }
  int run_calls() const { return run_calls_; }

  // Drives the server until the last applied k reaches `k`, with a cap on the
  // number of Run calls (the first calls may not advance simulation time).
  bool RunUntilK(int64_t k) {
    for (int call = 0; call < kMaxRunCalls; ++call) {
      const std::optional<PlaybackStep> last = playback_->last_step();
      if (last.has_value() && last->k >= k) {
        return true;
      }
      EXPECT_TRUE(server_->Run(/*_blocking=*/true, 1, /*_paused=*/false));
      ++run_calls_;
    }
    return false;
  }

  void RunCalls(int calls) {
    for (int i = 0; i < calls; ++i) {
      EXPECT_TRUE(server_->Run(/*_blocking=*/true, 1, /*_paused=*/false));
      ++run_calls_;
    }
  }

 private:
  std::unique_ptr<gz::sim::Server> server_;
  std::shared_ptr<TrajectoryPlayback> playback_;
  std::shared_ptr<Probe> probe_;
  int run_calls_ = 0;
};

// Compares one applied step against the oracle and the probe.
void ExpectStepMatches(const Harness& harness, const PlaybackStep& step,
                       const VehiclePlanningState& expected,
                       uint64_t expected_serial) {
  EXPECT_EQ(step.sim_time_ns, (step.k + 1) * kStepNs) << step.k;
  EXPECT_EQ(step.trajectory_serial, expected_serial) << step.k;
  EXPECT_TRUE(SameState(step.state, expected)) << step.k;
  const ProbeSample* probe = harness.probe().At(step.sim_time_ns);
  ASSERT_TRUE(probe != nullptr) << step.k;
  EXPECT_EQ(PoseBits(probe->pose), PoseBits(PoseOf(expected))) << step.k;
}

// ---- Plans used by the tests are what the README says they are -------------

TEST(PlaybackFixtureTest, PlansMatchTheReplanFixture) {
  const VehiclePlanResult first = Epoch1Plan();
  const VehiclePlanResult second = Epoch2Plan();
  ASSERT_EQ(first.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(second.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(first.samples.size(), 5u);
  ASSERT_EQ(second.samples.size(), 9u);
  EXPECT_EQ(SampleNs(first.samples.front()), 0);
  EXPECT_EQ(SampleNs(first.samples.back()), 2000000000LL);
  EXPECT_EQ(SampleNs(second.samples.front()), 0);
  EXPECT_EQ(SampleNs(second.samples.back()), 4000000000LL);
  EXPECT_EQ(first.frame_id, kFrame);
  EXPECT_TRUE(AssessVehicleTrajectory(AsVehicleTrajectoryView(first)).accepted);
  EXPECT_TRUE(
      AssessVehicleTrajectory(AsVehicleTrajectoryView(second)).accepted);
}

// ---- T1, T2
// -------------------------------------------------------------------

TEST(TrajectoryPlaybackTest, T1_PlaysTheEpoch1PlanBitExactly) {
  const VehiclePlanResult plan = Epoch1Plan();
  ASSERT_EQ(plan.status, VehiclePlanStatus::kOk);
  Harness harness(plan);
  ASSERT_TRUE(harness.RunUntilK(23)) << "run_calls=" << harness.run_calls();
  const std::vector<PlaybackStep>& steps = harness.playback().steps();
  ASSERT_EQ(steps.size(), 24u);
  for (int64_t k = 0; k < 24; ++k) {
    ASSERT_EQ(steps[k].k, k);
    ExpectStepMatches(harness, steps[k], StateAt(plan, (k + 1) * kStepNs), 1);
  }
  // k = 3 (t = 0.5) is on sample 1.
  EXPECT_TRUE(SameState(steps[3].state, SampleState(plan.samples[1])));
  // k = 4 (t = 0.625) is Interpolate(s1, s2, 0.25).
  const InterpolateResult mid = Interpolate(SampleState(plan.samples[1]),
                                            SampleState(plan.samples[2]), 0.25);
  ASSERT_EQ(mid.error, StateSpaceError::kOk);
  EXPECT_TRUE(SameState(steps[4].state, mid.state));
  // k >= 15 (t >= 2.0) is the last sample.
  for (int64_t k = 15; k < 24; ++k) {
    EXPECT_TRUE(SameState(steps[k].state, SampleState(plan.samples.back())))
        << k;
  }
  // Sample 4 is the last one and is reached exactly at k = 15.
  EXPECT_EQ(harness.playback().last_step()->k, 23);
  EXPECT_EQ(harness.playback().interpolation_errors(), 0);
  EXPECT_EQ(harness.playback().missing_entity_steps(), 0);
}

TEST(TrajectoryPlaybackTest, T2_LinearVelocityIsTheRotatedBodyTwist) {
  const VehiclePlanResult plan = Epoch1Plan();
  ASSERT_EQ(plan.status, VehiclePlanStatus::kOk);
  Harness harness(plan);
  ASSERT_TRUE(harness.RunUntilK(23));
  bool saw_motion = false;
  for (const PlaybackStep& step : harness.playback().steps()) {
    const embodiment::Vec3 expected = embodiment::RotateVector(
        step.state.orientation,
        {step.state.twist.linear_x, step.state.twist.linear_y,
         step.state.twist.linear_z});
    const ProbeSample* probe = harness.probe().At(step.sim_time_ns);
    ASSERT_TRUE(probe != nullptr) << step.k;
    EXPECT_NEAR(probe->velocity[0], expected.x, 1e-12) << step.k;
    EXPECT_NEAR(probe->velocity[1], expected.y, 1e-12) << step.k;
    EXPECT_NEAR(probe->velocity[2], expected.z, 1e-12) << step.k;
    saw_motion = saw_motion || std::abs(expected.x) > 0.1;
  }
  EXPECT_TRUE(saw_motion);
}

// A hand-built plan with a non-identity orientation checks the Hamilton
// (x, y, z, w) to gz (w, x, y, z) order, which the identity plans cannot.
TEST(TrajectoryPlaybackTest, OrientationOrderIsBitExact) {
  const double s = std::sqrt(0.5);
  VehiclePlanResult plan;
  plan.status = VehiclePlanStatus::kOk;
  plan.header_present = true;
  plan.validity_present = true;
  plan.validity_state = 1;
  plan.frame_id = std::string(kFrame);
  plan.trajectory_id = "yaw90";
  for (int i = 0; i < 2; ++i) {
    TrajectorySampleView sample;
    sample.time_present = true;
    sample.seconds = i;
    sample.position = {0.25 * i, 0.5, -1.0};
    sample.orientation = {0.0, 0.0, s, s};
    sample.twist_present = true;
    sample.twist.linear_x = 2.0;
    plan.samples.push_back(sample);
  }
  Harness harness(plan);
  ASSERT_TRUE(harness.RunUntilK(3));
  for (const PlaybackStep& step : harness.playback().steps()) {
    ExpectStepMatches(harness, step, StateAt(plan, (step.k + 1) * kStepNs), 1);
    const ProbeSample* probe = harness.probe().At(step.sim_time_ns);
    ASSERT_TRUE(probe != nullptr);
    EXPECT_NEAR(probe->velocity[0], 0.0, 1e-12);
    EXPECT_NEAR(probe->velocity[1], 2.0, 1e-12);
    EXPECT_NEAR(probe->velocity[2], 0.0, 1e-12);
  }
}

// ---- T3 swap
// -----------------------------------------------------------------

TEST(TrajectoryPlaybackTest, T3_SwapTakesEffectAtInstallStepPlusOne) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  const VehiclePlanResult plan2 = Epoch2Plan();
  ASSERT_EQ(plan1.status, VehiclePlanStatus::kOk);
  ASSERT_EQ(plan2.status, VehiclePlanStatus::kOk);
  Harness harness(plan1);
  ASSERT_TRUE(harness.RunUntilK(7));
  ASSERT_EQ(harness.playback().steps().size(), 8u);

  EXPECT_EQ(harness.playback().InstallTrajectory(plan2, 7),
            PlaybackInstallError::kOk);
  ASSERT_TRUE(harness.RunUntilK(23));
  const std::vector<PlaybackStep>& steps = harness.playback().steps();
  ASSERT_EQ(steps.size(), 24u);
  for (int64_t k = 0; k <= 7; ++k) {
    ExpectStepMatches(harness, steps[k], StateAt(plan1, (k + 1) * kStepNs), 1);
  }
  // Step 8 plays the epoch-2 plan at t = 0.125.
  EXPECT_EQ(steps[8].trajectory_serial, 2u);
  EXPECT_TRUE(SameState(steps[8].state, StateAt(plan2, kStepNs)));
  for (int64_t k = 8; k < 24; ++k) {
    ExpectStepMatches(harness, steps[k], StateAt(plan2, (k - 7) * kStepNs), 2);
  }
}

TEST(TrajectoryPlaybackTest,
     T3_StaleInstallStepIsRejectedAndKeepsTheTrajectory) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  const VehiclePlanResult plan2 = Epoch2Plan();
  Harness harness(plan1);
  ASSERT_TRUE(harness.RunUntilK(7));
  EXPECT_EQ(harness.playback().InstallTrajectory(plan2, 5),
            PlaybackInstallError::kStaleInstallStep);
  ASSERT_TRUE(harness.RunUntilK(11));
  const std::vector<PlaybackStep>& steps = harness.playback().steps();
  ASSERT_EQ(steps.size(), 12u);
  for (int64_t k = 0; k < 12; ++k) {
    ExpectStepMatches(harness, steps[k], StateAt(plan1, (k + 1) * kStepNs), 1);
  }
}

TEST(TrajectoryPlaybackTest, InstallAtTheLastAppliedStepIsAccepted) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  const VehiclePlanResult plan2 = Epoch2Plan();
  Harness harness(plan1);
  ASSERT_TRUE(harness.RunUntilK(7));
  EXPECT_EQ(harness.playback().InstallTrajectory(plan2, 7),
            PlaybackInstallError::kOk);
  // A later install replaces the pending one before it is applied.
  EXPECT_EQ(harness.playback().InstallTrajectory(plan1, 7),
            PlaybackInstallError::kOk);
  ASSERT_TRUE(harness.RunUntilK(8));
  const PlaybackStep step = *harness.playback().last_step();
  EXPECT_EQ(step.k, 8);
  EXPECT_EQ(step.trajectory_serial, 2u);
  EXPECT_TRUE(SameState(step.state, StateAt(plan1, kStepNs)));
}

// ---- T4 non-exact steps
// -------------------------------------------------------

TEST(TrajectoryPlaybackTest, T4_NonExactServerStepsAreSkipped) {
  const VehiclePlanResult plan = Epoch1Plan();
  constexpr int64_t kPlaybackDtNs = 250000000;
  TrajectoryPlaybackOptions options;
  options.dt_ns = kPlaybackDtNs;
  Harness harness(plan, options);
  ASSERT_TRUE(harness.RunUntilK(5)) << "run_calls=" << harness.run_calls();
  EXPECT_GT(harness.playback().skipped_steps(), 0);
  const std::vector<PlaybackStep>& steps = harness.playback().steps();
  ASSERT_EQ(steps.size(), 6u);
  for (int64_t k = 0; k < 6; ++k) {
    EXPECT_EQ(steps[k].k, k);
    EXPECT_EQ(steps[k].sim_time_ns, (k + 1) * kPlaybackDtNs);
    EXPECT_TRUE(
        SameState(steps[k].state, StateAt(plan, (k + 1) * kPlaybackDtNs)))
        << k;
  }
  // The pose does not change across a skipped (non-exact) server step.
  const std::vector<ProbeSample>& samples = harness.probe().samples();
  int skipped_probe_steps = 0;
  for (size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].sim_time_ns % kPlaybackDtNs != 0) {
      ++skipped_probe_steps;
      EXPECT_EQ(PoseBits(samples[i].pose), PoseBits(samples[i - 1].pose)) << i;
    }
  }
  EXPECT_GT(skipped_probe_steps, 0);
}

// ---- T5 errors
// ----------------------------------------------------------------

TEST(TrajectoryPlaybackTest, T5_InstallDefectsGiveTypedErrorsAndKeepPlayback) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  Harness harness(plan1);
  TrajectoryPlayback& playback = harness.playback();

  VehiclePlanResult not_ok = plan1;
  not_ok.status = VehiclePlanStatus::kNoSolution;
  EXPECT_EQ(playback.InstallTrajectory(not_ok, 0),
            PlaybackInstallError::kPlanNotOk);

  VehiclePlanResult empty = plan1;
  empty.samples.clear();
  EXPECT_EQ(playback.InstallTrajectory(empty, 0),
            PlaybackInstallError::kEmptyTrajectory);

  VehiclePlanResult no_id = plan1;
  no_id.trajectory_id.clear();
  EXPECT_EQ(playback.InstallTrajectory(no_id, 0),
            PlaybackInstallError::kTrajectoryRejected);

  VehiclePlanResult not_valid = plan1;
  not_valid.validity_state = 0;
  EXPECT_EQ(playback.InstallTrajectory(not_valid, 0),
            PlaybackInstallError::kTrajectoryRejected);

  VehiclePlanResult ned = plan1;
  ned.frame_id = "world_ned";
  EXPECT_EQ(playback.InstallTrajectory(ned, 0),
            PlaybackInstallError::kUnsupportedFrame);

  VehiclePlanResult no_twist = plan1;
  no_twist.samples[2].twist_present = false;
  EXPECT_EQ(playback.InstallTrajectory(no_twist, 0),
            PlaybackInstallError::kMissingTimeOrTwist);

  // The trajectory contract check runs before the per-sample check, so a
  // sample without a time is reported as a rejected trajectory.
  VehiclePlanResult no_time = plan1;
  no_time.samples[2].time_present = false;
  EXPECT_EQ(playback.InstallTrajectory(no_time, 0),
            PlaybackInstallError::kTrajectoryRejected);

  VehiclePlanResult shifted = plan1;
  for (TrajectorySampleView& sample : shifted.samples) {
    sample.seconds += 1;
  }
  EXPECT_EQ(playback.InstallTrajectory(shifted, 0),
            PlaybackInstallError::kNonZeroStartTime);

  // None of the defects changed the playback.
  ASSERT_TRUE(harness.RunUntilK(11));
  for (const PlaybackStep& step : playback.steps()) {
    ExpectStepMatches(harness, step, StateAt(plan1, (step.k + 1) * kStepNs), 1);
  }
}

TEST(TrajectoryPlaybackTest, T5_CreateRejectsABadInitialPlanOrOptions) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  PlaybackInstallError error = PlaybackInstallError::kOk;

  VehiclePlanResult not_ok = plan1;
  not_ok.status = VehiclePlanStatus::kNoSolution;
  EXPECT_EQ(TrajectoryPlayback::Create({}, not_ok, &error), nullptr);
  EXPECT_EQ(error, PlaybackInstallError::kPlanNotOk);

  VehiclePlanResult shifted = plan1;
  for (TrajectorySampleView& sample : shifted.samples) {
    sample.seconds += 1;
  }
  EXPECT_EQ(TrajectoryPlayback::Create({}, shifted, &error), nullptr);
  EXPECT_EQ(error, PlaybackInstallError::kNonZeroStartTime);

  for (const int64_t dt_ns : {int64_t{0}, int64_t{-125000000}}) {
    TrajectoryPlaybackOptions options;
    options.dt_ns = dt_ns;
    error = PlaybackInstallError::kOk;
    EXPECT_EQ(TrajectoryPlayback::Create(options, plan1, &error), nullptr)
        << dt_ns;
    EXPECT_NE(error, PlaybackInstallError::kOk) << dt_ns;
  }

  error = PlaybackInstallError::kPlanNotOk;
  EXPECT_NE(TrajectoryPlayback::Create({}, plan1, &error), nullptr);
  EXPECT_EQ(error, PlaybackInstallError::kOk);
}

// ---- T6 repeat
// ----------------------------------------------------------------

TEST(TrajectoryPlaybackTest, T6_RepeatOnAFreshServerIsBitIdentical) {
  const VehiclePlanResult plan1 = Epoch1Plan();
  const VehiclePlanResult plan2 = Epoch2Plan();
  const auto run = [&](Harness* harness) {
    ASSERT_TRUE(harness->RunUntilK(7));
    EXPECT_EQ(harness->playback().InstallTrajectory(plan2, 7),
              PlaybackInstallError::kOk);
    ASSERT_TRUE(harness->RunUntilK(23));
  };
  Harness a(plan1);
  Harness b(plan1);
  run(&a);
  run(&b);
  const std::vector<PlaybackStep>& steps_a = a.playback().steps();
  const std::vector<PlaybackStep>& steps_b = b.playback().steps();
  ASSERT_EQ(steps_a.size(), 24u);
  ASSERT_EQ(steps_a.size(), steps_b.size());
  for (size_t i = 0; i < steps_a.size(); ++i) {
    EXPECT_EQ(steps_a[i].k, steps_b[i].k);
    EXPECT_EQ(steps_a[i].sim_time_ns, steps_b[i].sim_time_ns);
    EXPECT_EQ(steps_a[i].trajectory_serial, steps_b[i].trajectory_serial);
    EXPECT_TRUE(SameState(steps_a[i].state, steps_b[i].state)) << i;
  }
  const std::vector<ProbeSample>& probe_a = a.probe().samples();
  const std::vector<ProbeSample>& probe_b = b.probe().samples();
  ASSERT_EQ(probe_a.size(), probe_b.size());
  for (size_t i = 0; i < probe_a.size(); ++i) {
    EXPECT_EQ(probe_a[i].sim_time_ns, probe_b[i].sim_time_ns) << i;
    EXPECT_EQ(PoseBits(probe_a[i].pose), PoseBits(probe_b[i].pose)) << i;
    for (size_t axis = 0; axis < 3; ++axis) {
      EXPECT_EQ(std::bit_cast<uint64_t>(probe_a[i].velocity[axis]),
                std::bit_cast<uint64_t>(probe_b[i].velocity[axis]))
          << i;
    }
  }
}

// ---- T7 missing entity
// --------------------------------------------------------

TEST(TrajectoryPlaybackTest, T7_MissingEntityCountsAndAppendsNothing) {
  TrajectoryPlaybackOptions options;
  options.model_name = "nope";
  Harness harness(Epoch1Plan(), options);
  harness.RunCalls(12);
  EXPECT_GT(harness.playback().missing_entity_steps(), 0);
  EXPECT_TRUE(harness.playback().steps().empty());
  EXPECT_FALSE(harness.playback().last_step().has_value());
}

TEST(TrajectoryPlaybackTest, MissingLinkCountsAndAppendsNothing) {
  TrajectoryPlaybackOptions options;
  options.link_name = "nope";
  Harness harness(Epoch1Plan(), options);
  harness.RunCalls(12);
  EXPECT_GT(harness.playback().missing_entity_steps(), 0);
  EXPECT_TRUE(harness.playback().steps().empty());
}

}  // namespace
}  // namespace intrinsic::simulation::uuv_reference
