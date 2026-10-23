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

#include "intrinsic/simulation/gazebo/plugins/imu/imu_plugin.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/log/log_entry.h"
#include "absl/log/log_sink.h"
#include "absl/log/log_sink_registry.h"
#include "gtest/gtest.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/simulation/gazebo/plugins/imu/imu_adapter.h"
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessImu;
using ::intrinsic::hardware::marine::ImuAssessment;
using ::intrinsic::hardware::marine::ImuError;
using ::intrinsic::hardware::marine::ImuMeasurementView;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::ImuMeasurement;

constexpr double kHalfPi = std::numbers::pi / 2.0;
constexpr double kGravity = 9.80665;

// The Mover turns the model 90 degrees about +z, sets the link world angular
// velocity to (0, 0, 0.5) rad/s and the link world linear velocity to
// v0 + a t with v0 = (0.5, 0, 0) m/s and a = (1, 0, 0) m/s^2, so the plugin's
// backward difference is a. Then f_b = R^T (a - g) = (0, -1, 9.80665) and
// omega_b = (0, 0, 0.5).
const gz::math::Vector3d kMoverV0(0.5, 0.0, 0.0);
const gz::math::Vector3d kMoverAcceleration(1.0, 0.0, 0.0);
constexpr std::string_view kValidChildren = "";

std::string PluginXml(std::string_view children) {
  return std::string(R"(
      <plugin filename="static://intrinsic::simulation::Imu"
              name="intrinsic::simulation::Imu">)") +
         std::string(children) + R"(
      </plugin>)";
}

std::string ModelWorld(std::string_view model_plugin_xml) {
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="imu_test">
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <model name="uuv">
      <pose>1 2 3 0 0 0</pose>
      <link name="hull">
        <inertial>
          <mass>32</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.4</iyy>
            <iyz>0</iyz>
            <izz>2.7</izz>
          </inertia>
        </inertial>
      </link>
)") + std::string(model_plugin_xml) +
         R"(
    </model>
  </world>
</sdf>
)";
}

// A world with the Physics system. The hull either rests on a ground plane
// or, without ground, falls freely. gravity_z_m_s2 is the SDF world gravity.
std::string PhysicsWorld(bool with_ground, std::string_view model_plugin_xml,
                         double gravity_z_m_s2 = -9.80665) {
  std::ostringstream gravity;
  gravity << std::setprecision(17) << gravity_z_m_s2;
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="imu_physics_test">
    <gravity>0 0 )") +
         gravity.str() + R"(</gravity>
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin filename="static://gz::sim::systems::Physics"
            name="gz::sim::systems::Physics">
    </plugin>)" +
         (with_ground ? R"(
    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>)"
                      : "") +
         R"(
    <model name="uuv">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="hull">
        <inertial>
          <mass>32</mass>
          <inertia>
            <ixx>5.4</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5.4</iyy>
            <iyz>0</iyz>
            <izz>5.4</izz>
          </inertia>
        </inertial>
        <collision name="hull_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>)" +
         std::string(model_plugin_xml) + R"(
    </model>
  </world>
</sdf>
)";
}

// The default MakeSixThrusterUuvExample model in a zero current. Without a
// <current_field> the plugin is inactive and applies no wrench.
constexpr std::string_view kHydrodynamicsPlugin = R"(
      <plugin filename="static://intrinsic::simulation::Hydrodynamics"
              name="intrinsic::simulation::Hydrodynamics">
        <current_field>
          <frame_id>world_enu</frame_id>
          <velocity_m_s>0 0 0</velocity_m_s>
        </current_field>
      </plugin>)";

struct PoseSample {
  double values[7] = {};
  int64_t sim_time_ns = 0;
};

// Records the link world pose and simulation time after every step.
class PoseProbe final : public gz::sim::System,
                        public gz::sim::ISystemPostUpdate {
 public:
  explicit PoseProbe(std::vector<PoseSample>* samples) : samples_(samples) {}

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    if (link == gz::sim::kNullEntity) {
      return;
    }
    const gz::math::Pose3d pose = gz::sim::worldPose(link, ecm);
    PoseSample sample;
    sample.values[0] = pose.Pos().X();
    sample.values[1] = pose.Pos().Y();
    sample.values[2] = pose.Pos().Z();
    sample.values[3] = pose.Rot().X();
    sample.values[4] = pose.Rot().Y();
    sample.values[5] = pose.Rot().Z();
    sample.values[6] = pose.Rot().W();
    sample.sim_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    samples_->push_back(sample);
  }

 private:
  std::vector<PoseSample>* samples_;
};

// Counts PostUpdate calls made while the simulation is paused and while it
// is running.
class PausedCounter final : public gz::sim::System,
                            public gz::sim::ISystemPostUpdate {
 public:
  PausedCounter(std::atomic<int>* paused, std::atomic<int>* running)
      : paused_(paused), running_(running) {}

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& /*ecm*/) override {
    (info.paused ? paused_ : running_)->fetch_add(1);
  }

 private:
  std::atomic<int>* paused_;
  std::atomic<int>* running_;
};

// Moves the model and sets the link's world angular velocity and world linear
// velocity v0 + a t, so the sensor has non-trivial kinematics without a physics
// system. The plugin backward-differences the velocity into the acceleration a.
class Mover final : public gz::sim::System, public gz::sim::ISystemPreUpdate {
 public:
  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
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
    ecm.SetComponentData<gz::sim::components::Pose>(
        model, gz::math::Pose3d(0.1 * step_, 0.0, 3.0 + 0.01 * step_, 0.0, 0.0,
                                kHalfPi));
    ecm.SetComponentData<gz::sim::components::WorldAngularVelocity>(
        link, gz::math::Vector3d(0.0, 0.0, 0.5));
    const double t_s = std::chrono::duration<double>(info.simTime).count();
    ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
        link, kMoverV0 + kMoverAcceleration * t_s);
    ++step_;
  }

 private:
  int step_ = 0;
};

// Collects the ERROR lines written by this plugin.
class ErrorSink final : public absl::LogSink {
 public:
  void Send(const absl::LogEntry& entry) override {
    if (entry.log_severity() != absl::LogSeverity::kError) {
      return;
    }
    const std::string text(entry.text_message());
    if (text.find("Imu:") == std::string::npos) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    lines_.push_back(text);
  }

  std::vector<std::string> Lines() {
    std::lock_guard<std::mutex> lock(mutex_);
    return lines_;
  }

 private:
  std::mutex mutex_;
  std::vector<std::string> lines_;
};

class ImuPartitionEnvironment : public testing::Environment {
 public:
  void SetUp() override {
    // Empty server config skips the default Physics, UserCommands, and
    // SceneBroadcaster systems. HOME avoids a write to /.gz.
    setenv("GZ_SIM_SERVER_CONFIG_PATH", "", 1);
    setenv("HOME", "/tmp", 1);
    setenv("GZ_PARTITION", "intrinsic_imu_test", 1);
  }
};

testing::Environment* const kPartitionEnvironment =
    testing::AddGlobalTestEnvironment(new ImuPartitionEnvironment);

std::unique_ptr<gz::sim::Server> MakeServer(const std::string& sdf,
                                            bool with_physics_engine = false) {
  gz::sim::ServerConfig config;
  config.SetHeadlessRendering(true);
  if (with_physics_engine) {
    config.SetPhysicsEngine(
        "static://gz::physics::bullet_featherstone::Plugin");
  }
  if (!config.SetSdfString(sdf)) {
    return nullptr;
  }
  auto server = std::make_unique<gz::sim::Server>(config);
  server->SetUpdatePeriod(std::chrono::steady_clock::duration::zero());
  return server;
}

// Adds the plugin to the model programmatically so a test can read its
// LastResultForTesting(). The plugin is configured with the same <plugin>
// element.
std::shared_ptr<Imu> AddImu(gz::sim::Server* server, std::string_view children,
                            sdf::Root* root) {
  const std::string wrapped = "<sdf version=\"1.11\"><world name=\"w\">" +
                              PluginXml(children) + "</world></sdf>";
  root->LoadSdfString(wrapped);
  const sdf::Plugins& plugins = root->WorldByIndex(0)->Plugins();
  EXPECT_FALSE(plugins.empty());
  const std::optional<gz::sim::Entity> model = server->EntityByName("uuv");
  EXPECT_TRUE(model.has_value());
  auto imu = std::make_shared<Imu>();
  if (plugins.empty() || !model.has_value()) {
    return imu;
  }
  std::shared_ptr<const sdf::Element> element = plugins.front().Element();
  EXPECT_TRUE(server
                  ->AddSystem(std::static_pointer_cast<gz::sim::System>(imu),
                              model, element)
                  .value_or(false));
  return imu;
}

ImuAssessment Assess(const ImuMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  ImuMeasurementView view;
  const auto& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.expected_frame_present = true;
  view.health.expected_frame = "body";
  view.health.source_time_present = health.header().has_source_time();
  view.health.source_time =
      embodiment::ClockReading{health.header().source_time().seconds(),
                               health.header().source_time().nanos()};
  view.health.receive_time_present = health.header().has_receive_time();
  view.health.receive_time =
      embodiment::ClockReading{health.header().receive_time().seconds(),
                               health.header().receive_time().nanos()};
  view.health.header_validity_present = health.header().has_validity();
  view.health.header_validity_state = health.header().validity().state();
  view.health.state_present = health.has_state();
  view.health.state = health.state();
  view.health.quality_present = health.has_quality();
  view.health.quality = health.quality();
  view.health.covariance_present = health.has_covariance();
  covariance.assign(health.covariance().values().begin(),
                    health.covariance().values().end());
  view.health.covariance = covariance;
  for (const auto& source : health.sources()) {
    sources.push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.health.sources = sources;
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.linear_acceleration_x_present = sample.has_linear_acceleration_x_m_s2();
  view.linear_acceleration_x_m_s2 = sample.linear_acceleration_x_m_s2();
  view.linear_acceleration_y_present = sample.has_linear_acceleration_y_m_s2();
  view.linear_acceleration_y_m_s2 = sample.linear_acceleration_y_m_s2();
  view.linear_acceleration_z_present = sample.has_linear_acceleration_z_m_s2();
  view.linear_acceleration_z_m_s2 = sample.linear_acceleration_z_m_s2();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
  return AssessImu(view);
}

// One recorded plugin output per plugin step.
struct TraceEntry {
  ImuStepResult result;
  std::optional<std::string> bytes;
  ImuKinematicsFreshness freshness;
};

TraceEntry EntryOf(const Imu& imu) {
  TraceEntry entry;
  entry.result = imu.LastResultForTesting();
  entry.freshness = imu.KinematicsFreshness();
  if (entry.result.measurement.has_value()) {
    entry.bytes = entry.result.measurement->SerializeAsString();
  }
  return entry;
}

// Runs single iterations until the plugin has taken `steps` new steps. An
// iteration in which the plugin did not advance by exactly one step is not
// recorded. The number of iterations gz-sim runs per request is not assumed.
std::vector<TraceEntry> StepAndRecord(gz::sim::Server* server, const Imu& imu,
                                      int steps) {
  std::vector<TraceEntry> trace;
  for (int i = 0; i < steps + 2 && static_cast<int>(trace.size()) < steps;
       ++i) {
    const int64_t before = imu.AdapterForTesting().StepIndexForTesting();
    EXPECT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    if (imu.AdapterForTesting().StepIndexForTesting() == before + 1) {
      trace.push_back(EntryOf(imu));
    }
  }
  return trace;
}

// Requests ResetAll and records the first `steps` plugin steps after the
// simulation has applied it. gz-sim applies a reset one iteration after it
// is requested: the iteration right after ResetAll is still an ordinary step
// and is not recorded. The iteration that applies the reset is the one in
// which the plugin's step index goes back down.
std::vector<TraceEntry> ResetAndRecord(gz::sim::Server* server, const Imu& imu,
                                       int steps) {
  server->ResetAll();
  std::vector<TraceEntry> trace;
  bool reset_applied = false;
  for (int i = 0; i < steps + 6 && static_cast<int>(trace.size()) < steps;
       ++i) {
    const int64_t before = imu.AdapterForTesting().StepIndexForTesting();
    EXPECT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    const int64_t after = imu.AdapterForTesting().StepIndexForTesting();
    if (!reset_applied) {
      reset_applied = after < before;
      if (reset_applied && after == 1) {
        trace.push_back(EntryOf(imu));
      }
      continue;
    }
    if (after == before + 1) {
      trace.push_back(EntryOf(imu));
    }
  }
  EXPECT_TRUE(reset_applied);
  return trace;
}

void ExpectTracesEqual(const std::vector<TraceEntry>& lhs,
                       const std::vector<TraceEntry>& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (size_t i = 0; i < lhs.size(); ++i) {
    EXPECT_EQ(lhs[i].result.step, rhs[i].result.step) << i;
    EXPECT_EQ(lhs[i].bytes.has_value(), rhs[i].bytes.has_value()) << i;
    if (lhs[i].bytes.has_value() && rhs[i].bytes.has_value()) {
      EXPECT_EQ(*lhs[i].bytes, *rhs[i].bytes) << i;
    }
  }
}

void ExpectPosesBitIdentical(const std::vector<PoseSample>& lhs,
                             const std::vector<PoseSample>& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (size_t i = 0; i < lhs.size(); ++i) {
    for (int j = 0; j < 7; ++j) {
      EXPECT_EQ(lhs[i].values[j], rhs[i].values[j]) << i << " " << j;
    }
    EXPECT_EQ(lhs[i].sim_time_ns, rhs[i].sim_time_ns) << i;
  }
}

std::vector<PoseSample> RunWorld(const std::string& sdf, int steps,
                                 bool add_imu,
                                 std::string_view children = kValidChildren) {
  std::vector<PoseSample> poses;
  std::unique_ptr<gz::sim::Server> server = MakeServer(sdf);
  EXPECT_TRUE(server != nullptr);
  if (server == nullptr) {
    return poses;
  }
  EXPECT_TRUE(
      server->AddSystem(std::make_shared<PoseProbe>(&poses)).value_or(false));
  EXPECT_TRUE(server->AddSystem(std::make_shared<Mover>()).value_or(false));
  sdf::Root root;
  if (add_imu) {
    AddImu(server.get(), children, &root);
  }
  EXPECT_TRUE(server->Run(/*_blocking=*/true, steps, /*_paused=*/false));
  return poses;
}

}  // namespace

// ---- P1: lifecycle --------------------------------------------------------

TEST(ImuPluginTest, ConfigureStepResetUnloadRunCleanly) {
  ASSERT_EQ(ImuStaticPluginAnchorForTesting(), 1);
  const std::string children =
      "<latency_steps>2</latency_steps>"
      "<angular_noise_amplitude_rad_s>0.05</angular_noise_amplitude_rad_s>"
      "<angular_drift_rad_s2>0.001</angular_drift_rad_s2>"
      "<dropout_window><start_step>4</start_step>"
      "<end_step>6</end_step></dropout_window>";
  std::vector<TraceEntry> first_run;
  std::vector<TraceEntry> after_reset;
  {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Imu> imu = AddImu(server.get(), children, &root);
    ASSERT_TRUE(imu->AdapterForTesting().ConfigStatusForTesting().ok());
    first_run = StepAndRecord(server.get(), *imu, 12);
    EXPECT_EQ(imu->AdapterForTesting().StepIndexForTesting(), 12);

    after_reset = ResetAndRecord(server.get(), *imu, 12);
  }
  ASSERT_EQ(first_run.size(), 12u);
  for (int k = 0; k < 12; ++k) {
    EXPECT_EQ(first_run[k].result.step, k);
    // Step 0 is the warm-up: no previous velocity, so kInvalidState.
    EXPECT_EQ(first_run[k].result.status.code,
              k == 0 ? ImuSimErrorCode::kInvalidState : ImuSimErrorCode::kOk)
        << k;
    // Latency 2: startup absent, and the warm-up slot (step 0) arrives empty at
    // step 2. Dropout [4, 6) shifted by the latency.
    const bool absent = k < 3 || k == 6 || k == 7;
    EXPECT_EQ(first_run[k].bytes.has_value(), !absent) << k;
    if (!absent) {
      const ImuAssessment assessment = Assess(*first_run[k].result.measurement);
      EXPECT_EQ(assessment.error, ImuError::kNone) << k;
      EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded) << k;
      EXPECT_FALSE(assessment.accepted) << k;
    }
  }
  // Reset clears the queue, the step counter, and the drift origin: the trace
  // equals a run from a fresh server, and startup is absent again.
  ExpectTracesEqual(first_run, after_reset);
  EXPECT_FALSE(after_reset[0].bytes.has_value());
  EXPECT_FALSE(after_reset[1].bytes.has_value());
  EXPECT_FALSE(after_reset[2].bytes.has_value());

  std::vector<TraceEntry> fresh_run;
  {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Imu> imu = AddImu(server.get(), children, &root);
    fresh_run = StepAndRecord(server.get(), *imu, 12);
  }
  ExpectTracesEqual(first_run, fresh_run);
}

TEST(ImuPluginTest, PausedStepsDoNotAdvanceTheStepIndex) {
  std::atomic<int> paused_updates{0};
  std::atomic<int> running_updates{0};
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  ASSERT_TRUE(server
                  ->AddSystem(std::make_shared<PausedCounter>(&paused_updates,
                                                              &running_updates))
                  .value_or(false));
  sdf::Root root;
  const std::shared_ptr<Imu> imu = AddImu(server.get(), kValidChildren, &root);

  // A paused blocking run never completes its iterations, so run paused in
  // the background until the plugin's neighbours have seen paused updates.
  ASSERT_TRUE(server->Run(/*_blocking=*/false, 0, /*_paused=*/true));
  for (int i = 0; i < 2000 && paused_updates.load() < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  server->Stop();
  for (int i = 0; i < 2000 && server->Running(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_FALSE(server->Running());
  EXPECT_GE(paused_updates.load(), 5);
  EXPECT_EQ(imu->AdapterForTesting().StepIndexForTesting(), 0);
  EXPECT_FALSE(imu->LastResultForTesting().measurement.has_value());

  ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
  // The first running step is the warm-up and has no sample. The measurement
  // comes on the next running step.
  EXPECT_FALSE(imu->LastResultForTesting().measurement.has_value());
  ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
  // The plugin's step index equals the number of non-paused updates the
  // server made, however many iterations gz-sim ran for the request.
  const int running = running_updates.load();
  ASSERT_GE(running, 2);
  EXPECT_EQ(imu->AdapterForTesting().StepIndexForTesting(), running);
  ASSERT_TRUE(imu->LastResultForTesting().measurement.has_value());
  EXPECT_EQ(imu->LastResultForTesting().step, running - 1);
  EXPECT_EQ(
      imu->LastResultForTesting().measurement->health().header().sequence(),
      42u + static_cast<uint64_t>(running - 1));
}

// ---- P2: reads pose, rates, acceleration, and time -------------------------

TEST(ImuPluginTest, ReadsLinkPoseRateAccelerationAndSimulationTime) {
  std::vector<PoseSample> poses;
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  ASSERT_TRUE(
      server->AddSystem(std::make_shared<PoseProbe>(&poses)).value_or(false));
  ASSERT_TRUE(server->AddSystem(std::make_shared<Mover>()).value_or(false));
  sdf::Root root;
  const std::shared_ptr<Imu> imu = AddImu(server.get(), kValidChildren, &root);
  int checked = 0;
  for (int i = 0; i < 12 && checked < 6; ++i) {
    const int64_t before = imu->AdapterForTesting().StepIndexForTesting();
    ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    if (imu->AdapterForTesting().StepIndexForTesting() != before + 1) {
      continue;
    }
    ASSERT_EQ(poses.size(), static_cast<size_t>(before + 1));
    const PoseSample& pose = poses.back();
    const ImuStepResult& result = imu->LastResultForTesting();
    if (before == 0) {
      // Warm-up: no previous velocity yet, so no sample.
      EXPECT_EQ(result.status.code, ImuSimErrorCode::kInvalidState);
      EXPECT_FALSE(result.measurement.has_value());
      continue;
    }
    ASSERT_TRUE(result.status.ok()) << result.status.message;
    ASSERT_TRUE(result.measurement.has_value()) << checked;
    const ImuMeasurement& sample = *result.measurement;
    // Yaw +90 degrees, world rate (0, 0, 0.5), world acceleration (1, 0, 0).
    EXPECT_NEAR(sample.angular_velocity_x_rad_s(), 0.0, 1e-12) << checked;
    EXPECT_NEAR(sample.angular_velocity_y_rad_s(), 0.0, 1e-12) << checked;
    EXPECT_NEAR(sample.angular_velocity_z_rad_s(), 0.5, 1e-12) << checked;
    EXPECT_NEAR(sample.linear_acceleration_x_m_s2(), 0.0, 1e-12) << checked;
    EXPECT_NEAR(sample.linear_acceleration_y_m_s2(), -1.0, 1e-12) << checked;
    EXPECT_NEAR(sample.linear_acceleration_z_m_s2(), kGravity, 1e-12)
        << checked;
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    ASSERT_TRUE(sample.has_orientation_xyzw());
    EXPECT_NEAR(sample.orientation_xyzw().z(), pose.values[5], 1e-12);
    EXPECT_NEAR(sample.orientation_xyzw().w(), pose.values[6], 1e-12);
    const auto& source = sample.health().header().source_time();
    EXPECT_EQ(source.seconds() * 1000000000LL + source.nanos(),
              pose.sim_time_ns)
        << checked;
    EXPECT_EQ(sample.health().header().receive_time().seconds(),
              source.seconds());
    EXPECT_EQ(sample.health().header().receive_time().nanos(), source.nanos());
    const ImuAssessment assessment = Assess(sample);
    EXPECT_EQ(assessment.error, ImuError::kNone) << checked;
    EXPECT_TRUE(assessment.accepted) << checked;
    ++checked;
  }
  EXPECT_EQ(checked, 6);
}

// ---- P3: opt-in regression ------------------------------------------------

TEST(ImuPluginTest, SensorIsReadOnlyAndWorldWithoutPluginIsUnchanged) {
  constexpr int kSteps = 20;
  const std::vector<PoseSample> bare =
      RunWorld(ModelWorld(""), kSteps, /*add_imu=*/false);
  const std::vector<PoseSample> added =
      RunWorld(ModelWorld(""), kSteps, /*add_imu=*/true);
  const std::vector<PoseSample> from_sdf =
      RunWorld(ModelWorld(PluginXml(kValidChildren)), kSteps,
               /*add_imu=*/false);
  ASSERT_EQ(bare.size(), static_cast<size_t>(kSteps));
  ExpectPosesBitIdentical(bare, added);
  ExpectPosesBitIdentical(bare, from_sdf);
  // The Mover really moves the link, so identical poses are not trivial.
  EXPECT_NE(bare.front().values[0], bare.back().values[0]);
}

TEST(ImuPluginTest, StaticAliasLoadsFromSdfAndUnloadsCleanly) {
  {
    const std::vector<PoseSample> poses =
        RunWorld(ModelWorld(PluginXml(kValidChildren)), 3, false);
    EXPECT_EQ(poses.size(), 3u);
  }
  const std::vector<PoseSample> again =
      RunWorld(ModelWorld(PluginXml(kValidChildren)), 2, false);
  EXPECT_EQ(again.size(), 2u);
}

// ---- P4: invalid SDF ------------------------------------------------------

TEST(ImuPluginTest, InvalidConfigurationProducesNoOutputAndOneLogLine) {
  const std::string_view bad[] = {
      "<angular_noise_amplitude_rad_s>nan</angular_noise_amplitude_rad_s>",
      "<linear_noise_amplitude_m_s2>-1</linear_noise_amplitude_m_s2>",
      "<angular_bias_x_rad_s>1.0x</angular_bias_x_rad_s>",
      "<linear_drift_m_s3>inf</linear_drift_m_s3>",
      "<latency_steps>1001</latency_steps>",
      "<dropout_window><start_step>5</start_step><end_step>4</end_step>"
      "</dropout_window>",
  };
  for (const std::string_view children : bad) {
    ErrorSink sink;
    absl::AddLogSink(&sink);
    {
      std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
      ASSERT_TRUE(server != nullptr);
      sdf::Root root;
      const std::shared_ptr<Imu> imu = AddImu(server.get(), children, &root);
      EXPECT_FALSE(imu->AdapterForTesting().ConfigStatusForTesting().ok())
          << children;
      for (int k = 0; k < 3; ++k) {
        ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
        EXPECT_FALSE(imu->LastResultForTesting().measurement.has_value())
            << children;
      }
      EXPECT_EQ(imu->AdapterForTesting().StepIndexForTesting(), 0) << children;
      server->ResetAll();
      ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
      EXPECT_FALSE(imu->LastResultForTesting().measurement.has_value())
          << children;
    }
    absl::RemoveLogSink(&sink);
    EXPECT_EQ(sink.Lines().size(), 1u) << children;
  }
  const std::vector<PoseSample> poses =
      RunWorld(ModelWorld(PluginXml("<angular_noise_amplitude_rad_s>nan"
                                    "</angular_noise_amplitude_rad_s>")),
               3, /*add_imu=*/false);
  EXPECT_EQ(poses.size(), 3u);
}

TEST(ImuPluginTest, UnknownLinkIsInvalidStateWithNoSampleAndOneLogLine) {
  ErrorSink sink;
  absl::AddLogSink(&sink);
  {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Imu> imu =
        AddImu(server.get(), "<link_name>missing_hull</link_name>", &root);
    ASSERT_TRUE(imu->AdapterForTesting().ConfigStatusForTesting().ok());
    for (int k = 0; k < 4; ++k) {
      ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
      EXPECT_EQ(imu->LastResultForTesting().status.code,
                ImuSimErrorCode::kInvalidState);
      EXPECT_FALSE(imu->LastResultForTesting().measurement.has_value());
    }
  }
  absl::RemoveLogSink(&sink);
  // One line for the unknown link at Configure, and one de-duplicated line
  // for the repeated invalid state.
  EXPECT_EQ(sink.Lines().size(), 2u);
}

TEST(ImuPluginTest, NamedLinkIsResolved) {
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  sdf::Root root;
  const std::shared_ptr<Imu> imu =
      AddImu(server.get(), "<link_name>hull</link_name>", &root);
  ASSERT_TRUE(server->Run(/*_blocking=*/true, 2, /*_paused=*/false));
  ASSERT_TRUE(imu->LastResultForTesting().measurement.has_value());
  EXPECT_TRUE(Assess(*imu->LastResultForTesting().measurement).accepted);
}

// ---- I10, H1, H2: true acceleration, gravity counted once ------------------

namespace {

constexpr int kWarmUpSteps = 1;

// Everything the I10 and H tests need about one plugin step.
struct PhysicsStep {
  int step = 0;
  ImuSimErrorCode code = ImuSimErrorCode::kOk;
  bool has_sample = false;
  ImuKinematicsFreshness freshness;
  // Specific force and angular rate in the body frame, from the sample.
  std::array<double, 3> specific_force_m_s2 = {};
  std::array<double, 3> body_rate_rad_s = {};
  // World linear acceleration recovered from the sample: a = R f_b + G where
  // G = (0, 0, -g) is the adapter's gravity.
  std::array<double, 3> world_acceleration_m_s2 = {};
  // Link world pose and linear velocity read from the ECM after the step.
  double z_m = 0;
  std::array<double, 3> world_velocity_m_s = {};
};

struct PhysicsTrace {
  std::vector<PhysicsStep> steps;
  std::vector<std::string> error_lines;
  int64_t sim_time_ns = 0;
};

// Records the link world linear velocity after every step.
class VelocityProbe final : public gz::sim::System,
                            public gz::sim::ISystemPostUpdate {
 public:
  explicit VelocityProbe(std::vector<std::array<double, 3>>* samples)
      : samples_(samples) {}

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    if (link == gz::sim::kNullEntity) {
      return;
    }
    const gz::sim::components::WorldLinearVelocity* velocity =
        ecm.Component<gz::sim::components::WorldLinearVelocity>(link);
    const gz::math::Vector3d value =
        velocity == nullptr ? gz::math::Vector3d::Zero : velocity->Data();
    samples_->push_back({value.X(), value.Y(), value.Z()});
  }

 private:
  std::vector<std::array<double, 3>>* samples_;
};

PhysicsTrace RunPhysicsTrace(bool with_ground,
                             std::string_view model_plugin_xml, int steps,
                             double gravity_z_m_s2 = -9.80665) {
  PhysicsTrace out;
  std::vector<PoseSample> poses;
  std::vector<std::array<double, 3>> velocities;
  ErrorSink sink;
  absl::AddLogSink(&sink);
  {
    std::unique_ptr<gz::sim::Server> server =
        MakeServer(PhysicsWorld(with_ground, model_plugin_xml, gravity_z_m_s2),
                   /*with_physics_engine=*/true);
    EXPECT_TRUE(server != nullptr);
    if (server == nullptr) {
      absl::RemoveLogSink(&sink);
      return out;
    }
    EXPECT_TRUE(
        server->AddSystem(std::make_shared<PoseProbe>(&poses)).value_or(false));
    EXPECT_TRUE(server->AddSystem(std::make_shared<VelocityProbe>(&velocities))
                    .value_or(false));
    sdf::Root root;
    const std::shared_ptr<Imu> imu = AddImu(server.get(), "", &root);
    const std::vector<TraceEntry> trace =
        StepAndRecord(server.get(), *imu, steps);
    EXPECT_EQ(static_cast<int>(trace.size()), steps);
    EXPECT_EQ(poses.size(), velocities.size());
    for (const TraceEntry& entry : trace) {
      PhysicsStep step;
      step.step = static_cast<int>(entry.result.step);
      step.code = entry.result.status.code;
      step.has_sample = entry.result.measurement.has_value();
      step.freshness = entry.freshness;
      if (step.step < 0 || static_cast<size_t>(step.step) >= poses.size() ||
          static_cast<size_t>(step.step) >= velocities.size()) {
        ADD_FAILURE() << "no pose or velocity for step " << step.step;
        continue;
      }
      const PoseSample& pose = poses[step.step];
      step.z_m = pose.values[2];
      step.world_velocity_m_s = velocities[step.step];
      if (step.has_sample) {
        const ImuMeasurement& m = *entry.result.measurement;
        step.specific_force_m_s2 = {m.linear_acceleration_x_m_s2(),
                                    m.linear_acceleration_y_m_s2(),
                                    m.linear_acceleration_z_m_s2()};
        step.body_rate_rad_s = {m.angular_velocity_x_rad_s(),
                                m.angular_velocity_y_rad_s(),
                                m.angular_velocity_z_rad_s()};
        const gz::math::Quaterniond q(pose.values[6], pose.values[3],
                                      pose.values[4], pose.values[5]);
        const gz::math::Vector3d a =
            q.RotateVector(gz::math::Vector3d(step.specific_force_m_s2[0],
                                              step.specific_force_m_s2[1],
                                              step.specific_force_m_s2[2])) +
            gz::math::Vector3d(0.0, 0.0, -kGravity);
        step.world_acceleration_m_s2 = {a.X(), a.Y(), a.Z()};
      }
      out.steps.push_back(step);
    }
    out.sim_time_ns = poses.empty() ? 0 : poses.back().sim_time_ns;
  }
  absl::RemoveLogSink(&sink);
  out.error_lines = sink.Lines();
  return out;
}

// Step 0 has no previous velocity: kInvalidState, no sample, at most one ERROR
// line. Every later step has a sample.
void ExpectWarmUpThenSamples(const PhysicsTrace& trace) {
  ASSERT_GT(trace.steps.size(), static_cast<size_t>(kWarmUpSteps));
  EXPECT_EQ(trace.steps[0].step, 0);
  EXPECT_EQ(trace.steps[0].code, ImuSimErrorCode::kInvalidState);
  EXPECT_FALSE(trace.steps[0].has_sample);
  EXPECT_LE(trace.error_lines.size(), 1u);
  for (size_t k = kWarmUpSteps; k < trace.steps.size(); ++k) {
    EXPECT_EQ(trace.steps[k].code, ImuSimErrorCode::kOk) << k;
    EXPECT_TRUE(trace.steps[k].has_sample) << k;
  }
}

std::array<double, 3> MeanSpecificForce(const PhysicsTrace& trace, int window) {
  std::array<double, 3> mean = {};
  const size_t first = trace.steps.size() - window;
  for (size_t k = first; k < trace.steps.size(); ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      mean[axis] += trace.steps[k].specific_force_m_s2[axis] / window;
    }
  }
  return mean;
}

double Norm(const std::array<double, 3>& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

// ---- Constants of lock imu-true-acceleration-hydro-gravity-v3 ---------------

constexpr double kStepS = 0.001;

// E4 (resting cases I10a and I10b).
constexpr int kRestingSettleSteps = 20;
constexpr int kPhysicsMeanWindow = 100;
constexpr double kMeanTolerance = 1e-3;
constexpr double kRestingSampleBound = 0.1;
// I10c.
constexpr double kFreeFallBound = 1e-6;

// E2: vertical drag model m dv/dt = F - c_l v - c_q v |v|.
// Source: intrinsic_vehicle/intrinsic/vehicle/parameters/
// six_thruster_uuv_example.cc (mass 32 kg, linear 35 N s/m, quadratic
// 90 N s^2/m^2 heave damping, buoyancy 1.005 W).
constexpr double kMassKg = 32.0;
constexpr double kLinearHeaveDragNsPerM = 35.0;
constexpr double kQuadraticHeaveDragNs2PerM2 = 90.0;
constexpr double kBuoyancyMinusWeightN = 0.005 * kMassKg * kGravity;
constexpr double kDoubleCountForceN =
    kBuoyancyMinusWeightN - kMassKg * kGravity;
constexpr int kH2CheckedSteps = 50;
constexpr double kH2AccelTolerance = 5e-3;
constexpr int kH1FirstBandStep = 20;
constexpr int kH1Steps = 1000;
constexpr double kH1AccelTolerance = 5e-4;
constexpr double kH1VelocityMin = 0.02894;
constexpr double kH1VelocityMax = 0.02914;
constexpr double kH1LateralAccelBound = 1e-4;
// H1 steps 1-19 are only checked to be finite and in [0, 0.06].
constexpr double kH1SettleAccelMin = 0.0;
constexpr double kH1SettleAccelMax = 0.06;

// F3: pitch rate of the H1 hull, I theta'' + d_l theta' + K theta = tau0.
// Sources, all in intrinsic_vehicle/intrinsic/vehicle/parameters/
// six_thruster_uuv_example.cc unless noted: mass (kMassKg line 23), gravity
// (kGravityMPerS2 line 24), fluid density (line 25), displaced volume
// 1.005 m / rho (lines 27-28), center_of_gravity_m (line 74),
// center_of_buoyancy _m (line 75), pitch linear damping (line 82,
// DiagonalSpatial 5th entry), and the hull's <iyy> in PhysicsWorld() in this
// file. The quadratic pitch damping (line 86, 4.0) is omitted: it is below 1%
// of the linear term for the rates reached here.
constexpr double kFluidDensityKgPerM3 = 1025.0;
constexpr double kDisplacedVolumeM3 = 1.005 * kMassKg / kFluidDensityKgPerM3;
constexpr double kCenterOfGravityXM = 0.02;
constexpr double kCenterOfGravityZM = -0.01;
constexpr double kCenterOfBuoyancyZM = 0.03;
constexpr double kLinearPitchDragNmsPerRad = 1.5;
constexpr double kPitchInertiaKgM2 = 5.4;
constexpr double kPitchRateTolerance = 2e-5;
constexpr double kOffAxisRateBound = 1e-6;

double HeaveVelocityM_s(double force_n, double t_s) {
  // m v' = -c_q (u - r1)(u - r2) for u = |v| with the sign of force_n, where
  // r1 > 0 > r2 are the roots of c_q u^2 + c_l u - |F| = 0.
  const double sign = force_n >= 0 ? 1.0 : -1.0;
  const double f = std::fabs(force_n);
  const double disc =
      std::sqrt(kLinearHeaveDragNsPerM * kLinearHeaveDragNsPerM +
                4.0 * kQuadraticHeaveDragNs2PerM2 * f);
  const double r1 =
      (-kLinearHeaveDragNsPerM + disc) / (2.0 * kQuadraticHeaveDragNs2PerM2);
  const double r2 =
      (-kLinearHeaveDragNsPerM - disc) / (2.0 * kQuadraticHeaveDragNs2PerM2);
  const double e = (r1 / r2) * std::exp(-kQuadraticHeaveDragNs2PerM2 *
                                        (r1 - r2) * t_s / kMassKg);
  return sign * (r1 - r2 * e) / (1.0 - e);
}

// Expected world vertical acceleration of step k: the force at the velocity at
// the start of the step (sim time k ms), over the mass.
double ExpectedHeaveAccelM_s2(double force_n, int step) {
  const double v = HeaveVelocityM_s(force_n, step * kStepS);
  return (force_n - kLinearHeaveDragNsPerM * v -
          kQuadraticHeaveDragNs2PerM2 * v * std::fabs(v)) /
         kMassKg;
}

// Body pitch rate at the end of step k, t = (k + 1) ms.
double ExpectedPitchRateRadS(int step) {
  const double weight_n = kMassKg * kGravity;
  const double buoyancy_n =
      kFluidDensityKgPerM3 * kDisplacedVolumeM3 * kGravity;
  const double tau0 = -kCenterOfGravityXM * (buoyancy_n - weight_n);
  const double stiffness =
      buoyancy_n * kCenterOfBuoyancyZM - weight_n * kCenterOfGravityZM;
  const double wn = std::sqrt(stiffness / kPitchInertiaKgM2);
  const double sigma = kLinearPitchDragNmsPerRad / (2.0 * kPitchInertiaKgM2);
  const double wd = std::sqrt(wn * wn - sigma * sigma);
  const double t = (step + 1) * kStepS;
  return tau0 / (kPitchInertiaKgM2 * wd) * std::exp(-sigma * t) *
         std::sin(wd * t);
}

std::string FreshnessPattern(const PhysicsTrace& trace, int first, int last) {
  std::string pattern;
  for (int k = first; k <= last; ++k) {
    pattern += trace.steps[k].freshness.fresh ? 'F' : 'g';
  }
  return pattern;
}

}  // namespace

TEST(ImuPluginTest, RestingLinkWithGravityMeasuresOneGNotZeroNotTwoG) {
  const PhysicsTrace trace =
      RunPhysicsTrace(/*with_ground=*/true, "", /*steps=*/300);
  ExpectWarmUpThenSamples(trace);
  ASSERT_EQ(trace.steps.size(), 300u);
  // The box sits on the ground plane, so the support force is what the
  // accelerometer measures: f_b = (0, 0, +g).
  EXPECT_NEAR(trace.steps.back().z_m, 0.5, 5e-2);
  const std::array<double, 3> mean =
      MeanSpecificForce(trace, kPhysicsMeanWindow);
  EXPECT_NEAR(mean[0], 0.0, kMeanTolerance);
  EXPECT_NEAR(mean[1], 0.0, kMeanTolerance);
  EXPECT_NEAR(mean[2], kGravity, kMeanTolerance);
  EXPECT_GT(std::fabs(mean[2]), 1.0);
  EXPECT_GT(std::fabs(std::fabs(mean[2]) - 2 * kGravity), 1.0);
  // Steps 1-19 are a settle window (the box lands). Differencing a 1 ms
  // contact step amplifies solver jitter, which is bounded per sample after it.
  for (size_t k = kRestingSettleSteps; k < trace.steps.size(); ++k) {
    EXPECT_LE(std::fabs(trace.steps[k].specific_force_m_s2[2] - kGravity),
              kRestingSampleBound)
        << k;
  }
}

TEST(ImuPluginTest, RestingLinkWithHydrodynamicsDoesNotLoseOrDoubleGravity) {
  // This world breaks the zero-gravity rule: it loads Hydrodynamics and runs
  // Physics with gravity. bullet-featherstone ignores GravityEnabledCmd, so
  // engine gravity and the plugin's weight both act. The hull stays on the
  // ground, the contact carries the excess, the true acceleration is about 0,
  // and the specific force is +g.
  const PhysicsTrace trace = RunPhysicsTrace(
      /*with_ground=*/true, kHydrodynamicsPlugin, /*steps=*/300);
  ExpectWarmUpThenSamples(trace);
  ASSERT_EQ(trace.steps.size(), 300u);
  EXPECT_NEAR(trace.steps.back().z_m, 0.5, 5e-2);
  const std::array<double, 3> mean =
      MeanSpecificForce(trace, kPhysicsMeanWindow);
  EXPECT_NEAR(mean[0], 0.0, kMeanTolerance);
  EXPECT_NEAR(mean[1], 0.0, kMeanTolerance);
  EXPECT_NEAR(mean[2], kGravity, kMeanTolerance);
  for (size_t k = kRestingSettleSteps; k < trace.steps.size(); ++k) {
    EXPECT_LE(std::fabs(trace.steps[k].specific_force_m_s2[2] - kGravity),
              kRestingSampleBound)
        << k;
  }
}

TEST(ImuPluginTest, FreeFallCharacterizesTheEnginesAccelerationOutput) {
  // The default bullet-featherstone engine does not fill the link
  // acceleration, so the plugin differences the link world velocity. A
  // semi-implicit step makes that difference the applied acceleration, so a
  // free-falling accelerometer reads 0.
  const PhysicsTrace trace =
      RunPhysicsTrace(/*with_ground=*/false, "", /*steps=*/200);
  ExpectWarmUpThenSamples(trace);
  ASSERT_EQ(trace.steps.size(), 200u);
  EXPECT_LT(trace.steps.back().z_m, 0.5 - 0.05);
  for (size_t k = kWarmUpSteps; k < trace.steps.size(); ++k) {
    EXPECT_LE(Norm(trace.steps[k].specific_force_m_s2), kFreeFallBound) << k;
  }
}

TEST(ImuPluginTest, HydrodynamicsForceReachesTheEngineInAZeroGravityWorld) {
  // H1. A free hull with Hydrodynamics in a world with <gravity>0 0 0</gravity>
  // is pushed only by the plugin's restoring wrench, B - W = 0.005 m g. Its
  // vertical acceleration follows the drag closed form, f_z = a + g, and it
  // pitches as a damped hydrostatic oscillator.
  const PhysicsTrace trace =
      RunPhysicsTrace(/*with_ground=*/false, kHydrodynamicsPlugin, kH1Steps,
                      /*gravity_z_m_s2=*/0.0);
  ExpectWarmUpThenSamples(trace);
  ASSERT_EQ(trace.steps.size(), static_cast<size_t>(kH1Steps));
  RecordProperty("h1_freshness_steps_1_to_19",
                 FreshnessPattern(trace, 1, kH1FirstBandStep - 1));
  RecordProperty("h1_gated_steps",
                 std::to_string(trace.steps.back().freshness.gated_steps));

  // Steps 1-19: the pose gate leaves the velocity stale, so only sanity.
  for (int k = 1; k < kH1FirstBandStep; ++k) {
    const double a = trace.steps[k].world_acceleration_m_s2[2];
    EXPECT_TRUE(std::isfinite(a)) << k;
    EXPECT_GE(a, kH1SettleAccelMin) << k;
    EXPECT_LE(a, kH1SettleAccelMax) << k;
  }
  for (int k = kH1FirstBandStep; k < kH1Steps; ++k) {
    const PhysicsStep& step = trace.steps[k];
    EXPECT_TRUE(step.freshness.fresh) << k;
    const double expected = ExpectedHeaveAccelM_s2(kBuoyancyMinusWeightN, k);
    EXPECT_NEAR(step.world_acceleration_m_s2[2], expected, kH1AccelTolerance)
        << k;
    EXPECT_NEAR(step.specific_force_m_s2[2], expected + kGravity,
                kH1AccelTolerance)
        << k;
    EXPECT_LE(std::fabs(step.world_acceleration_m_s2[0]), kH1LateralAccelBound)
        << k;
    EXPECT_LE(std::fabs(step.world_acceleration_m_s2[1]), kH1LateralAccelBound)
        << k;
    EXPECT_NEAR(step.body_rate_rad_s[1], ExpectedPitchRateRadS(k),
                kPitchRateTolerance)
        << k;
    EXPECT_LE(std::fabs(step.body_rate_rad_s[0]), kOffAxisRateBound) << k;
    EXPECT_LE(std::fabs(step.body_rate_rad_s[2]), kOffAxisRateBound) << k;
  }
  const PhysicsStep& last = trace.steps.back();
  EXPECT_EQ(trace.sim_time_ns, 1000000000);
  EXPECT_GE(last.world_velocity_m_s[2], kH1VelocityMin);
  EXPECT_LE(last.world_velocity_m_s[2], kH1VelocityMax);
  EXPECT_GT(last.z_m, 0.5);
}

TEST(ImuPluginTest, HydrodynamicsInAGravityWorldCountsGravityTwice) {
  // H2, characterization. The same hull in a world with gravity. The engine
  // ignores GravityEnabledCmd, so engine gravity and the plugin's weight both
  // act: m a = (B - W) - m g - drag, about -0.995 g at first. If the engine
  // starts honoring GravityEnabledCmd, a becomes about +0.049 and this fails,
  // so the expectations get reviewed.
  const PhysicsTrace trace = RunPhysicsTrace(
      /*with_ground=*/false, kHydrodynamicsPlugin, /*steps=*/200);
  ExpectWarmUpThenSamples(trace);
  ASSERT_EQ(trace.steps.size(), 200u);
  for (int k = 1; k <= kH2CheckedSteps; ++k) {
    EXPECT_TRUE(trace.steps[k].freshness.fresh) << k;
    EXPECT_NEAR(trace.steps[k].world_acceleration_m_s2[2],
                ExpectedHeaveAccelM_s2(kDoubleCountForceN, k),
                kH2AccelTolerance)
        << k;
  }
}

}  // namespace intrinsic::simulation
