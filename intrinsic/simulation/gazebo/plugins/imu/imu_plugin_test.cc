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
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
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
// velocity to (0, 0, 0.5) rad/s and the link world linear acceleration to
// (1, 0, 0) m/s^2. Then f_b = R^T (a - g) = (0, -1, 9.80665) and
// omega_b = (0, 0, 0.5).
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
// or, without ground, falls freely. gravity is the SDF world gravity.
std::string PhysicsWorld(bool with_ground, std::string_view model_plugin_xml) {
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="imu_physics_test">
    <gravity>0 0 -9.80665</gravity>
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin filename="static://gz::sim::systems::Physics"
            name="gz::sim::systems::Physics">
    </plugin>)") +
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

constexpr std::string_view kHydrodynamicsPlugin = R"(
      <plugin filename="static://intrinsic::simulation::Hydrodynamics"
              name="intrinsic::simulation::Hydrodynamics">
        <current_field>
          <frame_id>world_enu</frame_id>
          <velocity_m_s>0.2 -0.1 0</velocity_m_s>
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
// acceleration, so the sensor has non-trivial kinematics without a physics
// system.
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
    ecm.SetComponentData<gz::sim::components::WorldLinearAcceleration>(
        link, gz::math::Vector3d(1.0, 0.0, 0.0));
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
};

TraceEntry EntryOf(const Imu& imu) {
  TraceEntry entry;
  entry.result = imu.LastResultForTesting();
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
    EXPECT_TRUE(first_run[k].result.status.ok()) << k;
    // Latency 2: startup absent. Dropout [4, 6) shifted by the latency.
    const bool absent = k < 2 || k == 6 || k == 7;
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
  // The plugin's step index equals the number of non-paused updates the
  // server made, however many iterations gz-sim ran for the request.
  const int running = running_updates.load();
  ASSERT_GE(running, 1);
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

// ---- I10: gravity double-count guard ---------------------------------------

namespace {

struct RestingObservation {
  bool ok = false;
  std::array<double, 3> specific_force_m_s2 = {};
  double hull_z_m = 0;
  int steps = 0;
};

RestingObservation RunPhysics(bool with_ground,
                              std::string_view model_plugin_xml,
                              std::string_view imu_children, int steps) {
  RestingObservation observation;
  std::vector<PoseSample> poses;
  std::unique_ptr<gz::sim::Server> server =
      MakeServer(PhysicsWorld(with_ground, model_plugin_xml),
                 /*with_physics_engine=*/true);
  EXPECT_TRUE(server != nullptr);
  if (server == nullptr) {
    return observation;
  }
  EXPECT_TRUE(
      server->AddSystem(std::make_shared<PoseProbe>(&poses)).value_or(false));
  sdf::Root root;
  const std::shared_ptr<Imu> imu = AddImu(server.get(), imu_children, &root);
  EXPECT_TRUE(server->Run(/*_blocking=*/true, steps, /*_paused=*/false));
  const ImuStepResult& result = imu->LastResultForTesting();
  EXPECT_TRUE(result.status.ok()) << result.status.message;
  if (!result.measurement.has_value() || poses.empty()) {
    return observation;
  }
  observation.ok = true;
  observation.specific_force_m_s2 = {
      result.measurement->linear_acceleration_x_m_s2(),
      result.measurement->linear_acceleration_y_m_s2(),
      result.measurement->linear_acceleration_z_m_s2()};
  observation.hull_z_m = poses.back().values[2];
  observation.steps =
      static_cast<int>(imu->AdapterForTesting().StepIndexForTesting());
  return observation;
}

double Norm(const std::array<double, 3>& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

}  // namespace

TEST(ImuPluginTest, RestingLinkWithGravityMeasuresOneGNotZeroNotTwoG) {
  const RestingObservation rest =
      RunPhysics(/*with_ground=*/true, "", "", /*steps=*/300);
  ASSERT_TRUE(rest.ok);
  // The box sits on the ground plane, so the support force is what the
  // accelerometer measures: f_b = (0, 0, +g).
  EXPECT_NEAR(rest.hull_z_m, 0.5, 5e-2);
  // The resting box carries a micro-tilt of the contact solver, so the
  // lateral components are tiny but not zero. The norm is exactly g.
  EXPECT_NEAR(rest.specific_force_m_s2[0], 0.0, 1e-4);
  EXPECT_NEAR(rest.specific_force_m_s2[1], 0.0, 1e-4);
  EXPECT_NEAR(rest.specific_force_m_s2[2], kGravity, 1e-4);
  EXPECT_NEAR(Norm(rest.specific_force_m_s2), kGravity, 1e-9);
  EXPECT_GT(std::fabs(rest.specific_force_m_s2[2]), 1.0);
  EXPECT_GT(std::fabs(std::fabs(rest.specific_force_m_s2[2]) - 2 * kGravity),
            1.0);
}

TEST(ImuPluginTest, RestingLinkWithHydrodynamicsDoesNotLoseOrDoubleGravity) {
  // The hydrodynamics plugin disables link gravity on the steps where it
  // applies its own wrench. Whatever the engine reports for the acceleration,
  // the specific force must not collapse to 0 or double to 2 g.
  const RestingObservation observation = RunPhysics(
      /*with_ground=*/true, kHydrodynamicsPlugin, "", /*steps=*/300);
  ASSERT_TRUE(observation.ok);
  const double magnitude = Norm(observation.specific_force_m_s2);
  EXPECT_GT(magnitude, 1.0);
  EXPECT_GT(std::fabs(magnitude - 2 * kGravity), 1.0);
  // Recorded in README.md: the measured value.
  EXPECT_NEAR(magnitude, kGravity, 1e-9);
}

TEST(ImuPluginTest, FreeFallCharacterizesTheEnginesAccelerationOutput) {
  // The default bullet-featherstone engine does not fill the link
  // acceleration, so Gazebo reports a_w = 0 even during free fall and the
  // adapter's f_b = R^T (a_w - g) stays at +g where a physical accelerometer
  // reads 0. This test records that current behaviour for numerics review. If
  // the engine starts reporting acceleration, it fails and the README and the
  // I10 expectations need a review.
  const RestingObservation fall =
      RunPhysics(/*with_ground=*/false, "", "", /*steps=*/200);
  ASSERT_TRUE(fall.ok);
  EXPECT_LT(fall.hull_z_m, 0.5 - 0.05);
  EXPECT_NEAR(Norm(fall.specific_force_m_s2), kGravity, 1e-9);
}

}  // namespace intrinsic::simulation
