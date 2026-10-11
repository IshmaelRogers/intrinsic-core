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

#include "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/dvl_policy.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.h"
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessDvl;
using ::intrinsic::hardware::marine::DvlAssessment;
using ::intrinsic::hardware::marine::DvlError;
using ::intrinsic::hardware::marine::DvlMeasurementView;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::DvlMeasurement;

constexpr double kHalfPi = std::numbers::pi / 2.0;

constexpr std::string_view kValidChildren = R"(
        <seabed_z_m>-10</seabed_z_m>
        <min_altitude_m>0.5</min_altitude_m>
        <max_altitude_m>30</max_altitude_m>)";

std::string PluginXml(std::string_view children) {
  return std::string(R"(
      <plugin filename="static://intrinsic::simulation::Dvl"
              name="intrinsic::simulation::Dvl">)") +
         std::string(children) + R"(
      </plugin>)";
}

std::string ModelWorld(std::string_view model_plugin_xml) {
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="dvl_test">
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

// Moves the model and sets the link's world linear velocity, so the sensor
// has non-trivial kinematics without a physics system.
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
    ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
        link, gz::math::Vector3d(0.0, 1.0, 0.0));
    ++step_;
  }

 private:
  int step_ = 0;
};

class DvlPartitionEnvironment : public testing::Environment {
 public:
  void SetUp() override {
    // Empty server config skips the default Physics, UserCommands, and
    // SceneBroadcaster systems. HOME avoids a write to /.gz.
    setenv("GZ_SIM_SERVER_CONFIG_PATH", "", 1);
    setenv("HOME", "/tmp", 1);
    setenv("GZ_PARTITION", "intrinsic_dvl_test", 1);
  }
};

testing::Environment* const kPartitionEnvironment =
    testing::AddGlobalTestEnvironment(new DvlPartitionEnvironment);

std::unique_ptr<gz::sim::Server> MakeServer(const std::string& sdf) {
  gz::sim::ServerConfig config;
  config.SetHeadlessRendering(true);
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
std::shared_ptr<Dvl> AddDvl(gz::sim::Server* server, std::string_view children,
                            sdf::Root* root) {
  const std::string wrapped = "<sdf version=\"1.11\"><world name=\"w\">" +
                              PluginXml(children) + "</world></sdf>";
  root->LoadSdfString(wrapped);
  const sdf::Plugins& plugins = root->WorldByIndex(0)->Plugins();
  EXPECT_FALSE(plugins.empty());
  const std::optional<gz::sim::Entity> model = server->EntityByName("uuv");
  EXPECT_TRUE(model.has_value());
  auto dvl = std::make_shared<Dvl>();
  if (plugins.empty() || !model.has_value()) {
    return dvl;
  }
  std::shared_ptr<const sdf::Element> element = plugins.front().Element();
  EXPECT_TRUE(server
                  ->AddSystem(std::static_pointer_cast<gz::sim::System>(dvl),
                              model, element)
                  .value_or(false));
  return dvl;
}

DvlAssessment Assess(const DvlMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  DvlMeasurementView view;
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
  view.mode_present = sample.has_mode();
  view.mode = sample.mode();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  view.bottom_lock_present = sample.has_bottom_lock();
  view.bottom_lock = sample.bottom_lock();
  view.altitude_present = sample.has_altitude_m();
  view.altitude_m = sample.altitude_m();
  return AssessDvl(view);
}

// One recorded plugin output per plugin step.
struct TraceEntry {
  DvlStepResult result;
  std::optional<std::string> bytes;
};

TraceEntry EntryOf(const Dvl& dvl) {
  TraceEntry entry;
  entry.result = dvl.LastResultForTesting();
  if (entry.result.measurement.has_value()) {
    entry.bytes = entry.result.measurement->SerializeAsString();
  }
  return entry;
}

// Runs single iterations until the plugin has taken `steps` new steps. An
// iteration in which the plugin did not advance by exactly one step is not
// recorded.
std::vector<TraceEntry> StepAndRecord(gz::sim::Server* server, const Dvl& dvl,
                                      int steps) {
  std::vector<TraceEntry> trace;
  for (int i = 0; i < steps + 2 && static_cast<int>(trace.size()) < steps;
       ++i) {
    const int64_t before = dvl.AdapterForTesting().StepIndexForTesting();
    EXPECT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    if (dvl.AdapterForTesting().StepIndexForTesting() == before + 1) {
      trace.push_back(EntryOf(dvl));
    }
  }
  return trace;
}

// Requests ResetAll and records the first `steps` plugin steps after the
// simulation has applied it. gz-sim applies a reset one iteration after it
// is requested: the iteration right after ResetAll is still an ordinary step
// and is not recorded. The iteration that applies the reset is the one in
// which the plugin's step index goes back down.
std::vector<TraceEntry> ResetAndRecord(gz::sim::Server* server, const Dvl& dvl,
                                       int steps) {
  server->ResetAll();
  std::vector<TraceEntry> trace;
  bool reset_applied = false;
  for (int i = 0; i < steps + 6 && static_cast<int>(trace.size()) < steps;
       ++i) {
    const int64_t before = dvl.AdapterForTesting().StepIndexForTesting();
    EXPECT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    const int64_t after = dvl.AdapterForTesting().StepIndexForTesting();
    if (!reset_applied) {
      reset_applied = after < before;
      if (reset_applied && after == 1) {
        trace.push_back(EntryOf(dvl));
      }
      continue;
    }
    if (after == before + 1) {
      trace.push_back(EntryOf(dvl));
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
                                 bool add_dvl,
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
  if (add_dvl) {
    AddDvl(server.get(), children, &root);
  }
  EXPECT_TRUE(server->Run(/*_blocking=*/true, steps, /*_paused=*/false));
  return poses;
}

}  // namespace

TEST(DvlPluginTest, ConfigureStepResetUnloadRunCleanly) {
  ASSERT_EQ(DvlStaticPluginAnchorForTesting(), 1);
  const std::string children = std::string(kValidChildren) +
                               "<latency_steps>2</latency_steps>"
                               "<noise_amplitude_m_s>0.05</noise_amplitude_m_s>"
                               "<dropout_window><start_step>4</start_step>"
                               "<end_step>6</end_step></dropout_window>";
  std::vector<TraceEntry> first_run;
  std::vector<TraceEntry> after_reset;
  {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), children, &root);
    ASSERT_TRUE(dvl->AdapterForTesting().ConfigStatusForTesting().ok());
    first_run = StepAndRecord(server.get(), *dvl, 12);
    EXPECT_EQ(dvl->AdapterForTesting().StepIndexForTesting(), 12);

    after_reset = ResetAndRecord(server.get(), *dvl, 12);
  }
  ASSERT_EQ(first_run.size(), 12u);
  for (int k = 0; k < 12; ++k) {
    EXPECT_EQ(first_run[k].result.step, k);
    EXPECT_TRUE(first_run[k].result.status.ok()) << k;
    // Latency 2: startup absent. Dropout [4, 6) shifted by the latency.
    const bool absent = k < 2 || k == 6 || k == 7;
    EXPECT_EQ(first_run[k].bytes.has_value(), !absent) << k;
    if (!absent) {
      const DvlAssessment assessment = Assess(*first_run[k].result.measurement);
      EXPECT_EQ(assessment.error, DvlError::kNone) << k;
      EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded) << k;
    }
  }
  // Reset clears the queue and the step counter: the trace equals a run
  // from a fresh server, and startup is absent again.
  ExpectTracesEqual(first_run, after_reset);
  EXPECT_FALSE(after_reset[0].bytes.has_value());
  EXPECT_FALSE(after_reset[1].bytes.has_value());

  std::vector<TraceEntry> fresh_run;
  {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), children, &root);
    fresh_run = StepAndRecord(server.get(), *dvl, 12);
  }
  ExpectTracesEqual(first_run, fresh_run);
}

TEST(DvlPluginTest, PausedStepsDoNotAdvanceTheStepIndex) {
  std::atomic<int> paused_updates{0};
  std::atomic<int> running_updates{0};
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  ASSERT_TRUE(server
                  ->AddSystem(std::make_shared<PausedCounter>(&paused_updates,
                                                              &running_updates))
                  .value_or(false));
  sdf::Root root;
  const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), kValidChildren, &root);

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
  EXPECT_EQ(dvl->AdapterForTesting().StepIndexForTesting(), 0);
  EXPECT_FALSE(dvl->LastResultForTesting().measurement.has_value());

  ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
  // The plugin's step index equals the number of non-paused updates the
  // server made, however many iterations gz-sim ran for the request.
  const int running = running_updates.load();
  ASSERT_GE(running, 1);
  EXPECT_EQ(dvl->AdapterForTesting().StepIndexForTesting(), running);
  ASSERT_TRUE(dvl->LastResultForTesting().measurement.has_value());
  EXPECT_EQ(dvl->LastResultForTesting().step, running - 1);
  EXPECT_EQ(
      dvl->LastResultForTesting().measurement->health().header().sequence(),
      42u + static_cast<uint64_t>(running - 1));
}

TEST(DvlPluginTest, ReadsLinkPoseVelocityAndSimulationTime) {
  std::vector<PoseSample> poses;
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  ASSERT_TRUE(
      server->AddSystem(std::make_shared<PoseProbe>(&poses)).value_or(false));
  ASSERT_TRUE(server->AddSystem(std::make_shared<Mover>()).value_or(false));
  sdf::Root root;
  const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), kValidChildren, &root);
  for (int k = 0; k < 6; ++k) {
    ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    ASSERT_EQ(poses.size(), static_cast<size_t>(k + 1));
    const DvlStepResult& result = dvl->LastResultForTesting();
    ASSERT_TRUE(result.status.ok()) << result.status.message;
    ASSERT_TRUE(result.measurement.has_value()) << k;
    const DvlMeasurement& sample = *result.measurement;
    // Yaw +90 degrees, world velocity (0, 1, 0): body velocity (1, 0, 0).
    EXPECT_NEAR(sample.velocity_x_m_s(), 1.0, 1e-12) << k;
    EXPECT_NEAR(sample.velocity_y_m_s(), 0.0, 1e-12) << k;
    EXPECT_NEAR(sample.velocity_z_m_s(), 0.0, 1e-12) << k;
    // Seabed at z = -10, link origin at z = 3 + 0.01 k.
    EXPECT_NEAR(sample.altitude_m(), poses[k].values[2] + 10.0, 1e-12) << k;
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    const auto& source = sample.health().header().source_time();
    EXPECT_EQ(source.seconds() * 1000000000LL + source.nanos(),
              poses[k].sim_time_ns)
        << k;
    EXPECT_EQ(sample.health().header().receive_time().seconds(),
              source.seconds());
    EXPECT_EQ(sample.health().header().receive_time().nanos(), source.nanos());
    const DvlAssessment assessment = Assess(sample);
    EXPECT_EQ(assessment.error, DvlError::kNone) << k;
    EXPECT_TRUE(assessment.accepted) << k;
  }
}

TEST(DvlPluginTest, SensorIsReadOnlyAndWorldWithoutPluginIsUnchanged) {
  constexpr int kSteps = 20;
  const std::vector<PoseSample> bare =
      RunWorld(ModelWorld(""), kSteps, /*add_dvl=*/false);
  const std::vector<PoseSample> added =
      RunWorld(ModelWorld(""), kSteps, /*add_dvl=*/true);
  const std::vector<PoseSample> from_sdf =
      RunWorld(ModelWorld(PluginXml(kValidChildren)), kSteps,
               /*add_dvl=*/false);
  ASSERT_EQ(bare.size(), static_cast<size_t>(kSteps));
  ExpectPosesBitIdentical(bare, added);
  ExpectPosesBitIdentical(bare, from_sdf);
  // The Mover really moves the link, so identical poses are not trivial.
  EXPECT_NE(bare.front().values[0], bare.back().values[0]);
}

TEST(DvlPluginTest, StaticAliasLoadsFromSdfAndUnloadsCleanly) {
  {
    const std::vector<PoseSample> poses =
        RunWorld(ModelWorld(PluginXml(kValidChildren)), 3, false);
    EXPECT_EQ(poses.size(), 3u);
  }
  const std::vector<PoseSample> again =
      RunWorld(ModelWorld(PluginXml(kValidChildren)), 2, false);
  EXPECT_EQ(again.size(), 2u);
}

TEST(DvlPluginTest, InvalidConfigurationProducesNoOutputAndNoCrash) {
  const std::string_view bad[] = {
      "<max_altitude_m>30</max_altitude_m>",
      "<seabed_z_m>-10</seabed_z_m>",
      "<seabed_z_m>nan</seabed_z_m><max_altitude_m>30</max_altitude_m>",
      "<seabed_z_m>-10</seabed_z_m><max_altitude_m>30</max_altitude_m>"
      "<mode>water_track</mode>",
      "<seabed_z_m>-10</seabed_z_m><max_altitude_m>30</max_altitude_m>"
      "<latency_steps>1001</latency_steps>",
  };
  for (const std::string_view children : bad) {
    std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
    ASSERT_TRUE(server != nullptr);
    sdf::Root root;
    const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), children, &root);
    EXPECT_FALSE(dvl->AdapterForTesting().ConfigStatusForTesting().ok())
        << children;
    for (int k = 0; k < 3; ++k) {
      ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
      EXPECT_FALSE(dvl->LastResultForTesting().measurement.has_value())
          << children;
    }
    EXPECT_EQ(dvl->AdapterForTesting().StepIndexForTesting(), 0) << children;
    server->ResetAll();
    ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    EXPECT_FALSE(dvl->LastResultForTesting().measurement.has_value())
        << children;
  }
  const std::vector<PoseSample> poses = RunWorld(
      ModelWorld(PluginXml("<mode>sideways</mode>")), 3, /*add_dvl=*/false);
  EXPECT_EQ(poses.size(), 3u);
}

TEST(DvlPluginTest, UnknownLinkIsInvalidStateWithNoSample) {
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  sdf::Root root;
  const std::string children =
      std::string(kValidChildren) + "<link_name>missing_hull</link_name>";
  const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), children, &root);
  ASSERT_TRUE(dvl->AdapterForTesting().ConfigStatusForTesting().ok());
  for (int k = 0; k < 4; ++k) {
    ASSERT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    EXPECT_EQ(dvl->LastResultForTesting().status.code,
              DvlSimErrorCode::kInvalidState);
    EXPECT_FALSE(dvl->LastResultForTesting().measurement.has_value());
    EXPECT_EQ(dvl->LastResultForTesting().step, k);
  }
}

TEST(DvlPluginTest, NamedLinkIsResolved) {
  std::unique_ptr<gz::sim::Server> server = MakeServer(ModelWorld(""));
  ASSERT_TRUE(server != nullptr);
  sdf::Root root;
  const std::string children =
      std::string(kValidChildren) + "<link_name>hull</link_name>";
  const std::shared_ptr<Dvl> dvl = AddDvl(server.get(), children, &root);
  ASSERT_TRUE(server->Run(/*_blocking=*/true, 2, /*_paused=*/false));
  ASSERT_TRUE(dvl->LastResultForTesting().measurement.has_value());
  EXPECT_TRUE(dvl->LastResultForTesting().measurement->bottom_lock());
}

}  // namespace intrinsic::simulation
