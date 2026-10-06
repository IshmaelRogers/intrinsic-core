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

#include "intrinsic/simulation/gazebo/plugins/hydrodynamics/hydrodynamics_plugin.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "intrinsic/simulation/gazebo/plugins/priority_constants.h"

namespace intrinsic::simulation {
namespace {

constexpr std::string_view kActivePlugin = R"(
      <plugin filename="static://intrinsic::simulation::Hydrodynamics"
              name="intrinsic::simulation::Hydrodynamics">
        <current_field>
          <frame_id>world_enu</frame_id>
          <velocity_m_s>0.2 -0.1 0</velocity_m_s>
        </current_field>
      </plugin>)";

constexpr std::string_view kInactivePlugin = R"(
      <plugin filename="static://intrinsic::simulation::Hydrodynamics"
              name="intrinsic::simulation::Hydrodynamics">
      </plugin>)";

constexpr std::string_view kUnknownLinkPlugin = R"(
      <plugin filename="static://intrinsic::simulation::Hydrodynamics"
              name="intrinsic::simulation::Hydrodynamics">
        <link_name>missing_hull</link_name>
        <current_field>
          <frame_id>world_enu</frame_id>
          <velocity_m_s>0.2 -0.1 0</velocity_m_s>
        </current_field>
      </plugin>)";

struct StepObservation {
  bool link_found = false;
  bool wrench_present = false;
  bool wrench_nonzero = false;
  bool wrench_finite = true;
  bool gravity_disabled = false;
  gz::math::Pose3d pose;
};

struct RunObservation {
  bool server_ok = false;
  std::vector<StepObservation> steps;
  std::vector<StepObservation> steps_after_reset;
};

class WrenchProbe final : public gz::sim::System,
                          public gz::sim::ISystemConfigurePriority,
                          public gz::sim::ISystemPreUpdate {
 public:
  WrenchProbe(std::vector<StepObservation>* steps, bool* record)
      : steps_(steps), record_(record) {}

  gz::sim::System::PriorityType ConfigurePriority() override {
    return plugins::kHardwareModuleLauncherPriority + 1;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (info.paused || record_ == nullptr || !*record_ || steps_ == nullptr) {
      return;
    }
    StepObservation step;
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    step.link_found = link != gz::sim::kNullEntity;
    if (step.link_found) {
      step.pose = gz::sim::worldPose(link, ecm);
      const auto* wrench =
          ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(link);
      if (wrench != nullptr) {
        step.wrench_present = true;
        const double values[] = {
            wrench->Data().force().x(),  wrench->Data().force().y(),
            wrench->Data().force().z(),  wrench->Data().torque().x(),
            wrench->Data().torque().y(), wrench->Data().torque().z(),
        };
        for (double value : values) {
          if (!std::isfinite(value)) {
            step.wrench_finite = false;
          }
          if (value != 0.0) {
            step.wrench_nonzero = true;
          }
        }
      }
      const auto* gravity =
          ecm.Component<gz::sim::components::GravityEnabledCmd>(link);
      step.gravity_disabled = gravity != nullptr && !gravity->Data();
    }
    steps_->push_back(step);
  }

 private:
  std::vector<StepObservation>* steps_ = nullptr;
  bool* record_ = nullptr;
};

std::string ModelWorld(std::string_view plugin_xml) {
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="hydrodynamics_test">
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
)") + std::string(plugin_xml) +
         R"(
    </model>
  </world>
</sdf>
)";
}

std::string WorldScopedPluginWorld() {
  return std::string(R"(<?xml version="1.0"?>
<sdf version="1.11">
  <world name="hydrodynamics_world_scope">
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <plugin filename="static://intrinsic::simulation::Hydrodynamics"
            name="intrinsic::simulation::Hydrodynamics">
      <current_field>
        <frame_id>world_enu</frame_id>
        <velocity_m_s>0.2 -0.1 0</velocity_m_s>
      </current_field>
    </plugin>
    <model name="uuv">
      <pose>1 2 3 0 0 0</pose>
      <link name="hull">
        <inertial>
          <mass>32</mass>
          <inertia>
            <ixx>0.5</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>2.4</iyy><iyz>0</iyz><izz>2.7</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
)");
}

RunObservation RunHeadless(const std::string& sdf, int steps, bool reset) {
  RunObservation observation;
  bool record = true;
  auto probe = std::make_shared<WrenchProbe>(&observation.steps, &record);
  gz::sim::ServerConfig config;
  config.SetHeadlessRendering(true);
  if (!config.SetSdfString(sdf)) {
    return observation;
  }
  gz::sim::Server server(config);
  server.SetUpdatePeriod(std::chrono::steady_clock::duration::zero());
  if (!server.AddSystem(probe).value_or(false)) {
    return observation;
  }
  observation.server_ok =
      server.Run(/*_blocking=*/true, steps, /*_paused=*/false);
  if (reset && observation.server_ok) {
    const std::vector<StepObservation> before = observation.steps;
    observation.steps.clear();
    server.ResetAll();
    observation.server_ok =
        server.Run(/*_blocking=*/true, steps, /*_paused=*/false);
    observation.steps_after_reset = observation.steps;
    observation.steps = before;
  }
  return observation;
}

bool SawWrench(const std::vector<StepObservation>& steps) {
  for (const StepObservation& step : steps) {
    if (step.wrench_present) {
      return true;
    }
  }
  return false;
}

bool SawFiniteNonZeroWrench(const std::vector<StepObservation>& steps) {
  for (const StepObservation& step : steps) {
    if (step.wrench_present && step.wrench_nonzero && step.wrench_finite) {
      return true;
    }
  }
  return false;
}

class HydrodynamicsPartitionEnvironment : public testing::Environment {
 public:
  void SetUp() override {
    // Empty server config skips the default Physics, UserCommands, and
    // SceneBroadcaster systems. This test only needs the hydrodynamics
    // plugin and the wrench probe. HOME avoids a write to /.gz.
    setenv("GZ_SIM_SERVER_CONFIG_PATH", "", 1);
    setenv("HOME", "/tmp", 1);
    setenv("GZ_PARTITION", "intrinsic_hydrodynamics_test", 1);
  }
};

testing::Environment* const kPartitionEnvironment =
    testing::AddGlobalTestEnvironment(new HydrodynamicsPartitionEnvironment);

void ExpectPosesEqual(const std::vector<StepObservation>& lhs,
                      const std::vector<StepObservation>& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (size_t index = 0; index < lhs.size(); ++index) {
    EXPECT_TRUE(lhs[index].link_found);
    EXPECT_TRUE(rhs[index].link_found);
    EXPECT_EQ(lhs[index].pose, rhs[index].pose) << index;
  }
}

}  // namespace

TEST(HydrodynamicsPluginTest,
     WorldWithoutPluginMatchesPluginWithoutCurrentField) {
  ASSERT_EQ(HydrodynamicsStaticPluginAnchor(), 1);
  const RunObservation bare = RunHeadless(ModelWorld(""), 4, false);
  const RunObservation inactive =
      RunHeadless(ModelWorld(kInactivePlugin), 4, false);
  ASSERT_TRUE(bare.server_ok);
  ASSERT_TRUE(inactive.server_ok);
  ASSERT_FALSE(bare.steps.empty());
  EXPECT_FALSE(SawWrench(bare.steps));
  EXPECT_FALSE(SawWrench(inactive.steps));
  ExpectPosesEqual(bare.steps, inactive.steps);
  for (const StepObservation& step : inactive.steps) {
    EXPECT_FALSE(step.gravity_disabled);
  }
}

TEST(HydrodynamicsPluginTest, LifecycleConfigureStepResetUnload) {
  {
    const RunObservation first =
        RunHeadless(ModelWorld(kActivePlugin), 4, true);
    ASSERT_TRUE(first.server_ok);
    ASSERT_FALSE(first.steps.empty());
    ASSERT_FALSE(first.steps_after_reset.empty());
    EXPECT_TRUE(SawFiniteNonZeroWrench(first.steps));
    EXPECT_TRUE(SawFiniteNonZeroWrench(first.steps_after_reset));
    for (const StepObservation& step : first.steps) {
      EXPECT_TRUE(step.wrench_finite);
    }
    for (const StepObservation& step : first.steps_after_reset) {
      EXPECT_TRUE(step.wrench_finite);
    }
  }
  const RunObservation reloaded =
      RunHeadless(ModelWorld(kActivePlugin), 2, false);
  ASSERT_TRUE(reloaded.server_ok);
  EXPECT_TRUE(SawFiniteNonZeroWrench(reloaded.steps));
}

TEST(HydrodynamicsPluginTest, UnknownLinkAppliesZeroWrench) {
  const RunObservation unknown =
      RunHeadless(ModelWorld(kUnknownLinkPlugin), 3, false);
  ASSERT_TRUE(unknown.server_ok);
  EXPECT_FALSE(SawWrench(unknown.steps));
  for (const StepObservation& step : unknown.steps) {
    EXPECT_TRUE(step.link_found);
    EXPECT_FALSE(step.gravity_disabled);
  }

  const RunObservation world_scope =
      RunHeadless(WorldScopedPluginWorld(), 3, false);
  ASSERT_TRUE(world_scope.server_ok);
  EXPECT_FALSE(SawWrench(world_scope.steps));
}

}  // namespace intrinsic::simulation
