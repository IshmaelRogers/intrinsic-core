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

#include "intrinsic/simulation/gazebo/plugins/hydrodynamics/hydrodynamics_adapter.h"

#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/scene/sdf/current_field_to_sdf.h"
#include "intrinsic/vehicle/dynamics/marine_force_dynamics.h"
#include "intrinsic/vehicle/dynamics/water_current_relative_velocity.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::ComputeWaterCurrentRelativeVelocity;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsDiagnostics;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::FrameId;
using ::intrinsic::vehicle::dynamics::MarineForceDynamics;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;
using ::intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using ::intrinsic::vehicle::parameters::MarineModel;

constexpr char kGoldenPath[] =
    "intrinsic/scene/sdf/testdata/"
    "current_field_hydrodynamics_constant.golden.sdf";
constexpr double kSumTolerance = 1e-12;
constexpr std::array<double, 3> kZero3 = {0, 0, 0};
constexpr std::array<double, 6> kZero6 = {0, 0, 0, 0, 0, 0};

std::string ReadFile(const char* path) {
  std::ifstream file(path, std::ios::binary);
  EXPECT_TRUE(file.is_open()) << path;
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

std::array<double, 3> WorldToBody(const std::array<double, 4>& orientation,
                                  const std::array<double, 3>& world) {
  const embodiment::Quaternion rotation{orientation[0], orientation[1],
                                        orientation[2], orientation[3]};
  const embodiment::Quaternion conjugate{-rotation.x, -rotation.y, -rotation.z,
                                         rotation.w};
  const embodiment::Vec3 body = embodiment::RotateVector(
      conjugate, embodiment::Vec3{world[0], world[1], world[2]});
  return {body.x, body.y, body.z};
}

HydrodynamicsSimulatorState MovingState() {
  HydrodynamicsSimulatorState state;
  state.link_resolved = true;
  state.kinematics_ready = true;
  state.position_m = {1.0, -2.0, 3.5};
  state.orientation_xyzw = {0, 0, 1, 0};
  state.world_linear_velocity_m_s = {0.4, -0.2, 0.1};
  state.world_angular_velocity_rad_s = {0.01, -0.02, 0.03};
  state.dt_s = 0.001;
  return state;
}

HydrodynamicsSimulatorState IdentityState() {
  HydrodynamicsSimulatorState state;
  state.link_resolved = true;
  state.kinematics_ready = true;
  state.dt_s = 0.001;
  return state;
}

void ExpectZeroWrench(const HydrodynamicsStepResult& result) {
  EXPECT_FALSE(result.dynamics_evaluated);
  EXPECT_FALSE(AppliesWrench(result));
  EXPECT_EQ(result.body_force_n, kZero3);
  EXPECT_EQ(result.body_torque_n_m, kZero3);
  EXPECT_EQ(result.world_force_n, kZero3);
  EXPECT_EQ(result.world_torque_n_m, kZero3);
  EXPECT_TRUE(std::isfinite(result.body_force_n[0]));
  EXPECT_TRUE(std::isfinite(result.world_torque_n_m[2]));
}

void ExpectDiagnosticsEqual(const DynamicsDiagnostics& lhs,
                            const DynamicsDiagnostics& rhs) {
  EXPECT_EQ(lhs.model_id, rhs.model_id);
  EXPECT_EQ(lhs.dt_s, rhs.dt_s);
  EXPECT_EQ(lhs.model_force_n, rhs.model_force_n);
  EXPECT_EQ(lhs.model_torque_n_m, rhs.model_torque_n_m);
  EXPECT_EQ(lhs.input_wrench_used, rhs.input_wrench_used);
  EXPECT_EQ(lhs.allocation_invoked, rhs.allocation_invoked);
  EXPECT_EQ(lhs.relative_twist, rhs.relative_twist);
  EXPECT_EQ(lhs.rigid_body_coriolis_wrench, rhs.rigid_body_coriolis_wrench);
  EXPECT_EQ(lhs.added_mass_coriolis_wrench, rhs.added_mass_coriolis_wrench);
  EXPECT_EQ(lhs.damping_wrench, rhs.damping_wrench);
  EXPECT_EQ(lhs.restoring_wrench, rhs.restoring_wrench);
  EXPECT_EQ(lhs.hydrodynamic_wrench, rhs.hydrodynamic_wrench);
  EXPECT_EQ(lhs.total_wrench, rhs.total_wrench);
}

TEST(HydrodynamicsAdapterTest, AbsentCurrentFieldAppliesZeroWrench) {
  HydrodynamicsAdapter adapter;
  const HydrodynamicsStatus status = adapter.ConfigureFromPluginXml(
      "<plugin filename=\"static://intrinsic::simulation::Hydrodynamics\" "
      "name=\"intrinsic::simulation::Hydrodynamics\"></plugin>");
  EXPECT_TRUE(status.ok());
  EXPECT_FALSE(adapter.active());
  EXPECT_EQ(status.code, HydrodynamicsErrorCode::kOk);

  HydrodynamicsSimulatorState state = MovingState();
  state.dt_s = 0;
  const HydrodynamicsStepResult result = adapter.Step(state);
  EXPECT_EQ(result.status.code, HydrodynamicsErrorCode::kAbsent);
  EXPECT_FALSE(result.dynamics_called);
  ExpectZeroWrench(result);
}

TEST(HydrodynamicsAdapterTest, ResetRestoresInitialState) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const HydrodynamicsStepResult initial = adapter.last_result();
  EXPECT_EQ(initial.status.code, HydrodynamicsErrorCode::kOk);
  EXPECT_FALSE(initial.dynamics_evaluated);
  ExpectZeroWrench(initial);

  const HydrodynamicsStepResult stepped = adapter.Step(MovingState());
  ASSERT_TRUE(stepped.dynamics_evaluated);
  EXPECT_NE(stepped.body_force_n, kZero3);

  adapter.Reset();
  const HydrodynamicsStepResult restored = adapter.last_result();
  EXPECT_EQ(restored.status.code, initial.status.code);
  EXPECT_EQ(restored.status.message, initial.status.message);
  EXPECT_EQ(restored.dynamics_evaluated, initial.dynamics_evaluated);
  EXPECT_EQ(restored.dynamics_called, initial.dynamics_called);
  ExpectZeroWrench(restored);
  ExpectDiagnosticsEqual(restored.diagnostics, initial.diagnostics);
}

TEST(HydrodynamicsAdapterTest, SharedModelParity) {
  const MarineModel model = MakeSixThrusterUuvExample();
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const HydrodynamicsSimulatorState state = MovingState();
  const HydrodynamicsStepResult result = adapter.Step(state);
  ASSERT_TRUE(result.dynamics_evaluated) << result.status.message;

  const std::array<double, 3> linear =
      WorldToBody(state.orientation_xyzw, state.world_linear_velocity_m_s);
  const std::array<double, 3> angular =
      WorldToBody(state.orientation_xyzw, state.world_angular_velocity_rad_s);
  VehicleStateRt expected_state;
  expected_state.pose_frame = FrameId::kWorldEnu;
  expected_state.position_m = state.position_m;
  expected_state.orientation_xyzw = state.orientation_xyzw;
  expected_state.body_twist = {linear[0],  linear[1],  linear[2],
                               angular[0], angular[1], angular[2]};
  BodyWrenchRt input;
  input.frame = FrameId::kBody;
  EnvironmentRt environment;
  environment.gravity_m_s2 = model.environment.gravity_m_s2;
  environment.fluid_density_kg_m3 = model.environment.fluid_density_kg_m3;
  environment.current_velocity_m_s = {0.2, -0.1, 0.0};
  environment.current_frame = FrameId::kWorldEnu;

  EXPECT_EQ(result.dynamics_state.pose_frame, expected_state.pose_frame);
  EXPECT_EQ(result.dynamics_state.position_m, expected_state.position_m);
  EXPECT_EQ(result.dynamics_state.orientation_xyzw,
            expected_state.orientation_xyzw);
  EXPECT_EQ(result.dynamics_state.body_twist, expected_state.body_twist);
  EXPECT_EQ(result.dynamics_input_wrench.frame, input.frame);
  EXPECT_EQ(result.dynamics_input_wrench.force_n, input.force_n);
  EXPECT_EQ(result.dynamics_input_wrench.torque_n_m, input.torque_n_m);
  EXPECT_EQ(result.dynamics_environment.gravity_m_s2, environment.gravity_m_s2);
  EXPECT_EQ(result.dynamics_environment.fluid_density_kg_m3,
            environment.fluid_density_kg_m3);
  EXPECT_EQ(result.dynamics_environment.current_velocity_m_s,
            environment.current_velocity_m_s);
  EXPECT_EQ(result.dynamics_environment.current_frame,
            environment.current_frame);
  EXPECT_EQ(result.dynamics_dt.seconds, state.dt_s);

  const MarineForceDynamics direct(MakeSixThrusterUuvExample());
  const auto evaluated =
      direct.Evaluate(expected_state, input, environment, Duration{state.dt_s});
  ASSERT_TRUE(evaluated.ok()) << evaluated.status().message;
  ExpectDiagnosticsEqual(result.diagnostics, evaluated.value().diagnostics);
  EXPECT_EQ(result.body_force_n, evaluated.value().diagnostics.model_force_n);
  EXPECT_EQ(result.body_torque_n_m,
            evaluated.value().diagnostics.model_torque_n_m);
  EXPECT_EQ(result.diagnostics.model_id,
            ::intrinsic::vehicle::dynamics::kMarineForceModelId);
  EXPECT_TRUE(result.diagnostics.input_wrench_used);
  EXPECT_FALSE(result.diagnostics.allocation_invoked);
}

TEST(HydrodynamicsAdapterTest, DiagnosticsExposeEveryTermAndMatchWrench) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const HydrodynamicsStepResult result = adapter.Step(MovingState());
  ASSERT_TRUE(result.dynamics_evaluated) << result.status.message;
  const DynamicsDiagnostics& diagnostics = result.diagnostics;
  EXPECT_EQ(diagnostics.model_id,
            ::intrinsic::vehicle::dynamics::kMarineForceModelId);
  EXPECT_EQ(diagnostics.dt_s, 0.001);
  EXPECT_TRUE(diagnostics.input_wrench_used);
  EXPECT_FALSE(diagnostics.allocation_invoked);
  for (int index = 0; index < 6; ++index) {
    const double sum = diagnostics.rigid_body_coriolis_wrench[index] +
                       diagnostics.added_mass_coriolis_wrench[index] +
                       diagnostics.damping_wrench[index] +
                       diagnostics.restoring_wrench[index];
    EXPECT_NEAR(sum, diagnostics.hydrodynamic_wrench[index], kSumTolerance);
    EXPECT_TRUE(std::isfinite(diagnostics.total_wrench[index]));
  }
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[0], result.body_force_n[0],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[1], result.body_force_n[1],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[2], result.body_force_n[2],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[3], result.body_torque_n_m[0],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[4], result.body_torque_n_m[1],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[5], result.body_torque_n_m[2],
              kSumTolerance);
  EXPECT_EQ(diagnostics.model_force_n, result.body_force_n);
  EXPECT_EQ(diagnostics.model_torque_n_m, result.body_torque_n_m);
  EXPECT_NEAR(diagnostics.total_wrench[0], result.body_force_n[0],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.total_wrench[1], result.body_force_n[1],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.total_wrench[2], result.body_force_n[2],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.total_wrench[3], result.body_torque_n_m[0],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.total_wrench[4], result.body_torque_n_m[1],
              kSumTolerance);
  EXPECT_NEAR(diagnostics.total_wrench[5], result.body_torque_n_m[2],
              kSumTolerance);
  EXPECT_NE(diagnostics.relative_twist, kZero6);
  EXPECT_NE(diagnostics.restoring_wrench, kZero6);
}

TEST(HydrodynamicsAdapterTest, ConstantCurrentGoldenRelativeVelocity) {
  HydrodynamicsAdapter adapter;
  const std::string golden = ReadFile(kGoldenPath);
  EXPECT_NE(golden.find("static://intrinsic::simulation::Hydrodynamics"),
            std::string::npos);
  EXPECT_NE(golden.find("<velocity_m_s>0.2 -0.1 0</velocity_m_s>"),
            std::string::npos);
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(golden).ok());
  const HydrodynamicsStepResult result = adapter.Step(IdentityState());
  ASSERT_TRUE(result.dynamics_evaluated) << result.status.message;
  EXPECT_EQ(result.dynamics_environment.current_frame, FrameId::kWorldEnu);
  EXPECT_EQ(result.dynamics_environment.current_velocity_m_s,
            (std::array<double, 3>{0.2, -0.1, 0.0}));

  ::intrinsic::vehicle::parameters::Environment environment;
  environment.current_velocity_m_s = {0.2, -0.1, 0.0};
  environment.current_frame_id = "world_enu";
  const auto relative = ComputeWaterCurrentRelativeVelocity(
      environment, FrameId::kWorldEnu, result.dynamics_state.orientation_xyzw,
      result.dynamics_state.body_twist);
  ASSERT_TRUE(relative.ok()) << relative.status().message;
  EXPECT_EQ(result.diagnostics.relative_twist, relative.value().components);
  EXPECT_EQ(result.dynamics_state.body_twist, kZero6);
}

TEST(HydrodynamicsAdapterTest, WorldNedCurrentUsesEmbodimentConversion) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(
      adapter
          .ConfigureFromPluginXml(
              "<plugin "
              "filename=\"static://intrinsic::simulation::Hydrodynamics\" "
              "name=\"intrinsic::simulation::Hydrodynamics\">"
              "<current_field><frame_id>world_ned</frame_id>"
              "<velocity_m_s>0.2 -0.1 0.05</velocity_m_s></current_field>"
              "</plugin>")
          .ok());
  const HydrodynamicsStepResult result = adapter.Step(IdentityState());
  ASSERT_TRUE(result.dynamics_evaluated) << result.status.message;
  const embodiment::Vec3 enu =
      embodiment::WorldVectorNedToEnu(embodiment::Vec3{0.2, -0.1, 0.05});
  EXPECT_EQ(result.dynamics_environment.current_frame, FrameId::kWorldEnu);
  EXPECT_EQ(result.dynamics_environment.current_velocity_m_s,
            (std::array<double, 3>{enu.x, enu.y, enu.z}));
}

TEST(HydrodynamicsAdapterTest, BodyCurrentPassesThrough) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(
      adapter
          .ConfigureFromPluginXml(
              "<plugin "
              "filename=\"static://intrinsic::simulation::Hydrodynamics\" "
              "name=\"intrinsic::simulation::Hydrodynamics\">"
              "<current_field><frame_id>body</frame_id>"
              "<velocity_m_s>0.2 -0.1 0</velocity_m_s></current_field>"
              "</plugin>")
          .ok());
  const HydrodynamicsStepResult result = adapter.Step(IdentityState());
  ASSERT_TRUE(result.dynamics_evaluated) << result.status.message;
  EXPECT_EQ(result.dynamics_environment.current_frame, FrameId::kBody);
  EXPECT_EQ(result.dynamics_environment.current_velocity_m_s,
            (std::array<double, 3>{0.2, -0.1, 0.0}));
}

TEST(HydrodynamicsAdapterTest, InvalidFrameAppliesZeroWrench) {
  HydrodynamicsAdapter adapter;
  const HydrodynamicsStatus status = adapter.ConfigureFromPluginXml(
      "<plugin filename=\"static://intrinsic::simulation::Hydrodynamics\" "
      "name=\"intrinsic::simulation::Hydrodynamics\">"
      "<current_field><frame_id>robot</frame_id>"
      "<velocity_m_s>nan -0.1 0</velocity_m_s></current_field>"
      "</plugin>");
  EXPECT_EQ(status.code, HydrodynamicsErrorCode::kInvalidConfig);
  EXPECT_EQ(status.field_error, world::CurrentFieldError::kFrameId);
  EXPECT_FALSE(adapter.active());
  const HydrodynamicsStepResult result = adapter.Step(MovingState());
  EXPECT_EQ(result.status.code, HydrodynamicsErrorCode::kInvalidConfig);
  EXPECT_EQ(result.status.field_error, world::CurrentFieldError::kFrameId);
  EXPECT_FALSE(result.dynamics_called);
  ExpectZeroWrench(result);
}

TEST(HydrodynamicsAdapterTest, NonFiniteVelocityAppliesZeroWrench) {
  for (const char* velocity : {"nan -0.1 0", "0.2 inf 0", "0.2 -0.1"}) {
    HydrodynamicsAdapter adapter;
    const HydrodynamicsStatus status = adapter.ConfigureFromPluginXml(
        std::string(
            "<plugin "
            "filename=\"static://intrinsic::simulation::Hydrodynamics\" "
            "name=\"intrinsic::simulation::Hydrodynamics\">"
            "<current_field><frame_id>world_enu</frame_id>"
            "<velocity_m_s>") +
        velocity + "</velocity_m_s></current_field></plugin>");
    EXPECT_EQ(status.code, HydrodynamicsErrorCode::kInvalidConfig) << velocity;
    EXPECT_EQ(status.field_error, world::CurrentFieldError::kVelocity)
        << velocity;
    const HydrodynamicsStepResult result = adapter.Step(MovingState());
    EXPECT_FALSE(result.dynamics_called) << velocity;
    ExpectZeroWrench(result);
  }
}

TEST(HydrodynamicsAdapterTest, MissingLinkAppliesZeroWrench) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  HydrodynamicsSimulatorState state = MovingState();
  state.link_resolved = false;
  const HydrodynamicsStepResult result = adapter.Step(state);
  EXPECT_EQ(result.status.code, HydrodynamicsErrorCode::kInvalidState);
  EXPECT_FALSE(result.dynamics_called);
  ExpectZeroWrench(result);

  state.link_resolved = true;
  state.kinematics_ready = false;
  const HydrodynamicsStepResult unavailable = adapter.Step(state);
  EXPECT_EQ(unavailable.status.code, HydrodynamicsErrorCode::kInvalidState);
  ExpectZeroWrench(unavailable);
}

TEST(HydrodynamicsAdapterTest, InvalidTimestepAppliesZeroWrench) {
  HydrodynamicsAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const double samples[] = {0.0, -0.001,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()};
  for (double dt : samples) {
    HydrodynamicsSimulatorState state = IdentityState();
    state.dt_s = dt;
    const HydrodynamicsStepResult result = adapter.Step(state);
    EXPECT_EQ(result.status.code, HydrodynamicsErrorCode::kInvalidState) << dt;
    EXPECT_FALSE(result.dynamics_called) << dt;
    ExpectZeroWrench(result);
  }
}

TEST(HydrodynamicsAdapterTest, ExtremeBoundedParametersApplyZeroWrench) {
  MarineModel zero_mass = MakeSixThrusterUuvExample();
  zero_mass.mass_inertia.mass_kg = 0;
  HydrodynamicsAdapter mass_adapter(std::move(zero_mass));
  ASSERT_TRUE(mass_adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const HydrodynamicsStepResult mass_result =
      mass_adapter.Step(IdentityState());
  EXPECT_EQ(mass_result.status.code, HydrodynamicsErrorCode::kDynamics);
  EXPECT_TRUE(mass_result.dynamics_called);
  ExpectZeroWrench(mass_result);
  EXPECT_TRUE(std::isfinite(mass_result.diagnostics.total_wrench[0]));

  MarineModel zero_gravity = MakeSixThrusterUuvExample();
  zero_gravity.environment.gravity_m_s2 = 0;
  HydrodynamicsAdapter gravity_adapter(std::move(zero_gravity));
  ASSERT_TRUE(
      gravity_adapter.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  const HydrodynamicsStepResult gravity_result =
      gravity_adapter.Step(IdentityState());
  EXPECT_EQ(gravity_result.status.code, HydrodynamicsErrorCode::kDynamics);
  ExpectZeroWrench(gravity_result);

  HydrodynamicsAdapter huge;
  ASSERT_TRUE(huge.ConfigureFromPluginXml(ReadFile(kGoldenPath)).ok());
  HydrodynamicsSimulatorState state = IdentityState();
  state.world_linear_velocity_m_s = {1e300, 0, 0};
  const HydrodynamicsStepResult huge_result = huge.Step(state);
  EXPECT_EQ(huge_result.status.code, HydrodynamicsErrorCode::kDynamics);
  ExpectZeroWrench(huge_result);
  for (double component : huge_result.diagnostics.total_wrench) {
    EXPECT_TRUE(std::isfinite(component));
  }
}

TEST(HydrodynamicsAdapterTest, RepeatedEvaluationIsIdentical) {
  const std::string golden = ReadFile(kGoldenPath);
  HydrodynamicsAdapter first;
  HydrodynamicsAdapter second;
  ASSERT_TRUE(first.ConfigureFromPluginXml(golden).ok());
  ASSERT_TRUE(second.ConfigureFromPluginXml(golden).ok());
  const HydrodynamicsSimulatorState state = MovingState();
  const HydrodynamicsStepResult once = first.Step(state);
  const HydrodynamicsStepResult twice = first.Step(state);
  const HydrodynamicsStepResult other = second.Step(state);
  ASSERT_TRUE(once.dynamics_evaluated);
  ExpectDiagnosticsEqual(once.diagnostics, twice.diagnostics);
  ExpectDiagnosticsEqual(once.diagnostics, other.diagnostics);
  EXPECT_EQ(once.body_force_n, twice.body_force_n);
  EXPECT_EQ(once.world_torque_n_m, other.world_torque_n_m);
  EXPECT_EQ(once.diagnostics.relative_twist, other.diagnostics.relative_twist);
}

TEST(HydrodynamicsAdapterTest, SourcesOmitSensorsRandomizationAndBathymetry) {
  const char* paths[] = {
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_adapter.h",
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_adapter.cc",
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_plugin.h",
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_plugin.cc",
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_plugin_register.cc",
  };
  const char* forbidden[] = {
      "bathymetry",
      "mt19937",
      "random_device",
      "uniform_real_distribution",
      "absl/random",
      "CameraPlugin",
      "RgbdCamera",
      "displaced_volume",
      "quadratic_coefficients",
      "C_RB",
      "C_A(",
  };
  for (const char* path : paths) {
    const std::string source = ReadFile(path);
    ASSERT_FALSE(source.empty()) << path;
    for (const char* needle : forbidden) {
      EXPECT_EQ(source.find(needle), std::string::npos)
          << path << " contains " << needle;
    }
  }
  const std::string adapter = ReadFile(
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_adapter.cc");
  EXPECT_EQ(adapter.find("gz/sim/"), std::string::npos);
  const std::string registration = ReadFile(
      "intrinsic/simulation/gazebo/plugins/hydrodynamics/"
      "hydrodynamics_plugin_register.cc");
  EXPECT_NE(registration.find("\"intrinsic::simulation::Hydrodynamics\""),
            std::string::npos);
  EXPECT_NE(registration.find("GZ_ADD_STATIC_PLUGIN_ALIAS"), std::string::npos);
  EXPECT_EQ(sdf::kHydrodynamicsPluginName,
            "intrinsic::simulation::Hydrodynamics");
  EXPECT_EQ(sdf::kHydrodynamicsPluginFilename,
            "static://intrinsic::simulation::Hydrodynamics");
}

}  // namespace
}  // namespace intrinsic::simulation
