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

#include "intrinsic/vehicle/dynamics/marine_force_dynamics.h"

#include <array>
#include <cmath>
#include <limits>
#include <string_view>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/added_mass_coriolis_matrix.h"
#include "intrinsic/vehicle/dynamics/gravity_buoyancy_restoring_wrench.h"
#include "intrinsic/vehicle/dynamics/linear_quadratic_damping_wrench.h"
#include "intrinsic/vehicle/dynamics/rigid_body_coriolis_matrix.h"
#include "intrinsic/vehicle/dynamics/water_current_relative_velocity.h"

namespace {

using intrinsic::vehicle::dynamics::AddedMassCoriolisMatrix;
using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::ComputeAddedMassCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeGravityBuoyancyRestoringWrench;
using intrinsic::vehicle::dynamics::ComputeLinearQuadraticDampingWrench;
using intrinsic::vehicle::dynamics::ComputeRigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeWaterCurrentRelativeVelocity;
using intrinsic::vehicle::dynamics::DampingWrench;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::DynamicsResult;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::FrameId;
using intrinsic::vehicle::dynamics::kHeave;
using intrinsic::vehicle::dynamics::kMarineForceModelId;
using intrinsic::vehicle::dynamics::kPitch;
using intrinsic::vehicle::dynamics::kRoll;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kSurge;
using intrinsic::vehicle::dynamics::kSway;
using intrinsic::vehicle::dynamics::kYaw;
using intrinsic::vehicle::dynamics::MarineForceDynamics;
using intrinsic::vehicle::dynamics::RestoringWrench;
using intrinsic::vehicle::dynamics::RigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::parameters::MarineModel;

constexpr double kSumTolerance = 1e-12;
constexpr std::array<double, 6> kZero6 = {0, 0, 0, 0, 0, 0};

std::array<double, kSpatialDof * kSpatialDof> Diagonal(
    double surge, double sway, double heave, double roll, double pitch,
    double yaw) {
  std::array<double, kSpatialDof * kSpatialDof> matrix = {};
  const double diagonal[] = {surge, sway, heave, roll, pitch, yaw};
  for (int index = 0; index < kSpatialDof; ++index) {
    matrix[index * kSpatialDof + index] = diagonal[index];
  }
  return matrix;
}

MarineModel MakeModel(double mass_kg, double volume_m3) {
  MarineModel model;
  model.model_id = "composition_fixture";
  model.mass_inertia.mass_kg = mass_kg;
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2 = {
      1, 0, 0, 0, 2, 0, 0, 0, 3,
  };
  model.buoyancy.displaced_volume_m3 = volume_m3;
  model.added_mass.coefficients = Diagonal(1, 1, 1, 1, 1, 1);
  model.damping.linear_coefficients = Diagonal(2, 2, 2, 2, 2, 2);
  // Stored environment differs from the evaluation snapshot. Evaluate must
  // read the snapshot.
  model.environment.gravity_m_s2 = 1;
  model.environment.fluid_density_kg_m3 = 1;
  model.environment.current_velocity_m_s = {9, 9, 9};
  model.environment.current_frame_id = "body";
  return model;
}

VehicleStateRt IdentityState(FrameId frame) {
  VehicleStateRt state;
  state.pose_frame = frame;
  state.orientation_xyzw = {0, 0, 0, 1};
  return state;
}

EnvironmentRt Snapshot(FrameId current_frame, double surge_current) {
  EnvironmentRt environment;
  environment.gravity_m_s2 = 10;
  environment.fluid_density_kg_m3 = 2;
  environment.current_velocity_m_s = {surge_current, 0, 0};
  environment.current_frame = current_frame;
  return environment;
}

BodyWrenchRt SampleWrench() {
  BodyWrenchRt wrench;
  wrench.force_n = {3, -1, 4};
  wrench.torque_n_m = {0.5, 0, -2};
  return wrench;
}

intrinsic::vehicle::parameters::Environment HelperEnvironment(
    const EnvironmentRt& environment) {
  intrinsic::vehicle::parameters::Environment mapped;
  mapped.gravity_m_s2 = environment.gravity_m_s2;
  mapped.fluid_density_kg_m3 = environment.fluid_density_kg_m3;
  mapped.current_velocity_m_s = {
      environment.current_velocity_m_s[0],
      environment.current_velocity_m_s[1],
      environment.current_velocity_m_s[2],
  };
  if (environment.current_frame == FrameId::kWorldEnu) {
    mapped.current_frame_id = "world_enu";
  } else if (environment.current_frame == FrameId::kWorldNed) {
    mapped.current_frame_id = "world_ned";
  } else {
    mapped.current_frame_id = "body";
  }
  return mapped;
}

std::array<double, kSpatialDof> Multiply(
    const std::array<double, kSpatialDof * kSpatialDof>& matrix,
    const std::array<double, kSpatialDof>& vector) {
  std::array<double, kSpatialDof> product = {};
  for (int row = 0; row < kSpatialDof; ++row) {
    double sum = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      sum += matrix[row * kSpatialDof + col] * vector[col];
    }
    product[row] = sum;
  }
  return product;
}

void ExpectFiniteZero(const DynamicsResult& result) {
  EXPECT_EQ(result.diagnostics.model_id, kMarineForceModelId);
  EXPECT_EQ(result.diagnostics.dt_s, 0);
  EXPECT_FALSE(result.diagnostics.input_wrench_used);
  EXPECT_FALSE(result.diagnostics.allocation_invoked);
  EXPECT_EQ(result.diagnostics.model_force_n, (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.diagnostics.model_torque_n_m,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.derivative.position_dot_m_s,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.derivative.orientation_dot_xyzw,
            (std::array<double, 4>{0, 0, 0, 0}));
  EXPECT_EQ(result.derivative.body_acceleration, kZero6);
  EXPECT_EQ(result.diagnostics.relative_twist, kZero6);
  EXPECT_EQ(result.diagnostics.rigid_body_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.added_mass_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.damping_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.restoring_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.hydrodynamic_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.total_wrench, kZero6);
  for (double value : result.derivative.body_acceleration) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (double value : result.diagnostics.total_wrench) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

void ExpectSumEqualsTotal(const DynamicsResult& result) {
  const auto& diagnostics = result.diagnostics;
  for (int index = 0; index < kSpatialDof; ++index) {
    const double hydro = diagnostics.rigid_body_coriolis_wrench[index] +
                         diagnostics.added_mass_coriolis_wrench[index] +
                         diagnostics.damping_wrench[index] +
                         diagnostics.restoring_wrench[index];
    EXPECT_NEAR(hydro, diagnostics.hydrodynamic_wrench[index], kSumTolerance);
  }
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kSurge],
              diagnostics.model_force_n[0], kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kSway],
              diagnostics.model_force_n[1], kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kHeave],
              diagnostics.model_force_n[2], kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kRoll],
              diagnostics.model_torque_n_m[0], kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kPitch],
              diagnostics.model_torque_n_m[1], kSumTolerance);
  EXPECT_NEAR(diagnostics.hydrodynamic_wrench[kYaw],
              diagnostics.model_torque_n_m[2], kSumTolerance);
}

TEST(MarineForceDynamics, TypesStayFixedSize) {
  static_assert(std::is_final_v<MarineForceDynamics>);
  static_assert(std::is_trivially_copyable_v<DynamicsResult>);
  EXPECT_EQ(kMarineForceModelId, std::string_view("marine_force"));
}

TEST(MarineForceDynamics, ZeroStateMatchesHelperZeros) {
  // W = m g = 8 * 10. B = ρ g V = 2 * 10 * 4. Centers coincide, so the
  // restoring helper is the zero wrench. Zero twist and zero current make
  // Coriolis and damping the zero wrench.
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(8, 4));
  const VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
  const EnvironmentRt environment = Snapshot(FrameId::kBody, 0);
  const StatusOr<DynamicsResult> evaluated =
      model.Evaluate(state, BodyWrenchRt{}, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  const DynamicsResult& result = evaluated.value();
  EXPECT_EQ(result.diagnostics.model_id, kMarineForceModelId);
  EXPECT_EQ(result.diagnostics.dt_s, 0.25);
  EXPECT_TRUE(result.diagnostics.input_wrench_used);
  EXPECT_FALSE(result.diagnostics.allocation_invoked);
  EXPECT_EQ(result.diagnostics.relative_twist, kZero6);
  EXPECT_EQ(result.diagnostics.rigid_body_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.added_mass_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.damping_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.restoring_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.hydrodynamic_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.total_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.model_force_n, (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.diagnostics.model_torque_n_m,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.derivative.position_dot_m_s,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(result.derivative.orientation_dot_xyzw,
            (std::array<double, 4>{0, 0, 0, 0}));
  EXPECT_EQ(result.derivative.body_acceleration, kZero6);
  ExpectSumEqualsTotal(result);
}

TEST(MarineForceDynamics, RestoringHeaveMatchesWeightMinusBuoyancy) {
  // m = 4, g = 10, ρ = 2, V = 1. W - B = 20. Identity pose, zero twist.
  // ENU down is -z, so heave is -(W - B). NED down is +z.
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(4, 1));
  const EnvironmentRt environment = Snapshot(FrameId::kWorldEnu, 0);
  const StatusOr<DynamicsResult> enu =
      model.Evaluate(IdentityState(FrameId::kWorldEnu), BodyWrenchRt{},
                     environment, Duration{0});
  ASSERT_TRUE(enu.ok());
  EXPECT_EQ(enu.value().diagnostics.rigid_body_coriolis_wrench, kZero6);
  EXPECT_EQ(enu.value().diagnostics.added_mass_coriolis_wrench, kZero6);
  EXPECT_EQ(enu.value().diagnostics.damping_wrench, kZero6);
  EXPECT_EQ(enu.value().diagnostics.restoring_wrench,
            (std::array<double, 6>{0, 0, -20, 0, 0, 0}));
  EXPECT_EQ(enu.value().diagnostics.hydrodynamic_wrench,
            (std::array<double, 6>{0, 0, -20, 0, 0, 0}));
  EXPECT_EQ(enu.value().diagnostics.model_force_n,
            (std::array<double, 3>{0, 0, -20}));
  ExpectSumEqualsTotal(enu.value());

  const StatusOr<DynamicsResult> ned =
      model.Evaluate(IdentityState(FrameId::kWorldNed), BodyWrenchRt{},
                     environment, Duration{0});
  ASSERT_TRUE(ned.ok());
  EXPECT_EQ(ned.value().diagnostics.restoring_wrench,
            (std::array<double, 6>{0, 0, 20, 0, 0, 0}));
  EXPECT_EQ(ned.value().diagnostics.hydrodynamic_wrench,
            (std::array<double, 6>{0, 0, 20, 0, 0, 0}));
  ExpectSumEqualsTotal(ned.value());
}

TEST(MarineForceDynamics, CompositionMatchesHelpers) {
  // ν = [1, 0, 0, 0, 0, 0.5], body current surge 0.25, so ν_r surge is
  // 0.75. Rigid-body Coriolis uses ν. Added-mass Coriolis and damping use
  // ν_r. With the center of gravity at the origin, -C_RB(ν) ν sway is
  // -m u r = -2. -C_A(ν_r) ν_r sway is -u_r r = -0.375. Damping surge is
  // -2 * 0.75. Restoring heave is -20.
  const MarineModel model = MakeModel(4, 1);
  const MarineForceDynamics dynamics(model);
  VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
  state.body_twist = {1, 0, 0, 0, 0, 0.5};
  const EnvironmentRt environment = Snapshot(FrameId::kBody, 0.25);
  const BodyWrenchRt wrench = SampleWrench();
  const StatusOr<DynamicsResult> evaluated =
      dynamics.Evaluate(state, wrench, environment, Duration{0.1});
  ASSERT_TRUE(evaluated.ok());

  const intrinsic::vehicle::parameters::Environment helper_environment =
      HelperEnvironment(environment);
  const StatusOr<intrinsic::vehicle::dynamics::CurrentRelativeTwist> relative =
      ComputeWaterCurrentRelativeVelocity(helper_environment, state.pose_frame,
                                          state.orientation_xyzw,
                                          state.body_twist);
  const StatusOr<RigidBodyCoriolisMatrix> rigid =
      ComputeRigidBodyCoriolisMatrix(model.mass_inertia,
                                     model.centers.center_of_gravity_m,
                                     state.body_twist);
  ASSERT_TRUE(relative.ok());
  ASSERT_TRUE(rigid.ok());
  const StatusOr<AddedMassCoriolisMatrix> added =
      ComputeAddedMassCoriolisMatrix(model.added_mass,
                                     relative.value().components);
  const StatusOr<DampingWrench> damping = ComputeLinearQuadraticDampingWrench(
      model.damping, relative.value().components);
  const StatusOr<RestoringWrench> restoring =
      ComputeGravityBuoyancyRestoringWrench(
          model.mass_inertia, model.buoyancy, model.centers, helper_environment,
          state.pose_frame, state.orientation_xyzw);
  ASSERT_TRUE(added.ok());
  ASSERT_TRUE(damping.ok());
  ASSERT_TRUE(restoring.ok());

  const std::array<double, kSpatialDof> rigid_force = [&] {
    const std::array<double, kSpatialDof> product =
        Multiply(rigid.value().coefficients, state.body_twist);
    std::array<double, kSpatialDof> negated = {};
    for (int index = 0; index < kSpatialDof; ++index) {
      negated[index] = -product[index];
    }
    return negated;
  }();
  const std::array<double, kSpatialDof> added_force = [&] {
    const std::array<double, kSpatialDof> product =
        Multiply(added.value().coefficients, relative.value().components);
    std::array<double, kSpatialDof> negated = {};
    for (int index = 0; index < kSpatialDof; ++index) {
      negated[index] = -product[index];
    }
    return negated;
  }();

  const DynamicsResult& result = evaluated.value();
  EXPECT_EQ(result.diagnostics.relative_twist, relative.value().components);
  EXPECT_EQ(result.diagnostics.relative_twist,
            (std::array<double, 6>{0.75, 0, 0, 0, 0, 0.5}));
  for (int index = 0; index < kSpatialDof; ++index) {
    EXPECT_NEAR(result.diagnostics.rigid_body_coriolis_wrench[index],
                rigid_force[index], kSumTolerance);
    EXPECT_NEAR(result.diagnostics.added_mass_coriolis_wrench[index],
                added_force[index], kSumTolerance);
    EXPECT_NEAR(result.diagnostics.damping_wrench[index],
                damping.value().components[index], kSumTolerance);
    EXPECT_NEAR(result.diagnostics.restoring_wrench[index],
                restoring.value().components[index], kSumTolerance);
  }
  EXPECT_NEAR(result.diagnostics.rigid_body_coriolis_wrench[kSway], -2,
              kSumTolerance);
  EXPECT_NEAR(result.diagnostics.added_mass_coriolis_wrench[kSway], -0.375,
              kSumTolerance);
  EXPECT_NEAR(result.diagnostics.damping_wrench[kSurge], -1.5, kSumTolerance);
  EXPECT_NEAR(result.diagnostics.damping_wrench[kYaw], -1, kSumTolerance);
  EXPECT_NEAR(result.diagnostics.restoring_wrench[kHeave], -20, kSumTolerance);
  ExpectSumEqualsTotal(result);

  std::array<double, kSpatialDof> total =
      result.diagnostics.hydrodynamic_wrench;
  total[kSurge] += wrench.force_n[0];
  total[kSway] += wrench.force_n[1];
  total[kHeave] += wrench.force_n[2];
  total[kRoll] += wrench.torque_n_m[0];
  total[kPitch] += wrench.torque_n_m[1];
  total[kYaw] += wrench.torque_n_m[2];
  for (int index = 0; index < kSpatialDof; ++index) {
    EXPECT_NEAR(result.diagnostics.total_wrench[index], total[index],
                kSumTolerance);
  }
  EXPECT_TRUE(result.diagnostics.input_wrench_used);
  EXPECT_FALSE(result.diagnostics.allocation_invoked);
  EXPECT_EQ(result.derivative.body_acceleration, kZero6);
  EXPECT_EQ(result.derivative.position_dot_m_s,
            (std::array<double, 3>{1, 0, 0}));
}

TEST(MarineForceDynamics, YawedPoseRateStaysKinematic) {
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(8, 4));
  VehicleStateRt state = IdentityState(FrameId::kWorldNed);
  state.orientation_xyzw = {0, 0, 1, 0};
  state.body_twist[kSurge] = 1;
  state.body_twist[kYaw] = 2;
  const StatusOr<DynamicsResult> evaluated = model.Evaluate(
      state, BodyWrenchRt{}, Snapshot(FrameId::kWorldNed, 0), Duration{0.5});
  ASSERT_TRUE(evaluated.ok());
  EXPECT_EQ(evaluated.value().derivative.position_dot_m_s,
            (std::array<double, 3>{-1, 0, 0}));
  EXPECT_EQ(evaluated.value().derivative.orientation_dot_xyzw,
            (std::array<double, 4>{0, 0, 0, -1}));
  EXPECT_EQ(evaluated.value().derivative.body_acceleration, kZero6);
  EXPECT_EQ(evaluated.value().diagnostics.dt_s, 0.5);
}

TEST(MarineForceDynamics, RepeatedEvaluationMatches) {
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(4, 1));
  VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
  state.body_twist = {1, 0, 0, 0, 0, 0.5};
  const EnvironmentRt environment = Snapshot(FrameId::kWorldEnu, 0.1);
  const BodyWrenchRt wrench = SampleWrench();
  const StatusOr<DynamicsResult> first =
      model.Evaluate(state, wrench, environment, Duration{0.02});
  ASSERT_TRUE(first.ok());
  for (int attempt = 0; attempt < 3; ++attempt) {
    const StatusOr<DynamicsResult> again =
        model.Evaluate(state, wrench, environment, Duration{0.02});
    ASSERT_TRUE(again.ok());
    EXPECT_EQ(again.value().diagnostics.relative_twist,
              first.value().diagnostics.relative_twist);
    EXPECT_EQ(again.value().diagnostics.rigid_body_coriolis_wrench,
              first.value().diagnostics.rigid_body_coriolis_wrench);
    EXPECT_EQ(again.value().diagnostics.added_mass_coriolis_wrench,
              first.value().diagnostics.added_mass_coriolis_wrench);
    EXPECT_EQ(again.value().diagnostics.damping_wrench,
              first.value().diagnostics.damping_wrench);
    EXPECT_EQ(again.value().diagnostics.restoring_wrench,
              first.value().diagnostics.restoring_wrench);
    EXPECT_EQ(again.value().diagnostics.hydrodynamic_wrench,
              first.value().diagnostics.hydrodynamic_wrench);
    EXPECT_EQ(again.value().diagnostics.total_wrench,
              first.value().diagnostics.total_wrench);
    EXPECT_EQ(again.value().derivative.position_dot_m_s,
              first.value().derivative.position_dot_m_s);
  }
}

TEST(MarineForceDynamics, FailedTermReturnsTypedErrorAndFiniteZeros) {
  MarineModel model = MakeModel(4, 1);
  model.added_mass.coefficients = {};
  const MarineForceDynamics dynamics(model);
  VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
  state.body_twist[kSurge] = 1;
  const StatusOr<DynamicsResult> evaluated = dynamics.Evaluate(
      state, SampleWrench(), Snapshot(FrameId::kBody, 0), Duration{0.2});
  ASSERT_FALSE(evaluated.ok());
  EXPECT_EQ(evaluated.status().code, DynamicsErrorCode::kInvalidArgument);
  EXPECT_EQ(
      evaluated.status().message,
      std::string_view("added_mass.coefficients is not positive definite"));
  ExpectFiniteZero(evaluated.value());
}

TEST(MarineForceDynamics, FirstDefectWins) {
  MarineModel model = MakeModel(0, 1);
  model.damping.quadratic_coefficients[kSurge] = -1;
  const MarineForceDynamics dynamics(model);
  const StatusOr<DynamicsResult> mass_before_damping =
      dynamics.Evaluate(IdentityState(FrameId::kWorldEnu), BodyWrenchRt{},
                        Snapshot(FrameId::kBody, 0), Duration{0});
  ASSERT_FALSE(mass_before_damping.ok());
  EXPECT_EQ(mass_before_damping.status().message,
            std::string_view("mass_inertia.mass_kg must be greater than zero"));
  ExpectFiniteZero(mass_before_damping.value());

  MarineModel damping_first = MakeModel(4, 1);
  damping_first.damping.quadratic_coefficients[kYaw] = -1;
  damping_first.buoyancy.displaced_volume_m3 = 0;
  const MarineForceDynamics damping_model(damping_first);
  const StatusOr<DynamicsResult> damping_before_restoring =
      damping_model.Evaluate(IdentityState(FrameId::kWorldEnu), BodyWrenchRt{},
                             Snapshot(FrameId::kBody, 0), Duration{1});
  ASSERT_FALSE(damping_before_restoring.ok());
  EXPECT_EQ(
      damping_before_restoring.status().message,
      std::string_view("damping.quadratic_coefficients must be greater than or "
                       "equal to zero"));
  ExpectFiniteZero(damping_before_restoring.value());

  const MarineForceDynamics valid(MakeModel(4, 1));
  EnvironmentRt zero_gravity = Snapshot(FrameId::kBody, 0);
  zero_gravity.gravity_m_s2 = 0;
  VehicleStateRt moving = IdentityState(FrameId::kWorldEnu);
  moving.body_twist[kSurge] = 2;
  const StatusOr<DynamicsResult> restoring_after_other_terms =
      valid.Evaluate(moving, SampleWrench(), zero_gravity, Duration{0.3});
  ASSERT_FALSE(restoring_after_other_terms.ok());
  EXPECT_EQ(
      restoring_after_other_terms.status().message,
      std::string_view("environment.gravity_m_s2 must be greater than zero"));
  ExpectFiniteZero(restoring_after_other_terms.value());
}

TEST(MarineForceDynamics, InputValidationPrecedesTermErrors) {
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(0, 0));
  VehicleStateRt state = IdentityState(FrameId::kBody);
  state.body_twist[kSurge] = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<DynamicsResult> evaluated = model.Evaluate(
      state, BodyWrenchRt{}, Snapshot(FrameId::kBody, 0), Duration{-1});
  ASSERT_FALSE(evaluated.ok());
  EXPECT_EQ(evaluated.status().code, DynamicsErrorCode::kInvalidArgument);
  EXPECT_EQ(
      evaluated.status().message,
      std::string_view("state pose frame must be world_enu or world_ned"));
  ExpectFiniteZero(evaluated.value());
}

TEST(MarineForceDynamics, NonFiniteProductReturnsFiniteZeros) {
  MarineModel model = MakeModel(1e200, 1);
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2 = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };
  const MarineForceDynamics dynamics(model);
  VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
  state.body_twist = {1e100, 0, 0, 0, 1e100, 0};
  const StatusOr<DynamicsResult> evaluated = dynamics.Evaluate(
      state, BodyWrenchRt{}, Snapshot(FrameId::kBody, 0), Duration{0});
  ASSERT_FALSE(evaluated.ok());
  EXPECT_EQ(evaluated.status().code, DynamicsErrorCode::kInvalidArgument);
  EXPECT_EQ(evaluated.status().message,
            std::string_view("composed wrench is not finite"));
  ExpectFiniteZero(evaluated.value());
}

TEST(MarineForceDynamics, ConcurrentEvaluationsStayIndependent) {
  const MarineForceDynamics model = MarineForceDynamics(MakeModel(8, 4));
  StatusOr<DynamicsResult> forward = StatusOr<DynamicsResult>::Failure(
      intrinsic::vehicle::dynamics::DynamicsStatus::InvalidArgument("unset"));
  StatusOr<DynamicsResult> reversed = StatusOr<DynamicsResult>::Failure(
      intrinsic::vehicle::dynamics::DynamicsStatus::InvalidArgument("unset"));
  std::thread ahead([&] {
    VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
    state.body_twist[kSurge] = 1;
    forward = model.Evaluate(state, BodyWrenchRt{}, Snapshot(FrameId::kBody, 0),
                             Duration{0.25});
  });
  std::thread flipped([&] {
    VehicleStateRt state = IdentityState(FrameId::kWorldEnu);
    state.orientation_xyzw = {1, 0, 0, 0};
    state.body_twist[kSway] = 1;
    reversed = model.Evaluate(state, BodyWrenchRt{},
                              Snapshot(FrameId::kBody, 0), Duration{0.5});
  });
  ahead.join();
  flipped.join();
  ASSERT_TRUE(forward.ok());
  ASSERT_TRUE(reversed.ok());
  EXPECT_EQ(forward.value().derivative.position_dot_m_s,
            (std::array<double, 3>{1, 0, 0}));
  EXPECT_EQ(reversed.value().derivative.position_dot_m_s,
            (std::array<double, 3>{0, -1, 0}));
  EXPECT_EQ(forward.value().diagnostics.dt_s, 0.25);
  EXPECT_EQ(reversed.value().diagnostics.dt_s, 0.5);
  EXPECT_EQ(forward.value().derivative.body_acceleration, kZero6);
  EXPECT_EQ(reversed.value().derivative.body_acceleration, kZero6);
}

}  // namespace
