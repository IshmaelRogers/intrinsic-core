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

#include <array>
#include <cmath>
#include <iterator>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/marine_force_dynamics.h"
#include "intrinsic_vehicle/intrinsic/vehicle/dynamics/marine_force_analytic_fixtures.h"

namespace {

using intrinsic::vehicle::dynamics::AnalyticFixture;
using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::DynamicsResult;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::kAnalyticExactTolerance;
using intrinsic::vehicle::dynamics::kAnalyticIdentityQuaternion;
using intrinsic::vehicle::dynamics::kHeave;
using intrinsic::vehicle::dynamics::kMarineForceAnalyticFixtures;
using intrinsic::vehicle::dynamics::kMarineForceModelId;
using intrinsic::vehicle::dynamics::kPitch;
using intrinsic::vehicle::dynamics::kRoll;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kSurge;
using intrinsic::vehicle::dynamics::kSway;
using intrinsic::vehicle::dynamics::kYaw;
using intrinsic::vehicle::dynamics::MarineForceDynamics;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::parameters::MarineModel;

constexpr std::array<double, kSpatialDof> kZero6 = {};

const AnalyticFixture* FindFixture(std::string_view name) {
  for (const AnalyticFixture& fixture : kMarineForceAnalyticFixtures) {
    if (fixture.name == name) {
      return &fixture;
    }
  }
  return nullptr;
}

std::array<double, kSpatialDof * kSpatialDof> Diagonal6(
    const std::array<double, kSpatialDof>& diagonal) {
  std::array<double, kSpatialDof * kSpatialDof> matrix = {};
  for (int index = 0; index < kSpatialDof; ++index) {
    matrix[index * kSpatialDof + index] = diagonal[index];
  }
  return matrix;
}

double Digest(const std::array<double, kSpatialDof>& wrench) {
  double digest = 0;
  for (int index = 0; index < kSpatialDof; ++index) {
    digest += static_cast<double>(index + 1) * wrench[index];
  }
  return digest;
}

double Dot(const std::array<double, kSpatialDof>& left,
           const std::array<double, kSpatialDof>& right) {
  double sum = 0;
  for (int index = 0; index < kSpatialDof; ++index) {
    sum += left[index] * right[index];
  }
  return sum;
}

double InputComponent(const AnalyticFixture& fixture, int index) {
  if (index < 3) {
    return fixture.input_force_n[index];
  }
  return fixture.input_torque_n_m[index - 3];
}

// Stored environment is a sentinel. Evaluate must read the snapshot.
MarineModel MakeModel(const AnalyticFixture& fixture) {
  MarineModel model;
  model.model_id = "analytic_regression";
  model.mass_inertia.mass_kg = fixture.mass_kg;
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2 = {
      fixture.inertia_diag[0], 0, 0, 0, fixture.inertia_diag[1], 0, 0, 0,
      fixture.inertia_diag[2],
  };
  model.centers.center_of_gravity_m.x = fixture.cog_m[0];
  model.centers.center_of_gravity_m.y = fixture.cog_m[1];
  model.centers.center_of_gravity_m.z = fixture.cog_m[2];
  model.centers.center_of_buoyancy_m.x = fixture.cob_m[0];
  model.centers.center_of_buoyancy_m.y = fixture.cob_m[1];
  model.centers.center_of_buoyancy_m.z = fixture.cob_m[2];
  model.buoyancy.displaced_volume_m3 = fixture.displaced_volume_m3;
  model.added_mass.coefficients = Diagonal6(fixture.added_mass_diag);
  model.damping.linear_coefficients = Diagonal6(fixture.damping_linear_diag);
  model.damping.quadratic_coefficients = fixture.damping_quadratic;
  model.environment.gravity_m_s2 = 1;
  model.environment.fluid_density_kg_m3 = 1;
  model.environment.current_velocity_m_s = {9, 9, 9};
  model.environment.current_frame_id = "body";
  return model;
}

StatusOr<DynamicsResult> EvaluateFixture(const AnalyticFixture& fixture) {
  const MarineForceDynamics dynamics(MakeModel(fixture));
  VehicleStateRt state;
  state.pose_frame = fixture.pose_frame;
  state.orientation_xyzw = kAnalyticIdentityQuaternion;
  state.body_twist = fixture.body_twist;
  EnvironmentRt environment;
  environment.gravity_m_s2 = fixture.gravity_m_s2;
  environment.fluid_density_kg_m3 = fixture.fluid_density_kg_m3;
  environment.current_velocity_m_s = fixture.current_velocity_m_s;
  environment.current_frame = fixture.current_frame;
  BodyWrenchRt wrench;
  wrench.force_n = fixture.input_force_n;
  wrench.torque_n_m = fixture.input_torque_n_m;
  return dynamics.Evaluate(state, wrench, environment, Duration{fixture.dt_s});
}

void ExpectNear6(const std::array<double, kSpatialDof>& actual,
                 const std::array<double, kSpatialDof>& expected,
                 double tolerance) {
  for (int index = 0; index < kSpatialDof; ++index) {
    EXPECT_NEAR(actual[index], expected[index], tolerance) << index;
  }
}

void ExpectFiniteZero(const DynamicsResult& result) {
  EXPECT_EQ(result.diagnostics.model_id, kMarineForceModelId);
  EXPECT_EQ(result.diagnostics.dt_s, 0);
  EXPECT_TRUE(std::isfinite(result.diagnostics.dt_s));
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
  for (double value : result.diagnostics.total_wrench) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

void ExpectTermSum(const AnalyticFixture& fixture) {
  for (int index = 0; index < kSpatialDof; ++index) {
    const double hydro = fixture.rigid_body_coriolis[index] +
                         fixture.added_mass_coriolis[index] +
                         fixture.damping[index] + fixture.restoring[index];
    EXPECT_NEAR(hydro, fixture.hydrodynamic[index], fixture.tolerance) << index;
    EXPECT_NEAR(fixture.hydrodynamic[index] + InputComponent(fixture, index),
                fixture.total[index], fixture.tolerance)
        << index;
  }
  EXPECT_EQ(Digest(fixture.hydrodynamic), fixture.hydrodynamic_digest);
}

TEST(MarineForceAnalyticRegression, FixturesMatchHandValues) {
  ASSERT_GE(std::size(kMarineForceAnalyticFixtures), 15);
  for (const AnalyticFixture& fixture : kMarineForceAnalyticFixtures) {
    SCOPED_TRACE(fixture.name);
    const StatusOr<DynamicsResult> evaluated = EvaluateFixture(fixture);
    if (!fixture.expect_ok) {
      ASSERT_FALSE(evaluated.ok());
      EXPECT_EQ(evaluated.status().code, DynamicsErrorCode::kInvalidArgument);
      EXPECT_EQ(evaluated.status().message, fixture.status_message);
      ExpectFiniteZero(evaluated.value());
      continue;
    }

    ASSERT_TRUE(evaluated.ok()) << evaluated.status().message;
    const DynamicsResult& result = evaluated.value();
    ExpectTermSum(fixture);
    EXPECT_LE(fixture.tolerance, kAnalyticExactTolerance);
    EXPECT_EQ(result.diagnostics.model_id, kMarineForceModelId);
    EXPECT_EQ(result.diagnostics.dt_s, fixture.dt_s);
    EXPECT_TRUE(result.diagnostics.input_wrench_used);
    EXPECT_FALSE(result.diagnostics.allocation_invoked);
    // Composition does not form M and does not solve ν̇.
    EXPECT_EQ(result.derivative.body_acceleration, kZero6);
    ExpectNear6(result.diagnostics.relative_twist, fixture.relative_twist,
                fixture.tolerance);
    ExpectNear6(result.diagnostics.rigid_body_coriolis_wrench,
                fixture.rigid_body_coriolis, fixture.tolerance);
    ExpectNear6(result.diagnostics.added_mass_coriolis_wrench,
                fixture.added_mass_coriolis, fixture.tolerance);
    ExpectNear6(result.diagnostics.damping_wrench, fixture.damping,
                fixture.tolerance);
    ExpectNear6(result.diagnostics.restoring_wrench, fixture.restoring,
                fixture.tolerance);
    ExpectNear6(result.diagnostics.hydrodynamic_wrench, fixture.hydrodynamic,
                fixture.tolerance);
    ExpectNear6(result.diagnostics.total_wrench, fixture.total,
                fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_force_n[0],
                fixture.hydrodynamic[kSurge], fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_force_n[1],
                fixture.hydrodynamic[kSway], fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_force_n[2],
                fixture.hydrodynamic[kHeave], fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_torque_n_m[0],
                fixture.hydrodynamic[kRoll], fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_torque_n_m[1],
                fixture.hydrodynamic[kPitch], fixture.tolerance);
    EXPECT_NEAR(result.diagnostics.model_torque_n_m[2],
                fixture.hydrodynamic[kYaw], fixture.tolerance);
    EXPECT_EQ(Digest(result.diagnostics.hydrodynamic_wrench),
              fixture.hydrodynamic_digest);
    EXPECT_NEAR(Dot(result.diagnostics.relative_twist,
                    result.diagnostics.damping_wrench),
                fixture.damping_power, fixture.tolerance);
    // C is skew-symmetric, so ν^T C ν = 0. Damping dissipates.
    EXPECT_NEAR(
        Dot(fixture.body_twist, result.diagnostics.rigid_body_coriolis_wrench),
        0, fixture.tolerance);
    EXPECT_NEAR(Dot(result.diagnostics.relative_twist,
                    result.diagnostics.added_mass_coriolis_wrench),
                0, fixture.tolerance);
    EXPECT_LE(Dot(result.diagnostics.relative_twist,
                  result.diagnostics.damping_wrench),
              fixture.tolerance);
    for (double value : result.derivative.position_dot_m_s) {
      EXPECT_TRUE(std::isfinite(value));
    }
    for (double value : result.derivative.orientation_dot_xyzw) {
      EXPECT_TRUE(std::isfinite(value));
    }
    if (fixture.witness_twist_split) {
      const double rigid_sway =
          result.diagnostics.rigid_body_coriolis_wrench[kSway];
      const double added_sway =
          result.diagnostics.added_mass_coriolis_wrench[kSway];
      const double damping_surge = result.diagnostics.damping_wrench[kSurge];
      EXPECT_GT(std::abs(rigid_sway - fixture.rigid_sway_if_nu_r), 1);
      EXPECT_GT(std::abs(added_sway - fixture.added_sway_if_nu), 1);
      EXPECT_GT(std::abs(damping_surge - fixture.damping_surge_if_nu), 1);
    }
  }
}

TEST(MarineForceAnalyticRegression, RepeatedEvaluationIsIdentical) {
  for (const AnalyticFixture& fixture : kMarineForceAnalyticFixtures) {
    if (!fixture.expect_ok) {
      continue;
    }
    SCOPED_TRACE(fixture.name);
    const StatusOr<DynamicsResult> first = EvaluateFixture(fixture);
    const StatusOr<DynamicsResult> second = EvaluateFixture(fixture);
    ASSERT_TRUE(first.ok());
    ASSERT_TRUE(second.ok());
    EXPECT_EQ(second.value().diagnostics.dt_s, first.value().diagnostics.dt_s);
    EXPECT_EQ(second.value().diagnostics.relative_twist,
              first.value().diagnostics.relative_twist);
    EXPECT_EQ(second.value().diagnostics.rigid_body_coriolis_wrench,
              first.value().diagnostics.rigid_body_coriolis_wrench);
    EXPECT_EQ(second.value().diagnostics.added_mass_coriolis_wrench,
              first.value().diagnostics.added_mass_coriolis_wrench);
    EXPECT_EQ(second.value().diagnostics.damping_wrench,
              first.value().diagnostics.damping_wrench);
    EXPECT_EQ(second.value().diagnostics.restoring_wrench,
              first.value().diagnostics.restoring_wrench);
    EXPECT_EQ(second.value().diagnostics.hydrodynamic_wrench,
              first.value().diagnostics.hydrodynamic_wrench);
    EXPECT_EQ(second.value().diagnostics.total_wrench,
              first.value().diagnostics.total_wrench);
    EXPECT_EQ(second.value().diagnostics.model_force_n,
              first.value().diagnostics.model_force_n);
    EXPECT_EQ(second.value().diagnostics.model_torque_n_m,
              first.value().diagnostics.model_torque_n_m);
    EXPECT_EQ(second.value().derivative.body_acceleration,
              first.value().derivative.body_acceleration);
  }
}

TEST(MarineForceAnalyticRegression, ReversedVelocityFlipsOddTermsOnly) {
  const AnalyticFixture* forward =
      FindFixture("constant_velocity_coriolis_damping");
  const AnalyticFixture* reversed = FindFixture("reversed_velocity_odd_terms");
  ASSERT_NE(forward, nullptr);
  ASSERT_NE(reversed, nullptr);
  const StatusOr<DynamicsResult> forward_result = EvaluateFixture(*forward);
  const StatusOr<DynamicsResult> reversed_result = EvaluateFixture(*reversed);
  ASSERT_TRUE(forward_result.ok());
  ASSERT_TRUE(reversed_result.ok());
  for (int index = 0; index < kSpatialDof; ++index) {
    // Coriolis products are homogeneous of degree two.
    EXPECT_NEAR(
        reversed_result.value().diagnostics.rigid_body_coriolis_wrench[index],
        forward_result.value().diagnostics.rigid_body_coriolis_wrench[index],
        kAnalyticExactTolerance)
        << index;
    EXPECT_NEAR(
        reversed_result.value().diagnostics.added_mass_coriolis_wrench[index],
        forward_result.value().diagnostics.added_mass_coriolis_wrench[index],
        kAnalyticExactTolerance)
        << index;
    // Linear and quadratic damping are odd in ν_r.
    EXPECT_NEAR(reversed_result.value().diagnostics.damping_wrench[index],
                -forward_result.value().diagnostics.damping_wrench[index],
                kAnalyticExactTolerance)
        << index;
  }
}

}  // namespace
