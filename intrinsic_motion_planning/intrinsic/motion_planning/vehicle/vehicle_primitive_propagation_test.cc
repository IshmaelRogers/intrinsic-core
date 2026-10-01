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

#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsResult;
using ::intrinsic::vehicle::dynamics::DynamicsStatus;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::StatusOr;
using ::intrinsic::vehicle::dynamics::VehicleDynamics;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;
using ::intrinsic::vehicle::dynamics::ZeroForceDynamics;

constexpr double kTol = 1e-9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

VehiclePlanningState Origin() { return VehiclePlanningState{}; }

VehicleMotionPrimitive SurgePrimitive(double duration_s) {
  VehicleMotionPrimitive primitive;
  primitive.id = "uuv-prim-000";
  primitive.control.linear_x = 10.0;
  primitive.duration_s = duration_s;
  return primitive;
}

PropagationConfig NominalConfig() {
  PropagationConfig config;
  config.dt_s = 0.5;
  config.max_steps = 4;
  config.mass_diag = {10, 10, 10, 10, 10, 10};
  config.gravity_m_s2 = 0.0;
  config.fluid_density_kg_m3 = 0.0;
  config.current_world_enu_m_s = {0.0, 0.0, 0.0};
  return config;
}

void ExpectFailure(const VehiclePlanningState& start,
                   const VehicleMotionPrimitive& primitive,
                   const PropagationConfig& config,
                   const VehicleDynamics& dynamics, PropagationError error) {
  const PropagationResult result =
      PropagateUuvMotionPrimitive(start, primitive, config, dynamics);
  EXPECT_EQ(result.error, error);
  EXPECT_TRUE(result.samples.empty());
}

void ExpectOtherComponentsZero(const VehiclePlanningState& state) {
  EXPECT_DOUBLE_EQ(state.position.y, 0.0);
  EXPECT_DOUBLE_EQ(state.position.z, 0.0);
  EXPECT_NEAR(state.orientation.x, 0.0, kTol);
  EXPECT_NEAR(state.orientation.y, 0.0, kTol);
  EXPECT_NEAR(state.orientation.z, 0.0, kTol);
  EXPECT_NEAR(state.orientation.w, 1.0, kTol);
  EXPECT_DOUBLE_EQ(state.twist.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.linear_z, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.angular_x, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.angular_y, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.angular_z, 0.0);
}

class FailingDynamics final : public VehicleDynamics {
 public:
  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt&, const BodyWrenchRt&,
                                    const EnvironmentRt&,
                                    Duration) const override {
    return StatusOr<DynamicsResult>::Failure(
        DynamicsStatus::InvalidArgument("injected failure"));
  }
};

class SecondEvaluateFails final : public VehicleDynamics {
 public:
  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt& state,
                                    const BodyWrenchRt& wrench,
                                    const EnvironmentRt& environment,
                                    Duration dt) const override {
    ++calls_;
    if (calls_ >= 2) {
      return StatusOr<DynamicsResult>::Failure(
          DynamicsStatus::InvalidArgument("second evaluate"));
    }
    return inner_.Evaluate(state, wrench, environment, dt);
  }

 private:
  mutable int calls_ = 0;
  ZeroForceDynamics inner_;
};

// Reports input_wrench_used so planning must take total_wrench, not the
// primitive control. Pose rates still come from ZeroForceDynamics.
class TotalWrenchDynamics final : public VehicleDynamics {
 public:
  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt& state,
                                    const BodyWrenchRt& wrench,
                                    const EnvironmentRt& environment,
                                    Duration dt) const override {
    StatusOr<DynamicsResult> evaluated =
        inner_.Evaluate(state, wrench, environment, dt);
    if (!evaluated.ok()) {
      return evaluated;
    }
    DynamicsResult result = evaluated.value();
    result.diagnostics.input_wrench_used = true;
    result.diagnostics.total_wrench = {20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return StatusOr<DynamicsResult>::Ok(result);
  }

 private:
  ZeroForceDynamics inner_;
};

class NonFiniteOrientationRate final : public VehicleDynamics {
 public:
  StatusOr<DynamicsResult> Evaluate(const VehicleStateRt& state,
                                    const BodyWrenchRt& wrench,
                                    const EnvironmentRt& environment,
                                    Duration dt) const override {
    StatusOr<DynamicsResult> evaluated =
        inner_.Evaluate(state, wrench, environment, dt);
    if (!evaluated.ok()) {
      return evaluated;
    }
    DynamicsResult result = evaluated.value();
    result.derivative.orientation_dot_xyzw = {1e308, 1e308, 1e308, 1e308};
    return StatusOr<DynamicsResult>::Ok(result);
  }

 private:
  ZeroForceDynamics inner_;
};

TEST(VehiclePrimitivePropagationTest, ZeroCurrentNominal) {
  const ZeroForceDynamics dynamics;
  const PropagationResult result = PropagateUuvMotionPrimitive(
      Origin(), SurgePrimitive(1.0), NominalConfig(), dynamics);
  ASSERT_EQ(result.error, PropagationError::kOk);
  ASSERT_EQ(result.samples.size(), 3u);

  EXPECT_DOUBLE_EQ(result.samples[0].time_s, 0.0);
  EXPECT_DOUBLE_EQ(result.samples[0].state.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.samples[0].state.twist.linear_x, 0.0);

  EXPECT_DOUBLE_EQ(result.samples[1].time_s, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.twist.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.position.x, 0.25);
  ExpectOtherComponentsZero(result.samples[1].state);

  EXPECT_DOUBLE_EQ(result.samples[2].time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.samples[2].state.twist.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result.samples[2].state.position.x, 0.75);
  ExpectOtherComponentsZero(result.samples[2].state);
}

TEST(VehiclePrimitivePropagationTest, ConstantCurrentAddsWorldDrift) {
  PropagationConfig config = NominalConfig();
  config.current_world_enu_m_s = {0.2, 0.0, 0.0};
  const ZeroForceDynamics dynamics;
  const PropagationResult result = PropagateUuvMotionPrimitive(
      Origin(), SurgePrimitive(1.0), config, dynamics);
  ASSERT_EQ(result.error, PropagationError::kOk);
  ASSERT_EQ(result.samples.size(), 3u);

  EXPECT_DOUBLE_EQ(result.samples[1].time_s, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.twist.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.position.x, 0.35);
  ExpectOtherComponentsZero(result.samples[1].state);

  EXPECT_DOUBLE_EQ(result.samples[2].time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.samples[2].state.twist.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result.samples[2].state.position.x, 0.95);
  ExpectOtherComponentsZero(result.samples[2].state);
}

TEST(VehiclePrimitivePropagationTest, BadConfigIsRejected) {
  const ZeroForceDynamics dynamics;
  const VehiclePlanningState start = Origin();
  const VehicleMotionPrimitive primitive = SurgePrimitive(1.0);

  PropagationConfig dt = NominalConfig();
  dt.dt_s = 0.0;
  ExpectFailure(start, primitive, dt, dynamics, PropagationError::kBadConfig);
  dt.dt_s = -0.5;
  ExpectFailure(start, primitive, dt, dynamics, PropagationError::kBadConfig);
  dt.dt_s = kNaN;
  ExpectFailure(start, primitive, dt, dynamics, PropagationError::kBadConfig);

  PropagationConfig steps = NominalConfig();
  steps.max_steps = 0;
  ExpectFailure(start, primitive, steps, dynamics,
                PropagationError::kBadConfig);
  steps.max_steps = -3;
  ExpectFailure(start, primitive, steps, dynamics,
                PropagationError::kBadConfig);

  PropagationConfig mass = NominalConfig();
  mass.mass_diag[2] = 0.0;
  ExpectFailure(start, primitive, mass, dynamics, PropagationError::kBadConfig);
  mass.mass_diag[2] = -1.0;
  ExpectFailure(start, primitive, mass, dynamics, PropagationError::kBadConfig);
  mass.mass_diag[2] = kNaN;
  ExpectFailure(start, primitive, mass, dynamics, PropagationError::kBadConfig);

  PropagationConfig gravity = NominalConfig();
  gravity.gravity_m_s2 = -1.0;
  ExpectFailure(start, primitive, gravity, dynamics,
                PropagationError::kBadConfig);
  gravity.gravity_m_s2 = kInf;
  ExpectFailure(start, primitive, gravity, dynamics,
                PropagationError::kBadConfig);

  PropagationConfig density = NominalConfig();
  density.fluid_density_kg_m3 = -1.0;
  ExpectFailure(start, primitive, density, dynamics,
                PropagationError::kBadConfig);

  PropagationConfig current = NominalConfig();
  current.current_world_enu_m_s[1] = kNaN;
  ExpectFailure(start, primitive, current, dynamics,
                PropagationError::kBadConfig);
}

TEST(VehiclePrimitivePropagationTest, BadConfigBeatsBadStart) {
  const ZeroForceDynamics dynamics;
  VehiclePlanningState start = Origin();
  start.position.x = kNaN;
  PropagationConfig config = NominalConfig();
  config.dt_s = 0.0;
  ExpectFailure(start, SurgePrimitive(1.0), config, dynamics,
                PropagationError::kBadConfig);
}

TEST(VehiclePrimitivePropagationTest, BadStartIsRejected) {
  const ZeroForceDynamics dynamics;
  const PropagationConfig config = NominalConfig();
  const VehicleMotionPrimitive primitive = SurgePrimitive(1.0);

  VehiclePlanningState position = Origin();
  position.position.y = kInf;
  ExpectFailure(position, primitive, config, dynamics,
                PropagationError::kBadStart);

  VehiclePlanningState twist = Origin();
  twist.twist.angular_z = kNaN;
  ExpectFailure(twist, primitive, config, dynamics,
                PropagationError::kBadStart);

  VehiclePlanningState orientation = Origin();
  orientation.orientation = embodiment::Quaternion{0, 0, 0, 2};
  ExpectFailure(orientation, primitive, config, dynamics,
                PropagationError::kBadStart);
}

TEST(VehiclePrimitivePropagationTest, BadPrimitiveIsRejected) {
  const ZeroForceDynamics dynamics;
  const PropagationConfig config = NominalConfig();
  VehicleMotionPrimitive primitive = SurgePrimitive(0.0);
  ExpectFailure(Origin(), primitive, config, dynamics,
                PropagationError::kBadPrimitive);
  primitive.duration_s = -1.0;
  ExpectFailure(Origin(), primitive, config, dynamics,
                PropagationError::kBadPrimitive);
  primitive.duration_s = kNaN;
  ExpectFailure(Origin(), primitive, config, dynamics,
                PropagationError::kBadPrimitive);
  primitive = SurgePrimitive(1.0);
  primitive.control.angular_y = kInf;
  ExpectFailure(Origin(), primitive, config, dynamics,
                PropagationError::kBadPrimitive);
}

TEST(VehiclePrimitivePropagationTest, StepBudgetFailsBeforeIntegrating) {
  const ZeroForceDynamics dynamics;
  PropagationConfig config = NominalConfig();
  config.max_steps = 1;
  ExpectFailure(Origin(), SurgePrimitive(1.0), config, dynamics,
                PropagationError::kStepBudget);

  config.max_steps = 2;
  const PropagationResult on_budget = PropagateUuvMotionPrimitive(
      Origin(), SurgePrimitive(1.0), config, dynamics);
  EXPECT_EQ(on_budget.error, PropagationError::kOk);
  EXPECT_EQ(on_budget.samples.size(), 3u);
}

TEST(VehiclePrimitivePropagationTest, DynamicsFailureClearsSamples) {
  const FailingDynamics failing;
  ExpectFailure(Origin(), SurgePrimitive(1.0), NominalConfig(), failing,
                PropagationError::kDynamicsFailed);

  const SecondEvaluateFails second;
  ExpectFailure(Origin(), SurgePrimitive(1.0), NominalConfig(), second,
                PropagationError::kDynamicsFailed);

  const NonFiniteOrientationRate overflow;
  ExpectFailure(Origin(), SurgePrimitive(1.0), NominalConfig(), overflow,
                PropagationError::kDynamicsFailed);
}

TEST(VehiclePrimitivePropagationTest, InputWrenchUsedSelectsTotalWrench) {
  const TotalWrenchDynamics dynamics;
  const PropagationResult result = PropagateUuvMotionPrimitive(
      Origin(), SurgePrimitive(0.5), NominalConfig(), dynamics);
  ASSERT_EQ(result.error, PropagationError::kOk);
  ASSERT_EQ(result.samples.size(), 2u);
  // a = 20/10 = 2. Semi-implicit: twist = 1, position = 0.5 * 1.
  EXPECT_DOUBLE_EQ(result.samples[1].time_s, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.twist.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result.samples[1].state.position.x, 0.5);
}

TEST(VehiclePrimitivePropagationTest, IsDeterministic) {
  const ZeroForceDynamics dynamics;
  const PropagationConfig config = NominalConfig();
  const VehicleMotionPrimitive primitive = SurgePrimitive(1.0);
  const PropagationResult first =
      PropagateUuvMotionPrimitive(Origin(), primitive, config, dynamics);
  const PropagationResult second =
      PropagateUuvMotionPrimitive(Origin(), primitive, config, dynamics);
  ASSERT_EQ(first.error, PropagationError::kOk);
  ASSERT_EQ(second.error, PropagationError::kOk);
  ASSERT_EQ(first.samples.size(), second.samples.size());
  for (std::size_t i = 0; i < first.samples.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.samples[i].time_s, second.samples[i].time_s);
    EXPECT_DOUBLE_EQ(first.samples[i].state.position.x,
                     second.samples[i].state.position.x);
    EXPECT_DOUBLE_EQ(first.samples[i].state.position.y,
                     second.samples[i].state.position.y);
    EXPECT_DOUBLE_EQ(first.samples[i].state.position.z,
                     second.samples[i].state.position.z);
    EXPECT_DOUBLE_EQ(first.samples[i].state.orientation.x,
                     second.samples[i].state.orientation.x);
    EXPECT_DOUBLE_EQ(first.samples[i].state.orientation.y,
                     second.samples[i].state.orientation.y);
    EXPECT_DOUBLE_EQ(first.samples[i].state.orientation.z,
                     second.samples[i].state.orientation.z);
    EXPECT_DOUBLE_EQ(first.samples[i].state.orientation.w,
                     second.samples[i].state.orientation.w);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.linear_x,
                     second.samples[i].state.twist.linear_x);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.linear_y,
                     second.samples[i].state.twist.linear_y);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.linear_z,
                     second.samples[i].state.twist.linear_z);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.angular_x,
                     second.samples[i].state.twist.angular_x);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.angular_y,
                     second.samples[i].state.twist.angular_y);
    EXPECT_DOUBLE_EQ(first.samples[i].state.twist.angular_z,
                     second.samples[i].state.twist.angular_z);
  }
}

TEST(VehiclePrimitivePropagationTest, RemainderStepLandsOnDuration) {
  const ZeroForceDynamics dynamics;
  PropagationConfig config = NominalConfig();
  config.dt_s = 0.5;
  config.max_steps = 4;
  const PropagationResult result = PropagateUuvMotionPrimitive(
      Origin(), SurgePrimitive(0.75), config, dynamics);
  ASSERT_EQ(result.error, PropagationError::kOk);
  ASSERT_EQ(result.samples.size(), 3u);
  EXPECT_DOUBLE_EQ(result.samples[1].time_s, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.twist.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.samples[1].state.position.x, 0.25);
  EXPECT_DOUBLE_EQ(result.samples[2].time_s, 0.75);
  EXPECT_DOUBLE_EQ(result.samples[2].state.twist.linear_x, 0.75);
  EXPECT_DOUBLE_EQ(result.samples[2].state.position.x, 0.4375);
  EXPECT_NEAR(result.samples[2].time_s, 0.75, 1e-12);
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
