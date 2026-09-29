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

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

#include <array>
#include <cmath>
#include <limits>
#include <string_view>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"

namespace {

using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::DynamicsResult;
using intrinsic::vehicle::dynamics::DynamicsStatus;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::FrameId;
using intrinsic::vehicle::dynamics::kHeave;
using intrinsic::vehicle::dynamics::kPitch;
using intrinsic::vehicle::dynamics::kRoll;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kSurge;
using intrinsic::vehicle::dynamics::kSway;
using intrinsic::vehicle::dynamics::kX;
using intrinsic::vehicle::dynamics::kY;
using intrinsic::vehicle::dynamics::kYaw;
using intrinsic::vehicle::dynamics::kZ;
using intrinsic::vehicle::dynamics::kZeroForceModelId;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::ValidateEvaluationInputs;
using intrinsic::vehicle::dynamics::VehicleDynamics;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::dynamics::ZeroForceDynamics;

constexpr std::array<double, 3> kZero3 = {0, 0, 0};
constexpr std::array<double, 4> kZero4 = {0, 0, 0, 0};
constexpr std::array<double, 6> kZero6 = {0, 0, 0, 0, 0, 0};

VehicleStateRt ValidState() {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  return state;
}

BodyWrenchRt ValidWrench() { return {}; }

EnvironmentRt ValidEnvironment() {
  EnvironmentRt environment;
  environment.current_frame = FrameId::kWorldEnu;
  return environment;
}

Duration ValidDt() { return Duration{0.25}; }

void ExpectRejectedZero(const DynamicsResult& result) {
  EXPECT_EQ(result.derivative.position_dot_m_s, kZero3);
  EXPECT_EQ(result.derivative.orientation_dot_xyzw, kZero4);
  EXPECT_EQ(result.derivative.body_acceleration, kZero6);
  EXPECT_EQ(result.diagnostics.model_force_n, kZero3);
  EXPECT_EQ(result.diagnostics.model_torque_n_m, kZero3);
  EXPECT_EQ(result.diagnostics.relative_twist, kZero6);
  EXPECT_EQ(result.diagnostics.rigid_body_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.added_mass_coriolis_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.damping_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.restoring_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.hydrodynamic_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.total_wrench, kZero6);
  EXPECT_EQ(result.diagnostics.dt_s, 0);
  EXPECT_EQ(result.diagnostics.model_id, kZeroForceModelId);
  EXPECT_FALSE(result.diagnostics.input_wrench_used);
  EXPECT_FALSE(result.diagnostics.allocation_invoked);
  for (double value : result.derivative.position_dot_m_s) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (double value : result.derivative.orientation_dot_xyzw) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (double value : result.derivative.body_acceleration) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

void ExpectRejected(const char* name, std::string_view message,
                    const VehicleStateRt& state, const BodyWrenchRt& wrench,
                    const EnvironmentRt& environment, Duration dt) {
  const DynamicsStatus direct =
      ValidateEvaluationInputs(state, wrench, environment, dt);
  const ZeroForceDynamics model;
  const StatusOr<DynamicsResult> evaluated =
      model.Evaluate(state, wrench, environment, dt);
  EXPECT_FALSE(direct.ok()) << name;
  EXPECT_EQ(direct.code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(direct.message, message) << name;
  ASSERT_FALSE(evaluated.ok()) << name;
  EXPECT_EQ(evaluated.status().code, DynamicsErrorCode::kInvalidArgument)
      << name;
  EXPECT_EQ(evaluated.status().message, message) << name;
  ExpectRejectedZero(evaluated.value());
}

class StampDynamics : public VehicleDynamics {
 public:
  [[nodiscard]] StatusOr<DynamicsResult> Evaluate(
      const VehicleStateRt& state, const BodyWrenchRt& wrench,
      const EnvironmentRt& environment, Duration dt) const override {
    (void)state;
    (void)wrench;
    (void)environment;
    DynamicsResult result;
    result.diagnostics.model_id = "stamp";
    result.diagnostics.dt_s = dt.seconds;
    result.derivative.body_acceleration[kHeave] = 4;
    return StatusOr<DynamicsResult>::Ok(result);
  }
};

TEST(VehicleDynamicsInterface, TypesAreFixedSizeAndAbstract) {
  static_assert(std::is_abstract_v<VehicleDynamics>);
  static_assert(std::is_final_v<ZeroForceDynamics>);
  // The vtable pointer is the only storage. The double adds no members.
  static_assert(sizeof(ZeroForceDynamics) == sizeof(VehicleDynamics));
  static_assert(std::is_trivially_copyable_v<VehicleStateRt>);
  static_assert(std::is_trivially_copyable_v<BodyWrenchRt>);
  static_assert(std::is_trivially_copyable_v<EnvironmentRt>);
  static_assert(std::is_trivially_copyable_v<Duration>);
  static_assert(std::is_trivially_copyable_v<DynamicsResult>);
  static_assert(std::is_trivially_copyable_v<StatusOr<DynamicsResult>>);
  static_assert(static_cast<int>(FrameId::kUnspecified) == 0);
  static_assert(static_cast<int>(FrameId::kWorldEnu) == 1);
  static_assert(static_cast<int>(FrameId::kWorldNed) == 2);
  static_assert(static_cast<int>(FrameId::kBody) == 3);
  static_assert(kSurge == 0);
  static_assert(kYaw == 5);
  static_assert(kSpatialDof == 6);
  EXPECT_TRUE(DynamicsStatus::Ok().ok());
}

TEST(VehicleDynamicsInterface, PolymorphicEvaluateReturnsDerivative) {
  const ZeroForceDynamics concrete;
  const VehicleDynamics& model = concrete;
  VehicleStateRt state = ValidState();
  state.body_twist[kSurge] = 1;
  const StatusOr<DynamicsResult> result =
      model.Evaluate(state, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());
  EXPECT_EQ(result.value().derivative.position_dot_m_s,
            (std::array<double, 3>{1, 0, 0}));
  EXPECT_EQ(result.value().derivative.orientation_dot_xyzw, kZero4);
  EXPECT_EQ(result.value().derivative.body_acceleration, kZero6);
  EXPECT_EQ(result.value().diagnostics.model_id, kZeroForceModelId);
  EXPECT_EQ(result.value().diagnostics.dt_s, 0.25);
  EXPECT_EQ(result.value().diagnostics.model_force_n, kZero3);
  EXPECT_EQ(result.value().diagnostics.model_torque_n_m, kZero3);
  EXPECT_EQ(result.value().diagnostics.relative_twist, kZero6);
  EXPECT_EQ(result.value().diagnostics.hydrodynamic_wrench, kZero6);
  EXPECT_EQ(result.value().diagnostics.total_wrench, kZero6);
  EXPECT_FALSE(result.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(result.value().diagnostics.allocation_invoked);
}

TEST(VehicleDynamicsInterface, AlternateImplementationOverridesEvaluate) {
  const StampDynamics concrete;
  const VehicleDynamics& model = concrete;
  const StatusOr<DynamicsResult> result = model.Evaluate(
      ValidState(), ValidWrench(), ValidEnvironment(), Duration{0.5});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().diagnostics.model_id, std::string_view("stamp"));
  EXPECT_EQ(result.value().diagnostics.dt_s, 0.5);
  EXPECT_EQ(result.value().derivative.body_acceleration[kHeave], 4);
  EXPECT_FALSE(result.value().diagnostics.allocation_invoked);
}

TEST(ZeroForceDynamics, RestWithZeroLoadsIsZero) {
  const ZeroForceDynamics model;
  const StatusOr<DynamicsResult> result = model.Evaluate(
      ValidState(), ValidWrench(), ValidEnvironment(), Duration{0});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().derivative.position_dot_m_s, kZero3);
  EXPECT_EQ(result.value().derivative.orientation_dot_xyzw, kZero4);
  EXPECT_EQ(result.value().derivative.body_acceleration, kZero6);
  EXPECT_EQ(result.value().diagnostics.dt_s, 0);
  EXPECT_FALSE(result.value().diagnostics.allocation_invoked);
}

TEST(ZeroForceDynamics, IdentityMapsBodyVelocityToPoseFrame) {
  const ZeroForceDynamics model;
  VehicleStateRt state = ValidState();
  state.body_twist[kSurge] = 0.25;
  state.body_twist[kSway] = -0.5;
  state.body_twist[kHeave] = 0.125;
  const StatusOr<DynamicsResult> result =
      model.Evaluate(state, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().derivative.position_dot_m_s,
            (std::array<double, 3>{0.25, -0.5, 0.125}));
  EXPECT_EQ(result.value().derivative.orientation_dot_xyzw, kZero4);
  EXPECT_EQ(result.value().derivative.body_acceleration, kZero6);
}

TEST(ZeroForceDynamics, RotatesBodyVelocityAndDifferentiatesQuaternion) {
  const ZeroForceDynamics model;
  VehicleStateRt turned = ValidState();
  // 180 degrees about z. q = (x, y, z, w) = (0, 0, 1, 0).
  turned.orientation_xyzw = {0, 0, 1, 0};
  turned.body_twist[kSurge] = 1;
  turned.body_twist[kYaw] = 2;
  const StatusOr<DynamicsResult> yawed =
      model.Evaluate(turned, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(yawed.ok());
  EXPECT_EQ(yawed.value().derivative.position_dot_m_s,
            (std::array<double, 3>{-1, 0, 0}));
  // q ⊗ ω / 2 with ω = (0, 0, 2) and q = (0, 0, 1, 0) is (0, 0, 0, -1).
  EXPECT_EQ(yawed.value().derivative.orientation_dot_xyzw,
            (std::array<double, 4>{0, 0, 0, -1}));

  VehicleStateRt rolled = ValidState();
  // 180 degrees about x. q = (1, 0, 0, 0).
  rolled.orientation_xyzw = {1, 0, 0, 0};
  rolled.body_twist[kSway] = 1;
  rolled.body_twist[kRoll] = 2;
  const StatusOr<DynamicsResult> roll =
      model.Evaluate(rolled, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(roll.ok());
  EXPECT_EQ(roll.value().derivative.position_dot_m_s,
            (std::array<double, 3>{0, -1, 0}));
  EXPECT_EQ(roll.value().derivative.orientation_dot_xyzw,
            (std::array<double, 4>{0, 0, 0, -1}));
}

TEST(ZeroForceDynamics, OriginVelocityAndAttitudeRateStayDecoupled) {
  const ZeroForceDynamics model;
  VehicleStateRt spinning = ValidState();
  spinning.body_twist[kRoll] = 2;
  spinning.body_twist[kPitch] = 4;
  spinning.body_twist[kYaw] = 6;
  const StatusOr<DynamicsResult> spin =
      model.Evaluate(spinning, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(spin.ok());
  EXPECT_EQ(spin.value().derivative.position_dot_m_s, kZero3);
  EXPECT_EQ(spin.value().derivative.orientation_dot_xyzw,
            (std::array<double, 4>{1, 2, 3, 0}));
  EXPECT_EQ(spin.value().derivative.body_acceleration, kZero6);
}

TEST(ZeroForceDynamics, IgnoresWrenchGravityCurrentAndPosition) {
  const ZeroForceDynamics model;
  VehicleStateRt calm = ValidState();
  calm.position_m = {10, -4, 2};
  calm.body_twist[kSurge] = 1;
  EnvironmentRt environment = ValidEnvironment();
  environment.gravity_m_s2 = 9.80665;
  environment.fluid_density_kg_m3 = 1025;
  environment.current_velocity_m_s = {3, 0, 0};
  environment.current_frame = FrameId::kBody;
  BodyWrenchRt wrench = ValidWrench();
  wrench.force_n = {40, -5, 2};
  wrench.torque_n_m = {0, 7, -1};
  const StatusOr<DynamicsResult> loaded =
      model.Evaluate(calm, wrench, environment, Duration{0.125});

  VehicleStateRt shifted = calm;
  shifted.position_m = {-8, 1, 0};
  shifted.pose_frame = FrameId::kWorldNed;
  const StatusOr<DynamicsResult> plain =
      model.Evaluate(shifted, ValidWrench(), ValidEnvironment(), ValidDt());
  ASSERT_TRUE(loaded.ok());
  ASSERT_TRUE(plain.ok());
  EXPECT_EQ(loaded.value().derivative.position_dot_m_s,
            plain.value().derivative.position_dot_m_s);
  EXPECT_EQ(loaded.value().derivative.orientation_dot_xyzw,
            plain.value().derivative.orientation_dot_xyzw);
  EXPECT_EQ(loaded.value().derivative.body_acceleration, kZero6);
  EXPECT_EQ(loaded.value().diagnostics.model_force_n, kZero3);
  EXPECT_EQ(loaded.value().diagnostics.model_torque_n_m, kZero3);
  EXPECT_FALSE(loaded.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(loaded.value().diagnostics.allocation_invoked);
  EXPECT_EQ(loaded.value().diagnostics.dt_s, 0.125);
  EXPECT_EQ(plain.value().diagnostics.dt_s, 0.25);
}

TEST(ZeroForceDynamics, TimeStepIsRecordedAndNotIntegrated) {
  const ZeroForceDynamics model;
  VehicleStateRt state = ValidState();
  state.body_twist[kSurge] = 2;
  const StatusOr<DynamicsResult> small =
      model.Evaluate(state, ValidWrench(), ValidEnvironment(), Duration{0});
  const StatusOr<DynamicsResult> large =
      model.Evaluate(state, ValidWrench(), ValidEnvironment(), Duration{8});
  ASSERT_TRUE(small.ok());
  ASSERT_TRUE(large.ok());
  EXPECT_EQ(small.value().derivative.position_dot_m_s,
            large.value().derivative.position_dot_m_s);
  EXPECT_EQ(small.value().derivative.position_dot_m_s,
            (std::array<double, 3>{2, 0, 0}));
  EXPECT_EQ(small.value().diagnostics.dt_s, 0);
  EXPECT_EQ(large.value().diagnostics.dt_s, 8);
}

TEST(ZeroForceDynamics, RepeatedEvaluationMatchesBitForBit) {
  const ZeroForceDynamics model;
  VehicleStateRt state = ValidState();
  state.pose_frame = FrameId::kWorldNed;
  state.position_m = {1, 2, 3};
  state.orientation_xyzw = {0, 0, 1, 0};
  state.body_twist = {0.25, -0.5, 0.125, 0.5, 0, 2};
  EnvironmentRt environment = ValidEnvironment();
  environment.gravity_m_s2 = 9.81;
  environment.fluid_density_kg_m3 = 1000;
  environment.current_velocity_m_s = {1, 0, -1};
  environment.current_frame = FrameId::kWorldNed;
  BodyWrenchRt wrench = ValidWrench();
  wrench.force_n = {1, 2, 3};
  const Duration dt{0.02};
  const StatusOr<DynamicsResult> first =
      model.Evaluate(state, wrench, environment, dt);
  ASSERT_TRUE(first.ok());
  for (int i = 0; i < 3; ++i) {
    const StatusOr<DynamicsResult> again =
        model.Evaluate(state, wrench, environment, dt);
    ASSERT_TRUE(again.ok());
    EXPECT_EQ(again.value().derivative.position_dot_m_s,
              first.value().derivative.position_dot_m_s);
    EXPECT_EQ(again.value().derivative.orientation_dot_xyzw,
              first.value().derivative.orientation_dot_xyzw);
    EXPECT_EQ(again.value().derivative.body_acceleration,
              first.value().derivative.body_acceleration);
    EXPECT_EQ(again.value().diagnostics.dt_s, first.value().diagnostics.dt_s);
    EXPECT_EQ(again.value().diagnostics.model_id,
              first.value().diagnostics.model_id);
    EXPECT_EQ(again.value().diagnostics.model_force_n,
              first.value().diagnostics.model_force_n);
    EXPECT_EQ(again.value().diagnostics.model_torque_n_m,
              first.value().diagnostics.model_torque_n_m);
  }
}

TEST(ZeroForceDynamics, ConcurrentEvaluationsStayIndependent) {
  const ZeroForceDynamics model;
  StatusOr<DynamicsResult> forward = StatusOr<DynamicsResult>::Failure(
      DynamicsStatus::InvalidArgument("unset"));
  StatusOr<DynamicsResult> reversed = StatusOr<DynamicsResult>::Failure(
      DynamicsStatus::InvalidArgument("unset"));
  std::thread ahead([&] {
    VehicleStateRt state = ValidState();
    state.body_twist[kSurge] = 1;
    forward =
        model.Evaluate(state, ValidWrench(), ValidEnvironment(), ValidDt());
  });
  std::thread flipped([&] {
    VehicleStateRt state = ValidState();
    state.orientation_xyzw = {1, 0, 0, 0};
    state.body_twist[kSway] = 1;
    reversed =
        model.Evaluate(state, ValidWrench(), ValidEnvironment(), Duration{0.5});
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

TEST(ZeroForceDynamics, RejectsInvalidInputs) {
  ExpectRejected("default state has no pose frame",
                 "state pose frame must be world_enu or world_ned",
                 VehicleStateRt{}, ValidWrench(), ValidEnvironment(),
                 ValidDt());

  VehicleStateRt bad_frame = ValidState();
  bad_frame.pose_frame = FrameId::kBody;
  bad_frame.position_m[kX] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("pose frame precedes finiteness",
                 "state pose frame must be world_enu or world_ned", bad_frame,
                 ValidWrench(), ValidEnvironment(), ValidDt());

  VehicleStateRt nan_position = ValidState();
  nan_position.position_m[kY] = std::numeric_limits<double>::infinity();
  ExpectRejected("non-finite position", "state value must be finite",
                 nan_position, ValidWrench(), ValidEnvironment(), ValidDt());

  VehicleStateRt nan_twist = ValidState();
  nan_twist.body_twist[kYaw] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite twist", "state value must be finite", nan_twist,
                 ValidWrench(), ValidEnvironment(), ValidDt());

  VehicleStateRt stretched = ValidState();
  stretched.orientation_xyzw = {0, 0, 0, 1 + 1e-6};
  ExpectRejected("quaternion outside tolerance",
                 "orientation quaternion must have unit norm", stretched,
                 ValidWrench(), ValidEnvironment(), ValidDt());

  BodyWrenchRt world_wrench = ValidWrench();
  world_wrench.frame = FrameId::kWorldEnu;
  ExpectRejected("wrench frame", "wrench frame must be body", ValidState(),
                 world_wrench, ValidEnvironment(), ValidDt());

  BodyWrenchRt nan_wrench = ValidWrench();
  nan_wrench.torque_n_m[kZ] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite wrench", "wrench value must be finite",
                 ValidState(), nan_wrench, ValidEnvironment(), ValidDt());

  EnvironmentRt nan_gravity = ValidEnvironment();
  nan_gravity.gravity_m_s2 = std::numeric_limits<double>::infinity();
  ExpectRejected("non-finite gravity", "environment value must be finite",
                 ValidState(), ValidWrench(), nan_gravity, ValidDt());

  EnvironmentRt negative_gravity = ValidEnvironment();
  negative_gravity.gravity_m_s2 = -0.1;
  ExpectRejected("negative gravity",
                 "environment gravity must be greater than or equal to zero",
                 ValidState(), ValidWrench(), negative_gravity, ValidDt());

  EnvironmentRt negative_density = ValidEnvironment();
  negative_density.fluid_density_kg_m3 = -1;
  ExpectRejected("negative density",
                 "environment density must be greater than or equal to zero",
                 ValidState(), ValidWrench(), negative_density, ValidDt());

  EnvironmentRt nan_current = ValidEnvironment();
  nan_current.current_velocity_m_s[kX] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite current", "environment value must be finite",
                 ValidState(), ValidWrench(), nan_current, ValidDt());

  EnvironmentRt missing_frame = ValidEnvironment();
  missing_frame.current_frame = FrameId::kUnspecified;
  ExpectRejected(
      "current frame",
      "environment current frame must be world_enu, world_ned, or body",
      ValidState(), ValidWrench(), missing_frame, ValidDt());

  ExpectRejected("negative time step",
                 "time step must be finite and greater than or equal to zero",
                 ValidState(), ValidWrench(), ValidEnvironment(), Duration{-1});
  ExpectRejected("non-finite time step",
                 "time step must be finite and greater than or equal to zero",
                 ValidState(), ValidWrench(), ValidEnvironment(),
                 Duration{std::numeric_limits<double>::quiet_NaN()});
}

TEST(ZeroForceDynamics, AcceptsZeroEnvironmentAndNearUnitQuaternion) {
  const DynamicsStatus status = ValidateEvaluationInputs(
      ValidState(), ValidWrench(), ValidEnvironment(), Duration{0});
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(status.code, DynamicsErrorCode::kOk);

  VehicleStateRt state = ValidState();
  state.orientation_xyzw = {0, 0, 0, -1};
  EnvironmentRt environment = ValidEnvironment();
  environment.current_frame = FrameId::kBody;
  const ZeroForceDynamics model;
  const StatusOr<DynamicsResult> result =
      model.Evaluate(state, ValidWrench(), environment, Duration{0});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().derivative.body_acceleration, kZero6);

  VehicleStateRt near_unit = ValidState();
  near_unit.orientation_xyzw = {0, 0, 0, 1 + 1e-12};
  EXPECT_TRUE(ValidateEvaluationInputs(near_unit, ValidWrench(),
                                       ValidEnvironment(), Duration{0})
                  .ok());
}

}  // namespace
