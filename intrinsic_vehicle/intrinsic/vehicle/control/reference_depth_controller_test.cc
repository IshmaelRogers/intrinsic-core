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

#include "intrinsic/vehicle/control/reference_depth_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"

namespace {

using intrinsic::vehicle::control::BodyWrenchRt;
using intrinsic::vehicle::control::ControlStatusCode;
using intrinsic::vehicle::control::Duration;
using intrinsic::vehicle::control::FrameId;
using intrinsic::vehicle::control::MotionReferenceRt;
using intrinsic::vehicle::control::ReferenceDepthController;
using intrinsic::vehicle::control::StatusOr;
using intrinsic::vehicle::control::VehicleStateRt;
using intrinsic::vehicle::guidance::kHeave;

constexpr double kDt = 0.1;
constexpr double kCommandDepthM = 12.0;
constexpr double kInitialDepthM = 10.0;
constexpr double kStepM = 2.0;
constexpr double kMassKg = 60.0;
constexpr double kDamping = 3.0;
constexpr double kOvershootLimit = 0.15;
constexpr double kSettleLimitS = 30.0;
constexpr double kBandM = 0.05;

VehicleStateRt StateAt(double depth_m) {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  state.position_m = {0, 0, -depth_m};
  state.snapshot_id = 11;
  return state;
}

MotionReferenceRt DepthCommand(double depth_m) {
  MotionReferenceRt reference;
  reference.snapshot_id = 11;
  reference.update_period = Duration{kDt};
  reference.has_pose = true;
  reference.pose_frame = FrameId::kWorldEnu;
  reference.position_m = {0, 0, -depth_m};
  reference.orientation_xyzw = {0, 0, 0, 1};
  return reference;
}

Duration Period() { return Duration{kDt}; }

void ExpectHeaveOnly(const BodyWrenchRt& wrench, double heave_n) {
  EXPECT_EQ(wrench.frame, FrameId::kBody);
  EXPECT_DOUBLE_EQ(wrench.force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[2], heave_n);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[2], 0.0);
}

void ExpectFiniteZero(const BodyWrenchRt& wrench) {
  ExpectHeaveOnly(wrench, 0.0);
  const BodyWrenchRt neutral;
  EXPECT_EQ(std::memcmp(&wrench, &neutral, sizeof(neutral)), 0);
}

TEST(ReferenceDepthController, DeepenAtRestCommandsNegativeHeave) {
  ReferenceDepthController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectHeaveOnly(result.value(), -40.0);
  EXPECT_NEAR(controller.integrator_state(), -0.04, 1e-12);
}

TEST(ReferenceDepthController, IntegralForceChangesTheNextCommand) {
  ReferenceDepthController controller;
  ASSERT_TRUE(
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period()).ok());
  const double integral_force = controller.integrator_state();
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period());
  ASSERT_TRUE(second.ok());
  ExpectHeaveOnly(second.value(), -40.0 + integral_force);
  EXPECT_NEAR(controller.integrator_state(), integral_force - 0.04, 1e-12);
}

TEST(ReferenceDepthController, DerivativeOpposesPositiveDownRate) {
  ReferenceDepthController controller;
  VehicleStateRt state = StateAt(10.0);
  state.body_twist[kHeave] = -1.0;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(DepthCommand(10.0), state, Period());
  ASSERT_TRUE(result.ok());
  ExpectHeaveOnly(result.value(), 45.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceDepthController, RolledBodyHeaveChangesTheDepthRate) {
  ReferenceDepthController level;
  ReferenceDepthController rolled;
  VehicleStateRt identity = StateAt(10.0);
  identity.body_twist[kHeave] = 1.0;
  VehicleStateRt inverted = identity;
  inverted.orientation_xyzw = {1, 0, 0, 0};
  const StatusOr<BodyWrenchRt> up =
      level.Evaluate(DepthCommand(10.0), identity, Period());
  const StatusOr<BodyWrenchRt> down =
      rolled.Evaluate(DepthCommand(10.0), inverted, Period());
  ASSERT_TRUE(up.ok());
  ASSERT_TRUE(down.ok());
  ExpectHeaveOnly(up.value(), -45.0);
  ExpectHeaveOnly(down.value(), 45.0);
}

TEST(ReferenceDepthController, AntiWindupFeedsTheIntegrator) {
  ReferenceDepthController controller;
  // Proportional action alone is -60 N, past the -50 N heave rail.
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(DepthCommand(3.0), StateAt(0.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectHeaveOnly(result.value(), -50.0);
  const double pure_integral = -0.2 * 3.0 * kDt;
  const double u_unsat = -20.0 * 3.0;
  const double tracking = 0.2 * (-50.0 - u_unsat);
  const double expected = (-0.2 * 3.0 + tracking) * kDt;
  EXPECT_NEAR(controller.integrator_state(), expected, 1e-9);
  EXPECT_GT(controller.integrator_state(), pure_integral);
}

TEST(ReferenceDepthController, SaturatedHoldStaysInsideTheClamp) {
  ReferenceDepthController controller;
  StatusOr<BodyWrenchRt> result = StatusOr<BodyWrenchRt>::Failure(
      intrinsic::vehicle::control::ControlStatus{});
  for (int i = 0; i < 20; ++i) {
    result = controller.Evaluate(DepthCommand(100.0), StateAt(0.0), Period());
    ASSERT_TRUE(result.ok());
    EXPECT_DOUBLE_EQ(result.value().force_n[2], -50.0);
    EXPECT_LE(controller.integrator_state(), 40.0);
    EXPECT_GE(controller.integrator_state(), -40.0);
  }
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 40.0);
}

TEST(ReferenceDepthController, RejectedInputDoesNotWriteTheIntegrator) {
  ReferenceDepthController controller;
  ASSERT_TRUE(
      controller.Evaluate(DepthCommand(100.0), StateAt(0.0), Period()).ok());
  const double held = controller.integrator_state();
  ASSERT_NE(held, 0.0);

  VehicleStateRt stale = StateAt(0.0);
  stale.snapshot_id = 12;
  const StatusOr<BodyWrenchRt> stale_result =
      controller.Evaluate(DepthCommand(100.0), stale, Period());
  ASSERT_FALSE(stale_result.ok());
  EXPECT_EQ(stale_result.status().code, ControlStatusCode::kStale);
  EXPECT_EQ(stale_result.status().message,
            std::string_view(
                "snapshot_id does not match the vehicle state snapshot"));
  ExpectFiniteZero(stale_result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  VehicleStateRt non_finite = StateAt(0.0);
  non_finite.position_m[2] = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<BodyWrenchRt> nan_result =
      controller.Evaluate(DepthCommand(100.0), non_finite, Period());
  ASSERT_FALSE(nan_result.ok());
  EXPECT_EQ(nan_result.status().code, ControlStatusCode::kInvalidArgument);
  ExpectFiniteZero(nan_result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  const StatusOr<BodyWrenchRt> negative_dt =
      controller.Evaluate(DepthCommand(100.0), StateAt(0.0), Duration{-0.1});
  ASSERT_FALSE(negative_dt.ok());
  EXPECT_EQ(negative_dt.status().code, ControlStatusCode::kInvalidArgument);
  ExpectFiniteZero(negative_dt.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceDepthController, ResetClearsTheIntegralForce) {
  ReferenceDepthController controller;
  ASSERT_TRUE(
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period()).ok());
  ASSERT_NE(controller.integrator_state(), 0.0);
  controller.Reset();
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  ReferenceDepthController fresh;
  const StatusOr<BodyWrenchRt> restored =
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period());
  const StatusOr<BodyWrenchRt> first =
      fresh.Evaluate(DepthCommand(12.0), StateAt(10.0), Period());
  ASSERT_TRUE(restored.ok());
  ASSERT_TRUE(first.ok());
  EXPECT_EQ(
      std::memcmp(&restored.value(), &first.value(), sizeof(first.value())), 0);
}

TEST(ReferenceDepthController, TwistOnlyReferenceMissesTheDepthObjective) {
  ReferenceDepthController controller;
  ASSERT_TRUE(
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Period()).ok());
  const double held = controller.integrator_state();
  MotionReferenceRt twist;
  twist.snapshot_id = 11;
  twist.update_period = Duration{kDt};
  twist.has_twist = true;
  twist.twist_frame = FrameId::kBody;
  twist.body_twist = {1.0, 0, 0, 0, 0, 0.4};
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(twist, StateAt(10.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no depth objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceDepthController, EmptyReferenceIsMissingBeforeDepthChecks) {
  const ReferenceDepthController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(MotionReferenceRt{}, StateAt(10.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no pose or twist objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceDepthController, NedFramesAreInvalid) {
  ReferenceDepthController controller;
  VehicleStateRt state = StateAt(10.0);
  state.pose_frame = FrameId::kWorldNed;
  const StatusOr<BodyWrenchRt> state_ned =
      controller.Evaluate(DepthCommand(12.0), state, Period());
  ASSERT_FALSE(state_ned.ok());
  EXPECT_EQ(state_ned.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(state_ned.status().message,
            std::string_view("state pose frame must be world_enu"));
  ExpectFiniteZero(state_ned.value());

  MotionReferenceRt reference = DepthCommand(12.0);
  reference.pose_frame = FrameId::kWorldNed;
  const StatusOr<BodyWrenchRt> reference_ned =
      controller.Evaluate(reference, StateAt(10.0), Period());
  ASSERT_FALSE(reference_ned.ok());
  EXPECT_EQ(reference_ned.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(reference_ned.status().message,
            std::string_view("reference pose frame must be world_enu"));
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceDepthController, ZeroTimeStepDoesNotIntegrate) {
  ReferenceDepthController controller;
  const StatusOr<BodyWrenchRt> first =
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Duration{0});
  ASSERT_TRUE(first.ok());
  ExpectHeaveOnly(first.value(), -40.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(DepthCommand(12.0), StateAt(10.0), Duration{0});
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(std::memcmp(&first.value(), &second.value(), sizeof(first.value())),
            0);
}

TEST(ReferenceDepthController, SurgeYawAndHorizontalPositionAreUnused) {
  ReferenceDepthController plain;
  ReferenceDepthController distracted;
  const StatusOr<BodyWrenchRt> heave_only =
      plain.Evaluate(DepthCommand(12.0), StateAt(10.0), Period());
  VehicleStateRt state = StateAt(10.0);
  state.position_m[0] = 40.0;
  state.position_m[1] = -7.0;
  state.body_twist[0] = 1.5;
  state.body_twist[5] = -0.3;
  MotionReferenceRt reference = DepthCommand(12.0);
  reference.position_m[0] = -12.0;
  reference.position_m[1] = 4.0;
  reference.orientation_xyzw = {0, 0, 1, 0};
  reference.has_twist = true;
  reference.twist_frame = FrameId::kBody;
  reference.body_twist = {2.0, 0, 0, 0, 0, 1.0};
  const StatusOr<BodyWrenchRt> other =
      distracted.Evaluate(reference, state, Period());
  ASSERT_TRUE(heave_only.ok());
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(
      std::memcmp(&heave_only.value(), &other.value(), sizeof(other.value())),
      0);
  ExpectHeaveOnly(other.value(), -40.0);
}

TEST(ReferenceDepthController, StepResponseMeetsApprovedBounds) {
  // Point-mass heave, semi-implicit Euler. m = 60 kg, b = 3 N*s/m,
  // dt = 0.1 s. The depth command steps from 10 m to 12 m. The vehicle
  // stays level, so body heave is the world +z force. The expects at
  // the end are the approved 15% and 30 s bounds.
  constexpr double kHorizonS = 120.0;
  const int steps = static_cast<int>(std::lround(kHorizonS / kDt));
  ReferenceDepthController controller;
  ReferenceDepthController repeat;
  double z_m = -kInitialDepthM;
  double v_up = 0.0;
  double repeat_z = z_m;
  double repeat_v = v_up;
  std::vector<double> depths;
  depths.reserve(steps);
  int stray_axis = 0;

  for (int i = 0; i < steps; ++i) {
    const double depth_m = -z_m;
    depths.push_back(depth_m);
    VehicleStateRt state = StateAt(depth_m);
    state.body_twist[kHeave] = v_up;
    const StatusOr<BodyWrenchRt> result =
        controller.Evaluate(DepthCommand(kCommandDepthM), state, Period());
    ASSERT_TRUE(result.ok()) << i * kDt;
    const BodyWrenchRt& wrench = result.value();
    if (wrench.force_n[0] != 0.0 || wrench.force_n[1] != 0.0 ||
        wrench.torque_n_m[0] != 0.0 || wrench.torque_n_m[1] != 0.0 ||
        wrench.torque_n_m[2] != 0.0) {
      ++stray_axis;
    }
    const double heave = wrench.force_n[2];
    EXPECT_LE(heave, 50.0);
    EXPECT_GE(heave, -50.0);

    VehicleStateRt repeat_state = StateAt(-repeat_z);
    repeat_state.body_twist[kHeave] = repeat_v;
    const StatusOr<BodyWrenchRt> again =
        repeat.Evaluate(DepthCommand(kCommandDepthM), repeat_state, Period());
    ASSERT_TRUE(again.ok());
    EXPECT_DOUBLE_EQ(again.value().force_n[2], heave);

    const double acceleration = (heave - kDamping * v_up) / kMassKg;
    v_up += acceleration * kDt;
    z_m += v_up * kDt;
    const double repeat_acceleration = (heave - kDamping * repeat_v) / kMassKg;
    repeat_v += repeat_acceleration * kDt;
    repeat_z += repeat_v * kDt;
    EXPECT_DOUBLE_EQ(repeat_z, z_m);
  }

  double peak_depth = depths.front();
  for (double depth : depths) {
    peak_depth = std::max(peak_depth, depth);
  }
  int settle_index = steps;
  for (int i = steps - 1; i >= 0; --i) {
    if (std::abs(depths[i] - kCommandDepthM) > kBandM) {
      break;
    }
    settle_index = i;
  }
  const bool stayed = settle_index < steps;
  const double settle_s = stayed ? settle_index * kDt : kHorizonS;
  const double overshoot_fraction = (peak_depth - kCommandDepthM) / kStepM;
  EXPECT_EQ(stray_axis, 0);
  EXPECT_LE(overshoot_fraction, kOvershootLimit)
      << "peak depth " << peak_depth << " m";
  EXPECT_TRUE(stayed) << "final depth " << depths.back()
                      << " m did not remain inside +/- " << kBandM << " m";
  EXPECT_LE(settle_s, kSettleLimitS) << "settle " << settle_s << " s";
}

}  // namespace
