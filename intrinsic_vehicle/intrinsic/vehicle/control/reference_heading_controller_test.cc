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

#include "intrinsic/vehicle/control/reference_heading_controller.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"

namespace {

using intrinsic::vehicle::control::BodyWrenchRt;
using intrinsic::vehicle::control::ControlStatusCode;
using intrinsic::vehicle::control::Duration;
using intrinsic::vehicle::control::FrameId;
using intrinsic::vehicle::control::MotionReferenceRt;
using intrinsic::vehicle::control::ReferenceHeadingController;
using intrinsic::vehicle::control::StatusOr;
using intrinsic::vehicle::control::VehicleStateRt;
using intrinsic::vehicle::guidance::kHeave;
using intrinsic::vehicle::guidance::kSurge;
using intrinsic::vehicle::guidance::kYaw;

constexpr double kPi = std::numbers::pi;
constexpr double kDt = 0.1;
constexpr double kCommandYawRad = 0.5;
constexpr double kStepRad = 0.5;
constexpr double kInertia = 5.0;
constexpr double kDamping = 1.0;
constexpr double kOvershootLimit = 0.15;
constexpr double kSettleLimitS = 20.0;
constexpr double kBandRad = 0.02;

std::array<double, 4> YawQuaternion(double yaw_rad) {
  return {0, 0, std::sin(yaw_rad / 2.0), std::cos(yaw_rad / 2.0)};
}

VehicleStateRt StateAt(double yaw_rad, double yaw_rate = 0.0) {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = YawQuaternion(yaw_rad);
  state.body_twist[kYaw] = yaw_rate;
  state.snapshot_id = 11;
  return state;
}

MotionReferenceRt HeadingCommand(double yaw_rad) {
  MotionReferenceRt reference;
  reference.snapshot_id = 11;
  reference.update_period = Duration{kDt};
  reference.has_pose = true;
  reference.pose_frame = FrameId::kWorldEnu;
  reference.orientation_xyzw = YawQuaternion(yaw_rad);
  return reference;
}

Duration Period() { return Duration{kDt}; }

void ExpectYawOnly(const BodyWrenchRt& wrench, double yaw_n_m) {
  EXPECT_EQ(wrench.frame, FrameId::kBody);
  EXPECT_DOUBLE_EQ(wrench.force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[2], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[2], yaw_n_m);
}

void ExpectYawNear(const BodyWrenchRt& wrench, double yaw_n_m) {
  EXPECT_EQ(wrench.frame, FrameId::kBody);
  EXPECT_DOUBLE_EQ(wrench.force_n[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[2], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[1], 0.0);
  EXPECT_NEAR(wrench.torque_n_m[2], yaw_n_m, 1e-12);
}

void ExpectFiniteZero(const BodyWrenchRt& wrench) {
  ExpectYawOnly(wrench, 0.0);
  const BodyWrenchRt neutral;
  EXPECT_EQ(std::memcmp(&wrench, &neutral, sizeof(neutral)), 0);
}

TEST(ReferenceHeadingController, PositiveErrorCommandsPositiveYawTorque) {
  ReferenceHeadingController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectYawOnly(result.value(), 5.0);
  EXPECT_NEAR(controller.integrator_state(), 0.015, 1e-12);
}

TEST(ReferenceHeadingController, IntegralTorqueChangesTheNextCommand) {
  ReferenceHeadingController controller;
  ASSERT_TRUE(
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period()).ok());
  const double integral_torque = controller.integrator_state();
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(second.ok());
  ExpectYawOnly(second.value(), 5.0 + integral_torque);
  EXPECT_NEAR(controller.integrator_state(), integral_torque + 0.015, 1e-12);
}

TEST(ReferenceHeadingController, DerivativeOpposesPositiveYawRate) {
  ReferenceHeadingController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(HeadingCommand(0.2), StateAt(0.2, 1.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectYawOnly(result.value(), -8.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceHeadingController, AntiWindupFeedsTheIntegrator) {
  ReferenceHeadingController controller;
  // Proportional action alone is 10*pi N*m, past the +15 N*m rail.
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(HeadingCommand(kPi), StateAt(0.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectYawOnly(result.value(), 15.0);
  const double pure_integral = 0.3 * kPi * kDt;
  const double u_unsat = 10.0 * kPi;
  const double tracking = 0.2 * (15.0 - u_unsat);
  const double expected = (0.3 * kPi + tracking) * kDt;
  EXPECT_NEAR(controller.integrator_state(), expected, 1e-9);
  EXPECT_LT(controller.integrator_state(), pure_integral);
}

TEST(ReferenceHeadingController, SaturatedHoldStaysInsideTheClamp) {
  ReferenceHeadingController controller;
  StatusOr<BodyWrenchRt> result = StatusOr<BodyWrenchRt>::Failure(
      intrinsic::vehicle::control::ControlStatus{});
  for (int i = 0; i < 500; ++i) {
    result = controller.Evaluate(HeadingCommand(kPi), StateAt(0.0), Period());
    ASSERT_TRUE(result.ok());
    EXPECT_DOUBLE_EQ(result.value().torque_n_m[2], 15.0);
    EXPECT_LE(controller.integrator_state(), 12.0);
    EXPECT_GE(controller.integrator_state(), -12.0);
  }
  // |e| <= pi, so the saturated equilibrium lies inside [-12, 12].
  const double equilibrium = 15.0 - 8.5 * kPi;
  EXPECT_NEAR(controller.integrator_state(), equilibrium, 1e-2);
  EXPECT_GT(controller.integrator_state(), -12.0);
}

TEST(ReferenceHeadingController, RejectedInputDoesNotWriteTheIntegrator) {
  ReferenceHeadingController controller;
  ASSERT_TRUE(
      controller.Evaluate(HeadingCommand(kPi), StateAt(0.0), Period()).ok());
  const double held = controller.integrator_state();
  ASSERT_NE(held, 0.0);

  VehicleStateRt stale = StateAt(0.0);
  stale.snapshot_id = 12;
  const StatusOr<BodyWrenchRt> stale_result =
      controller.Evaluate(HeadingCommand(kPi), stale, Period());
  ASSERT_FALSE(stale_result.ok());
  EXPECT_EQ(stale_result.status().code, ControlStatusCode::kStale);
  EXPECT_EQ(stale_result.status().message,
            std::string_view(
                "snapshot_id does not match the vehicle state snapshot"));
  ExpectFiniteZero(stale_result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  VehicleStateRt non_finite = StateAt(0.0);
  non_finite.orientation_xyzw[0] = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<BodyWrenchRt> nan_result =
      controller.Evaluate(HeadingCommand(kPi), non_finite, Period());
  ASSERT_FALSE(nan_result.ok());
  EXPECT_EQ(nan_result.status().code, ControlStatusCode::kInvalidArgument);
  ExpectFiniteZero(nan_result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  VehicleStateRt non_unit = StateAt(0.0);
  non_unit.orientation_xyzw = {0.2, 0.2, 0.2, 0.2};
  const StatusOr<BodyWrenchRt> bad_attitude =
      controller.Evaluate(HeadingCommand(kPi), non_unit, Period());
  ASSERT_FALSE(bad_attitude.ok());
  EXPECT_EQ(bad_attitude.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(
      bad_attitude.status().message,
      std::string_view("state orientation quaternion must have unit norm"));
  ExpectFiniteZero(bad_attitude.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  const StatusOr<BodyWrenchRt> negative_dt =
      controller.Evaluate(HeadingCommand(kPi), StateAt(0.0), Duration{-0.1});
  ASSERT_FALSE(negative_dt.ok());
  EXPECT_EQ(negative_dt.status().code, ControlStatusCode::kInvalidArgument);
  ExpectFiniteZero(negative_dt.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceHeadingController, ResetClearsTheIntegralTorque) {
  ReferenceHeadingController controller;
  ASSERT_TRUE(
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period()).ok());
  ASSERT_NE(controller.integrator_state(), 0.0);
  controller.Reset();
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  ReferenceHeadingController fresh;
  const StatusOr<BodyWrenchRt> restored =
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period());
  const StatusOr<BodyWrenchRt> first =
      fresh.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(restored.ok());
  ASSERT_TRUE(first.ok());
  EXPECT_EQ(
      std::memcmp(&restored.value(), &first.value(), sizeof(first.value())), 0);
}

TEST(ReferenceHeadingController, TwistOnlyReferenceMissesTheYawObjective) {
  ReferenceHeadingController controller;
  ASSERT_TRUE(
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Period()).ok());
  const double held = controller.integrator_state();
  MotionReferenceRt twist;
  twist.snapshot_id = 11;
  twist.update_period = Duration{kDt};
  twist.has_twist = true;
  twist.twist_frame = FrameId::kBody;
  twist.body_twist = {1.0, 0, 0, 0, 0, 0.4};
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(twist, StateAt(0.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no yaw objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceHeadingController, EmptyReferenceIsMissingBeforeYawChecks) {
  const ReferenceHeadingController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(MotionReferenceRt{}, StateAt(0.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no pose or twist objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceHeadingController, NedFramesAreInvalid) {
  ReferenceHeadingController controller;
  VehicleStateRt state = StateAt(0.0);
  state.pose_frame = FrameId::kWorldNed;
  const StatusOr<BodyWrenchRt> state_ned =
      controller.Evaluate(HeadingCommand(0.5), state, Period());
  ASSERT_FALSE(state_ned.ok());
  EXPECT_EQ(state_ned.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(state_ned.status().message,
            std::string_view("state pose frame must be world_enu"));
  ExpectFiniteZero(state_ned.value());

  MotionReferenceRt reference = HeadingCommand(0.5);
  reference.pose_frame = FrameId::kWorldNed;
  const StatusOr<BodyWrenchRt> reference_ned =
      controller.Evaluate(reference, StateAt(0.0), Period());
  ASSERT_FALSE(reference_ned.ok());
  EXPECT_EQ(reference_ned.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(reference_ned.status().message,
            std::string_view("reference pose frame must be world_enu"));
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceHeadingController, ZeroTimeStepDoesNotIntegrate) {
  ReferenceHeadingController controller;
  const StatusOr<BodyWrenchRt> first =
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Duration{0});
  ASSERT_TRUE(first.ok());
  ExpectYawOnly(first.value(), 5.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(HeadingCommand(0.5), StateAt(0.0), Duration{0});
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(std::memcmp(&first.value(), &second.value(), sizeof(first.value())),
            0);
}

TEST(ReferenceHeadingController, DepthAndForwardSpeedAreUnused) {
  ReferenceHeadingController plain;
  ReferenceHeadingController distracted;
  const StatusOr<BodyWrenchRt> yaw_only =
      plain.Evaluate(HeadingCommand(0.5), StateAt(0.0, 0.25), Period());
  VehicleStateRt state = StateAt(0.0, 0.25);
  state.position_m = {40.0, -7.0, -12.0};
  state.body_twist[kSurge] = 1.5;
  state.body_twist[kHeave] = -0.4;
  state.body_twist[3] = 0.2;
  state.body_twist[4] = -0.3;
  MotionReferenceRt reference = HeadingCommand(0.5);
  reference.position_m = {-12.0, 4.0, -30.0};
  reference.has_twist = true;
  reference.twist_frame = FrameId::kBody;
  reference.body_twist = {2.0, 0.1, -0.5, 0, 0, 0.9};
  const StatusOr<BodyWrenchRt> other =
      distracted.Evaluate(reference, state, Period());
  ASSERT_TRUE(yaw_only.ok());
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(
      std::memcmp(&yaw_only.value(), &other.value(), sizeof(other.value())), 0);
  ExpectYawOnly(other.value(), 5.0 - 8.0 * 0.25);
  EXPECT_DOUBLE_EQ(other.value().force_n[kSurge], 0.0);
  EXPECT_DOUBLE_EQ(other.value().force_n[kHeave], 0.0);
}

TEST(ReferenceHeadingController, ShortestDirectionAcrossPlusMinusPi) {
  ReferenceHeadingController across;
  // Measured just below +pi, command just above -pi. The short error is
  // +0.1 rad. The long way is about -2*pi and would saturate at -15 N*m.
  const StatusOr<BodyWrenchRt> short_positive = across.Evaluate(
      HeadingCommand(-kPi + 0.05), StateAt(kPi - 0.05), Period());
  ASSERT_TRUE(short_positive.ok());
  ExpectYawNear(short_positive.value(), 1.0);
  EXPECT_LT(std::abs(short_positive.value().torque_n_m[2]), 15.0);

  ReferenceHeadingController back;
  const StatusOr<BodyWrenchRt> short_negative =
      back.Evaluate(HeadingCommand(kPi - 0.05), StateAt(-kPi + 0.05), Period());
  ASSERT_TRUE(short_negative.ok());
  ExpectYawNear(short_negative.value(), -1.0);
  EXPECT_LT(std::abs(short_negative.value().torque_n_m[2]), 15.0);

  ReferenceHeadingController equivalent;
  VehicleStateRt measured = StateAt(0.0);
  measured.orientation_xyzw = {0, 0, -1, 0};
  MotionReferenceRt command = HeadingCommand(0.0);
  command.orientation_xyzw = {0, 0, 1, 0};
  const StatusOr<BodyWrenchRt> same =
      equivalent.Evaluate(command, measured, Period());
  ASSERT_TRUE(same.ok());
  ExpectYawOnly(same.value(), 0.0);
  EXPECT_DOUBLE_EQ(equivalent.integrator_state(), 0.0);
}

TEST(ReferenceHeadingController, StepResponseMeetsApprovedBounds) {
  // Point-mass yaw, semi-implicit Euler. J = 5 kg*m^2, b = 1 N*m*s/rad,
  // dt = 0.1 s. The heading command steps from 0 to 0.5 rad. The
  // expects at the end are the approved 15% and 20 s bounds.
  constexpr double kHorizonS = 60.0;
  const int steps = static_cast<int>(std::lround(kHorizonS / kDt));
  ReferenceHeadingController controller;
  ReferenceHeadingController repeat;
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double repeat_yaw = yaw;
  double repeat_rate = yaw_rate;
  std::vector<double> headings;
  headings.reserve(steps);
  int stray_axis = 0;

  for (int i = 0; i < steps; ++i) {
    headings.push_back(yaw);
    const StatusOr<BodyWrenchRt> result = controller.Evaluate(
        HeadingCommand(kCommandYawRad), StateAt(yaw, yaw_rate), Period());
    ASSERT_TRUE(result.ok()) << i * kDt;
    const BodyWrenchRt& wrench = result.value();
    if (wrench.force_n[0] != 0.0 || wrench.force_n[1] != 0.0 ||
        wrench.force_n[2] != 0.0 || wrench.torque_n_m[0] != 0.0 ||
        wrench.torque_n_m[1] != 0.0) {
      ++stray_axis;
    }
    const double yaw_torque = wrench.torque_n_m[2];
    EXPECT_LE(yaw_torque, 15.0);
    EXPECT_GE(yaw_torque, -15.0);

    const StatusOr<BodyWrenchRt> again =
        repeat.Evaluate(HeadingCommand(kCommandYawRad),
                        StateAt(repeat_yaw, repeat_rate), Period());
    ASSERT_TRUE(again.ok());
    EXPECT_DOUBLE_EQ(again.value().torque_n_m[2], yaw_torque);

    const double acceleration = (yaw_torque - kDamping * yaw_rate) / kInertia;
    yaw_rate += acceleration * kDt;
    yaw += yaw_rate * kDt;
    const double repeat_acceleration =
        (yaw_torque - kDamping * repeat_rate) / kInertia;
    repeat_rate += repeat_acceleration * kDt;
    repeat_yaw += repeat_rate * kDt;
    EXPECT_DOUBLE_EQ(repeat_yaw, yaw);
  }

  double peak_yaw = headings.front();
  for (double heading : headings) {
    peak_yaw = std::max(peak_yaw, heading);
  }
  int settle_index = steps;
  for (int i = steps - 1; i >= 0; --i) {
    if (std::abs(headings[i] - kCommandYawRad) > kBandRad) {
      break;
    }
    settle_index = i;
  }
  const bool stayed = settle_index < steps;
  const double settle_s = stayed ? settle_index * kDt : kHorizonS;
  const double overshoot_fraction = (peak_yaw - kCommandYawRad) / kStepRad;
  EXPECT_EQ(stray_axis, 0);
  EXPECT_LE(overshoot_fraction, kOvershootLimit)
      << "peak yaw " << peak_yaw << " rad";
  EXPECT_TRUE(stayed) << "final yaw " << headings.back()
                      << " rad did not remain inside +/- " << kBandRad
                      << " rad";
  EXPECT_LE(settle_s, kSettleLimitS) << "settle " << settle_s << " s";
}

}  // namespace
