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

#include "intrinsic/vehicle/control/reference_forward_speed_controller.h"

#include <algorithm>
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
using intrinsic::vehicle::control::ReferenceForwardSpeedController;
using intrinsic::vehicle::control::StatusOr;
using intrinsic::vehicle::control::VehicleStateRt;
using intrinsic::vehicle::guidance::kHeave;
using intrinsic::vehicle::guidance::kPitch;
using intrinsic::vehicle::guidance::kRoll;
using intrinsic::vehicle::guidance::kSurge;
using intrinsic::vehicle::guidance::kSway;
using intrinsic::vehicle::guidance::kYaw;

constexpr double kDt = 0.1;
constexpr double kCommandSpeed = 0.5;
constexpr double kStepMps = 0.5;
constexpr double kMassKg = 60.0;
constexpr double kDamping = 5.0;
constexpr double kOvershootLimit = 0.15;
constexpr double kSettleLimitS = 20.0;
constexpr double kBandMps = 0.02;

VehicleStateRt StateAt(double surge_m_s) {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  state.body_twist[kSurge] = surge_m_s;
  state.snapshot_id = 11;
  return state;
}

MotionReferenceRt SpeedCommand(double surge_m_s) {
  MotionReferenceRt reference;
  reference.snapshot_id = 11;
  reference.update_period = Duration{kDt};
  reference.has_twist = true;
  reference.twist_frame = FrameId::kBody;
  reference.body_twist[kSurge] = surge_m_s;
  return reference;
}

Duration Period() { return Duration{kDt}; }

void ExpectSurgeOnly(const BodyWrenchRt& wrench, double surge_n) {
  EXPECT_EQ(wrench.frame, FrameId::kBody);
  EXPECT_DOUBLE_EQ(wrench.force_n[kSurge], surge_n);
  EXPECT_DOUBLE_EQ(wrench.force_n[kSway], 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_n[kHeave], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[0], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[1], 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_n_m[2], 0.0);
}

void ExpectFiniteZero(const BodyWrenchRt& wrench) {
  ExpectSurgeOnly(wrench, 0.0);
  const BodyWrenchRt neutral;
  EXPECT_EQ(std::memcmp(&wrench, &neutral, sizeof(neutral)), 0);
}

TEST(ReferenceForwardSpeedController, PositiveErrorCommandsPositiveSurge) {
  ReferenceForwardSpeedController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectSurgeOnly(result.value(), 25.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.25);
}

TEST(ReferenceForwardSpeedController, NegativeErrorCommandsNegativeSurge) {
  ReferenceForwardSpeedController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(SpeedCommand(0.0), StateAt(0.25), Period());
  ASSERT_TRUE(result.ok());
  ExpectSurgeOnly(result.value(), -12.5);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), -0.125);
}

TEST(ReferenceForwardSpeedController, ZeroErrorCommandsZeroSurge) {
  ReferenceForwardSpeedController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.5), Period());
  ASSERT_TRUE(result.ok());
  ExpectSurgeOnly(result.value(), 0.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceForwardSpeedController, IntegralForceChangesTheNextCommand) {
  ReferenceForwardSpeedController controller;
  ASSERT_TRUE(
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period()).ok());
  const double integral_force = controller.integrator_state();
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(second.ok());
  ExpectSurgeOnly(second.value(), 25.0 + integral_force);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), integral_force + 0.25);
}

TEST(ReferenceForwardSpeedController, AntiWindupFeedsTheIntegrator) {
  ReferenceForwardSpeedController controller;
  // Proportional action alone is 100 N, past the +60 N rail.
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(SpeedCommand(2.0), StateAt(0.0), Period());
  ASSERT_TRUE(result.ok());
  ExpectSurgeOnly(result.value(), 60.0);
  const double pure_integral = 5.0 * 2.0 * kDt;
  const double u_unsat = 50.0 * 2.0;
  const double tracking = 0.2 * (60.0 - u_unsat);
  const double expected = (5.0 * 2.0 + tracking) * kDt;
  EXPECT_DOUBLE_EQ(controller.integrator_state(), expected);
  EXPECT_LT(controller.integrator_state(), pure_integral);
}

TEST(ReferenceForwardSpeedController, SaturatedHoldStaysInsideTheClamp) {
  ReferenceForwardSpeedController controller;
  StatusOr<BodyWrenchRt> result = StatusOr<BodyWrenchRt>::Failure(
      intrinsic::vehicle::control::ControlStatus{});
  for (int i = 0; i < 40; ++i) {
    result = controller.Evaluate(SpeedCommand(10.0), StateAt(0.0), Period());
    ASSERT_TRUE(result.ok());
    EXPECT_DOUBLE_EQ(result.value().force_n[kSurge], 60.0);
    EXPECT_LE(controller.integrator_state(), 48.0);
    EXPECT_GE(controller.integrator_state(), -48.0);
  }
  // The unsaturated equilibrium of S is below -48 N.
  EXPECT_DOUBLE_EQ(controller.integrator_state(), -48.0);
}

TEST(ReferenceForwardSpeedController, RejectedInputDoesNotWriteTheIntegrator) {
  ReferenceForwardSpeedController controller;
  ASSERT_TRUE(
      controller.Evaluate(SpeedCommand(2.0), StateAt(0.0), Period()).ok());
  const double held = controller.integrator_state();
  ASSERT_NE(held, 0.0);

  VehicleStateRt stale = StateAt(0.0);
  stale.snapshot_id = 12;
  const StatusOr<BodyWrenchRt> stale_result =
      controller.Evaluate(SpeedCommand(2.0), stale, Period());
  ASSERT_FALSE(stale_result.ok());
  EXPECT_EQ(stale_result.status().code, ControlStatusCode::kStale);
  EXPECT_EQ(stale_result.status().message,
            std::string_view(
                "snapshot_id does not match the vehicle state snapshot"));
  ExpectFiniteZero(stale_result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  VehicleStateRt non_finite_meas = StateAt(0.0);
  non_finite_meas.body_twist[kSurge] = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<BodyWrenchRt> nan_meas =
      controller.Evaluate(SpeedCommand(2.0), non_finite_meas, Period());
  ASSERT_FALSE(nan_meas.ok());
  EXPECT_EQ(nan_meas.status().code, ControlStatusCode::kInvalidArgument);
  EXPECT_EQ(nan_meas.status().message,
            std::string_view("state value must be finite"));
  ExpectFiniteZero(nan_meas.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  MotionReferenceRt non_finite_cmd = SpeedCommand(2.0);
  non_finite_cmd.body_twist[kSurge] = std::numeric_limits<double>::infinity();
  const StatusOr<BodyWrenchRt> nan_cmd =
      controller.Evaluate(non_finite_cmd, StateAt(0.0), Period());
  ASSERT_FALSE(nan_cmd.ok());
  EXPECT_EQ(nan_cmd.status().code, ControlStatusCode::kInvalidArgument);
  EXPECT_EQ(nan_cmd.status().message,
            std::string_view("twist value must be finite"));
  ExpectFiniteZero(nan_cmd.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  const StatusOr<BodyWrenchRt> overflow =
      controller.Evaluate(SpeedCommand(1e308), StateAt(0.0), Period());
  ASSERT_FALSE(overflow.ok());
  EXPECT_EQ(overflow.status().code, ControlStatusCode::kInvalidArgument);
  EXPECT_EQ(overflow.status().message,
            std::string_view("surge command must be finite"));
  ExpectFiniteZero(overflow.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  MotionReferenceRt world_twist = SpeedCommand(2.0);
  world_twist.twist_frame = FrameId::kWorldEnu;
  const StatusOr<BodyWrenchRt> bad_frame =
      controller.Evaluate(world_twist, StateAt(0.0), Period());
  ASSERT_FALSE(bad_frame.ok());
  EXPECT_EQ(bad_frame.status().code, ControlStatusCode::kInvalid);
  EXPECT_EQ(bad_frame.status().message,
            std::string_view("twist frame must be body"));
  ExpectFiniteZero(bad_frame.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);

  const StatusOr<BodyWrenchRt> negative_dt =
      controller.Evaluate(SpeedCommand(2.0), StateAt(0.0), Duration{-0.1});
  ASSERT_FALSE(negative_dt.ok());
  EXPECT_EQ(negative_dt.status().code, ControlStatusCode::kInvalidArgument);
  ExpectFiniteZero(negative_dt.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceForwardSpeedController, ResetClearsTheIntegralForce) {
  ReferenceForwardSpeedController controller;
  ASSERT_TRUE(
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period()).ok());
  ASSERT_NE(controller.integrator_state(), 0.0);
  controller.Reset();
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  ReferenceForwardSpeedController fresh;
  const StatusOr<BodyWrenchRt> restored =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period());
  const StatusOr<BodyWrenchRt> first =
      fresh.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period());
  ASSERT_TRUE(restored.ok());
  ASSERT_TRUE(first.ok());
  EXPECT_EQ(
      std::memcmp(&restored.value(), &first.value(), sizeof(first.value())), 0);
}

TEST(ReferenceForwardSpeedController,
     PoseOnlyReferenceMissesTheSurgeObjective) {
  ReferenceForwardSpeedController controller;
  ASSERT_TRUE(
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Period()).ok());
  const double held = controller.integrator_state();
  MotionReferenceRt pose;
  pose.snapshot_id = 11;
  pose.update_period = Duration{kDt};
  pose.has_pose = true;
  pose.pose_frame = FrameId::kWorldEnu;
  pose.orientation_xyzw = {0, 0, 0, 1};
  pose.position_m = {1.0, 2.0, -3.0};
  // Stored surge is not a reference while has_twist is false.
  pose.body_twist[kSurge] = 4.0;
  pose.twist_frame = FrameId::kBody;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(pose, StateAt(0.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no surge objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), held);
}

TEST(ReferenceForwardSpeedController,
     EmptyReferenceIsMissingBeforeSurgeChecks) {
  const ReferenceForwardSpeedController controller;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(MotionReferenceRt{}, StateAt(0.0), Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("reference has no pose or twist objective"));
  ExpectFiniteZero(result.value());
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
}

TEST(ReferenceForwardSpeedController, ZeroTimeStepDoesNotIntegrate) {
  ReferenceForwardSpeedController controller;
  const StatusOr<BodyWrenchRt> first =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Duration{0});
  ASSERT_TRUE(first.ok());
  ExpectSurgeOnly(first.value(), 25.0);
  EXPECT_DOUBLE_EQ(controller.integrator_state(), 0.0);
  const StatusOr<BodyWrenchRt> second =
      controller.Evaluate(SpeedCommand(0.5), StateAt(0.0), Duration{0});
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(std::memcmp(&first.value(), &second.value(), sizeof(first.value())),
            0);
}

TEST(ReferenceForwardSpeedController, OnlyBodySurgeIsRead) {
  ReferenceForwardSpeedController plain;
  ReferenceForwardSpeedController distracted;
  const StatusOr<BodyWrenchRt> surge_only =
      plain.Evaluate(SpeedCommand(0.5), StateAt(0.2), Period());
  VehicleStateRt state = StateAt(0.2);
  state.pose_frame = FrameId::kWorldNed;
  state.position_m = {40.0, -7.0, -12.0};
  state.orientation_xyzw = {1, 0, 0, 0};
  state.body_twist[kSway] = 0.4;
  state.body_twist[kHeave] = -0.3;
  state.body_twist[kRoll] = 0.2;
  state.body_twist[kPitch] = -0.1;
  state.body_twist[kYaw] = 0.8;
  MotionReferenceRt reference = SpeedCommand(0.5);
  reference.has_pose = true;
  reference.pose_frame = FrameId::kWorldNed;
  reference.position_m = {-12.0, 4.0, -30.0};
  reference.orientation_xyzw = {0, 0, 1, 0};
  reference.body_twist[kSway] = 1.0;
  reference.body_twist[kHeave] = -0.5;
  reference.body_twist[kYaw] = 0.9;
  const StatusOr<BodyWrenchRt> other =
      distracted.Evaluate(reference, state, Period());
  ASSERT_TRUE(surge_only.ok());
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(
      std::memcmp(&surge_only.value(), &other.value(), sizeof(other.value())),
      0);
  ExpectSurgeOnly(other.value(), 15.0);
  EXPECT_DOUBLE_EQ(other.value().force_n[kHeave], 0.0);
  EXPECT_DOUBLE_EQ(other.value().torque_n_m[2], 0.0);
  EXPECT_DOUBLE_EQ(distracted.integrator_state(), plain.integrator_state());
}

TEST(ReferenceForwardSpeedController,
     GroundRelativePolicyDoesNotSubtractCurrent) {
  // A 0.3 m/s current is not an input. The law uses body surge as given.
  // Subtracting that current would change the error from 0.3 m/s to
  // 0.6 m/s and the force from 15 N to 30 N.
  constexpr double kCurrentMps = 0.3;
  constexpr double kGroundSurge = 0.2;
  ReferenceForwardSpeedController ground;
  const StatusOr<BodyWrenchRt> as_measured =
      ground.Evaluate(SpeedCommand(0.5), StateAt(kGroundSurge), Period());
  ASSERT_TRUE(as_measured.ok());
  ExpectSurgeOnly(as_measured.value(), 15.0);
  EXPECT_DOUBLE_EQ(ground.integrator_state(), 0.15);
  const double current_relative = kGroundSurge - kCurrentMps;
  const double wrongly_subtracted = 50.0 * (0.5 - current_relative);
  EXPECT_NE(as_measured.value().force_n[kSurge], wrongly_subtracted);

  // The same relative speed is only a different u_meas. The controller
  // does not transform a current of its own.
  ReferenceForwardSpeedController presented;
  const StatusOr<BodyWrenchRt> as_relative = presented.Evaluate(
      SpeedCommand(0.5), StateAt(current_relative), Period());
  ASSERT_TRUE(as_relative.ok());
  ExpectSurgeOnly(as_relative.value(), wrongly_subtracted);
  EXPECT_NE(as_relative.value().force_n[kSurge],
            as_measured.value().force_n[kSurge]);
}

TEST(ReferenceForwardSpeedController, RepeatedInputsMatchBitForBit) {
  ReferenceForwardSpeedController first;
  ReferenceForwardSpeedController second;
  const double commands[] = {0.5, 2.0, 0.0};
  const double measured[] = {0.0, 0.1, 0.4};
  for (int i = 0; i < 3; ++i) {
    const StatusOr<BodyWrenchRt> left = first.Evaluate(
        SpeedCommand(commands[i]), StateAt(measured[i]), Period());
    const StatusOr<BodyWrenchRt> right = second.Evaluate(
        SpeedCommand(commands[i]), StateAt(measured[i]), Period());
    ASSERT_TRUE(left.ok());
    ASSERT_TRUE(right.ok());
    EXPECT_EQ(std::memcmp(&left.value(), &right.value(), sizeof(left.value())),
              0);
    EXPECT_DOUBLE_EQ(first.integrator_state(), second.integrator_state());
  }
}

TEST(ReferenceForwardSpeedController, StepResponseMeetsApprovedBounds) {
  // Point-mass surge, semi-implicit Euler. m = 60 kg, b = 5 N*s/m,
  // dt = 0.1 s. The speed command steps from 0 to 0.5 m/s. The
  // expects at the end are the approved 15% and 20 s bounds.
  constexpr double kHorizonS = 60.0;
  const int steps = static_cast<int>(std::lround(kHorizonS / kDt));
  ReferenceForwardSpeedController controller;
  ReferenceForwardSpeedController repeat;
  double speed = 0.0;
  double repeat_speed = speed;
  std::vector<double> speeds;
  speeds.reserve(steps);
  int stray_axis = 0;

  for (int i = 0; i < steps; ++i) {
    speeds.push_back(speed);
    const StatusOr<BodyWrenchRt> result = controller.Evaluate(
        SpeedCommand(kCommandSpeed), StateAt(speed), Period());
    ASSERT_TRUE(result.ok()) << i * kDt;
    const BodyWrenchRt& wrench = result.value();
    if (wrench.force_n[kSway] != 0.0 || wrench.force_n[kHeave] != 0.0 ||
        wrench.torque_n_m[0] != 0.0 || wrench.torque_n_m[1] != 0.0 ||
        wrench.torque_n_m[2] != 0.0) {
      ++stray_axis;
    }
    const double surge = wrench.force_n[kSurge];
    EXPECT_LE(surge, 60.0);
    EXPECT_GE(surge, -60.0);

    const StatusOr<BodyWrenchRt> again = repeat.Evaluate(
        SpeedCommand(kCommandSpeed), StateAt(repeat_speed), Period());
    ASSERT_TRUE(again.ok());
    EXPECT_DOUBLE_EQ(again.value().force_n[kSurge], surge);
    EXPECT_EQ(std::memcmp(&wrench, &again.value(), sizeof(wrench)), 0);

    const double acceleration = (surge - kDamping * speed) / kMassKg;
    speed += acceleration * kDt;
    const double repeat_acceleration =
        (surge - kDamping * repeat_speed) / kMassKg;
    repeat_speed += repeat_acceleration * kDt;
    EXPECT_DOUBLE_EQ(repeat_speed, speed);
  }

  double peak_speed = speeds.front();
  for (double sample : speeds) {
    peak_speed = std::max(peak_speed, sample);
  }
  int settle_index = steps;
  for (int i = steps - 1; i >= 0; --i) {
    if (std::abs(speeds[i] - kCommandSpeed) > kBandMps) {
      break;
    }
    settle_index = i;
  }
  const bool stayed = settle_index < steps;
  const double settle_s = stayed ? settle_index * kDt : kHorizonS;
  const double overshoot_fraction = (peak_speed - kCommandSpeed) / kStepMps;
  EXPECT_EQ(stray_axis, 0);
  EXPECT_LE(overshoot_fraction, kOvershootLimit)
      << "peak speed " << peak_speed << " m/s";
  EXPECT_TRUE(stayed) << "final speed " << speeds.back()
                      << " m/s did not remain inside +/- " << kBandMps
                      << " m/s";
  EXPECT_LE(settle_s, kSettleLimitS) << "settle " << settle_s << " s";
}

}  // namespace
