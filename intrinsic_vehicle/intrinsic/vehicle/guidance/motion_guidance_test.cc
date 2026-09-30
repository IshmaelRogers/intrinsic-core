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
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/guidance/echo_guidance.h"
#include "intrinsic/vehicle/guidance/guidance_step.h"
#include "intrinsic/vehicle/guidance/null_guidance.h"

namespace {

using intrinsic::vehicle::guidance::AssignTrajectoryId;
using intrinsic::vehicle::guidance::DesiredMotionRt;
using intrinsic::vehicle::guidance::Duration;
using intrinsic::vehicle::guidance::EchoGuidance;
using intrinsic::vehicle::guidance::FrameId;
using intrinsic::vehicle::guidance::GuidanceStatus;
using intrinsic::vehicle::guidance::GuidanceStatusCode;
using intrinsic::vehicle::guidance::GuidanceStep;
using intrinsic::vehicle::guidance::kSoftRealtimeMaxHz;
using intrinsic::vehicle::guidance::kSoftRealtimeMinHz;
using intrinsic::vehicle::guidance::kSpatialDof;
using intrinsic::vehicle::guidance::kTrajectoryIdCapacity;
using intrinsic::vehicle::guidance::MotionReferenceRt;
using intrinsic::vehicle::guidance::NullGuidance;
using intrinsic::vehicle::guidance::ObjectiveKind;
using intrinsic::vehicle::guidance::StatusOr;
using intrinsic::vehicle::guidance::ValidateGuidanceInputs;
using intrinsic::vehicle::guidance::VehicleStateRt;

constexpr std::array<double, 3> kZero3 = {0, 0, 0};
constexpr std::array<double, 4> kZero4 = {0, 0, 0, 0};
constexpr std::array<double, 6> kZero6 = {0, 0, 0, 0, 0, 0};

VehicleStateRt ValidState() {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  state.snapshot_id = 7;
  return state;
}

DesiredMotionRt PoseIntent() {
  DesiredMotionRt intent;
  intent.snapshot_id = 7;
  intent.objective = ObjectiveKind::kPose;
  intent.pose_frame = FrameId::kWorldEnu;
  intent.position_m = {1, 2, 3};
  intent.orientation_xyzw = {0, 0, 0, 1};
  return intent;
}

DesiredMotionRt TwistIntent() {
  DesiredMotionRt intent;
  intent.snapshot_id = 7;
  intent.objective = ObjectiveKind::kTwist;
  intent.twist_frame = FrameId::kBody;
  intent.body_twist = {0.5, 0, 0, 0, 0, 0.25};
  return intent;
}

Duration Period() { return Duration{0.02}; }

void ExpectEmpty(const MotionReferenceRt& reference) {
  const MotionReferenceRt empty;
  EXPECT_EQ(std::memcmp(&reference, &empty, sizeof(reference)), 0);
  EXPECT_FALSE(reference.has_pose);
  EXPECT_FALSE(reference.has_twist);
  EXPECT_EQ(reference.position_m, kZero3);
  EXPECT_EQ(reference.orientation_xyzw, kZero4);
  EXPECT_EQ(reference.body_twist, kZero6);
  EXPECT_EQ(reference.snapshot_id, 0u);
  EXPECT_EQ(reference.update_period.seconds, 0);
  EXPECT_TRUE(std::isfinite(reference.update_period.seconds));
}

void ExpectRejected(const char* name, GuidanceStatusCode code,
                    std::string_view message, const DesiredMotionRt& intent,
                    const VehicleStateRt* state, Duration update_period) {
  const GuidanceStatus direct =
      ValidateGuidanceInputs(intent, state, update_period);
  const EchoGuidance echo;
  const NullGuidance null_guidance;
  const StatusOr<MotionReferenceRt> echoed =
      echo.Evaluate(intent, state, update_period);
  const StatusOr<MotionReferenceRt> nulled =
      null_guidance.Evaluate(intent, state, update_period);
  EXPECT_FALSE(direct.ok()) << name;
  EXPECT_EQ(direct.code, code) << name;
  EXPECT_EQ(direct.message, message) << name;
  ASSERT_FALSE(echoed.ok()) << name;
  EXPECT_EQ(echoed.status().code, code) << name;
  EXPECT_EQ(echoed.status().message, message) << name;
  ExpectEmpty(echoed.value());
  ASSERT_FALSE(nulled.ok()) << name;
  EXPECT_EQ(nulled.status().code, code) << name;
  EXPECT_EQ(nulled.status().message, message) << name;
  ExpectEmpty(nulled.value());
}

TEST(GuidanceStepInterface, TypesAreFixedSizeAndAbstract) {
  static_assert(std::is_abstract_v<GuidanceStep>);
  static_assert(std::is_final_v<EchoGuidance>);
  static_assert(std::is_final_v<NullGuidance>);
  static_assert(sizeof(EchoGuidance) == sizeof(GuidanceStep));
  static_assert(sizeof(NullGuidance) == sizeof(GuidanceStep));
  static_assert(std::is_trivially_copyable_v<DesiredMotionRt>);
  static_assert(std::is_trivially_copyable_v<VehicleStateRt>);
  static_assert(std::is_trivially_copyable_v<MotionReferenceRt>);
  static_assert(std::is_trivially_copyable_v<Duration>);
  static_assert(std::is_trivially_copyable_v<StatusOr<MotionReferenceRt>>);
  static_assert(static_cast<int>(FrameId::kUnspecified) == 0);
  static_assert(static_cast<int>(FrameId::kWorldEnu) == 1);
  static_assert(static_cast<int>(FrameId::kWorldNed) == 2);
  static_assert(static_cast<int>(FrameId::kBody) == 3);
  static_assert(static_cast<int>(ObjectiveKind::kUnset) == 0);
  static_assert(static_cast<int>(ObjectiveKind::kPose) == 1);
  static_assert(static_cast<int>(ObjectiveKind::kTwist) == 2);
  static_assert(static_cast<int>(ObjectiveKind::kTrajectory) == 3);
  static_assert(static_cast<int>(GuidanceStatusCode::kOk) == 0);
  static_assert(static_cast<int>(GuidanceStatusCode::kInvalidArgument) == 1);
  static_assert(static_cast<int>(GuidanceStatusCode::kStale) == 2);
  static_assert(static_cast<int>(GuidanceStatusCode::kInvalid) == 3);
  static_assert(static_cast<int>(GuidanceStatusCode::kMissingObjective) == 4);
  static_assert(kSpatialDof == 6);
  static_assert(kTrajectoryIdCapacity == 64);
  EXPECT_EQ(kSoftRealtimeMinHz, 10);
  EXPECT_EQ(kSoftRealtimeMaxHz, 50);
  EXPECT_TRUE(GuidanceStatus::Ok().ok());
}

TEST(EchoGuidance, PoseObjectiveEchoesReference) {
  const EchoGuidance concrete;
  const GuidanceStep& step = concrete;
  VehicleStateRt state = ValidState();
  state.position_m = {9, 9, 9};
  state.body_twist = {4, 0, 0, 0, 0, 0};
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(PoseIntent(), &state, Period());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());
  EXPECT_EQ(result.value().snapshot_id, 7u);
  EXPECT_EQ(result.value().update_period.seconds, 0.02);
  EXPECT_TRUE(result.value().has_pose);
  EXPECT_FALSE(result.value().has_twist);
  EXPECT_EQ(result.value().pose_frame, FrameId::kWorldEnu);
  EXPECT_EQ(result.value().position_m, (std::array<double, 3>{1, 2, 3}));
  EXPECT_EQ(result.value().orientation_xyzw,
            (std::array<double, 4>{0, 0, 0, 1}));
  EXPECT_EQ(result.value().twist_frame, FrameId::kUnspecified);
  EXPECT_EQ(result.value().body_twist, kZero6);
}

TEST(EchoGuidance, TwistObjectiveEchoesReference) {
  const EchoGuidance step;
  DesiredMotionRt intent = TwistIntent();
  intent.pose_frame = FrameId::kWorldEnu;
  intent.position_m = {std::numeric_limits<double>::quiet_NaN(), 0, 0};
  intent.orientation_xyzw = {0, 0, 0, 0};
  VehicleStateRt state = ValidState();
  state.pose_frame = FrameId::kWorldNed;
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, &state, Duration{0});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().snapshot_id, 7u);
  EXPECT_EQ(result.value().update_period.seconds, 0);
  EXPECT_FALSE(result.value().has_pose);
  EXPECT_TRUE(result.value().has_twist);
  EXPECT_EQ(result.value().pose_frame, FrameId::kUnspecified);
  EXPECT_EQ(result.value().position_m, kZero3);
  EXPECT_EQ(result.value().orientation_xyzw, kZero4);
  EXPECT_EQ(result.value().twist_frame, FrameId::kBody);
  EXPECT_EQ(result.value().body_twist,
            (std::array<double, 6>{0.5, 0, 0, 0, 0, 0.25}));
}

TEST(EchoGuidance, NullStateSkipsSnapshotCheck) {
  const EchoGuidance step;
  DesiredMotionRt intent = PoseIntent();
  intent.snapshot_id = 3;
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Period());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().snapshot_id, 3u);
  EXPECT_TRUE(result.value().has_pose);
}

TEST(EchoGuidance, SuppliedZeroConfidenceAndMatchingHorizonAreKept) {
  const EchoGuidance step;
  DesiredMotionRt intent = PoseIntent();
  intent.has_confidence = true;
  intent.confidence = 0;
  intent.has_horizon = true;
  intent.horizon_s = 0.02;
  const GuidanceStatus direct =
      ValidateGuidanceInputs(intent, nullptr, Period());
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Period());
  EXPECT_TRUE(direct.ok());
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.value().has_pose);
}

TEST(EchoGuidance, ZeroHorizonWithZeroPeriodIsInstantaneous) {
  const EchoGuidance step;
  DesiredMotionRt intent = PoseIntent();
  intent.has_horizon = true;
  intent.horizon_s = 0;
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Duration{0});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().update_period.seconds, 0);
}

TEST(EchoGuidance, WorldNedPoseIsAccepted) {
  const EchoGuidance step;
  DesiredMotionRt intent = PoseIntent();
  intent.pose_frame = FrameId::kWorldNed;
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Period());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().pose_frame, FrameId::kWorldNed);
}

TEST(EchoGuidance, TrajectoryIdIsNotSampled) {
  const EchoGuidance step;
  DesiredMotionRt intent;
  intent.snapshot_id = 7;
  intent.objective = ObjectiveKind::kTrajectory;
  ASSERT_TRUE(AssignTrajectoryId(intent.trajectory_id, "traj-1"));
  const VehicleStateRt state = ValidState();
  const GuidanceStatus direct =
      ValidateGuidanceInputs(intent, &state, Period());
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, &state, Period());
  EXPECT_TRUE(direct.ok());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, GuidanceStatusCode::kInvalid);
  EXPECT_EQ(result.status().message,
            std::string_view("trajectory objective is not sampled"));
  ExpectEmpty(result.value());
}

TEST(EchoGuidance, FullTrajectoryBufferIsNotSampled) {
  const EchoGuidance step;
  DesiredMotionRt intent;
  intent.objective = ObjectiveKind::kTrajectory;
  const std::string id(kTrajectoryIdCapacity, 'a');
  ASSERT_TRUE(AssignTrajectoryId(intent.trajectory_id, id));
  EXPECT_EQ(intent.trajectory_id.view(), id);
  EXPECT_FALSE(AssignTrajectoryId(intent.trajectory_id,
                                  std::string(kTrajectoryIdCapacity + 1, 'b')));
  EXPECT_EQ(intent.trajectory_id.view(), id);
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Duration{0});
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, GuidanceStatusCode::kInvalid);
  ExpectEmpty(result.value());
}

TEST(EchoGuidance, RepeatedCallsAreBitStable) {
  const EchoGuidance step;
  const VehicleStateRt state = ValidState();
  DesiredMotionRt intent = PoseIntent();
  intent.has_confidence = true;
  intent.confidence = 1;
  intent.has_horizon = true;
  intent.horizon_s = 1;
  const StatusOr<MotionReferenceRt> first =
      step.Evaluate(intent, &state, Period());
  ASSERT_TRUE(first.ok());
  for (int i = 0; i < 3; ++i) {
    const StatusOr<MotionReferenceRt> again =
        step.Evaluate(intent, &state, Period());
    ASSERT_TRUE(again.ok());
    EXPECT_EQ(
        std::memcmp(&again.value(), &first.value(), sizeof(first.value())), 0);
  }
}

TEST(EchoGuidance, ConcurrentEvaluationsStayIndependent) {
  const EchoGuidance step;
  StatusOr<MotionReferenceRt> pose = StatusOr<MotionReferenceRt>::Failure(
      GuidanceStatus::InvalidArgument("unset"));
  StatusOr<MotionReferenceRt> twist = StatusOr<MotionReferenceRt>::Failure(
      GuidanceStatus::InvalidArgument("unset"));
  std::thread pose_thread(
      [&] { pose = step.Evaluate(PoseIntent(), nullptr, Period()); });
  std::thread twist_thread(
      [&] { twist = step.Evaluate(TwistIntent(), nullptr, Duration{0.05}); });
  pose_thread.join();
  twist_thread.join();
  ASSERT_TRUE(pose.ok());
  ASSERT_TRUE(twist.ok());
  EXPECT_TRUE(pose.value().has_pose);
  EXPECT_FALSE(pose.value().has_twist);
  EXPECT_TRUE(twist.value().has_twist);
  EXPECT_FALSE(twist.value().has_pose);
  EXPECT_EQ(pose.value().position_m, (std::array<double, 3>{1, 2, 3}));
  EXPECT_EQ(twist.value().body_twist[0], 0.5);
  EXPECT_EQ(pose.value().update_period.seconds, 0.02);
  EXPECT_EQ(twist.value().update_period.seconds, 0.05);
}

TEST(NullGuidance, UnsetObjectiveIsMissing) {
  const NullGuidance step;
  const DesiredMotionRt intent;
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(intent, nullptr, Period());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, GuidanceStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message, std::string_view("objective is unset"));
  ExpectEmpty(result.value());
  EXPECT_FALSE(result.value().has_pose);
}

TEST(NullGuidance, ValidPoseDoesNotBecomeAReference) {
  const NullGuidance concrete;
  const GuidanceStep& step = concrete;
  const VehicleStateRt state = ValidState();
  const GuidanceStatus direct =
      ValidateGuidanceInputs(PoseIntent(), &state, Period());
  const StatusOr<MotionReferenceRt> result =
      step.Evaluate(PoseIntent(), &state, Period());
  EXPECT_TRUE(direct.ok());
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, GuidanceStatusCode::kMissingObjective);
  EXPECT_EQ(result.status().message,
            std::string_view("null guidance does not produce a reference"));
  ExpectEmpty(result.value());
}

TEST(GuidanceInputs, RejectsDefectsInOrder) {
  const VehicleStateRt* no_state = nullptr;
  DesiredMotionRt intent = PoseIntent();

  ExpectRejected("negative update period", GuidanceStatusCode::kInvalidArgument,
                 "update period must be finite and greater than or equal to "
                 "zero",
                 intent, no_state, Duration{-1});

  ExpectRejected(
      "non-finite update period", GuidanceStatusCode::kInvalidArgument,
      "update period must be finite and greater than or equal to zero", intent,
      no_state, Duration{std::numeric_limits<double>::quiet_NaN()});

  VehicleStateRt bad_frame = ValidState();
  bad_frame.pose_frame = FrameId::kBody;
  bad_frame.position_m[0] = std::numeric_limits<double>::infinity();
  ExpectRejected("state frame precedes finiteness",
                 GuidanceStatusCode::kInvalid,
                 "state pose frame must be world_enu or world_ned", intent,
                 &bad_frame, Period());

  VehicleStateRt bad_state = ValidState();
  bad_state.body_twist[0] = std::numeric_limits<double>::quiet_NaN();
  intent.objective = ObjectiveKind::kUnset;
  ExpectRejected("non-finite state precedes a missing objective",
                 GuidanceStatusCode::kInvalidArgument,
                 "state value must be finite", intent, &bad_state, Period());

  VehicleStateRt bad_quat = ValidState();
  bad_quat.orientation_xyzw = {0, 0, 0, 0};
  intent = PoseIntent();
  ExpectRejected("state quaternion", GuidanceStatusCode::kInvalid,
                 "state orientation quaternion must have unit norm", intent,
                 &bad_quat, Period());

  intent = PoseIntent();
  intent.has_confidence = true;
  intent.confidence = std::numeric_limits<double>::quiet_NaN();
  intent.objective = ObjectiveKind::kUnset;
  ExpectRejected("non-finite confidence precedes a missing objective",
                 GuidanceStatusCode::kInvalidArgument,
                 "confidence must be finite and in [0, 1]", intent, no_state,
                 Period());

  intent = PoseIntent();
  intent.has_confidence = true;
  intent.confidence = 1.1;
  ExpectRejected("confidence above one", GuidanceStatusCode::kInvalidArgument,
                 "confidence must be finite and in [0, 1]", intent, no_state,
                 Period());

  intent.confidence = -0.1;
  ExpectRejected("negative confidence", GuidanceStatusCode::kInvalidArgument,
                 "confidence must be finite and in [0, 1]", intent, no_state,
                 Period());

  intent = PoseIntent();
  intent.has_horizon = true;
  intent.horizon_s = -1;
  ExpectRejected("negative horizon", GuidanceStatusCode::kInvalidArgument,
                 "horizon must be finite and greater than or equal to zero",
                 intent, no_state, Period());

  intent.horizon_s = std::numeric_limits<double>::infinity();
  ExpectRejected("non-finite horizon", GuidanceStatusCode::kInvalidArgument,
                 "horizon must be finite and greater than or equal to zero",
                 intent, no_state, Period());

  intent = PoseIntent();
  intent.objective = ObjectiveKind::kUnset;
  VehicleStateRt stale = ValidState();
  stale.snapshot_id = 8;
  ExpectRejected("missing objective precedes a stale snapshot",
                 GuidanceStatusCode::kMissingObjective, "objective is unset",
                 intent, &stale, Period());

  intent = PoseIntent();
  intent.position_m[1] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite pose", GuidanceStatusCode::kInvalidArgument,
                 "pose value must be finite", intent, no_state, Period());

  intent = PoseIntent();
  intent.orientation_xyzw = {0, 0, 0, 0};
  ExpectRejected("pose quaternion", GuidanceStatusCode::kInvalid,
                 "pose orientation quaternion must have unit norm", intent,
                 no_state, Period());

  intent = PoseIntent();
  intent.pose_frame = FrameId::kBody;
  ExpectRejected("pose frame", GuidanceStatusCode::kInvalid,
                 "pose frame must be world_enu or world_ned", intent, no_state,
                 Period());

  intent = TwistIntent();
  intent.body_twist[5] = std::numeric_limits<double>::infinity();
  ExpectRejected("non-finite twist", GuidanceStatusCode::kInvalidArgument,
                 "twist value must be finite", intent, no_state, Period());

  intent = TwistIntent();
  intent.twist_frame = FrameId::kWorldEnu;
  ExpectRejected("twist frame", GuidanceStatusCode::kInvalid,
                 "twist frame must be body", intent, no_state, Period());

  intent = DesiredMotionRt{};
  intent.objective = ObjectiveKind::kTrajectory;
  ExpectRejected("empty trajectory id", GuidanceStatusCode::kInvalid,
                 "trajectory id is empty", intent, no_state, Period());

  intent.trajectory_id.length = 65;
  ExpectRejected("trajectory id longer than the buffer",
                 GuidanceStatusCode::kInvalidArgument,
                 "trajectory id is longer than 64 bytes", intent, no_state,
                 Period());

  intent = PoseIntent();
  intent.snapshot_id = 1;
  VehicleStateRt other = ValidState();
  intent.has_horizon = true;
  intent.horizon_s = 0;
  ExpectRejected("stale snapshot precedes horizon expiry",
                 GuidanceStatusCode::kStale,
                 "snapshot_id does not match the vehicle state snapshot",
                 intent, &other, Period());

  intent = PoseIntent();
  intent.has_horizon = true;
  intent.horizon_s = 0.01;
  const VehicleStateRt state = ValidState();
  ExpectRejected("expired horizon", GuidanceStatusCode::kStale,
                 "horizon does not cover the update period", intent, &state,
                 Period());

  intent = PoseIntent();
  intent.has_horizon = false;
  intent.horizon_s = -5;
  intent.has_confidence = false;
  intent.confidence = std::numeric_limits<double>::quiet_NaN();
  const GuidanceStatus ignored =
      ValidateGuidanceInputs(intent, &state, Period());
  EXPECT_TRUE(ignored.ok());
}

}  // namespace
