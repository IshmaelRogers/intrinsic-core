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

#include "intrinsic/vehicle/control/reference_control.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/control/zero_wrench_controller.h"
#include "intrinsic/vehicle/guidance/guidance_step.h"

namespace {

using intrinsic::vehicle::control::BodyWrenchRt;
using intrinsic::vehicle::control::ControlStatus;
using intrinsic::vehicle::control::ControlStatusCode;
using intrinsic::vehicle::control::Duration;
using intrinsic::vehicle::control::FrameId;
using intrinsic::vehicle::control::MotionReferenceRt;
using intrinsic::vehicle::control::ReferenceController;
using intrinsic::vehicle::control::StatusOr;
using intrinsic::vehicle::control::ValidateControlInputs;
using intrinsic::vehicle::control::VehicleStateRt;
using intrinsic::vehicle::control::ZeroWrenchController;
using intrinsic::vehicle::guidance::GuidanceStatusCode;

constexpr std::array<double, 3> kZero3 = {0, 0, 0};

VehicleStateRt ValidState() {
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  state.snapshot_id = 11;
  return state;
}

MotionReferenceRt PoseReference() {
  MotionReferenceRt reference;
  reference.snapshot_id = 11;
  reference.update_period = Duration{0.02};
  reference.has_pose = true;
  reference.pose_frame = FrameId::kWorldEnu;
  reference.position_m = {1, 2, 3};
  reference.orientation_xyzw = {0, 0, 0, 1};
  return reference;
}

MotionReferenceRt TwistReference() {
  MotionReferenceRt reference;
  reference.snapshot_id = 11;
  reference.update_period = Duration{0.02};
  reference.has_twist = true;
  reference.twist_frame = FrameId::kBody;
  reference.body_twist = {0.25, 0, 0, 0, 0, 0.1};
  return reference;
}

Duration Period() { return Duration{0.02}; }

void ExpectFiniteZero(const BodyWrenchRt& wrench) {
  EXPECT_EQ(wrench.frame, FrameId::kBody);
  EXPECT_EQ(wrench.force_n, kZero3);
  EXPECT_EQ(wrench.torque_n_m, kZero3);
  for (double value : wrench.force_n) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (double value : wrench.torque_n_m) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

void ExpectRejected(const char* name, ControlStatusCode code,
                    std::string_view message,
                    const MotionReferenceRt& reference,
                    const VehicleStateRt& state, Duration update_period) {
  const ControlStatus direct =
      ValidateControlInputs(reference, state, update_period);
  const ZeroWrenchController controller;
  const StatusOr<BodyWrenchRt> evaluated =
      controller.Evaluate(reference, state, update_period);
  EXPECT_FALSE(direct.ok()) << name;
  EXPECT_EQ(direct.code, code) << name;
  EXPECT_EQ(direct.message, message) << name;
  ASSERT_FALSE(evaluated.ok()) << name;
  EXPECT_EQ(evaluated.status().code, code) << name;
  EXPECT_EQ(evaluated.status().message, message) << name;
  ExpectFiniteZero(evaluated.value());
  const BodyWrenchRt neutral;
  EXPECT_EQ(std::memcmp(&evaluated.value(), &neutral, sizeof(neutral)), 0);
}

TEST(ReferenceControllerInterface, TypesAreFixedSizeAndAbstract) {
  static_assert(std::is_abstract_v<ReferenceController>);
  static_assert(std::is_final_v<ZeroWrenchController>);
  static_assert(sizeof(ZeroWrenchController) == sizeof(ReferenceController));
  static_assert(std::is_trivially_copyable_v<BodyWrenchRt>);
  static_assert(std::is_trivially_copyable_v<MotionReferenceRt>);
  static_assert(std::is_trivially_copyable_v<VehicleStateRt>);
  static_assert(std::is_trivially_copyable_v<StatusOr<BodyWrenchRt>>);
  static_assert(static_cast<int>(ControlStatusCode::kOk) ==
                static_cast<int>(GuidanceStatusCode::kOk));
  static_assert(static_cast<int>(ControlStatusCode::kInvalidArgument) ==
                static_cast<int>(GuidanceStatusCode::kInvalidArgument));
  static_assert(static_cast<int>(ControlStatusCode::kStale) ==
                static_cast<int>(GuidanceStatusCode::kStale));
  static_assert(static_cast<int>(ControlStatusCode::kInvalid) ==
                static_cast<int>(GuidanceStatusCode::kInvalid));
  static_assert(static_cast<int>(ControlStatusCode::kMissingObjective) ==
                static_cast<int>(GuidanceStatusCode::kMissingObjective));
  static_assert(static_cast<int>(FrameId::kBody) == 3);
  EXPECT_TRUE(ControlStatus::Ok().ok());
}

TEST(ZeroWrenchController, ValidPoseIsNeutralZeroWrench) {
  const ZeroWrenchController concrete;
  const ReferenceController& controller = concrete;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(PoseReference(), ValidState(), Period());
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());
  ExpectFiniteZero(result.value());
}

TEST(ZeroWrenchController, PoseValuesDoNotChangeTheWrench) {
  const ZeroWrenchController controller;
  MotionReferenceRt first = PoseReference();
  MotionReferenceRt second = PoseReference();
  second.position_m = {8, -3, 0.5};
  second.orientation_xyzw = {0, 0, 1, 0};
  second.has_twist = true;
  second.twist_frame = FrameId::kBody;
  second.body_twist = {1, 2, 3, 4, 5, 6};
  const StatusOr<BodyWrenchRt> pose =
      controller.Evaluate(first, ValidState(), Period());
  const StatusOr<BodyWrenchRt> other =
      controller.Evaluate(second, ValidState(), Duration{0});
  ASSERT_TRUE(pose.ok());
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(std::memcmp(&pose.value(), &other.value(), sizeof(pose.value())),
            0);
  ExpectFiniteZero(other.value());
}

TEST(ZeroWrenchController, TwistReferenceIsNeutralZeroWrench) {
  const ZeroWrenchController controller;
  VehicleStateRt state = ValidState();
  state.pose_frame = FrameId::kWorldNed;
  const StatusOr<BodyWrenchRt> result =
      controller.Evaluate(TwistReference(), state, Duration{0});
  ASSERT_TRUE(result.ok());
  ExpectFiniteZero(result.value());
}

TEST(ZeroWrenchController, RepeatedCallsAreBitStable) {
  const ZeroWrenchController controller;
  const MotionReferenceRt reference = PoseReference();
  const VehicleStateRt state = ValidState();
  const StatusOr<BodyWrenchRt> first =
      controller.Evaluate(reference, state, Period());
  ASSERT_TRUE(first.ok());
  for (int i = 0; i < 3; ++i) {
    const StatusOr<BodyWrenchRt> again =
        controller.Evaluate(reference, state, Period());
    ASSERT_TRUE(again.ok());
    EXPECT_EQ(
        std::memcmp(&again.value(), &first.value(), sizeof(first.value())), 0);
  }
}

TEST(ZeroWrenchController, ConcurrentEvaluationsStayIndependent) {
  const ZeroWrenchController controller;
  StatusOr<BodyWrenchRt> ahead =
      StatusOr<BodyWrenchRt>::Failure(ControlStatus::InvalidArgument("unset"));
  StatusOr<BodyWrenchRt> rejected =
      StatusOr<BodyWrenchRt>::Failure(ControlStatus::InvalidArgument("unset"));
  std::thread ok_thread([&] {
    ahead = controller.Evaluate(PoseReference(), ValidState(), Period());
  });
  std::thread bad_thread([&] {
    rejected = controller.Evaluate(MotionReferenceRt{}, ValidState(), Period());
  });
  ok_thread.join();
  bad_thread.join();
  ASSERT_TRUE(ahead.ok());
  ASSERT_FALSE(rejected.ok());
  ExpectFiniteZero(ahead.value());
  ExpectFiniteZero(rejected.value());
  EXPECT_EQ(rejected.status().code, ControlStatusCode::kMissingObjective);
}

TEST(ControlInputs, RejectsDefectsInOrder) {
  MotionReferenceRt reference = PoseReference();
  VehicleStateRt state = ValidState();

  ExpectRejected("negative update period", ControlStatusCode::kInvalidArgument,
                 "update period must be finite and greater than or equal to "
                 "zero",
                 reference, state, Duration{-0.01});

  ExpectRejected(
      "non-finite update period", ControlStatusCode::kInvalidArgument,
      "update period must be finite and greater than or equal to zero",
      reference, state, Duration{std::numeric_limits<double>::infinity()});

  state.pose_frame = FrameId::kBody;
  state.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("state frame precedes finiteness", ControlStatusCode::kInvalid,
                 "state pose frame must be world_enu or world_ned", reference,
                 state, Period());

  state = ValidState();
  state.body_twist[2] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite state", ControlStatusCode::kInvalidArgument,
                 "state value must be finite", reference, state, Period());

  state = ValidState();
  state.orientation_xyzw = {1, 0, 0, 1};
  ExpectRejected("state quaternion", ControlStatusCode::kInvalid,
                 "state orientation quaternion must have unit norm", reference,
                 state, Period());

  state = ValidState();
  reference.update_period = Duration{-1};
  ExpectRejected(
      "negative reference update period", ControlStatusCode::kInvalidArgument,
      "reference update period must be finite and greater than or equal to "
      "zero",
      reference, state, Period());

  reference = PoseReference();
  reference.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("non-finite pose", ControlStatusCode::kInvalidArgument,
                 "pose value must be finite", reference, state, Period());

  reference = PoseReference();
  reference.orientation_xyzw = {0, 0, 0, 0};
  ExpectRejected("pose quaternion", ControlStatusCode::kInvalid,
                 "pose orientation quaternion must have unit norm", reference,
                 state, Period());

  reference = PoseReference();
  reference.pose_frame = FrameId::kUnspecified;
  ExpectRejected("pose frame", ControlStatusCode::kInvalid,
                 "pose frame must be world_enu or world_ned", reference, state,
                 Period());

  reference = TwistReference();
  reference.body_twist[0] = std::numeric_limits<double>::infinity();
  ExpectRejected("non-finite twist", ControlStatusCode::kInvalidArgument,
                 "twist value must be finite", reference, state, Period());

  reference = TwistReference();
  reference.twist_frame = FrameId::kWorldNed;
  ExpectRejected("twist frame", ControlStatusCode::kInvalid,
                 "twist frame must be body", reference, state, Period());

  reference = PoseReference();
  reference.has_pose = false;
  reference.position_m = {1, 2, 3};
  reference.orientation_xyzw = {0, 0, 0, 1};
  reference.body_twist[0] = std::numeric_limits<double>::quiet_NaN();
  ExpectRejected("stored pose without has_pose is missing",
                 ControlStatusCode::kMissingObjective,
                 "reference has no pose or twist objective", reference, state,
                 Period());

  reference = PoseReference();
  reference.snapshot_id = 4;
  ExpectRejected("stale snapshot", ControlStatusCode::kStale,
                 "snapshot_id does not match the vehicle state snapshot",
                 reference, state, Period());
}

}  // namespace
