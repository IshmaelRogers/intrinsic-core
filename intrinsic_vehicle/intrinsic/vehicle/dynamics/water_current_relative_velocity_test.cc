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

#include "intrinsic/vehicle/dynamics/water_current_relative_velocity.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <type_traits>

#include "intrinsic/vehicle/dynamics/gravity_buoyancy_restoring_wrench.h"
#include "intrinsic/vehicle/dynamics/linear_quadratic_damping_wrench.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"
#include "gtest/gtest.h"

namespace {

using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::ComputeGravityBuoyancyRestoringWrench;
using intrinsic::vehicle::dynamics::ComputeLinearQuadraticDampingWrench;
using intrinsic::vehicle::dynamics::ComputeWaterCurrentRelativeVelocity;
using intrinsic::vehicle::dynamics::CurrentRelativeTwist;
using intrinsic::vehicle::dynamics::DampingWrench;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::FrameId;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kUnitQuaternionTolerance;
using intrinsic::vehicle::dynamics::RestoringWrench;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::dynamics::ZeroForceDynamics;
using intrinsic::vehicle::parameters::Damping;
using intrinsic::vehicle::parameters::Environment;
using intrinsic::vehicle::parameters::kBodyFrameId;
using intrinsic::vehicle::parameters::kWorldEnuFrameId;
using intrinsic::vehicle::parameters::kWorldNedFrameId;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::Vec3;

// Dyadic fixtures are exact in IEEE binary64. The 90 degree ENU heading
// uses sqrt(2)/2, so that one comparison uses the same 1e-12 absolute
// tolerance as the restoring and damping fixtures. Repeats are compared
// with memcmp.
constexpr double kAbsTolerance = 1e-12;

constexpr std::string_view kCurrentNonFiniteMessage =
    "environment.current_velocity_m_s must be finite";
constexpr std::string_view kCurrentFrameEmptyMessage =
    "environment.current_frame_id must be non-empty";
constexpr std::string_view kCurrentFrameMessage =
    "environment.current_frame_id must be world_enu, world_ned, or body";
constexpr std::string_view kPoseFrameMessage =
    "pose_frame must be world_enu or world_ned";
constexpr std::string_view kOrientationNonFiniteMessage =
    "orientation_xyzw must be finite";
constexpr std::string_view kOrientationUnitMessage =
    "orientation_xyzw must have unit norm";
constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kRelativeVelocityNonFiniteMessage =
    "relative velocity is not finite";

using Twist = std::array<double, kSpatialDof>;
using Quaternion = std::array<double, 4>;

constexpr Quaternion kIdentity = {0.0, 0.0, 0.0, 1.0};
// 180 deg about body x. Exact. In NED this is level, heading north:
// body +x = NED +x, body +y = NED -y, body +z = NED -z.
constexpr Quaternion kRoll180 = {1.0, 0.0, 0.0, 0.0};
// 180 deg about body z. Exact. Body +x points along navigation -x.
constexpr Quaternion kYaw180 = {0.0, 0.0, 1.0, 0.0};

// ν = [4, 1, 1/2, 1/4, -1/8, 1/16]. All values are dyadic.
constexpr Twist kTwist = {4.0, 1.0, 0.5, 0.25, -0.125, 0.0625};
// Current (1, -2, 1/4) m/s in the named frame.
constexpr Vec3 kCurrent = {1.0, -2.0, 0.25};

// Body frame, or any world frame with R = I. v_c^b = (1, -2, 1/4).
// ν_r = [3, 3, 1/4, 1/4, -1/8, 1/16].
constexpr Twist kAlignedRelative = {3.0, 3.0, 0.25, 0.25, -0.125, 0.0625};

// 180 deg yaw. R^T (cx, cy, cz) = (-cx, -cy, cz).
// v_c^b = (-1, 2, 1/4). ν_r = [5, -1, 1/4, 1/4, -1/8, 1/16].
constexpr Twist kYaw180Relative = {5.0, -1.0, 0.25, 0.25, -0.125, 0.0625};

// 180 deg roll. R^T (cx, cy, cz) = (cx, -cy, -cz).
// v_c^b = (1, 2, -1/4). ν_r = [3, -1, 3/4, 1/4, -1/8, 1/16].
constexpr Twist kRoll180Relative = {3.0, -1.0, 0.75, 0.25, -0.125, 0.0625};

// Level vehicle heading north. Current is 2 m/s north and 0 up.
// NED vector (2, 0, 0), q = kRoll180, so v_c^b = (2, 0, 0).
// ENU vector (0, 2, 0), q = 90 deg yaw. Same body current.
// ν_r = [2, 1, 1/2, 1/4, -1/8, 1/16].
constexpr Vec3 kNorthNed = {2.0, 0.0, 0.0};
constexpr Vec3 kNorthEnu = {0.0, 2.0, 0.0};
constexpr Twist kNorthRelative = {2.0, 1.0, 0.5, 0.25, -0.125, 0.0625};

// Opposing surge current. v_c^b = (-4, 0, 0). ν_r surge = 4 - (-4) = 8.
constexpr Vec3 kReversedSurge = {-4.0, 0.0, 0.0};
constexpr Twist kReversedRelative = {8.0, 1.0, 0.5, 0.25, -0.125, 0.0625};

// Sway only. v_c^b = (0, 1/2, 0). ν_r sway = 1 - 1/2 = 1/2.
constexpr Vec3 kSwayOnly = {0.0, 0.5, 0.0};
constexpr Twist kSwayRelative = {4.0, 0.5, 0.5, 0.25, -0.125, 0.0625};

Environment MakeCurrent(std::string_view frame_id, const Vec3 &velocity) {
  Environment environment;
  environment.current_frame_id = std::string(frame_id);
  environment.current_velocity_m_s = velocity;
  environment.gravity_m_s2 = 9.81;
  environment.fluid_density_kg_m3 = 1025.0;
  return environment;
}

Quaternion Yaw90() {
  const double half_sqrt2 = std::numbers::sqrt2 / 2.0;
  return {0.0, 0.0, half_sqrt2, half_sqrt2};
}

Quaternion Negate(const Quaternion &quaternion) {
  return {-quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]};
}

StatusOr<CurrentRelativeTwist> Compute(const Environment &environment,
                                       FrameId pose_frame,
                                       const Quaternion &orientation,
                                       const Twist &twist) {
  return ComputeWaterCurrentRelativeVelocity(environment, pose_frame,
                                             orientation, twist);
}

void ExpectBitIdentical(const Twist &actual, const Twist &expected) {
  EXPECT_EQ(0, std::memcmp(actual.data(), expected.data(),
                           actual.size() * sizeof(double)));
}

void ExpectNearTwist(const Twist &actual, const Twist &expected) {
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_NEAR(actual[i], expected[i], kAbsTolerance) << "component " << i;
  }
}

void ExpectPositiveZero(const Twist &twist) {
  const double zero = 0.0;
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_TRUE(std::isfinite(twist[i])) << i;
    EXPECT_EQ(0, std::memcmp(&twist[i], &zero, sizeof(double))) << i;
  }
}

void ExpectInvalid(const char *name, std::string_view message,
                   const Environment &environment, FrameId pose_frame,
                   const Quaternion &orientation, const Twist &twist) {
  const StatusOr<CurrentRelativeTwist> result =
      Compute(environment, pose_frame, orientation, twist);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  ExpectPositiveZero(result.value().components);
}

TEST(WaterCurrentRelativeVelocity, ZeroCurrentReturnsBodyTwist) {
  const Environment body = MakeCurrent(kBodyFrameId, {0.0, 0.0, 0.0});
  const Environment enu = MakeCurrent(kWorldEnuFrameId, {0.0, 0.0, 0.0});
  const Environment ned = MakeCurrent(kWorldNedFrameId, {0.0, 0.0, 0.0});
  const StatusOr<CurrentRelativeTwist> body_result =
      Compute(body, FrameId::kWorldEnu, kRoll180, kTwist);
  const StatusOr<CurrentRelativeTwist> enu_result =
      Compute(enu, FrameId::kWorldEnu, kYaw180, kTwist);
  const StatusOr<CurrentRelativeTwist> ned_result =
      Compute(ned, FrameId::kWorldNed, kIdentity, kTwist);
  ASSERT_TRUE(body_result.ok());
  ASSERT_TRUE(enu_result.ok());
  ASSERT_TRUE(ned_result.ok());
  EXPECT_EQ(body_result.status().message, std::string_view());
  ExpectBitIdentical(body_result.value().components, kTwist);
  ExpectBitIdentical(enu_result.value().components, kTwist);
  ExpectBitIdentical(ned_result.value().components, kTwist);

  // The calm-water example stores a zero ENU current. Rotation of a zero
  // vector is still zero, so ν_r stays ν.
  const MarineModel model = MakeSixThrusterUuvExample();
  const StatusOr<CurrentRelativeTwist> example =
      Compute(model.environment, FrameId::kWorldEnu, kYaw180, kTwist);
  ASSERT_TRUE(example.ok());
  ExpectBitIdentical(example.value().components, kTwist);
}

TEST(WaterCurrentRelativeVelocity, BodyFrameCurrentDoesNotRotate) {
  const Environment current = MakeCurrent(kBodyFrameId, kCurrent);
  const StatusOr<CurrentRelativeTwist> aligned =
      Compute(current, FrameId::kWorldEnu, kIdentity, kTwist);
  const StatusOr<CurrentRelativeTwist> rolled =
      Compute(current, FrameId::kWorldNed, kRoll180, kTwist);
  const StatusOr<CurrentRelativeTwist> yawed =
      Compute(current, FrameId::kWorldEnu, kYaw180, kTwist);
  ASSERT_TRUE(aligned.ok());
  ASSERT_TRUE(rolled.ok());
  ASSERT_TRUE(yawed.ok());
  ExpectBitIdentical(aligned.value().components, kAlignedRelative);
  ExpectBitIdentical(rolled.value().components, kAlignedRelative);
  ExpectBitIdentical(yawed.value().components, kAlignedRelative);

  // Reversed surge and an isolated sway axis. Angular rates are copied.
  const StatusOr<CurrentRelativeTwist> reversed =
      Compute(MakeCurrent(kBodyFrameId, kReversedSurge), FrameId::kWorldEnu,
              kIdentity, kTwist);
  const StatusOr<CurrentRelativeTwist> sway =
      Compute(MakeCurrent(kBodyFrameId, kSwayOnly), FrameId::kWorldNed, kYaw180,
              kTwist);
  ASSERT_TRUE(reversed.ok());
  ASSERT_TRUE(sway.ok());
  ExpectBitIdentical(reversed.value().components, kReversedRelative);
  ExpectBitIdentical(sway.value().components, kSwayRelative);
  EXPECT_EQ(reversed.value().components[3], kTwist[3]);
  EXPECT_EQ(reversed.value().components[4], kTwist[4]);
  EXPECT_EQ(reversed.value().components[5], kTwist[5]);

  // Equal surge components cancel to +0. The other components stay ν.
  const StatusOr<CurrentRelativeTwist> cancel =
      Compute(MakeCurrent(kBodyFrameId, {4.0, 0.0, 0.0}), FrameId::kWorldEnu,
              kIdentity, kTwist);
  ASSERT_TRUE(cancel.ok());
  const double positive_zero = 0.0;
  EXPECT_EQ(0, std::memcmp(&cancel.value().components[0], &positive_zero,
                           sizeof(double)));
  EXPECT_EQ(cancel.value().components[1], kTwist[1]);
  EXPECT_EQ(cancel.value().components[2], kTwist[2]);
}

TEST(WaterCurrentRelativeVelocity, BodyAlignedWorldCurrentMatchesHandExample) {
  // R = I in both world frames. v_c^b is the supplied vector. ENU and NED
  // stay bit-identical: the frame id does not flip axes.
  const Environment enu = MakeCurrent(kWorldEnuFrameId, kCurrent);
  const Environment ned = MakeCurrent(kWorldNedFrameId, kCurrent);
  const StatusOr<CurrentRelativeTwist> enu_result =
      Compute(enu, FrameId::kWorldEnu, kIdentity, kTwist);
  const StatusOr<CurrentRelativeTwist> ned_result =
      Compute(ned, FrameId::kWorldNed, kIdentity, kTwist);
  const StatusOr<CurrentRelativeTwist> body_result =
      Compute(MakeCurrent(kBodyFrameId, kCurrent), FrameId::kWorldEnu,
              kIdentity, kTwist);
  ASSERT_TRUE(enu_result.ok());
  ASSERT_TRUE(ned_result.ok());
  ASSERT_TRUE(body_result.ok());
  ExpectBitIdentical(enu_result.value().components, kAlignedRelative);
  ExpectBitIdentical(ned_result.value().components, kAlignedRelative);
  ExpectBitIdentical(enu_result.value().components,
                     ned_result.value().components);
  ExpectBitIdentical(enu_result.value().components,
                     body_result.value().components);

  // A pose labeled ENU with a NED current still applies R^T and does not
  // convert the vector. NED +z here stays +z in the body frame. An
  // ENU/NED axis flip would have changed the sign of heave.
  const Environment mismatched = MakeCurrent(kWorldNedFrameId, {0.0, 0.0, 1.0});
  const StatusOr<CurrentRelativeTwist> unflipped =
      Compute(mismatched, FrameId::kWorldEnu, kIdentity, kTwist);
  ASSERT_TRUE(unflipped.ok());
  ExpectBitIdentical(unflipped.value().components,
                     Twist{4.0, 1.0, -0.5, 0.25, -0.125, 0.0625});
}

TEST(WaterCurrentRelativeVelocity, RotatedWorldCurrentMatchesHandExample) {
  const Environment enu = MakeCurrent(kWorldEnuFrameId, kCurrent);
  const Environment ned = MakeCurrent(kWorldNedFrameId, kCurrent);

  const StatusOr<CurrentRelativeTwist> yaw_enu =
      Compute(enu, FrameId::kWorldEnu, kYaw180, kTwist);
  const StatusOr<CurrentRelativeTwist> yaw_ned =
      Compute(ned, FrameId::kWorldNed, kYaw180, kTwist);
  ASSERT_TRUE(yaw_enu.ok());
  ASSERT_TRUE(yaw_ned.ok());
  ExpectBitIdentical(yaw_enu.value().components, kYaw180Relative);
  ExpectBitIdentical(yaw_ned.value().components, kYaw180Relative);

  const StatusOr<CurrentRelativeTwist> roll_enu =
      Compute(enu, FrameId::kWorldEnu, kRoll180, kTwist);
  const StatusOr<CurrentRelativeTwist> roll_ned =
      Compute(ned, FrameId::kWorldNed, kRoll180, kTwist);
  ASSERT_TRUE(roll_enu.ok());
  ASSERT_TRUE(roll_ned.ok());
  ExpectBitIdentical(roll_enu.value().components, kRoll180Relative);
  ExpectBitIdentical(roll_ned.value().components, kRoll180Relative);

  // Same physical current, 2 m/s north. NED is the exact 180 deg roll.
  // ENU is a 90 deg yaw and matches within 1e-12. Angular ν2 is unchanged.
  const StatusOr<CurrentRelativeTwist> north_ned =
      Compute(MakeCurrent(kWorldNedFrameId, kNorthNed), FrameId::kWorldNed,
              kRoll180, kTwist);
  const StatusOr<CurrentRelativeTwist> north_enu =
      Compute(MakeCurrent(kWorldEnuFrameId, kNorthEnu), FrameId::kWorldEnu,
              Yaw90(), kTwist);
  ASSERT_TRUE(north_ned.ok());
  ASSERT_TRUE(north_enu.ok());
  ExpectBitIdentical(north_ned.value().components, kNorthRelative);
  ExpectNearTwist(north_enu.value().components, kNorthRelative);
  ExpectNearTwist(north_enu.value().components, north_ned.value().components);
  EXPECT_NEAR(north_enu.value().components[3], kTwist[3], kAbsTolerance);
  EXPECT_NEAR(north_enu.value().components[4], kTwist[4], kAbsTolerance);
  EXPECT_NEAR(north_enu.value().components[5], kTwist[5], kAbsTolerance);
}

TEST(WaterCurrentRelativeVelocity, QuaternionSignIsTheSameRotation) {
  const Environment enu = MakeCurrent(kWorldEnuFrameId, kCurrent);
  const StatusOr<CurrentRelativeTwist> positive =
      Compute(enu, FrameId::kWorldEnu, kYaw180, kTwist);
  const StatusOr<CurrentRelativeTwist> negative =
      Compute(enu, FrameId::kWorldEnu, Negate(kYaw180), kTwist);
  ASSERT_TRUE(positive.ok());
  ASSERT_TRUE(negative.ok());
  ExpectBitIdentical(positive.value().components, kYaw180Relative);
  ExpectBitIdentical(negative.value().components, positive.value().components);
}

TEST(WaterCurrentRelativeVelocity, DoesNotComputeForceOrWrench) {
  // The return value is a 6-vector twist. It has no force or torque field.
  static_assert(std::is_standard_layout_v<CurrentRelativeTwist>);
  static_assert(sizeof(CurrentRelativeTwist) ==
                sizeof(std::array<double, kSpatialDof>));

  Environment current = MakeCurrent(kBodyFrameId, kCurrent);
  const StatusOr<CurrentRelativeTwist> baseline =
      Compute(current, FrameId::kWorldEnu, kIdentity, kTwist);
  ASSERT_TRUE(baseline.ok());
  ExpectBitIdentical(baseline.value().components, kAlignedRelative);

  // Gravity and density build restoring force. They are unread here, so a
  // non-finite pair does not change ν_r and does not produce a wrench.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  current.gravity_m_s2 = nan;
  current.fluid_density_kg_m3 = std::numeric_limits<double>::infinity();
  const StatusOr<CurrentRelativeTwist> unread =
      Compute(current, FrameId::kWorldEnu, kIdentity, kTwist);
  ASSERT_TRUE(unread.ok());
  ExpectBitIdentical(unread.value().components, baseline.value().components);

  Damping damping;
  for (int i = 0; i < kSpatialDof; ++i) {
    damping.linear_coefficients[i * kSpatialDof + i] = 4.0;
  }
  const StatusOr<DampingWrench> drag =
      ComputeLinearQuadraticDampingWrench(damping, baseline.value().components);
  ASSERT_TRUE(drag.ok());
  EXPECT_LT(drag.value().components[0], 0.0);
  EXPECT_GT(baseline.value().components[0], 0.0);
  EXPECT_NE(0, std::memcmp(drag.value().components.data(),
                           baseline.value().components.data(),
                           kSpatialDof * sizeof(double)));

  const MarineModel model = MakeSixThrusterUuvExample();
  const StatusOr<RestoringWrench> restoring =
      ComputeGravityBuoyancyRestoringWrench(model.mass_inertia, model.buoyancy,
                                            model.centers, model.environment,
                                            FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(restoring.ok());
  EXPECT_NE(0, std::memcmp(restoring.value().components.data(),
                           baseline.value().components.data(),
                           kSpatialDof * sizeof(double)));

  // Evaluate still reports no model wrench. This helper is not wired in.
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = kIdentity;
  state.body_twist = kTwist;
  EnvironmentRt environment;
  environment.gravity_m_s2 = 9.81;
  environment.fluid_density_kg_m3 = 1025.0;
  environment.current_frame = FrameId::kWorldEnu;
  environment.current_velocity_m_s = {kCurrent.x, kCurrent.y, kCurrent.z};
  const ZeroForceDynamics dynamics;
  const StatusOr<intrinsic::vehicle::dynamics::DynamicsResult> evaluated =
      dynamics.Evaluate(state, BodyWrenchRt{}, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  for (double acceleration : evaluated.value().derivative.body_acceleration) {
    EXPECT_EQ(acceleration, 0.0);
  }
  EXPECT_EQ(evaluated.value().diagnostics.model_force_n,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(evaluated.value().diagnostics.model_torque_n_m,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_FALSE(evaluated.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(evaluated.value().diagnostics.allocation_invoked);
}

TEST(WaterCurrentRelativeVelocity, InvalidInputsReturnStatusAndZeroTwist) {
  const Environment valid = MakeCurrent(kWorldEnuFrameId, kCurrent);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  ExpectInvalid("nan current", kCurrentNonFiniteMessage,
                MakeCurrent(kWorldEnuFrameId, {nan, 0.0, 0.0}),
                FrameId::kWorldEnu, kIdentity, kTwist);
  ExpectInvalid("infinite current", kCurrentNonFiniteMessage,
                MakeCurrent(kBodyFrameId, {0.0, -inf, 0.0}), FrameId::kWorldNed,
                kRoll180, kTwist);
  ExpectInvalid("nan current z", kCurrentNonFiniteMessage,
                MakeCurrent(kWorldNedFrameId, {1.0, -2.0, nan}),
                FrameId::kWorldNed, kIdentity, kTwist);

  ExpectInvalid("empty frame", kCurrentFrameEmptyMessage,
                MakeCurrent("", kCurrent), FrameId::kWorldEnu, kIdentity,
                kTwist);
  ExpectInvalid("robot frame", kCurrentFrameMessage,
                MakeCurrent("robot", kCurrent), FrameId::kWorldEnu, kIdentity,
                kTwist);
  ExpectInvalid("uppercase frame", kCurrentFrameMessage,
                MakeCurrent("WORLD_ENU", kCurrent), FrameId::kWorldEnu,
                kIdentity, kTwist);
  ExpectInvalid("bare enu", kCurrentFrameMessage, MakeCurrent("enu", kCurrent),
                FrameId::kWorldEnu, kIdentity, kTwist);

  ExpectInvalid("body pose frame", kPoseFrameMessage, valid, FrameId::kBody,
                kIdentity, kTwist);
  ExpectInvalid("unspecified pose frame", kPoseFrameMessage, valid,
                FrameId::kUnspecified, kIdentity, kTwist);

  ExpectInvalid("nan quaternion", kOrientationNonFiniteMessage, valid,
                FrameId::kWorldEnu, {nan, 0.0, 0.0, 1.0}, kTwist);
  ExpectInvalid("infinite quaternion", kOrientationNonFiniteMessage, valid,
                FrameId::kWorldNed, {0.0, inf, 0.0, 0.0}, kTwist);
  ExpectInvalid("zero quaternion", kOrientationUnitMessage, valid,
                FrameId::kWorldEnu, {0.0, 0.0, 0.0, 0.0}, kTwist);
  ExpectInvalid("stretched quaternion", kOrientationUnitMessage, valid,
                FrameId::kWorldEnu, {0.0, 0.0, 0.0, 1.0 + 1e-6}, kTwist);

  Twist nan_twist = kTwist;
  nan_twist[5] = nan;
  ExpectInvalid("nan yaw", kBodyTwistNonFiniteMessage, valid,
                FrameId::kWorldEnu, kIdentity, nan_twist);
  Twist inf_twist = kTwist;
  inf_twist[0] = inf;
  ExpectInvalid("infinite surge", kBodyTwistNonFiniteMessage, valid,
                FrameId::kWorldEnu, kIdentity, inf_twist);

  // A difference of 1e-12 on w is inside the 1e-9 unit tolerance. A zero
  // current keeps ν.
  const StatusOr<CurrentRelativeTwist> near_unit =
      Compute(MakeCurrent(kWorldEnuFrameId, {0.0, 0.0, 0.0}),
              FrameId::kWorldEnu, {0.0, 0.0, 0.0, 1.0 + 1e-12}, kTwist);
  ASSERT_TRUE(near_unit.ok());
  ExpectBitIdentical(near_unit.value().components, kTwist);
  EXPECT_GT(kUnitQuaternionTolerance, 1e-12);

  // Category order: current, frame, pose, orientation, twist, then ν_r.
  ExpectInvalid("current before empty frame", kCurrentNonFiniteMessage,
                MakeCurrent("", {nan, 0.0, 0.0}), FrameId::kBody,
                {nan, 0.0, 0.0, 0.0}, nan_twist);
  ExpectInvalid("empty frame before pose", kCurrentFrameEmptyMessage,
                MakeCurrent("", kCurrent), FrameId::kBody, {nan, 0.0, 0.0, 4.0},
                nan_twist);
  ExpectInvalid("frame id before pose", kCurrentFrameMessage,
                MakeCurrent("map", kCurrent), FrameId::kUnspecified,
                {nan, 0.0, 0.0, 1.0}, nan_twist);
  ExpectInvalid("pose frame before orientation", kPoseFrameMessage, valid,
                FrameId::kBody, {nan, 0.0, 0.0, 4.0}, nan_twist);
  ExpectInvalid("orientation finite before unit", kOrientationNonFiniteMessage,
                valid, FrameId::kWorldEnu, {nan, 0.0, 0.0, 4.0}, nan_twist);
  ExpectInvalid("unit before twist", kOrientationUnitMessage, valid,
                FrameId::kWorldEnu, {0.0, 0.0, 0.0, 2.0}, nan_twist);

  Twist overflow = kTwist;
  overflow[0] = 1.0e308;
  Environment opposing = MakeCurrent(kBodyFrameId, {-1.0e308, 0.0, 0.0});
  ExpectInvalid("relative overflow", kRelativeVelocityNonFiniteMessage,
                opposing, FrameId::kWorldEnu, kIdentity, overflow);
}

TEST(WaterCurrentRelativeVelocity, RepeatedEvaluationIsBitIdentical) {
  const Environment ned = MakeCurrent(kWorldNedFrameId, kNorthNed);
  const StatusOr<CurrentRelativeTwist> first =
      Compute(ned, FrameId::kWorldNed, kRoll180, kTwist);
  const StatusOr<CurrentRelativeTwist> second =
      Compute(ned, FrameId::kWorldNed, kRoll180, kTwist);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  ExpectBitIdentical(first.value().components, second.value().components);
  ExpectBitIdentical(first.value().components, kNorthRelative);

  const Environment enu = MakeCurrent(kWorldEnuFrameId, kNorthEnu);
  const Quaternion yaw90 = Yaw90();
  const StatusOr<CurrentRelativeTwist> enu_first =
      Compute(enu, FrameId::kWorldEnu, yaw90, kTwist);
  const StatusOr<CurrentRelativeTwist> enu_second =
      Compute(enu, FrameId::kWorldEnu, yaw90, kTwist);
  ASSERT_TRUE(enu_first.ok());
  ASSERT_TRUE(enu_second.ok());
  ExpectBitIdentical(enu_first.value().components,
                     enu_second.value().components);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<CurrentRelativeTwist> bad_first =
      Compute(MakeCurrent(kWorldEnuFrameId, {0.0, nan, 0.0}),
              FrameId::kWorldEnu, kIdentity, kTwist);
  const StatusOr<CurrentRelativeTwist> bad_second =
      Compute(MakeCurrent(kWorldEnuFrameId, {0.0, nan, 0.0}),
              FrameId::kWorldEnu, kIdentity, kTwist);
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.status().message, kCurrentNonFiniteMessage);
  EXPECT_EQ(bad_first.status().code, DynamicsErrorCode::kInvalidArgument);
  ExpectBitIdentical(bad_first.value().components,
                     bad_second.value().components);
  ExpectPositiveZero(bad_first.value().components);
}

TEST(WaterCurrentRelativeVelocity, EvaluateDoesNotApplyRelativeVelocity) {
  const StatusOr<CurrentRelativeTwist> relative =
      Compute(MakeCurrent(kWorldEnuFrameId, kCurrent), FrameId::kWorldEnu,
              kIdentity, kTwist);
  ASSERT_TRUE(relative.ok());
  EXPECT_NE(relative.value().components[0], kTwist[0]);

  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = kIdentity;
  state.body_twist = kTwist;
  BodyWrenchRt actuator;
  actuator.force_n = {40.0, -5.0, 12.0};
  actuator.torque_n_m = {1.0, -2.0, 3.0};
  EnvironmentRt environment;
  environment.gravity_m_s2 = 9.81;
  environment.fluid_density_kg_m3 = 1025.0;
  environment.current_frame = FrameId::kBody;
  environment.current_velocity_m_s = {3.0, -1.0, 0.5};
  const ZeroForceDynamics model;
  const StatusOr<intrinsic::vehicle::dynamics::DynamicsResult> evaluated =
      model.Evaluate(state, actuator, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  for (double acceleration : evaluated.value().derivative.body_acceleration) {
    EXPECT_EQ(acceleration, 0.0);
  }
  EXPECT_EQ(evaluated.value().diagnostics.model_force_n,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(evaluated.value().diagnostics.model_torque_n_m,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_FALSE(evaluated.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(evaluated.value().diagnostics.allocation_invoked);
  // Pose rate still follows the body twist. Current is not subtracted.
  EXPECT_EQ(evaluated.value().derivative.position_dot_m_s,
            (std::array<double, 3>{kTwist[0], kTwist[1], kTwist[2]}));
}

} // namespace
