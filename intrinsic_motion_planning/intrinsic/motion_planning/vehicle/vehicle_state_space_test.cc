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

#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <vector>

#include "Eigen/Geometry"
#include "gtest/gtest.h"
#include "intrinsic/eigenmath/interpolation.h"
#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

constexpr double kTol = 1e-9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kHalfSqrt2 = std::numbers::sqrt2 / 2.0;

VehiclePlanningState MakeState(embodiment::Vec3 p, embodiment::Quaternion q,
                               BodyVector v = BodyVector{}) {
  VehiclePlanningState state;
  state.position = p;
  state.orientation = q;
  state.twist = v;
  return state;
}

// Rotation of `angle` radians about the z axis.
embodiment::Quaternion YawQuaternion(double angle) {
  return embodiment::Quaternion{0, 0, std::sin(angle / 2), std::cos(angle / 2)};
}

embodiment::Quaternion Negated(embodiment::Quaternion q) {
  return embodiment::Quaternion{-q.x, -q.y, -q.z, -q.w};
}

void ExpectStateEq(const VehiclePlanningState& actual,
                   const VehiclePlanningState& expected, double tol) {
  EXPECT_NEAR(actual.position.x, expected.position.x, tol);
  EXPECT_NEAR(actual.position.y, expected.position.y, tol);
  EXPECT_NEAR(actual.position.z, expected.position.z, tol);
  EXPECT_NEAR(actual.orientation.x, expected.orientation.x, tol);
  EXPECT_NEAR(actual.orientation.y, expected.orientation.y, tol);
  EXPECT_NEAR(actual.orientation.z, expected.orientation.z, tol);
  EXPECT_NEAR(actual.orientation.w, expected.orientation.w, tol);
  EXPECT_NEAR(actual.twist.linear_x, expected.twist.linear_x, tol);
  EXPECT_NEAR(actual.twist.linear_y, expected.twist.linear_y, tol);
  EXPECT_NEAR(actual.twist.linear_z, expected.twist.linear_z, tol);
  EXPECT_NEAR(actual.twist.angular_x, expected.twist.angular_x, tol);
  EXPECT_NEAR(actual.twist.angular_y, expected.twist.angular_y, tol);
  EXPECT_NEAR(actual.twist.angular_z, expected.twist.angular_z, tol);
}

void ExpectStateExactlyEq(const VehiclePlanningState& actual,
                          const VehiclePlanningState& expected) {
  EXPECT_EQ(actual.position.x, expected.position.x);
  EXPECT_EQ(actual.position.y, expected.position.y);
  EXPECT_EQ(actual.position.z, expected.position.z);
  EXPECT_EQ(actual.orientation.x, expected.orientation.x);
  EXPECT_EQ(actual.orientation.y, expected.orientation.y);
  EXPECT_EQ(actual.orientation.z, expected.orientation.z);
  EXPECT_EQ(actual.orientation.w, expected.orientation.w);
  EXPECT_EQ(actual.twist.linear_x, expected.twist.linear_x);
  EXPECT_EQ(actual.twist.linear_y, expected.twist.linear_y);
  EXPECT_EQ(actual.twist.linear_z, expected.twist.linear_z);
  EXPECT_EQ(actual.twist.angular_x, expected.twist.angular_x);
  EXPECT_EQ(actual.twist.angular_y, expected.twist.angular_y);
  EXPECT_EQ(actual.twist.angular_z, expected.twist.angular_z);
}

// Fixture pair A: identity at the origin to a 90 degree yaw.
VehiclePlanningState FixtureA() {
  return MakeState({0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0, 0, 0, 0});
}
VehiclePlanningState FixtureB() {
  return MakeState({2, -4, 6}, {0, 0, kHalfSqrt2, kHalfSqrt2},
                   {1, 2, 3, 0.5, -1, 0.25});
}

TEST(InterpolateTest, EndpointsReturnInputs) {
  const VehiclePlanningState a = FixtureA();
  const VehiclePlanningState b = FixtureB();
  const InterpolateResult at_a = Interpolate(a, b, 0.0);
  ASSERT_EQ(at_a.error, StateSpaceError::kOk);
  ExpectStateExactlyEq(at_a.state, a);
  const InterpolateResult at_b = Interpolate(a, b, 1.0);
  ASSERT_EQ(at_b.error, StateSpaceError::kOk);
  ExpectStateExactlyEq(at_b.state, b);
}

TEST(InterpolateTest, EndpointReturnsBWhenShortestPathFlipsSign) {
  const VehiclePlanningState a = FixtureA();
  const VehiclePlanningState b =
      MakeState({1, 1, 1}, Negated(YawQuaternion(1.0)));
  const InterpolateResult at_b = Interpolate(a, b, 1.0);
  ASSERT_EQ(at_b.error, StateSpaceError::kOk);
  ExpectStateExactlyEq(at_b.state, b);
}

TEST(InterpolateTest, MidpointGolden) {
  // Position and twist are the componentwise mean. Orientation is a 45 degree
  // yaw: (0, 0, sin(22.5 deg), cos(22.5 deg)).
  const InterpolateResult result = Interpolate(FixtureA(), FixtureB(), 0.5);
  ASSERT_EQ(result.error, StateSpaceError::kOk);
  const VehiclePlanningState expected =
      MakeState({1, -2, 3}, {0, 0, 0.38268343236508978, 0.92387953251128674},
                {0.5, 1, 1.5, 0.25, -0.5, 0.125});
  ExpectStateEq(result.state, expected, kTol);
}

TEST(InterpolateTest, MidpointGoldenWithNonIdentityStart) {
  // Yaw 30 degrees to yaw 90 degrees; midpoint is yaw 60 degrees.
  const double pi = std::numbers::pi;
  const VehiclePlanningState a =
      MakeState({1, 2, 3}, YawQuaternion(pi / 6), {-1, 0, 1, 0, 0, 2});
  const VehiclePlanningState b =
      MakeState({3, -2, 7}, YawQuaternion(pi / 2), {1, 0, -1, 0, 0, 4});
  const InterpolateResult result = Interpolate(a, b, 0.5);
  ASSERT_EQ(result.error, StateSpaceError::kOk);
  const VehiclePlanningState expected = MakeState(
      {2, 0, 5}, {0, 0, 0.5, 0.86602540378443865}, {0, 0, 0, 0, 0, 3});
  ExpectStateEq(result.state, expected, kTol);
}

TEST(InterpolateTest, QuarterPointUsesSlerpNotNlerp) {
  // Yaw 0 to yaw 180 degrees at u = 0.25 is yaw 45 degrees. A normalized
  // linear blend would land on a different angle (about 37 degrees).
  const double pi = std::numbers::pi;
  const VehiclePlanningState a = MakeState({0, 0, 0}, {0, 0, 0, 1});
  const VehiclePlanningState b = MakeState({0, 0, 0}, YawQuaternion(pi));
  const InterpolateResult result = Interpolate(a, b, 0.25);
  ASSERT_EQ(result.error, StateSpaceError::kOk);
  const embodiment::Quaternion expected = YawQuaternion(pi / 4);
  EXPECT_NEAR(result.state.orientation.z, expected.z, kTol);
  EXPECT_NEAR(result.state.orientation.w, expected.w, kTol);
}

TEST(InterpolateTest, ShortestPathTakesAcuteArcForNegativeDot) {
  // b is the 120 degree yaw written with the opposite sign, so dot(a, b) < 0.
  // The short way is 120 degrees; the long way would be 240 degrees.
  const double pi = std::numbers::pi;
  const double total = 2 * pi / 3;
  const VehiclePlanningState a = MakeState({0, 0, 0}, {0, 0, 0, 1});
  const VehiclePlanningState b =
      MakeState({0, 0, 0}, Negated(YawQuaternion(total)));
  ASSERT_LT(a.orientation.w * b.orientation.w, 0.0);

  double previous_from_a = -1.0;
  double previous_to_b = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= 20; ++i) {
    const double u = i / 20.0;
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    const VehiclePlanningState at_u =
        MakeState({0, 0, 0}, result.state.orientation);
    const double from_a = Distance(a, at_u);
    const double to_b = Distance(at_u, b);
    EXPECT_NEAR(from_a, u * total, kTol) << "u=" << u;
    EXPECT_GT(from_a, previous_from_a) << "u=" << u;
    EXPECT_LT(to_b, previous_to_b) << "u=" << u;
    previous_from_a = from_a;
    previous_to_b = to_b;
  }

  const InterpolateResult mid = Interpolate(a, b, 0.5);
  ASSERT_EQ(mid.error, StateSpaceError::kOk);
  EXPECT_NEAR(mid.state.orientation.z, 0.5, kTol);
  EXPECT_NEAR(mid.state.orientation.w, 0.86602540378443865, kTol);
}

TEST(InterpolateTest, SignFlippedEndpointGivesSameRotationsAsUnflipped) {
  const double pi = std::numbers::pi;
  const VehiclePlanningState a = MakeState({0, 0, 0}, YawQuaternion(0.3));
  const VehiclePlanningState b =
      MakeState({0, 0, 0}, YawQuaternion(0.3 + pi / 2));
  const VehiclePlanningState b_flipped =
      MakeState({0, 0, 0}, Negated(b.orientation));
  for (double u : {0.1, 0.25, 0.5, 0.9}) {
    const InterpolateResult plain = Interpolate(a, b, u);
    const InterpolateResult flipped = Interpolate(a, b_flipped, u);
    ASSERT_EQ(plain.error, StateSpaceError::kOk);
    ASSERT_EQ(flipped.error, StateSpaceError::kOk);
    ExpectStateEq(flipped.state, plain.state, kTol);
  }
}

TEST(InterpolateTest, MatchesEigenmathSlerp) {
  const double angle_a = 0.4;
  const double angle_b = 2.2;
  const Eigen::Quaterniond qa(std::cos(angle_a / 2), 0, 0,
                              std::sin(angle_a / 2));
  const Eigen::Quaterniond qb(std::cos(angle_b / 2), 0, 0,
                              std::sin(angle_b / 2));
  const VehiclePlanningState a = MakeState({0, 0, 0}, YawQuaternion(angle_a));
  const VehiclePlanningState b = MakeState({0, 0, 0}, YawQuaternion(angle_b));
  for (double u : {0.125, 0.5, 0.875}) {
    const Eigen::Quaterniond expected = eigenmath::Interpolate(u, qa, qb);
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    EXPECT_NEAR(result.state.orientation.x, expected.x(), kTol);
    EXPECT_NEAR(result.state.orientation.y, expected.y(), kTol);
    EXPECT_NEAR(result.state.orientation.z, expected.z(), kTol);
    EXPECT_NEAR(result.state.orientation.w, expected.w(), kTol);
  }
}

TEST(InterpolateTest, EqualOrientationsAreStable) {
  const embodiment::Quaternion q = YawQuaternion(0.7);
  const VehiclePlanningState a = MakeState({0, 0, 0}, q);
  const VehiclePlanningState b = MakeState({1, 0, 0}, q);
  const InterpolateResult result = Interpolate(a, b, 0.3);
  ASSERT_EQ(result.error, StateSpaceError::kOk);
  EXPECT_NEAR(result.state.orientation.z, q.z, kTol);
  EXPECT_NEAR(result.state.orientation.w, q.w, kTol);
  EXPECT_NEAR(result.state.position.x, 0.3, kTol);
}

TEST(InterpolateTest, AntipodalPairFoldsToFirstOrientation) {
  // q and -q are the same rotation. After the shortest-path fold the delta is
  // identity, so every sample is a's orientation.
  const embodiment::Quaternion q = YawQuaternion(1.1);
  const VehiclePlanningState a = MakeState({0, 0, 0}, q);
  const VehiclePlanningState b = MakeState({0, 0, 0}, Negated(q));
  for (double u : {0.1, 0.5, 0.9}) {
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    EXPECT_NEAR(result.state.orientation.x, q.x, kTol);
    EXPECT_NEAR(result.state.orientation.y, q.y, kTol);
    EXPECT_NEAR(result.state.orientation.z, q.z, kTol);
    EXPECT_NEAR(result.state.orientation.w, q.w, kTol);
  }
}

TEST(InterpolateTest, NearPiBranchMatchesEigenmath) {
  // 180 degrees about x: dot(a, b) is 0, which eigenmath does not flip, so the
  // short-arc tie breaks toward +90 degrees about the b axis. Negating b picks
  // the other branch (-90 degrees). This locks the deterministic choice.
  const VehiclePlanningState a = MakeState({0, 0, 0}, {0, 0, 0, 1});
  const VehiclePlanningState b = MakeState({0, 0, 0}, {1, 0, 0, 0});
  const VehiclePlanningState b_negated = MakeState({0, 0, 0}, {-1, 0, 0, 0});

  const InterpolateResult positive = Interpolate(a, b, 0.5);
  ASSERT_EQ(positive.error, StateSpaceError::kOk);
  EXPECT_NEAR(positive.state.orientation.x, kHalfSqrt2, kTol);
  EXPECT_NEAR(positive.state.orientation.y, 0.0, kTol);
  EXPECT_NEAR(positive.state.orientation.z, 0.0, kTol);
  EXPECT_NEAR(positive.state.orientation.w, kHalfSqrt2, kTol);

  const InterpolateResult negative = Interpolate(a, b_negated, 0.5);
  ASSERT_EQ(negative.error, StateSpaceError::kOk);
  EXPECT_NEAR(negative.state.orientation.x, -kHalfSqrt2, kTol);
  EXPECT_NEAR(negative.state.orientation.w, kHalfSqrt2, kTol);

  const Eigen::Quaterniond expected = eigenmath::Interpolate(
      0.5, Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(0, 1, 0, 0));
  EXPECT_NEAR(positive.state.orientation.x, expected.x(), kTol);
  EXPECT_NEAR(positive.state.orientation.w, expected.w(), kTol);
}

TEST(InterpolateTest, MixParameterOutsideUnitIntervalIsRejected) {
  const VehiclePlanningState a = FixtureA();
  const VehiclePlanningState b = FixtureB();
  for (double u : {-1e-12, -0.5, 1.0 + 1e-12, 2.0, kNaN, kInf, -kInf}) {
    EXPECT_EQ(Interpolate(a, b, u).error, StateSpaceError::kMixParameter)
        << "u=" << u;
  }
}

TEST(InterpolateTest, NonFiniteInputsAreRejected) {
  const VehiclePlanningState good = FixtureA();
  const std::vector<double> bad_values = {kNaN, kInf, -kInf};
  for (double bad : bad_values) {
    VehiclePlanningState p = good;
    p.position.y = bad;
    VehiclePlanningState q = good;
    q.orientation.x = bad;
    VehiclePlanningState t = good;
    t.twist.angular_z = bad;
    for (const VehiclePlanningState& state : {p, q, t}) {
      EXPECT_EQ(Interpolate(state, good, 0.5).error,
                StateSpaceError::kNonFinite);
      EXPECT_EQ(Interpolate(good, state, 0.5).error,
                StateSpaceError::kNonFinite);
    }
  }
}

TEST(InterpolateTest, NonUnitOrientationIsRejectedNotRenormalized) {
  const VehiclePlanningState good = FixtureA();
  VehiclePlanningState scaled = good;
  scaled.orientation = {0, 0, 0, 2};
  EXPECT_EQ(Interpolate(scaled, good, 0.5).error, StateSpaceError::kQuaternion);
  EXPECT_EQ(Interpolate(good, scaled, 0.5).error, StateSpaceError::kQuaternion);
  // Endpoints are checked too.
  EXPECT_EQ(Interpolate(scaled, good, 0.0).error, StateSpaceError::kQuaternion);
  VehiclePlanningState zero = good;
  zero.orientation = {0, 0, 0, 0};
  EXPECT_EQ(Interpolate(good, zero, 1.0).error, StateSpaceError::kQuaternion);
  VehiclePlanningState slightly_off = good;
  slightly_off.orientation = {0, 0, 0, 1.0 + 1e-8};
  EXPECT_EQ(Interpolate(slightly_off, good, 0.5).error,
            StateSpaceError::kQuaternion);
}

TEST(InterpolateTest, CheckOrderIsMixThenFiniteThenQuaternion) {
  VehiclePlanningState bad = FixtureA();
  bad.position.x = kNaN;
  bad.orientation = {0, 0, 0, 2};
  EXPECT_EQ(Interpolate(bad, FixtureB(), 2.0).error,
            StateSpaceError::kMixParameter);
  EXPECT_EQ(Interpolate(bad, FixtureB(), 0.5).error,
            StateSpaceError::kNonFinite);
  bad.position.x = 0;
  EXPECT_EQ(Interpolate(bad, FixtureB(), 0.5).error,
            StateSpaceError::kQuaternion);
}

TEST(InterpolateTest, AcceptsInputsWithinNormTolerance) {
  VehiclePlanningState a = FixtureA();
  a.orientation = {0, 0, 0, 1.0 + 5e-10};
  const InterpolateResult result = Interpolate(a, FixtureB(), 0.5);
  ASSERT_EQ(result.error, StateSpaceError::kOk);
  EXPECT_NEAR(embodiment::QuaternionNorm(result.state.orientation), 1.0, 1e-15);
}

TEST(DistanceTest, ZeroForIdenticalStates) {
  EXPECT_EQ(Distance(FixtureB(), FixtureB()), 0.0);
}

TEST(DistanceTest, SameRotationWithOppositeSignIsZeroAngle) {
  const VehiclePlanningState a = MakeState({0, 0, 0}, YawQuaternion(0.8));
  const VehiclePlanningState b =
      MakeState({0, 0, 0}, Negated(YawQuaternion(0.8)));
  EXPECT_NEAR(Distance(a, b), 0.0, kTol);
}

TEST(DistanceTest, CombinesAllTermsWithUnitWeights) {
  const double pi = std::numbers::pi;
  const VehiclePlanningState a = MakeState({0, 0, 0}, {0, 0, 0, 1});
  // Position 2-3-6 is 7 m. Yaw is pi/2 rad. Linear twist 1-2-2 is 3. Angular
  // twist 0-0-4 is 4.
  const VehiclePlanningState b =
      MakeState({2, 3, 6}, YawQuaternion(pi / 2), {1, 2, 2, 0, 0, 4});
  const double expected = std::sqrt(49.0 + (pi / 2) * (pi / 2) + 9.0 + 16.0);
  EXPECT_NEAR(Distance(a, b), expected, kTol);
}

TEST(DistanceTest, AngleIsGeodesicAndAtMostPi) {
  const double pi = std::numbers::pi;
  const VehiclePlanningState a = MakeState({0, 0, 0}, {0, 0, 0, 1});
  EXPECT_NEAR(Distance(a, MakeState({0, 0, 0}, YawQuaternion(pi))), pi, kTol);
  // 270 degrees is the same rotation as -90 degrees.
  EXPECT_NEAR(Distance(a, MakeState({0, 0, 0}, YawQuaternion(1.5 * pi))),
              pi / 2, kTol);
  EXPECT_NEAR(Distance(a, MakeState({0, 0, 0}, YawQuaternion(1e-7))), 1e-7,
              1e-15);
}

TEST(DistanceTest, InfiniteForInvalidInputs) {
  const VehiclePlanningState good = FixtureA();
  VehiclePlanningState nan_position = good;
  nan_position.position.x = kNaN;
  VehiclePlanningState inf_twist = good;
  inf_twist.twist.linear_z = kInf;
  VehiclePlanningState non_unit = good;
  non_unit.orientation = {0, 0, 0, 2};
  for (const VehiclePlanningState& bad : {nan_position, inf_twist, non_unit}) {
    EXPECT_EQ(Distance(good, bad), kInf);
    EXPECT_EQ(Distance(bad, good), kInf);
    EXPECT_EQ(Distance(bad, bad), kInf);
  }
}

TEST(ValidateTest, DefaultBoundsAcceptFiniteUnitStates) {
  EXPECT_EQ(Validate(FixtureB(), VehicleStateBounds{}), StateSpaceError::kOk);
}

TEST(ValidateTest, StructuralDefectsWithDefaultBounds) {
  VehiclePlanningState nan_state = FixtureA();
  nan_state.twist.angular_x = kNaN;
  EXPECT_EQ(Validate(nan_state, VehicleStateBounds{}),
            StateSpaceError::kNonFinite);
  VehiclePlanningState non_unit = FixtureA();
  non_unit.orientation = {0, 0, 0, 0.5};
  EXPECT_EQ(Validate(non_unit, VehicleStateBounds{}),
            StateSpaceError::kQuaternion);
}

VehicleStateBounds EngagedBounds() {
  VehicleStateBounds bounds;
  bounds.position_limits_present = true;
  bounds.position_min = {-10, -10, -20};
  bounds.position_max = {10, 10, 0};
  bounds.max_linear_speed_present = true;
  bounds.max_linear_speed_m_s = 5.0;
  bounds.max_angular_speed_present = true;
  bounds.max_angular_speed_rad_s = 1.0;
  return bounds;
}

TEST(ValidateTest, InsideBoundsIsOk) {
  const VehiclePlanningState state =
      MakeState({1, -2, -3}, {0, 0, 0, 1}, {3, 4, 0, 0, 0.6, 0.8});
  EXPECT_EQ(Validate(state, EngagedBounds()), StateSpaceError::kOk);
}

TEST(ValidateTest, PositionLimitsAreInclusive) {
  VehicleStateBounds bounds = EngagedBounds();
  EXPECT_EQ(Validate(MakeState({-10, 10, 0}, {0, 0, 0, 1}), bounds),
            StateSpaceError::kOk);
  EXPECT_EQ(Validate(MakeState({-10.0001, 0, 0}, {0, 0, 0, 1}), bounds),
            StateSpaceError::kBounds);
  EXPECT_EQ(Validate(MakeState({0, 10.0001, 0}, {0, 0, 0, 1}), bounds),
            StateSpaceError::kBounds);
  EXPECT_EQ(Validate(MakeState({0, 0, 0.0001}, {0, 0, 0, 1}), bounds),
            StateSpaceError::kBounds);
  EXPECT_EQ(Validate(MakeState({0, 0, -20.0001}, {0, 0, 0, 1}), bounds),
            StateSpaceError::kBounds);
}

TEST(ValidateTest, SpeedLimitsAreInclusive) {
  const VehicleStateBounds bounds = EngagedBounds();
  EXPECT_EQ(
      Validate(MakeState({0, 0, 0}, {0, 0, 0, 1}, {3, 4, 0, 0, 0, 1}), bounds),
      StateSpaceError::kOk);
  EXPECT_EQ(Validate(MakeState({0, 0, 0}, {0, 0, 0, 1}, {3, 4, 0.01, 0, 0, 0}),
                     bounds),
            StateSpaceError::kBounds);
  EXPECT_EQ(Validate(MakeState({0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0, 1, 0, 0.01}),
                     bounds),
            StateSpaceError::kBounds);
}

TEST(ValidateTest, AbsentLimitsDoNotConstrainThatAxis) {
  VehicleStateBounds bounds;
  bounds.max_linear_speed_present = true;
  bounds.max_linear_speed_m_s = 1.0;
  // Position is far outside the (absent) box; angular rate is unconstrained.
  const VehiclePlanningState state =
      MakeState({1e6, -1e6, 1e6}, {0, 0, 0, 1}, {0, 0, 0, 100, 100, 100});
  EXPECT_EQ(Validate(state, bounds), StateSpaceError::kOk);
}

TEST(ValidateTest, ZeroSpeedLimitAllowsOnlyZeroTwist) {
  VehicleStateBounds bounds;
  bounds.max_linear_speed_present = true;
  bounds.max_linear_speed_m_s = 0.0;
  EXPECT_EQ(Validate(FixtureA(), bounds), StateSpaceError::kOk);
  EXPECT_EQ(Validate(MakeState({0, 0, 0}, {0, 0, 0, 1}, {1e-3, 0, 0, 0, 0, 0}),
                     bounds),
            StateSpaceError::kBounds);
}

TEST(ValidateTest, BadBoundsConfiguration) {
  const VehiclePlanningState state = FixtureA();

  VehicleStateBounds inverted = EngagedBounds();
  inverted.position_min.y = 11;
  EXPECT_EQ(Validate(state, inverted), StateSpaceError::kBadBounds);

  VehicleStateBounds nan_limit = EngagedBounds();
  nan_limit.position_max.z = kNaN;
  EXPECT_EQ(Validate(state, nan_limit), StateSpaceError::kBadBounds);

  for (double bad : {-1.0, kNaN, kInf, -kInf}) {
    VehicleStateBounds linear = EngagedBounds();
    linear.max_linear_speed_m_s = bad;
    EXPECT_EQ(Validate(state, linear), StateSpaceError::kBadBounds)
        << "linear=" << bad;
    VehicleStateBounds angular = EngagedBounds();
    angular.max_angular_speed_rad_s = bad;
    EXPECT_EQ(Validate(state, angular), StateSpaceError::kBadBounds)
        << "angular=" << bad;
  }
}

TEST(ValidateTest, DisengagedInvalidBoundsAreIgnored) {
  VehicleStateBounds bounds;
  bounds.position_min = {1, 1, 1};
  bounds.position_max = {-1, -1, -1};
  bounds.max_linear_speed_m_s = -5.0;
  bounds.max_angular_speed_rad_s = kNaN;
  EXPECT_EQ(Validate(FixtureB(), bounds), StateSpaceError::kOk);
}

TEST(ValidateTest, FirstDefectWins) {
  VehicleStateBounds bounds = EngagedBounds();
  VehiclePlanningState state =
      MakeState({100, 0, 0}, {0, 0, 0, 2}, {100, 0, 0, 0, 0, 0});
  state.position.y = kNaN;
  // Bad bounds beat every state defect.
  VehicleStateBounds bad = bounds;
  bad.max_angular_speed_rad_s = -1;
  EXPECT_EQ(Validate(state, bad), StateSpaceError::kBadBounds);
  // Non-finite beats quaternion, which beats bounds.
  EXPECT_EQ(Validate(state, bounds), StateSpaceError::kNonFinite);
  state.position.y = 0;
  EXPECT_EQ(Validate(state, bounds), StateSpaceError::kQuaternion);
  state.orientation = {0, 0, 0, 1};
  EXPECT_EQ(Validate(state, bounds), StateSpaceError::kBounds);
}

// Property tests. Fixed seed so failures reproduce.
class PropertyTest : public ::testing::Test {
 protected:
  static constexpr int kIterations = 2000;

  double Uniform(double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng_);
  }

  embodiment::Quaternion RandomUnitQuaternion() {
    std::normal_distribution<double> normal(0.0, 1.0);
    embodiment::Quaternion q{normal(rng_), normal(rng_), normal(rng_),
                             normal(rng_)};
    const double norm = embodiment::QuaternionNorm(q);
    if (norm < 1e-3) {
      return embodiment::Quaternion{0, 0, 0, 1};
    }
    return embodiment::Quaternion{q.x / norm, q.y / norm, q.z / norm,
                                  q.w / norm};
  }

  // Position in [-pos, pos], linear speed <= lin, angular speed <= ang.
  VehiclePlanningState RandomState(double pos = 100.0, double lin = 5.0,
                                   double ang = 2.0) {
    VehiclePlanningState state;
    state.position = {Uniform(-pos, pos), Uniform(-pos, pos),
                      Uniform(-pos, pos)};
    state.orientation = RandomUnitQuaternion();
    const double lx = Uniform(-lin, lin) / std::sqrt(3.0);
    const double ly = Uniform(-lin, lin) / std::sqrt(3.0);
    const double lz = Uniform(-lin, lin) / std::sqrt(3.0);
    const double ax = Uniform(-ang, ang) / std::sqrt(3.0);
    const double ay = Uniform(-ang, ang) / std::sqrt(3.0);
    const double az = Uniform(-ang, ang) / std::sqrt(3.0);
    state.twist = BodyVector{lx, ly, lz, ax, ay, az};
    return state;
  }

  std::mt19937_64 rng_{0x1020304050607080ULL};
};

TEST_F(PropertyTest, InterpolateOutputIsFiniteUnitAndValid) {
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState a = RandomState();
    const VehiclePlanningState b = RandomState();
    const double u = Uniform(0.0, 1.0);
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk) << "i=" << i;
    EXPECT_TRUE(embodiment::IsFinite(result.state.position));
    EXPECT_TRUE(embodiment::IsFinite(result.state.orientation));
    EXPECT_TRUE(IsFinite(result.state.twist));
    EXPECT_NEAR(embodiment::QuaternionNorm(result.state.orientation), 1.0,
                1e-12);
    EXPECT_EQ(Validate(result.state, VehicleStateBounds{}),
              StateSpaceError::kOk);
  }
}

TEST_F(PropertyTest, InterpolateEndpointIdentity) {
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState a = RandomState();
    const VehiclePlanningState b = RandomState();
    const InterpolateResult at_a = Interpolate(a, b, 0.0);
    const InterpolateResult at_b = Interpolate(a, b, 1.0);
    ASSERT_EQ(at_a.error, StateSpaceError::kOk);
    ASSERT_EQ(at_b.error, StateSpaceError::kOk);
    ExpectStateEq(at_a.state, a, 1e-12);
    ExpectStateEq(at_b.state, b, 1e-12);
  }
}

TEST_F(PropertyTest, InterpolatePositionAndTwistAreLinear) {
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState a = RandomState();
    const VehiclePlanningState b = RandomState();
    const double u = Uniform(0.0, 1.0);
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    EXPECT_NEAR(result.state.position.x,
                a.position.x + u * (b.position.x - a.position.x), 1e-9);
    EXPECT_NEAR(result.state.twist.angular_z,
                a.twist.angular_z + u * (b.twist.angular_z - a.twist.angular_z),
                1e-9);
  }
}

TEST_F(PropertyTest, InterpolatedRotationAngleIsLinearAndShortest) {
  // theta(a, slerp(a, b, u)) = u * theta(a, b), and the total never exceeds
  // pi, so the interpolation always takes the short way.
  for (int i = 0; i < kIterations; ++i) {
    VehiclePlanningState a = RandomState();
    VehiclePlanningState b = RandomState();
    const double u = Uniform(0.0, 1.0);
    const InterpolateResult result = Interpolate(a, b, u);
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    a.position = b.position = {0, 0, 0};
    a.twist = b.twist = BodyVector{};
    VehiclePlanningState mid = a;
    mid.orientation = result.state.orientation;
    const double total = Distance(a, b);
    EXPECT_LE(total, std::numbers::pi + 1e-12);
    EXPECT_NEAR(Distance(a, mid), u * total, 1e-7) << "i=" << i;
    EXPECT_NEAR(Distance(mid, b), (1.0 - u) * total, 1e-7) << "i=" << i;
  }
}

TEST_F(PropertyTest, DistanceIsSymmetricNonNegativeAndZeroOnIdentity) {
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState a = RandomState();
    const VehiclePlanningState b = RandomState();
    const double ab = Distance(a, b);
    const double ba = Distance(b, a);
    EXPECT_GE(ab, 0.0);
    EXPECT_TRUE(std::isfinite(ab));
    EXPECT_DOUBLE_EQ(ab, ba);
    EXPECT_EQ(Distance(a, a), 0.0);
  }
}

TEST_F(PropertyTest, DistanceSatisfiesTriangleInequality) {
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState a = RandomState();
    const VehiclePlanningState b = RandomState();
    const VehiclePlanningState c = RandomState();
    EXPECT_LE(Distance(a, c), Distance(a, b) + Distance(b, c) + 1e-9);
  }
}

TEST_F(PropertyTest, InterpolantsStayInsideConvexBounds) {
  VehicleStateBounds bounds;
  bounds.position_limits_present = true;
  bounds.position_min = {-100, -100, -100};
  bounds.position_max = {100, 100, 100};
  bounds.max_linear_speed_present = true;
  bounds.max_linear_speed_m_s = 5.0;
  bounds.max_angular_speed_present = true;
  bounds.max_angular_speed_rad_s = 2.0;
  for (int i = 0; i < kIterations; ++i) {
    // Endpoints strictly inside, so rounding cannot push the blend out.
    const VehiclePlanningState a = RandomState(99.0, 4.9, 1.9);
    const VehiclePlanningState b = RandomState(99.0, 4.9, 1.9);
    ASSERT_EQ(Validate(a, bounds), StateSpaceError::kOk);
    ASSERT_EQ(Validate(b, bounds), StateSpaceError::kOk);
    const InterpolateResult result = Interpolate(a, b, Uniform(0.0, 1.0));
    ASSERT_EQ(result.error, StateSpaceError::kOk);
    EXPECT_EQ(Validate(result.state, bounds), StateSpaceError::kOk);
  }
}

TEST_F(PropertyTest, ValidateRejectsStatesOutsideBounds) {
  VehicleStateBounds bounds;
  bounds.position_limits_present = true;
  bounds.position_min = {-10, -10, -10};
  bounds.position_max = {10, 10, 10};
  for (int i = 0; i < kIterations; ++i) {
    const VehiclePlanningState state = RandomState(20.0);
    const embodiment::Vec3& p = state.position;
    const bool inside =
        std::abs(p.x) <= 10 && std::abs(p.y) <= 10 && std::abs(p.z) <= 10;
    EXPECT_EQ(Validate(state, bounds),
              inside ? StateSpaceError::kOk : StateSpaceError::kBounds);
  }
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
