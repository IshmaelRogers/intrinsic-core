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

#include "intrinsic/embodiment/frame_policy.h"

#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::embodiment {
namespace {

constexpr double kTol = 1e-12;

void ExpectVecNear(Vec3 actual, Vec3 expected) {
  EXPECT_NEAR(actual.x, expected.x, kTol);
  EXPECT_NEAR(actual.y, expected.y, kTol);
  EXPECT_NEAR(actual.z, expected.z, kTol);
}

TEST(FramePolicyTest, EnuNedMapsCardinalAxes) {
  ExpectVecNear(WorldVectorEnuToNed(Vec3{1.0, 0.0, 0.0}), Vec3{0.0, 1.0, 0.0});
  ExpectVecNear(WorldVectorEnuToNed(Vec3{0.0, 1.0, 0.0}), Vec3{1.0, 0.0, 0.0});
  ExpectVecNear(WorldVectorEnuToNed(Vec3{0.0, 0.0, 1.0}), Vec3{0.0, 0.0, -1.0});
}

TEST(FramePolicyTest, EnuNedVectorIsAnInvolution) {
  const Vec3 enu{-1.5, 2.25, 0.5};
  const Vec3 ned = WorldVectorEnuToNed(enu);
  EXPECT_DOUBLE_EQ(WorldVectorNedToEnu(ned).x, enu.x);
  EXPECT_DOUBLE_EQ(WorldVectorNedToEnu(ned).y, enu.y);
  EXPECT_DOUBLE_EQ(WorldVectorNedToEnu(ned).z, enu.z);
  EXPECT_DOUBLE_EQ(WorldVectorEnuToNed(ned).x, enu.x);
  EXPECT_DOUBLE_EQ(WorldVectorEnuToNed(ned).y, enu.y);
  EXPECT_DOUBLE_EQ(WorldVectorEnuToNed(ned).z, enu.z);
}

TEST(FramePolicyTest, WorldAdapterIsNotBodyFluToFrd) {
  // ENU +x (east) becomes NED +y. A body FLU-to-FRD map would keep +x.
  const Vec3 east = WorldVectorEnuToNed(Vec3{1.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(east.x, 0.0);
  EXPECT_DOUBLE_EQ(east.y, 1.0);
  EXPECT_DOUBLE_EQ(east.z, 0.0);
}

TEST(FramePolicyTest, FrameIdIsExplicit) {
  EXPECT_TRUE(FrameIdMatches(kWorldEnuFrameId, WorldFrame::kEnu));
  EXPECT_TRUE(FrameIdMatches(kWorldNedFrameId, WorldFrame::kNed));
  EXPECT_FALSE(FrameIdMatches(kWorldEnuFrameId, WorldFrame::kNed));
  EXPECT_FALSE(FrameIdMatches("world", WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("map", WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("odom", WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("base_link", WorldFrame::kNed));
  EXPECT_FALSE(FrameIdMatches("WORLD_ENU", WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("StampedHeader", WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("", WorldFrame::kEnu));
}

TEST(FramePolicyTest, QuaternionStorageOrderAndNedFromEnu) {
  const Quaternion rotation = NedFromEnuWorldRotation();
  const double half_sqrt2 = std::numbers::sqrt2 / 2.0;
  EXPECT_NEAR(rotation.x, half_sqrt2, kTol);
  EXPECT_NEAR(rotation.y, half_sqrt2, kTol);
  EXPECT_NEAR(rotation.z, 0.0, kTol);
  EXPECT_NEAR(rotation.w, 0.0, kTol);
  EXPECT_TRUE(IsNormalized(rotation));
}

TEST(FramePolicyTest, OrientationRoundTripMatchesUpToSign) {
  const Quaternion q_enu{0.0, 0.0, 0.0, 1.0};
  const Quaternion q_ned = WorldOrientationEnuToNed(q_enu);
  EXPECT_TRUE(QuaternionsEquivalent(q_ned, NedFromEnuWorldRotation()));
  EXPECT_TRUE(QuaternionsEquivalent(WorldOrientationNedToEnu(q_ned), q_enu));
}

TEST(FramePolicyTest, BodyVectorAgreesAfterWorldConversion) {
  // q is world-from-body. Mapping the ENU world vector into NED matches
  // rotating the same body vector by the converted orientation.
  const Quaternion q_z90{0.0, 0.0, std::numbers::sqrt2 / 2.0,
                         std::numbers::sqrt2 / 2.0};
  const Quaternion q_ned = WorldOrientationEnuToNed(q_z90);
  const Vec3 forward{1.0, 0.0, 0.0};
  const Vec3 left{0.0, 1.0, 0.0};
  ExpectVecNear(RotateVector(q_z90, forward), Vec3{0.0, 1.0, 0.0});
  ExpectVecNear(WorldVectorEnuToNed(RotateVector(q_z90, forward)),
                RotateVector(q_ned, forward));
  ExpectVecNear(WorldVectorEnuToNed(RotateVector(q_z90, left)),
                RotateVector(q_ned, left));
  EXPECT_TRUE(IsNormalized(q_ned));
}

TEST(FramePolicyTest, HelperDoesNotRenormalize) {
  const Quaternion doubled{0.0, 0.0, 0.0, 2.0};
  EXPECT_FALSE(IsNormalized(doubled));
  const Quaternion converted = WorldOrientationEnuToNed(doubled);
  EXPECT_NEAR(QuaternionNorm(converted), 2.0, kTol);
  EXPECT_FALSE(IsNormalized(converted));
}

TEST(FramePolicyTest, NonFiniteStaysNonFinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Vec3 mapped = WorldVectorEnuToNed(Vec3{nan, 1.0, 2.0});
  EXPECT_FALSE(IsFinite(mapped));
  EXPECT_DOUBLE_EQ(mapped.x, 1.0);
  EXPECT_FALSE(IsFinite(Quaternion{nan, 0.0, 0.0, 1.0}));
  EXPECT_FALSE(IsNormalized(Quaternion{nan, 0.0, 0.0, 1.0}));
}

TEST(StampedHeaderPolicyTest, ValidityAbsentIsNotInvalid) {
  EXPECT_EQ(ClassifyValidity(false, 0), ValidityKind::kAbsent);
  EXPECT_EQ(ClassifyValidity(true, 0), ValidityKind::kUnspecified);
  EXPECT_EQ(ClassifyValidity(true, 2), ValidityKind::kInvalid);
  EXPECT_EQ(ClassifyValidity(true, 1), ValidityKind::kValid);
  EXPECT_EQ(ClassifyValidity(true, 99), ValidityKind::kUnspecified);
  EXPECT_NE(ClassifyValidity(false, 0), ClassifyValidity(true, 2));
  EXPECT_FALSE(SampleAccepted(ValidityKind::kAbsent, true));
  EXPECT_FALSE(SampleAccepted(ValidityKind::kInvalid, true));
  EXPECT_FALSE(SampleAccepted(ValidityKind::kValid, false));
  EXPECT_TRUE(SampleAccepted(ValidityKind::kValid, true));
}

TEST(StampedHeaderPolicyTest, MonotonicAgeIgnoresWallClock) {
  const ClockReading source{1700000000, 250000000};
  const ClockReading receive{1700000001, 0};
  const std::optional<double> age =
      MonotonicAgeSeconds(source, receive, kClockDomainMonotonic);
  ASSERT_TRUE(age.has_value());
  EXPECT_DOUBLE_EQ(*age, 0.75);
  EXPECT_FALSE(
      MonotonicAgeSeconds(source, receive, kClockDomainUtc).has_value());
  EXPECT_FALSE(MonotonicAgeSeconds(source, receive, "wall").has_value());
  EXPECT_FALSE(MonotonicAgeSeconds(source, receive, "").has_value());
  EXPECT_FALSE(MonotonicAgeSeconds(std::nullopt, receive, kClockDomainMonotonic)
                   .has_value());
  EXPECT_FALSE(
      MonotonicAgeSeconds(receive, source, kClockDomainMonotonic).has_value());
  EXPECT_FALSE(MonotonicAgeSeconds(ClockReading{1, -1}, ClockReading{2, 0},
                                   kClockDomainMonotonic)
                   .has_value());
  const std::optional<double> zero =
      MonotonicAgeSeconds(source, source, kClockDomainMonotonic);
  ASSERT_TRUE(zero.has_value());
  EXPECT_DOUBLE_EQ(*zero, 0.0);
}

TEST(StampedHeaderPolicyTest, SequenceAdvancesStrictly) {
  EXPECT_TRUE(SequenceAdvances(41, 42));
  EXPECT_FALSE(SequenceAdvances(42, 42));
  EXPECT_FALSE(SequenceAdvances(42, 41));
  EXPECT_FALSE(SequenceAdvances(std::numeric_limits<uint64_t>::max(), 0));
}

}  // namespace
}  // namespace intrinsic::embodiment
