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

#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"

#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void ExpectControl(const BodyVector& control, int axis, double value) {
  const double actual[6] = {control.linear_x,  control.linear_y,
                            control.linear_z,  control.angular_x,
                            control.angular_y, control.angular_z};
  for (int i = 0; i < 6; ++i) {
    const double want = (i == axis) ? value : 0.0;
    EXPECT_EQ(actual[i], want) << "component " << i;
  }
}

void ExpectPrimitive(const VehicleMotionPrimitive& primitive,
                     const std::string& id, int axis, double value,
                     double duration) {
  EXPECT_EQ(primitive.id, id);
  EXPECT_EQ(primitive.duration_s, duration);
  ExpectControl(primitive.control, axis, value);
}

VehicleMotionPrimitiveSetConfig SurgeConfig() {
  VehicleMotionPrimitiveSetConfig config;
  config.duration_s = 1.0;
  config.include_hover = true;
  config.surge_present = true;
  config.surge_m_s = 0.5;
  return config;
}

TEST(MotionPrimitiveAxis, ComponentIndexIsStable) {
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kSurge), 0);
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kSway), 1);
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kHeave), 2);
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kRoll), 3);
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kPitch), 4);
  EXPECT_EQ(static_cast<int>(MotionPrimitiveAxis::kYaw), 5);
}

TEST(GenerateMotionPrimitives, SurgeOnlyWithHover) {
  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(SurgeConfig());
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 3);
  ExpectPrimitive(result.primitives[0], "hover", -1, 0.0, 1.0);
  ExpectPrimitive(result.primitives[1], "surge_pos", 0, 0.5, 1.0);
  ExpectPrimitive(result.primitives[2], "surge_neg", 0, -0.5, 1.0);
}

TEST(GenerateMotionPrimitives, EmptySetWhenHoverOffAndNoAxes) {
  VehicleMotionPrimitiveSetConfig config;
  config.include_hover = false;
  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  EXPECT_EQ(result.error, MotionPrimitiveSetError::kOk);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(GenerateMotionPrimitives, AllSixAxesFollowLockedOrder) {
  VehicleMotionPrimitiveSetConfig config;
  config.duration_s = 0.25;
  config.include_hover = true;
  config.surge_present = true;
  config.surge_m_s = 1.0;
  config.sway_present = true;
  config.sway_m_s = 2.0;
  config.heave_present = true;
  config.heave_m_s = 3.0;
  config.roll_present = true;
  config.roll_rad_s = 0.1;
  config.pitch_present = true;
  config.pitch_rad_s = 0.2;
  config.yaw_present = true;
  config.yaw_rad_s = 0.3;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 13);

  const std::string ids[] = {"hover",    "surge_pos", "surge_neg", "sway_pos",
                             "sway_neg", "heave_pos", "heave_neg", "roll_pos",
                             "roll_neg", "pitch_pos", "pitch_neg", "yaw_pos",
                             "yaw_neg"};
  const int axes[] = {-1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
  const double values[] = {0.0, 1.0,  -1.0, 2.0,  -2.0, 3.0, -3.0,
                           0.1, -0.1, 0.2,  -0.2, 0.3,  -0.3};
  for (int i = 0; i < 13; ++i) {
    ExpectPrimitive(result.primitives[i], ids[i], axes[i], values[i], 0.25);
  }
}

TEST(GenerateMotionPrimitives, SkipsDisengagedAxes) {
  VehicleMotionPrimitiveSetConfig config;
  config.duration_s = 2.0;
  config.surge_present = true;
  config.surge_m_s = 0.4;
  config.yaw_present = true;
  config.yaw_rad_s = 0.05;
  // A disengaged axis may carry a garbage level. It is not validated.
  config.sway_m_s = kNaN;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 5);
  ExpectPrimitive(result.primitives[0], "hover", -1, 0.0, 2.0);
  ExpectPrimitive(result.primitives[1], "surge_pos", 0, 0.4, 2.0);
  ExpectPrimitive(result.primitives[2], "surge_neg", 0, -0.4, 2.0);
  ExpectPrimitive(result.primitives[3], "yaw_pos", 5, 0.05, 2.0);
  ExpectPrimitive(result.primitives[4], "yaw_neg", 5, -0.05, 2.0);
}

TEST(GenerateMotionPrimitives, BoundaryLevelEqualsEngagedMax) {
  VehicleMotionPrimitiveSetConfig config;
  config.surge_present = true;
  config.surge_m_s = 0.1;
  config.yaw_present = true;
  config.yaw_rad_s = 0.3;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = 0.1;
  config.bounds.max_angular_speed_present = true;
  config.bounds.max_angular_speed_rad_s = 0.3;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 5);
  ExpectPrimitive(result.primitives[1], "surge_pos", 0, 0.1, 1.0);
  ExpectPrimitive(result.primitives[4], "yaw_neg", 5, -0.3, 1.0);
}

TEST(GenerateMotionPrimitives, ZeroSpeedBoundAcceptsHoverOnly) {
  VehicleMotionPrimitiveSetConfig config;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = 0.0;
  config.bounds.max_angular_speed_present = true;
  config.bounds.max_angular_speed_rad_s = 0.0;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 1);
  ExpectPrimitive(result.primitives[0], "hover", -1, 0.0, 1.0);
}

TEST(GenerateMotionPrimitives, LinearLevelAboveMaxRejectsWholeSet) {
  VehicleMotionPrimitiveSetConfig config;
  config.surge_present = true;
  config.surge_m_s = 0.5;
  config.heave_present = true;
  config.heave_m_s = 2.0;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = 1.0;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  EXPECT_EQ(result.error, MotionPrimitiveSetError::kBoundsViolation);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(GenerateMotionPrimitives, AngularLevelAboveMaxRejectsWholeSet) {
  VehicleMotionPrimitiveSetConfig config;
  config.roll_present = true;
  config.roll_rad_s = 0.2;
  config.bounds.max_angular_speed_present = true;
  config.bounds.max_angular_speed_rad_s = 0.1;

  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  EXPECT_EQ(result.error, MotionPrimitiveSetError::kBoundsViolation);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(GenerateMotionPrimitives, LinearAxisIgnoresAngularBoundAndReverse) {
  VehicleMotionPrimitiveSetConfig config;
  config.include_hover = false;
  config.surge_present = true;
  config.surge_m_s = 4.0;
  config.bounds.max_angular_speed_present = true;
  config.bounds.max_angular_speed_rad_s = 0.0;

  GenerateMotionPrimitivesResult result = GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 2);

  config.surge_present = false;
  config.yaw_present = true;
  config.yaw_rad_s = 4.0;
  config.bounds.max_angular_speed_present = false;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = 0.0;
  result = GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 2);
  ExpectPrimitive(result.primitives[0], "yaw_pos", 5, 4.0, 1.0);
}

TEST(GenerateMotionPrimitives, BadDurationIsBadConfig) {
  const double durations[] = {0.0, -1.0, kNaN, kInf, -kInf};
  for (double duration : durations) {
    VehicleMotionPrimitiveSetConfig config = SurgeConfig();
    config.duration_s = duration;
    // A level that would also violate a bound must not hide the duration
    // defect behind kBoundsViolation.
    config.bounds.max_linear_speed_present = true;
    config.bounds.max_linear_speed_m_s = 0.1;
    const GenerateMotionPrimitivesResult result =
        GenerateMotionPrimitives(config);
    EXPECT_EQ(result.error, MotionPrimitiveSetError::kBadConfig) << duration;
    EXPECT_TRUE(result.primitives.empty());
  }
}

TEST(GenerateMotionPrimitives, BadEngagedBoundsAreBadConfig) {
  const double values[] = {-0.1, kNaN, kInf, -kInf};
  for (double value : values) {
    VehicleMotionPrimitiveSetConfig linear = SurgeConfig();
    linear.bounds.max_linear_speed_present = true;
    linear.bounds.max_linear_speed_m_s = value;
    GenerateMotionPrimitivesResult result = GenerateMotionPrimitives(linear);
    EXPECT_EQ(result.error, MotionPrimitiveSetError::kBadConfig) << value;
    EXPECT_TRUE(result.primitives.empty());

    VehicleMotionPrimitiveSetConfig angular = SurgeConfig();
    angular.bounds.max_angular_speed_present = true;
    angular.bounds.max_angular_speed_rad_s = value;
    result = GenerateMotionPrimitives(angular);
    EXPECT_EQ(result.error, MotionPrimitiveSetError::kBadConfig) << value;
    EXPECT_TRUE(result.primitives.empty());
  }
}

TEST(GenerateMotionPrimitives, DisengagedBoundsIgnoreInvalidStoredValues) {
  VehicleMotionPrimitiveSetConfig config = SurgeConfig();
  config.bounds.max_linear_speed_m_s = kNaN;
  config.bounds.max_angular_speed_rad_s = -5.0;
  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(result.error, MotionPrimitiveSetError::kOk);
  EXPECT_EQ(result.primitives.size(), 3);
}

TEST(GenerateMotionPrimitives, NonPositivePresentLevelIsBadConfig) {
  const double levels[] = {0.0, -0.2, kNaN, kInf, -kInf};
  for (double level : levels) {
    VehicleMotionPrimitiveSetConfig config;
    config.duration_s = 1.0;
    config.sway_present = true;
    config.sway_m_s = level;
    const GenerateMotionPrimitivesResult result =
        GenerateMotionPrimitives(config);
    EXPECT_EQ(result.error, MotionPrimitiveSetError::kBadConfig) << level;
    EXPECT_TRUE(result.primitives.empty());
  }
}

TEST(GenerateMotionPrimitives, BadBoundsWinOverAViolatingLevel) {
  VehicleMotionPrimitiveSetConfig config = SurgeConfig();
  config.surge_m_s = 5.0;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = -1.0;
  const GenerateMotionPrimitivesResult result =
      GenerateMotionPrimitives(config);
  EXPECT_EQ(result.error, MotionPrimitiveSetError::kBadConfig);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(GenerateMotionPrimitives, SameConfigIsDeterministic) {
  VehicleMotionPrimitiveSetConfig config;
  config.duration_s = 0.5;
  config.include_hover = true;
  config.heave_present = true;
  config.heave_m_s = 1.25;
  config.pitch_present = true;
  config.pitch_rad_s = 0.125;
  config.bounds.max_linear_speed_present = true;
  config.bounds.max_linear_speed_m_s = 1.25;
  config.bounds.max_angular_speed_present = true;
  config.bounds.max_angular_speed_rad_s = 0.125;

  const GenerateMotionPrimitivesResult first = GenerateMotionPrimitives(config);
  const GenerateMotionPrimitivesResult second =
      GenerateMotionPrimitives(config);
  ASSERT_EQ(first.error, second.error);
  ASSERT_EQ(first.error, MotionPrimitiveSetError::kOk);
  ASSERT_EQ(first.primitives.size(), second.primitives.size());
  for (size_t i = 0; i < first.primitives.size(); ++i) {
    EXPECT_EQ(first.primitives[i].id, second.primitives[i].id);
    EXPECT_EQ(first.primitives[i].duration_s, second.primitives[i].duration_s);
    const BodyVector& a = first.primitives[i].control;
    const BodyVector& b = second.primitives[i].control;
    EXPECT_EQ(a.linear_x, b.linear_x);
    EXPECT_EQ(a.linear_y, b.linear_y);
    EXPECT_EQ(a.linear_z, b.linear_z);
    EXPECT_EQ(a.angular_x, b.angular_x);
    EXPECT_EQ(a.angular_y, b.angular_y);
    EXPECT_EQ(a.angular_z, b.angular_z);
  }
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
