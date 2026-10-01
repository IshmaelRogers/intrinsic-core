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

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::vehicle::BodyVector;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

BodyVector Vec(double fx, double fy, double fz, double tx, double ty,
               double tz) {
  BodyVector v;
  v.linear_x = fx;
  v.linear_y = fy;
  v.linear_z = fz;
  v.angular_x = tx;
  v.angular_y = ty;
  v.angular_z = tz;
  return v;
}

std::array<double, 6> Arr(const BodyVector& v) {
  return {v.linear_x,  v.linear_y,  v.linear_z,
          v.angular_x, v.angular_y, v.angular_z};
}

VehicleMotionPrimitiveConfig Nominal() {
  VehicleMotionPrimitiveConfig config;
  config.max_force_torque = Vec(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  config.duration_s = 0.5;
  return config;
}

void ExpectEmptyFailure(const VehicleMotionPrimitiveConfig& config,
                        PrimitiveSetError error) {
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  EXPECT_EQ(result.error, error);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(VehicleMotionPrimitivesTest, NominalAxisAlignedHasThirteenOrdered) {
  const VehicleMotionPrimitiveConfig config = Nominal();
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  ASSERT_EQ(result.error, PrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 13u);

  const std::array<double, 6> max = Arr(config.max_force_torque);
  for (std::size_t i = 0; i < result.primitives.size(); ++i) {
    const VehicleMotionPrimitive& p = result.primitives[i];
    char expected_id[48];
    std::snprintf(expected_id, sizeof(expected_id), "uuv-prim-%03zu", i);
    EXPECT_EQ(p.id, expected_id);
    EXPECT_EQ(p.duration_s, 0.5);
    const std::array<double, 6> u = Arr(p.control);
    for (std::size_t axis = 0; axis < 6; ++axis) {
      EXPECT_LE(std::fabs(u[axis]), max[axis]);
    }
  }

  EXPECT_EQ(result.primitives[0].id, "uuv-prim-000");
  EXPECT_EQ(result.primitives[12].id, "uuv-prim-012");
  const std::array<double, 6> zero{};
  EXPECT_EQ(Arr(result.primitives[0].control), zero);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    std::array<double, 6> positive{};
    std::array<double, 6> negative{};
    positive[axis] = max[axis];
    negative[axis] = -max[axis];
    EXPECT_EQ(Arr(result.primitives[1 + 2 * axis].control), positive)
        << "axis " << axis;
    EXPECT_EQ(Arr(result.primitives[2 + 2 * axis].control), negative)
        << "axis " << axis;
  }
}

TEST(VehicleMotionPrimitivesTest, ZeroBoundAxisStillEmitsBothSlots) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.max_force_torque.linear_y = 0.0;
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  ASSERT_EQ(result.error, PrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 13u);
  const std::array<double, 6> zero{};
  EXPECT_EQ(Arr(result.primitives[3].control), zero);
  EXPECT_EQ(Arr(result.primitives[4].control), zero);
  EXPECT_EQ(result.primitives[3].id, "uuv-prim-003");
  EXPECT_EQ(result.primitives[4].id, "uuv-prim-004");
  EXPECT_FALSE(std::signbit(result.primitives[4].control.linear_y));

  const PrimitiveSetResult nominal = GenerateUuvMotionPrimitives(Nominal());
  for (std::size_t i = 0; i < 13; ++i) {
    if (i == 3 || i == 4) continue;
    EXPECT_EQ(Arr(result.primitives[i].control),
              Arr(nominal.primitives[i].control))
        << "index " << i;
  }
}

TEST(VehicleMotionPrimitivesTest, NonZeroPrimitivesSitExactlyOnBound) {
  const VehicleMotionPrimitiveConfig config = Nominal();
  const std::array<double, 6> max = Arr(config.max_force_torque);
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  ASSERT_EQ(result.error, PrimitiveSetError::kOk);
  for (const VehicleMotionPrimitive& p : result.primitives) {
    const std::array<double, 6> u = Arr(p.control);
    int nonzero = 0;
    for (std::size_t axis = 0; axis < 6; ++axis) {
      if (u[axis] == 0.0) continue;
      ++nonzero;
      EXPECT_EQ(std::fabs(u[axis]), max[axis]);
    }
    EXPECT_LE(nonzero, 1);
  }
}

TEST(VehicleMotionPrimitivesTest, BadConfigIsRejected) {
  for (std::size_t axis = 0; axis < 6; ++axis) {
    for (double bad : {-1.0, -1e-12, kNaN, kInf, -kInf}) {
      VehicleMotionPrimitiveConfig config = Nominal();
      std::array<double, 6> max = Arr(config.max_force_torque);
      max[axis] = bad;
      config.max_force_torque =
          Vec(max[0], max[1], max[2], max[3], max[4], max[5]);
      ExpectEmptyFailure(config, PrimitiveSetError::kBadConfig);
    }
  }
  for (double bad : {0.0, -0.5, kNaN, kInf, -kInf}) {
    VehicleMotionPrimitiveConfig config = Nominal();
    config.duration_s = bad;
    ExpectEmptyFailure(config, PrimitiveSetError::kBadConfig);
  }
  VehicleMotionPrimitiveConfig defaults;
  ExpectEmptyFailure(defaults, PrimitiveSetError::kBadConfig);
}

TEST(VehicleMotionPrimitivesTest, CustomEmptyIsEmpty) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.mode = PrimitiveGenerationMode::kCustom;
  ExpectEmptyFailure(config, PrimitiveSetError::kEmpty);
}

TEST(VehicleMotionPrimitivesTest, CustomBadConfigBeatsEmpty) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.mode = PrimitiveGenerationMode::kCustom;
  config.duration_s = 0.0;
  ExpectEmptyFailure(config, PrimitiveSetError::kBadConfig);
}

TEST(VehicleMotionPrimitivesTest, CustomOutOfBoundsIsRejectedNotClamped) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.mode = PrimitiveGenerationMode::kCustom;
  const BodyVector ok = Vec(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);
  const std::array<BodyVector, 6> over = {
      Vec(1.0000001, 0, 0, 0, 0, 0), Vec(0, -2.5, 0, 0, 0, 0),
      Vec(0, 0, 3.5, 0, 0, 0),       Vec(0, 0, 0, 0.11, 0, 0),
      Vec(0, 0, 0, 0, -0.21, 0),     Vec(0, 0, 0, 0, 0, 0.31)};
  for (const BodyVector& bad : over) {
    config.custom_controls = {ok, bad};
    ExpectEmptyFailure(config, PrimitiveSetError::kBadConfig);
  }
  for (double bad : {kNaN, kInf, -kInf}) {
    config.custom_controls = {Vec(0, 0, bad, 0, 0, 0)};
    ExpectEmptyFailure(config, PrimitiveSetError::kBadConfig);
  }
}

TEST(VehicleMotionPrimitivesTest, CustomInBoundsPreservesOrder) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.mode = PrimitiveGenerationMode::kCustom;
  config.custom_controls = {
      Vec(0.5, -1.0, 3.0, 0.0, 0.2, -0.3),
      Vec(0, 0, 0, 0, 0, 0),
      Vec(-1.0, 2.0, -3.0, 0.1, -0.2, 0.3),
  };
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  ASSERT_EQ(result.error, PrimitiveSetError::kOk);
  ASSERT_EQ(result.primitives.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(Arr(result.primitives[i].control),
              Arr(config.custom_controls[i]));
    EXPECT_EQ(result.primitives[i].duration_s, 0.5);
  }
  EXPECT_EQ(result.primitives[0].id, "uuv-prim-000");
  EXPECT_EQ(result.primitives[1].id, "uuv-prim-001");
  EXPECT_EQ(result.primitives[2].id, "uuv-prim-002");
}

TEST(VehicleMotionPrimitivesTest, AxisAlignedIgnoresCustomControls) {
  VehicleMotionPrimitiveConfig config = Nominal();
  config.custom_controls = {Vec(100, 0, 0, 0, 0, 0)};
  const PrimitiveSetResult result = GenerateUuvMotionPrimitives(config);
  EXPECT_EQ(result.error, PrimitiveSetError::kOk);
  EXPECT_EQ(result.primitives.size(), 13u);
}

TEST(VehicleMotionPrimitivesTest, IsDeterministic) {
  const VehicleMotionPrimitiveConfig config = Nominal();
  const PrimitiveSetResult a = GenerateUuvMotionPrimitives(config);
  const PrimitiveSetResult b = GenerateUuvMotionPrimitives(config);
  ASSERT_EQ(a.error, b.error);
  ASSERT_EQ(a.primitives.size(), b.primitives.size());
  for (std::size_t i = 0; i < a.primitives.size(); ++i) {
    EXPECT_EQ(a.primitives[i].id, b.primitives[i].id);
    EXPECT_EQ(Arr(a.primitives[i].control), Arr(b.primitives[i].control));
    EXPECT_EQ(a.primitives[i].duration_s, b.primitives[i].duration_s);
  }
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
