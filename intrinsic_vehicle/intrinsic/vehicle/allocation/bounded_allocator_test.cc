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

#include "intrinsic/vehicle/allocation/bounded_allocator.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/allocation/least_squares_allocator.h"
#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::allocation::AllocateBoundedLeastSquares;
using intrinsic::vehicle::allocation::AllocateUnconstrainedLeastSquares;
using intrinsic::vehicle::allocation::AllocationErrorCode;
using intrinsic::vehicle::allocation::AllocationStatus;
using intrinsic::vehicle::allocation::BuildThrusterEffectivenessMatrix;
using intrinsic::vehicle::allocation::EffectivenessColumn;
using intrinsic::vehicle::allocation::ThrustCommandBounds;
using intrinsic::vehicle::allocation::ThrustCommandBoundsFromGeometry;
using intrinsic::vehicle::parameters::kHeave;
using intrinsic::vehicle::parameters::kPitch;
using intrinsic::vehicle::parameters::kRoll;
using intrinsic::vehicle::parameters::kSixThrusterUuvThrusterCount;
using intrinsic::vehicle::parameters::kSpatialDof;
using intrinsic::vehicle::parameters::kSurge;
using intrinsic::vehicle::parameters::kSway;
using intrinsic::vehicle::parameters::kYaw;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::ThrusterGeometry;
using intrinsic::vehicle::parameters::ThrusterHealthState;

constexpr double kReconstructionTolerance = 1e-9;

constexpr std::string_view kEmptyColumnsMessage =
    "effectiveness columns must be non-empty";
constexpr std::string_view kThrustCountMessage =
    "thrust command count must equal the column count";
constexpr std::string_view kBoundsCountMessage =
    "thrust bounds count must equal the column count";
constexpr std::string_view kSaturatedCountMessage =
    "saturation flag count must equal the column count";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kWrenchNonFiniteMessage =
    "requested wrench must be finite";
constexpr std::string_view kBoundNonFiniteMessage =
    "thrust bound is not finite";
constexpr std::string_view kInvertedBoundsMessage =
    "thrust bound minimum exceeds maximum";
constexpr std::string_view kGramNonFiniteMessage =
    "Gram matrix B B^T is not finite";
constexpr std::string_view kRankDeficientMessage =
    "B B^T is singular; Cholesky pivot is not above the relative tolerance";
constexpr std::string_view kResidualNormNonFiniteMessage =
    "allocation residual norm is not finite";

using Wrench = std::array<double, kSpatialDof>;

EffectivenessColumn AxisColumn(int axis, double scale = 1.0) {
  EffectivenessColumn column;
  column.components[axis] = scale;
  return column;
}

ThrustCommandBounds Bounds(double min_thrust_n, double max_thrust_n) {
  ThrustCommandBounds bounds;
  bounds.min_thrust_n = min_thrust_n;
  bounds.max_thrust_n = max_thrust_n;
  return bounds;
}

void Poison(std::span<double> values) {
  for (double& value : values) {
    value = std::numeric_limits<double>::quiet_NaN();
  }
}

void PoisonFlags(std::span<bool> flags) {
  for (bool& flag : flags) {
    flag = true;
  }
}

void ExpectFiniteZeros(std::span<const double> values) {
  for (double value : values) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_FALSE(std::signbit(value));
    EXPECT_EQ(value, 0.0);
  }
}

void ExpectFlagsClear(std::span<const bool> flags) {
  for (bool flag : flags) {
    EXPECT_FALSE(flag);
  }
}

void ExpectSameBits(std::span<const double> left,
                    std::span<const double> right) {
  ASSERT_EQ(left.size(), right.size());
  EXPECT_EQ(
      0, std::memcmp(left.data(), right.data(), left.size() * sizeof(double)));
}

void ExpectSameFlagBits(std::span<const bool> left,
                        std::span<const bool> right) {
  ASSERT_EQ(left.size(), right.size());
  EXPECT_EQ(0,
            std::memcmp(left.data(), right.data(), left.size() * sizeof(bool)));
}

void ExpectInsideBounds(std::span<const double> thrust,
                        std::span<const ThrustCommandBounds> bounds) {
  ASSERT_EQ(thrust.size(), bounds.size());
  for (std::size_t i = 0; i < thrust.size(); ++i) {
    EXPECT_GE(thrust[i], bounds[i].min_thrust_n) << "thruster " << i;
    EXPECT_LE(thrust[i], bounds[i].max_thrust_n) << "thruster " << i;
  }
}

void ExpectResidualIdentity(const Wrench& achieved, const Wrench& residual,
                            const Wrench& wrench) {
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_NEAR(achieved[row] + residual[row], wrench[row],
                kReconstructionTolerance)
        << "row " << row;
    EXPECT_DOUBLE_EQ(residual[row], wrench[row] - achieved[row])
        << "row " << row;
  }
}

std::array<EffectivenessColumn, kSpatialDof> IdentityColumns() {
  return {AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
          AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
}

std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount>
SixThrusterColumns(const MarineModel& model) {
  EXPECT_EQ(model.thrusters.size(),
            static_cast<std::size_t>(kSixThrusterUuvThrusterCount));
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns);
  EXPECT_TRUE(status.ok());
  return columns;
}

TEST(BoundedLeastSquares, InteriorIdentityMatchesUnconstrained) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-20.0, 20.0), Bounds(-20.0, 20.0), Bounds(-20.0, 20.0),
      Bounds(-20.0, 20.0), Bounds(-20.0, 20.0), Bounds(-20.0, 20.0)};
  std::array<double, kSpatialDof> unconstrained = {};
  Wrench unconstrained_residual = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, unconstrained,
                                                unconstrained_residual)
                  .ok());

  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kOk);
  EXPECT_EQ(status.message, std::string_view());
  ExpectSameBits(thrust, unconstrained);
  ExpectSameBits(residual, unconstrained_residual);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(achieved[row], wrench[row]);
    EXPECT_DOUBLE_EQ(thrust[row], wrench[row]);
  }
}

TEST(BoundedLeastSquares, InteriorSixThrusterMatchesUnconstrained) {
  const MarineModel model = MakeSixThrusterUuvExample();
  const auto columns = SixThrusterColumns(model);
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    bounds[i] = ThrustCommandBoundsFromGeometry(model.thrusters[i]);
  }
  std::array<double, kSixThrusterUuvThrusterCount> unconstrained = {};
  Wrench unconstrained_residual = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, unconstrained,
                                                unconstrained_residual)
                  .ok());
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    ASSERT_GT(unconstrained[i], bounds[i].min_thrust_n) << i;
    ASSERT_LT(unconstrained[i], bounds[i].max_thrust_n) << i;
  }

  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSixThrusterUuvThrusterCount> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  ExpectSameBits(thrust, unconstrained);
  ExpectSameBits(residual, unconstrained_residual);
  ExpectFlagsClear(saturated);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_NEAR(achieved[row], wrench[row], kReconstructionTolerance)
        << "row " << row;
  }
}

TEST(BoundedLeastSquares, ZeroWrenchInsideBoundsIsNotSaturated) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
}

TEST(BoundedLeastSquares, ZeroCommandOnABoundIsSaturated) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(0.0, 10.0), Bounds(0.0, 10.0), Bounds(0.0, 10.0),
      Bounds(0.0, 10.0), Bounds(0.0, 10.0), Bounds(0.0, 10.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 1.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  for (bool flag : saturated) {
    EXPECT_TRUE(flag);
  }
  ExpectInsideBounds(thrust, bounds);
}

TEST(BoundedLeastSquares, SingleActuatorUpperSaturation) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-3.5, 4.25),    Bounds(-100.0, 100.0), Bounds(-100.0, 100.0),
      Bounds(-100.0, 100.0), Bounds(-100.0, 100.0), Bounds(-100.0, 100.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 4.25);
  for (int i = kSway; i < kSpatialDof; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], 0.0) << i;
    EXPECT_FALSE(saturated[i]) << i;
  }
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_DOUBLE_EQ(achieved[kSurge], 4.25);
  EXPECT_DOUBLE_EQ(residual[kSurge], 95.75);
  EXPECT_DOUBLE_EQ(norm, 95.75);
  ExpectFiniteZeros(std::span<const double>(achieved).subspan(1));
  ExpectFiniteZeros(std::span<const double>(residual).subspan(1));
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, SingleActuatorLowerSaturation) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {-100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-3.5, 4.25),    Bounds(-100.0, 100.0), Bounds(-100.0, 100.0),
      Bounds(-100.0, 100.0), Bounds(-100.0, 100.0), Bounds(-100.0, 100.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], -3.5);
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_FALSE(saturated[kSway]);
  EXPECT_DOUBLE_EQ(achieved[kSurge], -3.5);
  EXPECT_DOUBLE_EQ(residual[kSurge], -96.5);
  EXPECT_DOUBLE_EQ(norm, 96.5);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, MultiActuatorSaturation) {
  // Identity map. Surge 50 clamps to 10 and sway 40 clamps to 10.
  // Residual (40, 30) has L2 norm 50. The other four commands stay 0.
  const auto columns = IdentityColumns();
  const Wrench wrench = {50.0, 40.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-10.0, 10.0),   Bounds(-10.0, 10.0),   Bounds(-100.0, 100.0),
      Bounds(-100.0, 100.0), Bounds(-100.0, 100.0), Bounds(-100.0, 100.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 10.0);
  EXPECT_DOUBLE_EQ(thrust[kSway], 10.0);
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_TRUE(saturated[kSway]);
  for (int i = kHeave; i < kSpatialDof; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], 0.0) << i;
    EXPECT_FALSE(saturated[i]) << i;
  }
  EXPECT_DOUBLE_EQ(achieved[kSurge], 10.0);
  EXPECT_DOUBLE_EQ(achieved[kSway], 10.0);
  EXPECT_DOUBLE_EQ(residual[kSurge], 40.0);
  EXPECT_DOUBLE_EQ(residual[kSway], 30.0);
  EXPECT_DOUBLE_EQ(norm, 50.0);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, ExactlyAtBoundIsSaturated) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {10.0, -10.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-10.0, 10.0), Bounds(-10.0, 10.0), Bounds(-10.0, 10.0),
      Bounds(-10.0, 10.0), Bounds(-10.0, 10.0), Bounds(-10.0, 10.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 1.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 10.0);
  EXPECT_DOUBLE_EQ(thrust[kSway], -10.0);
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_TRUE(saturated[kSway]);
  for (int i = kHeave; i < kSpatialDof; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], 0.0) << i;
    EXPECT_FALSE(saturated[i]) << i;
  }
  ExpectFiniteZeros(residual);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, FixedBoundClampsAndPassesThrough) {
  const auto columns = IdentityColumns();
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(4.0, 4.0),    Bounds(-20.0, 20.0), Bounds(-20.0, 20.0),
      Bounds(-20.0, 20.0), Bounds(-20.0, 20.0), Bounds(-20.0, 20.0)};

  const Wrench on_bound = {4.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 0.0;
  ASSERT_TRUE(AllocateBoundedLeastSquares(columns, on_bound, bounds, thrust,
                                          achieved, residual, saturated, norm)
                  .ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 4.0);
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_FALSE(saturated[kSway]);
  ExpectFiniteZeros(residual);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  ExpectInsideBounds(thrust, bounds);

  const Wrench above = {9.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(AllocateBoundedLeastSquares(columns, above, bounds, thrust,
                                          achieved, residual, saturated, norm)
                  .ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 4.0);
  EXPECT_DOUBLE_EQ(thrust[kSway], 1.0);
  EXPECT_TRUE(saturated[kSurge]);
  EXPECT_FALSE(saturated[kSway]);
  EXPECT_DOUBLE_EQ(residual[kSurge], 5.0);
  EXPECT_DOUBLE_EQ(residual[kSway], 0.0);
  EXPECT_DOUBLE_EQ(norm, 5.0);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, above);
}

TEST(BoundedLeastSquares, DoesNotReallocateUnsaturatedThrusters) {
  // Four surge copies split a load of 4 into commands of 1. Clamping the
  // first copy to 0.25 must leave the other three at 1. A redistribution
  // would raise them.
  std::array<EffectivenessColumn, 9> columns = {
      AxisColumn(kSurge), AxisColumn(kSurge), AxisColumn(kSurge),
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  const Wrench wrench = {4.0, 3.0, 5.0, 6.0, 7.0, 8.0};
  std::array<ThrustCommandBounds, 9> bounds = {};
  for (ThrustCommandBounds& bound : bounds) {
    bound = Bounds(-100.0, 100.0);
  }
  bounds[0] = Bounds(0.0, 0.25);
  std::array<double, 9> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 9> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  const std::array<double, 9> expected = {0.25, 1.0, 1.0, 1.0, 3.0,
                                          5.0,  6.0, 7.0, 8.0};
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], expected[i]) << i;
  }
  EXPECT_TRUE(saturated[0]);
  for (int i = 1; i < 9; ++i) {
    EXPECT_FALSE(saturated[i]) << i;
  }
  EXPECT_DOUBLE_EQ(achieved[kSurge], 3.25);
  EXPECT_DOUBLE_EQ(residual[kSurge], 0.75);
  EXPECT_DOUBLE_EQ(achieved[kSway], 3.0);
  EXPECT_DOUBLE_EQ(residual[kSway], 0.0);
  EXPECT_DOUBLE_EQ(norm, 0.75);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, CoupledColumnsDoNotReallocate) {
  const auto columns = SixThrusterColumns(MakeSixThrusterUuvExample());
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<double, kSixThrusterUuvThrusterCount> unconstrained = {};
  Wrench unconstrained_residual = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, unconstrained,
                                                unconstrained_residual)
                  .ok());
  int saturated_index = 0;
  double largest = 0.0;
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    if (std::abs(unconstrained[i]) > largest) {
      largest = std::abs(unconstrained[i]);
      saturated_index = i;
    }
  }
  ASSERT_GT(largest, 1e-6);

  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  for (ThrustCommandBounds& bound : bounds) {
    bound = Bounds(-1e6, 1e6);
  }
  const double half = unconstrained[saturated_index] * 0.5;
  if (unconstrained[saturated_index] > 0.0) {
    bounds[saturated_index] = Bounds(-1e6, half);
  } else {
    bounds[saturated_index] = Bounds(half, 1e6);
  }

  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSixThrusterUuvThrusterCount> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[saturated_index], half);
  EXPECT_TRUE(saturated[saturated_index]);
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    if (i == saturated_index) {
      continue;
    }
    EXPECT_DOUBLE_EQ(thrust[i], unconstrained[i]) << i;
    EXPECT_FALSE(saturated[i]) << i;
  }
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
  EXPECT_GT(norm, 0.0);
}

TEST(BoundedLeastSquares, RepeatedCallsMatchBits) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {50.0, 40.0, 1.0, -2.0, 0.5, -0.25};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-10.0, 10.0), Bounds(-10.0, 10.0), Bounds(-20.0, 20.0),
      Bounds(-20.0, 20.0), Bounds(-20.0, 20.0), Bounds(-20.0, 20.0)};
  std::array<double, kSpatialDof> thrust_a = {};
  std::array<double, kSpatialDof> thrust_b = {};
  Wrench achieved_a = {};
  Wrench achieved_b = {};
  Wrench residual_a = {};
  Wrench residual_b = {};
  std::array<bool, kSpatialDof> saturated_a = {};
  std::array<bool, kSpatialDof> saturated_b = {};
  double norm_a = 0.0;
  double norm_b = 0.0;
  ASSERT_TRUE(AllocateBoundedLeastSquares(columns, wrench, bounds, thrust_a,
                                          achieved_a, residual_a, saturated_a,
                                          norm_a)
                  .ok());
  ASSERT_TRUE(AllocateBoundedLeastSquares(columns, wrench, bounds, thrust_b,
                                          achieved_b, residual_b, saturated_b,
                                          norm_b)
                  .ok());
  ExpectSameBits(thrust_a, thrust_b);
  ExpectSameBits(achieved_a, achieved_b);
  ExpectSameBits(residual_a, residual_b);
  ExpectSameFlagBits(saturated_a, saturated_b);
  EXPECT_EQ(0, std::memcmp(&norm_a, &norm_b, sizeof(double)));
  EXPECT_TRUE(saturated_a[kSurge]);
  EXPECT_TRUE(saturated_a[kSway]);
  ExpectInsideBounds(thrust_a, bounds);
}

TEST(BoundedLeastSquares, GeometryHelperReadsOnlyThrustLimits) {
  ThrusterGeometry thruster;
  thruster.max_forward_thrust_n = 12.0;
  thruster.max_reverse_thrust_n = 7.0;
  thruster.max_forward_slew_n_per_s = 0.01;
  thruster.max_reverse_slew_n_per_s = 0.02;
  thruster.efficiency = 0.25;
  thruster.health = ThrusterHealthState::kFailed;
  thruster.health_derate = 0.0;
  const ThrustCommandBounds failed = ThrustCommandBoundsFromGeometry(thruster);
  EXPECT_DOUBLE_EQ(failed.min_thrust_n, -7.0);
  EXPECT_DOUBLE_EQ(failed.max_thrust_n, 12.0);

  thruster.health = ThrusterHealthState::kNominal;
  thruster.health_derate = 1.0;
  thruster.efficiency = 1.0;
  thruster.max_forward_slew_n_per_s = 1000.0;
  thruster.max_reverse_slew_n_per_s = 1000.0;
  thruster.name = "renamed";
  thruster.position_m = {9.0, 8.0, 7.0};
  thruster.direction_body = {0.0, 1.0, 0.0};
  const ThrustCommandBounds nominal = ThrustCommandBoundsFromGeometry(thruster);
  EXPECT_EQ(0, std::memcmp(&failed, &nominal, sizeof(ThrustCommandBounds)));
}

TEST(BoundedLeastSquares, DoesNotSlewOrZeroAFailedActuator) {
  // The command interval comes from the helper. Health, derate,
  // efficiency, and slew are not arguments of the allocator. A failed
  // actuator is not forced to zero, and the clamp is not a slew step
  // from the previous command (there is no previous command).
  ThrusterGeometry thruster;
  thruster.max_forward_thrust_n = 50.0;
  thruster.max_reverse_thrust_n = 35.0;
  thruster.max_forward_slew_n_per_s = 1.0;
  thruster.max_reverse_slew_n_per_s = 1.0;
  thruster.efficiency = 0.1;
  thruster.health = ThrusterHealthState::kFailed;
  thruster.health_derate = 0.0;
  const ThrustCommandBounds surge_bounds =
      ThrustCommandBoundsFromGeometry(thruster);

  const auto columns = IdentityColumns();
  const Wrench wrench = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<ThrustCommandBounds, kSpatialDof> bounds = {};
  for (ThrustCommandBounds& bound : bounds) {
    bound = Bounds(-100.0, 100.0);
  }
  bounds[kSurge] = surge_bounds;
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = 0.0;
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(thrust[kSurge], 50.0);
  EXPECT_GT(std::abs(thrust[kSurge]), thruster.max_forward_slew_n_per_s);
  EXPECT_NE(thrust[kSurge], 0.0);
  EXPECT_TRUE(saturated[kSurge]);
  for (int i = kSway; i < kSpatialDof; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], 0.0) << i;
  }
  EXPECT_DOUBLE_EQ(residual[kSurge], 50.0);
  ExpectInsideBounds(thrust, bounds);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, RankDeficientDoesNotClamp) {
  std::array<EffectivenessColumn, kSpatialDof> columns = IdentityColumns();
  columns[kYaw] = AxisColumn(kSurge);
  const Wrench wrench = {0.0, 0.0, 0.0, 0.0, 0.0, 4.0};
  // Zero equals the lower bound. Flags stay false because no clamp runs.
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(0.0, 10.0), Bounds(0.0, 10.0), Bounds(0.0, 10.0),
      Bounds(0.0, 10.0), Bounds(0.0, 10.0), Bounds(0.0, 10.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFlagsClear(saturated);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(residual[row], wrench[row]);
  }
  EXPECT_DOUBLE_EQ(norm, 4.0);
  ExpectResidualIdentity(achieved, residual, wrench);
}

TEST(BoundedLeastSquares, RankDeficientZerosMayLieOutsideBounds) {
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge)};
  const Wrench wrench = {1.0, 2.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 1> bounds = {Bounds(5.0, 9.0)};
  std::array<double, 1> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 1> saturated = {};
  double norm = 0.0;
  Poison(thrust);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  ExpectFiniteZeros(thrust);
  EXPECT_LT(thrust[0], bounds[0].min_thrust_n);
  ExpectFlagsClear(saturated);
  ExpectFiniteZeros(achieved);
  EXPECT_DOUBLE_EQ(residual[kSurge], 1.0);
  EXPECT_DOUBLE_EQ(residual[kSway], 2.0);
  EXPECT_DOUBLE_EQ(norm, std::sqrt(5.0));
}

TEST(BoundedLeastSquares, RankDeficientNormOverflowIsInvalid) {
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge)};
  const Wrench wrench = {1e200, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 1> bounds = {Bounds(-10.0, 10.0)};
  std::array<double, 1> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 1> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kResidualNormNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
}

TEST(BoundedLeastSquares, RepeatedRankDeficientCallsMatchBits) {
  std::array<EffectivenessColumn, kSpatialDof> columns = IdentityColumns();
  columns[1] = columns[0];
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-10.0, 10.0), Bounds(-10.0, 10.0), Bounds(-10.0, 10.0),
      Bounds(-10.0, 10.0), Bounds(-10.0, 10.0), Bounds(-10.0, 10.0)};
  std::array<double, kSpatialDof> thrust_a = {};
  std::array<double, kSpatialDof> thrust_b = {};
  Wrench achieved_a = {};
  Wrench achieved_b = {};
  Wrench residual_a = {};
  Wrench residual_b = {};
  std::array<bool, kSpatialDof> saturated_a = {};
  std::array<bool, kSpatialDof> saturated_b = {};
  double norm_a = 0.0;
  double norm_b = 0.0;
  Poison(thrust_a);
  Poison(thrust_b);
  const AllocationStatus status_a =
      AllocateBoundedLeastSquares(columns, wrench, bounds, thrust_a, achieved_a,
                                  residual_a, saturated_a, norm_a);
  const AllocationStatus status_b =
      AllocateBoundedLeastSquares(columns, wrench, bounds, thrust_b, achieved_b,
                                  residual_b, saturated_b, norm_b);
  EXPECT_EQ(status_a.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status_a.message, kRankDeficientMessage);
  EXPECT_EQ(status_b.code, status_a.code);
  EXPECT_EQ(status_b.message, status_a.message);
  ExpectSameBits(thrust_a, thrust_b);
  ExpectSameBits(achieved_a, achieved_b);
  ExpectSameBits(residual_a, residual_b);
  ExpectSameFlagBits(saturated_a, saturated_b);
  EXPECT_EQ(0, std::memcmp(&norm_a, &norm_b, sizeof(double)));
  ExpectFiniteZeros(thrust_a);
  ExpectFlagsClear(saturated_a);
}

TEST(BoundedLeastSquares, RejectsEmptyColumnsBeforeOtherDefects) {
  const Wrench wrench = {
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 1> bounds = {
      Bounds(std::numeric_limits<double>::quiet_NaN(), 1.0)};
  std::array<double, 2> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 2> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      {}, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kEmptyColumnsMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
}

TEST(BoundedLeastSquares, RejectsThrustCountBeforeBounds) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 1> bounds = {Bounds(-1.0, 1.0)};
  std::array<double, 2> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kThrustCountMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsBoundsCount) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 2> bounds = {Bounds(-1.0, 1.0),
                                                     Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kBoundsCountMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsSaturationCount) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 2> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kSaturatedCountMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsNonFiniteColumnBeforeBounds) {
  auto columns = IdentityColumns();
  columns[2].components[kRoll] = std::numeric_limits<double>::infinity();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(std::numeric_limits<double>::quiet_NaN(), 1.0),
      Bounds(2.0, 1.0),
      Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kColumnNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsNonFiniteWrenchBeforeBounds) {
  const auto columns = IdentityColumns();
  Wrench wrench = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  wrench[kPitch] = std::numeric_limits<double>::quiet_NaN();
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(4.0, 1.0),  Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kWrenchNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsNonFiniteBoundBeforeInverted) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  bounds[0] = Bounds(5.0, 1.0);
  bounds[1].max_thrust_n = std::numeric_limits<double>::infinity();
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kBoundNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, RejectsInvertedBounds) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  bounds[3] = Bounds(2.0, -2.0);
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kInvertedBoundsMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
}

TEST(BoundedLeastSquares, PropagatesNonFiniteGramAndClearsOutputs) {
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge, 1e200)};
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, 1> bounds = {Bounds(-10.0, 10.0)};
  std::array<double, 1> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, 1> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kGramNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
}

TEST(BoundedLeastSquares, ResidualNormOverflowAfterClampIsInvalid) {
  const auto columns = IdentityColumns();
  const Wrench wrench = {1e200, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<ThrustCommandBounds, kSpatialDof> bounds = {
      Bounds(0.0, 0.0),  Bounds(-1.0, 1.0), Bounds(-1.0, 1.0),
      Bounds(-1.0, 1.0), Bounds(-1.0, 1.0), Bounds(-1.0, 1.0)};
  std::array<double, kSpatialDof> thrust = {};
  Wrench achieved = {};
  Wrench residual = {};
  std::array<bool, kSpatialDof> saturated = {};
  double norm = std::numeric_limits<double>::quiet_NaN();
  Poison(thrust);
  Poison(achieved);
  Poison(residual);
  PoisonFlags(saturated);
  const AllocationStatus status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, thrust, achieved, residual, saturated, norm);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kResidualNormNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(achieved);
  ExpectFiniteZeros(residual);
  ExpectFlagsClear(saturated);
  EXPECT_DOUBLE_EQ(norm, 0.0);
  EXPECT_FALSE(std::signbit(norm));
}

}  // namespace
