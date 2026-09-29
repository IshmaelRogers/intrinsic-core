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

#include "intrinsic/vehicle/allocation/unconstrained_least_squares.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::allocation::AllocateUnconstrainedLeastSquares;
using intrinsic::vehicle::allocation::AllocationErrorCode;
using intrinsic::vehicle::allocation::AllocationStatus;
using intrinsic::vehicle::allocation::BuildThrusterEffectivenessMatrix;
using intrinsic::vehicle::allocation::EffectivenessColumn;
using intrinsic::vehicle::allocation::kAllocationWrenchTolerance;
using intrinsic::vehicle::parameters::kSixThrusterUuvThrusterCount;
using intrinsic::vehicle::parameters::kSpatialDof;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;

constexpr std::string_view kEmptyColumnsMessage = "columns must be non-empty";
constexpr std::string_view kThrustLengthMessage =
    "thrust length must equal the column count";
constexpr std::string_view kWrenchNonFiniteMessage =
    "body wrench must be finite";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kGramNonFiniteMessage =
    "effectiveness gram matrix is not finite";
constexpr std::string_view kRankDeficientMessage =
    "effectiveness matrix is rank deficient";

void IdentityColumn(int axis, EffectivenessColumn* column) {
  column->components.fill(0.0);
  column->components[axis] = 1.0;
}

void ExpectZeroThrust(std::span<const double> thrust) {
  for (double command : thrust) {
    EXPECT_TRUE(std::isfinite(command));
    EXPECT_EQ(command, 0.0);
  }
}

void Poison(std::span<double> thrust) {
  for (double& command : thrust) {
    command = std::numeric_limits<double>::quiet_NaN();
  }
}

std::array<double, kSpatialDof> Apply(
    std::span<const EffectivenessColumn> columns,
    std::span<const double> thrust) {
  std::array<double, kSpatialDof> wrench = {};
  for (std::size_t i = 0; i < columns.size(); ++i) {
    for (int row = 0; row < kSpatialDof; ++row) {
      wrench[row] += columns[i].components[row] * thrust[i];
    }
  }
  return wrench;
}

void ExpectWrench(const std::array<double, kSpatialDof>& actual,
                  const std::array<double, kSpatialDof>& expected) {
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_TRUE(std::isfinite(actual[row])) << "row " << row;
    EXPECT_NEAR(actual[row], expected[row], kAllocationWrenchTolerance)
        << "row " << row;
  }
}

TEST(UnconstrainedLeastSquares, IdentityColumnsReconstructWrench) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  const std::array<double, kSpatialDof> wrench = {1.0,  -2.0, 3.0,
                                                  -4.0, 5.0,  -6.0};
  std::array<double, kSpatialDof> thrust = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  ASSERT_TRUE(status.ok());
  EXPECT_EQ(status.message, std::string_view());
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    EXPECT_EQ(thrust[axis], wrench[axis]) << "axis " << axis;
  }
  ExpectWrench(Apply(columns, thrust), wrench);
}

TEST(UnconstrainedLeastSquares, SixThrusterExampleReconstructsWrench) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_EQ(model.thrusters.size(),
            static_cast<std::size_t>(kSixThrusterUuvThrusterCount));
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns).ok());

  // Square full-rank B has one solution, so the allocator must recover
  // the thrust that produced τ. The map is the six-thruster example.
  const std::array<double, kSixThrusterUuvThrusterCount> chosen = {
      1.0, -0.5, 0.0, 2.0, 0.25, -1.0};
  const std::array<double, kSpatialDof> wrench = Apply(columns, chosen);
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  ASSERT_TRUE(status.ok());
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_NEAR(thrust[i], chosen[i], kAllocationWrenchTolerance)
        << "thruster " << i;
  }
  ExpectWrench(Apply(columns, thrust), wrench);
}

TEST(UnconstrainedLeastSquares, ArbitraryWrenchStaysWithinTolerance) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns).ok());
  const std::array<double, kSpatialDof> wrench = {4.0, -2.5,  1.5,
                                                  0.2, -0.15, 0.35};
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, thrust).ok());
  ExpectWrench(Apply(columns, thrust), wrench);
}

TEST(UnconstrainedLeastSquares, ZeroWrenchYieldsZeroThrust) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  const std::array<double, kSpatialDof> wrench = {};
  std::array<double, kSpatialDof> thrust = {};
  Poison(thrust);
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, thrust).ok());
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, RedundantColumnUsesMinimumNorm) {
  // Seven columns: the identity, plus a second copy of surge. τ is one
  // newton of surge. The minimum-norm solution splits that newton across
  // the two identical columns.
  std::array<EffectivenessColumn, kSpatialDof + 1> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  IdentityColumn(0, &columns[kSpatialDof]);
  const std::array<double, kSpatialDof> wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kSpatialDof + 1> thrust = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, thrust).ok());
  EXPECT_NEAR(thrust[0], 0.5, 1e-12);
  EXPECT_NEAR(thrust[kSpatialDof], 0.5, 1e-12);
  for (int axis = 1; axis < kSpatialDof; ++axis) {
    EXPECT_NEAR(thrust[axis], 0.0, 1e-12) << "axis " << axis;
  }
  ExpectWrench(Apply(columns, thrust), wrench);
}

TEST(UnconstrainedLeastSquares, ZeroColumnStaysZero) {
  std::array<EffectivenessColumn, kSpatialDof + 1> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  const std::array<double, kSpatialDof> wrench = {1.0,  -2.0, 3.0,
                                                  -4.0, 5.0,  -6.0};
  std::array<double, kSpatialDof + 1> thrust = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, thrust).ok());
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    EXPECT_EQ(thrust[axis], wrench[axis]) << "axis " << axis;
  }
  EXPECT_EQ(thrust[kSpatialDof], 0.0);
}

TEST(UnconstrainedLeastSquares, RepeatedCallsMatch) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns).ok());
  const std::array<double, kSpatialDof> wrench = {4.0, -2.5,  1.5,
                                                  0.2, -0.15, 0.35};
  std::array<double, kSixThrusterUuvThrusterCount> first = {};
  std::array<double, kSixThrusterUuvThrusterCount> second = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, first).ok());
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(columns, wrench, second).ok());
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_EQ(first[i], second[i]) << "thruster " << i;
  }
}

TEST(UnconstrainedLeastSquares, UnderactuatedColumnsAreRankDeficient) {
  std::array<EffectivenessColumn, 3> columns = {};
  IdentityColumn(0, &columns[0]);
  IdentityColumn(1, &columns[1]);
  IdentityColumn(2, &columns[2]);
  const std::array<double, kSpatialDof> wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> thrust = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  EXPECT_FALSE(status.ok());
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, DisabledHeaveOnExampleIsRankDeficient) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  enabled[4] = false;
  enabled[5] = false;
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns).ok());
  const std::array<double, kSpatialDof> wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, InRangeRankDeficientWrenchStaysZero) {
  // τ lies in the column space, and the solve is still refused. This
  // leaf does not return a minimum-residual command.
  std::array<EffectivenessColumn, 1> columns = {};
  IdentityColumn(0, &columns[0]);
  const std::array<double, kSpatialDof> wrench = {3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 1> thrust = {99.0};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, RejectsEmptyColumns) {
  const std::array<double, kSpatialDof> wrench = {};
  std::array<double, 1> thrust = {4.0};
  const AllocationStatus status = AllocateUnconstrainedLeastSquares(
      std::span<const EffectivenessColumn>(), wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kEmptyColumnsMessage);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, RejectsThrustLength) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  const std::array<double, kSpatialDof> wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 2> thrust = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kThrustLengthMessage);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteWrench) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  const double non_finite[] = {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()};
  for (double value : non_finite) {
    std::array<double, kSpatialDof> wrench = {};
    wrench[3] = value;
    std::array<double, kSpatialDof> thrust = {};
    Poison(thrust);
    const AllocationStatus status =
        AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
    EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
    EXPECT_EQ(status.message, kWrenchNonFiniteMessage);
    ExpectZeroThrust(thrust);
  }
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteColumn) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    IdentityColumn(axis, &columns[axis]);
  }
  columns[2].components[4] = std::numeric_limits<double>::quiet_NaN();
  const std::array<double, kSpatialDof> wrench = {};
  std::array<double, kSpatialDof> thrust = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kColumnNonFiniteMessage);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteGram) {
  std::array<EffectivenessColumn, 1> columns = {};
  columns[0].components.fill(1e200);
  const std::array<double, kSpatialDof> wrench = {};
  std::array<double, 1> thrust = {7.0};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kGramNonFiniteMessage);
  ExpectZeroThrust(thrust);
}

TEST(UnconstrainedLeastSquares, FirstDefectWins) {
  const std::array<double, kSpatialDof> wrench = {
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 1> thrust = {};
  const AllocationStatus status = AllocateUnconstrainedLeastSquares(
      std::span<const EffectivenessColumn>(), wrench, thrust);
  EXPECT_EQ(status.message, kEmptyColumnsMessage);
}

}  // namespace
