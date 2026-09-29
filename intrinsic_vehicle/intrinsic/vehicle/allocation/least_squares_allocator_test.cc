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

#include "intrinsic/vehicle/allocation/least_squares_allocator.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::allocation::AllocateUnconstrainedLeastSquares;
using intrinsic::vehicle::allocation::AllocationErrorCode;
using intrinsic::vehicle::allocation::AllocationStatus;
using intrinsic::vehicle::allocation::BuildThrusterEffectivenessMatrix;
using intrinsic::vehicle::allocation::EffectivenessColumn;
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
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kWrenchNonFiniteMessage =
    "requested wrench must be finite";
constexpr std::string_view kGramNonFiniteMessage =
    "Gram matrix B B^T is not finite";
constexpr std::string_view kRankDeficientMessage =
    "B B^T is singular; Cholesky pivot is not above the relative tolerance";

using Wrench = std::array<double, kSpatialDof>;

EffectivenessColumn AxisColumn(int axis, double scale = 1.0) {
  EffectivenessColumn column;
  column.components[axis] = scale;
  return column;
}

void Poison(std::span<double> values) {
  for (double& value : values) {
    value = std::numeric_limits<double>::quiet_NaN();
  }
}

void ExpectFiniteZeros(std::span<const double> values) {
  for (double value : values) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_FALSE(std::signbit(value));
    EXPECT_EQ(value, 0.0);
  }
}

void ExpectSameBits(std::span<const double> left,
                    std::span<const double> right) {
  ASSERT_EQ(left.size(), right.size());
  EXPECT_EQ(
      0, std::memcmp(left.data(), right.data(), left.size() * sizeof(double)));
}

void ExpectReconstructed(std::span<const EffectivenessColumn> columns,
                         std::span<const double> thrust, const Wrench& wrench,
                         const Wrench& residual) {
  for (int row = 0; row < kSpatialDof; ++row) {
    double produced = 0.0;
    for (std::size_t col = 0; col < columns.size(); ++col) {
      produced += columns[col].components[row] * thrust[col];
    }
    EXPECT_NEAR(produced, wrench[row], kReconstructionTolerance)
        << "row " << row;
    EXPECT_NEAR(residual[row], wrench[row] - produced, 0.0) << "row " << row;
  }
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

TEST(UnconstrainedLeastSquares, IdentityMapsWrenchOntoThrust) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  const Wrench wrench = {1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
  std::array<double, kSpatialDof> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kOk);
  EXPECT_EQ(status.message, std::string_view());
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(thrust[row], wrench[row]);
    EXPECT_DOUBLE_EQ(residual[row], 0.0);
    EXPECT_FALSE(std::signbit(residual[row]));
  }
}

TEST(UnconstrainedLeastSquares, ZeroWrenchYieldsFiniteZeroThrust) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  const Wrench wrench = {};
  std::array<double, kSpatialDof> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, SixThrusterExampleReconstructsWrench) {
  const std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns =
      SixThrusterColumns(MakeSixThrusterUuvExample());
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Wrench residual = {};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  ExpectReconstructed(columns, thrust, wrench, residual);
}

TEST(UnconstrainedLeastSquares, SmallFullRankColumnsStillReconstruct) {
  // Columns scaled by 1e-6 stay full rank. Absolute pivots drop to about
  // 1e-14, under a fixed floor, and the relative floor still accepts them.
  auto columns = SixThrusterColumns(MakeSixThrusterUuvExample());
  for (EffectivenessColumn& column : columns) {
    for (double& value : column.components) {
      value *= 1e-6;
    }
  }
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Wrench residual = {};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  ExpectReconstructed(columns, thrust, wrench, residual);
}

TEST(UnconstrainedLeastSquares, UnderdeterminedDuplicateUsesMinimumNorm) {
  // Nine columns. Four copies of the surge axis make the system
  // underdetermined. G_surge = 4, an exact square, so the min-norm right
  // inverse splits the surge load into four commands of 1. A basic
  // solution that puts all surge thrust on one column still reconstructs
  // τ and must not be returned.
  std::array<EffectivenessColumn, 9> columns = {
      AxisColumn(kSurge), AxisColumn(kSurge), AxisColumn(kSurge),
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  const Wrench wrench = {4.0, 3.0, 5.0, 6.0, 7.0, 8.0};
  const std::array<double, 9> expected = {1.0, 1.0, 1.0, 1.0, 3.0,
                                          5.0, 6.0, 7.0, 8.0};
  std::array<double, 9> thrust = {};
  Wrench residual = {};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(thrust[i], expected[i]) << "thruster " << i;
  }
  ExpectFiniteZeros(residual);
  ExpectReconstructed(columns, thrust, wrench, residual);
}

TEST(UnconstrainedLeastSquares, SixThrusterExtraColumnKeepsMinimumNorm) {
  auto columns = SixThrusterColumns(MakeSixThrusterUuvExample());
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount + 1> wide = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    wide[i] = columns[i];
  }
  wide[kSixThrusterUuvThrusterCount] = columns[kSurge];
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<double, kSixThrusterUuvThrusterCount + 1> thrust = {};
  Wrench residual = {};
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(wide, wrench, thrust, residual);
  ASSERT_TRUE(status.ok());
  ExpectReconstructed(wide, thrust, wrench, residual);
  EXPECT_DOUBLE_EQ(thrust[kSurge], thrust[kSixThrusterUuvThrusterCount]);
}

TEST(UnconstrainedLeastSquares, DuplicateColumnIsRankDeficient) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kSurge)};
  const Wrench wrench = {0.0, 0.0, 0.0, 0.0, 0.0, 4.0};
  std::array<double, kSpatialDof> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  ExpectFiniteZeros(thrust);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(residual[row], wrench[row]);
  }
}

TEST(UnconstrainedLeastSquares, ZeroedColumnIsRankDeficient) {
  const MarineModel model = MakeSixThrusterUuvExample();
  auto columns = SixThrusterColumns(model);
  columns[3].components.fill(0.0);
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kSixThrusterUuvThrusterCount> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(status.message, kRankDeficientMessage);
  ExpectFiniteZeros(thrust);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(residual[row], wrench[row]);
  }
}

TEST(UnconstrainedLeastSquares, RepeatedCallsMatchBits) {
  const auto columns = SixThrusterColumns(MakeSixThrusterUuvExample());
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  std::array<double, kSixThrusterUuvThrusterCount> thrust_a = {};
  std::array<double, kSixThrusterUuvThrusterCount> thrust_b = {};
  Wrench residual_a = {};
  Wrench residual_b = {};
  ASSERT_TRUE(
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust_a, residual_a)
          .ok());
  ASSERT_TRUE(
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust_b, residual_b)
          .ok());
  ExpectSameBits(thrust_a, thrust_b);
  ExpectSameBits(residual_a, residual_b);

  auto deficient = columns;
  deficient[1] = deficient[0];
  Poison(thrust_a);
  Poison(thrust_b);
  const AllocationStatus deficient_a = AllocateUnconstrainedLeastSquares(
      deficient, wrench, thrust_a, residual_a);
  const AllocationStatus deficient_b = AllocateUnconstrainedLeastSquares(
      deficient, wrench, thrust_b, residual_b);
  EXPECT_EQ(deficient_a.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(deficient_a.message, kRankDeficientMessage);
  EXPECT_EQ(deficient_b.code, deficient_a.code);
  EXPECT_EQ(deficient_b.message, deficient_a.message);
  ExpectSameBits(thrust_a, thrust_b);
  ExpectSameBits(residual_a, residual_b);
  ExpectFiniteZeros(thrust_a);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_DOUBLE_EQ(residual_a[row], wrench[row]);
  }
}

TEST(UnconstrainedLeastSquares, DoesNotApplyBoundsSlewHealthOrEfficiency) {
  // The allocator takes columns and τ. Health, derate, efficiency, slew,
  // and thrust bounds are not parameters of the call. Mutating them on
  // the geometry must not change B or the commands, and the commands may
  // exceed the configured bounds.
  MarineModel nominal = MakeSixThrusterUuvExample();
  MarineModel derated = MakeSixThrusterUuvExample();
  for (ThrusterGeometry& thruster : derated.thrusters) {
    thruster.health = ThrusterHealthState::kFailed;
    thruster.health_derate = 0.0;
    thruster.efficiency = 0.2;
    thruster.max_forward_thrust_n = 1.0;
    thruster.max_reverse_thrust_n = 1.0;
    thruster.max_forward_slew_n_per_s = 1.0;
    thruster.max_reverse_slew_n_per_s = 1.0;
  }
  const auto nominal_columns = SixThrusterColumns(nominal);
  const auto derated_columns = SixThrusterColumns(derated);
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    ExpectSameBits(nominal_columns[i].components,
                   derated_columns[i].components);
  }
  EXPECT_DOUBLE_EQ(derated_columns[0].components[kSurge], 1.0);

  const Wrench wrench = {1000.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kSixThrusterUuvThrusterCount> nominal_thrust = {};
  std::array<double, kSixThrusterUuvThrusterCount> derated_thrust = {};
  Wrench nominal_residual = {};
  Wrench derated_residual = {};
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(
                  nominal_columns, wrench, nominal_thrust, nominal_residual)
                  .ok());
  ASSERT_TRUE(AllocateUnconstrainedLeastSquares(
                  derated_columns, wrench, derated_thrust, derated_residual)
                  .ok());
  ExpectSameBits(nominal_thrust, derated_thrust);
  ExpectSameBits(nominal_residual, derated_residual);
  ExpectReconstructed(derated_columns, derated_thrust, wrench,
                      derated_residual);
  EXPECT_GT(std::abs(derated_thrust[0]),
            derated.thrusters[0].max_forward_thrust_n);
  EXPECT_GT(std::abs(derated_thrust[1]),
            derated.thrusters[1].max_forward_thrust_n);
  EXPECT_GT(std::abs(nominal_thrust[0]),
            nominal.thrusters[0].max_forward_thrust_n);
}

TEST(UnconstrainedLeastSquares, RejectsEmptyColumns) {
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 2> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares({}, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kEmptyColumnsMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, RejectsThrustCountMismatch) {
  std::array<EffectivenessColumn, 2> columns = {AxisColumn(kSurge),
                                                AxisColumn(kSway)};
  columns[0].components[kHeave] = std::numeric_limits<double>::quiet_NaN();
  const Wrench wrench = {
      std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kThrustCountMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteColumnBeforeWrench) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  columns[2].components[kRoll] = std::numeric_limits<double>::infinity();
  Wrench wrench = {};
  wrench[kYaw] = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kSpatialDof> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kColumnNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteWrench) {
  std::array<EffectivenessColumn, kSpatialDof> columns = {
      AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
      AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
  Wrench wrench = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  wrench[kPitch] = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kSpatialDof> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  ASSERT_FALSE(status.ok());
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kWrenchNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, NonFiniteWrenchWinsOverRankDeficiency) {
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge)};
  const Wrench wrench = {
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 1> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kWrenchNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

TEST(UnconstrainedLeastSquares, RejectsNonFiniteGramBeforeRank) {
  // Products overflow. The columns are finite and would also be rank
  // deficient. Gram finiteness is the earlier defect.
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge, 1e200)};
  const Wrench wrench = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 1> thrust = {};
  Wrench residual = {};
  Poison(thrust);
  Poison(residual);
  const AllocationStatus status =
      AllocateUnconstrainedLeastSquares(columns, wrench, thrust, residual);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kGramNonFiniteMessage);
  ExpectFiniteZeros(thrust);
  ExpectFiniteZeros(residual);
}

}  // namespace
