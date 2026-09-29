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

#include "intrinsic/vehicle/allocation/thruster_health_adapter.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/allocation/bounded_allocator.h"
#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::allocation::AllocateBoundedLeastSquares;
using intrinsic::vehicle::allocation::AllocationErrorCode;
using intrinsic::vehicle::allocation::AllocationStatus;
using intrinsic::vehicle::allocation::ApplyThrusterHealthToAllocationInputs;
using intrinsic::vehicle::allocation::BuildThrusterEffectivenessMatrix;
using intrinsic::vehicle::allocation::EffectivenessColumn;
using intrinsic::vehicle::allocation::ThrustCommandBounds;
using intrinsic::vehicle::allocation::ThrustCommandBoundsFromGeometry;
using intrinsic::vehicle::parameters::kBodyFrameId;
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
using intrinsic::vehicle::parameters::Vec3;

constexpr int kDuplicateCount = kSixThrusterUuvThrusterCount + 1;

constexpr std::string_view kEmptyThrustersMessage =
    "thrusters must be non-empty";
constexpr std::string_view kMaskLengthMessage =
    "enabled mask length must equal the thruster count";
constexpr std::string_view kBoundsCountMessage =
    "thrust bounds count must equal the thruster count";
constexpr std::string_view kColumnCountMessage =
    "column count must equal the thruster count";
constexpr std::string_view kDerateNonFiniteMessage =
    "health_derate is not finite";
constexpr std::string_view kUnknownHealthMessage =
    "thruster health is not a known state";
constexpr std::string_view kDerateMismatchMessage =
    "health_derate does not match thruster health";
constexpr std::string_view kThrustLimitNonFiniteMessage =
    "thruster thrust limit is not finite";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kRankDeficientMessage =
    "B B^T is singular; Cholesky pivot is not above the relative tolerance";

using Wrench = std::array<double, kSpatialDof>;

struct BoundedResult {
  AllocationStatus status;
  std::vector<double> thrust;
  Wrench achieved = {};
  Wrench residual = {};
  // bool storage is contiguous. std::vector<bool> is not a valid mask.
  std::unique_ptr<bool[]> saturated_owner;
  std::span<bool> saturated;
  double norm = 0.0;
};

ThrusterGeometry MakeThruster(const char* name, Vec3 direction, double forward,
                              double reverse) {
  ThrusterGeometry thruster;
  thruster.name = name;
  thruster.frame_id = std::string(kBodyFrameId);
  thruster.position_m = {0.0, 0.0, 0.0};
  thruster.direction_body = direction;
  thruster.max_forward_thrust_n = forward;
  thruster.max_reverse_thrust_n = reverse;
  thruster.max_forward_slew_n_per_s = 10.0;
  thruster.max_reverse_slew_n_per_s = 10.0;
  thruster.efficiency = 1.0;
  thruster.health = ThrusterHealthState::kNominal;
  thruster.health_derate = 1.0;
  return thruster;
}

ThrusterGeometry AxisThruster(const char* name, int axis, double forward,
                              double reverse) {
  Vec3 direction = {0.0, 0.0, 0.0};
  if (axis == kSurge) {
    direction.x = 1.0;
  } else if (axis == kSway) {
    direction.y = 1.0;
  } else {
    direction.z = 1.0;
  }
  return MakeThruster(name, direction, forward, reverse);
}

EffectivenessColumn AxisColumn(int axis, double scale = 1.0) {
  EffectivenessColumn column;
  column.components[axis] = scale;
  return column;
}

std::array<EffectivenessColumn, kSpatialDof> IdentityColumns() {
  return {AxisColumn(kSurge), AxisColumn(kSway),  AxisColumn(kHeave),
          AxisColumn(kRoll),  AxisColumn(kPitch), AxisColumn(kYaw)};
}

void Poison(std::span<double> values) {
  for (double& value : values) {
    value = std::numeric_limits<double>::quiet_NaN();
  }
}

void PoisonColumns(std::span<EffectivenessColumn> columns) {
  for (EffectivenessColumn& column : columns) {
    Poison(column.components);
  }
}

void PoisonBounds(std::span<ThrustCommandBounds> bounds) {
  for (ThrustCommandBounds& interval : bounds) {
    interval.min_thrust_n = std::numeric_limits<double>::quiet_NaN();
    interval.max_thrust_n = std::numeric_limits<double>::infinity();
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

void ExpectCleared(std::span<const bool> enabled,
                   std::span<const ThrustCommandBounds> bounds,
                   std::span<const EffectivenessColumn> columns) {
  for (bool flag : enabled) {
    EXPECT_FALSE(flag);
  }
  for (const ThrustCommandBounds& interval : bounds) {
    EXPECT_FALSE(std::signbit(interval.min_thrust_n));
    EXPECT_EQ(interval.min_thrust_n, 0.0);
    EXPECT_FALSE(std::signbit(interval.max_thrust_n));
    EXPECT_EQ(interval.max_thrust_n, 0.0);
  }
  for (const EffectivenessColumn& column : columns) {
    ExpectFiniteZeros(column.components);
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

void ExpectSameColumns(std::span<const EffectivenessColumn> left,
                       std::span<const EffectivenessColumn> right) {
  ASSERT_EQ(left.size(), right.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    ExpectSameBits(left[i].components, right[i].components);
  }
}

void ExpectSameBounds(std::span<const ThrustCommandBounds> left,
                      std::span<const ThrustCommandBounds> right) {
  ASSERT_EQ(left.size(), right.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    EXPECT_EQ(0, std::memcmp(&left[i].min_thrust_n, &right[i].min_thrust_n,
                             sizeof(double)))
        << "thruster " << i;
    EXPECT_EQ(0, std::memcmp(&left[i].max_thrust_n, &right[i].max_thrust_n,
                             sizeof(double)))
        << "thruster " << i;
  }
}

std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> SixColumns(
    const MarineModel& model) {
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns);
  EXPECT_TRUE(status.ok());
  return columns;
}

BoundedResult Allocate(std::span<const EffectivenessColumn> columns,
                       const Wrench& wrench,
                       std::span<const ThrustCommandBounds> bounds) {
  BoundedResult result;
  result.thrust.assign(columns.size(),
                       std::numeric_limits<double>::quiet_NaN());
  result.saturated_owner = std::make_unique<bool[]>(columns.size());
  result.saturated =
      std::span<bool>(result.saturated_owner.get(), columns.size());
  for (bool& flag : result.saturated) {
    flag = true;
  }
  result.achieved.fill(std::numeric_limits<double>::quiet_NaN());
  result.residual.fill(std::numeric_limits<double>::quiet_NaN());
  result.norm = std::numeric_limits<double>::quiet_NaN();
  result.status = AllocateBoundedLeastSquares(
      columns, wrench, bounds, result.thrust, result.achieved, result.residual,
      result.saturated, result.norm);
  return result;
}

void ExpectInsideBounds(std::span<const double> thrust,
                        std::span<const ThrustCommandBounds> bounds) {
  ASSERT_EQ(thrust.size(), bounds.size());
  for (std::size_t i = 0; i < thrust.size(); ++i) {
    EXPECT_GE(thrust[i], bounds[i].min_thrust_n) << "thruster " << i;
    EXPECT_LE(thrust[i], bounds[i].max_thrust_n) << "thruster " << i;
  }
}

// Six example thrusters plus a duplicate of thruster 0. The duplicate is
// the actuator whose health the test changes. Zeroing it leaves the
// original full-rank map.
struct DuplicateRig {
  std::array<ThrusterGeometry, kDuplicateCount> thrusters = {};
  std::array<EffectivenessColumn, kDuplicateCount> columns = {};
  std::array<bool, kDuplicateCount> enabled = {};
  std::array<ThrustCommandBounds, kDuplicateCount> bounds = {};
};

DuplicateRig MakeDuplicateRig(ThrusterHealthState health, double derate) {
  const MarineModel model = MakeSixThrusterUuvExample();
  DuplicateRig rig;
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    rig.thrusters[i] = model.thrusters[i];
  }
  rig.thrusters[kSixThrusterUuvThrusterCount] = model.thrusters[0];
  rig.thrusters[kSixThrusterUuvThrusterCount].name = "surge_port_duplicate";
  rig.thrusters[kSixThrusterUuvThrusterCount].health = health;
  rig.thrusters[kSixThrusterUuvThrusterCount].health_derate = derate;

  std::array<bool, kDuplicateCount> all_enabled = {};
  all_enabled.fill(true);
  EXPECT_TRUE(
      BuildThrusterEffectivenessMatrix(rig.thrusters, all_enabled, rig.columns)
          .ok());
  return rig;
}

void ExpectNeutralDuplicateExcluded(ThrusterHealthState health) {
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  const MarineModel model = MakeSixThrusterUuvExample();
  const auto healthy_columns = SixColumns(model);
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> healthy_bounds =
      {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    healthy_bounds[i] = ThrustCommandBoundsFromGeometry(model.thrusters[i]);
  }
  const BoundedResult healthy =
      Allocate(healthy_columns, wrench, healthy_bounds);
  ASSERT_TRUE(healthy.status.ok());

  DuplicateRig rig = MakeDuplicateRig(health, 0.0);
  const auto unscaled = rig.columns;
  PoisonFlags(rig.enabled);
  PoisonBounds(rig.bounds);
  const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
      rig.thrusters, rig.enabled, rig.bounds, rig.columns);
  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_EQ(status.code, AllocationErrorCode::kOk);

  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_TRUE(rig.enabled[i]) << i;
  }
  EXPECT_FALSE(rig.enabled[kSixThrusterUuvThrusterCount]);
  ExpectSameColumns(std::span<const EffectivenessColumn>(rig.columns)
                        .first(kSixThrusterUuvThrusterCount),
                    healthy_columns);
  ExpectFiniteZeros(rig.columns[kSixThrusterUuvThrusterCount].components);
  ExpectSameBounds(std::span<const ThrustCommandBounds>(rig.bounds)
                       .first(kSixThrusterUuvThrusterCount),
                   healthy_bounds);
  EXPECT_EQ(rig.bounds[kSixThrusterUuvThrusterCount].min_thrust_n, 0.0);
  EXPECT_EQ(rig.bounds[kSixThrusterUuvThrusterCount].max_thrust_n, 0.0);
  EXPECT_FALSE(
      std::signbit(rig.bounds[kSixThrusterUuvThrusterCount].min_thrust_n));
  EXPECT_FALSE(
      std::signbit(rig.bounds[kSixThrusterUuvThrusterCount].max_thrust_n));

  // enabled == false is the builder's zero column. Post-scale matches it.
  std::array<EffectivenessColumn, kDuplicateCount> from_mask = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(rig.thrusters, rig.enabled, from_mask)
          .ok());
  ExpectFiniteZeros(from_mask[kSixThrusterUuvThrusterCount].components);
  ExpectSameColumns(std::span<const EffectivenessColumn>(from_mask).first(
                        kSixThrusterUuvThrusterCount),
                    std::span<const EffectivenessColumn>(unscaled).first(
                        kSixThrusterUuvThrusterCount));

  const BoundedResult allocated = Allocate(rig.columns, wrench, rig.bounds);
  ASSERT_TRUE(allocated.status.ok()) << allocated.status.message;
  ExpectSameBits(std::span<const double>(allocated.thrust)
                     .first(kSixThrusterUuvThrusterCount),
                 healthy.thrust);
  EXPECT_EQ(allocated.thrust[kSixThrusterUuvThrusterCount], 0.0);
  EXPECT_FALSE(std::signbit(allocated.thrust[kSixThrusterUuvThrusterCount]));
  EXPECT_TRUE(allocated.saturated[kSixThrusterUuvThrusterCount]);
  ExpectSameFlagBits(allocated.saturated.first(kSixThrusterUuvThrusterCount),
                     healthy.saturated);
  ExpectSameBits(allocated.achieved, healthy.achieved);
  ExpectSameBits(allocated.residual, healthy.residual);
  EXPECT_EQ(0, std::memcmp(&allocated.norm, &healthy.norm, sizeof(double)));
  ExpectInsideBounds(allocated.thrust, rig.bounds);

  // The neutral column contributes nothing to the wrench map.
  for (int row = 0; row < kSpatialDof; ++row) {
    const double contribution =
        rig.columns[kSixThrusterUuvThrusterCount].components[row] *
        allocated.thrust[kSixThrusterUuvThrusterCount];
    EXPECT_EQ(contribution, 0.0) << "row " << row;
    EXPECT_FALSE(std::signbit(contribution)) << "row " << row;
  }
}

TEST(ThrusterHealthAdapter, NominalHealthMatchesBoundedAllocationBitForBit) {
  MarineModel model = MakeSixThrusterUuvExample();
  for (ThrusterGeometry& thruster : model.thrusters) {
    // Health is unused when every actuator stays nominal. Efficiency and
    // slew are unread, so changing them must not move the #68 result.
    thruster.efficiency = 0.5;
    thruster.max_forward_slew_n_per_s = 1e-3;
    thruster.max_reverse_slew_n_per_s = 1e-3;
    ASSERT_EQ(thruster.health, ThrusterHealthState::kNominal);
    ASSERT_EQ(thruster.health_derate, 1.0);
  }
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  auto built = SixColumns(model);
  // A signed zero must survive the nominal multiply-by-1.
  built[0].components[kYaw] = -0.0;

  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount>
      geometry_bounds = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    geometry_bounds[i] = ThrustCommandBoundsFromGeometry(model.thrusters[i]);
  }
  const BoundedResult direct = Allocate(built, wrench, geometry_bounds);
  ASSERT_TRUE(direct.status.ok());

  auto scaled = built;
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  PoisonFlags(enabled);
  PoisonBounds(bounds);
  const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
      model.thrusters, enabled, bounds, scaled);
  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_EQ(status.message, std::string_view());
  for (bool flag : enabled) {
    EXPECT_TRUE(flag);
  }
  ExpectSameColumns(scaled, built);
  ExpectSameBounds(bounds, geometry_bounds);

  const BoundedResult through_health = Allocate(scaled, wrench, bounds);
  ASSERT_TRUE(through_health.status.ok());
  ExpectSameBits(through_health.thrust, direct.thrust);
  ExpectSameBits(through_health.achieved, direct.achieved);
  ExpectSameBits(through_health.residual, direct.residual);
  ExpectSameFlagBits(through_health.saturated, direct.saturated);
  EXPECT_EQ(0, std::memcmp(&through_health.norm, &direct.norm, sizeof(double)));
}

TEST(ThrusterHealthAdapter, DisabledThrusterCommandIsNeutral) {
  ExpectNeutralDuplicateExcluded(ThrusterHealthState::kDisabled);
}

TEST(ThrusterHealthAdapter, StuckOffThrusterCommandIsNeutral) {
  ExpectNeutralDuplicateExcluded(ThrusterHealthState::kStuckOff);
}

TEST(ThrusterHealthAdapter, FailedThrusterCommandIsNeutral) {
  ExpectNeutralDuplicateExcluded(ThrusterHealthState::kFailed);
}

TEST(ThrusterHealthAdapter, NeutralStatesShareMaskBoundsAndColumns) {
  const DuplicateRig disabled =
      MakeDuplicateRig(ThrusterHealthState::kDisabled, 0.0);
  DuplicateRig stuck = MakeDuplicateRig(ThrusterHealthState::kStuckOff, 0.0);
  DuplicateRig failed = MakeDuplicateRig(ThrusterHealthState::kFailed, -0.0);
  DuplicateRig disabled_out = disabled;
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(
                  disabled_out.thrusters, disabled_out.enabled,
                  disabled_out.bounds, disabled_out.columns)
                  .ok());
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(
                  stuck.thrusters, stuck.enabled, stuck.bounds, stuck.columns)
                  .ok());
  ASSERT_TRUE(
      ApplyThrusterHealthToAllocationInputs(failed.thrusters, failed.enabled,
                                            failed.bounds, failed.columns)
          .ok());
  ExpectSameFlagBits(stuck.enabled, disabled_out.enabled);
  ExpectSameFlagBits(failed.enabled, disabled_out.enabled);
  ExpectSameBounds(stuck.bounds, disabled_out.bounds);
  ExpectSameBounds(failed.bounds, disabled_out.bounds);
  ExpectSameColumns(stuck.columns, disabled_out.columns);
  ExpectSameColumns(failed.columns, disabled_out.columns);
  EXPECT_EQ(failed.bounds[kSixThrusterUuvThrusterCount].min_thrust_n, 0.0);
  EXPECT_FALSE(
      std::signbit(failed.bounds[kSixThrusterUuvThrusterCount].min_thrust_n));
}

TEST(ThrusterHealthAdapter, DeratedScalesBoundsAndEffectivenessTogether) {
  // Identity columns. Surge is derated by 1/2. A request of 100 N on a
  // column of 1/2 asks for command 200, which clamps to the scaled bound
  // 5. Achieved surge is (1/2) * 5 = 2.5. Sway stays 3: surplus surge is
  // not moved onto another actuator.
  std::array<ThrusterGeometry, kSpatialDof> thrusters = {
      AxisThruster("surge", kSurge, 10.0, 10.0),
      AxisThruster("sway", kSway, 10.0, 10.0),
      AxisThruster("heave", kHeave, 10.0, 10.0),
      AxisThruster("roll", kSurge, 10.0, 10.0),
      AxisThruster("pitch", kSway, 10.0, 10.0),
      AxisThruster("yaw", kHeave, 10.0, 10.0),
  };
  thrusters[kSurge].health = ThrusterHealthState::kDerated;
  thrusters[kSurge].health_derate = 0.5;
  thrusters[kSurge].efficiency = 0.25;
  thrusters[kSurge].max_forward_slew_n_per_s = 0.01;
  thrusters[kSurge].max_reverse_slew_n_per_s = 0.01;
  for (int i = kSway; i < kSpatialDof; ++i) {
    thrusters[i].efficiency = 0.7;
  }

  std::array<ThrusterGeometry, kSpatialDof> unread_twin = thrusters;
  unread_twin[kSurge].efficiency = 1.0;
  unread_twin[kSurge].max_forward_slew_n_per_s = 100.0;
  unread_twin[kSurge].max_reverse_slew_n_per_s = 100.0;

  auto columns = IdentityColumns();
  const auto unscaled = columns;
  std::array<bool, kSpatialDof> enabled = {};
  std::array<ThrustCommandBounds, kSpatialDof> bounds = {};
  ASSERT_TRUE(
      ApplyThrusterHealthToAllocationInputs(thrusters, enabled, bounds, columns)
          .ok());

  auto twin_columns = IdentityColumns();
  std::array<bool, kSpatialDof> twin_enabled = {};
  std::array<ThrustCommandBounds, kSpatialDof> twin_bounds = {};
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(unread_twin, twin_enabled,
                                                    twin_bounds, twin_columns)
                  .ok());
  ExpectSameFlagBits(twin_enabled, enabled);
  ExpectSameBounds(twin_bounds, bounds);
  ExpectSameColumns(twin_columns, columns);

  EXPECT_TRUE(enabled[kSurge]);
  for (int row = 0; row < kSpatialDof; ++row) {
    const double expected = unscaled[kSurge].components[row] * 0.5;
    EXPECT_EQ(0, std::memcmp(&columns[kSurge].components[row], &expected,
                             sizeof(double)))
        << "row " << row;
  }
  // 0.25 efficiency must not be a second scale factor.
  EXPECT_EQ(columns[kSurge].components[kSurge], 0.5);
  const ThrustCommandBounds geometry =
      ThrustCommandBoundsFromGeometry(thrusters[kSurge]);
  const double expected_min = geometry.min_thrust_n * 0.5;
  const double expected_max = geometry.max_thrust_n * 0.5;
  EXPECT_EQ(0, std::memcmp(&bounds[kSurge].min_thrust_n, &expected_min,
                           sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&bounds[kSurge].max_thrust_n, &expected_max,
                           sizeof(double)));
  EXPECT_EQ(bounds[kSurge].min_thrust_n, -5.0);
  EXPECT_EQ(bounds[kSurge].max_thrust_n, 5.0);
  for (int i = kSway; i < kSpatialDof; ++i) {
    EXPECT_TRUE(enabled[i]) << i;
    ExpectSameBits(columns[i].components, unscaled[i].components);
    EXPECT_EQ(bounds[i].min_thrust_n, -10.0) << i;
    EXPECT_EQ(bounds[i].max_thrust_n, 10.0) << i;
  }

  const Wrench wrench = {100.0, 3.0, 0.0, 0.0, 0.0, 0.0};
  const BoundedResult allocated = Allocate(columns, wrench, bounds);
  ASSERT_TRUE(allocated.status.ok()) << allocated.status.message;
  EXPECT_EQ(allocated.thrust[kSurge], 5.0);
  EXPECT_EQ(allocated.thrust[kSway], 3.0);
  for (int i = kHeave; i < kSpatialDof; ++i) {
    EXPECT_EQ(allocated.thrust[i], 0.0) << i;
    EXPECT_FALSE(std::signbit(allocated.thrust[i])) << i;
  }
  EXPECT_TRUE(allocated.saturated[kSurge]);
  EXPECT_FALSE(allocated.saturated[kSway]);
  EXPECT_EQ(allocated.achieved[kSurge], 2.5);
  EXPECT_EQ(allocated.achieved[kSway], 3.0);
  EXPECT_EQ(allocated.residual[kSurge], 97.5);
  EXPECT_EQ(allocated.residual[kSway], 0.0);
  EXPECT_EQ(allocated.norm, 97.5);
  ExpectInsideBounds(allocated.thrust, bounds);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_EQ(allocated.achieved[row] + allocated.residual[row], wrench[row])
        << "row " << row;
  }
}

TEST(ThrusterHealthAdapter, SingleFailureRemainingThrustersAllocate) {
  const Wrench wrench = {1.0e6, -1.0e6, 1.0e6, 1.0e5, -1.0e5, 1.0e5};
  const MarineModel model = MakeSixThrusterUuvExample();
  const auto healthy_columns = SixColumns(model);
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> healthy_bounds =
      {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    healthy_bounds[i] = ThrustCommandBoundsFromGeometry(model.thrusters[i]);
  }
  const BoundedResult healthy =
      Allocate(healthy_columns, wrench, healthy_bounds);
  ASSERT_TRUE(healthy.status.ok());
  ASSERT_GT(healthy.norm, 0.0);

  DuplicateRig rig = MakeDuplicateRig(ThrusterHealthState::kFailed, 0.0);
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(rig.thrusters, rig.enabled,
                                                    rig.bounds, rig.columns)
                  .ok());
  const BoundedResult allocated = Allocate(rig.columns, wrench, rig.bounds);
  ASSERT_TRUE(allocated.status.ok()) << allocated.status.message;
  EXPECT_EQ(allocated.status.code, AllocationErrorCode::kOk);
  ExpectSameBits(std::span<const double>(allocated.thrust)
                     .first(kSixThrusterUuvThrusterCount),
                 healthy.thrust);
  EXPECT_EQ(allocated.thrust[kSixThrusterUuvThrusterCount], 0.0);
  EXPECT_FALSE(std::signbit(allocated.thrust[kSixThrusterUuvThrusterCount]));
  ExpectSameBits(allocated.achieved, healthy.achieved);
  ExpectSameBits(allocated.residual, healthy.residual);
  EXPECT_EQ(0, std::memcmp(&allocated.norm, &healthy.norm, sizeof(double)));
  EXPECT_GT(allocated.norm, 0.0);
  ExpectInsideBounds(allocated.thrust, rig.bounds);
  ExpectFiniteZeros(rig.columns[kSixThrusterUuvThrusterCount].components);
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_EQ(allocated.achieved[row] + allocated.residual[row], wrench[row])
        << "row " << row;
  }
}

TEST(ThrusterHealthAdapter, SingleFailureRankDeficiencyReportsResidual) {
  // One failed thruster and no spare drops the six-column map below full
  // row rank. The adapter does not invent a solver: #68 returns
  // kRankDeficient, zero commands, achieved 0, and residual τ.
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kFailed;
  model.thrusters[0].health_derate = 0.0;
  const Wrench wrench = {2.0, -1.0, 0.5, 0.1, -0.2, 0.3};
  auto columns = SixColumns(model);
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(model.thrusters, enabled,
                                                    bounds, columns)
                  .ok());
  EXPECT_FALSE(enabled[0]);
  ExpectFiniteZeros(columns[0].components);
  EXPECT_EQ(bounds[0].min_thrust_n, 0.0);
  EXPECT_EQ(bounds[0].max_thrust_n, 0.0);

  const BoundedResult allocated = Allocate(columns, wrench, bounds);
  EXPECT_EQ(allocated.status.code, AllocationErrorCode::kRankDeficient);
  EXPECT_EQ(allocated.status.message, kRankDeficientMessage);
  ExpectFiniteZeros(allocated.thrust);
  ExpectFiniteZeros(allocated.achieved);
  ExpectSameBits(allocated.residual, wrench);
  for (bool flag : allocated.saturated) {
    EXPECT_FALSE(flag);
  }
  double sum_of_squares = 0.0;
  for (double value : wrench) {
    sum_of_squares += value * value;
  }
  const double expected_norm = std::sqrt(sum_of_squares);
  EXPECT_EQ(0, std::memcmp(&allocated.norm, &expected_norm, sizeof(double)));
  EXPECT_GT(allocated.norm, 0.0);
}

TEST(ThrusterHealthAdapter, DeterministicForTheSameHealthInputs) {
  // Derate keeps the six-column map full rank. A neutral actuator would
  // drop row rank and the bounded solver would fail closed. That path is
  // covered by the rank-deficiency fixture.
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[1].health = ThrusterHealthState::kDerated;
  model.thrusters[1].health_derate = 0.4;
  model.thrusters[4].health = ThrusterHealthState::kDerated;
  model.thrusters[4].health_derate = 0.25;
  const auto built = SixColumns(model);
  const Wrench wrench = {4.0, -2.0, 1.0, 0.2, -0.3, 0.4};

  auto first_columns = built;
  auto second_columns = built;
  std::array<bool, kSixThrusterUuvThrusterCount> first_enabled = {};
  std::array<bool, kSixThrusterUuvThrusterCount> second_enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> first_bounds =
      {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> second_bounds =
      {};
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(
                  model.thrusters, first_enabled, first_bounds, first_columns)
                  .ok());
  ASSERT_TRUE(
      ApplyThrusterHealthToAllocationInputs(model.thrusters, second_enabled,
                                            second_bounds, second_columns)
          .ok());
  ExpectSameFlagBits(first_enabled, second_enabled);
  ExpectSameBounds(first_bounds, second_bounds);
  ExpectSameColumns(first_columns, second_columns);

  const BoundedResult first = Allocate(first_columns, wrench, first_bounds);
  const BoundedResult second = Allocate(second_columns, wrench, second_bounds);
  ASSERT_TRUE(first.status.ok());
  ASSERT_TRUE(second.status.ok());
  ExpectSameBits(first.thrust, second.thrust);
  ExpectSameBits(first.achieved, second.achieved);
  ExpectSameBits(first.residual, second.residual);
  ExpectSameFlagBits(first.saturated, second.saturated);
  EXPECT_EQ(0, std::memcmp(&first.norm, &second.norm, sizeof(double)));
  ExpectInsideBounds(first.thrust, first_bounds);
}

TEST(ThrusterHealthAdapter, MaskThenBuildThenScaleMatchesPostScale) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kDerated;
  model.thrusters[0].health_derate = 0.4;
  model.thrusters[1].health = ThrusterHealthState::kDisabled;
  model.thrusters[1].health_derate = 0.0;
  model.thrusters[2].health = ThrusterHealthState::kStuckOff;
  model.thrusters[2].health_derate = 0.0;
  model.thrusters[3].health = ThrusterHealthState::kFailed;
  model.thrusters[3].health_derate = 0.0;

  auto post_columns = SixColumns(model);
  std::array<bool, kSixThrusterUuvThrusterCount> post_enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> post_bounds =
      {};
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(
                  model.thrusters, post_enabled, post_bounds, post_columns)
                  .ok());

  std::array<bool, kSixThrusterUuvThrusterCount> mask = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  ASSERT_TRUE(
      ApplyThrusterHealthToAllocationInputs(model.thrusters, mask, bounds, {})
          .ok());
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> built = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, mask, built).ok());
  std::array<bool, kSixThrusterUuvThrusterCount> enabled_again = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds_again =
      {};
  ASSERT_TRUE(ApplyThrusterHealthToAllocationInputs(
                  model.thrusters, enabled_again, bounds_again, built)
                  .ok());

  ExpectSameFlagBits(post_enabled, mask);
  ExpectSameFlagBits(enabled_again, mask);
  ExpectSameBounds(post_bounds, bounds);
  ExpectSameBounds(bounds_again, bounds);
  ExpectSameColumns(post_columns, built);
  EXPECT_FALSE(mask[1]);
  EXPECT_FALSE(mask[2]);
  EXPECT_FALSE(mask[3]);
  EXPECT_TRUE(mask[0]);
  EXPECT_TRUE(mask[4]);
  ExpectFiniteZeros(post_columns[1].components);
  ExpectFiniteZeros(post_columns[2].components);
  ExpectFiniteZeros(post_columns[3].components);
}

TEST(ThrusterHealthAdapter, EmptyColumnSpanDoesNotWriteColumns) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> untouched = {};
  PoisonColumns(untouched);
  const auto poisoned = untouched;
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
      model.thrusters, enabled, bounds, {});
  ASSERT_TRUE(status.ok()) << status.message;
  ExpectSameColumns(untouched, poisoned);
  for (bool flag : enabled) {
    EXPECT_TRUE(flag);
  }
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    const ThrustCommandBounds geometry =
        ThrustCommandBoundsFromGeometry(model.thrusters[i]);
    EXPECT_EQ(0, std::memcmp(&bounds[i].min_thrust_n, &geometry.min_thrust_n,
                             sizeof(double)))
        << i;
    EXPECT_EQ(0, std::memcmp(&bounds[i].max_thrust_n, &geometry.max_thrust_n,
                             sizeof(double)))
        << i;
  }
}

TEST(ThrusterHealthAdapter, NonFiniteHealthDerateFailsClosed) {
  const double samples[] = {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()};
  for (double derate : samples) {
    MarineModel model = MakeSixThrusterUuvExample();
    model.thrusters[2].health_derate = derate;
    auto columns = SixColumns(model);
    std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
    std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
    PoisonFlags(enabled);
    PoisonBounds(bounds);
    PoisonColumns(columns);
    const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
        model.thrusters, enabled, bounds, columns);
    EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
    EXPECT_EQ(status.message, kDerateNonFiniteMessage);
    ExpectCleared(enabled, bounds, columns);
  }
}

TEST(ThrusterHealthAdapter, SizeMismatchFailsClosed) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<ThrusterGeometry, 2> two = {model.thrusters[0],
                                         model.thrusters[1]};
  std::array<bool, 2> enabled = {true, true};
  std::array<ThrustCommandBounds, 2> bounds = {};
  std::array<EffectivenessColumn, 2> columns = {};
  PoisonFlags(enabled);
  PoisonBounds(bounds);
  PoisonColumns(columns);
  const AllocationStatus mask = ApplyThrusterHealthToAllocationInputs(
      std::span<const ThrusterGeometry>(two).first(1), enabled,
      std::span<ThrustCommandBounds>(bounds).first(1),
      std::span<EffectivenessColumn>(columns).first(1));
  EXPECT_EQ(mask.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(mask.message, kMaskLengthMessage);
  ExpectCleared(enabled, std::span<const ThrustCommandBounds>(bounds).first(1),
                std::span<const EffectivenessColumn>(columns).first(1));

  PoisonFlags(enabled);
  PoisonBounds(bounds);
  PoisonColumns(columns);
  const AllocationStatus bound = ApplyThrusterHealthToAllocationInputs(
      two, enabled, std::span<ThrustCommandBounds>(bounds).first(1), columns);
  EXPECT_EQ(bound.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(bound.message, kBoundsCountMessage);
  ExpectCleared(enabled, std::span<const ThrustCommandBounds>(bounds).first(1),
                columns);

  PoisonFlags(enabled);
  PoisonBounds(bounds);
  PoisonColumns(columns);
  const AllocationStatus column = ApplyThrusterHealthToAllocationInputs(
      two, enabled, bounds, std::span<EffectivenessColumn>(columns).first(1));
  EXPECT_EQ(column.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(column.message, kColumnCountMessage);
  ExpectCleared(enabled, bounds,
                std::span<const EffectivenessColumn>(columns).first(1));
}

TEST(ThrusterHealthAdapter, EmptyThrustersFailClosed) {
  std::array<bool, 1> enabled = {true};
  std::array<ThrustCommandBounds, 1> bounds = {};
  std::array<EffectivenessColumn, 1> columns = {};
  PoisonBounds(bounds);
  PoisonColumns(columns);
  const AllocationStatus status =
      ApplyThrusterHealthToAllocationInputs({}, enabled, bounds, columns);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kEmptyThrustersMessage);
  ExpectCleared(enabled, bounds, columns);
}

TEST(ThrusterHealthAdapter, HealthDerateMismatchFailsClosed) {
  const MarineModel base = MakeSixThrusterUuvExample();
  const struct Sample {
    ThrusterHealthState health;
    double derate;
  } samples[] = {
      {ThrusterHealthState::kNominal, 0.5},
      {ThrusterHealthState::kNominal, 0.0},
      {ThrusterHealthState::kDerated, 0.0},
      {ThrusterHealthState::kDerated, 1.0},
      {ThrusterHealthState::kDerated, -0.2},
      {ThrusterHealthState::kDisabled, 1.0},
      {ThrusterHealthState::kStuckOff, 0.2},
      {ThrusterHealthState::kFailed, 0.2},
  };
  for (const Sample& sample : samples) {
    std::array<ThrusterGeometry, 1> thrusters = {base.thrusters[0]};
    thrusters[0].health = sample.health;
    thrusters[0].health_derate = sample.derate;
    std::array<bool, 1> enabled = {true};
    std::array<ThrustCommandBounds, 1> bounds = {};
    std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge, 4.0)};
    PoisonBounds(bounds);
    const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
        thrusters, enabled, bounds, columns);
    EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument)
        << sample.derate;
    EXPECT_EQ(status.message, kDerateMismatchMessage);
    ExpectCleared(enabled, bounds, columns);
  }
}

TEST(ThrusterHealthAdapter, UnknownHealthFailsClosed) {
  const MarineModel base = MakeSixThrusterUuvExample();
  std::array<ThrusterGeometry, 1> thrusters = {base.thrusters[0]};
  thrusters[0].health = static_cast<ThrusterHealthState>(99);
  thrusters[0].health_derate = 1.0;
  std::array<bool, 1> enabled = {true};
  std::array<ThrustCommandBounds, 1> bounds = {};
  std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge)};
  PoisonBounds(bounds);
  const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kUnknownHealthMessage);
  ExpectCleared(enabled, bounds, columns);
}

TEST(ThrusterHealthAdapter, NonFiniteThrustLimitFailsClosed) {
  const MarineModel base = MakeSixThrusterUuvExample();
  const double samples[] = {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()};
  for (double limit : samples) {
    std::array<ThrusterGeometry, 1> thrusters = {base.thrusters[0]};
    thrusters[0].health = ThrusterHealthState::kFailed;
    thrusters[0].health_derate = 0.0;
    thrusters[0].max_forward_thrust_n = limit;
    std::array<bool, 1> enabled = {true};
    std::array<ThrustCommandBounds, 1> bounds = {};
    std::array<EffectivenessColumn, 1> columns = {AxisColumn(kSurge, 3.0)};
    const AllocationStatus forward = ApplyThrusterHealthToAllocationInputs(
        thrusters, enabled, bounds, columns);
    EXPECT_EQ(forward.code, AllocationErrorCode::kInvalidArgument);
    EXPECT_EQ(forward.message, kThrustLimitNonFiniteMessage);
    ExpectCleared(enabled, bounds, columns);

    thrusters[0].max_forward_thrust_n = 10.0;
    thrusters[0].max_reverse_thrust_n = limit;
    columns[0] = AxisColumn(kSurge, 3.0);
    enabled[0] = true;
    const AllocationStatus reverse = ApplyThrusterHealthToAllocationInputs(
        thrusters, enabled, bounds, columns);
    EXPECT_EQ(reverse.code, AllocationErrorCode::kInvalidArgument);
    EXPECT_EQ(reverse.message, kThrustLimitNonFiniteMessage);
    ExpectCleared(enabled, bounds, columns);
  }
}

TEST(ThrusterHealthAdapter, NonFiniteColumnFailsClosed) {
  const MarineModel model = MakeSixThrusterUuvExample();
  auto columns = SixColumns(model);
  columns[3].components[kPitch] = std::numeric_limits<double>::quiet_NaN();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  std::array<ThrustCommandBounds, kSixThrusterUuvThrusterCount> bounds = {};
  PoisonFlags(enabled);
  PoisonBounds(bounds);
  const AllocationStatus status = ApplyThrusterHealthToAllocationInputs(
      model.thrusters, enabled, bounds, columns);
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument);
  EXPECT_EQ(status.message, kColumnNonFiniteMessage);
  ExpectCleared(enabled, bounds, columns);
}

TEST(ThrusterHealthAdapter, FirstDefectWins) {
  const MarineModel base = MakeSixThrusterUuvExample();
  std::array<ThrusterGeometry, 2> thrusters = {base.thrusters[0],
                                               base.thrusters[1]};
  std::array<bool, 2> enabled = {};
  std::array<ThrustCommandBounds, 2> bounds = {};
  std::array<EffectivenessColumn, 2> columns = {AxisColumn(kSurge),
                                                AxisColumn(kSway)};

  const AllocationStatus empty =
      ApplyThrusterHealthToAllocationInputs({}, enabled, bounds, columns);
  EXPECT_EQ(empty.message, kEmptyThrustersMessage);

  thrusters[0].health_derate = std::numeric_limits<double>::quiet_NaN();
  const AllocationStatus mask = ApplyThrusterHealthToAllocationInputs(
      std::span<const ThrusterGeometry>(thrusters).first(1), enabled, bounds,
      columns);
  EXPECT_EQ(mask.message, kMaskLengthMessage);

  const AllocationStatus bound = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, std::span<ThrustCommandBounds>(bounds).first(1),
      columns);
  EXPECT_EQ(bound.message, kBoundsCountMessage);

  const AllocationStatus column = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds,
      std::span<EffectivenessColumn>(columns).first(1));
  EXPECT_EQ(column.message, kColumnCountMessage);

  thrusters[1].health = static_cast<ThrusterHealthState>(99);
  thrusters[1].health_derate = 1.0;
  const AllocationStatus derate = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(derate.message, kDerateNonFiniteMessage);

  thrusters[0].health_derate = 1.0;
  const AllocationStatus unknown = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(unknown.message, kUnknownHealthMessage);

  thrusters[1].health = ThrusterHealthState::kNominal;
  thrusters[1].health_derate = 1.0;
  thrusters[0].health_derate = 0.5;
  thrusters[1].max_forward_thrust_n = std::numeric_limits<double>::quiet_NaN();
  const AllocationStatus mismatch = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(mismatch.message, kDerateMismatchMessage);

  thrusters[0].health_derate = 1.0;
  const AllocationStatus limit = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(limit.message, kThrustLimitNonFiniteMessage);

  thrusters[1].max_forward_thrust_n = base.thrusters[1].max_forward_thrust_n;
  columns[1].components[kYaw] = std::numeric_limits<double>::infinity();
  const AllocationStatus non_finite = ApplyThrusterHealthToAllocationInputs(
      thrusters, enabled, bounds, columns);
  EXPECT_EQ(non_finite.message, kColumnNonFiniteMessage);
  ExpectCleared(enabled, bounds, columns);
}

}  // namespace
