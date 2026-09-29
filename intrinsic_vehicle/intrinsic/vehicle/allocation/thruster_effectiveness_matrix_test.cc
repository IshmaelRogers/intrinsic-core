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

#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::allocation::AllocationErrorCode;
using intrinsic::vehicle::allocation::AllocationStatus;
using intrinsic::vehicle::allocation::BuildThrusterEffectivenessMatrix;
using intrinsic::vehicle::allocation::EffectivenessColumn;
using intrinsic::vehicle::parameters::kBodyFrameId;
using intrinsic::vehicle::parameters::kHeave;
using intrinsic::vehicle::parameters::kPitch;
using intrinsic::vehicle::parameters::kRoll;
using intrinsic::vehicle::parameters::kSixThrusterUuvThrusterCount;
using intrinsic::vehicle::parameters::kSpatialDof;
using intrinsic::vehicle::parameters::kSurge;
using intrinsic::vehicle::parameters::kSway;
using intrinsic::vehicle::parameters::kWorldEnuFrameId;
using intrinsic::vehicle::parameters::kWorldNedFrameId;
using intrinsic::vehicle::parameters::kYaw;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::ModelErrorCode;
using intrinsic::vehicle::parameters::ThrusterGeometry;
using intrinsic::vehicle::parameters::ThrusterHealthState;
using intrinsic::vehicle::parameters::ValidateMarineModel;
using intrinsic::vehicle::parameters::Vec3;

constexpr double kAbsTolerance = 1e-12;

constexpr std::string_view kEmptyThrustersMessage =
    "thrusters must be non-empty";
constexpr std::string_view kMaskLengthMessage =
    "enabled mask length must equal the thruster count";
constexpr std::string_view kColumnCountMessage =
    "column count must equal the thruster count";
constexpr std::string_view kFrameMessage = "thruster frame_id must be body";
constexpr std::string_view kPositionNonFiniteMessage =
    "thruster position_m must be finite";
constexpr std::string_view kDirectionNonFiniteMessage =
    "thruster direction_body must be finite";
constexpr std::string_view kZeroAxisMessage =
    "thruster direction_body must be a non-zero unit vector";
constexpr std::string_view kNotUnitMessage =
    "thruster direction_body must be a unit vector";

// Hand-calculated columns for MakeSixThrusterUuvExample, unit thrust.
// surge_port r=(0, 0.18, 0), u=(1, 0, 0): r×u=(0, 0, -0.18)
// surge_starboard r=(0, -0.18, 0.10), u=(1, 0, 0): r×u=(0, 0.10, 0.18)
// sway_fore r=(0.40, 0, 0), u=(0, 1, 0): r×u=(0, 0, 0.40)
// sway_aft r=(-0.40, 0, -0.12), u=(0, 1, 0): r×u=(0.12, 0, -0.40)
// heave_fore r=(0.28, 0.16, 0), u=(0, 0, 1): r×u=(0.16, -0.28, 0)
// heave_aft r=(-0.28, 0, 0), u=(0, 0, 1): r×u=(0, 0.28, 0)
constexpr double
    kSixThrusterColumns[kSixThrusterUuvThrusterCount][kSpatialDof] = {
        {1, 0, 0, 0, 0, -0.18},    {1, 0, 0, 0, 0.10, 0.18},
        {0, 1, 0, 0, 0, 0.40},     {0, 1, 0, 0.12, 0, -0.40},
        {0, 0, 1, 0.16, -0.28, 0}, {0, 0, 1, 0, 0.28, 0},
};

ThrusterGeometry UnitThruster(Vec3 position_m, Vec3 direction_body) {
  ThrusterGeometry thruster;
  thruster.name = "thruster";
  thruster.frame_id = std::string(kBodyFrameId);
  thruster.position_m = position_m;
  thruster.direction_body = direction_body;
  thruster.max_forward_thrust_n = 10;
  thruster.max_reverse_thrust_n = 5;
  thruster.max_forward_slew_n_per_s = 20;
  thruster.max_reverse_slew_n_per_s = 15;
  thruster.efficiency = 1;
  thruster.health = ThrusterHealthState::kNominal;
  thruster.health_derate = 1;
  return thruster;
}

void ExpectColumn(const EffectivenessColumn& actual,
                  const std::array<double, kSpatialDof>& expected) {
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_NEAR(actual.components[row], expected[row], kAbsTolerance)
        << "row " << row;
  }
}

void ExpectZeroColumn(const EffectivenessColumn& column) {
  for (double value : column.components) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_EQ(value, 0.0);
  }
}

void Poison(std::span<EffectivenessColumn> columns) {
  for (EffectivenessColumn& column : columns) {
    column.components.fill(std::numeric_limits<double>::quiet_NaN());
  }
}

void ExpectInvalid(const char* name, std::string_view message,
                   std::span<const ThrusterGeometry> thrusters,
                   std::span<const bool> enabled,
                   std::span<EffectivenessColumn> columns) {
  Poison(columns);
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(thrusters, enabled, columns);
  ASSERT_FALSE(status.ok()) << name;
  EXPECT_EQ(status.code, AllocationErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(status.message, message) << name;
  for (const EffectivenessColumn& column : columns) {
    ExpectZeroColumn(column);
  }
}

TEST(ThrusterEffectivenessMatrix, SingleThrusterColumnMatchesHandCalculation) {
  // r = (1, 2, -4) m, u = (0, 0, 1).
  // force = (0, 0, 1) N/N.
  // moment = r × u = (2*1 - (-4)*0, (-4)*0 - 1*1, 1*0 - 2*0) = (2, -1, 0)
  // N·m/N.
  const ThrusterGeometry thruster = UnitThruster(Vec3{1, 2, -4}, Vec3{0, 0, 1});
  const std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 1> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  EXPECT_EQ(status.message, std::string_view());
  ExpectColumn(columns[0], {0, 0, 1, 2, -1, 0});
  EXPECT_DOUBLE_EQ(columns[0].components[kHeave], 1.0);
  EXPECT_DOUBLE_EQ(columns[0].components[kRoll], 2.0);
  EXPECT_DOUBLE_EQ(columns[0].components[kPitch], -1.0);
  EXPECT_DOUBLE_EQ(columns[0].components[kYaw], 0.0);
}

TEST(ThrusterEffectivenessMatrix, SlantedAxisColumnMatchesHandCrossProduct) {
  // r = (1, 2, 3) m, u = (0, 0.6, 0.8), a 3-4-5 unit axis.
  // force = (0, 0.6, 0.8) N/N.
  // moment = r × u = (2*0.8 - 3*0.6, 3*0 - 1*0.8, 1*0.6 - 2*0)
  //        = (-0.2, -0.8, 0.6) N·m/N.
  const ThrusterGeometry thruster =
      UnitThruster(Vec3{1, 2, 3}, Vec3{0, 0.6, 0.8});
  const std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 1> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  ExpectColumn(columns[0], {0, 0.6, 0.8, -0.2, -0.8, 0.6});
}

TEST(ThrusterEffectivenessMatrix, SixThrusterColumnsMatchHandCalculation) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_TRUE(ValidateMarineModel(model).ok());
  ASSERT_EQ(model.thrusters.size(),
            static_cast<size_t>(kSixThrusterUuvThrusterCount));
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    std::array<double, kSpatialDof> expected = {};
    for (int row = 0; row < kSpatialDof; ++row) {
      expected[row] = kSixThrusterColumns[i][row];
    }
    ExpectColumn(columns[i], expected);
  }

  // τ = B u for a chosen thrust vector. This checks the linear map.
  // It does not solve for u.
  const std::array<double, kSixThrusterUuvThrusterCount> thrust = {
      1.0, -0.5, 0.0, 2.0, 0.25, -1.0};
  std::array<double, kSpatialDof> wrench = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    for (int row = 0; row < kSpatialDof; ++row) {
      wrench[row] += thrust[i] * columns[i].components[row];
    }
  }
  std::array<double, kSpatialDof> expected = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    for (int row = 0; row < kSpatialDof; ++row) {
      expected[row] += thrust[i] * kSixThrusterColumns[i][row];
    }
  }
  for (int row = 0; row < kSpatialDof; ++row) {
    EXPECT_NEAR(wrench[row], expected[row], kAbsTolerance) << "row " << row;
  }
}

TEST(ThrusterEffectivenessMatrix, MaskZerosDisabledColumnAndKeepsIndex) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  enabled[2] = false;
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  ExpectZeroColumn(columns[2]);
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    if (i == 2) {
      continue;
    }
    std::array<double, kSpatialDof> expected = {};
    for (int row = 0; row < kSpatialDof; ++row) {
      expected[row] = kSixThrusterColumns[i][row];
    }
    ExpectColumn(columns[i], expected);
  }

  // The masked thruster drops out of τ = B u. Column 2 stays in place.
  std::array<double, kSpatialDof> wrench = {};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    for (int row = 0; row < kSpatialDof; ++row) {
      wrench[row] += columns[i].components[row];
    }
  }
  // Sum of the unmasked hand columns. Column 2 (sway_fore) is excluded.
  EXPECT_NEAR(wrench[kSurge], 2.0, kAbsTolerance);
  EXPECT_NEAR(wrench[kSway], 1.0, kAbsTolerance);
  EXPECT_NEAR(wrench[kHeave], 2.0, kAbsTolerance);
  EXPECT_NEAR(wrench[kRoll], 0.28, kAbsTolerance);
  EXPECT_NEAR(wrench[kPitch], 0.10, kAbsTolerance);
  EXPECT_NEAR(wrench[kYaw], -0.40, kAbsTolerance);
}

TEST(ThrusterEffectivenessMatrix, AllDisabledMaskYieldsZeroColumns) {
  const ThrusterGeometry thruster = UnitThruster(Vec3{1, 0, 0}, Vec3{0, 1, 0});
  const std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 1> enabled = {false};
  std::array<EffectivenessColumn, 1> columns = {};
  Poison(columns);
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  ExpectZeroColumn(columns[0]);
}

TEST(ThrusterEffectivenessMatrix, HealthEfficiencyAndBoundsDoNotScaleColumn) {
  ThrusterGeometry thruster = UnitThruster(Vec3{1, 2, -4}, Vec3{0, 0, 1});
  thruster.health = ThrusterHealthState::kDisabled;
  thruster.health_derate = 0;
  thruster.efficiency = 0.25;
  thruster.max_forward_thrust_n = 50;
  thruster.max_reverse_thrust_n = 1;
  const std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 1> columns = {};
  const AllocationStatus status =
      BuildThrusterEffectivenessMatrix(thrusters, enabled, columns);
  ASSERT_TRUE(status.ok());
  ExpectColumn(columns[0], {0, 0, 1, 2, -1, 0});
}

TEST(ThrusterEffectivenessMatrix, RepeatedCallsMatch) {
  const MarineModel model = MakeSixThrusterUuvExample();
  std::array<bool, kSixThrusterUuvThrusterCount> enabled = {};
  enabled.fill(true);
  enabled[4] = false;
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> first = {};
  std::array<EffectivenessColumn, kSixThrusterUuvThrusterCount> second = {};
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, first).ok());
  ASSERT_TRUE(
      BuildThrusterEffectivenessMatrix(model.thrusters, enabled, second).ok());
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_EQ(first[i].components, second[i].components) << i;
  }
}

TEST(ThrusterEffectivenessMatrix, RejectsEmptyThrusters) {
  std::array<EffectivenessColumn, 1> columns = {};
  ExpectInvalid("empty", kEmptyThrustersMessage, {}, {}, columns);
}

TEST(ThrusterEffectivenessMatrix, RejectsMaskAndColumnLength) {
  const ThrusterGeometry thruster = UnitThruster(Vec3{0, 0, 0}, Vec3{1, 0, 0});
  const std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 2> long_mask = {true, false};
  std::array<EffectivenessColumn, 1> one_column = {};
  ExpectInvalid("mask", kMaskLengthMessage, thrusters, long_mask, one_column);

  const std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 2> two_columns = {};
  ExpectInvalid("columns", kColumnCountMessage, thrusters, enabled,
                two_columns);
}

TEST(ThrusterEffectivenessMatrix, RejectsNonFiniteGeometry) {
  ThrusterGeometry thruster = UnitThruster(Vec3{0, 0, 0}, Vec3{1, 0, 0});
  thruster.position_m.y = std::numeric_limits<double>::quiet_NaN();
  std::array<ThrusterGeometry, 1> thrusters = {thruster};
  const std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 1> columns = {};
  ExpectInvalid("nan position", kPositionNonFiniteMessage, thrusters, enabled,
                columns);

  thruster = UnitThruster(Vec3{0, 0, 0}, Vec3{1, 0, 0});
  thruster.position_m.z = std::numeric_limits<double>::infinity();
  thrusters[0] = thruster;
  ExpectInvalid("infinite position", kPositionNonFiniteMessage, thrusters,
                enabled, columns);

  thruster = UnitThruster(Vec3{0, 0, 0}, Vec3{1, 0, 0});
  thruster.direction_body.x = std::numeric_limits<double>::quiet_NaN();
  thrusters[0] = thruster;
  ExpectInvalid("nan direction", kDirectionNonFiniteMessage, thrusters, enabled,
                columns);
}

TEST(ThrusterEffectivenessMatrix, RejectsUnvalidatedGeometry) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].direction_body = Vec3{2, 0, 0};
  const auto validation = ValidateMarineModel(model);
  ASSERT_FALSE(validation.ok());
  EXPECT_EQ(validation.errors[0].code, ModelErrorCode::kNotUnit);

  std::array<bool, 1> enabled = {true};
  std::array<EffectivenessColumn, 1> columns = {};
  const std::array<ThrusterGeometry, 1> one = {model.thrusters[0]};
  ExpectInvalid("non-unit", kNotUnitMessage, one, enabled, columns);

  // A masked thruster is still rejected when its axis is not a unit vector.
  enabled[0] = false;
  ExpectInvalid("masked non-unit", kNotUnitMessage, one, enabled, columns);

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].direction_body = Vec3{0, 0, 0};
  ASSERT_FALSE(ValidateMarineModel(model).ok());
  const std::array<ThrusterGeometry, 1> zero_axis = {model.thrusters[0]};
  enabled[0] = true;
  ExpectInvalid("zero axis", kZeroAxisMessage, zero_axis, enabled, columns);

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].direction_body = Vec3{1e-12, 0, 0};
  const std::array<ThrusterGeometry, 1> short_axis = {model.thrusters[0]};
  ExpectInvalid("short axis", kZeroAxisMessage, short_axis, enabled, columns);
}

TEST(ThrusterEffectivenessMatrix, RejectsNonBodyFrameWithoutConverting) {
  // Pose and axis stay in the supplied frame. world_enu, world_ned, an
  // empty id, and any other id are not converted into a body column.
  const std::array<std::string, 4> frames = {
      std::string(kWorldEnuFrameId),
      std::string(kWorldNedFrameId),
      "",
      "map",
  };
  for (const std::string& frame : frames) {
    ThrusterGeometry thruster = UnitThruster(Vec3{1, 2, -4}, Vec3{0, 0, 1});
    thruster.frame_id = frame;
    MarineModel model = MakeSixThrusterUuvExample();
    model.thrusters[0] = thruster;
    ASSERT_FALSE(ValidateMarineModel(model).ok()) << frame;
    const std::array<ThrusterGeometry, 1> thrusters = {thruster};
    const std::array<bool, 1> enabled = {true};
    std::array<EffectivenessColumn, 1> columns = {};
    ExpectInvalid(frame.c_str(), kFrameMessage, thrusters, enabled, columns);
    ExpectZeroColumn(columns[0]);
  }
}

TEST(ThrusterEffectivenessMatrix, FirstDefectWins) {
  const ThrusterGeometry valid = UnitThruster(Vec3{0, 0, 0}, Vec3{1, 0, 0});
  std::array<EffectivenessColumn, 2> two_columns = {};

  ExpectInvalid("empty beats a long mask", kEmptyThrustersMessage, {},
                std::array<bool, 1>{true}, two_columns);

  const std::array<ThrusterGeometry, 1> one = {valid};
  ExpectInvalid("mask length beats column count", kMaskLengthMessage, one,
                std::array<bool, 2>{true, false}, two_columns);

  ThrusterGeometry non_body = valid;
  non_body.frame_id = std::string(kWorldEnuFrameId);
  const std::array<ThrusterGeometry, 1> bad_frame = {non_body};
  ExpectInvalid("column count beats a bad frame", kColumnCountMessage,
                bad_frame, std::array<bool, 1>{true}, two_columns);

  ThrusterGeometry bad = valid;
  bad.frame_id = std::string(kWorldEnuFrameId);
  bad.position_m.x = std::numeric_limits<double>::quiet_NaN();
  bad.direction_body = Vec3{2, 0, 0};
  const std::array<ThrusterGeometry, 1> framed = {bad};
  std::array<EffectivenessColumn, 1> one_column = {};
  ExpectInvalid("frame beats position", kFrameMessage, framed,
                std::array<bool, 1>{true}, one_column);

  bad = valid;
  bad.position_m.x = std::numeric_limits<double>::quiet_NaN();
  bad.direction_body.x = std::numeric_limits<double>::quiet_NaN();
  const std::array<ThrusterGeometry, 1> position = {bad};
  ExpectInvalid("position beats direction", kPositionNonFiniteMessage, position,
                std::array<bool, 1>{true}, one_column);

  std::array<ThrusterGeometry, 2> pair = {valid, valid};
  pair[0].direction_body = Vec3{2, 0, 0};
  pair[1].frame_id = std::string(kWorldNedFrameId);
  std::array<bool, 2> enabled = {true, true};
  ExpectInvalid("earlier thruster wins", kNotUnitMessage, pair, enabled,
                two_columns);
}

}  // namespace
