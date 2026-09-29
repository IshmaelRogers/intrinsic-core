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

#include "intrinsic/vehicle/parameters/marine_model.h"

#include <cmath>
#include <limits>
#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::parameters::FieldError;
using intrinsic::vehicle::parameters::kBodyFrameId;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::kSixThrusterUuvModelId;
using intrinsic::vehicle::parameters::kSixThrusterUuvThrusterCount;
using intrinsic::vehicle::parameters::kSpatialDof;
using intrinsic::vehicle::parameters::kUnitVectorTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::ModelErrorCode;
using intrinsic::vehicle::parameters::ThrusterHealthState;
using intrinsic::vehicle::parameters::ValidateMarineModel;
using intrinsic::vehicle::parameters::ValidationResult;
using intrinsic::vehicle::parameters::Vec3;

bool HasError(const ValidationResult& result, std::string_view field,
              ModelErrorCode code) {
  for (const FieldError& error : result.errors) {
    if (error.field == field && error.code == code) {
      return true;
    }
  }
  return false;
}

int CountCode(const ValidationResult& result, std::string_view field,
              ModelErrorCode code) {
  int count = 0;
  for (const FieldError& error : result.errors) {
    if (error.field == field && error.code == code) {
      ++count;
    }
  }
  return count;
}

Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

int Rank6(double matrix[6][6]) {
  constexpr double kTol = 1e-8;
  int rank = 0;
  int pivot_row = 0;
  for (int col = 0; col < 6 && pivot_row < 6; ++col) {
    int best = pivot_row;
    for (int row = pivot_row + 1; row < 6; ++row) {
      if (std::abs(matrix[row][col]) > std::abs(matrix[best][col])) {
        best = row;
      }
    }
    if (std::abs(matrix[best][col]) <= kTol) {
      continue;
    }
    if (best != pivot_row) {
      for (int k = col; k < 6; ++k) {
        const double swap = matrix[pivot_row][k];
        matrix[pivot_row][k] = matrix[best][k];
        matrix[best][k] = swap;
      }
    }
    const double pivot = matrix[pivot_row][col];
    for (int row = pivot_row + 1; row < 6; ++row) {
      const double factor = matrix[row][col] / pivot;
      for (int k = col; k < 6; ++k) {
        matrix[row][k] -= factor * matrix[pivot_row][k];
      }
    }
    ++pivot_row;
    ++rank;
  }
  return rank;
}

TEST(MarineModelValidation, SixThrusterExampleIsValid) {
  const MarineModel model = MakeSixThrusterUuvExample();
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.errors.empty());
  EXPECT_EQ(model.model_id, kSixThrusterUuvModelId);
  EXPECT_EQ(static_cast<int>(model.thrusters.size()),
            kSixThrusterUuvThrusterCount);
  const char* names[] = {"surge_port", "surge_starboard", "sway_fore",
                         "sway_aft",   "heave_fore",      "heave_aft"};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_EQ(model.thrusters[i].name, names[i]);
  }
  EXPECT_DOUBLE_EQ(model.mass_inertia.mass_kg, 32.0);
  EXPECT_DOUBLE_EQ(model.environment.gravity_m_s2, 9.80665);
  EXPECT_DOUBLE_EQ(model.environment.fluid_density_kg_m3, 1025.0);
  EXPECT_DOUBLE_EQ(model.buoyancy.displaced_volume_m3, (32.0 * 1.005) / 1025.0);
  EXPECT_EQ(model.environment.current_frame_id, "world_enu");
  EXPECT_DOUBLE_EQ(model.damping.quadratic_coefficients[3], 0.0);
  const double forward_slew[] = {250.0, 250.0, 150.0, 150.0, 200.0, 200.0};
  const double reverse_slew[] = {175.0, 175.0, 150.0, 150.0, 125.0, 125.0};
  for (int i = 0; i < kSixThrusterUuvThrusterCount; ++i) {
    EXPECT_EQ(model.thrusters[i].frame_id, kBodyFrameId);
    EXPECT_DOUBLE_EQ(model.thrusters[i].max_forward_slew_n_per_s,
                     forward_slew[i]);
    EXPECT_DOUBLE_EQ(model.thrusters[i].max_reverse_slew_n_per_s,
                     reverse_slew[i]);
    EXPECT_DOUBLE_EQ(model.thrusters[i].efficiency, 1.0);
    EXPECT_EQ(model.thrusters[i].health, ThrusterHealthState::kNominal);
    EXPECT_DOUBLE_EQ(model.thrusters[i].health_derate, 1.0);
  }
}

TEST(MarineModelValidation, ExampleThrustersSpanSixWrenchAxes) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_EQ(static_cast<int>(model.thrusters.size()), 6);
  double matrix[6][6] = {};
  for (int col = 0; col < 6; ++col) {
    const Vec3 force = model.thrusters[col].direction_body;
    const Vec3 moment = Cross(model.thrusters[col].position_m, force);
    matrix[0][col] = force.x;
    matrix[1][col] = force.y;
    matrix[2][col] = force.z;
    matrix[3][col] = moment.x;
    matrix[4][col] = moment.y;
    matrix[5][col] = moment.z;
  }
  EXPECT_EQ(Rank6(matrix), 6);
}

TEST(MarineModelValidation, RepeatedValidationMatches) {
  const MarineModel model = MakeSixThrusterUuvExample();
  const ValidationResult first = ValidateMarineModel(model);
  const ValidationResult second = ValidateMarineModel(model);
  ASSERT_EQ(first.errors.size(), second.errors.size());
  for (int i = 0; i < static_cast<int>(first.errors.size()); ++i) {
    EXPECT_EQ(first.errors[i].field, second.errors[i].field);
    EXPECT_EQ(first.errors[i].code, second.errors[i].code);
  }
}

TEST(MarineModelValidation, EmptyModelReturnsFieldErrors) {
  const ValidationResult result = ValidateMarineModel(MarineModel{});
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(HasError(result, "model_id", ModelErrorCode::kEmpty));
  EXPECT_TRUE(
      HasError(result, "mass_inertia.mass_kg", ModelErrorCode::kNotPositive));
  EXPECT_TRUE(HasError(result, "buoyancy.displaced_volume_m3",
                       ModelErrorCode::kNotPositive));
  EXPECT_TRUE(HasError(result, "environment.gravity_m_s2",
                       ModelErrorCode::kNotPositive));
  EXPECT_TRUE(HasError(result, "environment.fluid_density_kg_m3",
                       ModelErrorCode::kNotPositive));
  EXPECT_TRUE(
      HasError(result, "environment.current_frame_id", ModelErrorCode::kEmpty));
  EXPECT_TRUE(HasError(result, "added_mass.coefficients",
                       ModelErrorCode::kNotPositiveDefinite));
  EXPECT_TRUE(HasError(result,
                       "mass_inertia.inertia_about_center_of_gravity_kg_m2",
                       ModelErrorCode::kNotPositiveDefinite));
  EXPECT_TRUE(HasError(result, "damping.linear_coefficients",
                       ModelErrorCode::kNotPositiveDefinite));
  EXPECT_FALSE(HasError(result, "centers.center_of_gravity_m.x",
                        ModelErrorCode::kNonFinite));
  EXPECT_FALSE(HasError(result, "damping.quadratic_coefficients[0]",
                        ModelErrorCode::kNegative));
  for (const FieldError& error : result.errors) {
    EXPECT_FALSE(error.field.empty());
    EXPECT_FALSE(error.message.empty());
  }
}

TEST(MarineModelValidation, AsymmetricInertiaNamesTheEntry) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2[1] = 0.5;
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2[3] = 0.0;
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(HasError(
      result, "mass_inertia.inertia_about_center_of_gravity_kg_m2[0,1]",
      ModelErrorCode::kAsymmetric));
  EXPECT_FALSE(HasError(result,
                        "mass_inertia.inertia_about_center_of_gravity_kg_m2",
                        ModelErrorCode::kNotPositiveDefinite));
  EXPECT_EQ(result.errors.size(), 1u);
}

TEST(MarineModelValidation, SymmetryToleranceBoundary) {
  MarineModel inside = MakeSixThrusterUuvExample();
  inside.added_mass.coefficients[1] = kMatrixSymmetryTolerance;
  inside.added_mass.coefficients[kSpatialDof] = 0.0;
  EXPECT_TRUE(ValidateMarineModel(inside).ok());

  MarineModel outside = MakeSixThrusterUuvExample();
  outside.added_mass.coefficients[1] = 2e-9;
  outside.added_mass.coefficients[kSpatialDof] = 0.0;
  const ValidationResult result = ValidateMarineModel(outside);
  EXPECT_TRUE(HasError(result, "added_mass.coefficients[0,1]",
                       ModelErrorCode::kAsymmetric));
}

TEST(MarineModelValidation, ReportsEveryAsymmetricPair) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.added_mass.coefficients[1] = 1.0;
  model.added_mass.coefficients[kSpatialDof] = 0.0;
  model.added_mass.coefficients[2 * kSpatialDof + 3] = 1.0;
  model.added_mass.coefficients[3 * kSpatialDof + 2] = 0.0;
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "added_mass.coefficients[0,1]",
                       ModelErrorCode::kAsymmetric));
  EXPECT_TRUE(HasError(result, "added_mass.coefficients[2,3]",
                       ModelErrorCode::kAsymmetric));
  EXPECT_FALSE(HasError(result, "added_mass.coefficients",
                        ModelErrorCode::kNotPositiveDefinite));
}

TEST(MarineModelValidation, AddedMassMustBePositiveDefinite) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.added_mass.coefficients = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    model.added_mass.coefficients[i * kSpatialDof + i] = 1.0;
  }
  model.added_mass.coefficients[kSpatialDof * kSpatialDof - 1] = -1.0;
  const ValidationResult result = ValidateMarineModel(model);
  ASSERT_EQ(CountCode(result, "added_mass.coefficients",
                      ModelErrorCode::kNotPositiveDefinite),
            1);
  EXPECT_NE(result.errors[0].message.find("pivot 5"), std::string::npos);
}

TEST(MarineModelValidation, IndefiniteInertiaNamesTheMatrix) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2 = {
      1, 2, 0, 2, 1, 0, 0, 0, 1,
  };
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result,
                       "mass_inertia.inertia_about_center_of_gravity_kg_m2",
                       ModelErrorCode::kNotPositiveDefinite));
  EXPECT_EQ(result.errors.size(), 1u);
}

TEST(MarineModelValidation, LinearDampingRejectsZeroEigenvalue) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.damping.linear_coefficients[0] = 0.0;
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "damping.linear_coefficients",
                       ModelErrorCode::kNotPositiveDefinite));
  EXPECT_NE(result.errors[0].message.find("pivot 0"), std::string::npos);
}

TEST(MarineModelValidation, QuadraticDampingRejectsNegative) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.damping.quadratic_coefficients[3] = -0.2;
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "damping.quadratic_coefficients[3]",
                       ModelErrorCode::kNegative));
  EXPECT_EQ(result.errors.size(), 1u);
}

TEST(MarineModelValidation, NonFiniteValuesNameTheComponent) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.mass_inertia.mass_kg = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult mass = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(mass, "mass_inertia.mass_kg", ModelErrorCode::kNonFinite));
  EXPECT_FALSE(
      HasError(mass, "mass_inertia.mass_kg", ModelErrorCode::kNotPositive));
  EXPECT_EQ(mass.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.centers.center_of_buoyancy_m.y =
      std::numeric_limits<double>::infinity();
  const ValidationResult center = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(center, "centers.center_of_buoyancy_m.y",
                       ModelErrorCode::kNonFinite));
  EXPECT_EQ(center.errors.size(), 1u);
}

TEST(MarineModelValidation, CurrentFrameMustBeExplicit) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.environment.current_frame_id = "robot";
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(result, "environment.current_frame_id", ModelErrorCode::kFrame));
  EXPECT_EQ(result.errors.size(), 1u);

  model.environment.current_frame_id = "body";
  model.environment.current_velocity_m_s = {0.1, -0.2, 0.0};
  EXPECT_TRUE(ValidateMarineModel(model).ok());
  model.environment.current_frame_id = "world_ned";
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

TEST(MarineModelValidation, ThrusterDirectionAndBounds) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[1].direction_body = {2, 0, 0};
  const ValidationResult direction = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(direction, "thrusters[1].direction_body",
                       ModelErrorCode::kNotUnit));

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].max_forward_thrust_n = 0;
  model.thrusters[0].max_reverse_thrust_n = 0;
  const ValidationResult bounds = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(bounds, "thrusters[0].thrust_bounds",
                       ModelErrorCode::kNotPositive));

  model = MakeSixThrusterUuvExample();
  model.thrusters[4].max_reverse_thrust_n = -1;
  const ValidationResult negative = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(negative, "thrusters[4].max_reverse_thrust_n",
                       ModelErrorCode::kNegative));
}

TEST(MarineModelValidation, NonFiniteMatrixEntrySkipsDefiniteness) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.added_mass.coefficients[4] = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "added_mass.coefficients[0,4]",
                       ModelErrorCode::kNonFinite));
  EXPECT_FALSE(HasError(result, "added_mass.coefficients",
                        ModelErrorCode::kNotPositiveDefinite));
  EXPECT_FALSE(HasError(result, "added_mass.coefficients[0,4]",
                        ModelErrorCode::kAsymmetric));
}

TEST(MarineModelValidation, DuplicateThrusterName) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[5].name = model.thrusters[0].name;
  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(result, "thrusters[5].name", ModelErrorCode::kDuplicate));
  EXPECT_FALSE(
      HasError(result, "thrusters[0].name", ModelErrorCode::kDuplicate));
  EXPECT_EQ(result.errors.size(), 1u);
}

TEST(MarineModelValidation, MultipleInvalidFieldsAreAllReported) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.model_id.clear();
  model.mass_inertia.mass_kg = -5;
  model.buoyancy.displaced_volume_m3 = 0;
  model.environment.gravity_m_s2 = std::numeric_limits<double>::quiet_NaN();
  model.environment.current_frame_id = "robot";
  model.damping.quadratic_coefficients[0] = -1;
  model.centers.center_of_gravity_m.z = std::numeric_limits<double>::infinity();
  model.thrusters[2].direction_body = {0, 0, 0};

  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "model_id", ModelErrorCode::kEmpty));
  EXPECT_TRUE(
      HasError(result, "mass_inertia.mass_kg", ModelErrorCode::kNotPositive));
  EXPECT_TRUE(HasError(result, "buoyancy.displaced_volume_m3",
                       ModelErrorCode::kNotPositive));
  EXPECT_TRUE(
      HasError(result, "environment.gravity_m_s2", ModelErrorCode::kNonFinite));
  EXPECT_TRUE(
      HasError(result, "environment.current_frame_id", ModelErrorCode::kFrame));
  EXPECT_TRUE(HasError(result, "damping.quadratic_coefficients[0]",
                       ModelErrorCode::kNegative));
  EXPECT_TRUE(HasError(result, "centers.center_of_gravity_m.z",
                       ModelErrorCode::kNonFinite));
  EXPECT_TRUE(HasError(result, "thrusters[2].direction_body",
                       ModelErrorCode::kZeroAxis));
  EXPECT_FALSE(HasError(result, "thrusters[2].direction_body",
                        ModelErrorCode::kNotUnit));
  EXPECT_FALSE(HasError(result, "added_mass.coefficients",
                        ModelErrorCode::kNotPositiveDefinite));
  EXPECT_GE(result.errors.size(), 8u);
}

TEST(MarineModelValidation, ZeroAxisIsDistinctFromNonUnit) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[2].direction_body = {0, 0, 0};
  const ValidationResult zero = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(zero, "thrusters[2].direction_body", ModelErrorCode::kZeroAxis));
  EXPECT_FALSE(
      HasError(zero, "thrusters[2].direction_body", ModelErrorCode::kNotUnit));
  EXPECT_EQ(zero.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[2].direction_body = {kUnitVectorTolerance, 0, 0};
  const ValidationResult near_zero = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(near_zero, "thrusters[2].direction_body",
                       ModelErrorCode::kZeroAxis));

  model = MakeSixThrusterUuvExample();
  model.thrusters[2].direction_body = {2 * kUnitVectorTolerance, 0, 0};
  const ValidationResult short_axis = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(short_axis, "thrusters[2].direction_body",
                       ModelErrorCode::kNotUnit));
  EXPECT_FALSE(HasError(short_axis, "thrusters[2].direction_body",
                        ModelErrorCode::kZeroAxis));
}

TEST(MarineModelValidation, SlewMustBePositiveAndFinite) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].max_forward_slew_n_per_s = 0;
  const ValidationResult zero = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(zero, "thrusters[0].max_forward_slew_n_per_s",
                       ModelErrorCode::kNotPositive));
  EXPECT_EQ(zero.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[1].max_reverse_slew_n_per_s = -5;
  const ValidationResult negative = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(negative, "thrusters[1].max_reverse_slew_n_per_s",
                       ModelErrorCode::kNotPositive));
  EXPECT_FALSE(HasError(negative, "thrusters[1].max_reverse_slew_n_per_s",
                        ModelErrorCode::kNegative));

  model = MakeSixThrusterUuvExample();
  model.thrusters[3].max_forward_slew_n_per_s =
      std::numeric_limits<double>::infinity();
  const ValidationResult infinite = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(infinite, "thrusters[3].max_forward_slew_n_per_s",
                       ModelErrorCode::kNonFinite));
  EXPECT_FALSE(HasError(infinite, "thrusters[3].max_forward_slew_n_per_s",
                        ModelErrorCode::kNotPositive));

  model = MakeSixThrusterUuvExample();
  model.thrusters[4].max_forward_slew_n_per_s = 1.0e12;
  model.thrusters[4].max_reverse_slew_n_per_s = 1.0e-12;
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

TEST(MarineModelValidation, EfficiencyInterval) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].efficiency = 0;
  const ValidationResult zero = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(zero, "thrusters[0].efficiency", ModelErrorCode::kNotPositive));
  EXPECT_FALSE(
      HasError(zero, "thrusters[0].efficiency", ModelErrorCode::kOutOfRange));

  model = MakeSixThrusterUuvExample();
  model.thrusters[1].efficiency = -0.2;
  const ValidationResult negative = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(negative, "thrusters[1].efficiency",
                       ModelErrorCode::kNotPositive));

  model = MakeSixThrusterUuvExample();
  model.thrusters[2].efficiency = std::nextafter(1.0, 2.0);
  const ValidationResult above = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(above, "thrusters[2].efficiency", ModelErrorCode::kOutOfRange));

  model = MakeSixThrusterUuvExample();
  model.thrusters[3].efficiency = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult nan = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(nan, "thrusters[3].efficiency", ModelErrorCode::kNonFinite));
  EXPECT_FALSE(
      HasError(nan, "thrusters[3].efficiency", ModelErrorCode::kNotPositive));

  model = MakeSixThrusterUuvExample();
  model.thrusters[5].efficiency = std::nextafter(0.0, 1.0);
  EXPECT_TRUE(ValidateMarineModel(model).ok());
  model.thrusters[5].efficiency = 1.0;
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

TEST(MarineModelValidation, ThrusterFrameMustBeBody) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].frame_id.clear();
  const ValidationResult empty = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(empty, "thrusters[0].frame_id", ModelErrorCode::kEmpty));
  EXPECT_EQ(empty.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[1].frame_id = "world_enu";
  const ValidationResult world = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(world, "thrusters[1].frame_id", ModelErrorCode::kFrame));
  EXPECT_EQ(world.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[2].frame_id = "body";
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

TEST(MarineModelValidation, HealthDerateMatchesState) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kDerated;
  model.thrusters[0].health_derate = 0.4;
  model.thrusters[1].health = ThrusterHealthState::kDisabled;
  model.thrusters[1].health_derate = 0.0;
  model.thrusters[2].health = ThrusterHealthState::kStuckOff;
  model.thrusters[2].health_derate = 0.0;
  model.thrusters[3].health = ThrusterHealthState::kFailed;
  model.thrusters[3].health_derate = 0.0;
  EXPECT_TRUE(ValidateMarineModel(model).ok());

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health_derate = 0.5;
  const ValidationResult nominal = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(nominal, "thrusters[0].health_derate",
                       ModelErrorCode::kOutOfRange));
  EXPECT_EQ(nominal.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kDerated;
  model.thrusters[0].health_derate = 0.0;
  const ValidationResult zero_derate = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(zero_derate, "thrusters[0].health_derate",
                       ModelErrorCode::kNotPositive));

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kDerated;
  model.thrusters[0].health_derate = 1.0;
  const ValidationResult full_derate = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(full_derate, "thrusters[0].health_derate",
                       ModelErrorCode::kOutOfRange));

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = ThrusterHealthState::kDisabled;
  const ValidationResult disabled = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(disabled, "thrusters[0].health_derate",
                       ModelErrorCode::kOutOfRange));

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = static_cast<ThrusterHealthState>(99);
  const ValidationResult unknown = ValidateMarineModel(model);
  EXPECT_TRUE(
      HasError(unknown, "thrusters[0].health", ModelErrorCode::kOutOfRange));
  EXPECT_FALSE(HasError(unknown, "thrusters[0].health_derate",
                        ModelErrorCode::kOutOfRange));
  EXPECT_EQ(unknown.errors.size(), 1u);

  model = MakeSixThrusterUuvExample();
  model.thrusters[0].health = static_cast<ThrusterHealthState>(99);
  model.thrusters[0].health_derate = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult unknown_nan = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(unknown_nan, "thrusters[0].health",
                       ModelErrorCode::kOutOfRange));
  EXPECT_TRUE(HasError(unknown_nan, "thrusters[0].health_derate",
                       ModelErrorCode::kNonFinite));
}

TEST(MarineModelValidation, NonFiniteThrusterFields) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[0].position_m.z = std::numeric_limits<double>::quiet_NaN();
  model.thrusters[0].direction_body.x =
      std::numeric_limits<double>::quiet_NaN();
  model.thrusters[0].max_forward_thrust_n =
      std::numeric_limits<double>::infinity();
  model.thrusters[0].max_reverse_thrust_n = -1;
  model.thrusters[0].max_forward_slew_n_per_s = 0;
  model.thrusters[0].efficiency = 2;
  model.thrusters[0].frame_id = "world_ned";

  const ValidationResult result = ValidateMarineModel(model);
  EXPECT_TRUE(HasError(result, "thrusters[0].position_m.z",
                       ModelErrorCode::kNonFinite));
  EXPECT_FALSE(HasError(result, "thrusters[0].direction_body",
                        ModelErrorCode::kZeroAxis));
  EXPECT_TRUE(HasError(result, "thrusters[0].direction_body.x",
                       ModelErrorCode::kNonFinite));
  EXPECT_TRUE(HasError(result, "thrusters[0].max_forward_thrust_n",
                       ModelErrorCode::kNonFinite));
  EXPECT_TRUE(HasError(result, "thrusters[0].max_reverse_thrust_n",
                       ModelErrorCode::kNegative));
  EXPECT_FALSE(HasError(result, "thrusters[0].thrust_bounds",
                        ModelErrorCode::kNotPositive));
  EXPECT_TRUE(HasError(result, "thrusters[0].max_forward_slew_n_per_s",
                       ModelErrorCode::kNotPositive));
  EXPECT_TRUE(
      HasError(result, "thrusters[0].efficiency", ModelErrorCode::kOutOfRange));
  EXPECT_TRUE(
      HasError(result, "thrusters[0].frame_id", ModelErrorCode::kFrame));
}

TEST(MarineModelValidation, UnidirectionalThrustRemainsValid) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.thrusters[5].max_reverse_thrust_n = 0;
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

}  // namespace
