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

#include "intrinsic/vehicle/dynamics/added_mass_matrix.h"

#include <array>
#include <cmath>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::AddedMassMatrix;
using intrinsic::vehicle::dynamics::ComputeAddedMassMatrix;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::kHeave;
using intrinsic::vehicle::dynamics::kPitch;
using intrinsic::vehicle::dynamics::kRoll;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kSurge;
using intrinsic::vehicle::dynamics::kSway;
using intrinsic::vehicle::dynamics::kYaw;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::parameters::AddedMass;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::ModelErrorCode;
using intrinsic::vehicle::parameters::ValidateMarineModel;

constexpr double kAbsTolerance = 1e-12;

constexpr std::string_view kAddedMassNonFiniteMessage =
    "added_mass.coefficients must be finite";
constexpr std::string_view kAddedMassAsymmetricMessage =
    "added_mass.coefficients differs from the transpose by more than 1e-9";
constexpr std::string_view kAddedMassNotPositiveDefiniteMessage =
    "added_mass.coefficients is not positive definite";

// Hand-chosen symmetric, diagonally dominant M_A. One row per body axis:
// surge, sway, heave, roll, pitch, yaw. Negative couplings stay negative.
constexpr std::array<double, kSpatialDof * kSpatialDof> kReferenceAddedMass = {
    10.0, 0.4,   0.0,  0.0,  0.2, 0.0,    // surge
    0.4,  20.0,  0.0,  0.0,  0.0, -0.15,  // sway
    0.0,  0.0,   30.0, -0.3, 0.0, 0.0,    // heave
    0.0,  0.0,   -0.3, 1.5,  0.0, 0.05,   // roll
    0.2,  0.0,   0.0,  0.0,  2.5, 0.0,    // pitch
    0.0,  -0.15, 0.0,  0.05, 0.0, 3.5,    // yaw
};

int At(int row, int col) { return row * kSpatialDof + col; }

AddedMass FromCoefficients(
    const std::array<double, kSpatialDof * kSpatialDof>& coefficients) {
  AddedMass added;
  added.coefficients = coefficients;
  return added;
}

AddedMass DiagonalAddedMass(double surge, double sway, double heave,
                            double roll, double pitch, double yaw) {
  AddedMass added;
  const double diagonal[] = {surge, sway, heave, roll, pitch, yaw};
  for (int i = 0; i < kSpatialDof; ++i) {
    added.coefficients[At(i, i)] = diagonal[i];
  }
  return added;
}

void ExpectNearMatrix(
    const std::array<double, kSpatialDof * kSpatialDof>& actual,
    const std::array<double, kSpatialDof * kSpatialDof>& expected) {
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      EXPECT_NEAR(actual[At(row, col)], expected[At(row, col)], kAbsTolerance)
          << "row " << row << " col " << col;
    }
  }
}

void ExpectSymmetric(
    const std::array<double, kSpatialDof * kSpatialDof>& matrix) {
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      EXPECT_NEAR(matrix[At(row, col)], matrix[At(col, row)], kAbsTolerance)
          << "row " << row << " col " << col;
    }
  }
}

// Unpivoted Cholesky. Success means the matrix is positive definite.
bool CholeskySucceeds(
    const std::array<double, kSpatialDof * kSpatialDof>& matrix) {
  double factor[kSpatialDof * kSpatialDof] = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = matrix[At(i, j)];
      for (int k = 0; k < j; ++k) {
        sum -= factor[At(i, k)] * factor[At(j, k)];
      }
      if (i == j) {
        if (!(sum > 0.0) || !std::isfinite(sum)) {
          return false;
        }
        factor[At(i, i)] = std::sqrt(sum);
      } else {
        factor[At(i, j)] = sum / factor[At(j, j)];
      }
    }
  }
  return true;
}

void ExpectInvalid(const char* name, std::string_view message,
                   const AddedMass& added) {
  const StatusOr<AddedMassMatrix> result = ComputeAddedMassMatrix(added);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value)) << name;
    EXPECT_EQ(value, 0.0) << name;
  }
}

TEST(AddedMassMatrix, SymmetricDiagonalReferenceMatchesHandValues) {
  // Six-thruster example added mass, calm water, body axes.
  constexpr double kSurgeMass = 16.0;
  constexpr double kSwayMass = 40.0;
  constexpr double kHeaveMass = 48.0;
  constexpr double kRollMass = 0.25;
  constexpr double kPitchMass = 2.0;
  constexpr double kYawMass = 2.2;
  const AddedMass added = DiagonalAddedMass(kSurgeMass, kSwayMass, kHeaveMass,
                                            kRollMass, kPitchMass, kYawMass);
  const StatusOr<AddedMassMatrix> result = ComputeAddedMassMatrix(added);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());

  std::array<double, kSpatialDof * kSpatialDof> expected = {};
  const double diagonal[] = {kSurgeMass, kSwayMass,  kHeaveMass,
                             kRollMass,  kPitchMass, kYawMass};
  for (int i = 0; i < kSpatialDof; ++i) {
    expected[At(i, i)] = diagonal[i];
  }
  ExpectNearMatrix(result.value().coefficients, expected);
  EXPECT_EQ(result.value().coefficients, added.coefficients);
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(AddedMassMatrix, FullyPopulatedSymmetricReferenceMatchesBodyOrder) {
  const AddedMass added = FromCoefficients(kReferenceAddedMass);
  const StatusOr<AddedMassMatrix> result = ComputeAddedMassMatrix(added);
  ASSERT_TRUE(result.ok());
  ExpectNearMatrix(result.value().coefficients, kReferenceAddedMass);
  EXPECT_EQ(result.value().coefficients, added.coefficients);

  EXPECT_NEAR(result.value().coefficients[At(kSurge, kSurge)], 10.0,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(kSway, kYaw)], -0.15,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(kHeave, kRoll)], -0.3,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(kRoll, kYaw)], 0.05,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(kPitch, kSurge)], 0.2,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(kYaw, kYaw)], 3.5, kAbsTolerance);
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(AddedMassMatrix, AxisIsolatedDiagonalEntryAppearsOnlyOnThatIndex) {
  // Other diagonals stay positive so the matrix is positive definite.
  // The distinctive entry is the only nonzero coefficient of that axis.
  constexpr double kBaseline = 1.0;
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    const double distinctive = 8.0 + static_cast<double>(axis);
    AddedMass added = DiagonalAddedMass(kBaseline, kBaseline, kBaseline,
                                        kBaseline, kBaseline, kBaseline);
    added.coefficients[At(axis, axis)] = distinctive;
    const StatusOr<AddedMassMatrix> result = ComputeAddedMassMatrix(added);
    ASSERT_TRUE(result.ok()) << axis;
    EXPECT_EQ(result.value().coefficients, added.coefficients) << axis;
    int matches = 0;
    for (int row = 0; row < kSpatialDof; ++row) {
      for (int col = 0; col < kSpatialDof; ++col) {
        const double value = result.value().coefficients[At(row, col)];
        if (row != col) {
          EXPECT_EQ(value, 0.0) << axis << " row " << row << " col " << col;
        }
        if (value == distinctive) {
          ++matches;
          EXPECT_EQ(row, axis) << axis;
          EXPECT_EQ(col, axis) << axis;
        }
      }
    }
    EXPECT_EQ(matches, 1) << axis;
    ExpectSymmetric(result.value().coefficients);
    EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
  }
}

TEST(AddedMassMatrix, ExampleParametersAreSymmetricAndPositiveDefinite) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_TRUE(ValidateMarineModel(model).ok());
  const StatusOr<AddedMassMatrix> result =
      ComputeAddedMassMatrix(model.added_mass);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().coefficients, model.added_mass.coefficients);
  ExpectNearMatrix(
      result.value().coefficients,
      DiagonalAddedMass(16.0, 40.0, 48.0, 0.25, 2.0, 2.2).coefficients);
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(AddedMassMatrix, AsymmetricInputReturnsInvalid) {
  MarineModel model = MakeSixThrusterUuvExample();
  model.added_mass.coefficients[At(kSurge, kSway)] = 1.0;
  model.added_mass.coefficients[At(kSway, kSurge)] = 0.0;
  const auto validation = ValidateMarineModel(model);
  EXPECT_FALSE(validation.ok());
  bool found = false;
  for (const auto& error : validation.errors) {
    if (error.field == "added_mass.coefficients[0,1]" &&
        error.code == ModelErrorCode::kAsymmetric) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
  ExpectInvalid("asymmetric added mass", kAddedMassAsymmetricMessage,
                model.added_mass);

  AddedMass just_over = DiagonalAddedMass(4.0, 5.0, 6.0, 1.0, 2.0, 3.0);
  just_over.coefficients[At(0, 1)] = kMatrixSymmetryTolerance + 1e-12;
  ExpectInvalid("asymmetry beyond 1e-9", kAddedMassAsymmetricMessage,
                just_over);

  // Symmetry is reported before positive definiteness.
  AddedMass both = DiagonalAddedMass(1.0, 1.0, 1.0, 1.0, 1.0, -1.0);
  both.coefficients[At(0, 1)] = 1.0;
  ExpectInvalid("asymmetry before definiteness", kAddedMassAsymmetricMessage,
                both);
}

TEST(AddedMassMatrix, InvalidInputsReturnStatusAndZeros) {
  const AddedMass valid = DiagonalAddedMass(4.0, 5.0, 6.0, 1.0, 2.0, 3.0);

  AddedMass nan_entry = valid;
  nan_entry.coefficients[At(0, 4)] = std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan entry", kAddedMassNonFiniteMessage, nan_entry);

  AddedMass inf_entry = valid;
  inf_entry.coefficients[At(kYaw, kYaw)] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite entry", kAddedMassNonFiniteMessage, inf_entry);

  AddedMass negative_inf = valid;
  negative_inf.coefficients[At(1, 2)] =
      -std::numeric_limits<double>::infinity();
  ExpectInvalid("negative infinity", kAddedMassNonFiniteMessage, negative_inf);

  // A non-finite entry wins over asymmetry.
  AddedMass both = valid;
  both.coefficients[At(0, 1)] = std::numeric_limits<double>::quiet_NaN();
  both.coefficients[At(1, 0)] = 4.0;
  ExpectInvalid("non-finite before asymmetry", kAddedMassNonFiniteMessage,
                both);

  AddedMass singular = DiagonalAddedMass(1.0, 1.0, 1.0, 1.0, 1.0, 0.0);
  ExpectInvalid("singular diagonal", kAddedMassNotPositiveDefiniteMessage,
                singular);

  AddedMass indefinite = DiagonalAddedMass(1.0, 1.0, 1.0, 1.0, 1.0, -1.0);
  ExpectInvalid("indefinite diagonal", kAddedMassNotPositiveDefiniteMessage,
                indefinite);

  // One nonzero diagonal entry is not positive definite, so it is not copied.
  AddedMass axis_only;
  axis_only.coefficients[At(kSway, kSway)] = 12.0;
  ExpectInvalid("single nonzero diagonal", kAddedMassNotPositiveDefiniteMessage,
                axis_only);

  ExpectInvalid("zero matrix", kAddedMassNotPositiveDefiniteMessage,
                AddedMass{});
}

TEST(AddedMassMatrix, SymmetryToleranceMatchesMarineModel) {
  AddedMass inside = DiagonalAddedMass(4.0, 5.0, 6.0, 1.0, 2.0, 3.0);
  inside.coefficients[At(0, 1)] = kMatrixSymmetryTolerance;
  inside.coefficients[At(1, 0)] = 0.0;
  const StatusOr<AddedMassMatrix> result = ComputeAddedMassMatrix(inside);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().coefficients, inside.coefficients);
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }

  MarineModel model = MakeSixThrusterUuvExample();
  model.added_mass = inside;
  EXPECT_TRUE(ValidateMarineModel(model).ok());

  AddedMass outside = inside;
  outside.coefficients[At(0, 1)] = 2e-9;
  const StatusOr<AddedMassMatrix> rejected = ComputeAddedMassMatrix(outside);
  ASSERT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.status().message, kAddedMassAsymmetricMessage);
  model.added_mass = outside;
  EXPECT_FALSE(ValidateMarineModel(model).ok());
}

TEST(AddedMassMatrix, RepeatedEvaluationIsBitIdentical) {
  const AddedMass added = FromCoefficients(kReferenceAddedMass);
  const StatusOr<AddedMassMatrix> first = ComputeAddedMassMatrix(added);
  const StatusOr<AddedMassMatrix> second = ComputeAddedMassMatrix(added);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  EXPECT_EQ(first.value().coefficients, second.value().coefficients);

  AddedMass invalid = added;
  invalid.coefficients[At(2, 2)] = std::numeric_limits<double>::quiet_NaN();
  const StatusOr<AddedMassMatrix> bad_first = ComputeAddedMassMatrix(invalid);
  const StatusOr<AddedMassMatrix> bad_second = ComputeAddedMassMatrix(invalid);
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.value().coefficients, bad_second.value().coefficients);
  EXPECT_EQ(bad_first.status().message, kAddedMassNonFiniteMessage);
  EXPECT_EQ(bad_first.status().code, DynamicsErrorCode::kInvalidArgument);
}

}  // namespace
