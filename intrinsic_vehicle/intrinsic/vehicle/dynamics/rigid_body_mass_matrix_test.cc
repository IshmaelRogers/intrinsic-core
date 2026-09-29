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

#include "intrinsic/vehicle/dynamics/rigid_body_mass_matrix.h"

#include <array>
#include <cmath>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::ComputeRigidBodyMassMatrix;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::RigidBodyMassMatrix;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::MassInertia;
using intrinsic::vehicle::parameters::ValidateMarineModel;
using intrinsic::vehicle::parameters::Vec3;

constexpr double kAbsTolerance = 1e-12;
constexpr int kInertia = 3;

constexpr std::string_view kMassNonFiniteMessage =
    "mass_inertia.mass_kg must be finite";
constexpr std::string_view kMassNotPositiveMessage =
    "mass_inertia.mass_kg must be greater than zero";
constexpr std::string_view kInertiaNonFiniteMessage =
    "mass_inertia.inertia_about_center_of_gravity_kg_m2 must be finite";
constexpr std::string_view kInertiaAsymmetricMessage =
    "mass_inertia.inertia_about_center_of_gravity_kg_m2 differs from the "
    "transpose by more than 1e-9";
constexpr std::string_view kInertiaNotPositiveDefiniteMessage =
    "mass_inertia.inertia_about_center_of_gravity_kg_m2 is not positive "
    "definite";
constexpr std::string_view kCenterOfGravityNonFiniteMessage =
    "center_of_gravity_m must be finite";
constexpr std::string_view kMassMatrixNonFiniteMessage =
    "rigid-body mass matrix is not finite";

MassInertia DiagonalInertia(double mass, double ix, double iy, double iz) {
  MassInertia inertia;
  inertia.mass_kg = mass;
  inertia.inertia_about_center_of_gravity_kg_m2 = {
      ix, 0, 0, 0, iy, 0, 0, 0, iz,
  };
  return inertia;
}

int At(int row, int col) { return row * kSpatialDof + col; }

// Independent reference. Coupling blocks come from S(r) b = r cross b.
// I_b uses the parallel-axis form I_g + m ((r·r) I - r r^T), which is
// I_g - m S(r) S(r).
std::array<double, kSpatialDof * kSpatialDof> ReferenceMassMatrix(
    double mass, const std::array<double, 9>& inertia, Vec3 r) {
  std::array<double, kSpatialDof * kSpatialDof> matrix = {};
  const double components[kInertia] = {r.x, r.y, r.z};
  // m S(r_g), row-major.
  const double coupling[kInertia][kInertia] = {
      {0.0, -mass * r.z, mass * r.y},
      {mass * r.z, 0.0, -mass * r.x},
      {-mass * r.y, mass * r.x, 0.0},
  };
  const double radius_sq = r.x * r.x + r.y * r.y + r.z * r.z;
  for (int i = 0; i < kInertia; ++i) {
    matrix[At(i, i)] = mass;
    for (int j = 0; j < kInertia; ++j) {
      matrix[At(kInertia + i, j)] = coupling[i][j];
      matrix[At(i, kInertia + j)] = -coupling[i][j];
      const double parallel =
          (i == j ? radius_sq : 0.0) - components[i] * components[j];
      matrix[At(kInertia + i, kInertia + j)] =
          inertia[i * kInertia + j] + mass * parallel;
    }
  }
  return matrix;
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
                   const MassInertia& mass, Vec3 center) {
  const StatusOr<RigidBodyMassMatrix> result =
      ComputeRigidBodyMassMatrix(mass, center);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value)) << name;
    EXPECT_EQ(value, 0.0) << name;
  }
}

TEST(RigidBodyMassMatrix, DiagonalCenterIsDiagonalMatrix) {
  constexpr double kMass = 12.5;
  constexpr double kIx = 0.4;
  constexpr double kIy = 1.25;
  constexpr double kIz = 3.0;
  const MassInertia mass = DiagonalInertia(kMass, kIx, kIy, kIz);
  const StatusOr<RigidBodyMassMatrix> result =
      ComputeRigidBodyMassMatrix(mass, Vec3{0, 0, 0});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());

  const std::array<double, 9> inertia =
      mass.inertia_about_center_of_gravity_kg_m2;
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceMassMatrix(kMass, inertia, Vec3{0, 0, 0}));

  std::array<double, kSpatialDof * kSpatialDof> diagonal = {};
  const double entries[] = {kMass, kMass, kMass, kIx, kIy, kIz};
  for (int i = 0; i < kSpatialDof; ++i) {
    diagonal[At(i, i)] = entries[i];
  }
  ExpectNearMatrix(result.value().coefficients, diagonal);
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(RigidBodyMassMatrix, CoupledCenterMatchesFossenBlocks) {
  constexpr double kMass = 10.0;
  const MassInertia mass = DiagonalInertia(kMass, 1.5, 2.5, 3.5);
  const Vec3 center = {0.1, -0.05, 0.2};
  const StatusOr<RigidBodyMassMatrix> result =
      ComputeRigidBodyMassMatrix(mass, center);
  ASSERT_TRUE(result.ok());

  const std::array<double, 9> inertia =
      mass.inertia_about_center_of_gravity_kg_m2;
  const std::array<double, kSpatialDof * kSpatialDof> expected =
      ReferenceMassMatrix(kMass, inertia, center);
  ExpectNearMatrix(result.value().coefficients, expected);

  // Upper-right block is -m S(r_g). Spot-check signs from a cross b.
  EXPECT_NEAR(result.value().coefficients[At(0, 4)], kMass * center.z,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(0, 5)], -kMass * center.y,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(1, 3)], -kMass * center.z,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(1, 5)], kMass * center.x,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(2, 3)], kMass * center.y,
              kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(2, 4)], -kMass * center.x,
              kAbsTolerance);
  // Lower-left block m S is the elementwise negation of upper-right -m S.
  EXPECT_NEAR(result.value().coefficients[At(4, 0)],
              -result.value().coefficients[At(1, 3)], kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(5, 1)],
              -result.value().coefficients[At(2, 4)], kAbsTolerance);
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(RigidBodyMassMatrix, ExampleParametersAreSymmetricAndPositiveDefinite) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_TRUE(ValidateMarineModel(model).ok());
  const StatusOr<RigidBodyMassMatrix> result = ComputeRigidBodyMassMatrix(
      model.mass_inertia, model.centers.center_of_gravity_m);
  ASSERT_TRUE(result.ok());
  const std::array<double, 9> inertia =
      model.mass_inertia.inertia_about_center_of_gravity_kg_m2;
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceMassMatrix(model.mass_inertia.mass_kg, inertia,
                                       model.centers.center_of_gravity_m));
  ExpectSymmetric(result.value().coefficients);
  EXPECT_TRUE(CholeskySucceeds(result.value().coefficients));
}

TEST(RigidBodyMassMatrix, InvalidInputsReturnStatusAndZeros) {
  const Vec3 center = {0.1, -0.05, 0.2};
  const MassInertia valid = DiagonalInertia(10.0, 1.5, 2.5, 3.5);

  MassInertia zero_mass = valid;
  zero_mass.mass_kg = 0.0;
  ExpectInvalid("zero mass", kMassNotPositiveMessage, zero_mass, center);

  MassInertia negative_mass = valid;
  negative_mass.mass_kg = -4.0;
  ExpectInvalid("negative mass", kMassNotPositiveMessage, negative_mass,
                center);

  MassInertia nan_mass = valid;
  nan_mass.mass_kg = std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan mass", kMassNonFiniteMessage, nan_mass, center);

  MassInertia inf_mass = valid;
  inf_mass.mass_kg = std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite mass", kMassNonFiniteMessage, inf_mass, center);

  MassInertia nan_inertia = valid;
  nan_inertia.inertia_about_center_of_gravity_kg_m2[8] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan inertia", kInertiaNonFiniteMessage, nan_inertia, center);

  MassInertia inf_inertia = valid;
  inf_inertia.inertia_about_center_of_gravity_kg_m2[0] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite inertia", kInertiaNonFiniteMessage, inf_inertia,
                center);

  MassInertia asymmetric = valid;
  asymmetric.inertia_about_center_of_gravity_kg_m2[1] = 0.5;
  asymmetric.inertia_about_center_of_gravity_kg_m2[3] = 0.0;
  ExpectInvalid("asymmetric inertia", kInertiaAsymmetricMessage, asymmetric,
                center);

  MassInertia just_over = valid;
  just_over.inertia_about_center_of_gravity_kg_m2[1] =
      kMatrixSymmetryTolerance + 1e-12;
  ExpectInvalid("asymmetry beyond 1e-9", kInertiaAsymmetricMessage, just_over,
                center);

  MassInertia singular = valid;
  singular.inertia_about_center_of_gravity_kg_m2 = {
      1, 0, 0, 0, 1, 0, 0, 0, 0,
  };
  ExpectInvalid("singular inertia", kInertiaNotPositiveDefiniteMessage,
                singular, center);

  MassInertia indefinite = valid;
  indefinite.inertia_about_center_of_gravity_kg_m2 = {
      1, 2, 0, 2, 1, 0, 0, 0, 1,
  };
  ExpectInvalid("indefinite inertia", kInertiaNotPositiveDefiniteMessage,
                indefinite, center);

  // Mass is reported before inertia, matching ValidateMarineModel order.
  MassInertia both = singular;
  both.mass_kg = -1.0;
  ExpectInvalid("mass before inertia", kMassNotPositiveMessage, both, center);

  ExpectInvalid("nan center", kCenterOfGravityNonFiniteMessage, valid,
                Vec3{0.1, std::numeric_limits<double>::quiet_NaN(), 0.2});
  ExpectInvalid("infinite center", kCenterOfGravityNonFiniteMessage, valid,
                Vec3{std::numeric_limits<double>::infinity(), 0.0, 0.0});

  // Finite inputs whose products overflow. The result must not be Inf.
  MassInertia huge = DiagonalInertia(1e300, 1.0, 1.0, 1.0);
  ExpectInvalid("overflow", kMassMatrixNonFiniteMessage, huge,
                Vec3{1e10, 0.0, 0.0});
}

TEST(RigidBodyMassMatrix, SymmetryToleranceMatchesMarineModel) {
  MassInertia inertia = DiagonalInertia(4.0, 2.0, 3.0, 4.0);
  inertia.inertia_about_center_of_gravity_kg_m2[1] = kMatrixSymmetryTolerance;
  inertia.inertia_about_center_of_gravity_kg_m2[3] = 0.0;
  const StatusOr<RigidBodyMassMatrix> result =
      ComputeRigidBodyMassMatrix(inertia, Vec3{0, 0, 0});
  ASSERT_TRUE(result.ok());
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

TEST(RigidBodyMassMatrix, RepeatedEvaluationIsBitIdentical) {
  const MassInertia mass = DiagonalInertia(10.0, 1.5, 2.5, 3.5);
  const Vec3 center = {0.1, -0.05, 0.2};
  const StatusOr<RigidBodyMassMatrix> first =
      ComputeRigidBodyMassMatrix(mass, center);
  const StatusOr<RigidBodyMassMatrix> second =
      ComputeRigidBodyMassMatrix(mass, center);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  EXPECT_EQ(first.value().coefficients, second.value().coefficients);

  MassInertia invalid = mass;
  invalid.mass_kg = 0.0;
  const StatusOr<RigidBodyMassMatrix> bad_first =
      ComputeRigidBodyMassMatrix(invalid, center);
  const StatusOr<RigidBodyMassMatrix> bad_second =
      ComputeRigidBodyMassMatrix(invalid, center);
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.value().coefficients, bad_second.value().coefficients);
  EXPECT_EQ(bad_first.status().message, kMassNotPositiveMessage);
}

}  // namespace
