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

#include "intrinsic/vehicle/dynamics/rigid_body_coriolis_matrix.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/rigid_body_mass_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::ComputeRigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeRigidBodyMassMatrix;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::RigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::MassInertia;
using intrinsic::vehicle::parameters::Vec3;

// Approved numeric fixtures and νᵀ C ν use this absolute tolerance.
// Hand values are exact in real arithmetic. 1e-12 covers roundoff,
// including the binary expansion of a 0.1 m center offset.
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
constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kCoriolisMatrixNonFiniteMessage =
    "rigid-body Coriolis matrix is not finite";

using Twist = std::array<double, kSpatialDof>;
using Matrix = std::array<double, kSpatialDof * kSpatialDof>;

MassInertia DiagonalInertia(double mass, double ix, double iy, double iz) {
  MassInertia inertia;
  inertia.mass_kg = mass;
  inertia.inertia_about_center_of_gravity_kg_m2 = {
      ix, 0, 0, 0, iy, 0, 0, 0, iz,
  };
  return inertia;
}

Twist MakeTwist(double u, double v, double w, double p, double q, double r) {
  return {u, v, w, p, q, r};
}

int At(int row, int col) { return row * kSpatialDof + col; }

void ExpectBitIdentical(const Matrix& actual, const Matrix& expected) {
  EXPECT_EQ(0, std::memcmp(actual.data(), expected.data(),
                           actual.size() * sizeof(double)));
}

void ExpectNearMatrix(const Matrix& actual, const Matrix& expected) {
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      EXPECT_NEAR(actual[At(row, col)], expected[At(row, col)], kAbsTolerance)
          << "row " << row << " col " << col;
    }
  }
}

void ExpectUpperLeftExactlyZero(const Matrix& matrix) {
  const double zero = 0.0;
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kInertia; ++col) {
      EXPECT_EQ(0, std::memcmp(&matrix[At(row, col)], &zero, sizeof(double)))
          << "row " << row << " col " << col;
    }
  }
}

// Independent reference. S(a) b = a cross b. I_b = I_g - m S(r) S(r).
// p1 = m ν1 - m S(r) ν2. p2 = m S(r) ν1 + I_b ν2.
// C11 = 0, C12 = C21 = -S(p1), C22 = -S(p2). Added mass is not used.
Matrix ReferenceCoriolis(double mass, const std::array<double, 9>& inertia,
                         Vec3 r, const Twist& twist) {
  const double skew[kInertia][kInertia] = {
      {0.0, -r.z, r.y},
      {r.z, 0.0, -r.x},
      {-r.y, r.x, 0.0},
  };
  double skew_sq[kInertia][kInertia] = {};
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kInertia; ++col) {
      double sum = 0.0;
      for (int k = 0; k < kInertia; ++k) {
        sum += skew[row][k] * skew[k][col];
      }
      skew_sq[row][col] = sum;
    }
  }
  double inertia_about_origin[kInertia][kInertia] = {};
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kInertia; ++col) {
      inertia_about_origin[row][col] =
          inertia[row * kInertia + col] - mass * skew_sq[row][col];
    }
  }

  const double nu1[kInertia] = {twist[0], twist[1], twist[2]};
  const double nu2[kInertia] = {twist[3], twist[4], twist[5]};
  double p1[kInertia] = {};
  double p2[kInertia] = {};
  for (int row = 0; row < kInertia; ++row) {
    double skew_nu2 = 0.0;
    double skew_nu1 = 0.0;
    double angular = 0.0;
    for (int col = 0; col < kInertia; ++col) {
      skew_nu2 += skew[row][col] * nu2[col];
      skew_nu1 += skew[row][col] * nu1[col];
      angular += inertia_about_origin[row][col] * nu2[col];
    }
    p1[row] = mass * nu1[row] - mass * skew_nu2;
    p2[row] = mass * skew_nu1 + angular;
  }

  auto negated_skew = [](const double a[kInertia],
                         double out[kInertia][kInertia]) {
    out[0][0] = 0.0;
    out[0][1] = a[2];
    out[0][2] = -a[1];
    out[1][0] = -a[2];
    out[1][1] = 0.0;
    out[1][2] = a[0];
    out[2][0] = a[1];
    out[2][1] = -a[0];
    out[2][2] = 0.0;
  };
  double neg_p1[kInertia][kInertia] = {};
  double neg_p2[kInertia][kInertia] = {};
  negated_skew(p1, neg_p1);
  negated_skew(p2, neg_p2);

  Matrix matrix = {};
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kInertia; ++col) {
      matrix[At(row, kInertia + col)] = neg_p1[row][col];
      matrix[At(kInertia + row, col)] = neg_p1[row][col];
      matrix[At(kInertia + row, kInertia + col)] = neg_p2[row][col];
    }
  }
  return matrix;
}

double Power(const Matrix& coriolis, const Twist& twist) {
  double power = 0.0;
  for (int row = 0; row < kSpatialDof; ++row) {
    double force = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      force += coriolis[At(row, col)] * twist[col];
    }
    power += twist[row] * force;
  }
  return power;
}

double CoriolisForceNorm(const Matrix& coriolis, const Twist& twist) {
  double sum = 0.0;
  for (int row = 0; row < kSpatialDof; ++row) {
    double force = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      force += coriolis[At(row, col)] * twist[col];
    }
    sum += force * force;
  }
  return std::sqrt(sum);
}

void ExpectSkewSymmetric(const Matrix& matrix) {
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      EXPECT_NEAR(matrix[At(row, col)] + matrix[At(col, row)], 0.0,
                  kAbsTolerance)
          << "row " << row << " col " << col;
    }
  }
}

void ExpectInvalid(const char* name, std::string_view message,
                   const MassInertia& mass, Vec3 center, const Twist& twist) {
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  const Matrix zeros = {};
  ExpectBitIdentical(result.value().coefficients, zeros);
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value)) << name;
  }
}

TEST(RigidBodyCoriolisMatrix, ZeroTwistIsZeroMatrix) {
  const Twist zero = {};
  const MassInertia centered = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const StatusOr<RigidBodyCoriolisMatrix> centered_result =
      ComputeRigidBodyCoriolisMatrix(centered, Vec3{0, 0, 0}, zero);
  ASSERT_TRUE(centered_result.ok());
  ExpectBitIdentical(centered_result.value().coefficients, Matrix{});

  const MassInertia offset_mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const StatusOr<RigidBodyCoriolisMatrix> offset_result =
      ComputeRigidBodyCoriolisMatrix(offset_mass, Vec3{0.1, -0.2, 0.3}, zero);
  ASSERT_TRUE(offset_result.ok());
  ExpectBitIdentical(offset_result.value().coefficients, Matrix{});

  const MarineModel model = MakeSixThrusterUuvExample();
  EXPECT_NE(model.added_mass.coefficients[0], 0.0);
  const StatusOr<RigidBodyCoriolisMatrix> example =
      ComputeRigidBodyCoriolisMatrix(model.mass_inertia,
                                     model.centers.center_of_gravity_m, zero);
  ASSERT_TRUE(example.ok());
  ExpectBitIdentical(example.value().coefficients, Matrix{});
}

TEST(RigidBodyCoriolisMatrix, SurgeVelocityMatchesApprovedFixture) {
  // r_g = 0, m = 10, I_g = diag(1, 2, 3), ν = (1, 0, 0, 0, 0, 0).
  // p1 = (10, 0, 0), p2 = (0, 0, 0).
  // -S(p1) = [[0, 0, 0], [0, 0, 10], [0, -10, 0]].
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Twist twist = MakeTwist(1, 0, 0, 0, 0, 0);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0}, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 10, 0, 0,   0, 0, -10, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0,  0, -10, 0, 0, 0,   0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, YawRateMatchesApprovedFixture) {
  // r_g = 0, m = 10, I_g = diag(1, 2, 3), ν = (0, 0, 0, 0, 0, 1).
  // p1 = (0, 0, 0), p2 = (0, 0, 3).
  // -S(p2) = [[0, 3, 0], [-3, 0, 0], [0, 0, 0]].
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Twist twist = MakeTwist(0, 0, 0, 0, 0, 1);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0}, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 3, 0, 0, 0, 0, -3, 0, 0, 0, 0, 0, 0, 0, 0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  EXPECT_NEAR(result.value().coefficients[At(0, 3)], 0.0, kAbsTolerance);
  EXPECT_NEAR(result.value().coefficients[At(3, 0)], 0.0, kAbsTolerance);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, OffsetCenterSwayMatchesApprovedBlocks) {
  // m = 10, r_g = (0.1, 0, 0), I_g = diag(1, 2, 3), ν = (0, 1, 0, 0, 0, 0).
  // p1 = (0, 10, 0), p2 = (0, 0, 1).
  // -S(p1) = [[0, 0, -10], [0, 0, 0], [10, 0, 0]].
  // -S(p2) = [[0, 1, 0], [-1, 0, 0], [0, 0, 0]].
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Vec3 center = {0.1, 0, 0};
  const Twist twist = MakeTwist(0, 1, 0, 0, 0, 0);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0,   0, 0, -10, 0, 0, 0, 0,  0, 0, 0,  0, 0, 10, 0, 0,
      0, 0, -10, 0, 1, 0,   0, 0, 0, -1, 0, 0, 10, 0, 0, 0,  0, 0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  const std::array<double, 9> inertia =
      mass.inertia_about_center_of_gravity_kg_m2;
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(10.0, inertia, center, twist));
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, OffsetCenterYawRateUsesBodyOriginInertia) {
  // m = 10, r_g = (0.1, 0, 0), I_g = diag(1, 2, 3), ν = (0, 0, 0, 0, 0, 1).
  // I_b = diag(1, 2.1, 3.1). p1 = (0, 1, 0), p2 = (0, 0, 3.1).
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Vec3 center = {0.1, 0, 0};
  const Twist twist = MakeTwist(0, 0, 0, 0, 0, 1);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0,  0, 0,   -1, 0, 0, 0, 0,    0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, -1, 0, 3.1, 0,  0, 0, 0, -3.1, 0, 0, 1, 0, 0, 0, 0, 0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, ReversedVelocityNegatesMatrix) {
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Vec3 center = {0.1, -0.05, 0.2};
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const Twist reversed = MakeTwist(-0.2, 0.4, -0.6, -0.1, 0.3, -0.5);
  const StatusOr<RigidBodyCoriolisMatrix> forward =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  const StatusOr<RigidBodyCoriolisMatrix> backward =
      ComputeRigidBodyCoriolisMatrix(mass, center, reversed);
  ASSERT_TRUE(forward.ok());
  ASSERT_TRUE(backward.ok());

  bool any_nonzero = false;
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    EXPECT_NEAR(backward.value().coefficients[i],
                -forward.value().coefficients[i], kAbsTolerance)
        << "index " << i;
    if (std::abs(forward.value().coefficients[i]) > 1.0) {
      any_nonzero = true;
    }
  }
  EXPECT_TRUE(any_nonzero);

  // Axis-isolated surge reverses sign as well.
  const StatusOr<RigidBodyCoriolisMatrix> surge =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0},
                                     MakeTwist(1, 0, 0, 0, 0, 0));
  const StatusOr<RigidBodyCoriolisMatrix> surge_back =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0},
                                     MakeTwist(-1, 0, 0, 0, 0, 0));
  ASSERT_TRUE(surge.ok());
  ASSERT_TRUE(surge_back.ok());
  EXPECT_NEAR(surge.value().coefficients[At(1, 5)], 10.0, kAbsTolerance);
  EXPECT_NEAR(surge_back.value().coefficients[At(1, 5)], -10.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, CoupledTwistCancelsPower) {
  // Surge plus yaw. C ν is nonzero, and νᵀ C ν is still zero.
  const MassInertia mass = DiagonalInertia(10.0, 1.0, 2.0, 3.0);
  const Twist twist = MakeTwist(1, 0, 0, 0, 0, 1);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0}, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0, 10, 0, 0,   0, 0, -10, 0,
      0, 0, 0, 0, 3, 0, 0, 0, 10, -3, 0, 0,  0, -10, 0, 0, 0,   0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  ExpectSkewSymmetric(result.value().coefficients);
  EXPECT_GT(CoriolisForceNorm(result.value().coefficients, twist), 1.0);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);

  const StatusOr<RigidBodyCoriolisMatrix> surge =
      ComputeRigidBodyCoriolisMatrix(mass, Vec3{0, 0, 0},
                                     MakeTwist(1, 0, 0, 0, 0, 0));
  const StatusOr<RigidBodyCoriolisMatrix> yaw = ComputeRigidBodyCoriolisMatrix(
      mass, Vec3{0, 0, 0}, MakeTwist(0, 0, 0, 0, 0, 1));
  ASSERT_TRUE(surge.ok());
  ASSERT_TRUE(yaw.ok());
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    EXPECT_NEAR(result.value().coefficients[i],
                surge.value().coefficients[i] + yaw.value().coefficients[i],
                kAbsTolerance)
        << "index " << i;
  }
}

TEST(RigidBodyCoriolisMatrix, GeneralTwistMatchesBlocksAndPower) {
  MassInertia mass = DiagonalInertia(10.0, 1.5, 2.5, 3.5);
  mass.inertia_about_center_of_gravity_kg_m2[1] = 0.05;
  mass.inertia_about_center_of_gravity_kg_m2[3] = 0.05;
  const Vec3 center = {0.1, -0.2, 0.3};
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  ASSERT_TRUE(result.ok());
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(mass.mass_kg,
                                     mass.inertia_about_center_of_gravity_kg_m2,
                                     center, twist));
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  ExpectSkewSymmetric(result.value().coefficients);
  // C ν is not the zero vector, so the power check is a cancellation.
  EXPECT_GT(CoriolisForceNorm(result.value().coefficients, twist), 1e-3);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

TEST(RigidBodyCoriolisMatrix, ExampleModelIgnoresAddedMass) {
  const MarineModel model = MakeSixThrusterUuvExample();
  EXPECT_GT(model.added_mass.coefficients[0], 1.0);
  const Twist twist = MakeTwist(1.0, 0.2, -0.3, 0.05, -0.02, 0.1);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(model.mass_inertia,
                                     model.centers.center_of_gravity_m, twist);
  ASSERT_TRUE(result.ok());

  const Matrix rigid_only = ReferenceCoriolis(
      model.mass_inertia.mass_kg,
      model.mass_inertia.inertia_about_center_of_gravity_kg_m2,
      model.centers.center_of_gravity_m, twist);
  ExpectNearMatrix(result.value().coefficients, rigid_only);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);

  const auto mass = ComputeRigidBodyMassMatrix(
      model.mass_inertia, model.centers.center_of_gravity_m);
  ASSERT_TRUE(mass.ok());
  Matrix total = mass.value().coefficients;
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    total[i] += model.added_mass.coefficients[i];
  }
  double linear[kInertia] = {};
  double angular[kInertia] = {};
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      linear[row] += total[At(row, col)] * twist[col];
      angular[row] += total[At(kInertia + row, col)] * twist[col];
    }
  }
  // -S(p1) upper-right entry (0, 5) is -p1_y. Added mass changes p1.
  const double with_added = -linear[1];
  EXPECT_GT(std::abs(with_added - result.value().coefficients[At(0, 5)]), 1.0);
}

TEST(RigidBodyCoriolisMatrix, InvalidInputsReturnStatusAndZeros) {
  const Vec3 center = {0.1, -0.05, 0.2};
  const MassInertia valid = DiagonalInertia(10.0, 1.5, 2.5, 3.5);
  const Twist twist = MakeTwist(0.2, -0.1, 0.3, 0.0, 0.1, -0.2);
  const Twist nan_twist =
      MakeTwist(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0);
  const Twist inf_twist =
      MakeTwist(0, 0, 0, 0, std::numeric_limits<double>::infinity(), 0);

  MassInertia zero_mass = valid;
  zero_mass.mass_kg = 0.0;
  ExpectInvalid("zero mass", kMassNotPositiveMessage, zero_mass, center, twist);

  MassInertia negative_mass = valid;
  negative_mass.mass_kg = -4.0;
  ExpectInvalid("negative mass", kMassNotPositiveMessage, negative_mass, center,
                twist);

  MassInertia nan_mass = valid;
  nan_mass.mass_kg = std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan mass", kMassNonFiniteMessage, nan_mass, center, twist);

  MassInertia inf_mass = valid;
  inf_mass.mass_kg = std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite mass", kMassNonFiniteMessage, inf_mass, center,
                twist);

  MassInertia nan_inertia = valid;
  nan_inertia.inertia_about_center_of_gravity_kg_m2[8] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan inertia", kInertiaNonFiniteMessage, nan_inertia, center,
                twist);

  MassInertia inf_inertia = valid;
  inf_inertia.inertia_about_center_of_gravity_kg_m2[0] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite inertia", kInertiaNonFiniteMessage, inf_inertia,
                center, twist);

  MassInertia asymmetric = valid;
  asymmetric.inertia_about_center_of_gravity_kg_m2[1] = 0.5;
  asymmetric.inertia_about_center_of_gravity_kg_m2[3] = 0.0;
  ExpectInvalid("asymmetric inertia", kInertiaAsymmetricMessage, asymmetric,
                center, twist);

  MassInertia just_over = valid;
  just_over.inertia_about_center_of_gravity_kg_m2[1] =
      kMatrixSymmetryTolerance + 1e-12;
  ExpectInvalid("asymmetry beyond 1e-9", kInertiaAsymmetricMessage, just_over,
                center, twist);

  MassInertia singular = valid;
  singular.inertia_about_center_of_gravity_kg_m2 = {
      1, 0, 0, 0, 1, 0, 0, 0, 0,
  };
  ExpectInvalid("singular inertia", kInertiaNotPositiveDefiniteMessage,
                singular, center, twist);

  MassInertia indefinite = valid;
  indefinite.inertia_about_center_of_gravity_kg_m2 = {
      1, 2, 0, 2, 1, 0, 0, 0, 1,
  };
  ExpectInvalid("indefinite inertia", kInertiaNotPositiveDefiniteMessage,
                indefinite, center, twist);

  ExpectInvalid("nan center", kCenterOfGravityNonFiniteMessage, valid,
                Vec3{0.1, std::numeric_limits<double>::quiet_NaN(), 0.2},
                twist);
  ExpectInvalid("infinite center", kCenterOfGravityNonFiniteMessage, valid,
                Vec3{std::numeric_limits<double>::infinity(), 0.0, 0.0}, twist);

  for (int index = 0; index < kSpatialDof; ++index) {
    Twist slot = twist;
    slot[index] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalid("nan twist", kBodyTwistNonFiniteMessage, valid, center, slot);
    slot[index] = std::numeric_limits<double>::infinity();
    ExpectInvalid("infinite twist", kBodyTwistNonFiniteMessage, valid, center,
                  slot);
  }
  ExpectInvalid("nan twist vector", kBodyTwistNonFiniteMessage, valid, center,
                nan_twist);
  ExpectInvalid("infinite twist vector", kBodyTwistNonFiniteMessage, valid,
                center, inf_twist);

  // Mass, inertia, and center defects win over a non-finite twist.
  ExpectInvalid("mass before twist", kMassNotPositiveMessage, zero_mass, center,
                nan_twist);
  ExpectInvalid("inertia before twist", kInertiaNotPositiveDefiniteMessage,
                singular, center, inf_twist);
  ExpectInvalid("center before twist", kCenterOfGravityNonFiniteMessage, valid,
                Vec3{std::numeric_limits<double>::quiet_NaN(), 0, 0},
                nan_twist);

  MassInertia huge = DiagonalInertia(1e300, 1.0, 1.0, 1.0);
  ExpectInvalid("mass overflow", kMassMatrixNonFiniteMessage, huge,
                Vec3{1e10, 0.0, 0.0}, twist);

  MassInertia large = DiagonalInertia(1e200, 1.0, 1.0, 1.0);
  ExpectInvalid("coriolis overflow", kCoriolisMatrixNonFiniteMessage, large,
                Vec3{0, 0, 0}, MakeTwist(1e200, 0, 0, 0, 0, 0));
}

TEST(RigidBodyCoriolisMatrix, SymmetryToleranceMatchesMassMatrix) {
  MassInertia inertia = DiagonalInertia(4.0, 2.0, 3.0, 4.0);
  inertia.inertia_about_center_of_gravity_kg_m2[1] = kMatrixSymmetryTolerance;
  inertia.inertia_about_center_of_gravity_kg_m2[3] = 0.0;
  const Twist twist = MakeTwist(0.1, 0.0, 0.0, 0.0, 0.2, 0.0);
  const StatusOr<RigidBodyCoriolisMatrix> result =
      ComputeRigidBodyCoriolisMatrix(inertia, Vec3{0.01, 0, 0}, twist);
  ASSERT_TRUE(result.ok());
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(RigidBodyCoriolisMatrix, RepeatedEvaluationIsBitIdentical) {
  const MassInertia mass = DiagonalInertia(10.0, 1.5, 2.5, 3.5);
  const Vec3 center = {0.1, -0.05, 0.2};
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const StatusOr<RigidBodyCoriolisMatrix> first =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  const StatusOr<RigidBodyCoriolisMatrix> second =
      ComputeRigidBodyCoriolisMatrix(mass, center, twist);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  ExpectBitIdentical(first.value().coefficients, second.value().coefficients);

  const StatusOr<RigidBodyCoriolisMatrix> bad_first =
      ComputeRigidBodyCoriolisMatrix(
          mass, center,
          MakeTwist(std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0));
  const StatusOr<RigidBodyCoriolisMatrix> bad_second =
      ComputeRigidBodyCoriolisMatrix(
          mass, center,
          MakeTwist(std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0));
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.status().message, kBodyTwistNonFiniteMessage);
  ExpectBitIdentical(bad_first.value().coefficients,
                     bad_second.value().coefficients);
}

}  // namespace
