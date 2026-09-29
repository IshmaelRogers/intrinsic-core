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

#include "intrinsic/vehicle/dynamics/added_mass_coriolis_matrix.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/rigid_body_coriolis_matrix.h"
#include "intrinsic/vehicle/dynamics/rigid_body_mass_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::AddedMassCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeAddedMassCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeRigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::ComputeRigidBodyMassMatrix;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::RigidBodyCoriolisMatrix;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::parameters::AddedMass;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::MassInertia;
using intrinsic::vehicle::parameters::Vec3;

// Approved numeric fixtures, skew-symmetry, and νᵀ C_A ν use this absolute
// tolerance. Hand values for the diagonal fixtures are exact in real
// arithmetic. 1e-12 covers roundoff on a general twist, the same bound
// used by the rigid-body Coriolis fixtures.
constexpr double kAbsTolerance = 1e-12;
constexpr int kInertia = 3;

constexpr std::string_view kAddedMassNonFiniteMessage =
    "added_mass.coefficients must be finite";
constexpr std::string_view kAddedMassAsymmetricMessage =
    "added_mass.coefficients differs from the transpose by more than 1e-9";
constexpr std::string_view kAddedMassNotPositiveDefiniteMessage =
    "added_mass.coefficients is not positive definite";
constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kCoriolisMatrixNonFiniteMessage =
    "added-mass Coriolis matrix is not finite";

using Twist = std::array<double, kSpatialDof>;
using Matrix = std::array<double, kSpatialDof * kSpatialDof>;

// Diagonal M_A used by the hand fixtures. Surge through yaw.
constexpr double kSurge = 4.0;
constexpr double kSway = 5.0;
constexpr double kHeave = 6.0;
constexpr double kRoll = 1.0;
constexpr double kPitch = 2.0;
constexpr double kYaw = 3.0;

// Hand-chosen symmetric M_A. Same coefficients as the added-mass matrix
// reference. One row per body axis.
constexpr Matrix kCoupledAddedMass = {
    10.0, 0.4,   0.0,  0.0,  0.2, 0.0,    // surge
    0.4,  20.0,  0.0,  0.0,  0.0, -0.15,  // sway
    0.0,  0.0,   30.0, -0.3, 0.0, 0.0,    // heave
    0.0,  0.0,   -0.3, 1.5,  0.0, 0.05,   // roll
    0.2,  0.0,   0.0,  0.0,  2.5, 0.0,    // pitch
    0.0,  -0.15, 0.0,  0.05, 0.0, 3.5,    // yaw
};

AddedMass DiagonalAddedMass(double surge, double sway, double heave,
                            double roll, double pitch, double yaw) {
  AddedMass added;
  const double diagonal[] = {surge, sway, heave, roll, pitch, yaw};
  for (int i = 0; i < kSpatialDof; ++i) {
    added.coefficients[i * kSpatialDof + i] = diagonal[i];
  }
  return added;
}

AddedMass FromCoefficients(const Matrix& coefficients) {
  AddedMass added;
  added.coefficients = coefficients;
  return added;
}

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

// Independent reference. p = M_A ν, split into p1 and p2.
// C11 = 0, C12 = C21 = -S(p1), C22 = -S(p2). Rigid-body mass is not used.
Matrix ReferenceCoriolis(const Matrix& added_mass, const Twist& twist) {
  double p1[kInertia] = {};
  double p2[kInertia] = {};
  for (int row = 0; row < kInertia; ++row) {
    for (int col = 0; col < kSpatialDof; ++col) {
      p1[row] += added_mass[At(row, col)] * twist[col];
      p2[row] += added_mass[At(kInertia + row, col)] * twist[col];
    }
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
                   const AddedMass& added, const Twist& twist) {
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(added, twist);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  const Matrix zeros = {};
  ExpectBitIdentical(result.value().coefficients, zeros);
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value)) << name;
  }
}

TEST(AddedMassCoriolisMatrix, ZeroTwistIsZeroMatrix) {
  const Twist zero = {};
  const AddedMass diagonal =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const StatusOr<AddedMassCoriolisMatrix> diagonal_result =
      ComputeAddedMassCoriolisMatrix(diagonal, zero);
  ASSERT_TRUE(diagonal_result.ok());
  ExpectBitIdentical(diagonal_result.value().coefficients, Matrix{});

  const StatusOr<AddedMassCoriolisMatrix> coupled =
      ComputeAddedMassCoriolisMatrix(FromCoefficients(kCoupledAddedMass), zero);
  ASSERT_TRUE(coupled.ok());
  ExpectBitIdentical(coupled.value().coefficients, Matrix{});

  const MarineModel model = MakeSixThrusterUuvExample();
  EXPECT_NE(model.added_mass.coefficients[0], 0.0);
  const StatusOr<AddedMassCoriolisMatrix> example =
      ComputeAddedMassCoriolisMatrix(model.added_mass, zero);
  ASSERT_TRUE(example.ok());
  ExpectBitIdentical(example.value().coefficients, Matrix{});
}

TEST(AddedMassCoriolisMatrix, SurgeVelocityMatchesApprovedFixture) {
  // M_A = diag(4, 5, 6, 1, 2, 3), ν = (1, 0, 0, 0, 0, 0).
  // p1 = (4, 0, 0), p2 = (0, 0, 0).
  // -S(p1) = [[0, 0, 0], [0, 0, 4], [0, -4, 0]].
  const AddedMass added =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const Twist twist = MakeTwist(1, 0, 0, 0, 0, 0);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(added, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0,  0, 0, -4, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, -4, 0, 0, 0,  0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(added.coefficients, twist));
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(AddedMassCoriolisMatrix, YawRateMatchesApprovedFixture) {
  // M_A = diag(4, 5, 6, 1, 2, 3), ν = (0, 0, 0, 0, 0, 1).
  // p1 = (0, 0, 0), p2 = (0, 0, 3).
  // -S(p2) = [[0, 3, 0], [-3, 0, 0], [0, 0, 0]].
  const AddedMass added =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const Twist twist = MakeTwist(0, 0, 0, 0, 0, 1);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(added, twist);
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

TEST(AddedMassCoriolisMatrix, AxisIsolatedDiagonalEntriesMatchSigns) {
  const AddedMass added =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);

  const Twist sway = MakeTwist(0, 1, 0, 0, 0, 0);
  const StatusOr<AddedMassCoriolisMatrix> sway_result =
      ComputeAddedMassCoriolisMatrix(added, sway);
  ASSERT_TRUE(sway_result.ok());
  // p1 = (0, 5, 0). -S(p1) places -5 at (0, 5) and 5 at (2, 3).
  EXPECT_NEAR(sway_result.value().coefficients[At(0, 5)], -kSway,
              kAbsTolerance);
  EXPECT_NEAR(sway_result.value().coefficients[At(2, 3)], kSway, kAbsTolerance);
  ExpectNearMatrix(sway_result.value().coefficients,
                   ReferenceCoriolis(added.coefficients, sway));

  const Twist heave = MakeTwist(0, 0, 1, 0, 0, 0);
  const StatusOr<AddedMassCoriolisMatrix> heave_result =
      ComputeAddedMassCoriolisMatrix(added, heave);
  ASSERT_TRUE(heave_result.ok());
  // p1 = (0, 0, 6). -S(p1) places 6 at (0, 4) and -6 at (1, 3).
  EXPECT_NEAR(heave_result.value().coefficients[At(0, 4)], kHeave,
              kAbsTolerance);
  EXPECT_NEAR(heave_result.value().coefficients[At(1, 3)], -kHeave,
              kAbsTolerance);

  const Twist roll = MakeTwist(0, 0, 0, 1, 0, 0);
  const StatusOr<AddedMassCoriolisMatrix> roll_result =
      ComputeAddedMassCoriolisMatrix(added, roll);
  ASSERT_TRUE(roll_result.ok());
  // p2 = (1, 0, 0). -S(p2) places 1 at (4, 5) and -1 at (5, 4).
  EXPECT_NEAR(roll_result.value().coefficients[At(4, 5)], kRoll, kAbsTolerance);
  EXPECT_NEAR(roll_result.value().coefficients[At(5, 4)], -kRoll,
              kAbsTolerance);
  EXPECT_NEAR(roll_result.value().coefficients[At(0, 5)], 0.0, kAbsTolerance);

  const Twist pitch = MakeTwist(0, 0, 0, 0, 1, 0);
  const StatusOr<AddedMassCoriolisMatrix> pitch_result =
      ComputeAddedMassCoriolisMatrix(added, pitch);
  ASSERT_TRUE(pitch_result.ok());
  // p2 = (0, 2, 0). -S(p2) places -2 at (3, 5) and 2 at (5, 3).
  EXPECT_NEAR(pitch_result.value().coefficients[At(3, 5)], -kPitch,
              kAbsTolerance);
  EXPECT_NEAR(pitch_result.value().coefficients[At(5, 3)], kPitch,
              kAbsTolerance);
}

TEST(AddedMassCoriolisMatrix, CoupledYawUsesOffDiagonalAddedMass) {
  // ν = (0, 0, 0, 0, 0, 1) on kCoupledAddedMass.
  // p1 = (0, -0.15, 0), p2 = (0.05, 0, 3.5).
  // The 0.15 and 0.05 entries are M_A couplings, not a rigid-body term.
  const Twist twist = MakeTwist(0, 0, 0, 0, 0, 1);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(FromCoefficients(kCoupledAddedMass),
                                     twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0,     0, 0.15, 0,     0, 0,    0, 0,     0,
      0, 0, 0, -0.15, 0, 0,    0,     0, 0.15, 0, 3.5,   0,
      0, 0, 0, -3.5,  0, 0.05, -0.15, 0, 0,    0, -0.05, 0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(kCoupledAddedMass, twist));
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  ExpectSkewSymmetric(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
}

TEST(AddedMassCoriolisMatrix, ReversedVelocityNegatesMatrix) {
  const AddedMass added = FromCoefficients(kCoupledAddedMass);
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const Twist reversed = MakeTwist(-0.2, 0.4, -0.6, -0.1, 0.3, -0.5);
  const StatusOr<AddedMassCoriolisMatrix> forward =
      ComputeAddedMassCoriolisMatrix(added, twist);
  const StatusOr<AddedMassCoriolisMatrix> backward =
      ComputeAddedMassCoriolisMatrix(added, reversed);
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

  const AddedMass diagonal =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const StatusOr<AddedMassCoriolisMatrix> surge =
      ComputeAddedMassCoriolisMatrix(diagonal, MakeTwist(1, 0, 0, 0, 0, 0));
  const StatusOr<AddedMassCoriolisMatrix> surge_back =
      ComputeAddedMassCoriolisMatrix(diagonal, MakeTwist(-1, 0, 0, 0, 0, 0));
  ASSERT_TRUE(surge.ok());
  ASSERT_TRUE(surge_back.ok());
  EXPECT_NEAR(surge.value().coefficients[At(1, 5)], kSurge, kAbsTolerance);
  EXPECT_NEAR(surge_back.value().coefficients[At(1, 5)], -kSurge,
              kAbsTolerance);
}

TEST(AddedMassCoriolisMatrix, CoupledTwistCancelsPower) {
  // Surge plus yaw. C_A ν is nonzero, and νᵀ C_A ν is still zero.
  const AddedMass added =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const Twist twist = MakeTwist(1, 0, 0, 0, 0, 1);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(added, twist);
  ASSERT_TRUE(result.ok());

  const Matrix expected = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 4, 0, 0,  0, 0, -4, 0,
      0, 0, 0, 0, 3, 0, 0, 0, 4, -3, 0, 0, 0, -4, 0, 0, 0,  0,
  };
  ExpectNearMatrix(result.value().coefficients, expected);
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  ExpectSkewSymmetric(result.value().coefficients);
  EXPECT_GT(CoriolisForceNorm(result.value().coefficients, twist), 1.0);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);

  const StatusOr<AddedMassCoriolisMatrix> surge =
      ComputeAddedMassCoriolisMatrix(added, MakeTwist(1, 0, 0, 0, 0, 0));
  const StatusOr<AddedMassCoriolisMatrix> yaw =
      ComputeAddedMassCoriolisMatrix(added, MakeTwist(0, 0, 0, 0, 0, 1));
  ASSERT_TRUE(surge.ok());
  ASSERT_TRUE(yaw.ok());
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    EXPECT_NEAR(result.value().coefficients[i],
                surge.value().coefficients[i] + yaw.value().coefficients[i],
                kAbsTolerance)
        << "index " << i;
  }
}

TEST(AddedMassCoriolisMatrix, GeneralTwistMatchesBlocksAndPower) {
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(FromCoefficients(kCoupledAddedMass),
                                     twist);
  ASSERT_TRUE(result.ok());
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(kCoupledAddedMass, twist));
  ExpectUpperLeftExactlyZero(result.value().coefficients);
  ExpectSkewSymmetric(result.value().coefficients);
  // C_A ν is not the zero vector, so the power check is a cancellation.
  EXPECT_GT(CoriolisForceNorm(result.value().coefficients, twist), 1e-3);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

TEST(AddedMassCoriolisMatrix, ResultIgnoresRigidBody) {
  const MarineModel model = MakeSixThrusterUuvExample();
  EXPECT_GT(model.mass_inertia.mass_kg, 1.0);
  EXPECT_GT(model.added_mass.coefficients[0], 1.0);
  const Twist twist = MakeTwist(1.0, 0.2, -0.3, 0.05, -0.02, 0.1);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(model.added_mass, twist);
  ASSERT_TRUE(result.ok());

  const Matrix added_only =
      ReferenceCoriolis(model.added_mass.coefficients, twist);
  ExpectNearMatrix(result.value().coefficients, added_only);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);

  const StatusOr<RigidBodyCoriolisMatrix> rigid =
      ComputeRigidBodyCoriolisMatrix(model.mass_inertia,
                                     model.centers.center_of_gravity_m, twist);
  ASSERT_TRUE(rigid.ok());
  double max_gap = 0.0;
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    max_gap = std::max(max_gap, std::abs(result.value().coefficients[i] -
                                         rigid.value().coefficients[i]));
  }
  EXPECT_GT(max_gap, 1.0);

  const auto mass = ComputeRigidBodyMassMatrix(
      model.mass_inertia, model.centers.center_of_gravity_m);
  ASSERT_TRUE(mass.ok());
  Matrix total = mass.value().coefficients;
  for (int i = 0; i < kSpatialDof * kSpatialDof; ++i) {
    total[i] += model.added_mass.coefficients[i];
  }
  const Matrix with_rigid_body = ReferenceCoriolis(total, twist);
  // -S(p1) entry (0, 5) is -p1_y. Adding M_RB changes p1.
  EXPECT_GT(std::abs(with_rigid_body[At(0, 5)] -
                     result.value().coefficients[At(0, 5)]),
            1.0);

  // A zero added-mass matrix is not positive definite. The failure payload
  // is the finite zero matrix, which is not the rigid-body Coriolis matrix
  // of a moving body.
  const Twist moving = MakeTwist(1, 0.2, -0.3, 0.1, -0.2, 0.4);
  ExpectInvalid("zero added mass", kAddedMassNotPositiveDefiniteMessage,
                AddedMass{}, moving);
  const StatusOr<RigidBodyCoriolisMatrix> moving_rigid =
      ComputeRigidBodyCoriolisMatrix(DiagonalInertia(10.0, 1.0, 2.0, 3.0),
                                     Vec3{0, 0, 0}, moving);
  ASSERT_TRUE(moving_rigid.ok());
  EXPECT_GT(CoriolisForceNorm(moving_rigid.value().coefficients, moving), 1.0);
}

TEST(AddedMassCoriolisMatrix, InvalidInputsReturnStatusAndZeros) {
  const AddedMass valid =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  const Twist twist = MakeTwist(0.2, -0.1, 0.3, 0.0, 0.1, -0.2);
  const Twist nan_twist =
      MakeTwist(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0);
  const Twist inf_twist =
      MakeTwist(0, 0, 0, 0, std::numeric_limits<double>::infinity(), 0);

  AddedMass nan_entry = valid;
  nan_entry.coefficients[0] = std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan entry", kAddedMassNonFiniteMessage, nan_entry, twist);

  AddedMass inf_entry = valid;
  inf_entry.coefficients[kSpatialDof + 1] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite entry", kAddedMassNonFiniteMessage, inf_entry, twist);

  AddedMass negative_inf = valid;
  negative_inf.coefficients[At(5, 5)] =
      -std::numeric_limits<double>::infinity();
  ExpectInvalid("negative infinity", kAddedMassNonFiniteMessage, negative_inf,
                twist);

  AddedMass both = valid;
  both.coefficients[0] = std::numeric_limits<double>::quiet_NaN();
  both.coefficients[1] = 1.0;
  ExpectInvalid("non-finite before asymmetry", kAddedMassNonFiniteMessage, both,
                twist);

  AddedMass asymmetric = valid;
  asymmetric.coefficients[1] = 0.5;
  ExpectInvalid("asymmetric added mass", kAddedMassAsymmetricMessage,
                asymmetric, twist);

  AddedMass just_over = valid;
  just_over.coefficients[1] = kMatrixSymmetryTolerance + 1e-12;
  ExpectInvalid("asymmetry beyond 1e-9", kAddedMassAsymmetricMessage, just_over,
                twist);

  AddedMass asymmetric_and_indefinite = valid;
  asymmetric_and_indefinite.coefficients[At(5, 5)] = -1.0;
  asymmetric_and_indefinite.coefficients[1] = 0.5;
  ExpectInvalid("asymmetry before definiteness", kAddedMassAsymmetricMessage,
                asymmetric_and_indefinite, twist);

  AddedMass singular = DiagonalAddedMass(1.0, 1.0, 1.0, 1.0, 1.0, 0.0);
  ExpectInvalid("singular diagonal", kAddedMassNotPositiveDefiniteMessage,
                singular, twist);

  AddedMass indefinite = DiagonalAddedMass(1.0, 1.0, 1.0, 1.0, 1.0, -1.0);
  ExpectInvalid("indefinite diagonal", kAddedMassNotPositiveDefiniteMessage,
                indefinite, twist);

  ExpectInvalid("zero matrix", kAddedMassNotPositiveDefiniteMessage,
                AddedMass{}, twist);

  for (int index = 0; index < kSpatialDof; ++index) {
    Twist slot = twist;
    slot[index] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalid("nan twist", kBodyTwistNonFiniteMessage, valid, slot);
    slot[index] = std::numeric_limits<double>::infinity();
    ExpectInvalid("infinite twist", kBodyTwistNonFiniteMessage, valid, slot);
    slot[index] = -std::numeric_limits<double>::infinity();
    ExpectInvalid("negative infinite twist", kBodyTwistNonFiniteMessage, valid,
                  slot);
  }
  ExpectInvalid("nan twist vector", kBodyTwistNonFiniteMessage, valid,
                nan_twist);
  ExpectInvalid("infinite twist vector", kBodyTwistNonFiniteMessage, valid,
                inf_twist);

  // Added-mass defects win over a non-finite twist.
  ExpectInvalid("added mass before twist", kAddedMassNonFiniteMessage,
                nan_entry, nan_twist);
  ExpectInvalid("asymmetry before twist", kAddedMassAsymmetricMessage,
                asymmetric, inf_twist);
  ExpectInvalid("definiteness before twist",
                kAddedMassNotPositiveDefiniteMessage, singular, nan_twist);

  AddedMass large = DiagonalAddedMass(1e200, 1.0, 1.0, 1.0, 1.0, 1.0);
  ExpectInvalid("coriolis overflow", kCoriolisMatrixNonFiniteMessage, large,
                MakeTwist(1e200, 0, 0, 0, 0, 0));
}

TEST(AddedMassCoriolisMatrix, SymmetryToleranceMatchesAddedMass) {
  AddedMass inside =
      DiagonalAddedMass(kSurge, kSway, kHeave, kRoll, kPitch, kYaw);
  inside.coefficients[1] = kMatrixSymmetryTolerance;
  const Twist twist = MakeTwist(0.1, 0.0, 0.0, 0.0, 0.2, 0.0);
  const StatusOr<AddedMassCoriolisMatrix> result =
      ComputeAddedMassCoriolisMatrix(inside, twist);
  ASSERT_TRUE(result.ok());
  for (double value : result.value().coefficients) {
    EXPECT_TRUE(std::isfinite(value));
  }
  ExpectSkewSymmetric(result.value().coefficients);
  EXPECT_NEAR(Power(result.value().coefficients, twist), 0.0, kAbsTolerance);
  ExpectNearMatrix(result.value().coefficients,
                   ReferenceCoriolis(inside.coefficients, twist));
}

TEST(AddedMassCoriolisMatrix, RepeatedEvaluationIsBitIdentical) {
  const AddedMass added = FromCoefficients(kCoupledAddedMass);
  const Twist twist = MakeTwist(0.2, -0.4, 0.6, 0.1, -0.3, 0.5);
  const StatusOr<AddedMassCoriolisMatrix> first =
      ComputeAddedMassCoriolisMatrix(added, twist);
  const StatusOr<AddedMassCoriolisMatrix> second =
      ComputeAddedMassCoriolisMatrix(added, twist);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  ExpectBitIdentical(first.value().coefficients, second.value().coefficients);

  const StatusOr<AddedMassCoriolisMatrix> bad_first =
      ComputeAddedMassCoriolisMatrix(
          added,
          MakeTwist(std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0));
  const StatusOr<AddedMassCoriolisMatrix> bad_second =
      ComputeAddedMassCoriolisMatrix(
          added,
          MakeTwist(std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0));
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.status().message, kBodyTwistNonFiniteMessage);
  EXPECT_EQ(bad_first.status().code, DynamicsErrorCode::kInvalidArgument);
  ExpectBitIdentical(bad_first.value().coefficients,
                     bad_second.value().coefficients);
}

}  // namespace
