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

#include "intrinsic/vehicle/dynamics/linear_quadratic_damping_wrench.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::ComputeLinearQuadraticDampingWrench;
using intrinsic::vehicle::dynamics::DampingWrench;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::FrameId;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::dynamics::ZeroForceDynamics;
using intrinsic::vehicle::parameters::Damping;
using intrinsic::vehicle::parameters::kMatrixSymmetryTolerance;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::ValidateMarineModel;

// Hand-calculated wrench fixtures use this absolute tolerance. The
// reference values below are dyadic rationals, so the approved products
// are exact in IEEE binary64. 1e-12 matches the Coriolis fixtures.
constexpr double kAbsTolerance = 1e-12;

constexpr std::string_view kLinearNonFiniteMessage =
    "damping.linear_coefficients must be finite";
constexpr std::string_view kLinearAsymmetricMessage =
    "damping.linear_coefficients differs from the transpose by more than "
    "1e-9";
constexpr std::string_view kLinearNotPositiveDefiniteMessage =
    "damping.linear_coefficients is not positive definite";
constexpr std::string_view kQuadraticNonFiniteMessage =
    "damping.quadratic_coefficients must be finite";
constexpr std::string_view kQuadraticNegativeMessage =
    "damping.quadratic_coefficients must be greater than or equal to zero";
constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kDampingWrenchNonFiniteMessage =
    "damping wrench is not finite";

using Twist = std::array<double, kSpatialDof>;
using Matrix = std::array<double, kSpatialDof * kSpatialDof>;

int At(int row, int col) { return row * kSpatialDof + col; }

Twist MakeTwist(double u, double v, double w, double p, double q, double r) {
  return {u, v, w, p, q, r};
}

Damping DiagonalDamping(const Twist& linear_diagonal, const Twist& quadratic) {
  Damping damping;
  damping.quadratic_coefficients = quadratic;
  for (int i = 0; i < kSpatialDof; ++i) {
    damping.linear_coefficients[At(i, i)] = linear_diagonal[i];
  }
  return damping;
}

// D_L = diag(4, 8, 2, 1, 1/2, 1/4). Each entry is a power of two.
constexpr Twist kDiagonalLinear = {4.0, 8.0, 2.0, 1.0, 0.5, 0.25};
// d_q with a zero pitch coefficient. Zero is an allowed coefficient.
constexpr Twist kQuadraticCoeffs = {2.0, 4.0, 8.0, 1.0, 0.0, 0.5};
// ν = [1, -1/2, 1/4, 2, -4, 8].
constexpr Twist kReferenceTwist = {1.0, -0.5, 0.25, 2.0, -4.0, 8.0};

// τ = -D_L ν with d_q = 0:
//   [-4, 4, -1/2, -2, 2, -2]
constexpr Twist kLinearOnlyWrench = {-4.0, 4.0, -0.5, -2.0, 2.0, -2.0};
// Quadratic contribution -d_q_i |ν_i| ν_i:
//   [-2, 1, -1/2, -4, 0, -32]
constexpr Twist kQuadraticOnlyWrench = {-2.0, 1.0, -0.5, -4.0, 0.0, -32.0};
// Sum of the two contributions:
//   [-6, 5, -1, -6, 2, -34]
constexpr Twist kCombinedWrench = {-6.0, 5.0, -1.0, -6.0, 2.0, -34.0};

void ExpectBitIdentical(const Twist& actual, const Twist& expected) {
  EXPECT_EQ(0, std::memcmp(actual.data(), expected.data(),
                           actual.size() * sizeof(double)));
}

void ExpectNearWrench(const Twist& actual, const Twist& expected) {
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_NEAR(actual[i], expected[i], kAbsTolerance) << "component " << i;
  }
}

void ExpectPositiveZero(const Twist& wrench) {
  const double zero = 0.0;
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_TRUE(std::isfinite(wrench[i])) << i;
    EXPECT_EQ(0, std::memcmp(&wrench[i], &zero, sizeof(double))) << i;
  }
}

double Power(const Twist& wrench, const Twist& twist) {
  double power = 0.0;
  for (int i = 0; i < kSpatialDof; ++i) {
    power += twist[i] * wrench[i];
  }
  return power;
}

void ExpectOpposesMotion(const Twist& wrench, const Twist& twist) {
  EXPECT_LE(Power(wrench, twist), kAbsTolerance);
  for (int i = 0; i < kSpatialDof; ++i) {
    if (twist[i] > 0.0) {
      EXPECT_LT(wrench[i], 0.0) << i;
    } else if (twist[i] < 0.0) {
      EXPECT_GT(wrench[i], 0.0) << i;
    } else {
      EXPECT_EQ(wrench[i], 0.0) << i;
    }
  }
}

void ExpectInvalid(const char* name, std::string_view message,
                   const Damping& damping, const Twist& twist) {
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, twist);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  ExpectPositiveZero(result.value().components);
}

TEST(LinearQuadraticDamping, ZeroRelativeVelocityReturnsZeroWrench) {
  const Damping damping = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, Twist{});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.status().message, std::string_view());
  ExpectPositiveZero(result.value().components);
  EXPECT_NEAR(Power(result.value().components, Twist{}), 0.0, kAbsTolerance);

  // A zero coefficient on every quadratic entry is still zero at rest.
  const Damping linear_only = DiagonalDamping(kDiagonalLinear, Twist{});
  const StatusOr<DampingWrench> linear =
      ComputeLinearQuadraticDampingWrench(linear_only, Twist{});
  ASSERT_TRUE(linear.ok());
  ExpectPositiveZero(linear.value().components);

  Twist negative_zero;
  negative_zero.fill(-0.0);
  const StatusOr<DampingWrench> signed_zero =
      ComputeLinearQuadraticDampingWrench(damping, negative_zero);
  ASSERT_TRUE(signed_zero.ok());
  ExpectPositiveZero(signed_zero.value().components);
}

TEST(LinearQuadraticDamping, LinearOnlyMatchesHandValues) {
  const Damping damping = DiagonalDamping(kDiagonalLinear, Twist{});
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, kReferenceTwist);
  ASSERT_TRUE(result.ok());
  ExpectNearWrench(result.value().components, kLinearOnlyWrench);
  ExpectOpposesMotion(result.value().components, kReferenceTwist);
  EXPECT_LT(Power(result.value().components, kReferenceTwist), 0.0);
}

TEST(LinearQuadraticDamping, QuadraticOnlyMatchesHandValues) {
  // D_L stays positive definite. The quadratic-only fixture is the
  // difference between the combined wrench and the linear-only wrench.
  const Damping linear_only = DiagonalDamping(kDiagonalLinear, Twist{});
  const Damping combined_damping =
      DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> linear =
      ComputeLinearQuadraticDampingWrench(linear_only, kReferenceTwist);
  const StatusOr<DampingWrench> combined =
      ComputeLinearQuadraticDampingWrench(combined_damping, kReferenceTwist);
  ASSERT_TRUE(linear.ok());
  ASSERT_TRUE(combined.ok());

  Twist quadratic = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    quadratic[i] =
        combined.value().components[i] - linear.value().components[i];
  }
  ExpectNearWrench(quadratic, kQuadraticOnlyWrench);
  // Pitch quadratic coefficient is zero, so that contribution is zero
  // while pitch velocity is not.
  EXPECT_NEAR(quadratic[4], 0.0, kAbsTolerance);
  EXPECT_NE(kReferenceTwist[4], 0.0);
  EXPECT_LE(Power(quadratic, kReferenceTwist), kAbsTolerance);
  EXPECT_LT(Power(quadratic, kReferenceTwist), 0.0);
}

TEST(LinearQuadraticDamping, CombinedMatchesHandValues) {
  const Damping damping = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, kReferenceTwist);
  ASSERT_TRUE(result.ok());
  ExpectNearWrench(result.value().components, kCombinedWrench);
  ExpectOpposesMotion(result.value().components, kReferenceTwist);
  EXPECT_LT(Power(result.value().components, kReferenceTwist), 0.0);

  Twist summed = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    summed[i] = kLinearOnlyWrench[i] + kQuadraticOnlyWrench[i];
  }
  ExpectNearWrench(result.value().components, summed);
}

TEST(LinearQuadraticDamping, AxisIsolatedSignOpposesMotion) {
  constexpr Twist kBaseline = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  constexpr Twist kQuadratic = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  for (int axis = 0; axis < kSpatialDof; ++axis) {
    Twist linear = kBaseline;
    linear[axis] = 4.0;
    const Damping damping = DiagonalDamping(linear, kQuadratic);
    for (double speed : {2.0, -2.0}) {
      Twist twist = {};
      twist[axis] = speed;
      const StatusOr<DampingWrench> result =
          ComputeLinearQuadraticDampingWrench(damping, twist);
      ASSERT_TRUE(result.ok()) << axis << " " << speed;
      // τ_i = -(4 ν_i + 2 |ν_i| ν_i). For |ν_i| = 2 this is -16 sign(ν_i).
      const double expected = speed > 0.0 ? -16.0 : 16.0;
      EXPECT_NEAR(result.value().components[axis], expected, kAbsTolerance)
          << axis;
      for (int other = 0; other < kSpatialDof; ++other) {
        if (other == axis) {
          continue;
        }
        EXPECT_EQ(result.value().components[other], 0.0)
            << axis << " leaks into " << other;
      }
      ExpectOpposesMotion(result.value().components, twist);
    }
  }

  // A zero quadratic coefficient still opposes through the linear term.
  Twist linear = kBaseline;
  linear[0] = 4.0;
  Twist quadratic = {};
  const Damping damping = DiagonalDamping(linear, quadratic);
  const Twist twist = MakeTwist(2.0, 0, 0, 0, 0, 0);
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, twist);
  ASSERT_TRUE(result.ok());
  EXPECT_NEAR(result.value().components[0], -8.0, kAbsTolerance);
  ExpectOpposesMotion(result.value().components, twist);
}

TEST(LinearQuadraticDamping, CoupledFixtureOpposesMotion) {
  // Hand values. D_L is block diagonal and positive definite.
  // Surge/sway block [[4, 1/2], [1/2, 8]], heave 2,
  // roll/pitch block [[1, 1/4], [1/4, 2]], yaw 1/2.
  // d_q = [1, 2, 0, 1/2, 0, 1]. Heave and pitch quadratic coefficients
  // are zero. ν = [1, -2, 1/2, -1/2, 1, -4].
  //
  // Linear contribution:
  //   [-3, 15.5, -1, 1/4, -1.875, 2]
  // Quadratic contribution:
  //   [-1, 8, 0, 1/8, 0, 16]
  // Combined:
  //   [-4, 23.5, -1, 3/8, -1.875, 18]
  Damping damping;
  damping.linear_coefficients = {
      4.0, 0.5, 0.0, 0.0,  0.0,  0.0,  // surge
      0.5, 8.0, 0.0, 0.0,  0.0,  0.0,  // sway
      0.0, 0.0, 2.0, 0.0,  0.0,  0.0,  // heave
      0.0, 0.0, 0.0, 1.0,  0.25, 0.0,  // roll
      0.0, 0.0, 0.0, 0.25, 2.0,  0.0,  // pitch
      0.0, 0.0, 0.0, 0.0,  0.0,  0.5,  // yaw
  };
  damping.quadratic_coefficients = {1.0, 2.0, 0.0, 0.5, 0.0, 1.0};
  const Twist twist = MakeTwist(1.0, -2.0, 0.5, -0.5, 1.0, -4.0);
  constexpr Twist kExpected = {-4.0, 23.5, -1.0, 0.375, -1.875, 18.0};

  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, twist);
  ASSERT_TRUE(result.ok());
  ExpectNearWrench(result.value().components, kExpected);
  ExpectOpposesMotion(result.value().components, twist);
  EXPECT_LT(Power(result.value().components, twist), 0.0);

  MarineModel model = MakeSixThrusterUuvExample();
  model.damping = damping;
  EXPECT_TRUE(ValidateMarineModel(model).ok());
}

TEST(LinearQuadraticDamping, ReversedVelocityFlipsWrenchSign) {
  const Damping diagonal = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> forward =
      ComputeLinearQuadraticDampingWrench(diagonal, kReferenceTwist);
  Twist reversed_twist = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    reversed_twist[i] = -kReferenceTwist[i];
  }
  const StatusOr<DampingWrench> reversed =
      ComputeLinearQuadraticDampingWrench(diagonal, reversed_twist);
  ASSERT_TRUE(forward.ok());
  ASSERT_TRUE(reversed.ok());
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_NEAR(reversed.value().components[i], -forward.value().components[i],
                kAbsTolerance)
        << i;
    EXPECT_LT(reversed.value().components[i] * forward.value().components[i],
              0.0)
        << i;
  }
  ExpectOpposesMotion(reversed.value().components, reversed_twist);

  Damping coupled;
  coupled.linear_coefficients = {
      4.0, 0.5, 0.0, 0.0,  0.0, 0.0, 0.5, 8.0, 0.0, 0.0, 0.0,  0.0,
      0.0, 0.0, 2.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.25, 0.0,
      0.0, 0.0, 0.0, 0.25, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.5,
  };
  coupled.quadratic_coefficients = {1.0, 2.0, 0.0, 0.5, 0.0, 1.0};
  const Twist coupled_twist = MakeTwist(1.0, -2.0, 0.5, -0.5, 1.0, -4.0);
  Twist coupled_reversed = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    coupled_reversed[i] = -coupled_twist[i];
  }
  const StatusOr<DampingWrench> coupled_forward =
      ComputeLinearQuadraticDampingWrench(coupled, coupled_twist);
  const StatusOr<DampingWrench> coupled_back =
      ComputeLinearQuadraticDampingWrench(coupled, coupled_reversed);
  ASSERT_TRUE(coupled_forward.ok());
  ASSERT_TRUE(coupled_back.ok());
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_NEAR(coupled_back.value().components[i],
                -coupled_forward.value().components[i], kAbsTolerance)
        << i;
  }
}

TEST(LinearQuadraticDamping, SuppliedTwistIsNotCurrentFrameConverted) {
  // The leaf has no pose, environment, or current argument. A positive
  // body heave rate stays positive in the formula. ENU/NED would flip the
  // world z axis; this wrench is the body-frame result for the twist as
  // given: τ_heave = -(2 * 1/2 + 8 * |1/2| * 1/2) = -3.
  constexpr Twist kLinear = {1.0, 1.0, 2.0, 1.0, 1.0, 1.0};
  constexpr Twist kQuadratic = {0.0, 0.0, 8.0, 0.0, 0.0, 0.0};
  const Damping damping = DiagonalDamping(kLinear, kQuadratic);
  const Twist twist = MakeTwist(0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(damping, twist);
  ASSERT_TRUE(result.ok());
  constexpr Twist kExpected = {0.0, 0.0, -3.0, 0.0, 0.0, 0.0};
  ExpectNearWrench(result.value().components, kExpected);
  EXPECT_LT(result.value().components[2], 0.0);
  EXPECT_GT(twist[2], 0.0);
}

TEST(LinearQuadraticDamping, ExampleParametersStayPassive) {
  const MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_TRUE(ValidateMarineModel(model).ok());
  const Twist twist = MakeTwist(0.5, -0.25, 0.25, 0.0, -0.5, 1.0);
  const StatusOr<DampingWrench> moving =
      ComputeLinearQuadraticDampingWrench(model.damping, twist);
  ASSERT_TRUE(moving.ok());
  ExpectOpposesMotion(moving.value().components, twist);
  EXPECT_LT(Power(moving.value().components, twist), 0.0);
  for (double component : moving.value().components) {
    EXPECT_TRUE(std::isfinite(component));
  }
  // Roll quadratic damping on the example is zero and remains valid.
  EXPECT_EQ(model.damping.quadratic_coefficients[3], 0.0);

  const StatusOr<DampingWrench> stopped =
      ComputeLinearQuadraticDampingWrench(model.damping, Twist{});
  ASSERT_TRUE(stopped.ok());
  ExpectPositiveZero(stopped.value().components);
}

TEST(LinearQuadraticDamping, InvalidInputsReturnStatusAndZeros) {
  const Damping valid = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const Twist twist = kReferenceTwist;
  const Twist nan_twist =
      MakeTwist(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0);
  const Twist inf_twist =
      MakeTwist(0, 0, 0, 0, std::numeric_limits<double>::infinity(), 0);

  Damping nan_linear = valid;
  nan_linear.linear_coefficients[At(0, 4)] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan linear", kLinearNonFiniteMessage, nan_linear, twist);

  Damping inf_linear = valid;
  inf_linear.linear_coefficients[At(5, 5)] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite linear", kLinearNonFiniteMessage, inf_linear, twist);

  Damping negative_inf = valid;
  negative_inf.linear_coefficients[At(1, 2)] =
      -std::numeric_limits<double>::infinity();
  ExpectInvalid("negative infinite linear", kLinearNonFiniteMessage,
                negative_inf, twist);

  Damping asymmetric = valid;
  asymmetric.linear_coefficients[At(0, 1)] = 1.0;
  asymmetric.linear_coefficients[At(1, 0)] = 0.0;
  ExpectInvalid("asymmetric linear", kLinearAsymmetricMessage, asymmetric,
                twist);

  Damping just_over = valid;
  just_over.linear_coefficients[At(0, 1)] = kMatrixSymmetryTolerance + 1e-12;
  ExpectInvalid("asymmetry beyond 1e-9", kLinearAsymmetricMessage, just_over,
                twist);

  Damping singular = DiagonalDamping(Twist{1, 1, 1, 1, 1, 0}, Twist{});
  ExpectInvalid("singular linear", kLinearNotPositiveDefiniteMessage, singular,
                twist);

  Damping indefinite = DiagonalDamping(Twist{1, 1, 1, 1, 1, -1}, Twist{});
  ExpectInvalid("indefinite linear", kLinearNotPositiveDefiniteMessage,
                indefinite, twist);

  Damping not_diagonal_dominant = valid;
  not_diagonal_dominant.linear_coefficients[At(0, 1)] = 100.0;
  not_diagonal_dominant.linear_coefficients[At(1, 0)] = 100.0;
  ExpectInvalid("indefinite coupling", kLinearNotPositiveDefiniteMessage,
                not_diagonal_dominant, twist);

  ExpectInvalid("zero linear", kLinearNotPositiveDefiniteMessage, Damping{},
                twist);

  Damping nan_quadratic = valid;
  nan_quadratic.quadratic_coefficients[2] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("nan quadratic", kQuadraticNonFiniteMessage, nan_quadratic,
                twist);

  Damping inf_quadratic = valid;
  inf_quadratic.quadratic_coefficients[0] =
      std::numeric_limits<double>::infinity();
  ExpectInvalid("infinite quadratic", kQuadraticNonFiniteMessage, inf_quadratic,
                twist);

  Damping negative_quadratic = valid;
  negative_quadratic.quadratic_coefficients[3] = -0.2;
  ExpectInvalid("negative quadratic", kQuadraticNegativeMessage,
                negative_quadratic, twist);

  Damping tiny_negative = valid;
  tiny_negative.quadratic_coefficients[1] = -1e-12;
  ExpectInvalid("tiny negative quadratic", kQuadraticNegativeMessage,
                tiny_negative, twist);

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

  // Category order: linear non-finite, asymmetry, definiteness, quadratic
  // non-finite, quadratic sign, then twist.
  Damping nonfinite_and_asymmetric = valid;
  nonfinite_and_asymmetric.linear_coefficients[At(5, 5)] =
      std::numeric_limits<double>::quiet_NaN();
  nonfinite_and_asymmetric.linear_coefficients[At(0, 1)] = 1.0;
  ExpectInvalid("non-finite before asymmetry", kLinearNonFiniteMessage,
                nonfinite_and_asymmetric, nan_twist);

  Damping asymmetric_and_indefinite =
      DiagonalDamping(Twist{1, 1, 1, 1, 1, -1}, Twist{});
  asymmetric_and_indefinite.linear_coefficients[At(0, 1)] = 1.0;
  asymmetric_and_indefinite.quadratic_coefficients[0] = -1.0;
  ExpectInvalid("asymmetry before definiteness", kLinearAsymmetricMessage,
                asymmetric_and_indefinite, inf_twist);

  Damping indefinite_and_quadratic = indefinite;
  indefinite_and_quadratic.quadratic_coefficients[0] = -1.0;
  ExpectInvalid("definiteness before quadratic",
                kLinearNotPositiveDefiniteMessage, indefinite_and_quadratic,
                nan_twist);

  Damping quadratic_both = valid;
  quadratic_both.quadratic_coefficients[0] = -1.0;
  quadratic_both.quadratic_coefficients[5] =
      std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid("quadratic non-finite before sign", kQuadraticNonFiniteMessage,
                quadratic_both, nan_twist);

  ExpectInvalid("quadratic sign before twist", kQuadraticNegativeMessage,
                negative_quadratic, nan_twist);

  Damping huge = DiagonalDamping(Twist{1e300, 1, 1, 1, 1, 1}, Twist{});
  ExpectInvalid("linear overflow", kDampingWrenchNonFiniteMessage, huge,
                MakeTwist(1e200, 0, 0, 0, 0, 0));

  Damping quadratic_huge =
      DiagonalDamping(Twist{1, 1, 1, 1, 1, 1}, Twist{1e200, 0, 0, 0, 0, 0});
  ExpectInvalid("quadratic overflow", kDampingWrenchNonFiniteMessage,
                quadratic_huge, MakeTwist(1e200, 0, 0, 0, 0, 0));
}

TEST(LinearQuadraticDamping, SymmetryToleranceUsesStoredEntries) {
  Damping inside = DiagonalDamping(Twist{4, 4, 4, 4, 4, 4}, Twist{});
  inside.linear_coefficients[At(0, 1)] = kMatrixSymmetryTolerance;
  inside.linear_coefficients[At(1, 0)] = 0.0;
  const Twist twist = MakeTwist(2.0, 0, 0, 0, 0, 0);
  const StatusOr<DampingWrench> result =
      ComputeLinearQuadraticDampingWrench(inside, twist);
  ASSERT_TRUE(result.ok());
  // Stored lower entry is 0, so sway force is 0. The upper entry is not
  // copied into the product.
  EXPECT_NEAR(result.value().components[0], -8.0, kAbsTolerance);
  EXPECT_EQ(result.value().components[1], 0.0);
  for (double component : result.value().components) {
    EXPECT_TRUE(std::isfinite(component));
  }
  EXPECT_LE(Power(result.value().components, twist), kAbsTolerance);

  MarineModel model = MakeSixThrusterUuvExample();
  model.damping = inside;
  EXPECT_TRUE(ValidateMarineModel(model).ok());

  Damping outside = inside;
  outside.linear_coefficients[At(0, 1)] = 2e-9;
  const StatusOr<DampingWrench> rejected =
      ComputeLinearQuadraticDampingWrench(outside, twist);
  ASSERT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.status().message, kLinearAsymmetricMessage);
  model.damping = outside;
  EXPECT_FALSE(ValidateMarineModel(model).ok());
}

TEST(LinearQuadraticDamping, RepeatedEvaluationIsBitIdentical) {
  const Damping damping = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> first =
      ComputeLinearQuadraticDampingWrench(damping, kReferenceTwist);
  const StatusOr<DampingWrench> second =
      ComputeLinearQuadraticDampingWrench(damping, kReferenceTwist);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  ExpectBitIdentical(first.value().components, second.value().components);
  ExpectNearWrench(first.value().components, kCombinedWrench);

  const Twist bad_twist =
      MakeTwist(std::numeric_limits<double>::infinity(), 0, 0, 0, 0, 0);
  const StatusOr<DampingWrench> bad_first =
      ComputeLinearQuadraticDampingWrench(damping, bad_twist);
  const StatusOr<DampingWrench> bad_second =
      ComputeLinearQuadraticDampingWrench(damping, bad_twist);
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.status().message, kBodyTwistNonFiniteMessage);
  EXPECT_EQ(bad_first.status().code, DynamicsErrorCode::kInvalidArgument);
  ExpectBitIdentical(bad_first.value().components,
                     bad_second.value().components);
  ExpectPositiveZero(bad_first.value().components);
}

TEST(LinearQuadraticDamping, EvaluateDoesNotApplyDamping) {
  const Damping damping = DiagonalDamping(kDiagonalLinear, kQuadraticCoeffs);
  const StatusOr<DampingWrench> damping_wrench =
      ComputeLinearQuadraticDampingWrench(damping, kReferenceTwist);
  ASSERT_TRUE(damping_wrench.ok());
  EXPECT_LT(Power(damping_wrench.value().components, kReferenceTwist), 0.0);

  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = {0, 0, 0, 1};
  state.body_twist = kReferenceTwist;
  EnvironmentRt environment;
  environment.current_frame = FrameId::kWorldEnu;
  environment.current_velocity_m_s = {3.0, -1.0, 0.5};
  const ZeroForceDynamics model;
  const StatusOr<intrinsic::vehicle::dynamics::DynamicsResult> evaluated =
      model.Evaluate(state, BodyWrenchRt{}, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  for (double acceleration : evaluated.value().derivative.body_acceleration) {
    EXPECT_EQ(acceleration, 0.0);
  }
  EXPECT_EQ(evaluated.value().diagnostics.model_force_n,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_EQ(evaluated.value().diagnostics.model_torque_n_m,
            (std::array<double, 3>{0, 0, 0}));
  EXPECT_FALSE(evaluated.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(evaluated.value().diagnostics.allocation_invoked);
}

}  // namespace
