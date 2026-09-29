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
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

using parameters::kInertiaDof;
using parameters::kMatrixSymmetryTolerance;

static_assert(kSpatialDof == parameters::kSpatialDof);
static_assert(kInertiaDof == 3);

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

StatusOr<RigidBodyMassMatrix> Reject(std::string_view message) {
  return StatusOr<RigidBodyMassMatrix>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// Unpivoted Cholesky of a symmetric 3x3 row-major matrix. A pivot that is
// not strictly positive fails. No pivot slack, matching ValidateMarineModel.
bool IsPositiveDefinite3(const double* a) {
  double factor[kInertiaDof * kInertiaDof] = {};
  for (int i = 0; i < kInertiaDof; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = a[i * kInertiaDof + j];
      for (int k = 0; k < j; ++k) {
        sum -= factor[i * kInertiaDof + k] * factor[j * kInertiaDof + k];
      }
      if (i == j) {
        if (!(sum > 0.0)) {
          return false;
        }
        factor[i * kInertiaDof + i] = std::sqrt(sum);
      } else {
        factor[i * kInertiaDof + j] = sum / factor[j * kInertiaDof + j];
      }
    }
  }
  return true;
}

}  // namespace

StatusOr<RigidBodyMassMatrix> ComputeRigidBodyMassMatrix(
    const parameters::MassInertia& mass_inertia,
    const parameters::Vec3& center_of_gravity_m) {
  const double mass = mass_inertia.mass_kg;
  if (!std::isfinite(mass)) {
    return Reject(kMassNonFiniteMessage);
  }
  if (!(mass > 0.0)) {
    return Reject(kMassNotPositiveMessage);
  }

  const std::array<double, kInertiaDof * kInertiaDof>& inertia =
      mass_inertia.inertia_about_center_of_gravity_kg_m2;
  for (double entry : inertia) {
    if (!std::isfinite(entry)) {
      return Reject(kInertiaNonFiniteMessage);
    }
  }
  for (int row = 0; row < kInertiaDof; ++row) {
    for (int col = row + 1; col < kInertiaDof; ++col) {
      const double upper = inertia[row * kInertiaDof + col];
      const double lower = inertia[col * kInertiaDof + row];
      if (std::abs(upper - lower) > kMatrixSymmetryTolerance) {
        return Reject(kInertiaAsymmetricMessage);
      }
    }
  }

  // Positive definiteness uses the upper triangle, matching
  // ValidateMarineModel. The mass matrix itself uses the stored entries.
  double symmetric_inertia[kInertiaDof * kInertiaDof] = {};
  for (int row = 0; row < kInertiaDof; ++row) {
    symmetric_inertia[row * kInertiaDof + row] =
        inertia[row * kInertiaDof + row];
    for (int col = row + 1; col < kInertiaDof; ++col) {
      const double upper = inertia[row * kInertiaDof + col];
      symmetric_inertia[row * kInertiaDof + col] = upper;
      symmetric_inertia[col * kInertiaDof + row] = upper;
    }
  }
  if (!IsPositiveDefinite3(symmetric_inertia)) {
    return Reject(kInertiaNotPositiveDefiniteMessage);
  }

  const double x = center_of_gravity_m.x;
  const double y = center_of_gravity_m.y;
  const double z = center_of_gravity_m.z;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return Reject(kCenterOfGravityNonFiniteMessage);
  }

  // S(r_g) with S(a) b = a cross b.
  const double skew[kInertiaDof][kInertiaDof] = {
      {0.0, -z, y},
      {z, 0.0, -x},
      {-y, x, 0.0},
  };
  // S(r_g) S(r_g) is symmetric. Mirror the upper triangle so I_b stays
  // symmetric when I_g is symmetric.
  double skew_sq[kInertiaDof][kInertiaDof] = {};
  for (int row = 0; row < kInertiaDof; ++row) {
    for (int col = row; col < kInertiaDof; ++col) {
      double sum = 0.0;
      for (int k = 0; k < kInertiaDof; ++k) {
        sum += skew[row][k] * skew[k][col];
      }
      skew_sq[row][col] = sum;
      skew_sq[col][row] = sum;
    }
  }

  RigidBodyMassMatrix matrix;
  for (int i = 0; i < kInertiaDof; ++i) {
    matrix.coefficients[i * kSpatialDof + i] = mass;
    for (int j = 0; j < kInertiaDof; ++j) {
      const double coupling = mass * skew[i][j];
      // Lower-left block is m S(r_g). Upper-right block is -m S(r_g).
      matrix.coefficients[(kInertiaDof + i) * kSpatialDof + j] = coupling;
      matrix.coefficients[i * kSpatialDof + (kInertiaDof + j)] = -coupling;
      // I_b = I_g - m S(r_g) S(r_g).
      const double inertia_about_origin =
          inertia[i * kInertiaDof + j] - mass * skew_sq[i][j];
      matrix.coefficients[(kInertiaDof + i) * kSpatialDof + (kInertiaDof + j)] =
          inertia_about_origin;
    }
  }

  for (double entry : matrix.coefficients) {
    if (!std::isfinite(entry)) {
      return Reject(kMassMatrixNonFiniteMessage);
    }
  }
  return StatusOr<RigidBodyMassMatrix>::Ok(matrix);
}

}  // namespace intrinsic::vehicle::dynamics
