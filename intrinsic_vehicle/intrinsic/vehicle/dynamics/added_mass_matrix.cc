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
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

using parameters::kMatrixSymmetryTolerance;

static_assert(kSpatialDof == parameters::kSpatialDof);

constexpr std::string_view kAddedMassNonFiniteMessage =
    "added_mass.coefficients must be finite";
constexpr std::string_view kAddedMassAsymmetricMessage =
    "added_mass.coefficients differs from the transpose by more than 1e-9";
constexpr std::string_view kAddedMassNotPositiveDefiniteMessage =
    "added_mass.coefficients is not positive definite";
constexpr std::string_view kAddedMassMatrixNonFiniteMessage =
    "added-mass matrix is not finite";

StatusOr<AddedMassMatrix> Reject(std::string_view message) {
  return StatusOr<AddedMassMatrix>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// Unpivoted Cholesky of a symmetric 6x6 row-major matrix. A pivot that is
// not strictly positive fails. No pivot slack, matching ValidateMarineModel.
bool IsPositiveDefinite6(const double* a) {
  double factor[kSpatialDof * kSpatialDof] = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = a[i * kSpatialDof + j];
      for (int k = 0; k < j; ++k) {
        sum -= factor[i * kSpatialDof + k] * factor[j * kSpatialDof + k];
      }
      if (i == j) {
        if (!(sum > 0.0)) {
          return false;
        }
        factor[i * kSpatialDof + i] = std::sqrt(sum);
      } else {
        factor[i * kSpatialDof + j] = sum / factor[j * kSpatialDof + j];
      }
    }
  }
  return true;
}

}  // namespace

StatusOr<AddedMassMatrix> ComputeAddedMassMatrix(
    const parameters::AddedMass& added_mass) {
  const std::array<double, kSpatialDof * kSpatialDof>& coefficients =
      added_mass.coefficients;
  for (double entry : coefficients) {
    if (!std::isfinite(entry)) {
      return Reject(kAddedMassNonFiniteMessage);
    }
  }
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = row + 1; col < kSpatialDof; ++col) {
      const double upper = coefficients[row * kSpatialDof + col];
      const double lower = coefficients[col * kSpatialDof + row];
      if (std::abs(upper - lower) > kMatrixSymmetryTolerance) {
        return Reject(kAddedMassAsymmetricMessage);
      }
    }
  }

  // Positive definiteness uses the upper triangle, matching
  // ValidateMarineModel. The returned matrix uses the stored entries.
  double symmetric[kSpatialDof * kSpatialDof] = {};
  for (int row = 0; row < kSpatialDof; ++row) {
    symmetric[row * kSpatialDof + row] = coefficients[row * kSpatialDof + row];
    for (int col = row + 1; col < kSpatialDof; ++col) {
      const double upper = coefficients[row * kSpatialDof + col];
      symmetric[row * kSpatialDof + col] = upper;
      symmetric[col * kSpatialDof + row] = upper;
    }
  }
  if (!IsPositiveDefinite6(symmetric)) {
    return Reject(kAddedMassNotPositiveDefiniteMessage);
  }

  AddedMassMatrix matrix;
  matrix.coefficients = coefficients;
  for (double entry : matrix.coefficients) {
    if (!std::isfinite(entry)) {
      return Reject(kAddedMassMatrixNonFiniteMessage);
    }
  }
  return StatusOr<AddedMassMatrix>::Ok(matrix);
}

}  // namespace intrinsic::vehicle::dynamics
