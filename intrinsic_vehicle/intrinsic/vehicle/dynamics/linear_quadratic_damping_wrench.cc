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
#include <string_view>

namespace intrinsic::vehicle::dynamics {
namespace {

using parameters::kMatrixSymmetryTolerance;

static_assert(kSpatialDof == parameters::kSpatialDof);
static_assert(kSpatialDof == 6);

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

StatusOr<DampingWrench> Reject(std::string_view message) {
  return StatusOr<DampingWrench>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// -0 compares equal to 0 and is a different bit pattern. Stored zeros use
// +0 so a zero-velocity result is the zero wrench.
double PositiveZero(double value) { return value == 0.0 ? 0.0 : value; }

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

StatusOr<DampingWrench> ComputeLinearQuadraticDampingWrench(
    const parameters::Damping& damping,
    const std::array<double, kSpatialDof>& body_twist) {
  const std::array<double, kSpatialDof * kSpatialDof>& linear =
      damping.linear_coefficients;
  for (double entry : linear) {
    if (!std::isfinite(entry)) {
      return Reject(kLinearNonFiniteMessage);
    }
  }
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = row + 1; col < kSpatialDof; ++col) {
      const double upper = linear[row * kSpatialDof + col];
      const double lower = linear[col * kSpatialDof + row];
      if (std::abs(upper - lower) > kMatrixSymmetryTolerance) {
        return Reject(kLinearAsymmetricMessage);
      }
    }
  }

  // Positive definiteness uses the upper triangle, matching
  // ValidateMarineModel. The wrench uses the stored entries.
  double symmetric[kSpatialDof * kSpatialDof] = {};
  for (int row = 0; row < kSpatialDof; ++row) {
    symmetric[row * kSpatialDof + row] = linear[row * kSpatialDof + row];
    for (int col = row + 1; col < kSpatialDof; ++col) {
      const double upper = linear[row * kSpatialDof + col];
      symmetric[row * kSpatialDof + col] = upper;
      symmetric[col * kSpatialDof + row] = upper;
    }
  }
  if (!IsPositiveDefinite6(symmetric)) {
    return Reject(kLinearNotPositiveDefiniteMessage);
  }

  const std::array<double, kSpatialDof>& quadratic =
      damping.quadratic_coefficients;
  for (double entry : quadratic) {
    if (!std::isfinite(entry)) {
      return Reject(kQuadraticNonFiniteMessage);
    }
  }
  for (double entry : quadratic) {
    if (entry < 0.0) {
      return Reject(kQuadraticNegativeMessage);
    }
  }

  for (double component : body_twist) {
    if (!std::isfinite(component)) {
      return Reject(kBodyTwistNonFiniteMessage);
    }
  }

  // τ_d = -(D_L + D_Q(|ν|)) ν, with D_Q diagonal. No frame conversion.
  DampingWrench wrench;
  for (int row = 0; row < kSpatialDof; ++row) {
    double linear_term = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      linear_term += linear[row * kSpatialDof + col] * body_twist[col];
    }
    const double speed = body_twist[row];
    const double quadratic_term = quadratic[row] * std::abs(speed) * speed;
    wrench.components[row] = PositiveZero(-(linear_term + quadratic_term));
  }

  for (double component : wrench.components) {
    if (!std::isfinite(component)) {
      return Reject(kDampingWrenchNonFiniteMessage);
    }
  }
  return StatusOr<DampingWrench>::Ok(wrench);
}

}  // namespace intrinsic::vehicle::dynamics
