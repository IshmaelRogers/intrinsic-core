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

#include "intrinsic/vehicle/allocation/unconstrained_least_squares.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::allocation {
namespace {

using parameters::kSpatialDof;

static_assert(kSpatialDof == 6);

constexpr std::string_view kEmptyColumnsMessage = "columns must be non-empty";
constexpr std::string_view kThrustLengthMessage =
    "thrust length must equal the column count";
constexpr std::string_view kWrenchNonFiniteMessage =
    "body wrench must be finite";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kGramNonFiniteMessage =
    "effectiveness gram matrix is not finite";
constexpr std::string_view kRankDeficientMessage =
    "effectiveness matrix is rank deficient";
constexpr std::string_view kThrustNonFiniteMessage =
    "thrust command is not finite";

void Zero(std::span<double> thrust_n) {
  for (double& command : thrust_n) {
    command = 0.0;
  }
}

AllocationStatus Fail(std::span<double> thrust_n, AllocationStatus status) {
  Zero(thrust_n);
  return status;
}

bool FiniteWrench(const std::array<double, kSpatialDof>& body_wrench) {
  for (double value : body_wrench) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool FiniteColumn(const EffectivenessColumn& column) {
  for (double value : column.components) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

// Unpivoted Cholesky of a 6×6 row-major Gram matrix. `factor` receives
// the lower triangle. A pivot that is not strictly above `pivot_floor`
// is numerical rank loss.
bool CholeskyFullRank(const double* gram, double pivot_floor, double* factor) {
  for (int i = 0; i < kSpatialDof; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = gram[(i * kSpatialDof) + j];
      for (int k = 0; k < j; ++k) {
        sum -= factor[(i * kSpatialDof) + k] * factor[(j * kSpatialDof) + k];
      }
      if (i == j) {
        if (!(sum > pivot_floor)) {
          return false;
        }
        factor[(i * kSpatialDof) + i] = std::sqrt(sum);
      } else {
        factor[(i * kSpatialDof) + j] = sum / factor[(j * kSpatialDof) + j];
      }
    }
  }
  return true;
}

bool SolveCholesky(const double* factor,
                   const std::array<double, kSpatialDof>& body_wrench,
                   std::array<double, kSpatialDof>* y) {
  std::array<double, kSpatialDof> z = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    double sum = body_wrench[i];
    for (int k = 0; k < i; ++k) {
      sum -= factor[(i * kSpatialDof) + k] * z[k];
    }
    z[i] = sum / factor[(i * kSpatialDof) + i];
    if (!std::isfinite(z[i])) {
      return false;
    }
  }
  for (int i = kSpatialDof - 1; i >= 0; --i) {
    double sum = z[i];
    for (int k = i + 1; k < kSpatialDof; ++k) {
      sum -= factor[(k * kSpatialDof) + i] * (*y)[k];
    }
    (*y)[i] = sum / factor[(i * kSpatialDof) + i];
    if (!std::isfinite((*y)[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

AllocationStatus AllocateUnconstrainedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, kSpatialDof>& body_wrench,
    std::span<double> thrust_n) {
  if (columns.empty()) {
    return Fail(thrust_n,
                AllocationStatus::InvalidArgument(kEmptyColumnsMessage));
  }
  if (thrust_n.size() != columns.size()) {
    return Fail(thrust_n,
                AllocationStatus::InvalidArgument(kThrustLengthMessage));
  }
  if (!FiniteWrench(body_wrench)) {
    return Fail(thrust_n,
                AllocationStatus::InvalidArgument(kWrenchNonFiniteMessage));
  }
  for (const EffectivenessColumn& column : columns) {
    if (!FiniteColumn(column)) {
      return Fail(thrust_n,
                  AllocationStatus::InvalidArgument(kColumnNonFiniteMessage));
    }
  }

  double gram[kSpatialDof * kSpatialDof] = {};
  for (const EffectivenessColumn& column : columns) {
    for (int row = 0; row < kSpatialDof; ++row) {
      const double row_value = column.components[row];
      for (int col = 0; col < kSpatialDof; ++col) {
        gram[(row * kSpatialDof) + col] += row_value * column.components[col];
      }
    }
  }
  for (double value : gram) {
    if (!std::isfinite(value)) {
      return Fail(thrust_n,
                  AllocationStatus::InvalidArgument(kGramNonFiniteMessage));
    }
  }

  double max_diagonal = gram[0];
  for (int i = 1; i < kSpatialDof; ++i) {
    const double diagonal = gram[(i * kSpatialDof) + i];
    if (diagonal > max_diagonal) {
      max_diagonal = diagonal;
    }
  }
  const double pivot_floor = kAllocationRankRelativeTolerance * max_diagonal;
  double factor[kSpatialDof * kSpatialDof] = {};
  if (!CholeskyFullRank(gram, pivot_floor, factor)) {
    return Fail(thrust_n,
                AllocationStatus::RankDeficient(kRankDeficientMessage));
  }

  std::array<double, kSpatialDof> y = {};
  if (!SolveCholesky(factor, body_wrench, &y)) {
    return Fail(thrust_n,
                AllocationStatus::InvalidArgument(kThrustNonFiniteMessage));
  }

  for (std::size_t i = 0; i < columns.size(); ++i) {
    double command = 0.0;
    for (int row = 0; row < kSpatialDof; ++row) {
      command += columns[i].components[row] * y[row];
    }
    if (!std::isfinite(command)) {
      return Fail(thrust_n,
                  AllocationStatus::InvalidArgument(kThrustNonFiniteMessage));
    }
    thrust_n[i] = command;
  }
  return AllocationStatus::Ok();
}

}  // namespace intrinsic::vehicle::allocation
