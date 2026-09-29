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

#include "intrinsic/vehicle/allocation/least_squares_allocator.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::allocation {
namespace {

using parameters::kSpatialDof;

static_assert(kSpatialDof == 6);

constexpr std::string_view kEmptyColumnsMessage =
    "effectiveness columns must be non-empty";
constexpr std::string_view kThrustCountMessage =
    "thrust command count must equal the column count";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kWrenchNonFiniteMessage =
    "requested wrench must be finite";
constexpr std::string_view kGramNonFiniteMessage =
    "Gram matrix B B^T is not finite";
constexpr std::string_view kRankDeficientMessage =
    "B B^T is singular; Cholesky pivot is not above the relative tolerance";
constexpr std::string_view kThrustNonFiniteMessage =
    "thrust command is not finite";
constexpr std::string_view kResidualNonFiniteMessage =
    "allocation residual is not finite";

using Wrench = std::array<double, kSpatialDof>;
using Matrix = std::array<double, kSpatialDof * kSpatialDof>;

void ZeroThrust(std::span<double> thrust) {
  for (double& value : thrust) {
    value = 0.0;
  }
}

void ZeroResidual(Wrench& residual) { residual.fill(0.0); }

AllocationStatus RejectInvalid(std::span<double> thrust, Wrench& residual,
                               std::string_view message) {
  ZeroThrust(thrust);
  ZeroResidual(residual);
  return AllocationStatus::InvalidArgument(message);
}

bool ColumnIsFinite(const EffectivenessColumn& column) {
  for (double value : column.components) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool WrenchIsFinite(const Wrench& wrench) {
  for (double value : wrench) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

// Lower triangle of G = B B^T, including the diagonal. Upper entries stay
// zero. Cholesky reads only j <= i.
void AccumulateGram(std::span<const EffectivenessColumn> columns,
                    Matrix& gram) {
  gram.fill(0.0);
  for (const EffectivenessColumn& column : columns) {
    for (int row = 0; row < kSpatialDof; ++row) {
      for (int col = 0; col <= row; ++col) {
        gram[row * kSpatialDof + col] +=
            column.components[row] * column.components[col];
      }
    }
  }
}

bool GramIsFinite(const Matrix& gram) {
  for (int row = 0; row < kSpatialDof; ++row) {
    for (int col = 0; col <= row; ++col) {
      if (!std::isfinite(gram[row * kSpatialDof + col])) {
        return false;
      }
    }
  }
  return true;
}

double GramDiagonalScale(const Matrix& gram) {
  double scale = 0.0;
  for (int i = 0; i < kSpatialDof; ++i) {
    const double diagonal = gram[i * kSpatialDof + i];
    if (diagonal > scale) {
      scale = diagonal;
    }
  }
  return scale;
}

// Unpivoted lower Cholesky. A pivot on or under the relative floor, or a
// non-finite factor entry, fails. No Tikhonov term is added.
bool CholeskyLower(const Matrix& gram, Matrix& factor) {
  factor.fill(0.0);
  const double pivot_floor =
      kCholeskyPivotRelativeTolerance * GramDiagonalScale(gram);
  for (int i = 0; i < kSpatialDof; ++i) {
    for (int j = 0; j <= i; ++j) {
      double sum = gram[i * kSpatialDof + j];
      for (int k = 0; k < j; ++k) {
        sum -= factor[i * kSpatialDof + k] * factor[j * kSpatialDof + k];
      }
      if (i == j) {
        if (!(sum > pivot_floor) || !std::isfinite(sum)) {
          return false;
        }
        factor[i * kSpatialDof + i] = std::sqrt(sum);
        if (!std::isfinite(factor[i * kSpatialDof + i])) {
          return false;
        }
      } else {
        factor[i * kSpatialDof + j] = sum / factor[j * kSpatialDof + j];
        if (!std::isfinite(factor[i * kSpatialDof + j])) {
          return false;
        }
      }
    }
  }
  return true;
}

// Solves L L^T y = τ. Returns false when a multiplier is non-finite.
bool SolveCholesky(const Matrix& factor, const Wrench& wrench, Wrench& y) {
  Wrench z = {};
  for (int i = 0; i < kSpatialDof; ++i) {
    double sum = wrench[i];
    for (int k = 0; k < i; ++k) {
      sum -= factor[i * kSpatialDof + k] * z[k];
    }
    z[i] = sum / factor[i * kSpatialDof + i];
    if (!std::isfinite(z[i])) {
      return false;
    }
  }
  for (int i = kSpatialDof - 1; i >= 0; --i) {
    double sum = z[i];
    for (int k = i + 1; k < kSpatialDof; ++k) {
      sum -= factor[k * kSpatialDof + i] * y[k];
    }
    y[i] = sum / factor[i * kSpatialDof + i];
    if (!std::isfinite(y[i])) {
      return false;
    }
  }
  return true;
}

void WriteThrust(std::span<const EffectivenessColumn> columns, const Wrench& y,
                 std::span<double> thrust) {
  for (std::size_t i = 0; i < columns.size(); ++i) {
    double command = 0.0;
    for (int row = 0; row < kSpatialDof; ++row) {
      command += columns[i].components[row] * y[row];
    }
    thrust[i] = command;
  }
}

bool ThrustIsFinite(std::span<const double> thrust) {
  for (double value : thrust) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

// residual = τ - B u. Column-major accumulation order is part of the
// bit-stable result.
void WriteResidual(std::span<const EffectivenessColumn> columns,
                   std::span<const double> thrust, const Wrench& wrench,
                   Wrench& residual) {
  for (int row = 0; row < kSpatialDof; ++row) {
    double produced = 0.0;
    for (std::size_t col = 0; col < columns.size(); ++col) {
      produced += columns[col].components[row] * thrust[col];
    }
    residual[row] = wrench[row] - produced;
  }
}

}  // namespace

AllocationStatus AllocateUnconstrainedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, kSpatialDof>& requested_wrench,
    std::span<double> thrust_command_n,
    std::array<double, kSpatialDof>& residual_wrench) {
  if (columns.empty()) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kEmptyColumnsMessage);
  }
  if (thrust_command_n.size() != columns.size()) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kThrustCountMessage);
  }
  for (const EffectivenessColumn& column : columns) {
    if (!ColumnIsFinite(column)) {
      return RejectInvalid(thrust_command_n, residual_wrench,
                           kColumnNonFiniteMessage);
    }
  }
  if (!WrenchIsFinite(requested_wrench)) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kWrenchNonFiniteMessage);
  }

  Matrix gram = {};
  AccumulateGram(columns, gram);
  if (!GramIsFinite(gram)) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kGramNonFiniteMessage);
  }

  Matrix factor = {};
  if (!CholeskyLower(gram, factor)) {
    ZeroThrust(thrust_command_n);
    WriteResidual(columns, thrust_command_n, requested_wrench, residual_wrench);
    return AllocationStatus::RankDeficient(kRankDeficientMessage);
  }

  Wrench y = {};
  if (!SolveCholesky(factor, requested_wrench, y)) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kThrustNonFiniteMessage);
  }
  WriteThrust(columns, y, thrust_command_n);
  if (!ThrustIsFinite(thrust_command_n)) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kThrustNonFiniteMessage);
  }
  WriteResidual(columns, thrust_command_n, requested_wrench, residual_wrench);
  if (!WrenchIsFinite(residual_wrench)) {
    return RejectInvalid(thrust_command_n, residual_wrench,
                         kResidualNonFiniteMessage);
  }
  return AllocationStatus::Ok();
}

}  // namespace intrinsic::vehicle::allocation
