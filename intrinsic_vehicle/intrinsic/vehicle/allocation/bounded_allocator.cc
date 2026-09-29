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

#include "intrinsic/vehicle/allocation/bounded_allocator.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

#include "intrinsic/vehicle/allocation/least_squares_allocator.h"

namespace intrinsic::vehicle::allocation {
namespace {

using parameters::kSpatialDof;

static_assert(kSpatialDof == 6);

constexpr std::string_view kEmptyColumnsMessage =
    "effectiveness columns must be non-empty";
constexpr std::string_view kThrustCountMessage =
    "thrust command count must equal the column count";
constexpr std::string_view kBoundsCountMessage =
    "thrust bounds count must equal the column count";
constexpr std::string_view kSaturatedCountMessage =
    "saturation flag count must equal the column count";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";
constexpr std::string_view kWrenchNonFiniteMessage =
    "requested wrench must be finite";
constexpr std::string_view kBoundNonFiniteMessage =
    "thrust bound is not finite";
constexpr std::string_view kInvertedBoundsMessage =
    "thrust bound minimum exceeds maximum";
constexpr std::string_view kWrenchAfterClampMessage =
    "bounded allocation wrench is not finite";
constexpr std::string_view kResidualNormNonFiniteMessage =
    "allocation residual norm is not finite";

using Wrench = std::array<double, kSpatialDof>;

void ZeroThrust(std::span<double> thrust) {
  for (double& value : thrust) {
    value = 0.0;
  }
}

void ZeroWrench(Wrench& wrench) { wrench.fill(0.0); }

void ZeroSaturated(std::span<bool> saturated) {
  for (bool& flag : saturated) {
    flag = false;
  }
}

void ZeroOutputs(std::span<double> thrust, Wrench& achieved, Wrench& residual,
                 std::span<bool> saturated, double& residual_l2_norm) {
  ZeroThrust(thrust);
  ZeroWrench(achieved);
  ZeroWrench(residual);
  ZeroSaturated(saturated);
  residual_l2_norm = 0.0;
}

AllocationStatus RejectInvalid(std::span<double> thrust, Wrench& achieved,
                               Wrench& residual, std::span<bool> saturated,
                               double& residual_l2_norm,
                               std::string_view message) {
  ZeroOutputs(thrust, achieved, residual, saturated, residual_l2_norm);
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

bool BoundIsFinite(const ThrustCommandBounds& bounds) {
  return std::isfinite(bounds.min_thrust_n) &&
         std::isfinite(bounds.max_thrust_n);
}

// achieved = B u and residual = τ - B u. Column order matches the
// unconstrained residual so an unclamped command keeps the same bits.
void WriteAchievedAndResidual(std::span<const EffectivenessColumn> columns,
                              std::span<const double> thrust,
                              const Wrench& wrench, Wrench& achieved,
                              Wrench& residual) {
  for (int row = 0; row < kSpatialDof; ++row) {
    double produced = 0.0;
    for (std::size_t col = 0; col < columns.size(); ++col) {
      produced += columns[col].components[row] * thrust[col];
    }
    achieved[row] = produced;
    residual[row] = wrench[row] - produced;
  }
}

// Sequential sum of squares, then sqrt. Row order is part of the
// bit-stable result.
double ResidualL2Norm(const Wrench& residual) {
  double sum_of_squares = 0.0;
  for (double value : residual) {
    sum_of_squares += value * value;
  }
  return std::sqrt(sum_of_squares);
}

void ClampThrust(std::span<const ThrustCommandBounds> thrust_bounds,
                 std::span<double> thrust, std::span<bool> saturated) {
  for (std::size_t i = 0; i < thrust.size(); ++i) {
    const double lower = thrust_bounds[i].min_thrust_n;
    const double upper = thrust_bounds[i].max_thrust_n;
    double command = thrust[i];
    if (command < lower) {
      command = lower;
    } else if (command > upper) {
      command = upper;
    }
    thrust[i] = command;
    saturated[i] = (command == lower) || (command == upper);
  }
}

}  // namespace

AllocationStatus AllocateBoundedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, kSpatialDof>& requested_wrench,
    std::span<const ThrustCommandBounds> thrust_bounds,
    std::span<double> thrust_command_n, Wrench& achieved_wrench,
    Wrench& residual_wrench, std::span<bool> saturated,
    double& residual_l2_norm) {
  if (columns.empty()) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kEmptyColumnsMessage);
  }
  if (thrust_command_n.size() != columns.size()) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kThrustCountMessage);
  }
  if (thrust_bounds.size() != columns.size()) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kBoundsCountMessage);
  }
  if (saturated.size() != columns.size()) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kSaturatedCountMessage);
  }
  for (const EffectivenessColumn& column : columns) {
    if (!ColumnIsFinite(column)) {
      return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                           saturated, residual_l2_norm,
                           kColumnNonFiniteMessage);
    }
  }
  if (!WrenchIsFinite(requested_wrench)) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kWrenchNonFiniteMessage);
  }
  for (const ThrustCommandBounds& bounds : thrust_bounds) {
    if (!BoundIsFinite(bounds)) {
      return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                           saturated, residual_l2_norm, kBoundNonFiniteMessage);
    }
  }
  for (const ThrustCommandBounds& bounds : thrust_bounds) {
    if (bounds.min_thrust_n > bounds.max_thrust_n) {
      return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                           saturated, residual_l2_norm, kInvertedBoundsMessage);
    }
  }

  const AllocationStatus unconstrained = AllocateUnconstrainedLeastSquares(
      columns, requested_wrench, thrust_command_n, residual_wrench);
  if (!unconstrained.ok()) {
    if (unconstrained.code != AllocationErrorCode::kRankDeficient) {
      ZeroOutputs(thrust_command_n, achieved_wrench, residual_wrench, saturated,
                  residual_l2_norm);
      return unconstrained;
    }
    // Fail closed with the solver's zero command. Do not clamp.
    ZeroSaturated(saturated);
    WriteAchievedAndResidual(columns, thrust_command_n, requested_wrench,
                             achieved_wrench, residual_wrench);
    residual_l2_norm = ResidualL2Norm(residual_wrench);
    if (!WrenchIsFinite(achieved_wrench) || !WrenchIsFinite(residual_wrench)) {
      return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                           saturated, residual_l2_norm,
                           kWrenchAfterClampMessage);
    }
    if (!std::isfinite(residual_l2_norm)) {
      return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                           saturated, residual_l2_norm,
                           kResidualNormNonFiniteMessage);
    }
    return unconstrained;
  }

  ClampThrust(thrust_bounds, thrust_command_n, saturated);
  WriteAchievedAndResidual(columns, thrust_command_n, requested_wrench,
                           achieved_wrench, residual_wrench);
  if (!WrenchIsFinite(achieved_wrench) || !WrenchIsFinite(residual_wrench)) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm, kWrenchAfterClampMessage);
  }
  residual_l2_norm = ResidualL2Norm(residual_wrench);
  if (!std::isfinite(residual_l2_norm)) {
    return RejectInvalid(thrust_command_n, achieved_wrench, residual_wrench,
                         saturated, residual_l2_norm,
                         kResidualNormNonFiniteMessage);
  }
  return AllocationStatus::Ok();
}

}  // namespace intrinsic::vehicle::allocation
