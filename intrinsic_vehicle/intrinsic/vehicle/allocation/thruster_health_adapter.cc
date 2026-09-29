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

#include "intrinsic/vehicle/allocation/thruster_health_adapter.h"

#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::allocation {
namespace {

using parameters::ThrusterGeometry;
using parameters::ThrusterHealthState;

constexpr std::string_view kEmptyThrustersMessage =
    "thrusters must be non-empty";
constexpr std::string_view kMaskLengthMessage =
    "enabled mask length must equal the thruster count";
constexpr std::string_view kBoundsCountMessage =
    "thrust bounds count must equal the thruster count";
constexpr std::string_view kColumnCountMessage =
    "column count must equal the thruster count";
constexpr std::string_view kDerateNonFiniteMessage =
    "health_derate is not finite";
constexpr std::string_view kUnknownHealthMessage =
    "thruster health is not a known state";
constexpr std::string_view kDerateMismatchMessage =
    "health_derate does not match thruster health";
constexpr std::string_view kThrustLimitNonFiniteMessage =
    "thruster thrust limit is not finite";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";

bool KnownHealth(ThrusterHealthState health) {
  switch (health) {
    case ThrusterHealthState::kNominal:
    case ThrusterHealthState::kDisabled:
    case ThrusterHealthState::kDerated:
    case ThrusterHealthState::kStuckOff:
    case ThrusterHealthState::kFailed:
      return true;
  }
  return false;
}

bool DerateMatchesHealth(ThrusterHealthState health, double derate) {
  switch (health) {
    case ThrusterHealthState::kNominal:
      return derate == 1.0;
    case ThrusterHealthState::kDerated:
      return derate > 0.0 && derate < 1.0;
    case ThrusterHealthState::kDisabled:
    case ThrusterHealthState::kStuckOff:
    case ThrusterHealthState::kFailed:
      return derate == 0.0;
  }
  return false;
}

bool IsNeutral(ThrusterHealthState health) {
  switch (health) {
    case ThrusterHealthState::kDisabled:
    case ThrusterHealthState::kStuckOff:
    case ThrusterHealthState::kFailed:
      return true;
    case ThrusterHealthState::kNominal:
    case ThrusterHealthState::kDerated:
      return false;
  }
  return false;
}

bool ColumnIsFinite(const EffectivenessColumn& column) {
  for (double value : column.components) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool ThrustLimitsFinite(const ThrusterGeometry& thruster) {
  // Efficiency and slew are not inputs. Only the two thrust limits that
  // become the command interval are read.
  return std::isfinite(thruster.max_forward_thrust_n) &&
         std::isfinite(thruster.max_reverse_thrust_n);
}

void ZeroOutputs(std::span<bool> enabled,
                 std::span<ThrustCommandBounds> thrust_bounds,
                 std::span<EffectivenessColumn> columns) {
  for (bool& flag : enabled) {
    flag = false;
  }
  for (ThrustCommandBounds& bounds : thrust_bounds) {
    bounds.min_thrust_n = 0.0;
    bounds.max_thrust_n = 0.0;
  }
  for (EffectivenessColumn& column : columns) {
    column.components.fill(0.0);
  }
}

AllocationStatus Reject(std::span<bool> enabled,
                        std::span<ThrustCommandBounds> thrust_bounds,
                        std::span<EffectivenessColumn> columns,
                        std::string_view message) {
  ZeroOutputs(enabled, thrust_bounds, columns);
  return AllocationStatus::InvalidArgument(message);
}

void ScaleColumn(EffectivenessColumn& column, double derate) {
  for (double& component : column.components) {
    component *= derate;
  }
}

}  // namespace

AllocationStatus ApplyThrusterHealthToAllocationInputs(
    std::span<const ThrusterGeometry> thrusters, std::span<bool> enabled,
    std::span<ThrustCommandBounds> thrust_bounds,
    std::span<EffectivenessColumn> columns) {
  if (thrusters.empty()) {
    return Reject(enabled, thrust_bounds, columns, kEmptyThrustersMessage);
  }
  if (enabled.size() != thrusters.size()) {
    return Reject(enabled, thrust_bounds, columns, kMaskLengthMessage);
  }
  if (thrust_bounds.size() != thrusters.size()) {
    return Reject(enabled, thrust_bounds, columns, kBoundsCountMessage);
  }
  const bool scale_columns = !columns.empty();
  if (scale_columns && columns.size() != thrusters.size()) {
    return Reject(enabled, thrust_bounds, columns, kColumnCountMessage);
  }

  for (const ThrusterGeometry& thruster : thrusters) {
    if (!std::isfinite(thruster.health_derate)) {
      return Reject(enabled, thrust_bounds, columns, kDerateNonFiniteMessage);
    }
    if (!KnownHealth(thruster.health)) {
      return Reject(enabled, thrust_bounds, columns, kUnknownHealthMessage);
    }
    if (!DerateMatchesHealth(thruster.health, thruster.health_derate)) {
      return Reject(enabled, thrust_bounds, columns, kDerateMismatchMessage);
    }
    if (!ThrustLimitsFinite(thruster)) {
      return Reject(enabled, thrust_bounds, columns,
                    kThrustLimitNonFiniteMessage);
    }
  }
  if (scale_columns) {
    for (const EffectivenessColumn& column : columns) {
      if (!ColumnIsFinite(column)) {
        return Reject(enabled, thrust_bounds, columns, kColumnNonFiniteMessage);
      }
    }
  }

  for (std::size_t i = 0; i < thrusters.size(); ++i) {
    const ThrusterGeometry& thruster = thrusters[i];
    if (IsNeutral(thruster.health)) {
      enabled[i] = false;
      thrust_bounds[i].min_thrust_n = 0.0;
      thrust_bounds[i].max_thrust_n = 0.0;
      if (scale_columns) {
        columns[i].components.fill(0.0);
      }
      continue;
    }
    enabled[i] = true;
    // kNominal multiplies by 1. kDerated multiplies by health_derate.
    // Both the interval and the column use that one factor.
    ThrustCommandBounds bounds = ThrustCommandBoundsFromGeometry(thruster);
    bounds.min_thrust_n *= thruster.health_derate;
    bounds.max_thrust_n *= thruster.health_derate;
    thrust_bounds[i] = bounds;
    if (scale_columns) {
      ScaleColumn(columns[i], thruster.health_derate);
    }
  }
  return AllocationStatus::Ok();
}

}  // namespace intrinsic::vehicle::allocation
