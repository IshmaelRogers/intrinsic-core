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

#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"

#include <cmath>
#include <cstddef>
#include <string_view>

namespace intrinsic::vehicle::allocation {
namespace {

using parameters::kBodyFrameId;
using parameters::kHeave;
using parameters::kPitch;
using parameters::kRoll;
using parameters::kSurge;
using parameters::kSway;
using parameters::kUnitVectorTolerance;
using parameters::kYaw;
using parameters::ThrusterGeometry;
using parameters::Vec3;

static_assert(parameters::kSpatialDof == 6);

constexpr std::string_view kEmptyThrustersMessage =
    "thrusters must be non-empty";
constexpr std::string_view kMaskLengthMessage =
    "enabled mask length must equal the thruster count";
constexpr std::string_view kColumnCountMessage =
    "column count must equal the thruster count";
constexpr std::string_view kFrameMessage = "thruster frame_id must be body";
constexpr std::string_view kPositionNonFiniteMessage =
    "thruster position_m must be finite";
constexpr std::string_view kDirectionNonFiniteMessage =
    "thruster direction_body must be finite";
constexpr std::string_view kZeroAxisMessage =
    "thruster direction_body must be a non-zero unit vector";
constexpr std::string_view kNotUnitMessage =
    "thruster direction_body must be a unit vector";
constexpr std::string_view kColumnNonFiniteMessage =
    "effectiveness column is not finite";

AllocationStatus Reject(std::span<EffectivenessColumn> columns,
                        std::string_view message) {
  for (EffectivenessColumn& column : columns) {
    column.components.fill(0.0);
  }
  return AllocationStatus::InvalidArgument(message);
}

bool IsFinite(const Vec3& vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y) &&
         std::isfinite(vector.z);
}

double Norm(const Vec3& vector) {
  return std::sqrt((vector.x * vector.x) + (vector.y * vector.y) +
                   (vector.z * vector.z));
}

// Unit thrust along direction_body. Moment is r × u about the body origin.
EffectivenessColumn UnitThrustColumn(const ThrusterGeometry& thruster) {
  const double rx = thruster.position_m.x;
  const double ry = thruster.position_m.y;
  const double rz = thruster.position_m.z;
  const double ux = thruster.direction_body.x;
  const double uy = thruster.direction_body.y;
  const double uz = thruster.direction_body.z;
  EffectivenessColumn column;
  column.components[kSurge] = ux;
  column.components[kSway] = uy;
  column.components[kHeave] = uz;
  column.components[kRoll] = (ry * uz) - (rz * uy);
  column.components[kPitch] = (rz * ux) - (rx * uz);
  column.components[kYaw] = (rx * uy) - (ry * ux);
  return column;
}

bool ColumnIsFinite(const EffectivenessColumn& column) {
  for (double value : column.components) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

AllocationStatus BuildThrusterEffectivenessMatrix(
    std::span<const ThrusterGeometry> thrusters, std::span<const bool> enabled,
    std::span<EffectivenessColumn> columns) {
  if (thrusters.empty()) {
    return Reject(columns, kEmptyThrustersMessage);
  }
  if (enabled.size() != thrusters.size()) {
    return Reject(columns, kMaskLengthMessage);
  }
  if (columns.size() != thrusters.size()) {
    return Reject(columns, kColumnCountMessage);
  }

  for (const ThrusterGeometry& thruster : thrusters) {
    if (std::string_view(thruster.frame_id) != kBodyFrameId) {
      return Reject(columns, kFrameMessage);
    }
    if (!IsFinite(thruster.position_m)) {
      return Reject(columns, kPositionNonFiniteMessage);
    }
    if (!IsFinite(thruster.direction_body)) {
      return Reject(columns, kDirectionNonFiniteMessage);
    }
    const double norm = Norm(thruster.direction_body);
    if (!std::isfinite(norm)) {
      return Reject(columns, kDirectionNonFiniteMessage);
    }
    if (norm <= kUnitVectorTolerance) {
      return Reject(columns, kZeroAxisMessage);
    }
    if (std::abs(norm - 1.0) > kUnitVectorTolerance) {
      return Reject(columns, kNotUnitMessage);
    }
  }

  for (std::size_t i = 0; i < thrusters.size(); ++i) {
    EffectivenessColumn column;
    if (enabled[i]) {
      column = UnitThrustColumn(thrusters[i]);
      if (!ColumnIsFinite(column)) {
        return Reject(columns, kColumnNonFiniteMessage);
      }
    }
    columns[i] = column;
  }
  return AllocationStatus::Ok();
}

}  // namespace intrinsic::vehicle::allocation
