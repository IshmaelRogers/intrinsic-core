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

// Body-frame thruster effectiveness matrix.
//
// Column i of B is the body wrench produced by one newton of thrust along
// ThrusterGeometry::direction_body. Force is that unit axis, in newtons
// per newton. Moment is position_m × direction_body, in newton-meters per
// newton, about the body origin. Row order is surge, sway, heave, roll,
// pitch, yaw (REP-103: x forward, y left, z up). B is 6 by N, with
// B[row, col] = columns[col][row].
//
// ThrusterGeometry::frame_id must be "body". This function does not
// convert world_enu or world_ned poses into the body frame.
//
// enabled[i] == false writes a zero column. The column is excluded from
// the wrench map and is not deleted, so column i stays thruster i. The
// mask is the only exclusion input. Health, health_derate, efficiency,
// slew, and thrust bounds are not read and do not scale B.
//
// The function does not solve for thrust, saturate, or report a residual.
// Storage is caller-owned. The function does not allocate. A failed call
// returns kInvalidArgument and writes a finite zero into every output
// column. The first defect wins. Check order is: empty thruster list,
// mask length, column count, then each thruster in order (frame id,
// finite position, finite direction, zero axis, unit direction). A
// non-finite cross product is checked while the columns are written.
// The same inputs produce the same columns.

#ifndef INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_EFFECTIVENESS_MATRIX_H_
#define INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_EFFECTIVENESS_MATRIX_H_

#include <array>
#include <span>
#include <string_view>

#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::allocation {

// Status codes. kOk is the only success code.
// kInvalidArgument is a bad size or a non-finite input.
// kRankDeficient is returned by AllocateUnconstrainedLeastSquares when
// B B^T is singular or its unpivoted Cholesky factor is not safe.
enum class AllocationErrorCode {
  kOk = 0,
  kInvalidArgument = 1,
  kRankDeficient = 2,
};

struct AllocationStatus {
  AllocationErrorCode code = AllocationErrorCode::kOk;
  std::string_view message;

  [[nodiscard]] bool ok() const { return code == AllocationErrorCode::kOk; }

  [[nodiscard]] static AllocationStatus Ok() { return {}; }

  [[nodiscard]] static AllocationStatus InvalidArgument(
      std::string_view message) {
    return {AllocationErrorCode::kInvalidArgument, message};
  }

  [[nodiscard]] static AllocationStatus RankDeficient(
      std::string_view message) {
    return {AllocationErrorCode::kRankDeficient, message};
  }
};

// One column of B. components[kSurge] through components[kYaw].
struct EffectivenessColumn {
  std::array<double, parameters::kSpatialDof> components = {};
};

// Writes B into columns. columns.size() and enabled.size() must equal
// thrusters.size(). enabled must address contiguous bool objects.
// std::vector<bool> is not a valid mask.
[[nodiscard]] AllocationStatus BuildThrusterEffectivenessMatrix(
    std::span<const parameters::ThrusterGeometry> thrusters,
    std::span<const bool> enabled, std::span<EffectivenessColumn> columns);

}  // namespace intrinsic::vehicle::allocation

#endif  // INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_EFFECTIVENESS_MATRIX_H_
