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

// Thruster health adapter for allocation inputs.
//
// Applies ThrusterHealthState and health_derate to the enabled mask,
// the thrust-command bounds, and, when a column span is provided, the
// effectiveness columns. The caller then passes those inputs to
// AllocateBoundedLeastSquares. This function does not solve, clamp, or
// factor B B^T. It does not read efficiency or slew, and it does not
// discover faults or command hardware.
//
// Approved mapping, per thruster:
//
//   kNominal: health_derate is 1. Enabled. Bounds are the geometry
//   interval from ThrustCommandBoundsFromGeometry, times 1. A provided
//   column is multiplied by 1 and stays bit-identical.
//
//   kDerated: health_derate is in (0, 1). Enabled. The geometry command
//   interval and the effectiveness column are both multiplied by that
//   same health_derate. Efficiency is not a second factor.
//
//   kDisabled, kStuckOff, kFailed: health_derate is 0. Disabled. The
//   command interval is the neutral point [0, 0]. A provided column is
//   cleared to +0, which is the column BuildThrusterEffectivenessMatrix
//   writes for enabled == false. The neutral command is 0. No other
//   fixed command is substituted.
//
// Caller order. Build the unscaled body columns first; that builder
// does not read health. Then call this function. An empty column span
// leaves columns untouched and still writes the mask and the bounds.
// A second call can scale columns that were built from the mask. Pass
// each column span once: scaling mutates the current values, while
// bounds are always recomputed from geometry.
//
// Storage is caller-owned. The function does not allocate. enabled
// must address contiguous bool objects. std::vector<bool> is not a
// valid mask.
//
// kInvalidArgument writes finite zeros: enabled false, bounds [+0, +0],
// and, when columns were passed, every column component +0. The first
// defect wins. Check order: empty thruster list, enabled length, bounds
// length, column length (empty or equal to the thruster count), then
// each thruster in order (non-finite health_derate, unknown health,
// health_derate that does not match health, non-finite thrust limit),
// then each provided column in order for a non-finite component. The
// same inputs produce the same outputs.

#ifndef INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_HEALTH_ADAPTER_H_
#define INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_HEALTH_ADAPTER_H_

#include <span>

#include "intrinsic/vehicle/allocation/bounded_allocator.h"
#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::allocation {

// Writes enabled, thrust_bounds, and, when columns is non-empty, the
// health-scaled columns. columns.size() must be 0 or thrusters.size().
// enabled.size() and thrust_bounds.size() must equal thrusters.size().
[[nodiscard]] AllocationStatus ApplyThrusterHealthToAllocationInputs(
    std::span<const parameters::ThrusterGeometry> thrusters,
    std::span<bool> enabled, std::span<ThrustCommandBounds> thrust_bounds,
    std::span<EffectivenessColumn> columns);

}  // namespace intrinsic::vehicle::allocation

#endif  // INTRINSIC_VEHICLE_ALLOCATION_THRUSTER_HEALTH_ADAPTER_H_
