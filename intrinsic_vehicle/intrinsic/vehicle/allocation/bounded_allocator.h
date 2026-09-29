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

// Bounded least-squares thruster allocation.
//
// Projects the unconstrained minimum-norm command from
// AllocateUnconstrainedLeastSquares onto inclusive per-actuator thrust
// bounds, then reports the wrench that bounded command produces.
//
//   u = B^T (B B^T)^{-1} τ
//   u_i = clamp(u_i, min_i, max_i)
//   achieved = B u
//   residual = τ - achieved
//   residual_l2_norm = ||residual||_2
//
// The clamp is independent per actuator. A command below the minimum
// becomes the minimum. A command above the maximum becomes the maximum.
// Every other command stays at the unconstrained value. Surplus wrench
// is not moved onto unsaturated actuators. There is no quadratic
// program, no failure reallocation, and no slew limit. Health,
// health_derate, efficiency, and slew are not inputs.
//
// ThrustCommandBounds is a command interval in newtons, not a pair of
// magnitudes. ThrustCommandBoundsFromGeometry reads only
// max_reverse_thrust_n and max_forward_thrust_n:
//
//   min_thrust_n = -max_reverse_thrust_n
//   max_thrust_n = max_forward_thrust_n
//
// An actuator is saturated when the stored command equals min_thrust_n
// or max_thrust_n. Equality is exact. A command strictly between the
// bounds is not saturated. A command the unconstrained solve already
// placed on a bound is saturated. Saturation is a diagnostic. It is not
// an error: a successful clamp returns kOk.
//
// Storage is caller-owned. The function does not allocate. The thrust
// command span, the bounds span, and the saturation span must have the
// same length as columns. saturated must address contiguous bool
// objects. std::vector<bool> is not a valid mask.
//
// kInvalidArgument writes finite zeros: thrust commands 0, achieved and
// residual wrenches 0, saturation flags false, and residual norm +0.
// The first defect wins. Check order: empty columns, thrust length,
// bounds length, saturation length, non-finite columns in order,
// non-finite wrench, non-finite bounds in order, minimum greater than
// maximum in order, then the unconstrained solver. A non-finite Gram
// matrix, thrust, or residual reported by that solver is propagated as
// kInvalidArgument and those outputs are cleared. A non-finite achieved
// wrench, residual wrench, or residual norm after the clamp is also
// kInvalidArgument and clears every output.
//
// kRankDeficient is propagated from AllocateUnconstrainedLeastSquares.
// This function does not invent another solver. The thrust commands stay
// the finite zeros from that call and are not clamped, so a bound
// interval that excludes 0 can contain none of them. Achieved wrench is
// the zero wrench. Residual wrench is τ - B u, which equals τ.
// Saturation flags are false because no projection ran: a zero that
// happens to equal a bound is not marked saturated. The residual norm is
// ||τ||_2 when that value is finite. A non-finite norm returns
// kInvalidArgument and finite zeros instead of kRankDeficient.
//
// When the unconstrained command already lies inside every interval, the
// bounded command matches it. Achieved plus residual reconstructs τ
// within an absolute tolerance of 1e-9, the same tolerance as the
// unconstrained solver. After a clamp, every command lies in its
// configured interval, and the stored residual is τ - B u. The same
// inputs produce the same outputs. ICON is not called.

#ifndef INTRINSIC_VEHICLE_ALLOCATION_BOUNDED_ALLOCATOR_H_
#define INTRINSIC_VEHICLE_ALLOCATION_BOUNDED_ALLOCATOR_H_

#include <array>
#include <span>

#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::allocation {

// Inclusive thrust-command interval, in newtons. min_thrust_n may be
// negative. max_thrust_n may be positive. min_thrust_n == max_thrust_n
// is a fixed command and is valid.
struct ThrustCommandBounds {
  double min_thrust_n = 0;
  double max_thrust_n = 0;
};

// Reads only the two thrust limits. Health, derate, efficiency, slew,
// pose, and axis are ignored.
[[nodiscard]] inline ThrustCommandBounds ThrustCommandBoundsFromGeometry(
    const parameters::ThrusterGeometry& thruster) {
  ThrustCommandBounds bounds;
  bounds.min_thrust_n = -thruster.max_reverse_thrust_n;
  bounds.max_thrust_n = thruster.max_forward_thrust_n;
  return bounds;
}

// Writes the bounded command, achieved wrench, residual wrench,
// per-actuator saturation flags, and residual L2 norm.
[[nodiscard]] AllocationStatus AllocateBoundedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, parameters::kSpatialDof>& requested_wrench,
    std::span<const ThrustCommandBounds> thrust_bounds,
    std::span<double> thrust_command_n,
    std::array<double, parameters::kSpatialDof>& achieved_wrench,
    std::array<double, parameters::kSpatialDof>& residual_wrench,
    std::span<bool> saturated, double& residual_l2_norm);

}  // namespace intrinsic::vehicle::allocation

#endif  // INTRINSIC_VEHICLE_ALLOCATION_BOUNDED_ALLOCATOR_H_
