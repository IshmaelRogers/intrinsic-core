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

// Unconstrained least-squares thruster allocation.
//
// Solves min ||u||_2 subject to B u = τ when B has full row rank. The
// solution is the Moore-Penrose right inverse
//
//   u = B^T (B B^T)^{-1} τ
//
// B is 6 by N, stored as columns from BuildThrusterEffectivenessMatrix
// or an equivalent span. Column i is thruster i. τ is the requested body
// wrench in surge, sway, heave, roll, pitch, yaw (REP-103: x forward,
// y left, z up). Force rows are newtons. Moment rows are newton-meters.
// Thrust commands are newtons along each column and may be negative.
// N = 6 is square. N > 6 is underdetermined. N < 6 cannot have full row
// rank.
//
// B B^T is factored with unpivoted Cholesky. A diagonal pivot must be
// strictly greater than kCholeskyPivotRelativeTolerance times the largest
// diagonal entry of B B^T. That relative floor is 1e-12. It is above
// roundoff on a singular Gram matrix (about 1e-16 times the diagonal
// scale) and below the six-thruster full-rank pivots (smallest pivot
// over the largest diagonal is about 1e-2). A pivot on or under the
// floor, or a non-finite factor entry, returns kRankDeficient. No
// Tikhonov term, weight, or damping is added. The test is
// scale-invariant: a full-rank matrix and a small multiple of it use the
// same pivot ratios.
//
// The function reads only columns and τ. It does not read thrust bounds,
// slew, efficiency, health, or health_derate. It does not project,
// saturate, scale, or call ICON.
//
// Storage is caller-owned. The function does not allocate. On every
// failure, thrust commands are finite zeros. kInvalidArgument also writes
// a finite-zero residual. kRankDeficient writes the residual wrench
// τ - B u, which equals τ because u is zero. The first defect wins.
// Check order: empty columns, thrust length, non-finite columns in order,
// non-finite wrench, non-finite Gram entry, Cholesky, non-finite solved
// command, non-finite residual. The same inputs produce the same commands.

#ifndef INTRINSIC_VEHICLE_ALLOCATION_LEAST_SQUARES_ALLOCATOR_H_
#define INTRINSIC_VEHICLE_ALLOCATION_LEAST_SQUARES_ALLOCATOR_H_

#include <array>
#include <span>

#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::allocation {

// Relative Cholesky floor for B B^T. See the file comment.
inline constexpr double kCholeskyPivotRelativeTolerance = 1e-12;

// Writes thrust_command_n and residual_wrench (τ - B u).
// thrust_command_n.size() must equal columns.size().
[[nodiscard]] AllocationStatus AllocateUnconstrainedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, parameters::kSpatialDof>& requested_wrench,
    std::span<double> thrust_command_n,
    std::array<double, parameters::kSpatialDof>& residual_wrench);

}  // namespace intrinsic::vehicle::allocation

#endif  // INTRINSIC_VEHICLE_ALLOCATION_LEAST_SQUARES_ALLOCATOR_H_
