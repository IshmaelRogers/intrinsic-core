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

// Unconstrained minimum-norm least-squares thrust allocation.
//
// Solves B u = τ for the thrust command u when the body-frame
// effectiveness matrix B has full row rank. B is 6 by N, stored as
// columns from BuildThrusterEffectivenessMatrix. τ is the requested
// body wrench in surge, sway, heave, roll, pitch, yaw. u[i] is the
// thrust command for column i, in newtons. This function does not
// read actuator bounds, slew, efficiency, or health, and it does not
// scale or project u. Those layers are later issues.
//
// Method: left normal equations. G = B B^T is 6 by 6. Unpivoted
// Cholesky factors G when every pivot is strictly above
// kAllocationRankRelativeTolerance times the largest diagonal entry
// of G. Then G y = τ and u = B^T y. When rank(B) = 6 this is the
// Moore-Penrose solution, the unique minimum-Euclidean-norm u with
// B u = τ. Normal equations square the condition number of B. That
// is accepted here for small 6×N thruster maps. Replacing it with a
// thin QR of B^T changes the numerics and needs senior review.
//
// Rank: a pivot at or below that floor, including a non-positive
// pivot, returns AllocationErrorCode::kRankDeficient. Underactuated
// maps take this path. The function does not return a
// minimum-residual thrust for a rank-deficient B, even when τ lies
// in the column space. Residual and saturation reporting belong to a
// later bound-projection layer.
//
// kAllocationWrenchTolerance is the absolute reconstruction tolerance
// tests apply to each component of B u - τ, in newtons and
// newton-meters, on the documented full-rank fixtures.
//
// Storage is caller-owned. The function does not allocate. Work is
// linear in the column count plus a fixed 6×6 factorization. A failed
// call writes a finite zero into every thrust output. The first
// defect wins. Check order is: empty columns, thrust length, finite
// wrench, finite columns, finite G, numerical rank, finite thrust.
// The same inputs produce the same thrust.

#ifndef INTRINSIC_VEHICLE_ALLOCATION_UNCONSTRAINED_LEAST_SQUARES_H_
#define INTRINSIC_VEHICLE_ALLOCATION_UNCONSTRAINED_LEAST_SQUARES_H_

#include <array>
#include <span>

#include "intrinsic/vehicle/allocation/thruster_effectiveness_matrix.h"
#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::allocation {

// Relative floor for Cholesky pivots of G = B B^T, compared with the
// largest diagonal entry of G. Senior controls review owns this value.
inline constexpr double kAllocationRankRelativeTolerance = 1e-10;

// Absolute |B u - τ| tolerance for full-rank fixture checks. Force
// rows are newtons. Moment rows are newton-meters.
inline constexpr double kAllocationWrenchTolerance = 1e-8;

// columns and thrust_n are caller storage. thrust_n.size() must equal
// columns.size(). body_wrench is surge through yaw.
[[nodiscard]] AllocationStatus AllocateUnconstrainedLeastSquares(
    std::span<const EffectivenessColumn> columns,
    const std::array<double, parameters::kSpatialDof>& body_wrench,
    std::span<double> thrust_n);

}  // namespace intrinsic::vehicle::allocation

#endif  // INTRINSIC_VEHICLE_ALLOCATION_UNCONSTRAINED_LEAST_SQUARES_H_
