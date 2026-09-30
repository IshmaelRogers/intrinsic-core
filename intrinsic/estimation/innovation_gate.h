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

#ifndef INTRINSIC_ESTIMATION_INNOVATION_GATE_H_
#define INTRINSIC_ESTIMATION_INNOVATION_GATE_H_

#include <optional>
#include <vector>

#include "intrinsic/estimation/eskf_state.h"

// Reusable Mahalanobis innovation gate only (#84). No sensor residual, Kalman
// gain, Joseph form, state or covariance write. The caller supplies the chi2
// threshold. See INNOVATION_GATE.md. Mirrors innovation_gate.py.

namespace intrinsic::estimation {

enum class GateStatus {
  kOkAccept = 0,
  kOkReject,
  kInvalidDim,
  kInvalidP,
  kInvalidR,
  kInvalidNu,
  kInvalidThreshold,
  kSingularS,
  kNonFinite,
};

// Set only when the gate evaluated (kOkAccept or kOkReject).
struct GateDiagnostics {
  // d^2 = nu^T S^-1 nu. A tiny negative value is stored as 0.0.
  double mahalanobis_sq = 0.0;
  // Echo of the caller's chi2 threshold.
  double threshold = 0.0;
  // ||nu||_2.
  double innovation_norm = 0.0;
  // Degrees of freedom, m = nu.size().
  int dof = 0;
  // Symmetrized innovation covariance S_s, row-major m x m.
  std::vector<double> S;
};

struct GateResult {
  // True for kOkAccept and kOkReject. Equal to `evaluated`.
  bool ok = false;
  // True when the gate ran to a decision (accept or reject).
  bool evaluated = false;
  // True only for kOkAccept.
  bool accepted = false;
  GateStatus status = GateStatus::kInvalidDim;
  // Unset on every error status. No partial accept or reject.
  std::optional<GateDiagnostics> diagnostics;
};

// Absolute symmetry tolerance for R, and the Cholesky pivot floor for S.
inline constexpr double kRSymmetryTol = 1e-12;
inline constexpr double kMinCholeskyPivot = 1e-12;
// d^2 below this (more negative) is an unusable result, not a clamp.
inline constexpr double kMinMahalanobisSq = -1e-9;

// m = nu.size() >= 1. `H` is row-major m x kCovDim, `R` is row-major m x m.
// Computes S = H sym(P) H^T + R, d^2 = nu^T sym(S)^-1 nu by Cholesky, and
// accepts iff d^2 <= chi2_threshold. Never modifies its inputs.
GateResult GateInnovation(const std::vector<double>& nu,
                          const std::vector<double>& H, const EskfCovariance& P,
                          const std::vector<double>& R, double chi2_threshold);

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_INNOVATION_GATE_H_
