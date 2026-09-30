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

#ifndef INTRINSIC_ESTIMATION_SURFACE_POSITION_UPDATE_H_
#define INTRINSIC_ESTIMATION_SURFACE_POSITION_UPDATE_H_

#include <array>
#include <optional>

#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/innovation_gate.h"

// Surfaced horizontal ENU (East, North) position fix update only (#89). The
// caller decides, through SurfaceFixPolicy, whether a fix may be used. This
// leaf never decides or changes any navigation state. No Up, depth, altitude,
// DVL, lever arm, ICON, or hardware. See SURFACE_POSITION_UPDATE.md. Mirrors
// surface_position_update.py.

namespace intrinsic::estimation {

// Caller-supplied policy. Plain value, both flags decided upstream.
struct SurfaceFixPolicy {
  // False (for example submerged) means the fix must not be used.
  bool is_surfaced = false;
  // Stand-in for an SNR / HDOP / fix-type gate already decided upstream.
  bool quality_ok = false;
};

// One surfaced horizontal position sample as plain values (no marine proto).
struct SurfacePositionSample {
  // Measured {East, North} in the local ENU tangent frame, m. Must be finite.
  std::array<double, 2> position_en_m = {0.0, 0.0};
  // Row-major 2x2 measurement covariance, m^2. Must be finite, symmetric
  // within kRSymmetryTol, and Cholesky-capable with pivots > kMinCholeskyPivot.
  std::array<double, 4> R_en = {0.0, 0.0, 0.0, 0.0};
  // Standalone stand-in for MeasurementHealth VALID. Must be true.
  bool valid = false;
};

enum class SurfacePositionUpdateStatus {
  kOkAccept = 0,
  kOkReject,
  kSkippedPolicy,
  kSkippedInvalid,
  kSingular,
  kNonFinite,
};

struct SurfacePositionUpdateResult {
  // True when the gate ran to a decision (including a later kNonFinite).
  bool evaluated = false;
  // True only for kOkAccept.
  bool accepted = false;
  SurfacePositionUpdateStatus status =
      SurfacePositionUpdateStatus::kSkippedInvalid;
  // #84 diagnostics (d^2, threshold, ||nu||, dof = 2). Set iff evaluated.
  std::optional<GateDiagnostics> diagnostics;
  // Set only for kOkAccept. Every other status leaves the caller's x and P
  // untouched, and these stay unset.
  std::optional<EskfNominal> nominal;
  std::optional<EskfCovariance> P;
  // Error-state correction dx = K nu that was injected. Set only for
  // kOkAccept.
  std::optional<EskfError> delta_x;
};

// h(x) = [p_enu[0], p_enu[1]], nu = z.position_en_m - h(x), H = 2x15 with I2 on
// the dp_e / dp_n columns and 0 elsewhere. Gates with GateInnovation (dof = 2),
// then applies a Joseph update and injects dx into the nominal (right attitude
// error). Never modifies its inputs. Computes on temporaries and commits only
// when everything is finite. A policy that is not surfaced or not quality_ok
// skips the update before anything else is read.
SurfacePositionUpdateResult UpdateSurfacePosition(
    const EskfNominal& x, const EskfCovariance& P,
    const SurfacePositionSample& z, const SurfaceFixPolicy& policy,
    double chi2_threshold);

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_SURFACE_POSITION_UPDATE_H_
