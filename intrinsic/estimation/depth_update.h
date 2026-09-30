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

#ifndef INTRINSIC_ESTIMATION_DEPTH_UPDATE_H_
#define INTRINSIC_ESTIMATION_DEPTH_UPDATE_H_

#include <optional>

#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/innovation_gate.h"

// Depth pressure / depth-sensor update only (#87). Depth is positive down from
// a free surface given as an ENU Up coordinate. No altitude, barometric, or air
// model, no DVL, lever arm, ICON, or hardware. See DEPTH_UPDATE.md. Mirrors
// depth_update.py.

namespace intrinsic::estimation {

// One depth sample as plain values (no marine proto).
struct DepthSample {
  // Measured depth, positive down, SI meters. Must be finite.
  double depth_m = 0.0;
  // Scalar measurement variance, m^2. Must be finite and > kMinCholeskyPivot.
  double R = 0.0;
  // Standalone stand-in for MeasurementHealth VALID. Must be true.
  bool valid = false;
  // ENU Up coordinate of the free surface, m. Must be finite.
  double free_surface_up_m = 0.0;
};

enum class DepthUpdateStatus {
  kOkAccept = 0,
  kOkReject,
  kSkippedInvalid,
  kSingular,
  kNonFinite,
};

struct DepthUpdateResult {
  // True when the gate ran to a decision (including a later kNonFinite).
  bool evaluated = false;
  // True only for kOkAccept.
  bool accepted = false;
  DepthUpdateStatus status = DepthUpdateStatus::kSkippedInvalid;
  // #84 diagnostics (d^2, threshold, |nu|, dof = 1). Set iff evaluated.
  std::optional<GateDiagnostics> diagnostics;
  // Set only for kOkAccept. Every other status leaves the caller's x and P
  // untouched, and these stay unset.
  std::optional<EskfNominal> nominal;
  std::optional<EskfCovariance> P;
  // Error-state correction dx = K nu that was injected. Set only for
  // kOkAccept.
  std::optional<EskfError> delta_x;
};

// h(x) = free_surface_up_m - p_enu[2], nu = z.depth_m - h(x), H = 1x15 with
// -1 at the dp_z column and 0 elsewhere. Gates with GateInnovation (dof = 1),
// then applies a Joseph update and injects dx into the nominal (right attitude
// error). Never modifies its inputs. Computes on temporaries and commits only
// when everything is finite.
DepthUpdateResult UpdateDepth(const EskfNominal& x, const EskfCovariance& P,
                              const DepthSample& z, double chi2_threshold);

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_DEPTH_UPDATE_H_
