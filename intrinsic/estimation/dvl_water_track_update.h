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

#ifndef INTRINSIC_ESTIMATION_DVL_WATER_TRACK_UPDATE_H_
#define INTRINSIC_ESTIMATION_DVL_WATER_TRACK_UPDATE_H_

#include <array>
#include <optional>

#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/innovation_gate.h"

// DVL water-track body-velocity update only (#86). The water current is an
// external input, never an ESKF state. No ground-referenced velocity update,
// lever arm, Earth rate, other sensors, ICON, or hardware. See
// DVL_WATER_TRACK_UPDATE.md. Mirrors dvl_water_track_update.py.

namespace intrinsic::estimation {

// One water-track velocity sample as plain values (no marine proto).
struct DvlWaterTrackSample {
  // Linear velocity relative to the water in the body frame (REP-103), m/s.
  std::array<double, 3> velocity_body_m_s = {0.0, 0.0, 0.0};
  // Row-major 3x3 measurement covariance, (m/s)^2. Must be finite, symmetric
  // within kRSymmetryTol, and Cholesky-capable with pivots > kMinCholeskyPivot.
  // The caller folds any current uncertainty into this matrix.
  std::array<double, 9> R_body = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Standalone stand-in for MeasurementHealth VALID. Must be true.
  bool valid = false;
};

// External water-current context (not an ESKF state).
struct WaterCurrentEstimate {
  // When false the update is skipped.
  bool present = false;
  // Water current in world ENU, m/s. Must be finite when present.
  std::array<double, 3> v_enu_m_s = {0.0, 0.0, 0.0};
  // Row-major 3x3 current covariance. Unused by this update.
  std::array<double, 9> R_current_enu = {0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0};
};

enum class DvlWaterTrackStatus {
  kOkAccept = 0,
  kOkReject,
  kSkippedMissingCurrent,
  kSkippedInvalid,
  kSingular,
  kNonFinite,
};

struct DvlWaterTrackResult {
  // True when the gate ran to a decision (including a later kNonFinite).
  bool evaluated = false;
  // True only for kOkAccept.
  bool accepted = false;
  DvlWaterTrackStatus status = DvlWaterTrackStatus::kSkippedInvalid;
  // #84 diagnostics (d^2, threshold, ||nu||, dof = 3). Set iff evaluated.
  std::optional<GateDiagnostics> diagnostics;
  // Set only for kOkAccept. Every other status leaves the caller's x and P
  // untouched, and these stay unset.
  std::optional<EskfNominal> nominal;
  std::optional<EskfCovariance> P;
  // Error-state correction dx = K nu that was injected. Set only for
  // kOkAccept.
  std::optional<EskfError> delta_x;
};

// c_b = R(q)^T c_enu, h(x) = v_body - c_b, nu = z.velocity - h(x).
// H = [0 | -[c_b x] at dtheta | I3 at dv | 0]. Gates with GateInnovation, then
// applies a Joseph update and injects dx into the nominal (right attitude
// error). Never modifies its inputs. Computes on temporaries and commits only
// when everything is finite.
DvlWaterTrackResult UpdateDvlWaterTrack(const EskfNominal &x,
                                        const EskfCovariance &P,
                                        const DvlWaterTrackSample &z,
                                        const WaterCurrentEstimate &current,
                                        double chi2_threshold);

} // namespace intrinsic::estimation

#endif // INTRINSIC_ESTIMATION_DVL_WATER_TRACK_UPDATE_H_
