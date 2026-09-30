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

#ifndef INTRINSIC_ESTIMATION_ALTITUDE_UPDATE_H_
#define INTRINSIC_ESTIMATION_ALTITUDE_UPDATE_H_

#include <optional>

#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/innovation_gate.h"

// Terrain / seafloor altimeter clearance update only (#88). Altitude is the
// clearance above the seafloor, positive up, and the seafloor is an external
// ENU Up coordinate. Not barometric ASL, no free-surface model, no DVL, lever
// arm, ICON, or hardware. See ALTITUDE_UPDATE.md. Mirrors altitude_update.py.

namespace intrinsic::estimation {

// External seafloor under the vehicle. Plain value, supplied by the caller.
struct SeafloorContext {
  // False means no seafloor is known and the update is skipped.
  bool present = false;
  // ENU Up coordinate of the seafloor under the vehicle, m. Finite when
  // present.
  double seafloor_up_m = 0.0;
};

// One altimeter sample as plain values (no marine proto).
struct AltitudeSample {
  // Measured clearance above the seafloor, positive up, SI meters. Must be
  // finite.
  double altitude_m = 0.0;
  // Scalar measurement variance, m^2. Must be finite and > kMinCholeskyPivot.
  double R = 0.0;
  // Standalone stand-in for MeasurementHealth VALID. Must be true.
  bool valid = false;
};

enum class AltitudeUpdateStatus {
  kOkAccept = 0,
  kOkReject,
  kSkippedMissingSeafloor,
  kSkippedInvalid,
  kSingular,
  kNonFinite,
};

struct AltitudeUpdateResult {
  // True when the gate ran to a decision (including a later kNonFinite).
  bool evaluated = false;
  // True only for kOkAccept.
  bool accepted = false;
  AltitudeUpdateStatus status = AltitudeUpdateStatus::kSkippedInvalid;
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

// h(x) = p_enu[2] - seafloor_up_m, nu = z.altitude_m - h(x), H = 1x15 with +1
// at the dp_z column and 0 elsewhere. Gates with GateInnovation (dof = 1),
// then applies a Joseph update and injects dx into the nominal (right attitude
// error). Never modifies its inputs. Computes on temporaries and commits only
// when everything is finite.
AltitudeUpdateResult UpdateAltitude(const EskfNominal &x,
                                    const EskfCovariance &P,
                                    const AltitudeSample &z,
                                    const SeafloorContext &seafloor,
                                    double chi2_threshold);

} // namespace intrinsic::estimation

#endif // INTRINSIC_ESTIMATION_ALTITUDE_UPDATE_H_
