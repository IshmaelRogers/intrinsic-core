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

#ifndef INTRINSIC_ESTIMATION_ESKF_COV_PROPAGATE_H_
#define INTRINSIC_ESTIMATION_ESKF_COV_PROPAGATE_H_

#include <array>
#include <optional>

#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/estimation/eskf_state.h"

// ESKF covariance transition and process noise only (#83). No measurement
// update, gain, Joseph form, van Loan, or expm. See ESKF_COV_PROPAGATE.md.
// Mirrors eskf_cov_propagate.py. Nominal propagation stays in #82.

namespace intrinsic::estimation {

// Continuous noise sqrt-PSDs, applied isotropically on each triad. All must be
// finite and >= 0. Default is all zero (no process noise).
struct ProcessNoiseConfig {
  // Accel white noise, (m/s^2)/sqrt(Hz).
  double sigma_accel = 0.0;
  // Gyro white noise, (rad/s)/sqrt(Hz).
  double sigma_gyro = 0.0;
  // Accel bias random walk, (m/s^2)/sqrt(s).
  double sigma_accel_bias_rw = 0.0;
  // Gyro bias random walk, (rad/s)/sqrt(s).
  double sigma_gyro_bias_rw = 0.0;
};

enum class CovPropagateStatus {
  kOk = 0,
  kInvalidDt,
  kInvalidImu,
  kInvalidQuaternion,
  kInvalidP,
  kInvalidNoise,
  kNonFinite,
};

struct CovPropagateResult {
  bool ok = false;
  CovPropagateStatus status = CovPropagateStatus::kInvalidDt;
  // Set only when ok (15x15). A rejected call leaves no partial state.
  std::optional<EskfCovariance> P;
};

using EskfMatrix = std::array<double, kCovDim * kCovDim>;

// P_out = sym(Phi P Phi^T + Qd), using the pre-update nominal x and the same
// IMU sample and dt as the #82 step. Does not modify the nominal state.
CovPropagateResult PropagateCovariance(const EskfNominal &x,
                                       const ImuSample &imu, double dt_s,
                                       const EskfCovariance &P,
                                       const ProcessNoiseConfig &noise);

// Test helpers. Not a product API. Phi = I + F dt, row-major 15x15. Returns
// nullopt for the same dt, IMU, nominal, and quaternion defects that
// PropagateCovariance rejects.
std::optional<EskfMatrix> BuildPhi(const EskfNominal &x, const ImuSample &imu,
                                   double dt_s);
// Qd = G Qc G^T dt, row-major 15x15. Requires valid dt and noise, else nullopt.
std::optional<EskfMatrix> BuildQd(const ProcessNoiseConfig &noise, double dt_s);

} // namespace intrinsic::estimation

#endif // INTRINSIC_ESTIMATION_ESKF_COV_PROPAGATE_H_
