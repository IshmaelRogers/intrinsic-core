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

#ifndef INTRINSIC_ESTIMATION_ESKF_PROPAGATE_H_
#define INTRINSIC_ESTIMATION_ESKF_PROPAGATE_H_

#include <array>
#include <optional>

#include "intrinsic/estimation/eskf_state.h"

// IMU nominal-state propagation only (#82). No covariance, Phi, Q, or
// measurement update. See ESKF_PROPAGATE.md. Mirrors eskf_propagate.py.

namespace intrinsic::estimation {

// World ENU gravity (W0), m/s^2.
inline constexpr std::array<double, 3> kGravityEnu = {0.0, 0.0, -9.80665};
inline constexpr double kMaxPropagateDtS = 1.0;
inline constexpr double kMinQuaternionNorm = 1e-12;

// One IMU sample in the body frame, SI.
struct ImuSample {
  // Specific force as measured by the accelerometer (includes gravity effect).
  std::array<double, 3> accel_m_s2 = {0.0, 0.0, 0.0};
  // Angular rate.
  std::array<double, 3> gyro_rad_s = {0.0, 0.0, 0.0};
};

enum class PropagateStatus {
  kOk = 0,
  kInvalidDt,
  kInvalidImu,
  kInvalidQuaternion,
  kNonFiniteState,
};

struct PropagateResult {
  bool ok = false;
  PropagateStatus status = PropagateStatus::kInvalidDt;
  // Set only when ok. A rejected call leaves no partial state.
  std::optional<EskfNominal> nominal;
};

// Shared helpers, documented in ESKF_PROPAGATE.md.
std::array<double, 3> Cross(const std::array<double, 3>& a,
                            const std::array<double, 3>& b);
// Hamilton product a (x) b, wxyz order.
std::array<double, 4> QuaternionMultiply(const std::array<double, 4>& a,
                                         const std::array<double, 4>& b);
// Body->world rotation of q / ||q||, row-major 3x3.
std::array<double, 9> RotationBodyToWorld(const std::array<double, 4>& q_wxyz);

PropagateResult PropagateNominal(const EskfNominal& x, const ImuSample& imu,
                                 double dt_s);

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_ESKF_PROPAGATE_H_
