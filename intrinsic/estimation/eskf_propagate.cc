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

#include "intrinsic/estimation/eskf_propagate.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

template <std::size_t N>
bool AllFinite(const std::array<double, N>& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

PropagateResult Reject(PropagateStatus status) {
  PropagateResult result;
  result.ok = false;
  result.status = status;
  return result;
}

// R * v, R row-major.
std::array<double, 3> MatVec(const std::array<double, 9>& r,
                             const std::array<double, 3>& v) {
  return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
          r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
          r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

// R^T * v, R row-major.
std::array<double, 3> MatTVec(const std::array<double, 9>& r,
                              const std::array<double, 3>& v) {
  return {r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
          r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
          r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
}

}  // namespace

std::array<double, 3> Cross(const std::array<double, 3>& a,
                            const std::array<double, 3>& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

std::array<double, 4> QuaternionMultiply(const std::array<double, 4>& a,
                                         const std::array<double, 4>& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

std::array<double, 9> RotationBodyToWorld(const std::array<double, 4>& q_wxyz) {
  const double n = std::sqrt(q_wxyz[0] * q_wxyz[0] + q_wxyz[1] * q_wxyz[1] +
                             q_wxyz[2] * q_wxyz[2] + q_wxyz[3] * q_wxyz[3]);
  const double w = q_wxyz[0] / n;
  const double x = q_wxyz[1] / n;
  const double y = q_wxyz[2] / n;
  const double z = q_wxyz[3] / n;
  return {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
          2.0 * (x * z + w * y),       2.0 * (x * y + w * z),
          1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
          2.0 * (x * z - w * y),       2.0 * (y * z + w * x),
          1.0 - 2.0 * (x * x + y * y)};
}

PropagateResult PropagateNominal(const EskfNominal& x, const ImuSample& imu,
                                 double dt_s) {
  // 1. dt.
  if (!std::isfinite(dt_s) || !(dt_s > 0.0) || dt_s > kMaxPropagateDtS) {
    return Reject(PropagateStatus::kInvalidDt);
  }

  // 2. IMU, nominal, and quaternion norm.
  if (!AllFinite(imu.accel_m_s2) || !AllFinite(imu.gyro_rad_s)) {
    return Reject(PropagateStatus::kInvalidImu);
  }
  if (!AllFinite(x.p_enu) || !AllFinite(x.q_wxyz) || !AllFinite(x.v_body) ||
      !AllFinite(x.b_a) || !AllFinite(x.b_g)) {
    return Reject(PropagateStatus::kNonFiniteState);
  }
  const std::array<double, 4>& q = x.q_wxyz;
  const double q_norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(q_norm) || q_norm < kMinQuaternionNorm) {
    return Reject(PropagateStatus::kInvalidQuaternion);
  }

  const std::array<double, 3> omega = {imu.gyro_rad_s[0] - x.b_g[0],
                                       imu.gyro_rad_s[1] - x.b_g[1],
                                       imu.gyro_rad_s[2] - x.b_g[2]};
  const std::array<double, 3> a = {imu.accel_m_s2[0] - x.b_a[0],
                                   imu.accel_m_s2[1] - x.b_a[1],
                                   imu.accel_m_s2[2] - x.b_a[2]};

  // 3. Attitude: first-order Hamilton, then renormalize.
  const std::array<double, 4> q_dot = [&] {
    std::array<double, 4> d =
        QuaternionMultiply(q, {0.0, omega[0], omega[1], omega[2]});
    for (double& c : d) c *= 0.5;
    return d;
  }();
  std::array<double, 4> q_tmp;
  for (int i = 0; i < 4; ++i) q_tmp[i] = q[i] + dt_s * q_dot[i];
  const double tmp_norm = std::sqrt(q_tmp[0] * q_tmp[0] + q_tmp[1] * q_tmp[1] +
                                    q_tmp[2] * q_tmp[2] + q_tmp[3] * q_tmp[3]);
  if (!std::isfinite(tmp_norm) || tmp_norm < kMinQuaternionNorm) {
    return Reject(PropagateStatus::kInvalidQuaternion);
  }
  std::array<double, 4> q_new;
  for (int i = 0; i < 4; ++i) q_new[i] = q_tmp[i] / tmp_norm;

  // 4 and 5. Velocity and position use the pre-update q and v.
  const std::array<double, 9> r = RotationBodyToWorld(q);
  const std::array<double, 3> w_cross_v = Cross(omega, x.v_body);
  const std::array<double, 3> g_body = MatTVec(r, kGravityEnu);
  const std::array<double, 3> p_dot = MatVec(r, x.v_body);

  EskfNominal out = x;
  for (int i = 0; i < 3; ++i) {
    out.v_body[i] = x.v_body[i] + dt_s * (a[i] - w_cross_v[i] + g_body[i]);
    out.p_enu[i] = x.p_enu[i] + dt_s * p_dot[i];
  }
  out.q_wxyz = q_new;

  // 7. Never return a partial or non-finite state.
  if (!AllFinite(out.p_enu) || !AllFinite(out.q_wxyz) ||
      !AllFinite(out.v_body)) {
    return Reject(PropagateStatus::kNonFiniteState);
  }

  PropagateResult result;
  result.ok = true;
  result.status = PropagateStatus::kOk;
  result.nominal = out;
  return result;
}

}  // namespace intrinsic::estimation
