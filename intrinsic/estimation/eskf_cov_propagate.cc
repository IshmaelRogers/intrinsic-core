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

#include "intrinsic/estimation/eskf_cov_propagate.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;

template <std::size_t N> bool AllFinite(const std::array<double, N> &v) {
  for (double x : v) {
    if (!std::isfinite(x))
      return false;
  }
  return true;
}

bool ValidSigma(double s) { return std::isfinite(s) && s >= 0.0; }

CovPropagateResult Reject(CovPropagateStatus status) {
  CovPropagateResult result;
  result.ok = false;
  result.status = status;
  return result;
}

using Mat3 = std::array<double, 9>;

// [u x], row-major.
Mat3 Skew(const std::array<double, 3> &u) {
  return {0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0};
}

// A * B, row-major 3x3.
Mat3 Mul3(const Mat3 &a, const Mat3 &b) {
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        s += a[3 * i + k] * b[3 * k + j];
      c[3 * i + j] = s;
    }
  }
  return c;
}

// Adds scale * m into the 3x3 block of f at (block_row, block_col).
void AddBlock(EskfMatrix &f, int block_row, int block_col, const Mat3 &m,
              double scale) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      f[(block_row + i) * kN + (block_col + j)] += scale * m[3 * i + j];
    }
  }
}

constexpr Mat3 kIdentity3 = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// Order of checks mirrors #82: dt, IMU, nominal finite, quaternion norm.
std::optional<CovPropagateStatus>
CheckInputs(const EskfNominal &x, const ImuSample &imu, double dt_s) {
  if (!std::isfinite(dt_s) || !(dt_s > 0.0) || dt_s > kMaxPropagateDtS) {
    return CovPropagateStatus::kInvalidDt;
  }
  if (!AllFinite(imu.accel_m_s2) || !AllFinite(imu.gyro_rad_s)) {
    return CovPropagateStatus::kInvalidImu;
  }
  if (!AllFinite(x.p_enu) || !AllFinite(x.q_wxyz) || !AllFinite(x.v_body) ||
      !AllFinite(x.b_a) || !AllFinite(x.b_g)) {
    return CovPropagateStatus::kNonFinite;
  }
  const std::array<double, 4> &q = x.q_wxyz;
  const double q_norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(q_norm) || q_norm < kMinQuaternionNorm) {
    return CovPropagateStatus::kInvalidQuaternion;
  }
  return std::nullopt;
}

// Continuous F. Inputs already validated.
EskfMatrix BuildF(const EskfNominal &x, const ImuSample &imu) {
  const std::array<double, 3> omega = {imu.gyro_rad_s[0] - x.b_g[0],
                                       imu.gyro_rad_s[1] - x.b_g[1],
                                       imu.gyro_rad_s[2] - x.b_g[2]};
  const Mat3 r = RotationBodyToWorld(x.q_wxyz);
  // g_b = R^T g_enu.
  const std::array<double, 3> g_b = {
      r[0] * kGravityEnu[0] + r[3] * kGravityEnu[1] + r[6] * kGravityEnu[2],
      r[1] * kGravityEnu[0] + r[4] * kGravityEnu[1] + r[7] * kGravityEnu[2],
      r[2] * kGravityEnu[0] + r[5] * kGravityEnu[1] + r[8] * kGravityEnu[2]};
  const Mat3 v_x = Skew(x.v_body);
  const Mat3 w_x = Skew(omega);
  const Mat3 g_x = Skew(g_b);

  EskfMatrix f{};
  AddBlock(f, kErrorDp, kErrorDtheta, Mul3(r, v_x), -1.0);
  AddBlock(f, kErrorDp, kErrorDv, r, 1.0);
  AddBlock(f, kErrorDtheta, kErrorDtheta, w_x, -1.0);
  AddBlock(f, kErrorDtheta, kErrorDbg, kIdentity3, -1.0);
  AddBlock(f, kErrorDv, kErrorDtheta, g_x, 1.0);
  AddBlock(f, kErrorDv, kErrorDv, w_x, -1.0);
  AddBlock(f, kErrorDv, kErrorDba, kIdentity3, -1.0);
  AddBlock(f, kErrorDv, kErrorDbg, v_x, -1.0);
  return f;
}

EskfMatrix PhiFromF(const EskfMatrix &f, double dt_s) {
  EskfMatrix phi{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      phi[i * kN + j] = (i == j ? 1.0 : 0.0) + f[i * kN + j] * dt_s;
    }
  }
  return phi;
}

bool ValidNoise(const ProcessNoiseConfig &n) {
  return ValidSigma(n.sigma_accel) && ValidSigma(n.sigma_gyro) &&
         ValidSigma(n.sigma_accel_bias_rw) && ValidSigma(n.sigma_gyro_bias_rw);
}

// Qd = G Qc G^T dt. G maps n_a -> dv, n_g -> dtheta, n_ba -> dba,
// n_bg -> dbg, each with an identity block, so G Qc G^T is block diagonal.
EskfMatrix QdFromNoise(const ProcessNoiseConfig &n, double dt_s) {
  EskfMatrix qd{};
  const std::array<std::pair<int, double>, 4> blocks = {{
      {kErrorDtheta, n.sigma_gyro},
      {kErrorDv, n.sigma_accel},
      {kErrorDba, n.sigma_accel_bias_rw},
      {kErrorDbg, n.sigma_gyro_bias_rw},
  }};
  for (const auto &[first, sigma] : blocks) {
    for (int i = 0; i < 3; ++i) {
      qd[(first + i) * kN + (first + i)] = sigma * sigma * dt_s;
    }
  }
  return qd;
}

} // namespace

std::optional<EskfMatrix> BuildPhi(const EskfNominal &x, const ImuSample &imu,
                                   double dt_s) {
  if (CheckInputs(x, imu, dt_s).has_value())
    return std::nullopt;
  return PhiFromF(BuildF(x, imu), dt_s);
}

std::optional<EskfMatrix> BuildQd(const ProcessNoiseConfig &noise,
                                  double dt_s) {
  if (!std::isfinite(dt_s) || !(dt_s > 0.0) || dt_s > kMaxPropagateDtS ||
      !ValidNoise(noise)) {
    return std::nullopt;
  }
  return QdFromNoise(noise, dt_s);
}

CovPropagateResult PropagateCovariance(const EskfNominal &x,
                                       const ImuSample &imu, double dt_s,
                                       const EskfCovariance &P,
                                       const ProcessNoiseConfig &noise) {
  if (const auto status = CheckInputs(x, imu, dt_s))
    return Reject(*status);
  if (!P.has_value() || !AllFinite(P.row_major())) {
    return Reject(CovPropagateStatus::kInvalidP);
  }
  if (!ValidNoise(noise))
    return Reject(CovPropagateStatus::kInvalidNoise);

  const EskfMatrix phi = PhiFromF(BuildF(x, imu), dt_s);
  const EskfMatrix qd = QdFromNoise(noise, dt_s);
  const EskfMatrix &p = P.row_major();

  // tmp = Phi * P.
  EskfMatrix tmp{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int k = 0; k < kN; ++k)
        s += phi[i * kN + k] * p[k * kN + j];
      tmp[i * kN + j] = s;
    }
  }
  // p_tmp = tmp * Phi^T + Qd.
  EskfMatrix p_tmp{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int k = 0; k < kN; ++k)
        s += tmp[i * kN + k] * phi[j * kN + k];
      p_tmp[i * kN + j] = s + qd[i * kN + j];
    }
  }
  EskfMatrix out{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      out[i * kN + j] = 0.5 * (p_tmp[i * kN + j] + p_tmp[j * kN + i]);
    }
  }
  if (!AllFinite(out))
    return Reject(CovPropagateStatus::kNonFinite);

  const std::optional<EskfCovariance> cov =
      EskfCovariance::FromRowMajor(std::vector<double>(out.begin(), out.end()));
  CovPropagateResult result;
  result.ok = true;
  result.status = CovPropagateStatus::kOk;
  result.P = cov;
  return result;
}

} // namespace intrinsic::estimation
