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

#include "intrinsic/estimation/altitude_update.h"

#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/innovation_gate.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;
constexpr int kDpZ = kErrorDp + 2;

template <typename Container> bool AllFinite(const Container &v) {
  for (double x : v) {
    if (!std::isfinite(x))
      return false;
  }
  return true;
}

AltitudeUpdateResult Skip(AltitudeUpdateStatus status) {
  AltitudeUpdateResult result;
  result.status = status;
  return result;
}

double Norm4(const std::array<double, 4> &q) {
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

} // namespace

AltitudeUpdateResult UpdateAltitude(const EskfNominal &x,
                                    const EskfCovariance &P,
                                    const AltitudeSample &z,
                                    const SeafloorContext &seafloor,
                                    double chi2_threshold) {
  // 1. Health.
  if (!z.valid)
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);

  // 2. Threshold, nominal and quaternion, P.
  if (!std::isfinite(chi2_threshold) || !(chi2_threshold > 0.0)) {
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);
  }
  if (!AllFinite(x.p_enu) || !AllFinite(x.q_wxyz) || !AllFinite(x.v_body) ||
      !AllFinite(x.b_a) || !AllFinite(x.b_g)) {
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);
  }
  const double q_norm_in = Norm4(x.q_wxyz);
  if (!std::isfinite(q_norm_in) || q_norm_in < kMinQuaternionNorm) {
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);
  }
  if (!P.has_value() || !AllFinite(P.row_major())) {
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);
  }

  // 3. Seafloor context, then altitude and R.
  if (!seafloor.present || !std::isfinite(seafloor.seafloor_up_m)) {
    return Skip(AltitudeUpdateStatus::kSkippedMissingSeafloor);
  }
  if (!std::isfinite(z.altitude_m) || !std::isfinite(z.R)) {
    return Skip(AltitudeUpdateStatus::kSkippedInvalid);
  }

  // 4. R is 1x1, so its Cholesky pivot is R itself.
  if (z.R <= kMinCholeskyPivot)
    return Skip(AltitudeUpdateStatus::kSingular);

  // 5. Gate. h(x) = p_z - seafloor_up, so H has +1 at the dp_z column.
  const double h_x = x.p_enu[2] - seafloor.seafloor_up_m;
  const double nu = z.altitude_m - h_x;
  if (!std::isfinite(h_x) || !std::isfinite(nu)) {
    return Skip(AltitudeUpdateStatus::kNonFinite);
  }
  std::vector<double> h(kN, 0.0);
  h[kDpZ] = 1.0;

  const GateResult gate = GateInnovation({nu}, h, P, {z.R}, chi2_threshold);
  if (!gate.evaluated) {
    switch (gate.status) {
    case GateStatus::kSingularS:
      return Skip(AltitudeUpdateStatus::kSingular);
    case GateStatus::kNonFinite:
      return Skip(AltitudeUpdateStatus::kNonFinite);
    default:
      return Skip(AltitudeUpdateStatus::kSkippedInvalid);
    }
  }

  AltitudeUpdateResult result;
  result.evaluated = true;
  result.diagnostics = gate.diagnostics;
  if (!gate.accepted) {
    result.status = AltitudeUpdateStatus::kOkReject;
    return result;
  }
  auto fail_non_finite = [&result]() {
    result.status = AltitudeUpdateStatus::kNonFinite;
    result.accepted = false;
    return std::move(result);
  };

  // 6. Gain. Uses P_s and the gate's symmetrized scalar S_s. K = P_s H^T / S,
  // and H^T selects column dp_z.
  const auto &pm = P.row_major();
  std::array<double, kN * kN> p_s;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      p_s[i * kN + j] = 0.5 * (pm[i * kN + j] + pm[j * kN + i]);
    }
  }
  const double s = result.diagnostics->S[0];
  if (!std::isfinite(s) || s <= kMinCholeskyPivot)
    return fail_non_finite();
  std::array<double, kN> k;
  for (int i = 0; i < kN; ++i)
    k[i] = p_s[i * kN + kDpZ] / s;

  // 7. dx = K nu.
  std::array<double, kN> dx;
  for (int i = 0; i < kN; ++i)
    dx[i] = k[i] * nu;

  // 8. Inject into a nominal copy. Right attitude error, first-order Exp.
  EskfNominal x_new = x;
  for (int i = 0; i < 3; ++i) {
    x_new.p_enu[i] += dx[kErrorDp + i];
    x_new.v_body[i] += dx[kErrorDv + i];
    x_new.b_a[i] += dx[kErrorDba + i];
    x_new.b_g[i] += dx[kErrorDbg + i];
  }
  const std::array<double, 4> dq = {1.0, 0.5 * dx[kErrorDtheta],
                                    0.5 * dx[kErrorDtheta + 1],
                                    0.5 * dx[kErrorDtheta + 2]};
  std::array<double, 4> q_tmp = QuaternionMultiply(x.q_wxyz, dq);
  const double q_norm = Norm4(q_tmp);
  if (!std::isfinite(q_norm) || q_norm < kMinQuaternionNorm) {
    return fail_non_finite();
  }
  for (int i = 0; i < 4; ++i)
    x_new.q_wxyz[i] = q_tmp[i] / q_norm;

  // 9. Joseph: P = (I - K H) P_s (I - K H)^T + K R K^T, then symmetrize.
  // (K H)[i][j] = K[i] for j = dp_z and 0 otherwise.
  std::array<double, kN * kN> i_kh;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      const double kh = (j == kDpZ) ? k[i] : 0.0;
      i_kh[i * kN + j] = (i == j ? 1.0 : 0.0) - kh;
    }
  }
  std::array<double, kN * kN> a; // (I - K H) P_s
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double sum = 0.0;
      for (int m = 0; m < kN; ++m)
        sum += i_kh[i * kN + m] * p_s[m * kN + j];
      a[i * kN + j] = sum;
    }
  }
  std::array<double, kN * kN> p_tmp;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double sum = 0.0;
      for (int m = 0; m < kN; ++m)
        sum += a[i * kN + m] * i_kh[j * kN + m];
      sum += k[i] * z.R * k[j];
      p_tmp[i * kN + j] = sum;
    }
  }
  std::vector<double> p_out(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      p_out[i * kN + j] = 0.5 * (p_tmp[i * kN + j] + p_tmp[j * kN + i]);
    }
  }

  // 10. Commit only when everything is finite.
  if (!AllFinite(x_new.p_enu) || !AllFinite(x_new.q_wxyz) ||
      !AllFinite(x_new.v_body) || !AllFinite(x_new.b_a) ||
      !AllFinite(x_new.b_g) || !AllFinite(p_out) || !AllFinite(dx)) {
    return fail_non_finite();
  }
  const std::optional<EskfCovariance> p_new =
      EskfCovariance::FromRowMajor(p_out);
  const std::optional<EskfError> delta_x =
      EskfError::FromVector(std::vector<double>(dx.begin(), dx.end()));
  if (!p_new.has_value() || !delta_x.has_value())
    return fail_non_finite();

  result.status = AltitudeUpdateStatus::kOkAccept;
  result.accepted = true;
  result.nominal = x_new;
  result.P = *p_new;
  result.delta_x = *delta_x;
  return result;
}

} // namespace intrinsic::estimation
