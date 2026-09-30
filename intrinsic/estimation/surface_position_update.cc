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

#include "intrinsic/estimation/surface_position_update.h"

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
constexpr int kM = 2;

template <typename Container>
bool AllFinite(const Container& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

SurfacePositionUpdateResult Skip(SurfacePositionUpdateStatus status) {
  SurfacePositionUpdateResult result;
  result.status = status;
  return result;
}

// In-place lower Cholesky of a row-major 2x2. Returns false when a pivot is
// <= kMinCholeskyPivot or not finite.
bool Cholesky2(std::array<double, kM * kM>& a) {
  for (int j = 0; j < kM; ++j) {
    double pivot = a[j * kM + j];
    for (int k = 0; k < j; ++k) pivot -= a[j * kM + k] * a[j * kM + k];
    if (!std::isfinite(pivot) || pivot <= kMinCholeskyPivot) return false;
    const double diag = std::sqrt(pivot);
    a[j * kM + j] = diag;
    for (int i = j + 1; i < kM; ++i) {
      double s = a[i * kM + j];
      for (int k = 0; k < j; ++k) s -= a[i * kM + k] * a[j * kM + k];
      a[i * kM + j] = s / diag;
    }
  }
  return true;
}

// Solves (L L^T) y = b with L in the lower triangle of `l`.
std::array<double, kM> CholeskySolve2(const std::array<double, kM * kM>& l,
                                      const std::array<double, kM>& b) {
  std::array<double, kM> z;
  for (int i = 0; i < kM; ++i) {
    double s = b[i];
    for (int k = 0; k < i; ++k) s -= l[i * kM + k] * z[k];
    z[i] = s / l[i * kM + i];
  }
  std::array<double, kM> y;
  for (int i = kM - 1; i >= 0; --i) {
    double s = z[i];
    for (int k = i + 1; k < kM; ++k) s -= l[k * kM + i] * y[k];
    y[i] = s / l[i * kM + i];
  }
  return y;
}

double Norm4(const std::array<double, 4>& q) {
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

}  // namespace

SurfacePositionUpdateResult UpdateSurfacePosition(
    const EskfNominal& x, const EskfCovariance& P,
    const SurfacePositionSample& z, const SurfaceFixPolicy& policy,
    double chi2_threshold) {
  // 1. Policy, then health. Policy is decided by the caller and wins over
  // everything else, including a garbage sample.
  if (!policy.is_surfaced || !policy.quality_ok) {
    return Skip(SurfacePositionUpdateStatus::kSkippedPolicy);
  }
  if (!z.valid) return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);

  // 2. Threshold, nominal and quaternion, P, position, R.
  if (!std::isfinite(chi2_threshold) || !(chi2_threshold > 0.0)) {
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  }
  if (!AllFinite(x.p_enu) || !AllFinite(x.q_wxyz) || !AllFinite(x.v_body) ||
      !AllFinite(x.b_a) || !AllFinite(x.b_g)) {
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  }
  const double q_norm_in = Norm4(x.q_wxyz);
  if (!std::isfinite(q_norm_in) || q_norm_in < kMinQuaternionNorm) {
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  }
  if (!P.has_value() || !AllFinite(P.row_major())) {
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  }
  if (!AllFinite(z.position_en_m)) {
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  }
  if (!AllFinite(z.R_en))
    return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
  for (int i = 0; i < kM; ++i) {
    for (int j = i + 1; j < kM; ++j) {
      if (std::fabs(z.R_en[i * kM + j] - z.R_en[j * kM + i]) > kRSymmetryTol) {
        return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
      }
    }
  }

  // 3. R must factor with pivots above the floor. Cholesky reads only the
  // lower triangle, and R is already symmetric within tolerance.
  std::array<double, kM * kM> r_l = z.R_en;
  if (!Cholesky2(r_l)) return Skip(SurfacePositionUpdateStatus::kSingular);

  // 4. Gate. h(x) = [p_e, p_n], H selects the dp_e and dp_n columns.
  std::vector<double> nu(kM);
  for (int i = 0; i < kM; ++i) nu[i] = z.position_en_m[i] - x.p_enu[i];
  if (!AllFinite(nu)) return Skip(SurfacePositionUpdateStatus::kNonFinite);
  std::vector<double> h(kM * kN, 0.0);
  for (int i = 0; i < kM; ++i) h[i * kN + kErrorDp + i] = 1.0;
  const std::vector<double> r(z.R_en.begin(), z.R_en.end());

  const GateResult gate = GateInnovation(nu, h, P, r, chi2_threshold);
  if (!gate.evaluated) {
    switch (gate.status) {
      case GateStatus::kSingularS:
        return Skip(SurfacePositionUpdateStatus::kSingular);
      case GateStatus::kNonFinite:
        return Skip(SurfacePositionUpdateStatus::kNonFinite);
      default:
        return Skip(SurfacePositionUpdateStatus::kSkippedInvalid);
    }
  }

  SurfacePositionUpdateResult result;
  result.evaluated = true;
  result.diagnostics = gate.diagnostics;
  if (!gate.accepted) {
    result.status = SurfacePositionUpdateStatus::kOkReject;
    return result;
  }

  // 5. Gain. Uses P_s and the gate's symmetrized S_s.
  const auto& pm = P.row_major();
  std::array<double, kN * kN> p_s;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      p_s[i * kN + j] = 0.5 * (pm[i * kN + j] + pm[j * kN + i]);
    }
  }
  std::array<double, kM * kM> s_l;
  for (int i = 0; i < kM * kM; ++i) s_l[i] = result.diagnostics->S[i];
  auto fail_non_finite = [&result]() {
    result.status = SurfacePositionUpdateStatus::kNonFinite;
    result.accepted = false;
    return std::move(result);
  };
  if (!Cholesky2(s_l)) return fail_non_finite();

  // K = P_s H^T S^-1. Row i of K solves S k_i = (P_s H^T)_i, S symmetric.
  std::array<double, kN * kM> k;
  for (int i = 0; i < kN; ++i) {
    std::array<double, kM> pht;
    for (int c = 0; c < kM; ++c) {
      double s = 0.0;
      for (int a = 0; a < kN; ++a) s += p_s[i * kN + a] * h[c * kN + a];
      pht[c] = s;
    }
    const std::array<double, kM> ki = CholeskySolve2(s_l, pht);
    for (int c = 0; c < kM; ++c) k[i * kM + c] = ki[c];
  }

  // 6. dx = K nu.
  std::array<double, kN> dx;
  for (int i = 0; i < kN; ++i) {
    double s = 0.0;
    for (int c = 0; c < kM; ++c) s += k[i * kM + c] * nu[c];
    dx[i] = s;
  }

  // 7. Inject into a nominal copy. Right attitude error, first-order Exp.
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
  for (int i = 0; i < 4; ++i) x_new.q_wxyz[i] = q_tmp[i] / q_norm;

  // 8. Joseph: P = (I - K H) P_s (I - K H)^T + K R K^T, then symmetrize.
  std::array<double, kN * kN> i_kh;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double kh = 0.0;
      for (int c = 0; c < kM; ++c) kh += k[i * kM + c] * h[c * kN + j];
      i_kh[i * kN + j] = (i == j ? 1.0 : 0.0) - kh;
    }
  }
  std::array<double, kN * kN> a;  // (I - K H) P_s
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int m = 0; m < kN; ++m) s += i_kh[i * kN + m] * p_s[m * kN + j];
      a[i * kN + j] = s;
    }
  }
  std::array<double, kN * kM> kr;  // K R
  for (int i = 0; i < kN; ++i) {
    for (int c = 0; c < kM; ++c) {
      double s = 0.0;
      for (int m = 0; m < kM; ++m) s += k[i * kM + m] * z.R_en[m * kM + c];
      kr[i * kM + c] = s;
    }
  }
  std::array<double, kN * kN> p_tmp;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int m = 0; m < kN; ++m) s += a[i * kN + m] * i_kh[j * kN + m];
      for (int c = 0; c < kM; ++c) s += kr[i * kM + c] * k[j * kM + c];
      p_tmp[i * kN + j] = s;
    }
  }
  std::vector<double> p_out(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      p_out[i * kN + j] = 0.5 * (p_tmp[i * kN + j] + p_tmp[j * kN + i]);
    }
  }

  // 9. Commit only when everything is finite.
  if (!AllFinite(x_new.p_enu) || !AllFinite(x_new.q_wxyz) ||
      !AllFinite(x_new.v_body) || !AllFinite(x_new.b_a) ||
      !AllFinite(x_new.b_g) || !AllFinite(p_out) || !AllFinite(dx)) {
    return fail_non_finite();
  }
  const std::optional<EskfCovariance> p_new =
      EskfCovariance::FromRowMajor(p_out);
  const std::optional<EskfError> delta_x =
      EskfError::FromVector(std::vector<double>(dx.begin(), dx.end()));
  if (!p_new.has_value() || !delta_x.has_value()) return fail_non_finite();

  result.status = SurfacePositionUpdateStatus::kOkAccept;
  result.accepted = true;
  result.nominal = x_new;
  result.P = *p_new;
  result.delta_x = *delta_x;
  return result;
}

}  // namespace intrinsic::estimation
