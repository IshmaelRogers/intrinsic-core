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

#include "intrinsic/estimation/innovation_gate.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;

template <typename Container>
bool AllFinite(const Container& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

GateResult Reject(GateStatus status) {
  GateResult result;
  result.status = status;
  return result;
}

bool IsSymmetric(const std::vector<double>& a, int m, double tol) {
  for (int i = 0; i < m; ++i) {
    for (int j = i + 1; j < m; ++j) {
      if (std::fabs(a[i * m + j] - a[j * m + i]) > tol) return false;
    }
  }
  return true;
}

// In-place lower Cholesky S = L L^T, row-major m x m. The pivot is the value
// under the square root. Returns false when any pivot is <= kMinCholeskyPivot.
// Sets *non_finite instead when a pivot is not finite.
bool Cholesky(std::vector<double>& a, int m, bool* non_finite) {
  for (int j = 0; j < m; ++j) {
    double pivot = a[j * m + j];
    for (int k = 0; k < j; ++k) pivot -= a[j * m + k] * a[j * m + k];
    if (!std::isfinite(pivot)) {
      *non_finite = true;
      return false;
    }
    if (pivot <= kMinCholeskyPivot) return false;
    const double diag = std::sqrt(pivot);
    a[j * m + j] = diag;
    for (int i = j + 1; i < m; ++i) {
      double s = a[i * m + j];
      for (int k = 0; k < j; ++k) s -= a[i * m + k] * a[j * m + k];
      a[i * m + j] = s / diag;
    }
  }
  return true;
}

// Solves (L L^T) y = b with L in the lower triangle of `l`.
std::vector<double> CholeskySolve(const std::vector<double>& l, int m,
                                  const std::vector<double>& b) {
  std::vector<double> z(m);
  for (int i = 0; i < m; ++i) {
    double s = b[i];
    for (int k = 0; k < i; ++k) s -= l[i * m + k] * z[k];
    z[i] = s / l[i * m + i];
  }
  std::vector<double> y(m);
  for (int i = m - 1; i >= 0; --i) {
    double s = z[i];
    for (int k = i + 1; k < m; ++k) s -= l[k * m + i] * y[k];
    y[i] = s / l[i * m + i];
  }
  return y;
}

}  // namespace

GateResult GateInnovation(const std::vector<double>& nu,
                          const std::vector<double>& H, const EskfCovariance& P,
                          const std::vector<double>& R, double chi2_threshold) {
  const std::size_t m_size = nu.size();
  if (m_size < 1 || H.size() != m_size * kN || R.size() != m_size * m_size) {
    return Reject(GateStatus::kInvalidDim);
  }
  const int m = static_cast<int>(m_size);

  if (!P.has_value() || !AllFinite(P.row_major())) {
    return Reject(GateStatus::kInvalidP);
  }
  if (!AllFinite(R) || !IsSymmetric(R, m, kRSymmetryTol)) {
    return Reject(GateStatus::kInvalidR);
  }
  if (!AllFinite(nu)) return Reject(GateStatus::kInvalidNu);
  if (!std::isfinite(chi2_threshold) || !(chi2_threshold > 0.0)) {
    return Reject(GateStatus::kInvalidThreshold);
  }
  if (!AllFinite(H)) return Reject(GateStatus::kNonFinite);

  const auto& p = P.row_major();
  std::vector<double> p_s(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      p_s[i * kN + j] = 0.5 * (p[i * kN + j] + p[j * kN + i]);
    }
  }

  // HP = H P_s, m x kN.
  std::vector<double> hp(m_size * kN);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int k = 0; k < kN; ++k) s += H[i * kN + k] * p_s[k * kN + j];
      hp[i * kN + j] = s;
    }
  }

  // S = HP H^T + R, then S_s = 0.5 (S + S^T).
  std::vector<double> s_raw(m_size * m_size);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      double s = 0.0;
      for (int k = 0; k < kN; ++k) s += hp[i * kN + k] * H[j * kN + k];
      s_raw[i * m + j] = s + R[i * m + j];
    }
  }
  std::vector<double> s_s(m_size * m_size);
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      s_s[i * m + j] = 0.5 * (s_raw[i * m + j] + s_raw[j * m + i]);
    }
  }
  if (!AllFinite(s_s)) return Reject(GateStatus::kNonFinite);

  std::vector<double> l = s_s;
  bool non_finite = false;
  if (!Cholesky(l, m, &non_finite)) {
    return Reject(non_finite ? GateStatus::kNonFinite : GateStatus::kSingularS);
  }

  const std::vector<double> y = CholeskySolve(l, m, nu);
  double d2 = 0.0;
  for (int i = 0; i < m; ++i) d2 += nu[i] * y[i];
  double norm_sq = 0.0;
  for (int i = 0; i < m; ++i) norm_sq += nu[i] * nu[i];
  const double norm = std::sqrt(norm_sq);
  if (!std::isfinite(d2) || !std::isfinite(norm) || d2 < kMinMahalanobisSq) {
    return Reject(GateStatus::kNonFinite);
  }
  if (d2 < 0.0) d2 = 0.0;

  GateResult result;
  result.accepted = d2 <= chi2_threshold;
  result.ok = true;
  result.evaluated = true;
  result.status =
      result.accepted ? GateStatus::kOkAccept : GateStatus::kOkReject;
  GateDiagnostics diag;
  diag.mahalanobis_sq = d2;
  diag.threshold = chi2_threshold;
  diag.innovation_norm = norm;
  diag.dof = m;
  diag.S = std::move(s_s);
  result.diagnostics = std::move(diag);
  return result;
}

}  // namespace intrinsic::estimation
