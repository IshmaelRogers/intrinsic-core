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

#include "intrinsic/estimation/eskf_state.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace intrinsic::estimation {
namespace {

template <size_t N>
void Put(const std::array<double, N>& src, double* dst) {
  std::copy(src.begin(), src.end(), dst);
}

template <size_t N>
void Get(const double* src, std::array<double, N>& dst) {
  std::copy(src, src + N, dst.begin());
}

}  // namespace

std::array<double, kNominalDim> EskfNominal::ToArray() const {
  std::array<double, kNominalDim> out;
  Put(p_enu, out.data() + kNominalPEnu);
  Put(q_wxyz, out.data() + kNominalQWxyz);
  Put(v_body, out.data() + kNominalVBody);
  Put(b_a, out.data() + kNominalBa);
  Put(b_g, out.data() + kNominalBg);
  return out;
}

std::optional<EskfNominal> EskfNominal::FromVector(
    const std::vector<double>& values) {
  if (values.size() != static_cast<size_t>(kNominalDim)) return std::nullopt;
  EskfNominal out;
  Get(values.data() + kNominalPEnu, out.p_enu);
  Get(values.data() + kNominalQWxyz, out.q_wxyz);
  Get(values.data() + kNominalVBody, out.v_body);
  Get(values.data() + kNominalBa, out.b_a);
  Get(values.data() + kNominalBg, out.b_g);
  return out;
}

std::array<double, kErrorDim> EskfError::ToArray() const {
  std::array<double, kErrorDim> out;
  Put(dp, out.data() + kErrorDp);
  Put(dtheta, out.data() + kErrorDtheta);
  Put(dv, out.data() + kErrorDv);
  Put(dba, out.data() + kErrorDba);
  Put(dbg, out.data() + kErrorDbg);
  return out;
}

std::optional<EskfError> EskfError::FromVector(
    const std::vector<double>& values) {
  if (values.size() != static_cast<size_t>(kErrorDim)) return std::nullopt;
  EskfError out;
  Get(values.data() + kErrorDp, out.dp);
  Get(values.data() + kErrorDtheta, out.dtheta);
  Get(values.data() + kErrorDv, out.dv);
  Get(values.data() + kErrorDba, out.dba);
  Get(values.data() + kErrorDbg, out.dbg);
  return out;
}

EskfCovariance EskfCovariance::Identity() {
  EskfCovariance out;
  std::array<double, kCovDim * kCovDim> m;
  m.fill(0.0);
  for (int i = 0; i < kCovDim; ++i) m[i * kCovDim + i] = 1.0;
  out.data_ = m;
  return out;
}

std::optional<EskfCovariance> EskfCovariance::FromRowMajor(
    const std::vector<double>& values) {
  if (values.size() != static_cast<size_t>(kCovDim) * kCovDim) {
    return std::nullopt;
  }
  EskfCovariance out;
  std::array<double, kCovDim * kCovDim> m;
  std::copy(values.begin(), values.end(), m.begin());
  out.data_ = m;
  return out;
}

double EskfCovariance::At(int row, int col) const {
  return (*data_)[row * kCovDim + col];
}

}  // namespace intrinsic::estimation
