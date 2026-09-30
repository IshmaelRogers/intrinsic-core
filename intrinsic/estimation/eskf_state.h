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

#ifndef INTRINSIC_ESTIMATION_ESKF_STATE_H_
#define INTRINSIC_ESTIMATION_ESKF_STATE_H_

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

// ESKF layout only (#81). Index order, dimensions, defaults. No propagation,
// update, gating, injection, or reset. See ESKF_LAYOUT.md.

namespace intrinsic::estimation {

inline constexpr int kNominalDim = 16;
inline constexpr int kErrorDim = 15;
inline constexpr int kCovDim = 15;

// Nominal state indices (EskfNominal::ToArray order).
inline constexpr int kNominalPEnu = 0;  // 0..2
inline constexpr int kNominalQWxyz = 3;  // 3..6
inline constexpr int kNominalVBody = 7;  // 7..9
inline constexpr int kNominalBa = 10;  // 10..12
inline constexpr int kNominalBg = 13;  // 13..15

// Error state indices (EskfError::ToArray order).
inline constexpr int kErrorDp = 0;  // 0..2
inline constexpr int kErrorDtheta = 3;  // 3..5
inline constexpr int kErrorDv = 6;  // 6..8
inline constexpr int kErrorDba = 9;  // 9..11
inline constexpr int kErrorDbg = 12;  // 12..14

// One documented index range of a state vector.
struct EskfField {
  std::string_view symbol;
  int first;
  int count;
  std::string_view unit;
};

inline constexpr std::array<EskfField, 5> kNominalFields = {{
    {"p_enu", kNominalPEnu, 3, "m"},
    {"q_wxyz", kNominalQWxyz, 4, "1"},
    {"v_body", kNominalVBody, 3, "m/s"},
    {"b_a", kNominalBa, 3, "m/s^2"},
    {"b_g", kNominalBg, 3, "rad/s"},
}};

inline constexpr std::array<EskfField, 5> kErrorFields = {{
    {"dp", kErrorDp, 3, "m"},
    {"dtheta", kErrorDtheta, 3, "rad"},
    {"dv", kErrorDv, 3, "m/s"},
    {"dba", kErrorDba, 3, "m/s^2"},
    {"dbg", kErrorDbg, 3, "rad/s"},
}};

static_assert(kNominalDim == 16, "nominal state is 16 scalars");
static_assert(kErrorDim == 15, "attitude error is 3, not 4");
static_assert(kCovDim == kErrorDim, "P is indexed by the error state");
static_assert(kNominalBg + 3 == kNominalDim, "nominal layout is contiguous");
static_assert(kErrorDbg + 3 == kErrorDim, "error layout is contiguous");

// Nominal state. SI units. Position in world ENU, attitude body->world as a
// unit quaternion in w, x, y, z order, velocity and biases in the body frame.
// Default construction is deterministic: p = 0, q = (1, 0, 0, 0), rest 0.
struct EskfNominal {
  std::array<double, 3> p_enu = {0.0, 0.0, 0.0};
  std::array<double, 4> q_wxyz = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> v_body = {0.0, 0.0, 0.0};
  std::array<double, 3> b_a = {0.0, 0.0, 0.0};
  std::array<double, 3> b_g = {0.0, 0.0, 0.0};

  std::array<double, kNominalDim> ToArray() const;
  // Returns nullopt unless values.size() == kNominalDim.
  static std::optional<EskfNominal> FromVector(
      const std::vector<double>& values);
};

// Error state (15 scalars). Default is all zeros.
struct EskfError {
  std::array<double, 3> dp = {0.0, 0.0, 0.0};
  std::array<double, 3> dtheta = {0.0, 0.0, 0.0};
  std::array<double, 3> dv = {0.0, 0.0, 0.0};
  std::array<double, 3> dba = {0.0, 0.0, 0.0};
  std::array<double, 3> dbg = {0.0, 0.0, 0.0};

  std::array<double, kErrorDim> ToArray() const;
  // Returns nullopt unless values.size() == kErrorDim.
  static std::optional<EskfError> FromVector(
      const std::vector<double>& values);
};

// 15x15 covariance P, row-major, error-state ordering. A default-constructed
// value is unknown (no storage). Unknown is never the all-zero matrix.
class EskfCovariance {
 public:
  EskfCovariance() = default;

  // Known diagonal covariance for tests. Not the default.
  static EskfCovariance Identity();
  // Returns nullopt unless values.size() == kCovDim * kCovDim.
  static std::optional<EskfCovariance> FromRowMajor(
      const std::vector<double>& values);

  bool has_value() const { return data_.has_value(); }
  // Requires has_value().
  double At(int row, int col) const;
  const std::array<double, kCovDim * kCovDim>& row_major() const {
    return *data_;
  }

 private:
  std::optional<std::array<double, kCovDim * kCovDim>> data_;
};

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_ESKF_STATE_H_
