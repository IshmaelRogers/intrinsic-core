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

#ifndef INTRINSIC_EMBODIMENT_FRAME_POLICY_H_
#define INTRINSIC_EMBODIMENT_FRAME_POLICY_H_

#include <cmath>
#include <numbers>
#include <string_view>

namespace intrinsic::embodiment {

// Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.
//
// Quaternion storage is Hamilton (x, y, z, w), matching
// intrinsic_proto.Quaternion. These helpers convert world frames only.
// Body axes stay REP-103 (x forward, y left, z up).

inline constexpr std::string_view kWorldEnuFrameId = "world_enu";
inline constexpr std::string_view kWorldNedFrameId = "world_ned";

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

enum class WorldFrame { kEnu, kNed };

inline constexpr std::string_view WorldFrameId(WorldFrame frame) {
  return frame == WorldFrame::kEnu ? kWorldEnuFrameId : kWorldNedFrameId;
}

// True only for an exact well-known world id. Does not infer a frame.
inline constexpr bool FrameIdMatches(std::string_view frame_id,
                                     WorldFrame expected) {
  return frame_id == WorldFrameId(expected);
}

inline bool IsFinite(double value) { return std::isfinite(value); }

inline bool IsFinite(Vec3 value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

inline bool IsFinite(Quaternion value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) &&
         IsFinite(value.w);
}

inline double QuaternionNorm(Quaternion value) {
  return std::sqrt((value.x * value.x) + (value.y * value.y) +
                   (value.z * value.z) + (value.w * value.w));
}

// Unit Hamilton quaternions only. Non-finite input is not normalized.
inline bool IsNormalized(Quaternion value, double tolerance = 1e-9) {
  if (!IsFinite(value)) {
    return false;
  }
  return std::abs(QuaternionNorm(value) - 1.0) <= tolerance;
}

// a ~= b or a ~= -b. Both signs are the same rotation.
inline bool QuaternionsEquivalent(Quaternion lhs, Quaternion rhs,
                                  double tolerance = 1e-9) {
  const bool same = std::abs(lhs.x - rhs.x) <= tolerance &&
                    std::abs(lhs.y - rhs.y) <= tolerance &&
                    std::abs(lhs.z - rhs.z) <= tolerance &&
                    std::abs(lhs.w - rhs.w) <= tolerance;
  const bool flipped = std::abs(lhs.x + rhs.x) <= tolerance &&
                       std::abs(lhs.y + rhs.y) <= tolerance &&
                       std::abs(lhs.z + rhs.z) <= tolerance &&
                       std::abs(lhs.w + rhs.w) <= tolerance;
  return same || flipped;
}

// Hamilton product. q_a_from_c = HamiltonProduct(q_a_from_b, q_b_from_c).
inline Quaternion HamiltonProduct(Quaternion lhs, Quaternion rhs) {
  return Quaternion{
      (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
      (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x),
      (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w),
      (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
  };
}

// Active rotation. Caller supplies a normalized quaternion.
inline Vec3 RotateVector(Quaternion rotation, Vec3 vector) {
  const Quaternion pure{vector.x, vector.y, vector.z, 0.0};
  const Quaternion conjugate{-rotation.x, -rotation.y, -rotation.z, rotation.w};
  const Quaternion rotated =
      HamiltonProduct(HamiltonProduct(rotation, pure), conjugate);
  return Vec3{rotated.x, rotated.y, rotated.z};
}

// World ENU -> world NED. The map is an involution.
inline Vec3 WorldVectorEnuToNed(Vec3 enu) { return Vec3{enu.y, enu.x, -enu.z}; }

inline Vec3 WorldVectorNedToEnu(Vec3 ned) { return Vec3{ned.y, ned.x, -ned.z}; }

// Orientation of the NED world axes in the ENU world frame.
// (x, y, z, w) = (sqrt(2)/2, sqrt(2)/2, 0, 0).
inline Quaternion NedFromEnuWorldRotation() {
  constexpr double kHalfSqrt2 = std::numbers::sqrt2 / 2.0;
  return Quaternion{kHalfSqrt2, kHalfSqrt2, 0.0, 0.0};
}

// World-from-body orientation. Body axes are unchanged.
inline Quaternion WorldOrientationEnuToNed(Quaternion q_enu_from_body) {
  return HamiltonProduct(NedFromEnuWorldRotation(), q_enu_from_body);
}

inline Quaternion WorldOrientationNedToEnu(Quaternion q_ned_from_body) {
  return HamiltonProduct(NedFromEnuWorldRotation(), q_ned_from_body);
}

}  // namespace intrinsic::embodiment

#endif  // INTRINSIC_EMBODIMENT_FRAME_POLICY_H_
