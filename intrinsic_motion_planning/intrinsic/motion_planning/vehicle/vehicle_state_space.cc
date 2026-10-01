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

#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"

#include <cmath>
#include <limits>

#include "Eigen/Geometry"
#include "intrinsic/eigenmath/interpolation.h"
#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

// Orientations that are already unit to within a few ulps are copied
// verbatim so that endpoint interpolation is exact.
constexpr double kExactUnitTolerance = 1e-15;

bool IsFinite(const VehiclePlanningState& state) {
  return embodiment::IsFinite(state.position) &&
         embodiment::IsFinite(state.orientation) && IsFinite(state.twist);
}

bool IsUnit(embodiment::Quaternion q) {
  return embodiment::IsNormalized(q, kQuaternionNormTolerance);
}

embodiment::Quaternion UnitCopy(embodiment::Quaternion q) {
  const double norm = embodiment::QuaternionNorm(q);
  if (std::abs(norm - 1.0) <= kExactUnitTolerance) {
    return q;
  }
  return embodiment::Quaternion{q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

double Lerp(double u, double a, double b) { return (1.0 - u) * a + u * b; }

double Norm3(double x, double y, double z) {
  return std::sqrt((x * x) + (y * y) + (z * z));
}

bool SpeedBoundValid(bool present, double value) {
  return !present || (std::isfinite(value) && value >= 0.0);
}

bool PositionBoundsValid(const VehicleStateBounds& bounds) {
  if (!bounds.position_limits_present) {
    return true;
  }
  const embodiment::Vec3& lo = bounds.position_min;
  const embodiment::Vec3& hi = bounds.position_max;
  if (std::isnan(lo.x) || std::isnan(lo.y) || std::isnan(lo.z) ||
      std::isnan(hi.x) || std::isnan(hi.y) || std::isnan(hi.z)) {
    return false;
  }
  return lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z;
}

bool InRange(double value, double lo, double hi) {
  return value >= lo && value <= hi;
}

}  // namespace

InterpolateResult Interpolate(const VehiclePlanningState& a,
                              const VehiclePlanningState& b, double u) {
  InterpolateResult result;
  // Negated form so that NaN is rejected.
  if (!(u >= 0.0 && u <= 1.0)) {
    result.error = StateSpaceError::kMixParameter;
    return result;
  }
  if (!IsFinite(a) || !IsFinite(b)) {
    result.error = StateSpaceError::kNonFinite;
    return result;
  }
  if (!IsUnit(a.orientation) || !IsUnit(b.orientation)) {
    result.error = StateSpaceError::kQuaternion;
    return result;
  }

  VehiclePlanningState out;
  out.position = embodiment::Vec3{Lerp(u, a.position.x, b.position.x),
                                  Lerp(u, a.position.y, b.position.y),
                                  Lerp(u, a.position.z, b.position.z)};
  out.twist = BodyVector{Lerp(u, a.twist.linear_x, b.twist.linear_x),
                         Lerp(u, a.twist.linear_y, b.twist.linear_y),
                         Lerp(u, a.twist.linear_z, b.twist.linear_z),
                         Lerp(u, a.twist.angular_x, b.twist.angular_x),
                         Lerp(u, a.twist.angular_y, b.twist.angular_y),
                         Lerp(u, a.twist.angular_z, b.twist.angular_z)};

  if (u == 0.0) {
    out.orientation = UnitCopy(a.orientation);
  } else if (u == 1.0) {
    out.orientation = UnitCopy(b.orientation);
  } else {
    const Eigen::Quaterniond qa(a.orientation.w, a.orientation.x,
                                a.orientation.y, a.orientation.z);
    const Eigen::Quaterniond qb(b.orientation.w, b.orientation.x,
                                b.orientation.y, b.orientation.z);
    const Eigen::Quaterniond q = eigenmath::Interpolate(u, qa, qb);
    out.orientation =
        UnitCopy(embodiment::Quaternion{q.x(), q.y(), q.z(), q.w()});
  }

  if (!IsFinite(out)) {
    result.error = StateSpaceError::kNonFinite;
    return result;
  }
  result.state = out;
  return result;
}

double Distance(const VehiclePlanningState& a, const VehiclePlanningState& b) {
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  if (!IsFinite(a) || !IsFinite(b) || !IsUnit(a.orientation) ||
      !IsUnit(b.orientation)) {
    return kInfinity;
  }

  const double dx = b.position.x - a.position.x;
  const double dy = b.position.y - a.position.y;
  const double dz = b.position.z - a.position.z;
  const double position_sq = (dx * dx) + (dy * dy) + (dz * dz);

  // conj(a) * b has scalar part dot(a, b) and vector part
  // a.w * b.v - b.w * a.v - a.v x b.v. Swapping a and b negates the vector
  // part exactly, so theta is exactly symmetric. atan2 stays accurate for
  // small angles, unlike acos(|dot|).
  const embodiment::Quaternion& qa = a.orientation;
  const embodiment::Quaternion& qb = b.orientation;
  const double dot =
      (qa.x * qb.x) + (qa.y * qb.y) + (qa.z * qb.z) + (qa.w * qb.w);
  const double vx = (qa.w * qb.x - qb.w * qa.x) - (qa.y * qb.z - qa.z * qb.y);
  const double vy = (qa.w * qb.y - qb.w * qa.y) - (qa.z * qb.x - qa.x * qb.z);
  const double vz = (qa.w * qb.z - qb.w * qa.z) - (qa.x * qb.y - qa.y * qb.x);
  const double theta = 2.0 * std::atan2(Norm3(vx, vy, vz), std::abs(dot));

  const double lx = b.twist.linear_x - a.twist.linear_x;
  const double ly = b.twist.linear_y - a.twist.linear_y;
  const double lz = b.twist.linear_z - a.twist.linear_z;
  const double wx = b.twist.angular_x - a.twist.angular_x;
  const double wy = b.twist.angular_y - a.twist.angular_y;
  const double wz = b.twist.angular_z - a.twist.angular_z;
  const double linear_sq = (lx * lx) + (ly * ly) + (lz * lz);
  const double angular_sq = (wx * wx) + (wy * wy) + (wz * wz);

  const double distance =
      std::sqrt(position_sq + (theta * theta) + linear_sq + angular_sq);
  return std::isfinite(distance) ? distance : kInfinity;
}

StateSpaceError Validate(const VehiclePlanningState& state,
                         const VehicleStateBounds& bounds) {
  if (!PositionBoundsValid(bounds) ||
      !SpeedBoundValid(bounds.max_linear_speed_present,
                       bounds.max_linear_speed_m_s) ||
      !SpeedBoundValid(bounds.max_angular_speed_present,
                       bounds.max_angular_speed_rad_s)) {
    return StateSpaceError::kBadBounds;
  }
  if (!IsFinite(state)) {
    return StateSpaceError::kNonFinite;
  }
  if (!IsUnit(state.orientation)) {
    return StateSpaceError::kQuaternion;
  }
  if (bounds.position_limits_present) {
    const embodiment::Vec3& p = state.position;
    const embodiment::Vec3& lo = bounds.position_min;
    const embodiment::Vec3& hi = bounds.position_max;
    if (!InRange(p.x, lo.x, hi.x) || !InRange(p.y, lo.y, hi.y) ||
        !InRange(p.z, lo.z, hi.z)) {
      return StateSpaceError::kBounds;
    }
  }
  if (bounds.max_linear_speed_present) {
    const double speed =
        Norm3(state.twist.linear_x, state.twist.linear_y, state.twist.linear_z);
    if (!(speed <= bounds.max_linear_speed_m_s)) {
      return StateSpaceError::kBounds;
    }
  }
  if (bounds.max_angular_speed_present) {
    const double rate = Norm3(state.twist.angular_x, state.twist.angular_y,
                              state.twist.angular_z);
    if (!(rate <= bounds.max_angular_speed_rad_s)) {
      return StateSpaceError::kBounds;
    }
  }
  return StateSpaceError::kOk;
}

}  // namespace intrinsic::motion_planning::vehicle
