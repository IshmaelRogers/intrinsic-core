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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_STATE_SPACE_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_STATE_SPACE_H_

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Plain-value vehicle StateSpace: Interpolate, Distance, and Validate. There
// is no search, sampling, collision checking, or World access here. Inputs
// are never renormalized or converted between ENU and NED.

// Linear x, y, z then angular x, y, z, as in BodyTwist (REP-103 body axes).
using BodyVector = ::intrinsic::vehicle::BodyVector;

// Planning sample. This is not the estimator VehicleState wire message: no
// covariance, mode, sources, epoch, header, or acceleration.
struct VehiclePlanningState {
  // World-frame pose. Position in meters. Orientation is Hamilton (x, y, z, w)
  // with identity w = 1.
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;

  // Body-frame twist. Planning states always carry twist; use zeros when the
  // caller has no velocity.
  BodyVector twist;
};

// Absent limits do not constrain that axis.
struct VehicleStateBounds {
  bool position_limits_present = false;
  embodiment::Vec3 position_min;  // Inclusive when present.
  embodiment::Vec3 position_max;  // Inclusive when present.

  bool max_linear_speed_present = false;
  double max_linear_speed_m_s = 0;  // >= 0 and finite when present.

  bool max_angular_speed_present = false;
  double max_angular_speed_rad_s = 0;  // >= 0 and finite when present.
};

enum class StateSpaceError {
  kOk = 0,
  kMixParameter = 1,  // u not in [0, 1].
  kNonFinite = 2,
  kQuaternion = 3,  // Non-unit input orientation.
  kBounds = 4,      // Failed Validate against engaged limits.
  kBadBounds = 5,   // The bounds configuration itself is invalid.
};

struct InterpolateResult {
  StateSpaceError error = StateSpaceError::kOk;
  VehiclePlanningState state;  // Meaningful only when error == kOk.
};

// Tolerance on |norm(q) - 1| for input orientations. Same value as
// embodiment::IsNormalized.
inline constexpr double kQuaternionNormTolerance = 1e-9;

// Interpolates from a (u = 0) to b (u = 1). Check order: u, finiteness, unit
// orientations. The first defect wins.
//
// Position and twist are linear: (1 - u) * a + u * b. Orientation is the
// shortest-path slerp of intrinsic::eigenmath::Interpolate. The output
// orientation is unit length and lies in a's hemisphere, except that u == 1
// returns b exactly. u == 0 returns a exactly.
InterpolateResult Interpolate(const VehiclePlanningState& a,
                              const VehiclePlanningState& b, double u);

// sqrt(|dp|^2 + theta^2 + |dv_lin|^2 + |dw|^2) with all weights 1.0. theta is
// the geodesic angle in [0, pi] of the shortest rotation from a to b.
// Returns +infinity when either state is non-finite or has a non-unit
// orientation. Never throws.
double Distance(const VehiclePlanningState& a, const VehiclePlanningState& b);

// Check order: bounds configuration, finiteness, unit orientation, position
// box, linear speed, angular speed. The first defect wins. Default bounds
// check finiteness and unit orientation only.
StateSpaceError Validate(const VehiclePlanningState& state,
                         const VehicleStateBounds& bounds);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_STATE_SPACE_H_
