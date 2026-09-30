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

// Reference heading controller. Yaw torque only.
//
// Heading is the ENU yaw of the body +x axis, radians, about world +z.
// Yaw torque is body-wrench torque index 2 (spatial index 5),
// newton-meters, +z up. A positive heading error produces positive yaw
// torque. Surge, sway, heave, roll, and pitch stay zero. Depth and
// forward speed are not controlled.
//
// Contract revision 1:
//
//   e = HeadingError(yaw_cmd, yaw_meas)  // (-pi, pi]
//   r = body yaw rate
//   u_unsat = k_p * e + S - k_d * r
//   u = clamp(u_unsat, -15, 15)
//
// Yaw is atan2 of the horizontal projection of body +x, using the
// Hamilton quaternion stored x, y, z, w as an active body-to-navigation
// map. This package does not convert NED. k_p is 10 N*m/rad, k_i is
// 0.3 N*m/(rad*s), and k_d is 8 N*m/(rad/s). The derivative term damps
// body yaw rate.
//
// S is the integral torque in newton-meters, owned by this controller
// and clamped to [-12, 12]. Reset() sets S to 0. Away from the clamp
// and from saturation, S = k_i * integral(e dt), so the command is
// k_p * e + k_i * I - k_d * r. S is advanced by
// IntegrateBackCalculation. The integrator input is k_i * e and k_aw
// is 0.2, the gain on (u_sat - u_unsat). A rejected Evaluate does not
// write S.
//
// Evaluate stays const. S is a mutable member. One instance is not safe
// for concurrent Evaluate or Reset calls. The caller owns the
// reference, the state, and the result. Evaluate does not retain them
// and does not allocate. Status text is a static string view.
//
// On any status other than kOk the returned wrench is finite zeros and
// is not a command. Checks start with ValidateControlInputs. A missing
// pose is kMissingObjective. A world frame other than ENU is kInvalid.
// A non-finite yaw, yaw rate, or yaw-torque command is
// kInvalidArgument. Snapshot mismatch is kStale. This class does not
// write an actuator command and does not run on the ICON cycle.

#ifndef INTRINSIC_VEHICLE_CONTROL_REFERENCE_HEADING_CONTROLLER_H_
#define INTRINSIC_VEHICLE_CONTROL_REFERENCE_HEADING_CONTROLLER_H_

#include "intrinsic/vehicle/control/control_math.h"
#include "intrinsic/vehicle/control/reference_control.h"

namespace intrinsic::vehicle::control {

class ReferenceHeadingController final : public ReferenceController {
 public:
  ReferenceHeadingController();

  // Sets the integral torque to 0.
  void Reset();

  // Integral torque in newton-meters, inside [-12, 12].
  [[nodiscard]] double integrator_state() const;

  [[nodiscard]] StatusOr<BodyWrenchRt> Evaluate(
      const MotionReferenceRt& reference, const VehicleStateRt& state,
      Duration update_period) const override;

 private:
  mutable BoundedIntegrator integrator_;
};

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_REFERENCE_HEADING_CONTROLLER_H_
