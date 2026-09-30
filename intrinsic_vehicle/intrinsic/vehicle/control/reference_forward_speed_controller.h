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

// Reference forward-speed controller. Surge force only.
//
// Forward speed is body surge, meters per second, REP-103 +x. Surge
// force is body-wrench index 0, newtons. A positive speed error
// produces positive surge. Sway, heave, and every torque stay zero.
// Depth and heading are not controlled.
//
// The policy is ground-relative body surge. u_cmd is
// reference.body_twist[kSurge] when the reference has a twist. u_meas
// is state.body_twist[kSurge]. VehicleStateRt has no water-current
// field. This controller does not subtract a current and does not call
// a relative-velocity helper. A caller that wants current-relative
// speed presents that speed in state.body_twist[kSurge].
//
// Contract revision 2:
//
//   e = u_cmd - u_meas
//   u_unsat = k_p * e + S
//   u = clamp(u_unsat, -60, 60)
//
// k_p is 50 N/(m/s) and k_i is 5 N/(m/s)/s. There is no derivative
// term. reference.twist_frame must be kBody.
//
// S is the integral force in newtons, owned by this controller and
// clamped to [-48, 48]. Reset() sets S to 0. Away from the clamp and
// from saturation, S = k_i * integral(e dt), so the command is
// k_p * e + k_i * I. S is advanced by IntegrateBackCalculation. The
// integrator input is k_i * e and k_aw is 0.2, the gain on
// (u_sat - u_unsat). A rejected Evaluate does not write S.
//
// Evaluate stays const. S is a mutable member. One instance is not safe
// for concurrent Evaluate or Reset calls. The caller owns the
// reference, the state, and the result. Evaluate does not retain them
// and does not allocate. Status text is a static string view.
//
// On any status other than kOk the returned wrench is finite zeros and
// is not a command. Checks start with ValidateControlInputs. A missing
// twist, including a pose-only reference, is kMissingObjective. A
// non-body twist frame is kInvalid. A non-finite surge speed or surge
// command is kInvalidArgument. Snapshot mismatch is kStale. This class
// does not write an actuator command and does not run on the ICON
// cycle.

#ifndef INTRINSIC_VEHICLE_CONTROL_REFERENCE_FORWARD_SPEED_CONTROLLER_H_
#define INTRINSIC_VEHICLE_CONTROL_REFERENCE_FORWARD_SPEED_CONTROLLER_H_

#include "intrinsic/vehicle/control/control_math.h"
#include "intrinsic/vehicle/control/reference_control.h"

namespace intrinsic::vehicle::control {

class ReferenceForwardSpeedController final : public ReferenceController {
 public:
  ReferenceForwardSpeedController();

  // Sets the integral force to 0.
  void Reset();

  // Integral force in newtons, inside [-48, 48].
  [[nodiscard]] double integrator_state() const;

  [[nodiscard]] StatusOr<BodyWrenchRt> Evaluate(
      const MotionReferenceRt& reference, const VehicleStateRt& state,
      Duration update_period) const override;

 private:
  mutable BoundedIntegrator integrator_;
};

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_REFERENCE_FORWARD_SPEED_CONTROLLER_H_
