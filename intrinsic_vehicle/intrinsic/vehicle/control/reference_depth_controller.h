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

// Reference depth controller. Heave only.
//
// Depth is meters below the surface, positive down. The world frame is
// ENU, so depth_m = -z_world. Heave is body-wrench index 2, newtons,
// +z up. A positive depth error (the vehicle is too shallow) produces
// negative heave. Surge and every torque stay zero. Heading and
// forward speed are not controlled.
//
// Contract revision 1:
//
//   e = depth_cmd_m - depth_meas_m
//   v_depth = -v_z_world
//   u_unsat = -k_p * e + S + k_d * v_depth
//   u = clamp(u_unsat, -50, 50)
//
// v_z_world is the ENU up component of the body linear twist, using the
// state orientation. k_p is 20 N/m, k_i is 0.2 N/(m*s), and k_d is
// 45 N/(m/s). The derivative term opposes positive-down rate, so a
// descent produces positive heave. A positive depth error produces
// negative heave.
//
// S is the integral force in newtons, owned by this controller and
// clamped to [-40, 40]. Reset() sets S to 0. Away from the clamp and
// from saturation, S = -k_i * integral(e dt), so the command is
// -k_p * e - k_i * I + k_d * v_depth. S is advanced by
// IntegrateBackCalculation. The integrator input is -k_i * e and k_aw
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
// A non-finite depth, rate, or heave command is kInvalidArgument.
// Snapshot mismatch is kStale. This class does not write an actuator
// command and does not run on the ICON cycle.

#ifndef INTRINSIC_VEHICLE_CONTROL_REFERENCE_DEPTH_CONTROLLER_H_
#define INTRINSIC_VEHICLE_CONTROL_REFERENCE_DEPTH_CONTROLLER_H_

#include "intrinsic/vehicle/control/control_math.h"
#include "intrinsic/vehicle/control/reference_control.h"

namespace intrinsic::vehicle::control {

class ReferenceDepthController final : public ReferenceController {
 public:
  ReferenceDepthController();

  // Sets the integral force to 0.
  void Reset();

  // Integral force in newtons, inside [-40, 40].
  [[nodiscard]] double integrator_state() const;

  [[nodiscard]] StatusOr<BodyWrenchRt> Evaluate(
      const MotionReferenceRt& reference, const VehicleStateRt& state,
      Duration update_period) const override;

 private:
  mutable BoundedIntegrator integrator_;
};

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_REFERENCE_DEPTH_CONTROLLER_H_
