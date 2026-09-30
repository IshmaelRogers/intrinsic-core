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

#include "intrinsic/vehicle/control/reference_depth_controller.h"

#include <array>
#include <cmath>
#include <string_view>

namespace intrinsic::vehicle::control {
namespace {

// Contract revision 1.
constexpr double kProportionalGain = 20.0;
constexpr double kIntegralGain = 0.2;
constexpr double kDerivativeGain = 45.0;
constexpr double kAntiWindupGain = 0.2;
constexpr double kHeaveMinN = -50.0;
constexpr double kHeaveMaxN = 50.0;
constexpr double kIntegratorMin = -40.0;
constexpr double kIntegratorMax = 40.0;

constexpr std::string_view kMissingDepthMessage =
    "reference has no depth objective";
constexpr std::string_view kStateEnuMessage =
    "state pose frame must be world_enu";
constexpr std::string_view kReferenceEnuMessage =
    "reference pose frame must be world_enu";
constexpr std::string_view kDepthFiniteMessage = "depth must be finite";
constexpr std::string_view kDepthRateMessage = "depth rate must be finite";
constexpr std::string_view kHeaveMessage = "heave command must be finite";

double Clamp(double value, double lower, double upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

// ENU up component of a body linear twist. Hamilton quaternion, stored
// x, y, z, w, active body-to-navigation map. The third row matches the
// map used to integrate a body twist into a world pose elsewhere in
// this tree. This package does not convert NED.
double WorldUpVelocity(const std::array<double, 4>& quaternion,
                       const std::array<double, guidance::kSpatialDof>& twist) {
  const double x = quaternion[guidance::kQuatX];
  const double y = quaternion[guidance::kQuatY];
  const double z = quaternion[guidance::kQuatZ];
  const double w = quaternion[guidance::kQuatW];
  const double r20 = 2.0 * (x * z - w * y);
  const double r21 = 2.0 * (y * z + w * x);
  const double r22 = 1.0 - 2.0 * (x * x + y * y);
  return r20 * twist[guidance::kSurge] + r21 * twist[guidance::kSway] +
         r22 * twist[guidance::kHeave];
}

StatusOr<BodyWrenchRt> Reject(ControlStatus status) {
  return StatusOr<BodyWrenchRt>::Failure(status);
}

}  // namespace

ReferenceDepthController::ReferenceDepthController() { Reset(); }

void ReferenceDepthController::Reset() {
  integrator_ =
      BoundedIntegrator::Create(0.0, kIntegratorMin, kIntegratorMax).value();
}

double ReferenceDepthController::integrator_state() const {
  return integrator_.state();
}

StatusOr<BodyWrenchRt> ReferenceDepthController::Evaluate(
    const MotionReferenceRt& reference, const VehicleStateRt& state,
    Duration update_period) const {
  const ControlStatus inputs =
      ValidateControlInputs(reference, state, update_period);
  if (!inputs.ok()) {
    return Reject(inputs);
  }
  if (!reference.has_pose) {
    return Reject(ControlStatus::MissingObjective(kMissingDepthMessage));
  }
  if (state.pose_frame != FrameId::kWorldEnu) {
    return Reject(ControlStatus::Invalid(kStateEnuMessage));
  }
  if (reference.pose_frame != FrameId::kWorldEnu) {
    return Reject(ControlStatus::Invalid(kReferenceEnuMessage));
  }

  const double depth_cmd = -reference.position_m[guidance::kZ];
  const double depth_meas = -state.position_m[guidance::kZ];
  const double v_depth =
      -WorldUpVelocity(state.orientation_xyzw, state.body_twist);
  if (!std::isfinite(depth_cmd) || !std::isfinite(depth_meas)) {
    return Reject(ControlStatus::InvalidArgument(kDepthFiniteMessage));
  }
  if (!std::isfinite(v_depth)) {
    return Reject(ControlStatus::InvalidArgument(kDepthRateMessage));
  }

  const double error = depth_cmd - depth_meas;
  const double integral_force = integrator_.state();
  const double u_unsat =
      -kProportionalGain * error + integral_force + kDerivativeGain * v_depth;
  if (!std::isfinite(u_unsat)) {
    return Reject(ControlStatus::InvalidArgument(kHeaveMessage));
  }
  const double u_sat = Clamp(u_unsat, kHeaveMinN, kHeaveMaxN);
  const ControlMathResult<double> stepped =
      IntegrateBackCalculation(integrator_, -kIntegralGain * error, u_unsat,
                               u_sat, kAntiWindupGain, update_period.seconds);
  if (!stepped.ok()) {
    return Reject(ControlStatus::InvalidArgument(stepped.status().message));
  }

  BodyWrenchRt wrench;
  wrench.frame = FrameId::kBody;
  wrench.force_n[guidance::kHeave] = u_sat;
  return StatusOr<BodyWrenchRt>::Ok(wrench);
}

}  // namespace intrinsic::vehicle::control
