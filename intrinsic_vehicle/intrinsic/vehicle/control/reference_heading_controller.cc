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

#include "intrinsic/vehicle/control/reference_heading_controller.h"

#include <array>
#include <cmath>
#include <string_view>

namespace intrinsic::vehicle::control {
namespace {

// Contract revision 1.
constexpr double kProportionalGain = 10.0;
constexpr double kIntegralGain = 0.3;
constexpr double kDerivativeGain = 8.0;
constexpr double kAntiWindupGain = 0.2;
constexpr double kYawTorqueMinNm = -15.0;
constexpr double kYawTorqueMaxNm = 15.0;
constexpr double kIntegratorMin = -12.0;
constexpr double kIntegratorMax = 12.0;

// torque_n_m[2] is spatial yaw, guidance::kYaw.
constexpr int kYawTorque = 2;

constexpr std::string_view kMissingYawMessage =
    "reference has no yaw objective";
constexpr std::string_view kStateEnuMessage =
    "state pose frame must be world_enu";
constexpr std::string_view kReferenceEnuMessage =
    "reference pose frame must be world_enu";
constexpr std::string_view kYawFiniteMessage = "yaw must be finite";
constexpr std::string_view kYawRateMessage = "yaw rate must be finite";
constexpr std::string_view kYawTorqueMessage = "yaw torque must be finite";

double Clamp(double value, double lower, double upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

// ENU yaw of body +x. Hamilton quaternion, stored x, y, z, w, active
// body-to-navigation map. atan2(R10, R00) is the heading of that axis
// in the horizontal plane. This package does not convert NED.
double YawRad(const std::array<double, 4>& quaternion) {
  const double x = quaternion[guidance::kQuatX];
  const double y = quaternion[guidance::kQuatY];
  const double z = quaternion[guidance::kQuatZ];
  const double w = quaternion[guidance::kQuatW];
  const double r10 = 2.0 * (w * z + x * y);
  const double r00 = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(r10, r00);
}

StatusOr<BodyWrenchRt> Reject(ControlStatus status) {
  return StatusOr<BodyWrenchRt>::Failure(status);
}

}  // namespace

ReferenceHeadingController::ReferenceHeadingController() { Reset(); }

void ReferenceHeadingController::Reset() {
  integrator_ =
      BoundedIntegrator::Create(0.0, kIntegratorMin, kIntegratorMax).value();
}

double ReferenceHeadingController::integrator_state() const {
  return integrator_.state();
}

StatusOr<BodyWrenchRt> ReferenceHeadingController::Evaluate(
    const MotionReferenceRt& reference, const VehicleStateRt& state,
    Duration update_period) const {
  const ControlStatus inputs =
      ValidateControlInputs(reference, state, update_period);
  if (!inputs.ok()) {
    return Reject(inputs);
  }
  if (!reference.has_pose) {
    return Reject(ControlStatus::MissingObjective(kMissingYawMessage));
  }
  if (state.pose_frame != FrameId::kWorldEnu) {
    return Reject(ControlStatus::Invalid(kStateEnuMessage));
  }
  if (reference.pose_frame != FrameId::kWorldEnu) {
    return Reject(ControlStatus::Invalid(kReferenceEnuMessage));
  }

  const double yaw_cmd = YawRad(reference.orientation_xyzw);
  const double yaw_meas = YawRad(state.orientation_xyzw);
  const double yaw_rate = state.body_twist[guidance::kYaw];
  if (!std::isfinite(yaw_cmd) || !std::isfinite(yaw_meas)) {
    return Reject(ControlStatus::InvalidArgument(kYawFiniteMessage));
  }
  if (!std::isfinite(yaw_rate)) {
    return Reject(ControlStatus::InvalidArgument(kYawRateMessage));
  }

  const ControlMathResult<double> error = HeadingError(yaw_cmd, yaw_meas);
  if (!error.ok()) {
    return Reject(ControlStatus::InvalidArgument(error.status().message));
  }

  const double integral_torque = integrator_.state();
  const double u_unsat = kProportionalGain * error.value() + integral_torque -
                         kDerivativeGain * yaw_rate;
  if (!std::isfinite(u_unsat)) {
    return Reject(ControlStatus::InvalidArgument(kYawTorqueMessage));
  }
  const double u_sat = Clamp(u_unsat, kYawTorqueMinNm, kYawTorqueMaxNm);
  const ControlMathResult<double> stepped = IntegrateBackCalculation(
      integrator_, kIntegralGain * error.value(), u_unsat, u_sat,
      kAntiWindupGain, update_period.seconds);
  if (!stepped.ok()) {
    return Reject(ControlStatus::InvalidArgument(stepped.status().message));
  }

  BodyWrenchRt wrench;
  wrench.frame = FrameId::kBody;
  wrench.torque_n_m[kYawTorque] = u_sat;
  return StatusOr<BodyWrenchRt>::Ok(wrench);
}

}  // namespace intrinsic::vehicle::control
