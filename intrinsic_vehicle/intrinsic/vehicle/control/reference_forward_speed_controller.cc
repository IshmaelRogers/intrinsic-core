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

#include "intrinsic/vehicle/control/reference_forward_speed_controller.h"

#include <cmath>
#include <string_view>

namespace intrinsic::vehicle::control {
namespace {

// Contract revision 2. Saturation is the Lapris surge force scale.
// Gains are the revision 1 estimates.
constexpr double kProportionalGain = 50.0;
constexpr double kIntegralGain = 5.0;
constexpr double kAntiWindupGain = 0.2;
constexpr double kSurgeMinN = -60.0;
constexpr double kSurgeMaxN = 60.0;
constexpr double kIntegratorMin = -48.0;
constexpr double kIntegratorMax = 48.0;

constexpr std::string_view kMissingSurgeMessage =
    "reference has no surge objective";
constexpr std::string_view kTwistFrameMessage =
    "reference twist frame must be body";
constexpr std::string_view kSurgeSpeedMessage = "surge speed must be finite";
constexpr std::string_view kSurgeForceMessage = "surge command must be finite";

double Clamp(double value, double lower, double upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

StatusOr<BodyWrenchRt> Reject(ControlStatus status) {
  return StatusOr<BodyWrenchRt>::Failure(status);
}

}  // namespace

ReferenceForwardSpeedController::ReferenceForwardSpeedController() { Reset(); }

void ReferenceForwardSpeedController::Reset() {
  integrator_ =
      BoundedIntegrator::Create(0.0, kIntegratorMin, kIntegratorMax).value();
}

double ReferenceForwardSpeedController::integrator_state() const {
  return integrator_.state();
}

StatusOr<BodyWrenchRt> ReferenceForwardSpeedController::Evaluate(
    const MotionReferenceRt& reference, const VehicleStateRt& state,
    Duration update_period) const {
  const ControlStatus inputs =
      ValidateControlInputs(reference, state, update_period);
  if (!inputs.ok()) {
    return Reject(inputs);
  }
  if (!reference.has_twist) {
    return Reject(ControlStatus::MissingObjective(kMissingSurgeMessage));
  }
  if (reference.twist_frame != FrameId::kBody) {
    return Reject(ControlStatus::Invalid(kTwistFrameMessage));
  }

  const double u_cmd = reference.body_twist[guidance::kSurge];
  const double u_meas = state.body_twist[guidance::kSurge];
  if (!std::isfinite(u_cmd) || !std::isfinite(u_meas)) {
    return Reject(ControlStatus::InvalidArgument(kSurgeSpeedMessage));
  }

  const double error = u_cmd - u_meas;
  const double integral_force = integrator_.state();
  const double u_unsat = kProportionalGain * error + integral_force;
  if (!std::isfinite(u_unsat)) {
    return Reject(ControlStatus::InvalidArgument(kSurgeForceMessage));
  }
  const double u_sat = Clamp(u_unsat, kSurgeMinN, kSurgeMaxN);
  const ControlMathResult<double> stepped =
      IntegrateBackCalculation(integrator_, kIntegralGain * error, u_unsat,
                               u_sat, kAntiWindupGain, update_period.seconds);
  if (!stepped.ok()) {
    return Reject(ControlStatus::InvalidArgument(stepped.status().message));
  }

  BodyWrenchRt wrench;
  wrench.frame = FrameId::kBody;
  wrench.force_n[guidance::kSurge] = u_sat;
  return StatusOr<BodyWrenchRt>::Ok(wrench);
}

}  // namespace intrinsic::vehicle::control
