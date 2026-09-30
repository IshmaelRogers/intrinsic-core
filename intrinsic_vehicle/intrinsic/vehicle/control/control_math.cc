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

#include "intrinsic/vehicle/control/control_math.h"

#include <cmath>
#include <numbers>
#include <string_view>

namespace intrinsic::vehicle::control {
namespace {

constexpr std::string_view kAngleMessage = "angle must be finite";
constexpr std::string_view kWrappedAngleMessage = "wrapped angle is not finite";
constexpr std::string_view kHeadingMessage = "heading must be finite";
constexpr std::string_view kHeadingDifferenceMessage =
    "heading difference is not finite";
constexpr std::string_view kLimitsMessage =
    "integrator limits must be finite and ordered";
constexpr std::string_view kInitialStateMessage =
    "integrator state must be finite and inside the limits";
constexpr std::string_view kIntegratorInvalidMessage =
    "integrator is not valid";
constexpr std::string_view kInputMessage = "integrator input must be finite";
constexpr std::string_view kTimeStepMessage =
    "time step must be finite and greater than or equal to zero";
constexpr std::string_view kStepMessage = "integrator step is not finite";
constexpr std::string_view kBackCalculationMessage =
    "back-calculation arguments must be finite";
constexpr std::string_view kBackCalculationTermMessage =
    "back-calculation term is not finite";
constexpr std::string_view kBackCalculationSumMessage =
    "back-calculation integrator input is not finite";

using Result = ControlMathResult<double>;

Result Reject(std::string_view message, double value = 0) {
  return Result::Failure(ControlMathStatus::InvalidArgument(message), value);
}

double Clamp(double value, double lower, double upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

}  // namespace

ControlMathResult<double> ShortestSignedAngle(double delta_rad) {
  if (!std::isfinite(delta_rad)) {
    return Reject(kAngleMessage);
  }
  const double pi = std::numbers::pi;
  double wrapped = std::atan2(std::sin(delta_rad), std::cos(delta_rad));
  if (!std::isfinite(wrapped)) {
    return Reject(kWrappedAngleMessage);
  }
  // atan2 returns [-pi, pi]. The approved interval is (-pi, pi].
  if (wrapped <= -pi) {
    wrapped = pi;
  }
  if (!(wrapped > -pi && wrapped <= pi)) {
    return Reject(kWrappedAngleMessage);
  }
  return Result::Ok(wrapped);
}

ControlMathResult<double> HeadingError(double reference_rad,
                                       double measured_rad) {
  if (!std::isfinite(reference_rad) || !std::isfinite(measured_rad)) {
    return Reject(kHeadingMessage);
  }
  const double delta = reference_rad - measured_rad;
  if (!std::isfinite(delta)) {
    return Reject(kHeadingDifferenceMessage);
  }
  return ShortestSignedAngle(delta);
}

ControlMathResult<double> BackCalculation(double u_unsat, double u_sat,
                                          double k_aw) {
  if (!std::isfinite(u_unsat) || !std::isfinite(u_sat) ||
      !std::isfinite(k_aw)) {
    return Reject(kBackCalculationMessage);
  }
  const double difference = u_sat - u_unsat;
  if (!std::isfinite(difference)) {
    return Reject(kBackCalculationTermMessage);
  }
  const double term = k_aw * difference;
  if (!std::isfinite(term)) {
    return Reject(kBackCalculationTermMessage);
  }
  return Result::Ok(term);
}

ControlMathResult<BoundedIntegrator> BoundedIntegrator::Create(double initial,
                                                               double lower,
                                                               double upper) {
  if (!std::isfinite(lower) || !std::isfinite(upper) || !(lower <= upper)) {
    return ControlMathResult<BoundedIntegrator>::Failure(
        ControlMathStatus::InvalidArgument(kLimitsMessage));
  }
  if (!std::isfinite(initial) || initial < lower || initial > upper) {
    return ControlMathResult<BoundedIntegrator>::Failure(
        ControlMathStatus::InvalidArgument(kInitialStateMessage));
  }
  BoundedIntegrator integrator;
  integrator.valid_ = true;
  integrator.state_ = initial;
  integrator.lower_ = lower;
  integrator.upper_ = upper;
  return ControlMathResult<BoundedIntegrator>::Ok(integrator);
}

ControlMathResult<double> BoundedIntegrator::Integrate(double u, double dt) {
  if (!valid_) {
    return Reject(kIntegratorInvalidMessage, state_);
  }
  if (!std::isfinite(u)) {
    return Reject(kInputMessage, state_);
  }
  if (!std::isfinite(dt) || dt < 0) {
    return Reject(kTimeStepMessage, state_);
  }
  if (dt == 0) {
    return Result::Ok(state_);
  }
  const double product = u * dt;
  if (!std::isfinite(product)) {
    return Reject(kStepMessage, state_);
  }
  const double updated = state_ + product;
  if (!std::isfinite(updated)) {
    return Reject(kStepMessage, state_);
  }
  state_ = Clamp(updated, lower_, upper_);
  return Result::Ok(state_);
}

ControlMathResult<double> IntegrateBackCalculation(
    BoundedIntegrator& integrator, double integrator_input, double u_unsat,
    double u_sat, double k_aw, double dt) {
  if (!integrator.valid()) {
    return Reject(kIntegratorInvalidMessage, integrator.state());
  }
  const Result term = BackCalculation(u_unsat, u_sat, k_aw);
  if (!term.ok()) {
    return Result::Failure(term.status(), integrator.state());
  }
  if (!std::isfinite(integrator_input)) {
    return Reject(kInputMessage, integrator.state());
  }
  const double combined = integrator_input + term.value();
  if (!std::isfinite(combined)) {
    return Reject(kBackCalculationSumMessage, integrator.state());
  }
  return integrator.Integrate(combined, dt);
}

}  // namespace intrinsic::vehicle::control
