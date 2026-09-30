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

// Scalar control math. Heading wrap, a bounded integrator, and classic
// back-calculation. These functions take doubles only. They do not read
// a vehicle state, a body wrench, a gain schedule, or a plant.
//
// ShortestSignedAngle maps a finite angle with
//
//   atan2(sin(delta), cos(delta))
//
// into (-pi, pi]. A result of exactly -pi is the same angle as +pi and
// is returned as +pi. HeadingError applies that wrap to
// reference_rad - measured_rad. A positive result means measured_rad is
// short of reference_rad in the positive direction.
//
// BoundedIntegrator stores one scalar. Integrate computes
//
//   x = clamp(x + u * dt, lower, upper)
//
// and writes x only after every check succeeds. A zero time step with a
// finite input returns success and leaves x unchanged. A negative time
// step, a non-finite time step, a non-finite input, or a non-finite
// product or sum returns kInvalidArgument and leaves x unchanged.
//
// BackCalculation is the tracking term
//
//   k_aw * (u_sat - u_unsat)
//
// IntegrateBackCalculation feeds that term into the integrator:
//
//   x = clamp(x + (integrator_input + k_aw * (u_sat - u_unsat)) * dt,
//             lower, upper)
//
// The caller supplies both commands and the gain. This function does not
// clamp the command and does not choose k_aw. A negative finite gain is
// accepted. The same inputs produce the same outputs.
//
// Storage is the BoundedIntegrator object. These functions do not
// allocate. Status text is a static string view. Integrate and
// IntegrateBackCalculation mutate one integrator and are not safe for
// concurrent calls on that object. The pure functions do not share
// mutable state.

#ifndef INTRINSIC_VEHICLE_CONTROL_CONTROL_MATH_H_
#define INTRINSIC_VEHICLE_CONTROL_CONTROL_MATH_H_

#include <string_view>

namespace intrinsic::vehicle::control {

// kOk is the only success code.
enum class ControlMathCode {
  kOk = 0,
  kInvalidArgument = 1,
};

struct ControlMathStatus {
  ControlMathCode code = ControlMathCode::kOk;
  std::string_view message;

  [[nodiscard]] bool ok() const { return code == ControlMathCode::kOk; }

  [[nodiscard]] static ControlMathStatus Ok() { return {}; }

  [[nodiscard]] static ControlMathStatus InvalidArgument(
      std::string_view message) {
    return {ControlMathCode::kInvalidArgument, message};
  }
};

// Fixed-size status return. On failure, value() is the T stored by
// Failure. Angle and back-calculation failures store zero. Integrator
// failures store the unchanged state.
template <typename T>
class ControlMathResult {
 public:
  [[nodiscard]] static ControlMathResult Ok(T value) {
    ControlMathResult out;
    out.ok_ = true;
    out.status_ = ControlMathStatus::Ok();
    out.value_ = value;
    return out;
  }

  [[nodiscard]] static ControlMathResult Failure(ControlMathStatus status,
                                                 T value = T{}) {
    ControlMathResult out;
    out.ok_ = false;
    out.status_ = status;
    out.value_ = value;
    return out;
  }

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] const ControlMathStatus& status() const { return status_; }
  [[nodiscard]] const T& value() const { return value_; }

 private:
  ControlMathResult() = default;
  bool ok_ = false;
  ControlMathStatus status_;
  T value_{};
};

// Shortest signed angle of delta_rad, in radians, in (-pi, pi].
// A non-finite delta_rad or a non-finite reduction returns
// kInvalidArgument and value 0.
[[nodiscard]] ControlMathResult<double> ShortestSignedAngle(double delta_rad);

// ShortestSignedAngle(reference_rad - measured_rad). A non-finite
// argument, or a non-finite difference, returns kInvalidArgument and
// value 0.
[[nodiscard]] ControlMathResult<double> HeadingError(double reference_rad,
                                                     double measured_rad);

// k_aw * (u_sat - u_unsat). A non-finite argument or a non-finite
// product returns kInvalidArgument and value 0.
[[nodiscard]] ControlMathResult<double> BackCalculation(double u_unsat,
                                                        double u_sat,
                                                        double k_aw);

// One scalar state and inclusive limits. A default instance is invalid.
// Create rejects non-finite limits, a lower limit above the upper limit,
// a non-finite initial state, and an initial state outside the limits.
// The failure value is invalid: Integrate does not write it.
class BoundedIntegrator {
 public:
  BoundedIntegrator() = default;

  [[nodiscard]] static ControlMathResult<BoundedIntegrator> Create(
      double initial, double lower, double upper);

  [[nodiscard]] bool valid() const { return valid_; }
  [[nodiscard]] double state() const { return state_; }
  [[nodiscard]] double lower() const { return lower_; }
  [[nodiscard]] double upper() const { return upper_; }

  // Check order: this object is valid; u is finite; dt is finite and
  // greater than or equal to zero; dt is zero (success, no write); the
  // product and the updated state are finite; then clamp and store.
  // On failure, state() is unchanged and value() equals that state.
  [[nodiscard]] ControlMathResult<double> Integrate(double u, double dt);

 private:
  bool valid_ = false;
  double state_ = 0;
  double lower_ = 0;
  double upper_ = 0;
};

// Check order: integrator is valid; BackCalculation succeeds;
// integrator_input is finite; the sum of that input and the tracking
// term is finite; then Integrate. On failure the integrator is unchanged
// and value() equals its state.
[[nodiscard]] ControlMathResult<double> IntegrateBackCalculation(
    BoundedIntegrator& integrator, double integrator_input, double u_unsat,
    double u_sat, double k_aw, double dt);

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_CONTROL_MATH_H_
