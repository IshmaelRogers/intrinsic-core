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

#ifndef INTRINSIC_VEHICLE_CONTROL_ZERO_WRENCH_CONTROLLER_H_
#define INTRINSIC_VEHICLE_CONTROL_ZERO_WRENCH_CONTROLLER_H_

#include "intrinsic/vehicle/control/reference_control.h"

namespace intrinsic::vehicle::control {

// Deterministic neutral-wrench test double. Not a control law.
//
// For a valid input, Evaluate returns kOk and a body-frame wrench whose
// force and torque are zero. That is the neutral wrench. It is not an
// actuator command, a thruster command, or an allocation result. The
// reference and the state are validated and then left unused, so two
// different valid poses produce the same wrench.
//
// On an invalid input, Evaluate returns that status and a finite-zero
// wrench. The status is not kOk, so the zero wrench is not a command.
// The same inputs produce the same outputs.
//
// This class has no data members. Concurrent Evaluate calls on one
// instance do not share mutable state. Evaluate does not allocate heap
// memory and does not read a gain.
class ZeroWrenchController final : public ReferenceController {
 public:
  ZeroWrenchController() = default;

  [[nodiscard]] StatusOr<BodyWrenchRt> Evaluate(
      const MotionReferenceRt& reference, const VehicleStateRt& state,
      Duration update_period) const override;
};

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_ZERO_WRENCH_CONTROLLER_H_
