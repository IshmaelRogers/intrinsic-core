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

#ifndef INTRINSIC_VEHICLE_DYNAMICS_ZERO_FORCE_DYNAMICS_H_
#define INTRINSIC_VEHICLE_DYNAMICS_ZERO_FORCE_DYNAMICS_H_

#include <string_view>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

namespace intrinsic::vehicle::dynamics {

// Deterministic zero-force test double.
//
// For a valid input, body acceleration is zero and the pose derivative is
// the rigid-body kinematic map of the current body twist. The linear part
// is the body-frame velocity of the origin rotated into pose_frame. The
// quaternion rate is one half of the orientation times the body angular
// velocity. The input wrench, gravity, density, and current are validated
// and then left unused. The same inputs produce the same outputs.
//
// On an invalid input, Evaluate returns kInvalidArgument and a finite zero
// derivative. diagnostics.model_id still names this double. dt_s stays 0
// so a rejected call does not echo a non-finite time step.
//
// This class has no data members. Concurrent Evaluate calls on one instance
// do not share mutable state. Evaluate does not allocate heap memory and
// does not solve actuator commands.
inline constexpr std::string_view kZeroForceModelId = "zero_force";

class ZeroForceDynamics final : public VehicleDynamics {
 public:
  ZeroForceDynamics() = default;

  [[nodiscard]] StatusOr<DynamicsResult> Evaluate(
      const VehicleStateRt& state, const BodyWrenchRt& wrench,
      const EnvironmentRt& environment, Duration dt) const override;
};

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_ZERO_FORCE_DYNAMICS_H_
