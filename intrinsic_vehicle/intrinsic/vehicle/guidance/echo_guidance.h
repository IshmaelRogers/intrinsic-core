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

#ifndef INTRINSIC_VEHICLE_GUIDANCE_ECHO_GUIDANCE_H_
#define INTRINSIC_VEHICLE_GUIDANCE_ECHO_GUIDANCE_H_

#include "intrinsic/vehicle/guidance/guidance_step.h"

namespace intrinsic::vehicle::guidance {

// Deterministic identity-map test double. Not a guidance law.
//
// For a valid pose objective, the reference copies that pose and leaves
// the twist unset. For a valid twist objective, the reference copies that
// body twist and leaves the pose unset. snapshot_id and update_period are
// echoed. The vehicle-state pose and twist are validated and then left
// unused. The same inputs produce the same outputs.
//
// A trajectory id is not sampled. Evaluate returns kInvalid and an empty
// reference. ValidateGuidanceInputs still returns kOk for that id: this
// double refuses to invent a pose from it.
//
// On any rejected input, Evaluate returns that status and an empty
// reference. The empty reference does not echo a non-finite update period
// or a snapshot id.
//
// This class has no data members. Concurrent Evaluate calls on one
// instance do not share mutable state. Evaluate does not allocate heap
// memory and does not write actuator commands.
class EchoGuidance final : public GuidanceStep {
 public:
  EchoGuidance() = default;

  [[nodiscard]] StatusOr<MotionReferenceRt> Evaluate(
      const DesiredMotionRt& intent, const VehicleStateRt* state,
      Duration update_period) const override;
};

}  // namespace intrinsic::vehicle::guidance

#endif  // INTRINSIC_VEHICLE_GUIDANCE_ECHO_GUIDANCE_H_
