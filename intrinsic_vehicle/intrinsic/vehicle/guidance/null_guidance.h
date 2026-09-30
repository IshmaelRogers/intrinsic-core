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

#ifndef INTRINSIC_VEHICLE_GUIDANCE_NULL_GUIDANCE_H_
#define INTRINSIC_VEHICLE_GUIDANCE_NULL_GUIDANCE_H_

#include "intrinsic/vehicle/guidance/guidance_step.h"

namespace intrinsic::vehicle::guidance {

// Deterministic test double that never produces a reference.
//
// Evaluate runs ValidateGuidanceInputs. A defect returns that status.
// An input that would otherwise be usable, including a pose or twist
// objective, returns kMissingObjective and an empty reference. An unset
// objective also returns kMissingObjective, from the shared contract,
// with the message "objective is unset".
//
// This class has no data members. Concurrent Evaluate calls on one
// instance do not share mutable state. Evaluate does not allocate heap
// memory, does not echo a pose, and does not write actuator commands.
class NullGuidance final : public GuidanceStep {
 public:
  NullGuidance() = default;

  [[nodiscard]] StatusOr<MotionReferenceRt> Evaluate(
      const DesiredMotionRt& intent, const VehicleStateRt* state,
      Duration update_period) const override;
};

}  // namespace intrinsic::vehicle::guidance

#endif  // INTRINSIC_VEHICLE_GUIDANCE_NULL_GUIDANCE_H_
