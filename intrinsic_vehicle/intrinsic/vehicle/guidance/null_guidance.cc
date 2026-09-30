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

#include "intrinsic/vehicle/guidance/null_guidance.h"

#include <string_view>

namespace intrinsic::vehicle::guidance {
namespace {

constexpr std::string_view kNullMessage =
    "null guidance does not produce a reference";

}  // namespace

StatusOr<MotionReferenceRt> NullGuidance::Evaluate(
    const DesiredMotionRt& intent, const VehicleStateRt* state,
    Duration update_period) const {
  const GuidanceStatus status =
      ValidateGuidanceInputs(intent, state, update_period);
  if (!status.ok()) {
    return StatusOr<MotionReferenceRt>::Failure(status);
  }
  return StatusOr<MotionReferenceRt>::Failure(
      GuidanceStatus::MissingObjective(kNullMessage));
}

}  // namespace intrinsic::vehicle::guidance
