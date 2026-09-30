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

#ifndef INTRINSIC_WORLD_CURRENT_FIELD_COMPONENT_CURRENT_FIELD_COMPONENT_POLICY_H_
#define INTRINSIC_WORLD_CURRENT_FIELD_COMPONENT_CURRENT_FIELD_COMPONENT_POLICY_H_

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/current_field_component/README.md.
//
// Plain-value checks for CurrentFieldComponent. These functions do not parse
// protobuf, do not mutate World entities, do not convert frames, and do not
// call vehicle dynamics. Embedded validity is assessed by
// AssessMarineComponentValidity.
//
// Angular current is zero by convention. This view has no angular field.

// Matches vehicle Environment.current_frame_id. This package does not convert
// that id into world_enu or world_ned.
inline constexpr std::string_view kBodyFrameId = "body";

inline bool AllowedCurrentFrame(std::string_view frame_id) {
  return frame_id == embodiment::kWorldEnuFrameId ||
         frame_id == embodiment::kWorldNedFrameId || frame_id == kBodyFrameId;
}

enum class CurrentFieldError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kVelocity = 3,
};

struct CurrentFieldView {
  // False when the caller is not validating a present component. An empty
  // message is not present.
  bool present = false;
  MarineComponentValidityView validity;
  std::string_view frame_id;
  // False when velocity_m_s was not set. The vector is then unread.
  bool velocity_present = false;
  // Linear meters/second in frame_id. Angular current is not stored.
  embodiment::Vec3 velocity_m_s;
};

struct CurrentFieldAssessment {
  CurrentFieldError error = CurrentFieldError::kNone;
  MarineComponentAssessment validity;
  // True only when there is no structural defect and validity.accepted.
  bool accepted = false;
};

// Check order: embedded validity (missing, or a MarineComponentValidity
// structural error), frame id, then missing or non-finite velocity. The
// first defect wins. Unknown and expired stay on the nested assessment and
// are not a new error code. An empty view is not a component.
inline CurrentFieldAssessment AssessCurrentField(
    const CurrentFieldView& component, TimeParts query) {
  if (!component.present) {
    return CurrentFieldAssessment{};
  }
  const MarineComponentAssessment validity =
      AssessMarineComponentValidity(component.validity, query);
  CurrentFieldError error = CurrentFieldError::kNone;
  if (!component.validity.present ||
      validity.error != ComponentValidityError::kNone) {
    error = CurrentFieldError::kValidity;
  } else if (!AllowedCurrentFrame(component.frame_id)) {
    error = CurrentFieldError::kFrameId;
  } else if (!component.velocity_present ||
             !embodiment::IsFinite(component.velocity_m_s)) {
    error = CurrentFieldError::kVelocity;
  }
  const bool accepted = error == CurrentFieldError::kNone && validity.accepted;
  return CurrentFieldAssessment{error, validity, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_CURRENT_FIELD_COMPONENT_CURRENT_FIELD_COMPONENT_POLICY_H_
