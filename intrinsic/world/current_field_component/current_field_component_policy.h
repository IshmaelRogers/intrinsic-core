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

// Policy: intrinsic_apis/intrinsic/world/proto/README.md.
//
// Plain-value checks for CurrentFieldComponent. These functions do not
// parse protobuf and do not convert frames. validity_meta is assessed by
// AssessMarineComponentValidity. This file does not encode expiry.
// velocity_m_s is linear, SI meters per second, ENU.

enum class CurrentFieldError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kRepresentation = 3,
  kVelocity = 4,
};

struct CurrentFieldView {
  // False when the caller is not validating a present component.
  bool present = false;
  MarineComponentValidityView validity_meta;
  // ENU world frame id. Empty is a defect when present is true.
  std::string_view frame_id;
  // True when the ConstantCurrent arm is set.
  bool constant_present = false;
  // Linear current, meters per second. Checked only when constant_present.
  double velocity_x_m_s = 0;
  double velocity_y_m_s = 0;
  double velocity_z_m_s = 0;
};

struct CurrentFieldAssessment {
  CurrentFieldError error = CurrentFieldError::kNone;
  // Full #110 result, including unknown versus expired.
  MarineComponentAssessment validity;
  // True only when error is kNone and validity.accepted is true.
  bool accepted = false;
};

inline bool LinearVelocityFinite(double x_m_s, double y_m_s, double z_m_s) {
  return embodiment::IsFinite(x_m_s) && embodiment::IsFinite(y_m_s) &&
         embodiment::IsFinite(z_m_s);
}

// Check order: validity_meta, frame_id, constant arm, velocity components.
// The first defect wins. An empty view is not a component. A missing
// ConstantCurrent arm is a defect. Expiry stays on validity.freshness.
inline CurrentFieldAssessment AssessCurrentField(
    const CurrentFieldView &component, TimeParts query) {
  if (!component.present) {
    return CurrentFieldAssessment{
        CurrentFieldError::kNone,
        AssessMarineComponentValidity(MarineComponentValidityView{}, query),
        false};
  }
  const MarineComponentAssessment validity =
      AssessMarineComponentValidity(component.validity_meta, query);
  CurrentFieldError error = CurrentFieldError::kNone;
  if (!component.validity_meta.present ||
      validity.error != ComponentValidityError::kNone) {
    error = CurrentFieldError::kValidity;
  } else if (component.frame_id.empty()) {
    error = CurrentFieldError::kFrameId;
  } else if (!component.constant_present) {
    error = CurrentFieldError::kRepresentation;
  } else if (!LinearVelocityFinite(component.velocity_x_m_s,
                                   component.velocity_y_m_s,
                                   component.velocity_z_m_s)) {
    error = CurrentFieldError::kVelocity;
  }
  const bool accepted = error == CurrentFieldError::kNone && validity.accepted;
  return CurrentFieldAssessment{error, validity, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_CURRENT_FIELD_COMPONENT_CURRENT_FIELD_COMPONENT_POLICY_H_
