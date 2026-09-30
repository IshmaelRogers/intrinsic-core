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

#ifndef INTRINSIC_WORLD_BATHYMETRY_REFERENCE_COMPONENT_BATHYMETRY_REFERENCE_COMPONENT_POLICY_H_
#define INTRINSIC_WORLD_BATHYMETRY_REFERENCE_COMPONENT_BATHYMETRY_REFERENCE_COMPONENT_POLICY_H_

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/bathymetry_reference_component/README.md.
//
// Plain-value checks for BathymetryReferenceComponent. These functions do
// not parse protobuf, do not mutate World entities, and do not convert
// world_enu and world_ned. Embedded validity is assessed by
// AssessMarineComponentValidity.

inline bool AllowedBathymetryFrame(std::string_view frame_id) {
  return frame_id == embodiment::kWorldEnuFrameId ||
         frame_id == embodiment::kWorldNedFrameId;
}

enum class BathymetryReferenceError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kAssetReference = 3,
  kVerticalBias = 4,
};

struct BathymetryReferenceView {
  // False when the caller is not validating a present component. An empty
  // message is not present.
  bool present = false;
  MarineComponentValidityView validity;
  std::string_view frame_id;
  // Opaque reference. Empty is rejected when the component is present.
  std::string_view asset_reference;
  bool vertical_bias_present = false;
  // Meters along frame_id +Z. Ignored unless vertical_bias_present.
  double vertical_bias_m = 0;
};

struct BathymetryReferenceAssessment {
  BathymetryReferenceError error = BathymetryReferenceError::kNone;
  MarineComponentAssessment validity;
  // True only when there is no structural defect and validity.accepted.
  bool accepted = false;
};

// Check order: embedded validity (missing, or a MarineComponentValidity
// structural error), frame id, asset reference, then a present non-finite
// bias. The first defect wins. Unknown and expired stay on the nested
// assessment and are not a new error code. An empty view is not a component.
inline BathymetryReferenceAssessment AssessBathymetryReference(
    const BathymetryReferenceView& component, TimeParts query) {
  if (!component.present) {
    return BathymetryReferenceAssessment{};
  }
  const MarineComponentAssessment validity =
      AssessMarineComponentValidity(component.validity, query);
  BathymetryReferenceError error = BathymetryReferenceError::kNone;
  if (!component.validity.present ||
      validity.error != ComponentValidityError::kNone) {
    error = BathymetryReferenceError::kValidity;
  } else if (!AllowedBathymetryFrame(component.frame_id)) {
    error = BathymetryReferenceError::kFrameId;
  } else if (component.asset_reference.empty()) {
    error = BathymetryReferenceError::kAssetReference;
  } else if (component.vertical_bias_present &&
             !embodiment::IsFinite(component.vertical_bias_m)) {
    error = BathymetryReferenceError::kVerticalBias;
  }
  const bool accepted =
      error == BathymetryReferenceError::kNone && validity.accepted;
  return BathymetryReferenceAssessment{error, validity, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_BATHYMETRY_REFERENCE_COMPONENT_BATHYMETRY_REFERENCE_COMPONENT_POLICY_H_
