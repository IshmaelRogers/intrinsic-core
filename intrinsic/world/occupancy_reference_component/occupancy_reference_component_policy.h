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

#ifndef INTRINSIC_WORLD_OCCUPANCY_REFERENCE_COMPONENT_OCCUPANCY_REFERENCE_COMPONENT_POLICY_H_
#define INTRINSIC_WORLD_OCCUPANCY_REFERENCE_COMPONENT_OCCUPANCY_REFERENCE_COMPONENT_POLICY_H_

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/occupancy_reference_component/README.md.
//
// Plain-value checks for OccupancyReferenceComponent. These functions do
// not parse protobuf, do not mutate World entities, and do not convert
// world_enu and world_ned. Embedded validity is assessed by
// AssessMarineComponentValidity.

inline bool AllowedOccupancyFrame(std::string_view frame_id) {
  return frame_id == embodiment::kWorldEnuFrameId ||
         frame_id == embodiment::kWorldNedFrameId;
}

enum class OccupancyReferenceError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kAssetReference = 3,
};

struct OccupancyReferenceView {
  // False when the caller is not validating a present component. An empty
  // message is not present.
  bool present = false;
  MarineComponentValidityView validity;
  std::string_view frame_id;
  // Opaque reference. Empty is rejected when the component is present.
  std::string_view asset_reference;
};

struct OccupancyReferenceAssessment {
  OccupancyReferenceError error = OccupancyReferenceError::kNone;
  MarineComponentAssessment validity;
  // True only when there is no structural defect and validity.accepted.
  bool accepted = false;
};

// Check order: embedded validity (missing, or a MarineComponentValidity
// structural error), frame id, then asset reference. The first defect wins.
// Unknown and expired stay on the nested assessment and are not a new error
// code. An empty view is not a component.
inline OccupancyReferenceAssessment AssessOccupancyReference(
    const OccupancyReferenceView &component, TimeParts query) {
  if (!component.present) {
    return OccupancyReferenceAssessment{};
  }
  const MarineComponentAssessment validity =
      AssessMarineComponentValidity(component.validity, query);
  OccupancyReferenceError error = OccupancyReferenceError::kNone;
  if (!component.validity.present ||
      validity.error != ComponentValidityError::kNone) {
    error = OccupancyReferenceError::kValidity;
  } else if (!AllowedOccupancyFrame(component.frame_id)) {
    error = OccupancyReferenceError::kFrameId;
  } else if (component.asset_reference.empty()) {
    error = OccupancyReferenceError::kAssetReference;
  }
  const bool accepted =
      error == OccupancyReferenceError::kNone && validity.accepted;
  return OccupancyReferenceAssessment{error, validity, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_OCCUPANCY_REFERENCE_COMPONENT_OCCUPANCY_REFERENCE_COMPONENT_POLICY_H_
