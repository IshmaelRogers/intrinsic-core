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

// Policy: intrinsic_apis/intrinsic/world/proto/README.md.
//
// Plain-value checks for BathymetryReferenceComponent. These functions do
// not parse protobuf and do not convert ENU to NED. validity_meta is
// assessed by AssessMarineComponentValidity. This file does not encode
// expiry.

enum class BathymetryReferenceError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kAssetRef = 3,
  kReferenceZ = 4,
};

struct BathymetryReferenceView {
  // False when the caller is not validating a present component.
  bool present = false;
  MarineComponentValidityView validity_meta;
  // ENU world frame id. Empty is a defect when present is true.
  std::string_view frame_id;
  // Opaque asset key. Empty is a defect when present is true.
  std::string_view bathymetry_asset_ref;
  bool reference_z_present = false;
  // Meters, ENU +Z. Checked only when reference_z_present is true.
  double reference_z_m = 0;
};

struct BathymetryReferenceAssessment {
  BathymetryReferenceError error = BathymetryReferenceError::kNone;
  // Full #110 result, including unknown versus expired.
  MarineComponentAssessment validity;
  // True only when error is kNone and validity.accepted is true.
  bool accepted = false;
};

// Check order: validity_meta, frame_id, bathymetry_asset_ref, reference_z_m.
// The first defect wins. An empty view is not a component. A missing
// validity_meta is a defect. Expiry stays on validity.freshness.
inline BathymetryReferenceAssessment AssessBathymetryReference(
    const BathymetryReferenceView &component, TimeParts query) {
  if (!component.present) {
    return BathymetryReferenceAssessment{
        BathymetryReferenceError::kNone,
        AssessMarineComponentValidity(MarineComponentValidityView{}, query),
        false};
  }
  const MarineComponentAssessment validity =
      AssessMarineComponentValidity(component.validity_meta, query);
  BathymetryReferenceError error = BathymetryReferenceError::kNone;
  if (!component.validity_meta.present ||
      validity.error != ComponentValidityError::kNone) {
    error = BathymetryReferenceError::kValidity;
  } else if (component.frame_id.empty()) {
    error = BathymetryReferenceError::kFrameId;
  } else if (component.bathymetry_asset_ref.empty()) {
    error = BathymetryReferenceError::kAssetRef;
  } else if (component.reference_z_present &&
             !embodiment::IsFinite(component.reference_z_m)) {
    error = BathymetryReferenceError::kReferenceZ;
  }
  const bool accepted =
      error == BathymetryReferenceError::kNone && validity.accepted;
  return BathymetryReferenceAssessment{error, validity, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_BATHYMETRY_REFERENCE_COMPONENT_BATHYMETRY_REFERENCE_COMPONENT_POLICY_H_
