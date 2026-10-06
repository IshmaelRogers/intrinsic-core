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

#ifndef INTRINSIC_SCENE_SDF_CURRENT_FIELD_TO_SDF_H_
#define INTRINSIC_SCENE_SDF_CURRENT_FIELD_TO_SDF_H_

#include <string>
#include <string_view>

#include "absl/status/statusor.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic {
namespace sdf {

inline constexpr std::string_view kHydrodynamicsPluginFilename =
    "static://intrinsic::simulation::Hydrodynamics";
inline constexpr std::string_view kHydrodynamicsPluginName =
    "intrinsic::simulation::Hydrodynamics";

// Converts a CurrentFieldComponent plain-value view into a string containing
// one SDF <plugin> element that configures the Hydrodynamics plugin:
//
//   <plugin
//       filename="static://intrinsic::simulation::Hydrodynamics"
//       name="intrinsic::simulation::Hydrodynamics">
//     <current_field>
//       <frame_id>world_enu|world_ned|body</frame_id>
//       <velocity_m_s>vx vy vz</velocity_m_s>
//     </current_field>
//   </plugin>
//
// Returns an empty string when `current_field.present` is false, matching the
// empty-when-absent behavior of SimSpecToSdf.
//
// The component is checked with world::AssessCurrentField at `query` before
// anything is emitted. Errors name the failing field:
//   * InvalidArgument "current_field.validity": missing or structurally
//     invalid marine validity.
//   * InvalidArgument "current_field.frame_id": not world_enu, world_ned, or
//     body.
//   * InvalidArgument "current_field.velocity_m_s": missing or non-finite.
//   * FailedPrecondition "current_field.validity": structurally valid but not
//     accepted (unknown, expired, or not STATE_VALID).
// `validity` is a policy gate only and is never written to SDF. Frames are not
// converted. Angular current is zero by convention and is not emitted.
//
// This only builds plugin configuration text. It registers no plugin, applies
// no force, and evaluates no dynamics. Bathymetry is not handled.
absl::StatusOr<std::string> CurrentFieldToSdf(
    const world::CurrentFieldView& current_field, world::TimeParts query);

}  // namespace sdf
}  // namespace intrinsic

#endif  // INTRINSIC_SCENE_SDF_CURRENT_FIELD_TO_SDF_H_
