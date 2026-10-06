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

#include "intrinsic/scene/sdf/current_field_to_sdf.h"

#include <charconv>
#include <string>
#include <system_error>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic {
namespace sdf {

namespace {

// Shortest round-trip decimal text, independent of locale. Negative zero is
// written as "0" so equal currents render identically.
std::string FormatDouble(double value) {
  if (value == 0.0) {
    return "0";
  }
  char buffer[64];
  const std::to_chars_result result =
      std::to_chars(buffer, buffer + sizeof(buffer), value);
  return std::string(buffer, result.ptr);
}

absl::Status ValidityStatus(const world::CurrentFieldView& current_field,
                            const world::MarineComponentAssessment& validity) {
  if (!current_field.validity.present) {
    return absl::InvalidArgumentError(
        "current_field.validity: missing validity");
  }
  if (validity.error != world::ComponentValidityError::kNone) {
    return absl::InvalidArgumentError(absl::StrCat(
        "current_field.validity: structurally invalid (error code ",
        static_cast<int>(validity.error), ")"));
  }
  return absl::FailedPreconditionError(
      "current_field.validity: not accepted (unknown, expired, or not "
      "STATE_VALID)");
}

}  // namespace

absl::StatusOr<std::string> CurrentFieldToSdf(
    const world::CurrentFieldView& current_field, world::TimeParts query) {
  if (!current_field.present) {
    return std::string();
  }

  const world::CurrentFieldAssessment assessment =
      world::AssessCurrentField(current_field, query);
  switch (assessment.error) {
    case world::CurrentFieldError::kNone:
      break;
    case world::CurrentFieldError::kValidity:
      return ValidityStatus(current_field, assessment.validity);
    case world::CurrentFieldError::kFrameId:
      return absl::InvalidArgumentError(
          absl::StrCat("current_field.frame_id: \"", current_field.frame_id,
                       "\" is not one of world_enu, world_ned, body"));
    case world::CurrentFieldError::kVelocity:
      return absl::InvalidArgumentError(
          current_field.velocity_present
              ? "current_field.velocity_m_s: components must be finite"
              : "current_field.velocity_m_s: missing velocity");
  }
  if (!assessment.accepted) {
    return ValidityStatus(current_field, assessment.validity);
  }

  const embodiment::Vec3& velocity = current_field.velocity_m_s;
  return absl::StrCat(
      "<plugin filename=\"", kHydrodynamicsPluginFilename, "\"\n",
      "        name=\"", kHydrodynamicsPluginName, "\">\n",
      "  <current_field>\n", "    <frame_id>", current_field.frame_id,
      "</frame_id>\n", "    <velocity_m_s>", FormatDouble(velocity.x), " ",
      FormatDouble(velocity.y), " ", FormatDouble(velocity.z),
      "</velocity_m_s>\n", "  </current_field>\n", "</plugin>\n");
}

}  // namespace sdf
}  // namespace intrinsic
