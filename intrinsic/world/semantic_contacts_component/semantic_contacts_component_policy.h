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

#ifndef INTRINSIC_WORLD_SEMANTIC_CONTACTS_COMPONENT_SEMANTIC_CONTACTS_COMPONENT_POLICY_H_
#define INTRINSIC_WORLD_SEMANTIC_CONTACTS_COMPONENT_SEMANTIC_CONTACTS_COMPONENT_POLICY_H_

#include <string_view>
#include <unordered_set>
#include <vector>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/semantic_contacts_component/README.md.
//
// Plain-value checks for SemanticContactsComponent. These functions do not
// parse protobuf, do not mutate World entities, do not convert frames, and do
// not run perception. Embedded validity is assessed by
// AssessMarineComponentValidity.

inline bool AllowedSemanticContactsFrame(std::string_view frame_id) {
  return frame_id == embodiment::kWorldEnuFrameId ||
         frame_id == embodiment::kWorldNedFrameId;
}

// One check per contact. The first defect wins in the order below.
enum class SemanticContactError {
  kNone = 0,
  kContactId = 1,
  kDuplicateContactId = 2,
  kPose = 3,
  kVelocity = 4,
  kClassification = 5,
  kConfidence = 6,
  kAge = 7,
};

struct SemanticContactView {
  // Stable identity within the component. Empty is rejected.
  std::string_view contact_id;
  // False when pose was not set. The position and orientation are then
  // unread.
  bool pose_present = false;
  // Meters in the component frame_id.
  embodiment::Vec3 position;
  // Hamilton (x, y, z, w). Finite is required. Unit length is not checked.
  embodiment::Quaternion orientation;
  // False when velocity was not set. The vectors are then unread.
  bool velocity_present = false;
  // Meters/second and radians/second in the component frame_id.
  embodiment::Vec3 linear_velocity_m_s;
  embodiment::Vec3 angular_velocity_rad_s;
  // Opaque semantic class label. Empty is rejected.
  std::string_view classification;
  // Unset is absent and is not zero.
  bool confidence_present = false;
  double confidence = 0;
  // False when age was not set. A missing age is rejected.
  bool age_present = false;
  TimeParts age;
};

enum class SemanticContactsError {
  kNone = 0,
  kValidity = 1,
  kFrameId = 2,
  kContact = 3,
};

struct SemanticContactsView {
  // False when the caller is not validating a present component. An empty
  // message is not present.
  bool present = false;
  MarineComponentValidityView validity;
  std::string_view frame_id;
  // Zero or more contacts. An empty list is allowed.
  std::vector<SemanticContactView> contacts;
};

struct SemanticContactsAssessment {
  SemanticContactsError error = SemanticContactsError::kNone;
  MarineComponentAssessment validity;
  // Set only when error is kContact.
  SemanticContactError contact_error = SemanticContactError::kNone;
  // Index of the contact with the first defect. -1 when there is none.
  int contact_index = -1;
  // True only when there is no structural defect and validity.accepted.
  bool accepted = false;
};

// Checks one contact without looking at its siblings, so a duplicate id is
// not reported here. Order: contact id, pose, velocity, classification,
// confidence, age.
inline SemanticContactError AssessSemanticContact(
    const SemanticContactView &contact) {
  if (contact.contact_id.empty()) {
    return SemanticContactError::kContactId;
  }
  if (!contact.pose_present || !embodiment::IsFinite(contact.position) ||
      !embodiment::IsFinite(contact.orientation)) {
    return SemanticContactError::kPose;
  }
  if (!contact.velocity_present ||
      !embodiment::IsFinite(contact.linear_velocity_m_s) ||
      !embodiment::IsFinite(contact.angular_velocity_rad_s)) {
    return SemanticContactError::kVelocity;
  }
  if (contact.classification.empty()) {
    return SemanticContactError::kClassification;
  }
  if (contact.confidence_present && !ConfidenceInRange(contact.confidence)) {
    return SemanticContactError::kConfidence;
  }
  if (!contact.age_present || !DurationNonNegative(contact.age)) {
    return SemanticContactError::kAge;
  }
  return SemanticContactError::kNone;
}

// Check order: embedded validity (missing, or a MarineComponentValidity
// structural error), frame id, then contacts in list order. For each contact
// an empty id, then a repeated id, then the AssessSemanticContact checks. The
// first defect wins, so the first repeated id is the defect. Unknown and
// expired stay on the nested assessment and are not a new error code. An
// empty view is not a component. An empty contact list is accepted.
inline SemanticContactsAssessment AssessSemanticContacts(
    const SemanticContactsView &component, TimeParts query) {
  if (!component.present) {
    return SemanticContactsAssessment{};
  }
  SemanticContactsAssessment result;
  result.validity = AssessMarineComponentValidity(component.validity, query);
  if (!component.validity.present ||
      result.validity.error != ComponentValidityError::kNone) {
    result.error = SemanticContactsError::kValidity;
  } else if (!AllowedSemanticContactsFrame(component.frame_id)) {
    result.error = SemanticContactsError::kFrameId;
  } else {
    std::unordered_set<std::string_view> seen;
    for (int i = 0; i < static_cast<int>(component.contacts.size()); ++i) {
      const SemanticContactView &contact = component.contacts[i];
      SemanticContactError contact_error = SemanticContactError::kNone;
      if (contact.contact_id.empty()) {
        contact_error = SemanticContactError::kContactId;
      } else if (!seen.insert(contact.contact_id).second) {
        contact_error = SemanticContactError::kDuplicateContactId;
      } else {
        contact_error = AssessSemanticContact(contact);
      }
      if (contact_error != SemanticContactError::kNone) {
        result.error = SemanticContactsError::kContact;
        result.contact_error = contact_error;
        result.contact_index = i;
        break;
      }
    }
  }
  result.accepted =
      result.error == SemanticContactsError::kNone && result.validity.accepted;
  return result;
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_SEMANTIC_CONTACTS_COMPONENT_SEMANTIC_CONTACTS_COMPONENT_POLICY_H_
