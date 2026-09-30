# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Plain-value checks for SemanticContactsComponent.

Policy: intrinsic/world/semantic_contacts_component/README.md.

These helpers do not parse protobuf, do not mutate World entities, do not
convert frames, and do not run perception. Embedded validity is assessed by
assess_marine_component_validity.
"""

from dataclasses import dataclass
import enum

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_VALIDITY = marine_component_validity_policy


class SemanticContactError(enum.Enum):
  """One check per contact. The first defect wins in this order."""

  NONE = 0
  CONTACT_ID = 1
  DUPLICATE_CONTACT_ID = 2
  POSE = 3
  VELOCITY = 4
  CLASSIFICATION = 5
  CONFIDENCE = 6
  AGE = 7


class SemanticContactsError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  CONTACT = 3


@dataclass(frozen=True)
class SemanticContactView:
  """One contact. Pose and velocity are in the component frame_id."""

  contact_id: str = ""
  pose_present: bool = False
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  # Hamilton (x, y, z, w). Finite is required. Unit length is not checked.
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)
  velocity_present: bool = False
  linear_velocity_m_s: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  angular_velocity_rad_s: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  classification: str = ""
  confidence_present: bool = False
  confidence: float = 0.0
  age_present: bool = False
  age: _VALIDITY.TimeParts = (0, 0)


@dataclass(frozen=True)
class SemanticContactsView:
  """present is false for an empty message."""

  present: bool = False
  validity: _VALIDITY.MarineComponentValidityView = (
      _VALIDITY.MarineComponentValidityView()
  )
  frame_id: str = ""
  contacts: tuple[SemanticContactView, ...] = ()


@dataclass(frozen=True)
class SemanticContactsAssessment:
  error: SemanticContactsError
  validity: _VALIDITY.MarineComponentAssessment
  # Set only when error is CONTACT.
  contact_error: SemanticContactError
  # Index of the contact with the first defect. -1 when there is none.
  contact_index: int
  accepted: bool


def allowed_semantic_contacts_frame(frame_id: str) -> bool:
  """True for exact world_enu or world_ned. Does not convert frames."""
  return frame_id in (
      frame_policy.WORLD_ENU_FRAME_ID,
      frame_policy.WORLD_NED_FRAME_ID,
  )


def _empty_validity() -> _VALIDITY.MarineComponentAssessment:
  return _VALIDITY.MarineComponentAssessment(
      _VALIDITY.ComponentValidityError.NONE,
      stamped_header_policy.ValidityKind.ABSENT,
      _VALIDITY.ComponentFreshness.UNKNOWN,
      False,
  )


def assess_semantic_contact(
    contact: SemanticContactView,
) -> SemanticContactError:
  """Checks one contact without looking at its siblings.

  A duplicate id is not reported here. Order: contact id, pose, velocity,
  classification, confidence, age.
  """
  if contact.contact_id == "":
    return SemanticContactError.CONTACT_ID
  if (
      not contact.pose_present
      or not frame_policy.is_finite_vec3(contact.position)
      or not frame_policy.is_finite_quaternion(contact.orientation)
  ):
    return SemanticContactError.POSE
  if (
      not contact.velocity_present
      or not frame_policy.is_finite_vec3(contact.linear_velocity_m_s)
      or not frame_policy.is_finite_vec3(contact.angular_velocity_rad_s)
  ):
    return SemanticContactError.VELOCITY
  if contact.classification == "":
    return SemanticContactError.CLASSIFICATION
  if contact.confidence_present and not _VALIDITY.confidence_in_range(
      contact.confidence
  ):
    return SemanticContactError.CONFIDENCE
  if not contact.age_present or not _VALIDITY.duration_non_negative(
      contact.age
  ):
    return SemanticContactError.AGE
  return SemanticContactError.NONE


def assess_semantic_contacts(
    component: SemanticContactsView, query: _VALIDITY.TimeParts
) -> SemanticContactsAssessment:
  """First structural defect wins.

  Check order: embedded validity (missing, or a MarineComponentValidity
  structural error), frame id, then contacts in list order. For each contact
  an empty id, then a repeated id, then the assess_semantic_contact checks.
  The first repeated id is the defect. Unknown and expired stay on the nested
  assessment. An empty view is not a component. An empty contact list is
  accepted.
  """
  if not component.present:
    return SemanticContactsAssessment(
        SemanticContactsError.NONE,
        _empty_validity(),
        SemanticContactError.NONE,
        -1,
        False,
    )
  validity = _VALIDITY.assess_marine_component_validity(
      component.validity, query
  )
  error = SemanticContactsError.NONE
  contact_error = SemanticContactError.NONE
  contact_index = -1
  if (
      not component.validity.present
      or validity.error is not _VALIDITY.ComponentValidityError.NONE
  ):
    error = SemanticContactsError.VALIDITY
  elif not allowed_semantic_contacts_frame(component.frame_id):
    error = SemanticContactsError.FRAME_ID
  else:
    seen = set()
    for index, contact in enumerate(component.contacts):
      if contact.contact_id == "":
        found = SemanticContactError.CONTACT_ID
      elif contact.contact_id in seen:
        found = SemanticContactError.DUPLICATE_CONTACT_ID
      else:
        seen.add(contact.contact_id)
        found = assess_semantic_contact(contact)
      if found is not SemanticContactError.NONE:
        error = SemanticContactsError.CONTACT
        contact_error = found
        contact_index = index
        break
  accepted = error is SemanticContactsError.NONE and validity.accepted
  return SemanticContactsAssessment(
      error, validity, contact_error, contact_index, accepted
  )
