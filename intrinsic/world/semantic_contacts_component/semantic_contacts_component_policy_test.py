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

"""Validation tests for SemanticContactsComponent."""

import math
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.semantic_contacts_component import semantic_contacts_component_policy

_POLICY = semantic_contacts_component_policy
_VALIDITY = marine_component_validity_policy
_ContactError = _POLICY.SemanticContactError
_ComponentError = _POLICY.SemanticContactsError


def _valid_validity():
  return _VALIDITY.MarineComponentValidityView(
      present=True,
      source_id="fusion_0",
      observation_time_present=True,
      observation_time=(1700000000, 250000000),
      validity_horizon_present=True,
      validity_horizon=(10, 500000000),
      confidence_present=True,
      confidence=0.75,
      uncertainty_reference="cov-ref-7",
      validity_present=True,
      validity_state=1,
  )


def _contact(contact_id="buoy_1", classification="buoy"):
  return _POLICY.SemanticContactView(
      contact_id=contact_id,
      pose_present=True,
      position=(12.0, -3.5, -1.25),
      orientation=(0.0, 0.0, 0.0, 1.0),
      velocity_present=True,
      linear_velocity_m_s=(0.1, 0.0, -0.05),
      angular_velocity_rad_s=(0.0, 0.0, 0.02),
      classification=classification,
      confidence_present=True,
      confidence=0.9,
      age_present=True,
      age=(4, 500000000),
  )


def _valid_contacts():
  return _POLICY.SemanticContactsView(
      present=True,
      validity=_valid_validity(),
      frame_id=frame_policy.WORLD_ENU_FRAME_ID,
      contacts=(_contact("buoy_1", "buoy"), _contact("dock_2", "dock")),
  )


def _deadline():
  # 1700000000.250s + 10.500s = 1700000010.750s.
  return (1700000010, 750000000)


def _replace(view, **kwargs):
  data = dict(view.__dict__)
  data.update(kwargs)
  return type(view)(**data)


def _with_contact(component, index, **kwargs):
  contacts = list(component.contacts)
  contacts[index] = _replace(contacts[index], **kwargs)
  return _replace(component, contacts=tuple(contacts))


class SemanticContactsPolicyTest(unittest.TestCase):

  def _assess(self, view):
    return _POLICY.assess_semantic_contacts(view, _deadline())

  def test_empty_message_is_not_an_error(self):
    assessment = self._assess(_POLICY.SemanticContactsView())
    self.assertIs(assessment.error, _ComponentError.NONE)
    self.assertIs(assessment.contact_error, _ContactError.NONE)
    self.assertEqual(assessment.contact_index, -1)
    self.assertIs(
        assessment.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(
        assessment.validity.freshness, _VALIDITY.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(assessment.accepted)

  def test_missing_validity_is_first_defect(self):
    view = _replace(
        _valid_contacts(),
        validity=_replace(_valid_validity(), present=False),
        frame_id="robot",
        contacts=(_contact(""),),
    )
    assessment = self._assess(view)
    self.assertIs(assessment.error, _ComponentError.VALIDITY)
    self.assertFalse(assessment.accepted)

  def test_first_defect_wins(self):
    view = _replace(
        _valid_contacts(),
        validity=_replace(_valid_validity(), source_id=""),
        frame_id="robot",
        contacts=(_contact(""),),
    )
    assessment = self._assess(view)
    self.assertIs(assessment.error, _ComponentError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _VALIDITY.ComponentValidityError.SOURCE_ID
    )

    view = _replace(view, validity=_valid_validity())
    self.assertIs(self._assess(view).error, _ComponentError.FRAME_ID)

    view = _replace(view, frame_id=frame_policy.WORLD_NED_FRAME_ID)
    assessment = self._assess(view)
    self.assertIs(assessment.error, _ComponentError.CONTACT)
    self.assertIs(assessment.contact_error, _ContactError.CONTACT_ID)
    self.assertEqual(assessment.contact_index, 0)

    view = _replace(view, contacts=(_contact(),))
    assessment = self._assess(view)
    self.assertIs(assessment.error, _ComponentError.NONE)
    self.assertEqual(assessment.contact_index, -1)
    self.assertTrue(assessment.accepted)

  def test_frame_allow_list(self):
    view = _valid_contacts()
    for frame_id in (
        frame_policy.WORLD_ENU_FRAME_ID,
        frame_policy.WORLD_NED_FRAME_ID,
    ):
      accepted = self._assess(_replace(view, frame_id=frame_id))
      self.assertTrue(accepted.accepted, frame_id)
    for frame_id in (
        "",
        "enu",
        "ned",
        "world_ENU",
        "body",
        "robot",
        "world_enu ",
    ):
      assessment = self._assess(_replace(view, frame_id=frame_id))
      self.assertIs(assessment.error, _ComponentError.FRAME_ID, frame_id)

  def test_empty_contact_list_is_accepted(self):
    assessment = self._assess(_replace(_valid_contacts(), contacts=()))
    self.assertIs(assessment.error, _ComponentError.NONE)
    self.assertEqual(assessment.contact_index, -1)
    self.assertTrue(assessment.accepted)

  def test_contact_id_empty_and_duplicate(self):
    view = _valid_contacts()
    empty = self._assess(_with_contact(view, 1, contact_id=""))
    self.assertIs(empty.error, _ComponentError.CONTACT)
    self.assertIs(empty.contact_error, _ContactError.CONTACT_ID)
    self.assertEqual(empty.contact_index, 1)

    duplicate = self._assess(_with_contact(view, 1, contact_id="buoy_1"))
    self.assertIs(duplicate.error, _ComponentError.CONTACT)
    self.assertIs(duplicate.contact_error, _ContactError.DUPLICATE_CONTACT_ID)
    self.assertEqual(duplicate.contact_index, 1)
    self.assertFalse(duplicate.accepted)

    for near_miss in ("Buoy_1", "buoy_1 ", " buoy_1"):
      assessment = self._assess(_with_contact(view, 1, contact_id=near_miss))
      self.assertTrue(assessment.accepted, near_miss)

  def test_first_duplicate_is_the_defect(self):
    contacts = (
        _contact("a", "buoy"),
        _contact("b", "buoy"),
        _contact("b", "buoy"),
        _contact("a", "buoy"),
    )
    assessment = self._assess(_replace(_valid_contacts(), contacts=contacts))
    self.assertIs(assessment.contact_error, _ContactError.DUPLICATE_CONTACT_ID)
    self.assertEqual(assessment.contact_index, 2)

  def test_duplicate_id_precedes_later_field_defects_of_same_contact(self):
    contacts = (
        _contact("a"),
        _replace(_contact("a"), classification="", age_present=False),
    )
    assessment = self._assess(_replace(_valid_contacts(), contacts=contacts))
    self.assertIs(assessment.contact_error, _ContactError.DUPLICATE_CONTACT_ID)

  def test_earlier_contact_defect_wins_over_later_duplicate(self):
    contacts = (
        _replace(_contact("a"), classification=""),
        _contact("a"),
    )
    assessment = self._assess(_replace(_valid_contacts(), contacts=contacts))
    self.assertIs(assessment.contact_error, _ContactError.CLASSIFICATION)
    self.assertEqual(assessment.contact_index, 0)

  def test_pose_must_be_present_and_finite(self):
    view = _valid_contacts()
    missing = self._assess(_with_contact(view, 0, pose_present=False))
    self.assertIs(missing.contact_error, _ContactError.POSE)
    self.assertEqual(missing.contact_index, 0)

    for position in (
        (math.nan, 0.0, 0.0),
        (0.0, math.inf, 0.0),
        (0.0, 0.0, -math.inf),
    ):
      assessment = self._assess(_with_contact(view, 1, position=position))
      self.assertIs(assessment.contact_error, _ContactError.POSE)
      self.assertEqual(assessment.contact_index, 1)
    for orientation in (
        (math.nan, 0.0, 0.0, 1.0),
        (0.0, math.inf, 0.0, 1.0),
        (0.0, 0.0, -math.inf, 1.0),
        (0.0, 0.0, 0.0, math.nan),
    ):
      assessment = self._assess(_with_contact(view, 0, orientation=orientation))
      self.assertIs(assessment.contact_error, _ContactError.POSE)

  def test_quaternion_is_not_renormalized(self):
    view = _with_contact(_valid_contacts(), 0, orientation=(0.0, 0.0, 0.0, 0.0))
    self.assertTrue(self._assess(view).accepted)
    view = _with_contact(_valid_contacts(), 0, orientation=(3.0, 4.0, 0.0, 5.0))
    self.assertTrue(self._assess(view).accepted)

  def test_velocity_must_be_present_and_finite(self):
    view = _valid_contacts()
    missing = self._assess(_with_contact(view, 0, velocity_present=False))
    self.assertIs(missing.contact_error, _ContactError.VELOCITY)

    zero = self._assess(
        _with_contact(
            view,
            0,
            linear_velocity_m_s=(0.0, 0.0, 0.0),
            angular_velocity_rad_s=(0.0, 0.0, 0.0),
        )
    )
    self.assertTrue(zero.accepted)
    bad = (math.nan, math.inf, -math.inf)
    for index in range(3):
      vector = tuple(bad[index] if i == index else 0.0 for i in range(3))
      for field in ("linear_velocity_m_s", "angular_velocity_rad_s"):
        assessment = self._assess(_with_contact(view, 1, **{field: vector}))
        self.assertIs(assessment.contact_error, _ContactError.VELOCITY, field)
        self.assertEqual(assessment.contact_index, 1)

  def test_classification_is_opaque_and_required(self):
    view = _valid_contacts()
    empty = self._assess(_with_contact(view, 0, classification=""))
    self.assertIs(empty.contact_error, _ContactError.CLASSIFICATION)
    for label in ("unknown", " ", "Buoy", "a,b"):
      self.assertTrue(
          self._assess(_with_contact(view, 0, classification=label)).accepted,
          label,
      )

  def test_confidence_unset_is_not_zero_and_bounds_are_inclusive(self):
    view = _valid_contacts()
    unset = self._assess(
        _with_contact(view, 0, confidence_present=False, confidence=math.nan)
    )
    self.assertTrue(unset.accepted)
    for confidence in (0.0, -0.0, 0.5, 1.0):
      assessment = self._assess(
          _with_contact(view, 0, confidence_present=True, confidence=confidence)
      )
      self.assertTrue(assessment.accepted, confidence)
    for confidence in (
        -1e-12,
        1.0000000000000002,
        2.0,
        math.nan,
        math.inf,
        -math.inf,
    ):
      assessment = self._assess(
          _with_contact(view, 1, confidence_present=True, confidence=confidence)
      )
      self.assertIs(assessment.contact_error, _ContactError.CONFIDENCE)
      self.assertEqual(assessment.contact_index, 1)

  def test_age_must_be_present_and_non_negative(self):
    view = _valid_contacts()
    missing = self._assess(_with_contact(view, 0, age_present=False))
    self.assertIs(missing.contact_error, _ContactError.AGE)

    for age in ((0, 0), (0, 999999999), (3600, 0)):
      self.assertTrue(
          self._assess(_with_contact(view, 0, age=age)).accepted, age
      )
    for age in ((-1, 0), (-1, 500000000), (0, -1), (1, 1000000000)):
      assessment = self._assess(_with_contact(view, 0, age=age))
      self.assertIs(assessment.contact_error, _ContactError.AGE, age)
      self.assertEqual(assessment.contact_index, 0)

  def test_per_contact_check_order(self):
    bad = _POLICY.SemanticContactView()
    self.assertIs(
        _POLICY.assess_semantic_contact(bad), _ContactError.CONTACT_ID
    )
    bad = _replace(bad, contact_id="c")
    self.assertIs(_POLICY.assess_semantic_contact(bad), _ContactError.POSE)
    bad = _replace(bad, pose_present=True)
    self.assertIs(_POLICY.assess_semantic_contact(bad), _ContactError.VELOCITY)
    bad = _replace(bad, velocity_present=True)
    self.assertIs(
        _POLICY.assess_semantic_contact(bad), _ContactError.CLASSIFICATION
    )
    bad = _replace(bad, classification="unknown")
    self.assertIs(_POLICY.assess_semantic_contact(bad), _ContactError.AGE)
    bad = _replace(bad, confidence_present=True, confidence=2.0)
    self.assertIs(
        _POLICY.assess_semantic_contact(bad), _ContactError.CONFIDENCE
    )
    bad = _replace(bad, confidence_present=False, age_present=True)
    self.assertIs(_POLICY.assess_semantic_contact(bad), _ContactError.NONE)

  def test_same_numbers_accepted_in_each_frame(self):
    view = _valid_contacts()
    for frame_id in (
        frame_policy.WORLD_ENU_FRAME_ID,
        frame_policy.WORLD_NED_FRAME_ID,
    ):
      stored = _replace(view, frame_id=frame_id)
      self.assertTrue(self._assess(stored).accepted, frame_id)
      self.assertEqual(stored.contacts[0].position, (12.0, -3.5, -1.25))
      self.assertEqual(
          stored.contacts[0].linear_velocity_m_s, (0.1, 0.0, -0.05)
      )

  def test_accepted_example_at_deadline(self):
    deadline = _deadline()
    fresh = _POLICY.assess_semantic_contacts(_valid_contacts(), deadline)
    self.assertIs(fresh.error, _ComponentError.NONE)
    self.assertIs(fresh.validity.freshness, _VALIDITY.ComponentFreshness.FRESH)
    self.assertIs(
        fresh.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_semantic_contacts(
        _valid_contacts(), (deadline[0], deadline[1] + 1)
    )
    self.assertIs(expired.error, _ComponentError.NONE)
    self.assertIs(
        expired.validity.freshness, _VALIDITY.ComponentFreshness.EXPIRED
    )
    self.assertFalse(expired.accepted)

  def test_unknown_and_invalid_stay_nested(self):
    absent = self._assess(
        _replace(
            _valid_contacts(),
            validity=_replace(_valid_validity(), validity_present=False),
        )
    )
    self.assertIs(absent.error, _ComponentError.NONE)
    self.assertIs(
        absent.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(
        absent.validity.freshness, _VALIDITY.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(absent.accepted)

    invalid = self._assess(
        _replace(
            _valid_contacts(),
            validity=_replace(_valid_validity(), validity_state=2),
        )
    )
    self.assertIs(
        invalid.validity.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(invalid.accepted)

    unknown = self._assess(
        _replace(
            _valid_contacts(),
            validity=_replace(_valid_validity(), validity_state=99),
        )
    )
    self.assertIs(
        unknown.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertIs(unknown.error, _ComponentError.NONE)
    self.assertFalse(unknown.accepted)

  def test_bad_embedded_confidence_is_a_validity_defect(self):
    view = _replace(
        _valid_contacts(),
        validity=_replace(_valid_validity(), confidence=1.5),
    )
    assessment = self._assess(view)
    self.assertIs(assessment.error, _ComponentError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _VALIDITY.ComponentValidityError.CONFIDENCE
    )

  def test_expired_contact_defect_stays_a_contact_error(self):
    deadline = _deadline()
    assessment = _POLICY.assess_semantic_contacts(
        _with_contact(_valid_contacts(), 0, classification=""),
        (deadline[0], deadline[1] + 1),
    )
    self.assertIs(assessment.error, _ComponentError.CONTACT)
    self.assertIs(
        assessment.validity.freshness, _VALIDITY.ComponentFreshness.EXPIRED
    )
    self.assertFalse(assessment.accepted)


if __name__ == "__main__":
  unittest.main()
