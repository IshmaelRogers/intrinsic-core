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

"""Validation tests for CurrentFieldComponent."""

import math
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.current_field_component import current_field_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_POLICY = current_field_component_policy
_VALIDITY = marine_component_validity_policy


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


def _valid_current():
  return _POLICY.CurrentFieldView(
      present=True,
      validity=_valid_validity(),
      frame_id=frame_policy.WORLD_ENU_FRAME_ID,
      velocity_present=True,
      velocity_m_s=(0.2, -0.1, 0.0),
  )


def _deadline():
  # 1700000000.250s + 10.500s = 1700000010.750s.
  return (1700000010, 750000000)


def _replace(view, **kwargs):
  data = dict(view.__dict__)
  data.update(kwargs)
  return type(view)(**data)


class CurrentFieldPolicyTest(unittest.TestCase):

  def test_empty_message_is_not_an_error(self):
    assessment = _POLICY.assess_current_field(
        _POLICY.CurrentFieldView(), _deadline()
    )
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(
        assessment.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(
        assessment.validity.freshness, _VALIDITY.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(assessment.accepted)

  def test_missing_validity_is_first_defect(self):
    view = _replace(
        _valid_current(),
        validity=_replace(_valid_validity(), present=False),
        frame_id="robot",
        velocity_present=False,
    )
    assessment = _POLICY.assess_current_field(view, _deadline())
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.VALIDITY)
    self.assertFalse(assessment.accepted)

  def test_first_defect_wins(self):
    view = _replace(
        _valid_current(),
        validity=_replace(_valid_validity(), source_id=""),
        frame_id="robot",
        velocity_present=False,
    )
    assessment = _POLICY.assess_current_field(view, _deadline())
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _VALIDITY.ComponentValidityError.SOURCE_ID
    )

    view = _replace(view, validity=_valid_validity())
    assessment = _POLICY.assess_current_field(view, _deadline())
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.FRAME_ID)

    view = _replace(view, frame_id=_POLICY.BODY_FRAME_ID)
    assessment = _POLICY.assess_current_field(view, _deadline())
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.VELOCITY)

    view = _replace(view, velocity_present=True, velocity_m_s=(0.0, 0.0, 0.0))
    assessment = _POLICY.assess_current_field(view, _deadline())
    self.assertIs(assessment.error, _POLICY.CurrentFieldError.NONE)
    self.assertTrue(assessment.accepted)

  def test_frame_allow_list(self):
    view = _valid_current()
    for frame_id in (
        frame_policy.WORLD_ENU_FRAME_ID,
        frame_policy.WORLD_NED_FRAME_ID,
        _POLICY.BODY_FRAME_ID,
    ):
      accepted = _POLICY.assess_current_field(
          _replace(view, frame_id=frame_id), _deadline()
      )
      self.assertTrue(accepted.accepted, frame_id)
    for frame_id in ("", "enu", "ned", "world_ENU", "BODY", "robot", "body "):
      assessment = _POLICY.assess_current_field(
          _replace(view, frame_id=frame_id), _deadline()
      )
      self.assertIs(
          assessment.error, _POLICY.CurrentFieldError.FRAME_ID, frame_id
      )

  def test_velocity_must_be_present_and_finite(self):
    view = _valid_current()
    missing = _POLICY.assess_current_field(
        _replace(
            view,
            velocity_present=False,
            velocity_m_s=(math.nan, math.nan, math.nan),
        ),
        _deadline(),
    )
    self.assertIs(missing.error, _POLICY.CurrentFieldError.VELOCITY)
    zero = _POLICY.assess_current_field(
        _replace(view, velocity_present=True, velocity_m_s=(0.0, 0.0, 0.0)),
        _deadline(),
    )
    self.assertTrue(zero.accepted)
    for velocity in (
        (math.nan, 0.0, 0.0),
        (0.0, math.inf, 0.0),
        (0.0, 0.0, -math.inf),
    ):
      assessment = _POLICY.assess_current_field(
          _replace(view, velocity_m_s=velocity), _deadline()
      )
      self.assertIs(assessment.error, _POLICY.CurrentFieldError.VELOCITY)

  def test_same_numbers_accepted_in_each_frame(self):
    view = _replace(_valid_current(), velocity_m_s=(0.2, -0.1, 0.0))
    for frame_id in (
        frame_policy.WORLD_ENU_FRAME_ID,
        frame_policy.WORLD_NED_FRAME_ID,
        _POLICY.BODY_FRAME_ID,
    ):
      stored = _replace(view, frame_id=frame_id)
      assessment = _POLICY.assess_current_field(stored, _deadline())
      self.assertTrue(assessment.accepted, frame_id)
      self.assertEqual(stored.velocity_m_s, (0.2, -0.1, 0.0))

  def test_accepted_example_at_deadline(self):
    deadline = _deadline()
    fresh = _POLICY.assess_current_field(_valid_current(), deadline)
    self.assertIs(fresh.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(fresh.validity.freshness, _VALIDITY.ComponentFreshness.FRESH)
    self.assertIs(
        fresh.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_current_field(
        _valid_current(), (deadline[0], deadline[1] + 1)
    )
    self.assertIs(expired.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(
        expired.validity.freshness, _VALIDITY.ComponentFreshness.EXPIRED
    )
    self.assertFalse(expired.accepted)

  def test_unknown_and_invalid_stay_nested(self):
    absent = _POLICY.assess_current_field(
        _replace(
            _valid_current(),
            validity=_replace(_valid_validity(), validity_present=False),
        ),
        _deadline(),
    )
    self.assertIs(absent.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(
        absent.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(absent.accepted)

    invalid = _POLICY.assess_current_field(
        _replace(
            _valid_current(),
            validity=_replace(_valid_validity(), validity_state=2),
        ),
        _deadline(),
    )
    self.assertIs(
        invalid.validity.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(invalid.accepted)

    unknown = _POLICY.assess_current_field(
        _replace(
            _valid_current(),
            validity=_replace(_valid_validity(), validity_state=99),
        ),
        _deadline(),
    )
    self.assertIs(
        unknown.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertIs(unknown.error, _POLICY.CurrentFieldError.NONE)
    self.assertFalse(unknown.accepted)


if __name__ == "__main__":
  unittest.main()
