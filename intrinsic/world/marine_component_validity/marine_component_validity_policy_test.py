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

"""Validation tests for marine component validity."""

import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_POLICY = marine_component_validity_policy
_INT64_MAX = 9223372036854775807


def _valid_view():
  # 1000.5s + 1.6s = 1002.1s.
  return _POLICY.MarineComponentValidityView(
      present=True,
      source_id="fusion_0",
      observation_time_present=True,
      observation_time=(1000, 500000000),
      validity_horizon_present=True,
      validity_horizon=(1, 600000000),
      confidence_present=True,
      confidence=0.75,
      uncertainty_reference="cov-ref-7",
      validity_present=True,
      validity_state=1,
  )


def _deadline():
  return (1002, 100000000)


class MarineComponentValidityPolicyTest(unittest.TestCase):

  def test_empty_message_is_not_an_error(self):
    assessment = _POLICY.assess_marine_component_validity(
        _POLICY.MarineComponentValidityView(), _deadline()
    )
    self.assertIs(assessment.error, _POLICY.ComponentValidityError.NONE)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    self.assertFalse(assessment.accepted)

  def test_missing_source_id_on_present_component(self):
    view = _valid_view()
    view = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "source_id": ""}
    )
    assessment = _POLICY.assess_marine_component_validity(view, _deadline())
    self.assertIs(assessment.error, _POLICY.ComponentValidityError.SOURCE_ID)
    self.assertFalse(assessment.accepted)

  def test_first_defect_wins(self):
    view = _valid_view()
    broken = _POLICY.MarineComponentValidityView(
        **{
            **view.__dict__,
            "source_id": "",
            "observation_time": (1000, -1),
            "validity_horizon": (-5, 600000000),
            "confidence": math.nan,
        }
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(broken, _deadline()).error,
        _POLICY.ComponentValidityError.SOURCE_ID,
    )
    broken = _POLICY.MarineComponentValidityView(
        **{**broken.__dict__, "source_id": "fusion_0"}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(broken, _deadline()).error,
        _POLICY.ComponentValidityError.OBSERVATION_TIME,
    )
    broken = _POLICY.MarineComponentValidityView(
        **{**broken.__dict__, "observation_time": (1000, 500000000)}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(broken, _deadline()).error,
        _POLICY.ComponentValidityError.HORIZON,
    )
    broken = _POLICY.MarineComponentValidityView(
        **{**broken.__dict__, "validity_horizon": (1, 600000000)}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(broken, _deadline()).error,
        _POLICY.ComponentValidityError.CONFIDENCE,
    )

  def test_rejects_nonsense_horizon(self):
    view = _valid_view()
    for horizon in ((-1, 0), (0, -1), (0, 1000000000)):
      sample = _POLICY.MarineComponentValidityView(
          **{**view.__dict__, "validity_horizon": horizon}
      )
      assessment = _POLICY.assess_marine_component_validity(sample, _deadline())
      self.assertIs(assessment.error, _POLICY.ComponentValidityError.HORIZON)
      self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    zero = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_horizon": (0, 0)}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(
            zero, (1000, 500000000)
        ).freshness,
        _POLICY.ComponentFreshness.FRESH,
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(
            zero, (1000, 500000001)
        ).freshness,
        _POLICY.ComponentFreshness.EXPIRED,
    )

  def test_confidence_bounds(self):
    view = _valid_view()
    absent = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "confidence_present": False, "confidence": math.inf}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(absent, _deadline()).error,
        _POLICY.ComponentValidityError.NONE,
    )
    for confidence in (0.0, -0.0, 1.0, 0.75):
      sample = _POLICY.MarineComponentValidityView(
          **{**view.__dict__, "confidence": confidence}
      )
      self.assertIs(
          _POLICY.assess_marine_component_validity(sample, _deadline()).error,
          _POLICY.ComponentValidityError.NONE,
      )
    for confidence in (-0.1, math.nextafter(1.0, 2.0), math.nan, math.inf):
      sample = _POLICY.MarineComponentValidityView(
          **{**view.__dict__, "confidence": confidence}
      )
      self.assertIs(
          _POLICY.assess_marine_component_validity(sample, _deadline()).error,
          _POLICY.ComponentValidityError.CONFIDENCE,
      )

  def test_exact_horizon_boundary(self):
    view = _valid_view()
    deadline = _deadline()
    at_deadline = _POLICY.assess_marine_component_validity(view, deadline)
    self.assertIs(at_deadline.freshness, _POLICY.ComponentFreshness.FRESH)
    self.assertIs(
        at_deadline.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(at_deadline.accepted)
    just_before = _POLICY.assess_marine_component_validity(
        view, (1002, 99999999)
    )
    self.assertIs(just_before.freshness, _POLICY.ComponentFreshness.FRESH)
    self.assertTrue(just_before.accepted)
    just_after = _POLICY.assess_marine_component_validity(
        view, (deadline[0], deadline[1] + 1)
    )
    self.assertIs(just_after.freshness, _POLICY.ComponentFreshness.EXPIRED)
    self.assertIs(just_after.validity, stamped_header_policy.ValidityKind.VALID)
    self.assertFalse(just_after.accepted)
    self.assertIs(
        _POLICY.assess_marine_component_validity(view, (1000, 0)).freshness,
        _POLICY.ComponentFreshness.FRESH,
    )

  def test_unknown_is_distinct_from_expired(self):
    view = _valid_view()
    after = (1002, 100000001)
    unset = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_present": False}
    )
    assessment = _POLICY.assess_marine_component_validity(unset, after)
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIsNot(assessment.freshness, _POLICY.ComponentFreshness.EXPIRED)

    missing_observation = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "observation_time_present": False}
    )
    assessment = _POLICY.assess_marine_component_validity(
        missing_observation, after
    )
    self.assertIs(assessment.error, _POLICY.ComponentValidityError.NONE)
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    self.assertFalse(assessment.accepted)

    missing_horizon = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_horizon_present": False}
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(
            missing_horizon, after
        ).freshness,
        _POLICY.ComponentFreshness.UNKNOWN,
    )

    unspecified = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_state": 0}
    )
    assessment = _POLICY.assess_marine_component_validity(unspecified, after)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.UNSPECIFIED
    )
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.EXPIRED)
    self.assertFalse(assessment.accepted)

    invalid = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_state": 2}
    )
    assessment = _POLICY.assess_marine_component_validity(invalid, _deadline())
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.FRESH)
    self.assertFalse(assessment.accepted)

    unknown_enum = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "validity_state": 99}
    )
    assessment = _POLICY.assess_marine_component_validity(unknown_enum, after)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.UNSPECIFIED
    )
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.EXPIRED)

  def test_uncertainty_reference_is_opaque(self):
    view = _valid_view()
    empty = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "uncertainty_reference": ""}
    )
    self.assertTrue(
        _POLICY.assess_marine_component_validity(empty, _deadline()).accepted
    )
    opaque = _POLICY.MarineComponentValidityView(
        **{**view.__dict__, "uncertainty_reference": "1,nan,inf"}
    )
    self.assertTrue(
        _POLICY.assess_marine_component_validity(opaque, _deadline()).accepted
    )

  def test_nonsense_query_is_unknown(self):
    assessment = _POLICY.assess_marine_component_validity(
        _valid_view(), (1002, -1)
    )
    self.assertIs(assessment.error, _POLICY.ComponentValidityError.NONE)
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    self.assertFalse(assessment.accepted)

  def test_deadline_overflow_is_unknown(self):
    view = _valid_view()
    overflow = _POLICY.MarineComponentValidityView(
        **{
            **view.__dict__,
            "observation_time": (_INT64_MAX, 999999999),
            "validity_horizon": (0, 1),
        }
    )
    assessment = _POLICY.assess_marine_component_validity(overflow, (0, 0))
    self.assertIs(assessment.error, _POLICY.ComponentValidityError.NONE)
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    crossed = _POLICY.MarineComponentValidityView(
        **{
            **view.__dict__,
            "observation_time": (-1, 800000000),
            "validity_horizon": (0, 300000000),
        }
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(
            crossed, (0, 100000000)
        ).freshness,
        _POLICY.ComponentFreshness.FRESH,
    )
    self.assertIs(
        _POLICY.assess_marine_component_validity(
            crossed, (0, 100000001)
        ).freshness,
        _POLICY.ComponentFreshness.EXPIRED,
    )


if __name__ == "__main__":
  unittest.main()
