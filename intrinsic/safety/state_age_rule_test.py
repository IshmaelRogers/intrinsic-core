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

"""Boundary tests for the state.age rule."""

import unittest

from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result
from intrinsic.safety import state_age_rule

_NANOS = 1000000000
_RULE = state_age_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result


class StateAgeRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.UNSPECIFIED.value
    )

  def _expect_age_violation(self, result, summary):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "state.age")
    self.assertEqual(result.rule_id, _RULE.STATE_AGE_RULE_ID)
    self.assertEqual(result.severity, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertEqual(result.severity, _RESULT.SEVERITY_ERROR)
    self.assertEqual(result.summary, summary)
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertNotEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )

  def test_default_max_age_is_two_seconds(self):
    self.assertEqual(_RULE.DEFAULT_MAX_AGE, (2, 0))

  def test_exact_max_age_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_state_age_rule((12, 0), (10, 0)))
    self._expect_compliant(
        _RULE.evaluate_state_age_rule((12, 0), (10, 0), _RULE.DEFAULT_MAX_AGE)
    )

  def test_one_nanosecond_past_max_age_is_violated(self):
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((12, 1), (10, 0)),
        "state age exceeds max_age",
    )

  def test_zero_age_and_zero_max_age_are_compliant(self):
    stamp = (10, 1)
    self._expect_compliant(_RULE.evaluate_state_age_rule(stamp, stamp, (0, 0)))

  def test_one_nanosecond_past_zero_max_age_is_violated(self):
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((10, 1), (10, 0), (0, 0)),
        "state age exceeds max_age",
    )

  def test_borrow_across_second_matches_exact_nanosecond_bound(self):
    observation = (5, _NANOS - 1)
    self._expect_compliant(
        _RULE.evaluate_state_age_rule((6, 0), observation, (0, 1))
    )
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((6, 1), observation, (0, 1)),
        "state age exceeds max_age",
    )

  def test_negative_timestamps_use_the_same_boundary(self):
    observation = (-2, 0)
    self._expect_compliant(
        _RULE.evaluate_state_age_rule((0, 0), observation, (2, 0))
    )
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((0, 1), observation, (2, 0)),
        "state age exceeds max_age",
    )

  def test_future_observation_is_violated(self):
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((10, 0), (10, 1), (2, 0)),
        "state age input is not usable",
    )
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((10, _NANOS - 1), (11, 0), (5, 0)),
        "state age input is not usable",
    )

  def test_invalid_stamp_nanos_fail_closed(self):
    valid = (10, 0)
    max_age = (2, 0)
    for query, observation in (
        ((10, -1), valid),
        ((10, _NANOS), valid),
        (valid, (10, -1)),
        (valid, (10, _NANOS)),
    ):
      self._expect_age_violation(
          _RULE.evaluate_state_age_rule(query, observation, max_age),
          "state age input is not usable",
      )

  def test_negative_max_age_fails_closed(self):
    stamp = (10, 0)
    for max_age in ((-1, 0), (0, -1), (-1, -1)):
      self._expect_age_violation(
          _RULE.evaluate_state_age_rule(stamp, stamp, max_age),
          "state age input is not usable",
      )

  def test_max_age_nanos_out_of_range_fails_closed(self):
    stamp = (10, 0)
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule(stamp, stamp, (0, _NANOS)),
        "state age input is not usable",
    )

  def test_age_beyond_int64_seconds_fails_closed(self):
    observation = (-(2**63), 0)
    query = (2**63 - 1, 0)
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule(query, observation, (2, 0)),
        "state age exceeds max_age",
    )

  def test_int64_boundary_equality_is_compliant(self):
    observation = (-(2**63) + 1, 0)
    max_age = (2**63 - 1, 0)
    self._expect_compliant(
        _RULE.evaluate_state_age_rule((0, 0), observation, max_age)
    )
    self._expect_age_violation(
        _RULE.evaluate_state_age_rule((0, 1), observation, max_age),
        "state age exceeds max_age",
    )


if __name__ == "__main__":
  unittest.main()
