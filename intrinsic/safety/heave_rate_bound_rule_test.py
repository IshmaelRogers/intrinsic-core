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

"""Boundary tests for the rate.descent.max and rate.ascent.max rules."""

import math
import unittest

from intrinsic.safety import heave_rate_bound_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = heave_rate_bound_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result
_DESCENT = "rate.descent.max"
_ASCENT = "rate.ascent.max"
_evaluate = heave_rate_bound_rule.evaluate_heave_rate_bound_rule


class HeaveRateBoundRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_clamp(self, result, rule_id, limit):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, rule_id)
    self.assertEqual(result.severity, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 2)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_PROJECT)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )
    self.assertTrue(result.has_projected_value)
    self.assertEqual(result.projected_value, limit)

  def _expect_reject_critical(self, result, rule_id):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, rule_id)
    self.assertEqual(result.severity, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def test_rule_ids_and_defaults_are_locked(self):
    self.assertEqual(_RULE.DESCENT_RATE_MAX_RULE_ID, "rate.descent.max")
    self.assertEqual(_RULE.ASCENT_RATE_MAX_RULE_ID, "rate.ascent.max")
    self.assertEqual(_RULE.DEFAULT_MAX_DESCENT_RATE, 0.5)
    self.assertEqual(_RULE.DEFAULT_MAX_ASCENT_RATE, 0.5)

  def test_nominal_zero_rate_is_compliant(self):
    self._expect_compliant(_evaluate(0.0))
    self._expect_compliant(_evaluate(-0.0))

  def test_just_inside_is_compliant(self):
    self._expect_compliant(_evaluate(0.499))
    self._expect_compliant(_evaluate(-0.499))

  def test_equal_to_limit_is_compliant(self):
    self._expect_compliant(_evaluate(0.5))
    self._expect_compliant(_evaluate(-0.5))
    self._expect_compliant(_evaluate(0.2, 0.2, 0.7))
    self._expect_compliant(_evaluate(-0.7, 0.2, 0.7))
    self._expect_compliant(_evaluate(0.0, 0.0, 0.0))

  def test_pure_descent_over_clamps_to_descent_limit(self):
    self._expect_clamp(_evaluate(0.501), _DESCENT, 0.5)
    self._expect_clamp(_evaluate(2.0), _DESCENT, 0.5)
    self._expect_clamp(_evaluate(0.3, 0.2, 0.7), _DESCENT, 0.2)
    self._expect_clamp(_evaluate(0.001, 0.0, 0.7), _DESCENT, 0.0)

  def test_pure_ascent_over_clamps_to_negative_ascent_limit(self):
    self._expect_clamp(_evaluate(-0.501), _ASCENT, -0.5)
    self._expect_clamp(_evaluate(-2.0), _ASCENT, -0.5)
    self._expect_clamp(_evaluate(-0.8, 0.2, 0.7), _ASCENT, -0.7)
    self._expect_clamp(_evaluate(-0.001, 0.2, 0.0), _ASCENT, -0.0)

  def test_asymmetric_limits_are_independent(self):
    self._expect_compliant(_evaluate(0.2, 0.2, 1.0))
    self._expect_compliant(_evaluate(-1.0, 0.2, 1.0))
    self._expect_clamp(_evaluate(0.25, 0.2, 1.0), _DESCENT, 0.2)
    self._expect_clamp(_evaluate(-1.5, 0.2, 1.0), _ASCENT, -1.0)

  def test_non_finite_depth_rate_rejects(self):
    for value in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(_evaluate(value), _DESCENT)

  def test_bad_descent_max_rejects_as_descent(self):
    for limit in (math.nan, math.inf, -math.inf, -0.001):
      self._expect_reject_critical(_evaluate(0.1, limit, 0.5), _DESCENT)

  def test_bad_ascent_max_only_rejects_as_ascent(self):
    for limit in (math.nan, math.inf, -math.inf, -0.001):
      self._expect_reject_critical(_evaluate(0.1, 0.5, limit), _ASCENT)

  def test_both_maxes_bad_rejects_as_descent(self):
    self._expect_reject_critical(_evaluate(0.1, -1.0, -1.0), _DESCENT)
    self._expect_reject_critical(_evaluate(0.1, math.nan, math.nan), _DESCENT)

  def test_bad_limit_rejects_even_when_rate_would_be_compliant(self):
    self._expect_reject_critical(_evaluate(0.0, 0.5, -1.0), _ASCENT)
    self._expect_reject_critical(_evaluate(0.0, -1.0, 0.5), _DESCENT)

  def test_negative_zero_max_is_usable(self):
    self._expect_compliant(_evaluate(0.0, -0.0, -0.0))

  def test_simultaneous_violation_yields_single_result(self):
    self._expect_clamp(_evaluate(0.1, 0.0, 0.0), _DESCENT, 0.0)


if __name__ == "__main__":
  unittest.main()
