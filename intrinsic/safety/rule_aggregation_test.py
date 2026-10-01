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
"""Tests for safety rule aggregation."""

import itertools
import unittest

from intrinsic.safety import rule_aggregation
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_AGG = rule_aggregation
_DECISION = safety_decision_policy
_RESULT = safety_rule_result

_ERROR = _RESULT.SEVERITY_ERROR
_CRITICAL = _RESULT.SEVERITY_CRITICAL


def _compliant():
  return _RESULT.SafetyRuleResult()


def _project(rule_id, value, severity=_ERROR):
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=rule_id,
      severity=severity,
      summary="clamped",
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=value,
  )


def _reject(rule_id, severity=_CRITICAL, summary="rejected"):
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=rule_id,
      severity=severity,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def _audit_reject(rule_id, audit_value):
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=rule_id,
      severity=_ERROR,
      summary="audit",
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
      has_projected_value=True,
      projected_value=audit_value,
  )


class RuleAggregationTest(unittest.TestCase):

  def _expect_compliant(self, agg, violated_count=0):
    self.assertFalse(agg.violated)
    self.assertEqual(agg.primary_rule_id, "")
    self.assertEqual(agg.severity, 0)
    self.assertEqual(agg.summary, "")
    self.assertEqual(agg.recommended_kind, 0)
    self.assertFalse(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.0)
    self.assertEqual(agg.violated_count, violated_count)
    self.assertFalse(agg.projection_conflict)

  def _expect_permutation_invariant(self, results):
    expected = _AGG.aggregate_safety_rule_results(results)
    for perm in itertools.permutations(results):
      self.assertEqual(_AGG.aggregate_safety_rule_results(list(perm)), expected)
    return expected

  def test_wire_numbers_match_locked_enums(self):
    self.assertEqual(
        _RESULT.DECISION_KIND_PROJECT,
        _DECISION.SafetyDecisionKind.PROJECT.value,
    )
    self.assertEqual(
        _RESULT.DECISION_KIND_REJECT, _DECISION.SafetyDecisionKind.REJECT.value
    )

  def test_empty_is_compliant(self):
    self._expect_compliant(_AGG.aggregate_safety_rule_results([]))
    self._expect_compliant(_AGG.aggregate_safety_rule_results(()))

  def test_all_compliant_is_compliant(self):
    self._expect_compliant(
        _AGG.aggregate_safety_rule_results(
            [_compliant(), _compliant(), _compliant()]
        )
    )

  def test_single_reject_mirrors_finding(self):
    agg = _AGG.aggregate_safety_rule_results(
        [_compliant(), _reject("state.age", _CRITICAL, "stale")]
    )
    self.assertTrue(agg.violated)
    self.assertEqual(agg.primary_rule_id, "state.age")
    self.assertEqual(agg.severity, _CRITICAL)
    self.assertEqual(agg.summary, "stale")
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.0)
    self.assertEqual(agg.violated_count, 1)
    self.assertFalse(agg.projection_conflict)

  def test_single_project_keeps_clamp_value(self):
    agg = _AGG.aggregate_safety_rule_results(
        [_compliant(), _project("speed.max", 1.5), _compliant()]
    )
    self.assertTrue(agg.violated)
    self.assertEqual(agg.primary_rule_id, "speed.max")
    self.assertEqual(agg.severity, _ERROR)
    self.assertEqual(agg.summary, "clamped")
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_PROJECT)
    self.assertTrue(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 1.5)
    self.assertEqual(agg.violated_count, 1)
    self.assertFalse(agg.projection_conflict)

  def test_reject_beats_project_in_any_order(self):
    agg = self._expect_permutation_invariant(
        [_project("speed.max", 1.5), _reject("state.age", _CRITICAL, "stale")]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    self.assertFalse(agg.has_projected_value)
    self.assertEqual(agg.primary_rule_id, "state.age")
    self.assertEqual(agg.violated_count, 2)

  def test_reject_beats_project_even_when_project_is_primary(self):
    agg = self._expect_permutation_invariant(
        [
            _project("altitude.min", 3.0, _CRITICAL),
            _reject("state.age", _ERROR, "stale"),
        ]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    self.assertEqual(agg.primary_rule_id, "altitude.min")
    self.assertEqual(agg.severity, _CRITICAL)

  def test_two_projects_escalate_to_conflict_reject(self):
    agg = self._expect_permutation_invariant(
        [
            _project("speed.max", 1.5),
            _project("altitude.min", 3.0),
            _compliant(),
        ]
    )
    self.assertTrue(agg.violated)
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertTrue(agg.projection_conflict)
    self.assertFalse(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.0)
    self.assertEqual(agg.summary, "conflicting projections")
    self.assertEqual(agg.summary, _AGG.CONFLICTING_PROJECTIONS_SUMMARY)
    self.assertEqual(agg.primary_rule_id, "altitude.min")
    self.assertEqual(agg.severity, _ERROR)
    self.assertEqual(agg.violated_count, 2)

  def test_equal_projected_values_still_conflict(self):
    agg = self._expect_permutation_invariant(
        [
            _project("speed.max", 1.0),
            _project("depth.max", 1.0),
            _project("heave.rate", 1.0),
        ]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertTrue(agg.projection_conflict)
    self.assertEqual(agg.primary_rule_id, "depth.max")

  def test_severity_is_max_and_primary_is_lex_first_at_max(self):
    agg = self._expect_permutation_invariant(
        [
            _reject("nav.covariance", _ERROR, "e1"),
            _reject("state.age", _CRITICAL, "c2"),
            _reject("clearance.min", _ERROR, "e2"),
            _reject("geofence", _CRITICAL, "c1"),
        ]
    )
    self.assertEqual(agg.severity, _CRITICAL)
    self.assertEqual(agg.primary_rule_id, "geofence")
    self.assertEqual(agg.summary, "c1")
    self.assertEqual(agg.violated_count, 4)

  def test_two_errors_and_one_critical(self):
    agg = self._expect_permutation_invariant(
        [
            _reject("a.rule", _ERROR),
            _reject("c.rule", _ERROR),
            _reject("z.rule", _CRITICAL),
        ]
    )
    self.assertEqual(agg.severity, _CRITICAL)
    self.assertEqual(agg.primary_rule_id, "z.rule")

  def test_audit_reject_alone_is_not_a_conflict(self):
    agg = _AGG.aggregate_safety_rule_results(
        [_audit_reject("clearance.min", 0.4)]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    self.assertEqual(agg.primary_rule_id, "clearance.min")
    self.assertTrue(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.4)

  def test_two_audit_rejects_are_not_a_conflict(self):
    agg = self._expect_permutation_invariant(
        [
            _audit_reject("clearance.min", 0.4),
            _audit_reject("energy.reserve", 9.0),
        ]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    self.assertEqual(agg.primary_rule_id, "clearance.min")
    self.assertEqual(agg.projected_value, 0.4)

  def test_audit_reject_plus_one_project_reject_wins(self):
    agg = self._expect_permutation_invariant(
        [
            _audit_reject("clearance.min", 0.4),
            _project("speed.max", 1.5),
        ]
    )
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    self.assertNotEqual(agg.summary, "conflicting projections")
    self.assertEqual(agg.primary_rule_id, "clearance.min")
    self.assertTrue(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.4)

  def test_reject_aggregate_clears_projection_when_primary_has_none(self):
    agg = self._expect_permutation_invariant(
        [
            _audit_reject("nav.covariance", 0.9),
            _reject("state.age", _CRITICAL, "stale"),
        ]
    )
    self.assertEqual(agg.primary_rule_id, "state.age")
    self.assertFalse(agg.has_projected_value)
    self.assertEqual(agg.projected_value, 0.0)

  def test_unknown_kind_fails_closed_to_reject(self):
    odd = _RESULT.SafetyRuleResult(
        violated=True,
        rule_id="odd.rule",
        severity=_CRITICAL,
        summary="odd",
        recommended_kind=7,
    )
    agg = self._expect_permutation_invariant([odd, _project("speed.max", 1.5)])
    self.assertEqual(agg.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(agg.projection_conflict)
    unspecified = _RESULT.SafetyRuleResult(
        violated=True, rule_id="odd.rule", severity=_ERROR, summary="odd"
    )
    self.assertEqual(
        _AGG.aggregate_safety_rule_results([unspecified]).recommended_kind,
        _RESULT.DECISION_KIND_REJECT,
    )

  def test_violated_count_ignores_compliant_inputs(self):
    agg = _AGG.aggregate_safety_rule_results(
        [
            _compliant(),
            _reject("a.rule"),
            _compliant(),
            _reject("b.rule"),
            _compliant(),
        ]
    )
    self.assertEqual(agg.violated_count, 2)

  def test_does_not_mutate_inputs(self):
    results = [_project("speed.max", 1.5), _project("altitude.min", 3.0)]
    snapshot = list(results)
    _AGG.aggregate_safety_rule_results(results)
    self.assertEqual(results, snapshot)


if __name__ == "__main__":
  unittest.main()
