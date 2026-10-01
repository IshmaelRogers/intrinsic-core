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
"""Tests for the nav.covariance rule."""

import math
import unittest

from intrinsic.safety import nav_covariance_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = nav_covariance_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result

_EPS = 1e-6


def _diag(a, b=0.01, c=0.01):
  return [a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c]


def _sample(
    position_var=0.25,
    velocity_var=0.04,
    position_known=True,
    velocity_known=True,
):
  return _RULE.NavCovarianceSample(
      position_known=position_known,
      velocity_known=velocity_known,
      position_cov=_diag(position_var),
      velocity_cov=_diag(velocity_var),
  )


def _with(sample, **kwargs):
  fields = {
      "position_known": sample.position_known,
      "velocity_known": sample.velocity_known,
      "position_cov": list(sample.position_cov),
      "velocity_cov": list(sample.velocity_cov),
  }
  fields.update(kwargs)
  return _RULE.NavCovarianceSample(**fields)


def _set(cov, **index_values):
  cov = list(cov)
  for key, value in index_values.items():
    cov[int(key[1:])] = value
  return cov


class NavCovarianceRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_critical(self, result):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "nav.covariance")
    self.assertEqual(result.rule_id, _RULE.NAV_COVARIANCE_RULE_ID)
    self.assertEqual(result.severity, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_exceeded(self, result, metric, sigma):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "nav.covariance")
    self.assertEqual(result.severity, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertNotEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )
    self.assertTrue(result.has_projected_value)
    self.assertAlmostEqual(result.projected_value, sigma, places=12)
    self.assertIn(f"metric={metric}", result.summary)
    self.assertIn("sigma=", result.summary)

  def test_rule_id_and_defaults_are_locked(self):
    self.assertEqual(_RULE.NAV_COVARIANCE_RULE_ID, "nav.covariance")
    self.assertEqual(_RULE.DEFAULT_MAX_POSITION_SIGMA_M, 1.0)
    self.assertEqual(_RULE.DEFAULT_MAX_VELOCITY_SIGMA_MPS, 0.5)

  def test_nominal_diagonal_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_nav_covariance_rule(_sample()))
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(_sample(0.0, 0.0))
    )

  def test_position_just_under_equal_over(self):
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(_sample(1.0 - _EPS))
    )
    self._expect_compliant(_RULE.evaluate_nav_covariance_rule(_sample(1.0)))
    self._expect_exceeded(
        _RULE.evaluate_nav_covariance_rule(_sample(1.0 + _EPS)),
        "position",
        math.sqrt(1.0 + _EPS),
    )

  def test_velocity_just_under_equal_over(self):
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(_sample(velocity_var=0.25 - _EPS))
    )
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(_sample(velocity_var=0.25))
    )
    self._expect_exceeded(
        _RULE.evaluate_nav_covariance_rule(_sample(velocity_var=0.25 + _EPS)),
        "velocity",
        math.sqrt(0.25 + _EPS),
    )

  def test_sigma_is_sqrt_of_max_diagonal(self):
    sample = _with(_sample(), position_cov=_diag(0.25, 0.01, 4.0))
    self._expect_exceeded(
        _RULE.evaluate_nav_covariance_rule(sample), "position", 2.0
    )

  def test_exceeded_summary_includes_metric_and_sigma(self):
    self.assertEqual(
        _RULE.evaluate_nav_covariance_rule(_sample(4.0)).summary,
        "nav covariance exceeded metric=position sigma=2",
    )
    self.assertEqual(
        _RULE.evaluate_nav_covariance_rule(_sample(velocity_var=1.0)).summary,
        "nav covariance exceeded metric=velocity sigma=1",
    )

  def test_both_exceeded_reports_position_first(self):
    result = _RULE.evaluate_nav_covariance_rule(_sample(4.0, 1.0))
    self._expect_exceeded(result, "position", 2.0)
    self.assertNotIn("metric=velocity", result.summary)

  def test_custom_thresholds_are_honored(self):
    sample = _sample(4.0, 1.0)
    self._expect_compliant(_RULE.evaluate_nav_covariance_rule(sample, 2.0, 1.0))
    self._expect_exceeded(
        _RULE.evaluate_nav_covariance_rule(_sample(4.0 + _EPS, 1.0), 2.0, 1.0),
        "position",
        math.sqrt(4.0 + _EPS),
    )
    self._expect_exceeded(
        _RULE.evaluate_nav_covariance_rule(_sample(4.0, 1.0 + _EPS), 2.0, 1.0),
        "velocity",
        math.sqrt(1.0 + _EPS),
    )

  def test_unknown_covariance_is_critical_even_when_numbers_pass(self):
    result = _RULE.evaluate_nav_covariance_rule(_sample(position_known=False))
    self._expect_critical(result)
    self.assertIn("unknown covariance", result.summary)
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(_sample(velocity_known=False))
    )
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _sample(position_known=False, velocity_known=False)
        )
    )

  def test_negative_diagonal_is_critical(self):
    for index in (0, 4, 8):
      result = _RULE.evaluate_nav_covariance_rule(
          _with(
              _sample(), position_cov=_set(_diag(0.25), **{f"i{index}": -1.0})
          )
      )
      self._expect_critical(result)
      self.assertIn("position covariance not usable", result.summary)
      result = _RULE.evaluate_nav_covariance_rule(
          _with(
              _sample(), velocity_cov=_set(_diag(0.04), **{f"i{index}": -1.0})
          )
      )
      self._expect_critical(result)
      self.assertIn("velocity covariance not usable", result.summary)

  def test_non_psd_with_positive_diagonal_is_critical(self):
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _with(
                _sample(),
                position_cov=_set(_diag(1.0, 0.01, 0.01), i1=2.0, i3=2.0),
            )
        )
    )
    indefinite = [1, 0.9, 0.9, 0.9, 1, -0.9, 0.9, -0.9, 1]
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _with(_sample(), velocity_cov=indefinite)
        )
    )

  def test_correlated_psd_is_compliant(self):
    cov = _set(_diag(0.25, 0.25, 0.01), i1=0.2, i3=0.2)
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(_with(_sample(), position_cov=cov))
    )

  def test_asymmetric_matrix_is_critical(self):
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _with(_sample(), position_cov=_set(_diag(0.25), i1=0.01, i3=0.0))
        )
    )
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _with(_sample(), velocity_cov=_set(_diag(0.04), i5=0.0, i7=1e-3))
        )
    )

  def test_small_asymmetry_within_eps_is_accepted(self):
    self._expect_compliant(
        _RULE.evaluate_nav_covariance_rule(
            _with(_sample(), position_cov=_set(_diag(0.25), i1=5e-10, i3=0.0))
        )
    )

  def test_non_finite_entry_is_critical(self):
    for bad in (math.nan, math.inf, -math.inf):
      for index in range(9):
        self._expect_critical(
            _RULE.evaluate_nav_covariance_rule(
                _with(
                    _sample(),
                    position_cov=_set(_diag(0.25), **{f"i{index}": bad}),
                )
            )
        )
        self._expect_critical(
            _RULE.evaluate_nav_covariance_rule(
                _with(
                    _sample(),
                    velocity_cov=_set(_diag(0.04), **{f"i{index}": bad}),
                )
            )
        )

  def test_wrong_length_matrix_is_critical(self):
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(
            _with(_sample(), position_cov=[0.25, 0.0, 0.0])
        )
    )
    self._expect_critical(
        _RULE.evaluate_nav_covariance_rule(_with(_sample(), velocity_cov=[]))
    )

  def test_bad_thresholds_are_critical(self):
    sample = _sample()
    for bad in (0.0, -1.0, math.nan, math.inf, -math.inf):
      result = _RULE.evaluate_nav_covariance_rule(sample, bad, 0.5)
      self._expect_critical(result)
      self.assertIn("bad config", result.summary)
      self._expect_critical(
          _RULE.evaluate_nav_covariance_rule(sample, 1.0, bad)
      )

  def test_bad_threshold_beats_unknown(self):
    result = _RULE.evaluate_nav_covariance_rule(
        _sample(position_known=False), 0.0, 0.5
    )
    self._expect_critical(result)
    self.assertIn("bad config", result.summary)


if __name__ == "__main__":
  unittest.main()
