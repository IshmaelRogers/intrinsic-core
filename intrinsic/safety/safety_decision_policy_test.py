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

"""Plain-value tests for the SafetyDecision host assessment."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.safety import safety_decision_policy as policy
from intrinsic.vehicle import vehicle_contract_policy

_DIGEST = "a" * 64
_SNAPSHOT_ID = "0" * 64


def _twist_motion():
  return vehicle_contract_policy.DesiredMotionView(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id=vehicle_contract_policy.BODY_FRAME_ID,
      objective=vehicle_contract_policy.ObjectiveKind.TWIST,
      twist=(0.25, 0.0, 0.0, 0.0, 0.0, 0.0),
  )


def _decision(kind, **overrides):
  fields = dict(
      header_present=True,
      header_validity_present=True,
      header_validity_state=1,
      kind=kind,
      original_intent_digest=_DIGEST,
      snapshot_id=_SNAPSHOT_ID,
  )
  fields.update(overrides)
  return policy.SafetyDecisionView(**fields)


def _project(kind=2, **overrides):
  fields = dict(applied_intent_present=True, applied_intent=_twist_motion())
  fields.update(overrides)
  return _decision(kind, **fields)


class SafetyDecisionPolicyTest(unittest.TestCase):

  def test_empty_decision_is_not_engaged_and_not_an_error(self):
    assessment = policy.assess_safety_decision(policy.SafetyDecisionView())
    self.assertFalse(assessment.engaged)
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertFalse(assessment.accepted)
    self.assertEqual(assessment.finding_index, -1)

  def test_any_field_engages_the_decision(self):
    engaged = [
        policy.SafetyDecisionView(kind=3),
        policy.SafetyDecisionView(original_intent_digest=_DIGEST),
        policy.SafetyDecisionView(decision_epoch=1),
        policy.SafetyDecisionView(decision_epoch_present=True),
        policy.SafetyDecisionView(
            findings=(policy.SafetyFindingView(rule_id="r", severity=1),)
        ),
    ]
    for decision in engaged:
      assessment = policy.assess_safety_decision(decision)
      self.assertTrue(assessment.engaged)
      self.assertIs(assessment.error, policy.SafetyDecisionError.MISSING_HEADER)
      self.assertFalse(assessment.accepted)

  def test_accept_without_applied_intent_is_accepted(self):
    assessment = policy.assess_safety_decision(_decision(1))
    self.assertTrue(assessment.engaged)
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.ACCEPT)
    self.assertIs(
        assessment.header_validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(assessment.accepted)

  def test_project_and_surface_with_valid_applied_intent(self):
    self.assertTrue(policy.assess_safety_decision(_project()).accepted)
    assessment = policy.assess_safety_decision(_project(kind=5))
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.SURFACE)
    self.assertTrue(assessment.accepted)

  def test_reject_and_abort_are_accepted_without_applied_intent(self):
    for kind in (3, 4):
      assessment = policy.assess_safety_decision(_decision(kind))
      self.assertIs(assessment.error, policy.SafetyDecisionError.NONE, kind)
      self.assertTrue(assessment.accepted, kind)

  def test_applied_intent_presence_rules_per_kind(self):
    for kind in (2, 5):
      assessment = policy.assess_safety_decision(_decision(kind))
      self.assertIs(
          assessment.error,
          policy.SafetyDecisionError.APPLIED_INTENT_MISSING,
          kind,
      )
      self.assertFalse(assessment.accepted)
    for kind in (1, 3, 4):
      assessment = policy.assess_safety_decision(_project(kind=kind))
      self.assertIs(
          assessment.error,
          policy.SafetyDecisionError.APPLIED_INTENT_UNEXPECTED,
          kind,
      )
      self.assertFalse(assessment.accepted)

  def test_empty_applied_intent_message_is_still_present(self):
    accept = _decision(1, applied_intent_present=True)
    self.assertIs(
        policy.assess_safety_decision(accept).error,
        policy.SafetyDecisionError.APPLIED_INTENT_UNEXPECTED,
    )
    project = _decision(2, applied_intent_present=True)
    assessment = policy.assess_safety_decision(project)
    self.assertIs(
        assessment.error, policy.SafetyDecisionError.APPLIED_INTENT_INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_applied_intent_delegates_to_desired_motion(self):
    bad_frame = _project(
        applied_intent=dataclasses.replace(_twist_motion(), frame_id="world")
    )
    assessment = policy.assess_safety_decision(bad_frame)
    self.assertIs(
        assessment.error, policy.SafetyDecisionError.APPLIED_INTENT_INVALID
    )
    self.assertIs(
        assessment.applied_intent_error,
        vehicle_contract_policy.ContractError.BODY_FRAME,
    )
    self.assertFalse(assessment.accepted)

    for bad in (math.nan, math.inf, -math.inf):
      non_finite = _project(
          applied_intent=dataclasses.replace(
              _twist_motion(), twist=(bad, 0.0, 0.0, 0.0, 0.0, 0.0)
          )
      )
      assessment = policy.assess_safety_decision(non_finite)
      self.assertIs(
          assessment.applied_intent_error,
          vehicle_contract_policy.ContractError.NON_FINITE,
      )
      self.assertFalse(assessment.accepted)

    not_valid = _project(
        applied_intent=dataclasses.replace(_twist_motion(), validity_state=2)
    )
    self.assertIs(
        policy.assess_safety_decision(not_valid).error,
        policy.SafetyDecisionError.APPLIED_INTENT_INVALID,
    )

  def test_digest_and_snapshot_id_format(self):
    self.assertTrue(policy.is_lowercase_sha256_hex("f" * 64))
    self.assertTrue(policy.is_lowercase_sha256_hex("9" * 64))
    for bad in (
        "",
        "a" * 63,
        "a" * 65,
        "A" * 64,
        "g" * 64,
        "a" * 64 + "\n",
        "sha256:" + "a" * 57,
    ):
      self.assertFalse(policy.is_lowercase_sha256_hex(bad), repr(bad))
    for bad in ("", "a" * 63, "a" * 65, "A" * 64):
      digest = policy.assess_safety_decision(
          _decision(1, original_intent_digest=bad)
      )
      self.assertIs(
          digest.error, policy.SafetyDecisionError.ORIGINAL_INTENT_DIGEST
      )
      snapshot = policy.assess_safety_decision(_decision(1, snapshot_id=bad))
      self.assertIs(snapshot.error, policy.SafetyDecisionError.SNAPSHOT_ID)
      self.assertFalse(snapshot.accepted)

  def test_findings_require_rule_id_and_allow_duplicates(self):
    duplicates = (
        policy.SafetyFindingView(rule_id="speed_limit", severity=2),
        policy.SafetyFindingView(rule_id="speed_limit", severity=3),
    )
    self.assertTrue(
        policy.assess_safety_decision(
            _decision(3, findings=duplicates)
        ).accepted
    )
    with_empty = (
        policy.SafetyFindingView(rule_id="a", severity=1),
        policy.SafetyFindingView(rule_id="", severity=1),
        policy.SafetyFindingView(rule_id="", severity=2),
    )
    assessment = policy.assess_safety_decision(
        _decision(3, findings=with_empty)
    )
    self.assertIs(assessment.error, policy.SafetyDecisionError.FINDING_RULE_ID)
    self.assertEqual(assessment.finding_index, 1)
    self.assertFalse(assessment.accepted)

  def test_unknown_severity_is_not_a_defect(self):
    findings = (
        policy.SafetyFindingView(rule_id="a", severity=99),
        policy.SafetyFindingView(rule_id="b", severity=-1),
    )
    self.assertTrue(
        policy.assess_safety_decision(_decision(4, findings=findings)).accepted
    )
    classify = policy.classify_safety_finding_severity
    kinds = policy.SafetyFindingSeverityKind
    self.assertIs(classify(99), kinds.UNKNOWN)
    self.assertIs(classify(-1), kinds.UNKNOWN)
    self.assertIs(classify(0), kinds.UNSPECIFIED)
    self.assertIs(classify(1), kinds.INFO)
    self.assertIs(classify(2), kinds.WARNING)
    self.assertIs(classify(3), kinds.ERROR)
    self.assertIs(classify(4), kinds.CRITICAL)

  def test_unspecified_kind_is_a_structural_defect(self):
    assessment = policy.assess_safety_decision(_decision(0))
    self.assertIs(assessment.error, policy.SafetyDecisionError.UNSPECIFIED_KIND)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.UNSPECIFIED)
    self.assertFalse(assessment.accepted)

  def test_unknown_kind_is_not_a_structural_defect_and_not_accepted(self):
    for kind in (6, 99, -1):
      assessment = policy.assess_safety_decision(_decision(kind))
      self.assertTrue(assessment.engaged)
      self.assertIs(assessment.error, policy.SafetyDecisionError.NONE, kind)
      self.assertIs(assessment.kind, policy.SafetyDecisionKind.UNKNOWN, kind)
      self.assertFalse(assessment.accepted, kind)
    classified = policy.classify_safety_decision_kind(99)
    self.assertIs(classified, policy.SafetyDecisionKind.UNKNOWN)
    self.assertIsNot(classified, policy.SafetyDecisionKind.REJECT)
    self.assertIsNot(classified, policy.SafetyDecisionKind.ABORT)

  def test_unknown_kind_still_reports_other_defects(self):
    assessment = policy.assess_safety_decision(_decision(99, snapshot_id=""))
    self.assertIs(assessment.error, policy.SafetyDecisionError.SNAPSHOT_ID)
    with_intent = policy.assess_safety_decision(_project(kind=99))
    self.assertIs(with_intent.error, policy.SafetyDecisionError.NONE)
    self.assertFalse(with_intent.accepted)

  def test_kind_classification_for_known_values(self):
    expected = {
        0: policy.SafetyDecisionKind.UNSPECIFIED,
        1: policy.SafetyDecisionKind.ACCEPT,
        2: policy.SafetyDecisionKind.PROJECT,
        3: policy.SafetyDecisionKind.REJECT,
        4: policy.SafetyDecisionKind.ABORT,
        5: policy.SafetyDecisionKind.SURFACE,
        6: policy.SafetyDecisionKind.UNKNOWN,
    }
    for wire, kind in expected.items():
      self.assertIs(policy.classify_safety_decision_kind(wire), kind)

  def test_header_rules(self):
    no_header = policy.assess_safety_decision(
        _decision(1, header_present=False)
    )
    self.assertIs(no_header.error, policy.SafetyDecisionError.MISSING_HEADER)

    invalid = policy.assess_safety_decision(
        _decision(1, header_validity_state=2)
    )
    self.assertIs(invalid.error, policy.SafetyDecisionError.HEADER_INVALID)
    self.assertIs(
        invalid.header_validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(invalid.accepted)

    bad_source = policy.assess_safety_decision(
        _decision(
            1,
            header_source_time_present=True,
            header_source_time_nanos=1000000000,
        )
    )
    self.assertIs(bad_source.error, policy.SafetyDecisionError.HEADER_TIME)
    bad_receive = policy.assess_safety_decision(
        _decision(
            1,
            header_receive_time_present=True,
            header_receive_time_nanos=-1,
        )
    )
    self.assertIs(bad_receive.error, policy.SafetyDecisionError.HEADER_TIME)

  def test_header_without_valid_validity_is_not_accepted(self):
    for present in (False, True):
      assessment = policy.assess_safety_decision(
          _decision(3, header_validity_present=present, header_validity_state=0)
      )
      self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
      self.assertFalse(assessment.accepted)
    unknown = policy.assess_safety_decision(
        _decision(3, header_validity_state=99)
    )
    self.assertFalse(unknown.accepted)
    self.assertIs(
        unknown.header_validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )

  def test_first_defect_wins(self):
    bad = policy.SafetyDecisionView(
        kind=0,
        original_intent_digest="bad",
        snapshot_id="bad",
        applied_intent_present=True,
        findings=(policy.SafetyFindingView(rule_id="", severity=1),),
    )
    errors = policy.SafetyDecisionError

    def error_of(decision):
      return policy.assess_safety_decision(decision).error

    self.assertIs(error_of(bad), errors.MISSING_HEADER)
    bad = dataclasses.replace(bad, header_present=True)
    self.assertIs(error_of(bad), errors.UNSPECIFIED_KIND)
    bad = dataclasses.replace(bad, kind=1)
    self.assertIs(error_of(bad), errors.ORIGINAL_INTENT_DIGEST)
    bad = dataclasses.replace(bad, original_intent_digest=_DIGEST)
    self.assertIs(error_of(bad), errors.SNAPSHOT_ID)
    bad = dataclasses.replace(bad, snapshot_id=_SNAPSHOT_ID)
    self.assertIs(error_of(bad), errors.APPLIED_INTENT_UNEXPECTED)
    bad = dataclasses.replace(bad, applied_intent_present=False)
    self.assertIs(error_of(bad), errors.FINDING_RULE_ID)


if __name__ == "__main__":
  unittest.main()
