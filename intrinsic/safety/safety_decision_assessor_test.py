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

"""Protobuf-level tests for the SafetyDecision host assessment."""

import math
import unittest

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.safety import safety_decision_assessor as assessor
from intrinsic.safety import safety_decision_policy as policy
from intrinsic.safety.proto import safety_decision_pb2
from intrinsic.vehicle import vehicle_contract_policy
from intrinsic.vehicle.proto import vehicle_command_pb2
from intrinsic.world.world_snapshot import world_snapshot_policy

_KIND = safety_decision_pb2.SafetyDecisionKind
_ERROR = policy.SafetyDecisionError


def _fill_header(header, frame_id):
  header.sequence = 1
  header.source_time.seconds = 1700000010
  header.source_time.nanos = 500000000
  header.receive_time.seconds = 1700000011
  header.source_id = "safety_filter"
  header.frame_id = frame_id
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _twist_motion():
  motion = vehicle_command_pb2.DesiredMotion()
  _fill_header(motion.header, vehicle_contract_policy.BODY_FRAME_ID)
  motion.twist.body_twist.linear_x_m_s = 0.25
  return motion


def _ascent_pose_motion():
  motion = vehicle_command_pb2.DesiredMotion()
  _fill_header(motion.header, "world_enu")
  pose = motion.pose.pose_world_from_body
  pose.position.x = 4
  pose.position.z = 0
  pose.orientation.w = 1
  return motion


def _decision(kind, original=None):
  original = original if original is not None else _twist_motion()
  decision = safety_decision_pb2.SafetyDecision()
  _fill_header(decision.header, "")
  decision.kind = kind
  decision.original_intent_digest = assessor.compute_original_intent_digest(
      original
  )
  decision.snapshot_id = "0" * 64
  return decision


class SafetyDecisionAssessorTest(unittest.TestCase):

  def test_empty_decision_is_not_engaged(self):
    assessment = assessor.assess_safety_decision_proto(
        safety_decision_pb2.SafetyDecision()
    )
    self.assertFalse(assessment.engaged)
    self.assertIs(assessment.error, _ERROR.NONE)
    self.assertFalse(assessment.accepted)

  def test_accept_has_digest_and_snapshot_id_and_no_applied_intent(self):
    decision = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    self.assertFalse(decision.HasField("applied_intent"))
    assessment = assessor.assess_safety_decision_proto(decision)
    self.assertIs(assessment.error, _ERROR.NONE)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.ACCEPT)
    self.assertTrue(assessment.accepted)

  def test_project_with_valid_applied_intent_is_accepted(self):
    decision = _decision(_KIND.SAFETY_DECISION_KIND_PROJECT)
    decision.applied_intent.CopyFrom(_twist_motion())
    decision.applied_intent.twist.body_twist.linear_x_m_s = 0.1
    assessment = assessor.assess_safety_decision_proto(decision)
    self.assertIs(assessment.error, _ERROR.NONE)
    self.assertTrue(assessment.accepted)

  def test_reject_and_abort_without_applied_intent(self):
    for kind in (
        _KIND.SAFETY_DECISION_KIND_REJECT,
        _KIND.SAFETY_DECISION_KIND_ABORT,
    ):
      assessment = assessor.assess_safety_decision_proto(_decision(kind))
      self.assertIs(assessment.error, _ERROR.NONE, kind)
      self.assertTrue(assessment.accepted, kind)

  def test_surface_with_pose_and_twist_ascent_fixtures(self):
    for ascent in (_ascent_pose_motion(), _twist_motion()):
      decision = _decision(_KIND.SAFETY_DECISION_KIND_SURFACE)
      decision.applied_intent.CopyFrom(ascent)
      assessment = assessor.assess_safety_decision_proto(decision)
      self.assertIs(assessment.error, _ERROR.NONE)
      self.assertIs(assessment.kind, policy.SafetyDecisionKind.SURFACE)
      self.assertTrue(assessment.accepted)

  def test_invalid_combinations(self):
    def error_of(decision):
      return assessor.assess_safety_decision_proto(decision).error

    self.assertIs(
        error_of(_decision(_KIND.SAFETY_DECISION_KIND_PROJECT)),
        _ERROR.APPLIED_INTENT_MISSING,
    )

    accept_with_intent = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    accept_with_intent.applied_intent.CopyFrom(_twist_motion())
    self.assertIs(
        error_of(accept_with_intent), _ERROR.APPLIED_INTENT_UNEXPECTED
    )

    empty_applied = _decision(_KIND.SAFETY_DECISION_KIND_PROJECT)
    empty_applied.applied_intent.SetInParent()
    self.assertIs(error_of(empty_applied), _ERROR.APPLIED_INTENT_INVALID)

    bad_digest = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    bad_digest.original_intent_digest = "ABC"
    self.assertIs(error_of(bad_digest), _ERROR.ORIGINAL_INTENT_DIGEST)

    upper_digest = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    upper_digest.original_intent_digest = (
        upper_digest.original_intent_digest.upper()
    )
    self.assertIs(error_of(upper_digest), _ERROR.ORIGINAL_INTENT_DIGEST)

    short_snapshot = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    short_snapshot.snapshot_id = "0" * 63
    self.assertIs(error_of(short_snapshot), _ERROR.SNAPSHOT_ID)

    upper_snapshot = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    upper_snapshot.snapshot_id = "A" * 64
    self.assertIs(error_of(upper_snapshot), _ERROR.SNAPSHOT_ID)

    empty_rule = _decision(_KIND.SAFETY_DECISION_KIND_REJECT)
    empty_rule.findings.add().rule_id = "ok"
    empty_rule.findings.add().summary = "no rule id"
    assessment = assessor.assess_safety_decision_proto(empty_rule)
    self.assertIs(assessment.error, _ERROR.FINDING_RULE_ID)
    self.assertEqual(assessment.finding_index, 1)
    self.assertFalse(assessment.accepted)

    no_header = _decision(_KIND.SAFETY_DECISION_KIND_REJECT)
    no_header.ClearField("header")
    self.assertIs(error_of(no_header), _ERROR.MISSING_HEADER)

    self.assertIs(
        error_of(_decision(_KIND.SAFETY_DECISION_KIND_UNSPECIFIED)),
        _ERROR.UNSPECIFIED_KIND,
    )

  def test_applied_intent_must_pass_desired_motion_assessment(self):
    non_finite = _decision(_KIND.SAFETY_DECISION_KIND_PROJECT)
    non_finite.applied_intent.CopyFrom(_twist_motion())
    non_finite.applied_intent.twist.body_twist.linear_x_m_s = math.nan
    assessment = assessor.assess_safety_decision_proto(non_finite)
    self.assertIs(assessment.error, _ERROR.APPLIED_INTENT_INVALID)
    self.assertIs(
        assessment.applied_intent_error,
        vehicle_contract_policy.ContractError.NON_FINITE,
    )
    self.assertFalse(assessment.accepted)

    bad_quaternion = _decision(_KIND.SAFETY_DECISION_KIND_SURFACE)
    bad_quaternion.applied_intent.CopyFrom(_ascent_pose_motion())
    bad_quaternion.applied_intent.pose.pose_world_from_body.orientation.w = 2
    assessment = assessor.assess_safety_decision_proto(bad_quaternion)
    self.assertIs(
        assessment.applied_intent_error,
        vehicle_contract_policy.ContractError.QUATERNION,
    )

    wrong_frame = _decision(_KIND.SAFETY_DECISION_KIND_PROJECT)
    wrong_frame.applied_intent.CopyFrom(_twist_motion())
    wrong_frame.applied_intent.header.frame_id = "world_enu"
    assessment = assessor.assess_safety_decision_proto(wrong_frame)
    self.assertIs(
        assessment.applied_intent_error,
        vehicle_contract_policy.ContractError.BODY_FRAME,
    )

  def test_unknown_kind_is_not_accepted_and_not_rewritten(self):
    decision = _decision(99)
    before = decision.SerializeToString()
    assessment = assessor.assess_safety_decision_proto(decision)
    self.assertIs(assessment.error, _ERROR.NONE)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.UNKNOWN)
    self.assertFalse(assessment.accepted)
    self.assertEqual(decision.kind, 99)
    self.assertEqual(decision.SerializeToString(), before)

  def test_snapshot_id_from_world_snapshot_policy_is_accepted(self):
    snapshot_id = world_snapshot_policy.compute_snapshot_id(
        3,
        (
            world_snapshot_policy.ComponentRevisionView("current_field", 7),
            world_snapshot_policy.ComponentRevisionView(
                "occupancy_reference", 1
            ),
        ),
    )
    self.assertEqual(len(snapshot_id), policy.SHA256_HEX_LENGTH)
    decision = _decision(_KIND.SAFETY_DECISION_KIND_ACCEPT)
    decision.snapshot_id = snapshot_id
    self.assertTrue(assessor.assess_safety_decision_proto(decision).accepted)

  def test_original_intent_digest_is_stable_and_sensitive(self):
    digest = assessor.compute_original_intent_digest(_twist_motion())
    self.assertEqual(
        digest, assessor.compute_original_intent_digest(_twist_motion())
    )
    self.assertTrue(policy.is_lowercase_sha256_hex(digest))
    changed = _twist_motion()
    changed.twist.body_twist.linear_x_m_s = 0.5
    self.assertNotEqual(
        digest, assessor.compute_original_intent_digest(changed)
    )


if __name__ == "__main__":
  unittest.main()
