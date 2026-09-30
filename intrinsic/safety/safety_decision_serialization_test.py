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

"""Serialization and textproto tests for the SafetyDecision proto."""

import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.safety import safety_decision_assessor as assessor
from intrinsic.safety import safety_decision_policy as policy
from intrinsic.safety.proto import safety_decision_pb2
from intrinsic.vehicle.proto import vehicle_command_pb2

_DIGEST = "5c12e76d610e5cf3f5b0d6c402bcfef05db2ec4deb5faacd03cfecdd2f3e34ae"
_SNAPSHOT_ID = (
    "6ceb1af0dca7d2b01d4e876fe699fb8c5d5709d5a4938ea3f46e8d355d3b33de"
)

# Canonical serialization of _fill_project_decision. Keep in sync with
# safety_decision_serialization_test.cc.
_PROJECT_GOLDEN_HEX = (
    "0a36080b120c088ae2cfaa061080cab5ee011a06088be2cfaa06220d7361666574795f66"
    "696c74657232096d6f6e6f746f6e69633a02080110021a40356331326537366436313065"
    "356366336635623064366334303262636665663035646232656334646562356661616364"
    "3033636665636464326633653334616522580a3c080c120c088ae2cfaa061080cab5ee01"
    "1a06088be2cfaa06220d7361666574795f66696c7465722a04626f647932096d6f6e6f74"
    "6f6e69633a0208011a0b0a0909000000000000d03f2a02080131000000000000e03f2a40"
    "366365623161663064636137643262303164346538373666653639396662386335643537"
    "30396435613439333865613366343665386433353564336233336465323a0a0b73706565"
    "645f6c696d697410021a16466f727761726420737065656420726564756365642e221152"
    "657175657374656420302e35206d2f733802"
)
_APPLIED_INTENT_DIGEST = (
    "f800488ed44145af782a5d1ac6c9ca7fff4d548b03fa591c7b363fc7216f8d7f"
)
_EXAMPLES = (
    "accept.textproto",
    "project.textproto",
    "reject.textproto",
    "abort.textproto",
    "surface.textproto",
)


def _fill_header(header, sequence, frame_id):
  header.sequence = sequence
  header.source_time.seconds = 1700000010
  header.source_time.nanos = 500000000
  header.receive_time.seconds = 1700000011
  header.source_id = "safety_filter"
  header.frame_id = frame_id
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _twist_motion():
  motion = vehicle_command_pb2.DesiredMotion()
  _fill_header(motion.header, 12, "body")
  motion.twist.body_twist.linear_x_m_s = 0.25
  motion.horizon.seconds = 1
  motion.confidence = 0.5
  return motion


def _fill_project_decision():
  decision = safety_decision_pb2.SafetyDecision()
  _fill_header(decision.header, 11, "")
  decision.kind = safety_decision_pb2.SAFETY_DECISION_KIND_PROJECT
  decision.original_intent_digest = _DIGEST
  decision.applied_intent.CopyFrom(_twist_motion())
  decision.snapshot_id = _SNAPSHOT_ID
  finding = decision.findings.add()
  finding.rule_id = "speed_limit"
  finding.severity = safety_decision_pb2.SAFETY_FINDING_SEVERITY_WARNING
  finding.summary = "Forward speed reduced."
  finding.detail = "Requested 0.5 m/s"
  decision.decision_epoch = 2
  return decision


def _load_example(name):
  root = os.environ.get("TEST_SRCDIR", "")
  matches = []
  for dirpath, _, filenames in os.walk(root):
    if name in filenames:
      matches.append(os.path.join(dirpath, name))
  if not matches:
    raise AssertionError("missing %s under %s" % (name, root))
  matches.sort(key=lambda path: ("safety" not in path, len(path)))
  with open(matches[0], encoding="utf-8") as handle:
    return handle.read()


def _load_decision(name):
  return text_format.Parse(
      _load_example(name), safety_decision_pb2.SafetyDecision()
  )


class SafetyDecisionSerializationTest(unittest.TestCase):

  def test_enum_numbers_are_locked(self):
    kinds = safety_decision_pb2.SafetyDecisionKind.items()
    self.assertEqual(
        dict(kinds),
        {
            "SAFETY_DECISION_KIND_UNSPECIFIED": 0,
            "SAFETY_DECISION_KIND_ACCEPT": 1,
            "SAFETY_DECISION_KIND_PROJECT": 2,
            "SAFETY_DECISION_KIND_REJECT": 3,
            "SAFETY_DECISION_KIND_ABORT": 4,
            "SAFETY_DECISION_KIND_SURFACE": 5,
        },
    )
    severities = safety_decision_pb2.SafetyFindingSeverity.items()
    self.assertEqual(
        dict(severities),
        {
            "SAFETY_FINDING_SEVERITY_UNSPECIFIED": 0,
            "SAFETY_FINDING_SEVERITY_INFO": 1,
            "SAFETY_FINDING_SEVERITY_WARNING": 2,
            "SAFETY_FINDING_SEVERITY_ERROR": 3,
            "SAFETY_FINDING_SEVERITY_CRITICAL": 4,
        },
    )

  def test_field_numbers_are_locked(self):
    decision = safety_decision_pb2.SafetyDecision.DESCRIPTOR.fields_by_name
    self.assertEqual(
        {name: field.number for name, field in decision.items()},
        {
            "header": 1,
            "kind": 2,
            "original_intent_digest": 3,
            "applied_intent": 4,
            "snapshot_id": 5,
            "findings": 6,
            "decision_epoch": 7,
        },
    )
    finding = safety_decision_pb2.SafetyFinding.DESCRIPTOR.fields_by_name
    self.assertEqual(
        {name: field.number for name, field in finding.items()},
        {"rule_id": 1, "severity": 2, "summary": 3, "detail": 4},
    )
    message = safety_decision_pb2.SafetyDecision()
    message.decision_epoch = 0
    self.assertTrue(message.HasField("decision_epoch"))
    self.assertFalse(safety_decision_pb2.SafetyFinding().HasField("detail"))

  def test_empty_decision_serializes_to_zero_bytes(self):
    decision = safety_decision_pb2.SafetyDecision()
    self.assertEqual(decision.ByteSize(), 0)
    self.assertEqual(decision.SerializeToString(), b"")
    assessment = assessor.assess_safety_decision_proto(decision)
    self.assertFalse(assessment.engaged)
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertFalse(assessment.accepted)

  def test_project_golden_bytes(self):
    golden = bytes.fromhex(_PROJECT_GOLDEN_HEX)
    self.assertEqual(_fill_project_decision().SerializeToString(), golden)
    parsed = safety_decision_pb2.SafetyDecision.FromString(golden)
    self.assertEqual(parsed.SerializeToString(), golden)
    self.assertTrue(assessor.assess_safety_decision_proto(parsed).accepted)

  def test_clearing_epoch_leaves_a_prefix_of_the_golden(self):
    decision = _fill_project_decision()
    decision.ClearField("decision_epoch")
    golden = bytes.fromhex(_PROJECT_GOLDEN_HEX)
    self.assertEqual(decision.SerializeToString(), golden[:-2])
    self.assertEqual(decision.decision_epoch, 0)
    self.assertTrue(assessor.assess_safety_decision_proto(decision).accepted)

  def test_original_intent_digest_is_sha256_of_motion(self):
    digest = assessor.compute_original_intent_digest(_twist_motion())
    self.assertEqual(digest, _APPLIED_INTENT_DIGEST)
    self.assertTrue(policy.is_lowercase_sha256_hex(digest))

  def test_unknown_kind_round_trips_and_is_not_rewritten(self):
    decision = _fill_project_decision()
    decision.ClearField("applied_intent")
    decision.kind = 99
    wire = decision.SerializeToString()

    parsed = safety_decision_pb2.SafetyDecision.FromString(wire)
    self.assertEqual(parsed.kind, 99)
    self.assertEqual(parsed.SerializeToString(), wire)

    assessment = assessor.assess_safety_decision_proto(parsed)
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertIs(assessment.kind, policy.SafetyDecisionKind.UNKNOWN)
    self.assertFalse(assessment.accepted)
    self.assertNotEqual(
        parsed.kind, safety_decision_pb2.SAFETY_DECISION_KIND_REJECT
    )
    self.assertNotEqual(
        parsed.kind, safety_decision_pb2.SAFETY_DECISION_KIND_ABORT
    )
    self.assertEqual(parsed.kind, 99)
    self.assertEqual(parsed.SerializeToString(), wire)

  def test_unknown_kind_from_hand_built_wire_is_kept(self):
    decision = _fill_project_decision()
    decision.ClearField("applied_intent")
    decision.kind = safety_decision_pb2.SAFETY_DECISION_KIND_UNSPECIFIED
    # Field 2 (kind) as varint 99, appended after the known fields.
    wire = decision.SerializeToString() + b"\x10\x63"
    parsed = safety_decision_pb2.SafetyDecision.FromString(wire)
    self.assertEqual(parsed.kind, 99)
    assessment = assessor.assess_safety_decision_proto(parsed)
    self.assertIs(assessment.error, policy.SafetyDecisionError.NONE)
    self.assertFalse(assessment.accepted)

  def test_unknown_severity_round_trips_and_is_kept(self):
    decision = _fill_project_decision()
    decision.findings[0].severity = 77
    wire = decision.SerializeToString()
    parsed = safety_decision_pb2.SafetyDecision.FromString(wire)
    self.assertEqual(parsed.findings[0].severity, 77)
    self.assertEqual(parsed.SerializeToString(), wire)
    self.assertTrue(assessor.assess_safety_decision_proto(parsed).accepted)
    self.assertEqual(parsed.findings[0].severity, 77)
    self.assertIs(
        policy.classify_safety_finding_severity(77),
        policy.SafetyFindingSeverityKind.UNKNOWN,
    )

  def test_unknown_field_is_kept(self):
    with_unknown = bytes.fromhex(_PROJECT_GOLDEN_HEX) + b"\xa0\x06\x07"
    parsed = safety_decision_pb2.SafetyDecision.FromString(with_unknown)
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    self.assertTrue(assessor.assess_safety_decision_proto(parsed).accepted)

  def test_duplicate_rule_ids_are_not_rewritten(self):
    decision = _fill_project_decision()
    duplicate = decision.findings.add()
    duplicate.rule_id = "speed_limit"
    duplicate.severity = safety_decision_pb2.SAFETY_FINDING_SEVERITY_CRITICAL
    parsed = safety_decision_pb2.SafetyDecision.FromString(
        decision.SerializeToString()
    )
    self.assertEqual(len(parsed.findings), 2)
    self.assertEqual(parsed.findings[0].rule_id, parsed.findings[1].rule_id)
    self.assertTrue(assessor.assess_safety_decision_proto(parsed).accepted)

  def test_textproto_examples(self):
    expected = {
        "accept.textproto": (policy.SafetyDecisionKind.ACCEPT, False),
        "project.textproto": (policy.SafetyDecisionKind.PROJECT, True),
        "reject.textproto": (policy.SafetyDecisionKind.REJECT, False),
        "abort.textproto": (policy.SafetyDecisionKind.ABORT, False),
        "surface.textproto": (policy.SafetyDecisionKind.SURFACE, True),
    }
    for name, (kind, has_applied) in expected.items():
      decision = _load_decision(name)
      self.assertEqual(decision.HasField("applied_intent"), has_applied, name)
      assessment = assessor.assess_safety_decision_proto(decision)
      self.assertIs(assessment.kind, kind, name)
      self.assertTrue(assessment.accepted, name)
    self.assertEqual(len(_load_decision("reject.textproto").findings), 2)
    self.assertTrue(
        _load_decision("surface.textproto").applied_intent.HasField("pose")
    )

  def test_textproto_examples_round_trip_through_wire(self):
    for name in _EXAMPLES:
      example = _load_decision(name)
      wire = example.SerializeToString()
      parsed = safety_decision_pb2.SafetyDecision.FromString(wire)
      self.assertEqual(parsed.SerializeToString(), wire, name)
      self.assertTrue(
          assessor.assess_safety_decision_proto(parsed).accepted, name
      )


if __name__ == "__main__":
  unittest.main()
