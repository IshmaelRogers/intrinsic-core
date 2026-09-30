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

"""Tests for the authority transition proto wrappers."""

import csv
import io
import os
import unittest

from intrinsic.safety import authority_transition_assessor as assessor
from intrinsic.safety import authority_transition_policy as policy
from intrinsic.safety.proto import authority_mode_pb2
from intrinsic.safety.proto import safety_decision_pb2

_M = policy.AuthorityMode
_E = policy.AuthorityEvent
_ERR = policy.AuthorityTransitionError
_PB = authority_mode_pb2

_MODE_NUMBERS = {
    "AUTHORITY_MODE_UNSPECIFIED": 0,
    "AUTHORITY_MODE_SHADOW": 1,
    "AUTHORITY_MODE_RECOMMEND": 2,
    "AUTHORITY_MODE_CONSTRAINED": 3,
    "AUTHORITY_MODE_REVOKED": 4,
    "AUTHORITY_MODE_EMERGENCY": 5,
}

_EVENT_NUMBERS = {
    "AUTHORITY_EVENT_UNSPECIFIED": 0,
    "AUTHORITY_EVENT_ARM_RECOMMEND": 1,
    "AUTHORITY_EVENT_ENABLE_CONSTRAINED": 2,
    "AUTHORITY_EVENT_REVOKE": 3,
    "AUTHORITY_EVENT_ENTER_EMERGENCY": 4,
    "AUTHORITY_EVENT_CLEAR_EMERGENCY": 5,
    "AUTHORITY_EVENT_RESET_TO_SHADOW": 6,
    "AUTHORITY_EVENT_FAULT": 7,
}


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


def _number(enum_name):
  return (
      _PB.DESCRIPTOR.enum_types_by_name[
          "AuthorityMode" if "_MODE_" in enum_name else "AuthorityEvent"
      ]
      .values_by_name[enum_name]
      .number
  )


class AuthorityTransitionAssessorTest(unittest.TestCase):

  def test_locked_enum_numbers(self):
    mode_values = {
        v.name: v.number
        for v in _PB.DESCRIPTOR.enum_types_by_name["AuthorityMode"].values
    }
    event_values = {
        v.name: v.number
        for v in _PB.DESCRIPTOR.enum_types_by_name["AuthorityEvent"].values
    }
    self.assertEqual(mode_values, _MODE_NUMBERS)
    self.assertEqual(event_values, _EVENT_NUMBERS)

  def test_wrapper_matches_policy_for_every_live_cell(self):
    for mode_number in range(1, 6):
      for event_number in range(1, 8):
        with self.subTest(mode=mode_number, event=event_number):
          expected = policy.apply_authority_transition(
              policy.classify_authority_mode(mode_number),
              policy.classify_authority_event(event_number),
          )
          self.assertEqual(
              assessor.assess_authority_transition(mode_number, event_number),
              expected,
          )

  def test_arm_from_shadow(self):
    result = assessor.assess_authority_transition(
        _PB.AUTHORITY_MODE_SHADOW, _PB.AUTHORITY_EVENT_ARM_RECOMMEND
    )
    self.assertTrue(result.ok)
    self.assertEqual(result.next_mode, _M.RECOMMEND)

  def test_unknown_numbers_fail_closed(self):
    bad_mode = assessor.assess_authority_transition(
        99, _PB.AUTHORITY_EVENT_ARM_RECOMMEND
    )
    self.assertFalse(bad_mode.ok)
    self.assertEqual(bad_mode.next_mode, _M.UNSPECIFIED)
    self.assertEqual(bad_mode.error, _ERR.INVALID_MODE)
    bad_event = assessor.assess_authority_transition(
        _PB.AUTHORITY_MODE_SHADOW, 99
    )
    self.assertFalse(bad_event.ok)
    self.assertEqual(bad_event.next_mode, _M.SHADOW)
    self.assertEqual(bad_event.error, _ERR.INVALID_EVENT)
    self.assertFalse(
        assessor.assess_authority_transition(
            _PB.AUTHORITY_MODE_UNSPECIFIED, _PB.AUTHORITY_EVENT_UNSPECIFIED
        ).ok
    )

  def test_decision_kind_matrix_with_safety_decision_enum(self):
    sd = safety_decision_pb2
    kinds = (
        sd.SAFETY_DECISION_KIND_ACCEPT,
        sd.SAFETY_DECISION_KIND_PROJECT,
        sd.SAFETY_DECISION_KIND_REJECT,
        sd.SAFETY_DECISION_KIND_ABORT,
        sd.SAFETY_DECISION_KIND_SURFACE,
    )
    allowed = (
        (False, False, True, True, False),
        (False, False, True, True, False),
        (True, True, True, True, True),
        (False, False, True, True, True),
        (False, False, True, True, True),
    )
    for mode_number, row in zip(range(1, 6), allowed):
      for kind, expected in zip(kinds, row):
        with self.subTest(mode=mode_number, kind=kind):
          self.assertEqual(
              assessor.assess_decision_kind_under_authority(mode_number, kind),
              expected,
          )

  def test_decision_kind_unknown_is_false(self):
    sd = safety_decision_pb2
    self.assertFalse(
        assessor.assess_decision_kind_under_authority(
            99, sd.SAFETY_DECISION_KIND_REJECT
        )
    )
    self.assertFalse(
        assessor.assess_decision_kind_under_authority(
            _PB.AUTHORITY_MODE_CONSTRAINED, 99
        )
    )
    self.assertFalse(
        assessor.assess_decision_kind_under_authority(
            _PB.AUTHORITY_MODE_CONSTRAINED,
            sd.SAFETY_DECISION_KIND_UNSPECIFIED,
        )
    )

  def test_unknown_wire_numbers_survive_serialization(self):
    decision = safety_decision_pb2.SafetyDecision(kind=77)
    restored = safety_decision_pb2.SafetyDecision.FromString(
        decision.SerializeToString()
    )
    self.assertEqual(restored.kind, 77)
    self.assertFalse(
        assessor.assess_decision_kind_under_authority(
            _PB.AUTHORITY_MODE_CONSTRAINED, restored.kind
        )
    )
    self.assertEqual(restored.kind, 77)

  def test_transition_table_example_matches_policy(self):
    rows = list(
        csv.DictReader(
            io.StringIO(_load_example("authority_transition_table.csv"))
        )
    )
    self.assertEqual(len(rows), 35)
    for row in rows:
      result = assessor.assess_authority_transition(
          _number(row["from_mode"]), _number(row["event"])
      )
      with self.subTest(row=row):
        self.assertEqual(result.ok, row["ok"] == "true")
        if result.ok:
          self.assertEqual(result.next_mode.value, _number(row["next_mode"]))
        else:
          self.assertEqual(row["next_mode"], "")

  def test_decision_matrix_example_matches_policy(self):
    rows = list(
        csv.DictReader(
            io.StringIO(_load_example("authority_decision_matrix.csv"))
        )
    )
    self.assertEqual(len(rows), 25)
    kinds = safety_decision_pb2.SafetyDecisionKind
    for row in rows:
      allowed = assessor.assess_decision_kind_under_authority(
          _number(row["mode"]), kinds.Value(row["decision_kind"])
      )
      with self.subTest(row=row):
        self.assertEqual(allowed, row["allowed"] == "true")

  def test_recovery_path_example_matches_policy(self):
    steps = [
        line.split()
        for line in _load_example("authority_recovery_path.txt").splitlines()
        if line and not line.startswith("#")
    ]
    self.assertEqual(len(steps), 4)
    for current, arrow, expected in steps:
      event = arrow.strip("-<>")
      result = assessor.assess_authority_transition(
          _number(current), _number(event)
      )
      self.assertTrue(result.ok, arrow)
      self.assertEqual(result.next_mode.value, _number(expected))


if __name__ == "__main__":
  unittest.main()
