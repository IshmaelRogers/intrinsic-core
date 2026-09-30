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

"""Tests for the authority transition table and decision matrix."""

import unittest

from intrinsic.safety import authority_transition_policy as policy
from intrinsic.safety import safety_decision_policy

M = policy.AuthorityMode
E = policy.AuthorityEvent
K = safety_decision_policy.SafetyDecisionKind
ERR = policy.AuthorityTransitionError

_LIVE_MODES = (M.SHADOW, M.RECOMMEND, M.CONSTRAINED, M.REVOKED, M.EMERGENCY)
_LIVE_EVENTS = (
    E.ARM_RECOMMEND,
    E.ENABLE_CONSTRAINED,
    E.REVOKE,
    E.ENTER_EMERGENCY,
    E.CLEAR_EMERGENCY,
    E.RESET_TO_SHADOW,
    E.FAULT,
)

# Expected target per (from, event). None marks a forbidden cell. Written out
# independently of the implementation so every cell is asserted.
_EXPECTED = {
    M.SHADOW: {
        E.ARM_RECOMMEND: M.RECOMMEND,
        E.ENABLE_CONSTRAINED: None,
        E.REVOKE: None,
        E.ENTER_EMERGENCY: M.EMERGENCY,
        E.CLEAR_EMERGENCY: None,
        E.RESET_TO_SHADOW: None,
        E.FAULT: M.REVOKED,
    },
    M.RECOMMEND: {
        E.ARM_RECOMMEND: None,
        E.ENABLE_CONSTRAINED: M.CONSTRAINED,
        E.REVOKE: M.REVOKED,
        E.ENTER_EMERGENCY: M.EMERGENCY,
        E.CLEAR_EMERGENCY: None,
        E.RESET_TO_SHADOW: M.SHADOW,
        E.FAULT: M.REVOKED,
    },
    M.CONSTRAINED: {
        E.ARM_RECOMMEND: None,
        E.ENABLE_CONSTRAINED: None,
        E.REVOKE: M.REVOKED,
        E.ENTER_EMERGENCY: M.EMERGENCY,
        E.CLEAR_EMERGENCY: None,
        E.RESET_TO_SHADOW: M.SHADOW,
        E.FAULT: M.REVOKED,
    },
    M.REVOKED: {
        E.ARM_RECOMMEND: None,
        E.ENABLE_CONSTRAINED: None,
        E.REVOKE: None,
        E.ENTER_EMERGENCY: M.EMERGENCY,
        E.CLEAR_EMERGENCY: None,
        E.RESET_TO_SHADOW: M.SHADOW,
        E.FAULT: None,
    },
    M.EMERGENCY: {
        E.ARM_RECOMMEND: None,
        E.ENABLE_CONSTRAINED: None,
        E.REVOKE: None,
        E.ENTER_EMERGENCY: None,
        E.CLEAR_EMERGENCY: M.REVOKED,
        E.RESET_TO_SHADOW: None,
        E.FAULT: None,
    },
}

_DECISION_KINDS = (K.ACCEPT, K.PROJECT, K.REJECT, K.ABORT, K.SURFACE)

# Rows follow _LIVE_MODES. Columns follow _DECISION_KINDS.
_DECISION_ALLOWED = {
    M.SHADOW: (False, False, True, True, False),
    M.RECOMMEND: (False, False, True, True, False),
    M.CONSTRAINED: (True, True, True, True, True),
    M.REVOKED: (False, False, True, True, True),
    M.EMERGENCY: (False, False, True, True, True),
}


class AuthorityTransitionPolicyTest(unittest.TestCase):

  def test_initial_mode_is_shadow(self):
    self.assertEqual(policy.initial_authority_mode(), M.SHADOW)
    self.assertFalse(
        policy.decision_allowed_under_authority(
            policy.initial_authority_mode(), K.ACCEPT
        )
    )

  def test_classify_wire_numbers(self):
    for number, mode in enumerate(_LIVE_MODES, start=1):
      self.assertEqual(policy.classify_authority_mode(number), mode)
    self.assertEqual(policy.classify_authority_mode(0), M.UNSPECIFIED)
    for number in (6, 7, 99, -1):
      self.assertEqual(policy.classify_authority_mode(number), M.UNKNOWN)
    for number, event in enumerate(_LIVE_EVENTS, start=1):
      self.assertEqual(policy.classify_authority_event(number), event)
    self.assertEqual(policy.classify_authority_event(0), E.UNSPECIFIED)
    for number in (8, 9, 99, -1):
      self.assertEqual(policy.classify_authority_event(number), E.UNKNOWN)

  def test_live_predicates(self):
    for mode in _LIVE_MODES:
      self.assertTrue(policy.is_live_authority_mode(mode))
    self.assertFalse(policy.is_live_authority_mode(M.UNSPECIFIED))
    self.assertFalse(policy.is_live_authority_mode(M.UNKNOWN))
    for event in _LIVE_EVENTS:
      self.assertTrue(policy.is_live_authority_event(event))
    self.assertFalse(policy.is_live_authority_event(E.UNSPECIFIED))
    self.assertFalse(policy.is_live_authority_event(E.UNKNOWN))

  def test_expected_table_is_complete(self):
    self.assertEqual(set(_EXPECTED), set(_LIVE_MODES))
    for mode in _LIVE_MODES:
      self.assertEqual(set(_EXPECTED[mode]), set(_LIVE_EVENTS))

  def test_exhaustive_from_by_event_matrix(self):
    allowed = 0
    forbidden = 0
    for mode in _LIVE_MODES:
      for event in _LIVE_EVENTS:
        with self.subTest(mode=mode.name, event=event.name):
          expected = _EXPECTED[mode][event]
          result = policy.apply_authority_transition(mode, event)
          if expected is None:
            forbidden += 1
            self.assertFalse(result.ok)
            self.assertEqual(result.next_mode, mode)
            self.assertEqual(result.error, ERR.FORBIDDEN)
          else:
            allowed += 1
            self.assertTrue(result.ok)
            self.assertEqual(result.next_mode, expected)
            self.assertEqual(result.error, ERR.NONE)
    self.assertEqual(allowed, 15)
    self.assertEqual(forbidden, 20)

  def test_reset_from_shadow_is_forbidden_not_no_op(self):
    result = policy.apply_authority_transition(M.SHADOW, E.RESET_TO_SHADOW)
    self.assertFalse(result.ok)
    self.assertEqual(result.next_mode, M.SHADOW)
    self.assertEqual(result.error, ERR.FORBIDDEN)

  def test_emergency_clears_only_to_revoked(self):
    for event in _LIVE_EVENTS:
      result = policy.apply_authority_transition(M.EMERGENCY, event)
      if event == E.CLEAR_EMERGENCY:
        self.assertTrue(result.ok)
        self.assertEqual(result.next_mode, M.REVOKED)
      else:
        self.assertFalse(result.ok)
        self.assertEqual(result.next_mode, M.EMERGENCY)

  def test_revoked_regains_authority_only_via_shadow(self):
    for event in _LIVE_EVENTS:
      result = policy.apply_authority_transition(M.REVOKED, event)
      if result.ok:
        self.assertIn(result.next_mode, (M.SHADOW, M.EMERGENCY))

  def test_invalid_current_fails_closed_for_every_event(self):
    events = _LIVE_EVENTS + (E.UNSPECIFIED, E.UNKNOWN)
    for mode in (M.UNSPECIFIED, M.UNKNOWN):
      for event in events:
        with self.subTest(mode=mode.name, event=event.name):
          result = policy.apply_authority_transition(mode, event)
          self.assertFalse(result.ok)
          self.assertEqual(result.next_mode, M.UNSPECIFIED)
          self.assertEqual(result.error, ERR.INVALID_MODE)

  def test_invalid_event_fails_closed_for_every_mode(self):
    for mode in _LIVE_MODES:
      for event in (E.UNSPECIFIED, E.UNKNOWN):
        with self.subTest(mode=mode.name, event=event.name):
          result = policy.apply_authority_transition(mode, event)
          self.assertFalse(result.ok)
          self.assertEqual(result.next_mode, mode)
          self.assertEqual(result.error, ERR.INVALID_EVENT)

  def test_unknown_wire_numbers_fail_closed(self):
    for number in (6, 7, 99, -1):
      result = policy.apply_authority_transition(
          policy.classify_authority_mode(number), E.ARM_RECOMMEND
      )
      self.assertFalse(result.ok)
      self.assertEqual(result.next_mode, M.UNSPECIFIED)
    for number in (8, 9, 99, -1):
      result = policy.apply_authority_transition(
          M.SHADOW, policy.classify_authority_event(number)
      )
      self.assertFalse(result.ok)
      self.assertEqual(result.next_mode, M.SHADOW)

  def test_non_enum_inputs_fail_closed(self):
    for bad_mode, bad_event in ((1, E.ARM_RECOMMEND), (None, E.FAULT)):
      result = policy.apply_authority_transition(bad_mode, bad_event)
      self.assertFalse(result.ok)
      self.assertEqual(result.error, ERR.INVALID_MODE)
    for bad_event in (1, None, "ARM_RECOMMEND"):
      result = policy.apply_authority_transition(M.SHADOW, bad_event)
      self.assertFalse(result.ok)
      self.assertEqual(result.next_mode, M.SHADOW)
      self.assertEqual(result.error, ERR.INVALID_EVENT)

  def test_default_result_is_not_ok(self):
    result = policy.AuthorityTransitionResult()
    self.assertFalse(result.ok)
    self.assertEqual(result.next_mode, M.UNSPECIFIED)
    self.assertNotEqual(result.error, ERR.NONE)

  def test_emergency_to_constrained_happy_path(self):
    mode = M.EMERGENCY
    steps = (
        (E.CLEAR_EMERGENCY, M.REVOKED),
        (E.RESET_TO_SHADOW, M.SHADOW),
        (E.ARM_RECOMMEND, M.RECOMMEND),
        (E.ENABLE_CONSTRAINED, M.CONSTRAINED),
    )
    for event, expected in steps:
      result = policy.apply_authority_transition(mode, event)
      self.assertTrue(result.ok, (mode, event))
      self.assertEqual(result.next_mode, expected)
      mode = result.next_mode
    self.assertEqual(mode, M.CONSTRAINED)

  def test_cannot_skip_rearm_after_emergency_or_revoke(self):
    skips = (
        (M.EMERGENCY, E.ENABLE_CONSTRAINED),
        (M.EMERGENCY, E.ARM_RECOMMEND),
        (M.REVOKED, E.ARM_RECOMMEND),
        (M.REVOKED, E.ENABLE_CONSTRAINED),
        (M.SHADOW, E.ENABLE_CONSTRAINED),
    )
    for mode, event in skips:
      self.assertFalse(policy.apply_authority_transition(mode, event).ok)

  def test_decision_allowed_matrix(self):
    for mode in _LIVE_MODES:
      for kind, expected in zip(_DECISION_KINDS, _DECISION_ALLOWED[mode]):
        with self.subTest(mode=mode.name, kind=kind.name):
          self.assertEqual(
              policy.decision_allowed_under_authority(mode, kind), expected
          )

  def test_decision_allowed_unknown_is_false(self):
    kinds = _DECISION_KINDS + (K.UNSPECIFIED, K.UNKNOWN)
    for mode in (M.UNSPECIFIED, M.UNKNOWN):
      for kind in kinds:
        self.assertFalse(policy.decision_allowed_under_authority(mode, kind))
    for mode in _LIVE_MODES:
      self.assertFalse(
          policy.decision_allowed_under_authority(mode, K.UNSPECIFIED)
      )
      self.assertFalse(policy.decision_allowed_under_authority(mode, K.UNKNOWN))

  def test_decision_matrix_uses_classified_safety_decision_kinds(self):
    kind = safety_decision_policy.classify_safety_decision_kind(5)
    self.assertTrue(policy.decision_allowed_under_authority(M.EMERGENCY, kind))
    unknown = safety_decision_policy.classify_safety_decision_kind(99)
    self.assertFalse(
        policy.decision_allowed_under_authority(M.CONSTRAINED, unknown)
    )


if __name__ == "__main__":
  unittest.main()
