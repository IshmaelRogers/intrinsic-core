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

"""Python serialization tests for CurrentFieldComponent."""

import math
import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.world.current_field_component import current_field_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.proto import current_field_component_pb2

# Canonical serialization of testdata/constant_current.textproto.
# Keep in sync with current_field_component_serialization_test.cc.
_GOLDEN_HEX = (
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a1d0a1b09000000000000f83f11000000000000d0bf19000000000000c03f"
)
_EXAMPLE = "constant_current.textproto"
_POLICY = current_field_component_policy
_MARINE = marine_component_validity_policy
# 1700000000.250s + 10.500s.
_DEADLINE = (1700000010, 750000000)


def _load_text():
  here = os.path.dirname(os.path.abspath(__file__))
  path = os.path.join(here, "testdata", _EXAMPLE)
  if os.path.exists(path):
    with open(path, encoding="utf-8") as handle:
      return handle.read()
  root = os.environ.get("TEST_SRCDIR", "")
  for dirpath, _, filenames in os.walk(root):
    if _EXAMPLE in filenames and os.path.basename(dirpath) == "testdata":
      with open(os.path.join(dirpath, _EXAMPLE), encoding="utf-8") as handle:
        return handle.read()
  raise AssertionError("missing %s" % _EXAMPLE)


def _parse_example():
  message = current_field_component_pb2.CurrentFieldComponent()
  text_format.Parse(_load_text(), message)
  return message


def _view_of(message, present):
  meta = message.validity_meta
  velocity = message.constant.velocity_m_s
  return _POLICY.CurrentFieldView(
      present=present,
      validity_meta=_MARINE.MarineComponentValidityView(
          present=message.HasField("validity_meta"),
          source_id=meta.source_id,
          observation_time_present=meta.HasField("observation_time"),
          observation_time=(
              meta.observation_time.seconds,
              meta.observation_time.nanos,
          ),
          validity_horizon_present=meta.HasField("validity_horizon"),
          validity_horizon=(
              meta.validity_horizon.seconds,
              meta.validity_horizon.nanos,
          ),
          confidence_present=meta.HasField("confidence"),
          confidence=meta.confidence,
          uncertainty_reference=meta.uncertainty_reference,
          validity_present=meta.HasField("validity"),
          validity_state=meta.validity.state,
      ),
      frame_id=message.frame_id,
      constant_present=message.HasField("constant"),
      velocity_x_m_s=velocity.x,
      velocity_y_m_s=velocity.y,
      velocity_z_m_s=velocity.z,
  )


class CurrentFieldSerializationTest(unittest.TestCase):

  def test_textproto_matches_golden_and_validates(self):
    message = _parse_example()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    self.assertEqual(message.frame_id, "world_enu")
    self.assertTrue(message.HasField("constant"))
    self.assertEqual(message.constant.velocity_m_s.x, 1.5)
    self.assertEqual(message.constant.velocity_m_s.y, -0.25)
    self.assertEqual(message.constant.velocity_m_s.z, 0.125)
    self.assertEqual(
        message.validity_meta.validity.state,
        stamped_header_pb2.Validity.STATE_VALID,
    )
    parsed = current_field_component_pb2.CurrentFieldComponent()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.SerializeToString(), golden)
    fresh = _POLICY.assess_current_field(_view_of(parsed, True), _DEADLINE)
    self.assertIs(fresh.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(fresh.validity.freshness, _MARINE.ComponentFreshness.FRESH)
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_current_field(
        _view_of(parsed, True), (_DEADLINE[0], _DEADLINE[1] + 1)
    )
    self.assertIs(expired.error, _POLICY.CurrentFieldError.NONE)
    self.assertIs(
        expired.validity.freshness, _MARINE.ComponentFreshness.EXPIRED
    )
    self.assertIs(
        expired.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertFalse(expired.accepted)

  def test_default_message_is_empty(self):
    message = current_field_component_pb2.CurrentFieldComponent()
    self.assertEqual(message.SerializeToString(), b"")
    self.assertFalse(message.HasField("validity_meta"))
    self.assertFalse(message.HasField("constant"))
    self.assertEqual(message.frame_id, "")
    parsed = current_field_component_pb2.CurrentFieldComponent()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    empty = _POLICY.assess_current_field(_view_of(parsed, False), _DEADLINE)
    self.assertIs(empty.error, _POLICY.CurrentFieldError.NONE)
    self.assertFalse(empty.accepted)

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = current_field_component_pb2.CurrentFieldComponent()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertTrue(parsed.HasField("constant"))
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_explicit_zero_current_is_present(self):
    zeros = _parse_example()
    zeros.ClearField("constant")
    zeros.constant.SetInParent()
    self.assertTrue(zeros.HasField("constant"))
    self.assertEqual(zeros.constant.velocity_m_s.x, 0)
    self.assertEqual(zeros.constant.velocity_m_s.y, 0)
    self.assertEqual(zeros.constant.velocity_m_s.z, 0)
    missing = _parse_example()
    missing.ClearField("constant")
    self.assertNotEqual(zeros.SerializeToString(), missing.SerializeToString())
    self.assertTrue(
        _POLICY.assess_current_field(_view_of(zeros, True), _DEADLINE).accepted
    )
    self.assertIs(
        _POLICY.assess_current_field(_view_of(missing, True), _DEADLINE).error,
        _POLICY.CurrentFieldError.REPRESENTATION,
    )

  def test_omitted_axes_are_zero(self):
    message = _parse_example()
    message.ClearField("constant")
    message.constant.velocity_m_s.x = 1.0
    self.assertEqual(message.constant.velocity_m_s.y, 0)
    self.assertEqual(message.constant.velocity_m_s.z, 0)
    self.assertTrue(
        _POLICY.assess_current_field(
            _view_of(message, True), _DEADLINE
        ).accepted
    )

  def test_rejects_empty_frame_and_non_finite_velocity(self):
    bad = _parse_example()
    bad.frame_id = ""
    self.assertIs(
        _POLICY.assess_current_field(_view_of(bad, True), _DEADLINE).error,
        _POLICY.CurrentFieldError.FRAME_ID,
    )
    bad = _parse_example()
    bad.constant.velocity_m_s.y = math.nan
    self.assertIs(
        _POLICY.assess_current_field(_view_of(bad, True), _DEADLINE).error,
        _POLICY.CurrentFieldError.VELOCITY,
    )

  def test_unknown_validity_enum_round_trips(self):
    message = _parse_example()
    message.validity_meta.validity.state = 99
    raw = message.SerializeToString()
    parsed = current_field_component_pb2.CurrentFieldComponent()
    parsed.ParseFromString(raw)
    self.assertEqual(parsed.validity_meta.validity.state, 99)
    self.assertEqual(parsed.SerializeToString(), raw)
    assessment = _POLICY.assess_current_field(_view_of(parsed, True), _DEADLINE)
    self.assertIs(
        assessment.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertFalse(assessment.accepted)


if __name__ == "__main__":
  unittest.main()
