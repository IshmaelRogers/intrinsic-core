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

"""Python serialization tests for MarineComponentValidity."""

import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.proto import marine_component_validity_pb2

# Canonical serialization of _fill_fixture(). Keep in sync with
# marine_component_validity_serialization_test.cc.
_GOLDEN_HEX = (
    "0a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d3732020801"
)
_WITHOUT_VALIDITY_HEX = (
    "0a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37"
)
_POLICY = marine_component_validity_policy


def _fill_fixture():
  message = marine_component_validity_pb2.MarineComponentValidity()
  message.source_id = "fusion_0"
  message.observation_time.seconds = 1700000000
  message.observation_time.nanos = 250000000
  message.validity_horizon.seconds = 10
  message.validity_horizon.nanos = 500000000
  message.confidence = 0.75
  message.uncertainty_reference = "cov-ref-7"
  message.validity.state = stamped_header_pb2.Validity.STATE_VALID
  return message


def _view_of(message, present):
  return _POLICY.MarineComponentValidityView(
      present=present,
      source_id=message.source_id,
      observation_time_present=message.HasField("observation_time"),
      observation_time=(
          message.observation_time.seconds,
          message.observation_time.nanos,
      ),
      validity_horizon_present=message.HasField("validity_horizon"),
      validity_horizon=(
          message.validity_horizon.seconds,
          message.validity_horizon.nanos,
      ),
      confidence_present=message.HasField("confidence"),
      confidence=message.confidence,
      uncertainty_reference=message.uncertainty_reference,
      validity_present=message.HasField("validity"),
      validity_state=message.validity.state,
  )


class MarineComponentValiditySerializationTest(unittest.TestCase):

  def test_golden_round_trip(self):
    message = _fill_fixture()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.source_id, "fusion_0")
    self.assertEqual(parsed.observation_time.seconds, 1700000000)
    self.assertEqual(parsed.observation_time.nanos, 250000000)
    self.assertEqual(parsed.validity_horizon.seconds, 10)
    self.assertEqual(parsed.validity_horizon.nanos, 500000000)
    self.assertTrue(parsed.HasField("confidence"))
    self.assertEqual(parsed.confidence, 0.75)
    self.assertEqual(parsed.uncertainty_reference, "cov-ref-7")
    self.assertTrue(parsed.HasField("validity"))
    self.assertEqual(
        parsed.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(parsed.SerializeToString(), golden)

  def test_absent_validity_is_omitted_and_distinct(self):
    message = _fill_fixture()
    message.ClearField("validity")
    without = bytes.fromhex(_WITHOUT_VALIDITY_HEX)
    self.assertEqual(message.SerializeToString(), without)
    self.assertTrue(bytes.fromhex(_GOLDEN_HEX).startswith(without))
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(without)
    self.assertFalse(parsed.HasField("validity"))
    self.assertEqual(
        parsed.validity.state, stamped_header_pb2.Validity.STATE_UNSPECIFIED
    )
    assessment = _POLICY.assess_marine_component_validity(
        _view_of(parsed, True), (1700000010, 750000000)
    )
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(assessment.freshness, _POLICY.ComponentFreshness.UNKNOWN)
    self.assertIs(
        stamped_header_policy.classify_validity(
            True, stamped_header_pb2.Validity.STATE_INVALID
        ),
        stamped_header_policy.ValidityKind.INVALID,
    )

  def test_default_message_is_empty(self):
    message = marine_component_validity_pb2.MarineComponentValidity()
    self.assertEqual(message.SerializeToString(), b"")
    self.assertFalse(message.HasField("observation_time"))
    self.assertFalse(message.HasField("validity_horizon"))
    self.assertFalse(message.HasField("confidence"))
    self.assertFalse(message.HasField("validity"))
    self.assertEqual(message.source_id, "")
    self.assertEqual(message.uncertainty_reference, "")
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    empty = _POLICY.assess_marine_component_validity(
        _view_of(parsed, False), (0, 0)
    )
    self.assertIs(empty.error, _POLICY.ComponentValidityError.NONE)
    self.assertFalse(empty.accepted)
    present = _POLICY.assess_marine_component_validity(
        _view_of(parsed, True), (0, 0)
    )
    self.assertIs(present.error, _POLICY.ComponentValidityError.SOURCE_ID)

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.source_id, "fusion_0")
    self.assertEqual(parsed.uncertainty_reference, "cov-ref-7")
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_unknown_validity_enum_round_trips(self):
    self.assertEqual(stamped_header_pb2.Validity.STATE_UNSPECIFIED, 0)
    self.assertEqual(stamped_header_pb2.Validity.STATE_VALID, 1)
    self.assertEqual(stamped_header_pb2.Validity.STATE_INVALID, 2)
    state_values = stamped_header_pb2.Validity.DESCRIPTOR.enum_values_by_name
    self.assertIsNone(state_values.get("STATE_EXPIRED"))
    self.assertIsNone(state_values.get("STATE_DEGRADED"))
    self.assertEqual(
        set(state_values),
        {
            "STATE_UNSPECIFIED",
            "STATE_VALID",
            "STATE_INVALID",
        },
    )
    raw = bytes.fromhex("32020863")
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(raw)
    self.assertTrue(parsed.HasField("validity"))
    self.assertEqual(parsed.validity.state, 99)
    self.assertEqual(parsed.SerializeToString(), raw)
    self.assertIs(
        stamped_header_policy.classify_validity(True, 99),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    nested_raw = bytes.fromhex("320408011805")
    nested = marine_component_validity_pb2.MarineComponentValidity()
    nested.ParseFromString(nested_raw)
    self.assertEqual(
        nested.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(nested.SerializeToString(), nested_raw)

  def test_confidence_zero_is_distinct_from_unset(self):
    unset = _fill_fixture()
    unset.ClearField("confidence")
    self.assertFalse(unset.HasField("confidence"))
    zero = marine_component_validity_pb2.MarineComponentValidity()
    zero.CopyFrom(unset)
    zero.confidence = 0.0
    self.assertTrue(zero.HasField("confidence"))
    self.assertEqual(zero.confidence, 0.0)
    self.assertNotEqual(unset.SerializeToString(), zero.SerializeToString())
    parsed = marine_component_validity_pb2.MarineComponentValidity()
    parsed.ParseFromString(zero.SerializeToString())
    self.assertTrue(parsed.HasField("confidence"))
    self.assertEqual(parsed.confidence, 0.0)

  def test_parsed_deadline_boundary(self):
    message = _fill_fixture()
    view = _view_of(message, True)
    deadline = (1700000010, 750000000)
    fresh = _POLICY.assess_marine_component_validity(view, deadline)
    self.assertIs(fresh.error, _POLICY.ComponentValidityError.NONE)
    self.assertIs(fresh.freshness, _POLICY.ComponentFreshness.FRESH)
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_marine_component_validity(
        view, (deadline[0], deadline[1] + 1)
    )
    self.assertIs(expired.freshness, _POLICY.ComponentFreshness.EXPIRED)
    self.assertIs(expired.validity, stamped_header_policy.ValidityKind.VALID)
    self.assertFalse(expired.accepted)


if __name__ == "__main__":
  unittest.main()
