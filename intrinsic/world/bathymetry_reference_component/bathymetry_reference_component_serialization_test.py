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

"""Python serialization tests for BathymetryReferenceComponent."""

from dataclasses import dataclass
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.world.bathymetry_reference_component import bathymetry_reference_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.proto import bathymetry_reference_component_pb2

# Canonical serialization of _fill_fixture(). Keep in sync with
# bathymetry_reference_component_serialization_test.cc.
_GOLDEN_HEX = (
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a0b62617468792d7265662d3721000000000000f83f"
)
_WITHOUT_BIAS_HEX = (
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a0b62617468792d7265662d37"
)
_POLICY = bathymetry_reference_component_policy
_VALIDITY = marine_component_validity_policy
_PB2 = bathymetry_reference_component_pb2


@dataclass
class _BathymetryPlain:
  """Test-only plain value. Unknown fields are not copied."""

  present: bool = False
  validity_message_present: bool = False
  source_id: str = ""
  observation_time_present: bool = False
  observation_time: tuple[int, int] = (0, 0)
  validity_horizon_present: bool = False
  validity_horizon: tuple[int, int] = (0, 0)
  confidence_present: bool = False
  confidence: float = 0.0
  uncertainty_reference: str = ""
  embodiment_validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  asset_reference: str = ""
  vertical_bias_present: bool = False
  vertical_bias_m: float = 0.0


def _fill_fixture():
  message = _PB2.BathymetryReferenceComponent()
  message.validity.source_id = "fusion_0"
  message.validity.observation_time.seconds = 1700000000
  message.validity.observation_time.nanos = 250000000
  message.validity.validity_horizon.seconds = 10
  message.validity.validity_horizon.nanos = 500000000
  message.validity.confidence = 0.75
  message.validity.uncertainty_reference = "cov-ref-7"
  message.validity.validity.state = stamped_header_pb2.Validity.STATE_VALID
  message.frame_id = "world_enu"
  message.asset_reference = "bathy-ref-7"
  message.vertical_bias_m = 1.5
  return message


def _plain_from_proto(message, present):
  validity = message.validity
  return _BathymetryPlain(
      present=present,
      validity_message_present=message.HasField("validity"),
      source_id=validity.source_id,
      observation_time_present=validity.HasField("observation_time"),
      observation_time=(
          validity.observation_time.seconds,
          validity.observation_time.nanos,
      ),
      validity_horizon_present=validity.HasField("validity_horizon"),
      validity_horizon=(
          validity.validity_horizon.seconds,
          validity.validity_horizon.nanos,
      ),
      confidence_present=validity.HasField("confidence"),
      confidence=validity.confidence,
      uncertainty_reference=validity.uncertainty_reference,
      embodiment_validity_present=validity.HasField("validity"),
      validity_state=validity.validity.state,
      frame_id=message.frame_id,
      asset_reference=message.asset_reference,
      vertical_bias_present=message.HasField("vertical_bias_m"),
      vertical_bias_m=message.vertical_bias_m,
  )


def _apply_plain(plain, message):
  message.Clear()
  if plain.validity_message_present:
    validity = message.validity
    validity.source_id = plain.source_id
    if plain.observation_time_present:
      validity.observation_time.seconds = plain.observation_time[0]
      validity.observation_time.nanos = plain.observation_time[1]
    if plain.validity_horizon_present:
      validity.validity_horizon.seconds = plain.validity_horizon[0]
      validity.validity_horizon.nanos = plain.validity_horizon[1]
    if plain.confidence_present:
      validity.confidence = plain.confidence
    validity.uncertainty_reference = plain.uncertainty_reference
    if plain.embodiment_validity_present:
      validity.validity.state = plain.validity_state
  message.frame_id = plain.frame_id
  message.asset_reference = plain.asset_reference
  if plain.vertical_bias_present:
    message.vertical_bias_m = plain.vertical_bias_m


def _view_from_plain(plain):
  return _POLICY.BathymetryReferenceView(
      present=plain.present,
      validity=_VALIDITY.MarineComponentValidityView(
          present=plain.validity_message_present,
          source_id=plain.source_id,
          observation_time_present=plain.observation_time_present,
          observation_time=plain.observation_time,
          validity_horizon_present=plain.validity_horizon_present,
          validity_horizon=plain.validity_horizon,
          confidence_present=plain.confidence_present,
          confidence=plain.confidence,
          uncertainty_reference=plain.uncertainty_reference,
          validity_present=plain.embodiment_validity_present,
          validity_state=plain.validity_state,
      ),
      frame_id=plain.frame_id,
      asset_reference=plain.asset_reference,
      vertical_bias_present=plain.vertical_bias_present,
      vertical_bias_m=plain.vertical_bias_m,
  )


class BathymetryReferenceSerializationTest(unittest.TestCase):

  def test_schema_has_no_inline_grid(self):
    descriptor = _PB2.BathymetryReferenceComponent.DESCRIPTOR
    self.assertEqual(
        [field.name for field in descriptor.fields],
        ["validity", "frame_id", "asset_reference", "vertical_bias_m"],
    )
    self.assertEqual(
        [field.number for field in descriptor.fields], [1, 2, 3, 4]
    )
    self.assertIsNone(descriptor.fields_by_name.get("samples"))
    self.assertIsNone(descriptor.fields_by_name.get("heights_m"))

  def test_golden_round_trip(self):
    message = _fill_fixture()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = _PB2.BathymetryReferenceComponent()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.validity.source_id, "fusion_0")
    self.assertEqual(parsed.validity.observation_time.seconds, 1700000000)
    self.assertEqual(parsed.validity.observation_time.nanos, 250000000)
    self.assertEqual(parsed.validity.validity_horizon.seconds, 10)
    self.assertEqual(parsed.validity.validity_horizon.nanos, 500000000)
    self.assertEqual(parsed.validity.confidence, 0.75)
    self.assertEqual(parsed.validity.uncertainty_reference, "cov-ref-7")
    self.assertEqual(
        parsed.validity.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.asset_reference, "bathy-ref-7")
    self.assertTrue(parsed.HasField("vertical_bias_m"))
    self.assertEqual(parsed.vertical_bias_m, 1.5)
    self.assertEqual(parsed.SerializeToString(), golden)
    plain = _plain_from_proto(parsed, True)
    again = _PB2.BathymetryReferenceComponent()
    _apply_plain(plain, again)
    self.assertEqual(again.SerializeToString(), golden)
    assessment = _POLICY.assess_bathymetry_reference(
        _view_from_plain(plain), (1700000010, 750000000)
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertTrue(assessment.accepted)

  def test_bias_zero_is_distinct_from_unset(self):
    unset = _fill_fixture()
    unset.ClearField("vertical_bias_m")
    without = bytes.fromhex(_WITHOUT_BIAS_HEX)
    self.assertEqual(unset.SerializeToString(), without)
    self.assertTrue(bytes.fromhex(_GOLDEN_HEX).startswith(without))
    self.assertFalse(unset.HasField("vertical_bias_m"))
    zero = _PB2.BathymetryReferenceComponent()
    zero.CopyFrom(unset)
    zero.vertical_bias_m = 0.0
    self.assertTrue(zero.HasField("vertical_bias_m"))
    self.assertNotEqual(unset.SerializeToString(), zero.SerializeToString())
    plain = _plain_from_proto(zero, True)
    self.assertTrue(plain.vertical_bias_present)
    self.assertEqual(plain.vertical_bias_m, 0.0)
    assessment = _POLICY.assess_bathymetry_reference(
        _view_from_plain(plain), (1700000010, 750000000)
    )
    self.assertTrue(assessment.accepted)

  def test_default_message_is_empty(self):
    message = _PB2.BathymetryReferenceComponent()
    self.assertEqual(message.SerializeToString(), b"")
    self.assertFalse(message.HasField("validity"))
    self.assertFalse(message.HasField("vertical_bias_m"))
    self.assertEqual(message.frame_id, "")
    self.assertEqual(message.asset_reference, "")
    parsed = _PB2.BathymetryReferenceComponent()
    parsed.ParseFromString(b"")
    plain = _plain_from_proto(parsed, False)
    again = _PB2.BathymetryReferenceComponent()
    _apply_plain(plain, again)
    self.assertEqual(again.SerializeToString(), b"")
    empty = _POLICY.assess_bathymetry_reference(_view_from_plain(plain), (0, 0))
    self.assertIs(empty.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertFalse(empty.accepted)
    present_plain = _BathymetryPlain(**{**plain.__dict__, "present": True})
    present = _POLICY.assess_bathymetry_reference(
        _view_from_plain(present_plain), (0, 0)
    )
    self.assertIs(present.error, _POLICY.BathymetryReferenceError.VALIDITY)

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = _PB2.BathymetryReferenceComponent()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.asset_reference, "bathy-ref-7")
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_unknown_validity_enum_round_trips(self):
    raw = bytes.fromhex("0a0432020863")
    parsed = _PB2.BathymetryReferenceComponent()
    parsed.ParseFromString(raw)
    self.assertTrue(parsed.HasField("validity"))
    self.assertTrue(parsed.validity.HasField("validity"))
    self.assertEqual(parsed.validity.validity.state, 99)
    self.assertEqual(parsed.SerializeToString(), raw)
    plain = _plain_from_proto(parsed, True)
    self.assertIs(
        stamped_header_policy.classify_validity(True, plain.validity_state),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    nested_raw = bytes.fromhex("0a06320408011805")
    nested = _PB2.BathymetryReferenceComponent()
    nested.ParseFromString(nested_raw)
    self.assertEqual(
        nested.validity.validity.state, stamped_header_pb2.Validity.STATE_VALID
    )
    self.assertEqual(nested.SerializeToString(), nested_raw)


if __name__ == "__main__":
  unittest.main()
