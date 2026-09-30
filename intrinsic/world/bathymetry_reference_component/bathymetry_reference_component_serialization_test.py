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

import math
import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.world.bathymetry_reference_component import bathymetry_reference_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy
from intrinsic.world.proto import bathymetry_reference_component_pb2

# Canonical serialization of testdata/referenced_bathymetry.textproto.
# Keep in sync with bathymetry_reference_component_serialization_test.cc.
_GOLDEN_HEX = (
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a106361733a62617468792d677269642d312100000000000029c0"
)
_EXAMPLE = "referenced_bathymetry.textproto"
_POLICY = bathymetry_reference_component_policy
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
  message = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
  text_format.Parse(_load_text(), message)
  return message


def _view_of(message, present):
  meta = message.validity_meta
  return _POLICY.BathymetryReferenceView(
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
      bathymetry_asset_ref=message.bathymetry_asset_ref,
      reference_z_present=message.HasField("reference_z_m"),
      reference_z_m=message.reference_z_m,
  )


class BathymetryReferenceSerializationTest(unittest.TestCase):

  def test_textproto_matches_golden_and_validates(self):
    message = _parse_example()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    self.assertEqual(message.frame_id, "world_enu")
    self.assertEqual(message.bathymetry_asset_ref, "cas:bathy-grid-1")
    self.assertTrue(message.HasField("reference_z_m"))
    self.assertEqual(message.reference_z_m, -12.5)
    self.assertEqual(
        message.validity_meta.validity.state,
        stamped_header_pb2.Validity.STATE_VALID,
    )
    parsed = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.SerializeToString(), golden)
    fresh = _POLICY.assess_bathymetry_reference(
        _view_of(parsed, True), _DEADLINE
    )
    self.assertIs(fresh.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(fresh.validity.freshness, _MARINE.ComponentFreshness.FRESH)
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_bathymetry_reference(
        _view_of(parsed, True), (_DEADLINE[0], _DEADLINE[1] + 1)
    )
    self.assertIs(expired.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        expired.validity.freshness, _MARINE.ComponentFreshness.EXPIRED
    )
    self.assertIs(
        expired.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertFalse(expired.accepted)

  def test_default_message_is_empty(self):
    message = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
    self.assertEqual(message.SerializeToString(), b"")
    self.assertFalse(message.HasField("validity_meta"))
    self.assertFalse(message.HasField("reference_z_m"))
    self.assertEqual(message.frame_id, "")
    self.assertEqual(message.bathymetry_asset_ref, "")
    parsed = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    empty = _POLICY.assess_bathymetry_reference(
        _view_of(parsed, False), _DEADLINE
    )
    self.assertIs(empty.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertFalse(empty.accepted)

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.bathymetry_asset_ref, "cas:bathy-grid-1")
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_reference_z_zero_is_distinct_from_unset(self):
    zero = _parse_example()
    zero.reference_z_m = 0.0
    unset = _parse_example()
    unset.ClearField("reference_z_m")
    self.assertTrue(zero.HasField("reference_z_m"))
    self.assertFalse(unset.HasField("reference_z_m"))
    self.assertNotEqual(zero.SerializeToString(), unset.SerializeToString())
    self.assertTrue(
        _POLICY.assess_bathymetry_reference(
            _view_of(unset, True), _DEADLINE
        ).accepted
    )
    self.assertTrue(
        _POLICY.assess_bathymetry_reference(
            _view_of(zero, True), _DEADLINE
        ).accepted
    )

  def test_rejects_empty_frame_asset_and_nan_z(self):
    bad = _parse_example()
    bad.frame_id = ""
    self.assertIs(
        _POLICY.assess_bathymetry_reference(
            _view_of(bad, True), _DEADLINE
        ).error,
        _POLICY.BathymetryReferenceError.FRAME_ID,
    )
    bad = _parse_example()
    bad.bathymetry_asset_ref = ""
    self.assertIs(
        _POLICY.assess_bathymetry_reference(
            _view_of(bad, True), _DEADLINE
        ).error,
        _POLICY.BathymetryReferenceError.ASSET_REF,
    )
    bad = _parse_example()
    bad.reference_z_m = math.nan
    self.assertIs(
        _POLICY.assess_bathymetry_reference(
            _view_of(bad, True), _DEADLINE
        ).error,
        _POLICY.BathymetryReferenceError.REFERENCE_Z,
    )

  def test_unknown_validity_enum_round_trips(self):
    message = _parse_example()
    message.validity_meta.validity.state = 99
    raw = message.SerializeToString()
    parsed = bathymetry_reference_component_pb2.BathymetryReferenceComponent()
    parsed.ParseFromString(raw)
    self.assertEqual(parsed.validity_meta.validity.state, 99)
    self.assertEqual(parsed.SerializeToString(), raw)
    assessment = _POLICY.assess_bathymetry_reference(
        _view_of(parsed, True), _DEADLINE
    )
    self.assertIs(
        assessment.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertFalse(assessment.accepted)


if __name__ == "__main__":
  unittest.main()
