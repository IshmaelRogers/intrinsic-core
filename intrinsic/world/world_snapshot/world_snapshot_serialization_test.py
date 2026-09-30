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

"""Serialization tests for WorldSnapshotDescriptor."""

import unittest

from intrinsic.world.proto import world_snapshot_descriptor_pb2

_PB2 = world_snapshot_descriptor_pb2

# Canonical serialization of _fill_fixture(). Keep in sync with
# world_snapshot_serialization_test.cc.
_GOLDEN_HEX = (
    "0a40623433313937626235393035383435383332383864653235343832323636346165"
    "37303730396430623931303336393864323936623262633765663330663765120b0880"
    "e2cfaa061080e59a77182a22180a1462617468796d657472795f7265666572656e6365"
    "100322110a0d63757272656e745f6669656c641007"
)
_EPOCH_ONLY_HEX = (
    "0a40366333646430393434653934326266613735313136336535623534326132333063"
    "34393963663835656364306636626132646635303161646661653034623864120b0880"
    "e2cfaa061080e59a77"
)
_ZERO_REVISION_HEX = (
    "0a40623533613735313633656662386564646638373834373833666366643965346161"
    "6536343338623261386563386462393566313234386463626564343632616412020801"
    "1805220f0a0d63757272656e745f6669656c64"
)
_GOLDEN_ID = "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e"


def _fill_fixture():
  message = _PB2.WorldSnapshotDescriptor()
  message.snapshot_id = _GOLDEN_ID
  message.creation_time.seconds = 1700000000
  message.creation_time.nanos = 250000000
  message.state_epoch = 42
  bathymetry = message.components.add()
  bathymetry.component_kind = "bathymetry_reference"
  bathymetry.revision = 3
  current = message.components.add()
  current.component_kind = "current_field"
  current.revision = 7
  return message


class WorldSnapshotSerializationTest(unittest.TestCase):

  def test_default_message_serializes_to_zero_bytes(self):
    self.assertEqual(_PB2.WorldSnapshotDescriptor().SerializeToString(), b"")
    self.assertEqual(_PB2.WorldComponentRevision().SerializeToString(), b"")

  def test_schema_is_locked(self):
    descriptor = _PB2.WorldSnapshotDescriptor.DESCRIPTOR
    self.assertEqual(
        [(f.number, f.name) for f in descriptor.fields],
        [
            (1, "snapshot_id"),
            (2, "creation_time"),
            (3, "state_epoch"),
            (4, "components"),
        ],
    )
    revision = _PB2.WorldComponentRevision.DESCRIPTOR
    self.assertEqual(
        [(f.number, f.name) for f in revision.fields],
        [(1, "component_kind"), (2, "revision")],
    )

  def test_schema_has_no_skew_or_payload_fields(self):
    descriptor = _PB2.WorldSnapshotDescriptor.DESCRIPTOR
    for name in (
        "skew",
        "age",
        "max_age",
        "stale",
        "status",
        "partial",
        "payload",
        "freshness",
    ):
      self.assertNotIn(name, descriptor.fields_by_name)
    revision = _PB2.WorldComponentRevision.DESCRIPTOR
    for name in ("payload", "component", "data", "validity"):
      self.assertNotIn(name, revision.fields_by_name)

  def test_golden_round_trip(self):
    message = _fill_fixture()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = _PB2.WorldSnapshotDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.snapshot_id, _GOLDEN_ID)
    self.assertEqual(parsed.creation_time.seconds, 1700000000)
    self.assertEqual(parsed.creation_time.nanos, 250000000)
    self.assertEqual(parsed.state_epoch, 42)
    self.assertEqual(
        [(c.component_kind, c.revision) for c in parsed.components],
        [("bathymetry_reference", 3), ("current_field", 7)],
    )
    self.assertEqual(parsed.SerializeToString(), golden)

  def test_epoch_only_golden_round_trip(self):
    message = _PB2.WorldSnapshotDescriptor()
    message.snapshot_id = (
        "6c3dd0944e942bfa751163e5b542a230c499cf85ecd0f6ba2df501adfae04b8d"
    )
    message.creation_time.seconds = 1700000000
    message.creation_time.nanos = 250000000
    golden = bytes.fromhex(_EPOCH_ONLY_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = _PB2.WorldSnapshotDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.state_epoch, 0)
    self.assertEqual(len(parsed.components), 0)

  def test_zero_revision_kind_stays_present_on_the_wire(self):
    message = _PB2.WorldSnapshotDescriptor()
    message.snapshot_id = (
        "b53a75163efb8eddf8784783fcfd9e4aae6438b2a8ec8db95f1248dcbed462ad"
    )
    message.creation_time.seconds = 1
    message.state_epoch = 5
    message.components.add().component_kind = "current_field"
    golden = bytes.fromhex(_ZERO_REVISION_HEX)
    self.assertEqual(message.SerializeToString(), golden)
    parsed = _PB2.WorldSnapshotDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(len(parsed.components), 1)
    self.assertEqual(parsed.components[0].component_kind, "current_field")
    self.assertEqual(parsed.components[0].revision, 0)

  def test_unknown_fields_round_trip(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    # Top level: field 99, varint 7.
    with_unknown = golden + bytes((0x98, 0x06, 0x07))
    parsed = _PB2.WorldSnapshotDescriptor()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.state_epoch, 42)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

    # Nested component: field 77, length-delimited "future".
    component = bytes.fromhex("0a0d63757272656e745f6669656c641007")
    nested_component = component + bytes((0xEA, 0x04, 0x06)) + b"future"
    raw = b"\x22" + bytes((len(nested_component),)) + nested_component
    nested = _PB2.WorldSnapshotDescriptor()
    nested.ParseFromString(raw)
    self.assertEqual(nested.components[0].revision, 7)
    self.assertEqual(nested.SerializeToString(), raw)


if __name__ == "__main__":
  unittest.main()
