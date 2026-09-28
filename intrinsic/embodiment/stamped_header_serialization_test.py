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

"""Python serialization tests for StampedHeader."""

import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2

# Canonical serialization of _fill_fixture(). Keep in sync with
# stamped_header_serialization_test.cc.
_GOLDEN_HEX = (
    "082a120b0880e2cfaa061080e59a771a060881e2cfaa06"
    "2205696d755f302a09776f726c645f656e75"
    "32096d6f6e6f746f6e69633a020802"
)
_WITHOUT_VALIDITY_HEX = (
    "082a120b0880e2cfaa061080e59a771a060881e2cfaa06"
    "2205696d755f302a09776f726c645f656e75"
    "32096d6f6e6f746f6e6963"
)


def _fill_fixture():
  header = stamped_header_pb2.StampedHeader()
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.receive_time.nanos = 0
  header.source_id = "imu_0"
  header.frame_id = frame_policy.WORLD_ENU_FRAME_ID
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_INVALID
  return header


class StampedHeaderSerializationTest(unittest.TestCase):

  def test_golden_round_trip(self):
    header = _fill_fixture()
    golden = bytes.fromhex(_GOLDEN_HEX)
    self.assertEqual(header.SerializeToString(), golden)
    parsed = stamped_header_pb2.StampedHeader()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.sequence, 42)
    self.assertEqual(parsed.source_time.seconds, 1700000000)
    self.assertEqual(parsed.source_time.nanos, 250000000)
    self.assertEqual(parsed.receive_time.seconds, 1700000001)
    self.assertEqual(parsed.receive_time.nanos, 0)
    self.assertEqual(parsed.source_id, "imu_0")
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.clock_domain, "monotonic")
    self.assertTrue(parsed.HasField("validity"))
    self.assertEqual(
        parsed.validity.state, stamped_header_pb2.Validity.STATE_INVALID
    )
    self.assertEqual(parsed.SerializeToString(), golden)

  def test_absent_validity_is_omitted_and_distinct(self):
    header = _fill_fixture()
    header.ClearField("validity")
    without = bytes.fromhex(_WITHOUT_VALIDITY_HEX)
    self.assertEqual(header.SerializeToString(), without)
    self.assertTrue(bytes.fromhex(_GOLDEN_HEX).startswith(without))
    parsed = stamped_header_pb2.StampedHeader()
    parsed.ParseFromString(without)
    self.assertFalse(parsed.HasField("validity"))
    self.assertEqual(
        parsed.validity.state, stamped_header_pb2.Validity.STATE_UNSPECIFIED
    )
    self.assertIs(
        stamped_header_policy.classify_validity(
            parsed.HasField("validity"), parsed.validity.state
        ),
        stamped_header_policy.ValidityKind.ABSENT,
    )
    self.assertIs(
        stamped_header_policy.classify_validity(
            True, stamped_header_pb2.Validity.STATE_INVALID
        ),
        stamped_header_policy.ValidityKind.INVALID,
    )

  def test_default_header_is_empty_and_opt_in(self):
    header = stamped_header_pb2.StampedHeader()
    self.assertEqual(header.SerializeToString(), b"")
    self.assertFalse(header.HasField("validity"))
    self.assertFalse(header.HasField("source_time"))
    self.assertFalse(header.HasField("receive_time"))
    self.assertEqual(header.frame_id, "")
    parsed = stamped_header_pb2.StampedHeader()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = stamped_header_pb2.StampedHeader()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.sequence, 42)
    self.assertEqual(parsed.frame_id, "world_enu")
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_monotonic_age_uses_clock_domain(self):
    header = _fill_fixture()
    age = stamped_header_policy.monotonic_age_seconds(
        (header.source_time.seconds, header.source_time.nanos),
        (header.receive_time.seconds, header.receive_time.nanos),
        header.clock_domain,
    )
    self.assertEqual(age, 0.75)
    header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_UTC
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            (header.source_time.seconds, header.source_time.nanos),
            (header.receive_time.seconds, header.receive_time.nanos),
            header.clock_domain,
        )
    )
    self.assertTrue(frame_policy.frame_id_matches(header.frame_id, "enu"))
    self.assertFalse(frame_policy.frame_id_matches("StampedHeader", "enu"))


if __name__ == "__main__":
  unittest.main()
