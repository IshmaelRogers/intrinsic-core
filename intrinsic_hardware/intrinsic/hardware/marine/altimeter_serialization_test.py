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

"""Serialization and textproto tests for AltimeterMeasurement."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import altimeter_pb2
from intrinsic.hardware.marine import altimeter_policy
from intrinsic.hardware.marine import fake_altimeter
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakeAltimeter / _fill_nominal().
# Keep in sync with altimeter_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000d03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e6711000000000000"
    "24401a04646f776e21000000000000e03f2900000000000059403001"
)


def _fill_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[altimeter_policy.RANGE_VARIANCE_SLOT] = 0.25
  matrix.values[:] = values


def _fill_nominal(sample):
  health = sample.health
  header = health.header
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "nav_sensor"
  header.frame_id = "sensor"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.state = measurement_health_pb2.MeasurementHealth.VALID
  health.quality = 0.75
  _fill_covariance(health.covariance)
  primary = health.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.sources.add().source_id = "aiding"
  sample.range_m = 10.0
  sample.beam_id = "down"
  sample.min_range_m = 0.5
  sample.max_range_m = 100.0
  sample.has_return = True


def _view_of(sample):
  health = sample.health
  covariance = (
      tuple(health.covariance.values) if health.HasField("covariance") else ()
  )
  sources = tuple(
      measurement_health_policy.SourceHealthView(
          source.source_id,
          source.HasField("validity"),
          source.validity.state,
      )
      for source in health.sources
  )
  header = health.header
  health_view = measurement_health_policy.MeasurementHealthView(
      header_present=sample.HasField("health"),
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      header_validity_present=header.HasField("validity"),
      header_validity_state=header.validity.state,
      state_present=health.HasField("state"),
      state=health.state,
      quality_present=health.HasField("quality"),
      quality=health.quality,
      covariance_present=health.HasField("covariance"),
      covariance=covariance,
      sources=sources,
  )
  return altimeter_policy.AltimeterMeasurementView(
      health=health_view,
      range_present=sample.HasField("range_m"),
      range_m=sample.range_m,
      beam_present=sample.HasField("beam_id"),
      beam_id=sample.beam_id,
      min_present=sample.HasField("min_range_m"),
      min_range_m=sample.min_range_m,
      max_present=sample.HasField("max_range_m"),
      max_range_m=sample.max_range_m,
      has_return_present=sample.HasField("has_return"),
      has_return=sample.has_return,
  )


def _load_example(name):
  root = os.environ.get("TEST_SRCDIR", "")
  matches = []
  for dirpath, _, filenames in os.walk(root):
    if name in filenames and "examples" in dirpath:
      matches.append(os.path.join(dirpath, name))
  if not matches:
    raise AssertionError("missing %s under %s" % (name, root))
  matches.sort(key=len)
  with open(matches[0], encoding="utf-8") as handle:
    return handle.read()


class AltimeterSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = altimeter_pb2.AltimeterMeasurement.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(
        descriptor.fields_by_name["health"].message_type.full_name,
        "intrinsic_proto.hardware.marine.MeasurementHealth",
    )
    self.assertIsNone(descriptor.fields_by_name.get("altitude_m"))
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))
    self.assertIsNone(validity.values_by_name.get("DEGRADED"))

  def test_defaults_are_empty_and_opt_in(self):
    sample = altimeter_pb2.AltimeterMeasurement()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("health"))
    self.assertFalse(sample.HasField("range_m"))
    self.assertFalse(sample.HasField("beam_id"))
    self.assertFalse(sample.HasField("min_range_m"))
    self.assertFalse(sample.HasField("max_range_m"))
    self.assertFalse(sample.HasField("has_return"))

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = altimeter_pb2.AltimeterMeasurement()
    _fill_nominal(sample)
    faked = fake_altimeter.FakeAltimeter().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = altimeter_pb2.AltimeterMeasurement()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )
    self.assertEqual(parsed.health.header.frame_id, "sensor")
    self.assertEqual(parsed.range_m, 10.0)
    self.assertEqual(parsed.beam_id, "down")
    self.assertEqual(parsed.min_range_m, 0.5)
    self.assertEqual(parsed.max_range_m, 100.0)
    self.assertTrue(parsed.has_return)
    self.assertEqual(
        parsed.health.covariance.values[altimeter_policy.RANGE_VARIANCE_SLOT],
        0.25,
    )
    self.assertTrue(
        altimeter_policy.assess_altimeter(_view_of(parsed)).accepted
    )

  def test_presence_is_distinct_from_default_values(self):
    unset = altimeter_pb2.AltimeterMeasurement()
    range_zero = altimeter_pb2.AltimeterMeasurement()
    range_zero.range_m = 0.0
    self.assertTrue(range_zero.HasField("range_m"))
    self.assertNotEqual(
        range_zero.SerializeToString(), unset.SerializeToString()
    )
    no_return = altimeter_pb2.AltimeterMeasurement()
    no_return.has_return = False
    self.assertTrue(no_return.HasField("has_return"))
    self.assertFalse(no_return.has_return)
    self.assertNotEqual(
        no_return.SerializeToString(), unset.SerializeToString()
    )
    beam = altimeter_pb2.AltimeterMeasurement()
    beam.beam_id = ""
    self.assertTrue(beam.HasField("beam_id"))

  def test_clearing_has_return_is_a_prefix_of_the_golden(self):
    sample = altimeter_pb2.AltimeterMeasurement()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("has_return")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertNotEqual(full, sample.SerializeToString())

  def test_unknown_field_is_preserved(self):
    sample = altimeter_pb2.AltimeterMeasurement()
    _fill_nominal(sample)
    golden = sample.SerializeToString()
    with_unknown = golden + b"\xa0\x06\x07"
    parsed = altimeter_pb2.AltimeterMeasurement()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.range_m, 10.0)
    self.assertTrue(parsed.has_return)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_textproto_example_matches_the_fixture(self):
    parsed = altimeter_pb2.AltimeterMeasurement()
    text_format.Parse(_load_example("altimeter_nominal.textproto"), parsed)
    coded = altimeter_pb2.AltimeterMeasurement()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )


if __name__ == "__main__":
  unittest.main()
