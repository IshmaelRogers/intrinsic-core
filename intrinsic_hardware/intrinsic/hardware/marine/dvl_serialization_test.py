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

"""Serialization and textproto tests for DvlMeasurement."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import dvl_pb2
from intrinsic.hardware.marine import dvl_policy
from intrinsic.hardware.marine import fake_dvl
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakeDvl / _fill_nominal(). Keep in
# sync with dvl_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000d03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000d03f00000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000d03f0000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e6710011900000000"
    "0000e03f21000000000000d0bf2900000000000000003001390000000000002440"
)


def _fill_linear_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[vehicle_contract_policy.covariance_index(0, 0)] = 0.25
  values[vehicle_contract_policy.covariance_index(1, 1)] = 0.25
  values[vehicle_contract_policy.covariance_index(2, 2)] = 0.25
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
  _fill_linear_covariance(health.covariance)
  primary = health.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.sources.add().source_id = "aiding"
  sample.mode = dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK
  sample.velocity_x_m_s = 0.5
  sample.velocity_y_m_s = -0.25
  sample.velocity_z_m_s = 0.0
  sample.bottom_lock = True
  sample.altitude_m = 10.0


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
  return dvl_policy.DvlMeasurementView(
      health=health_view,
      mode_present=sample.HasField("mode"),
      mode=sample.mode,
      velocity_x_present=sample.HasField("velocity_x_m_s"),
      velocity_x_m_s=sample.velocity_x_m_s,
      velocity_y_present=sample.HasField("velocity_y_m_s"),
      velocity_y_m_s=sample.velocity_y_m_s,
      velocity_z_present=sample.HasField("velocity_z_m_s"),
      velocity_z_m_s=sample.velocity_z_m_s,
      bottom_lock_present=sample.HasField("bottom_lock"),
      bottom_lock=sample.bottom_lock,
      altitude_present=sample.HasField("altitude_m"),
      altitude_m=sample.altitude_m,
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


class DvlSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = dvl_pb2.DvlMeasurement.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(
        descriptor.fields_by_name["health"].message_type.full_name,
        "intrinsic_proto.hardware.marine.MeasurementHealth",
    )
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))
    self.assertIsNone(validity.values_by_name.get("DEGRADED"))
    self.assertEqual(dvl_pb2.DvlMeasurement.MODE_UNSPECIFIED, 0)
    self.assertEqual(dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK, 1)
    self.assertEqual(dvl_pb2.DvlMeasurement.MODE_WATER_TRACK, 2)

  def test_defaults_are_empty_and_opt_in(self):
    sample = dvl_pb2.DvlMeasurement()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("health"))
    self.assertFalse(sample.HasField("mode"))
    self.assertFalse(sample.HasField("velocity_x_m_s"))
    self.assertFalse(sample.HasField("bottom_lock"))
    self.assertFalse(sample.HasField("altitude_m"))

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = dvl_pb2.DvlMeasurement()
    _fill_nominal(sample)
    faked = fake_dvl.FakeDvl().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = dvl_pb2.DvlMeasurement()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )
    self.assertEqual(parsed.health.header.frame_id, "sensor")
    self.assertEqual(parsed.mode, dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK)
    self.assertEqual(parsed.velocity_x_m_s, 0.5)
    self.assertEqual(parsed.velocity_y_m_s, -0.25)
    self.assertEqual(parsed.velocity_z_m_s, 0.0)
    self.assertTrue(parsed.bottom_lock)
    self.assertEqual(parsed.altitude_m, 10.0)
    self.assertEqual(parsed.health.covariance.values[21], 0.0)
    self.assertEqual(parsed.health.covariance.values[28], 0.0)
    self.assertEqual(parsed.health.covariance.values[35], 0.0)
    self.assertTrue(dvl_policy.assess_dvl(_view_of(parsed)).accepted)

  def test_presence_is_distinct_from_default_values(self):
    unset = dvl_pb2.DvlMeasurement()
    specified = dvl_pb2.DvlMeasurement()
    specified.mode = dvl_pb2.DvlMeasurement.MODE_UNSPECIFIED
    self.assertFalse(unset.HasField("mode"))
    self.assertTrue(specified.HasField("mode"))
    self.assertNotEqual(
        unset.SerializeToString(), specified.SerializeToString()
    )
    locked = dvl_pb2.DvlMeasurement()
    locked.bottom_lock = False
    self.assertTrue(locked.HasField("bottom_lock"))
    with_false = locked.SerializeToString()
    locked.ClearField("bottom_lock")
    self.assertNotEqual(locked.SerializeToString(), with_false)
    altitude = dvl_pb2.DvlMeasurement()
    altitude.altitude_m = 0.0
    self.assertNotEqual(
        altitude.SerializeToString(),
        dvl_pb2.DvlMeasurement().SerializeToString(),
    )

  def test_clearing_altitude_is_a_prefix_of_the_golden(self):
    sample = dvl_pb2.DvlMeasurement()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("altitude_m")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertNotEqual(full, sample.SerializeToString())

  def test_unknown_field_and_mode_are_preserved(self):
    sample = dvl_pb2.DvlMeasurement()
    _fill_nominal(sample)
    golden = sample.SerializeToString()
    with_unknown = golden + b"\xa0\x06\x07"
    parsed = dvl_pb2.DvlMeasurement()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.mode, dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK)
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    parsed.mode = 100
    reparsed = dvl_pb2.DvlMeasurement()
    reparsed.ParseFromString(parsed.SerializeToString())
    self.assertEqual(reparsed.mode, 100)
    self.assertEqual(
        dvl_policy.classify_dvl_mode(reparsed.HasField("mode"), reparsed.mode),
        dvl_policy.DvlModeKind.UNRECOGNIZED,
    )

  def test_textproto_example_matches_the_fixture(self):
    parsed = dvl_pb2.DvlMeasurement()
    text_format.Parse(_load_example("dvl_bottom_track.textproto"), parsed)
    coded = dvl_pb2.DvlMeasurement()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )


if __name__ == "__main__":
  unittest.main()
