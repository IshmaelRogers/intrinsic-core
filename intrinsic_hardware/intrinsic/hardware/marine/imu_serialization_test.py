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

"""Serialization and textproto tests for ImuMeasurement."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import fake_imu
from intrinsic.hardware.marine import imu_pb2
from intrinsic.hardware.marine import imu_policy
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakeImu / _fill_nominal().
# Keep in sync with imu_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696d7532096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000d03f000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000e0"
    "3f000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000c03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000f03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000004000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000010402a0d0a077072696d6172791202"
    "08012a080a06616964696e6711000000000000d03f19000000000000e0bf21000000"
    "000000c03f2900000000000000003100000000000000003900000000000020404209"
    "21000000000000f03f"
)

_RATES_ONLY_GOLDEN_HEX = (
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696d7532096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000d03f000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000e0"
    "3f000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000c03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000f03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000004000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000010402a0d0a077072696d6172791202"
    "08012a080a06616964696e6711000000000000d03f19000000000000e0bf21000000"
    "000000c03f290000000000000000310000000000000000390000000000002040"
)


def _fill_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[imu_policy.ANGULAR_VELOCITY_X_VARIANCE_SLOT] = 0.25
  values[imu_policy.ANGULAR_VELOCITY_Y_VARIANCE_SLOT] = 0.5
  values[imu_policy.ANGULAR_VELOCITY_Z_VARIANCE_SLOT] = 0.125
  values[imu_policy.LINEAR_ACCELERATION_X_VARIANCE_SLOT] = 1.0
  values[imu_policy.LINEAR_ACCELERATION_Y_VARIANCE_SLOT] = 2.0
  values[imu_policy.LINEAR_ACCELERATION_Z_VARIANCE_SLOT] = 4.0
  matrix.values[:] = values


def _fill_health(sample):
  health = sample.health
  header = health.header
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "nav_sensor"
  header.frame_id = "imu"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.state = measurement_health_pb2.MeasurementHealth.VALID
  health.quality = 0.75
  _fill_covariance(health.covariance)
  primary = health.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.sources.add().source_id = "aiding"


def _fill_rates(sample):
  sample.angular_velocity_x_rad_s = 0.25
  sample.angular_velocity_y_rad_s = -0.5
  sample.angular_velocity_z_rad_s = 0.125
  sample.linear_acceleration_x_m_s2 = 0.0
  sample.linear_acceleration_y_m_s2 = 0.0
  sample.linear_acceleration_z_m_s2 = 8.0


def _fill_nominal(sample):
  _fill_health(sample)
  _fill_rates(sample)
  sample.orientation_xyzw.x = 0.0
  sample.orientation_xyzw.y = 0.0
  sample.orientation_xyzw.z = 0.0
  sample.orientation_xyzw.w = 1.0


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
  orientation = sample.orientation_xyzw
  return imu_policy.ImuMeasurementView(
      health=health_view,
      angular_velocity_x_present=sample.HasField("angular_velocity_x_rad_s"),
      angular_velocity_x_rad_s=sample.angular_velocity_x_rad_s,
      angular_velocity_y_present=sample.HasField("angular_velocity_y_rad_s"),
      angular_velocity_y_rad_s=sample.angular_velocity_y_rad_s,
      angular_velocity_z_present=sample.HasField("angular_velocity_z_rad_s"),
      angular_velocity_z_rad_s=sample.angular_velocity_z_rad_s,
      linear_acceleration_x_present=sample.HasField(
          "linear_acceleration_x_m_s2"
      ),
      linear_acceleration_x_m_s2=sample.linear_acceleration_x_m_s2,
      linear_acceleration_y_present=sample.HasField(
          "linear_acceleration_y_m_s2"
      ),
      linear_acceleration_y_m_s2=sample.linear_acceleration_y_m_s2,
      linear_acceleration_z_present=sample.HasField(
          "linear_acceleration_z_m_s2"
      ),
      linear_acceleration_z_m_s2=sample.linear_acceleration_z_m_s2,
      orientation_present=sample.HasField("orientation_xyzw"),
      orientation_x=orientation.x,
      orientation_y=orientation.y,
      orientation_z=orientation.z,
      orientation_w=orientation.w,
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


class ImuSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = imu_pb2.ImuMeasurement.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(
        descriptor.fields_by_name["health"].message_type.full_name,
        "intrinsic_proto.hardware.marine.MeasurementHealth",
    )
    self.assertIsNone(descriptor.fields_by_name.get("magnetic_field_x"))
    self.assertIsNone(descriptor.fields_by_name.get("altitude_m"))
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))

  def test_defaults_are_empty_and_opt_in(self):
    sample = imu_pb2.ImuMeasurement()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("health"))
    self.assertFalse(sample.HasField("angular_velocity_x_rad_s"))
    self.assertFalse(sample.HasField("orientation_xyzw"))

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = imu_pb2.ImuMeasurement()
    _fill_nominal(sample)
    faked = fake_imu.FakeImu().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = imu_pb2.ImuMeasurement()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(parsed.health.header.frame_id, "imu")
    self.assertEqual(parsed.angular_velocity_y_rad_s, -0.5)
    self.assertEqual(parsed.orientation_xyzw.w, 1.0)
    self.assertTrue(imu_policy.assess_imu(_view_of(parsed)).accepted)

  def test_rates_only_round_trip_matches_golden_and_fake(self):
    sample = imu_pb2.ImuMeasurement()
    _fill_health(sample)
    _fill_rates(sample)
    truth = fake_imu.ImuTruth(orientation_present=False)
    faked = fake_imu.FakeImu().measure(truth)
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _RATES_ONLY_GOLDEN_HEX)
    self.assertTrue(_NOMINAL_GOLDEN_HEX.startswith(_RATES_ONLY_GOLDEN_HEX))
    parsed = imu_pb2.ImuMeasurement()
    parsed.ParseFromString(bytes.fromhex(_RATES_ONLY_GOLDEN_HEX))
    self.assertFalse(parsed.HasField("orientation_xyzw"))
    self.assertTrue(imu_policy.assess_imu(_view_of(parsed)).accepted)

  def test_presence_is_distinct_from_zero(self):
    unset = imu_pb2.ImuMeasurement()
    zero = imu_pb2.ImuMeasurement()
    zero.angular_velocity_x_rad_s = 0.0
    self.assertTrue(zero.HasField("angular_velocity_x_rad_s"))
    self.assertNotEqual(zero.SerializeToString(), unset.SerializeToString())
    accel = imu_pb2.ImuMeasurement()
    accel.linear_acceleration_z_m_s2 = 0.0
    self.assertNotEqual(accel.SerializeToString(), unset.SerializeToString())

  def test_clearing_orientation_is_a_prefix_of_the_golden(self):
    sample = imu_pb2.ImuMeasurement()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("orientation_xyzw")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertEqual(sample.SerializeToString().hex(), _RATES_ONLY_GOLDEN_HEX)

  def test_unknown_field_is_preserved(self):
    sample = imu_pb2.ImuMeasurement()
    _fill_nominal(sample)
    with_unknown = sample.SerializeToString() + b"\xa0\x06\x07"
    parsed = imu_pb2.ImuMeasurement()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.angular_velocity_x_rad_s, 0.25)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_textproto_examples_match_the_fixtures(self):
    nominal = imu_pb2.ImuMeasurement()
    text_format.Parse(_load_example("imu_nominal.textproto"), nominal)
    coded = imu_pb2.ImuMeasurement()
    _fill_nominal(coded)
    self.assertEqual(nominal.SerializeToString(), coded.SerializeToString())
    self.assertEqual(nominal.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    rates = imu_pb2.ImuMeasurement()
    text_format.Parse(_load_example("imu_rates_only.textproto"), rates)
    self.assertEqual(rates.SerializeToString().hex(), _RATES_ONLY_GOLDEN_HEX)


if __name__ == "__main__":
  unittest.main()
