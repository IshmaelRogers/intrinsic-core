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

"""Serialization and textproto tests for InsSolution."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import fake_ins
from intrinsic.hardware.marine import ins_pb2
from intrinsic.hardware.marine import ins_policy
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakeIns / _fill_nominal().
# Keep in sync with ins_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696e7332096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000f03f000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000010"
    "40000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000d03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000b03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000c03f00"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000e03f2a0d0a077072696d6172791202"
    "08012a080a06616964696e671100000000000028401900000000000010c021000000"
    "000000e03f2a0921000000000000f03f31000000000000f83f390000000000000000"
    "41000000000000d0bf49000000000000000051000000000000c03f59000000000000"
    "00006001"
)


def _fill_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[ins_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[ins_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[ins_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  values[ins_policy.ATTITUDE_X_VARIANCE_SLOT] = 0.0625
  values[ins_policy.ATTITUDE_Y_VARIANCE_SLOT] = 0.125
  values[ins_policy.ATTITUDE_Z_VARIANCE_SLOT] = 0.5
  matrix.values[:] = values


def _fill_nominal(sample):
  health = sample.health
  header = health.header
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "nav_sensor"
  header.frame_id = "ins"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.state = measurement_health_pb2.MeasurementHealth.VALID
  health.quality = 0.75
  _fill_covariance(health.covariance)
  primary = health.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.sources.add().source_id = "aiding"
  sample.position_x_m = 12.0
  sample.position_y_m = -4.0
  sample.position_z_m = 0.5
  sample.orientation_xyzw.w = 1.0
  sample.linear_velocity_x_m_s = 1.5
  sample.linear_velocity_y_m_s = 0.0
  sample.linear_velocity_z_m_s = -0.25
  sample.angular_velocity_x_rad_s = 0.0
  sample.angular_velocity_y_rad_s = 0.125
  sample.angular_velocity_z_rad_s = 0.0
  sample.source = ins_pb2.InsSolution.SOURCE_VENDOR_INS


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
  return ins_policy.InsSolutionView(
      health=health_view,
      position_x_present=sample.HasField("position_x_m"),
      position_x_m=sample.position_x_m,
      position_y_present=sample.HasField("position_y_m"),
      position_y_m=sample.position_y_m,
      position_z_present=sample.HasField("position_z_m"),
      position_z_m=sample.position_z_m,
      orientation_present=sample.HasField("orientation_xyzw"),
      orientation_x=orientation.x,
      orientation_y=orientation.y,
      orientation_z=orientation.z,
      orientation_w=orientation.w,
      linear_velocity_x_present=sample.HasField("linear_velocity_x_m_s"),
      linear_velocity_x_m_s=sample.linear_velocity_x_m_s,
      linear_velocity_y_present=sample.HasField("linear_velocity_y_m_s"),
      linear_velocity_y_m_s=sample.linear_velocity_y_m_s,
      linear_velocity_z_present=sample.HasField("linear_velocity_z_m_s"),
      linear_velocity_z_m_s=sample.linear_velocity_z_m_s,
      angular_velocity_x_present=sample.HasField("angular_velocity_x_rad_s"),
      angular_velocity_x_rad_s=sample.angular_velocity_x_rad_s,
      angular_velocity_y_present=sample.HasField("angular_velocity_y_rad_s"),
      angular_velocity_y_rad_s=sample.angular_velocity_y_rad_s,
      angular_velocity_z_present=sample.HasField("angular_velocity_z_rad_s"),
      angular_velocity_z_rad_s=sample.angular_velocity_z_rad_s,
      source_present=sample.HasField("source"),
      source=sample.source,
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


class InsSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = ins_pb2.InsSolution.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertIsNone(descriptor.fields_by_name.get("magnetic_field_x"))
    source = descriptor.enum_types_by_name["SourceKind"]
    self.assertEqual(source.values_by_name["SOURCE_UNSPECIFIED"].number, 0)
    self.assertEqual(source.values_by_name["SOURCE_VENDOR_INS"].number, 1)
    self.assertEqual(source.values_by_name["SOURCE_EXTERNAL_NAV"].number, 2)
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))

  def test_defaults_are_empty_and_opt_in(self):
    sample = ins_pb2.InsSolution()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("position_x_m"))
    self.assertFalse(sample.HasField("orientation_xyzw"))
    self.assertFalse(sample.HasField("source"))

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = ins_pb2.InsSolution()
    _fill_nominal(sample)
    faked = fake_ins.FakeIns().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = ins_pb2.InsSolution()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(parsed.position_y_m, -4.0)
    self.assertEqual(parsed.source, ins_pb2.InsSolution.SOURCE_VENDOR_INS)
    assessment = ins_policy.assess_ins(_view_of(parsed))
    self.assertTrue(assessment.accepted)
    self.assertEqual(assessment.source, ins_policy.InsSourceKind.VENDOR_INS)

  def test_presence_is_distinct_from_unspecified(self):
    unset = ins_pb2.InsSolution()
    zero = ins_pb2.InsSolution()
    zero.position_z_m = 0.0
    self.assertTrue(zero.HasField("position_z_m"))
    self.assertNotEqual(zero.SerializeToString(), unset.SerializeToString())
    source = ins_pb2.InsSolution()
    source.source = ins_pb2.InsSolution.SOURCE_UNSPECIFIED
    self.assertTrue(source.HasField("source"))
    self.assertEqual(source.source, ins_pb2.InsSolution.SOURCE_UNSPECIFIED)
    self.assertNotEqual(source.SerializeToString(), unset.SerializeToString())

  def test_clearing_source_is_a_prefix_of_the_golden(self):
    sample = ins_pb2.InsSolution()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("source")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertNotEqual(full, sample.SerializeToString())

  def test_unknown_field_and_source_are_preserved(self):
    sample = ins_pb2.InsSolution()
    _fill_nominal(sample)
    with_unknown = sample.SerializeToString() + b"\xa0\x06\x07"
    parsed = ins_pb2.InsSolution()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.source, ins_pb2.InsSolution.SOURCE_VENDOR_INS)
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    parsed.source = 100
    reparsed = ins_pb2.InsSolution()
    reparsed.ParseFromString(parsed.SerializeToString())
    self.assertEqual(reparsed.source, 100)
    self.assertEqual(
        ins_policy.classify_ins_source(
            reparsed.HasField("source"), reparsed.source
        ),
        ins_policy.InsSourceKind.UNRECOGNIZED,
    )

  def test_textproto_example_matches_the_fixture(self):
    parsed = ins_pb2.InsSolution()
    text_format.Parse(_load_example("ins_nominal.textproto"), parsed)
    coded = ins_pb2.InsSolution()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())
    self.assertEqual(parsed.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)


if __name__ == "__main__":
  unittest.main()
