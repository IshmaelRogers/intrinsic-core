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

"""Serialization and textproto tests for SurfaceFix."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import fake_surface_fix
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import surface_fix_pb2
from intrinsic.hardware.marine import surface_fix_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakeSurfaceFix / _fill_nominal().
# Keep in sync with surface_fix_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0a80030a38082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a04676e737332096d6f6e6f746f6e69633a02080110011d0000403f22a3020a"
    "a002000000000000f03f0000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000001040000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000d03f00000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "00002a0d0a077072696d617279120208012a080a06616964696e67110000000000002940"
    "190000000000000ac021000000000000f03f2801300c"
)


def _fill_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[surface_fix_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[surface_fix_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[surface_fix_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  matrix.values[:] = values


def _fill_nominal(sample):
  health = sample.health
  header = health.header
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "nav_sensor"
  header.frame_id = "gnss"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.state = measurement_health_pb2.MeasurementHealth.VALID
  health.quality = 0.75
  _fill_covariance(health.covariance)
  primary = health.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  health.sources.add().source_id = "aiding"
  sample.position_x_m = 12.5
  sample.position_y_m = -3.25
  sample.position_z_m = 1.0
  sample.source = surface_fix_pb2.SurfaceFix.FIX_SOURCE_GNSS
  sample.satellite_count = 12


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
  return surface_fix_policy.SurfaceFixView(
      health=health_view,
      position_x_present=sample.HasField("position_x_m"),
      position_x_m=sample.position_x_m,
      position_y_present=sample.HasField("position_y_m"),
      position_y_m=sample.position_y_m,
      position_z_present=sample.HasField("position_z_m"),
      position_z_m=sample.position_z_m,
      source_present=sample.HasField("source"),
      source=sample.source,
      satellite_count_present=sample.HasField("satellite_count"),
      satellite_count=sample.satellite_count,
      beacon_count_present=sample.HasField("beacon_count"),
      beacon_count=sample.beacon_count,
      horizontal_accuracy_present=sample.HasField("horizontal_accuracy_m"),
      horizontal_accuracy_m=sample.horizontal_accuracy_m,
      vertical_accuracy_present=sample.HasField("vertical_accuracy_m"),
      vertical_accuracy_m=sample.vertical_accuracy_m,
      velocity_x_present=sample.HasField("velocity_x_m_s"),
      velocity_x_m_s=sample.velocity_x_m_s,
      velocity_y_present=sample.HasField("velocity_y_m_s"),
      velocity_y_m_s=sample.velocity_y_m_s,
      velocity_z_present=sample.HasField("velocity_z_m_s"),
      velocity_z_m_s=sample.velocity_z_m_s,
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


class SurfaceFixSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = surface_fix_pb2.SurfaceFix.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(descriptor.name, "SurfaceFix")
    self.assertEqual(
        descriptor.fields_by_name["health"].message_type.full_name,
        "intrinsic_proto.hardware.marine.MeasurementHealth",
    )
    self.assertIsNone(descriptor.fields_by_name.get("orientation_xyzw"))
    self.assertIsNone(descriptor.fields_by_name.get("angular_velocity_x_rad_s"))
    source = descriptor.enum_types_by_name["FixSource"]
    self.assertEqual(
        [(value.name, value.number) for value in source.values],
        [
            ("FIX_SOURCE_UNSPECIFIED", 0),
            ("FIX_SOURCE_GNSS", 1),
            ("FIX_SOURCE_ACOUSTIC", 2),
            ("FIX_SOURCE_OTHER", 3),
        ],
    )
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))

  def test_defaults_are_empty_and_opt_in(self):
    sample = surface_fix_pb2.SurfaceFix()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("health"))
    for name in (
        "position_x_m",
        "position_y_m",
        "position_z_m",
        "source",
        "satellite_count",
        "beacon_count",
        "horizontal_accuracy_m",
        "vertical_accuracy_m",
        "velocity_x_m_s",
        "velocity_y_m_s",
        "velocity_z_m_s",
    ):
      self.assertFalse(sample.HasField(name), name)

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = surface_fix_pb2.SurfaceFix()
    _fill_nominal(sample)
    faked = fake_surface_fix.FakeSurfaceFix().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = surface_fix_pb2.SurfaceFix()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )
    self.assertEqual(parsed.health.header.frame_id, "gnss")
    self.assertEqual(parsed.position_x_m, 12.5)
    self.assertEqual(parsed.position_y_m, -3.25)
    self.assertEqual(parsed.position_z_m, 1.0)
    self.assertEqual(parsed.source, surface_fix_pb2.SurfaceFix.FIX_SOURCE_GNSS)
    self.assertEqual(parsed.satellite_count, 12)
    self.assertEqual(parsed.health.quality, 0.75)
    values = parsed.health.covariance.values
    self.assertEqual(values[surface_fix_policy.POSITION_X_VARIANCE_SLOT], 1.0)
    self.assertEqual(values[surface_fix_policy.POSITION_Y_VARIANCE_SLOT], 4.0)
    self.assertEqual(values[surface_fix_policy.POSITION_Z_VARIANCE_SLOT], 0.25)
    self.assertTrue(
        surface_fix_policy.assess_surface_fix(_view_of(parsed)).accepted
    )

  def test_presence_is_distinct_from_default_values(self):
    unset = surface_fix_pb2.SurfaceFix()
    for name, value in (
        ("position_x_m", 0.0),
        ("satellite_count", 0),
        ("beacon_count", 0),
        ("horizontal_accuracy_m", 0.0),
        ("velocity_z_m_s", 0.0),
        ("source", surface_fix_pb2.SurfaceFix.FIX_SOURCE_UNSPECIFIED),
    ):
      sample = surface_fix_pb2.SurfaceFix()
      setattr(sample, name, value)
      self.assertTrue(sample.HasField(name), name)
      self.assertNotEqual(sample.SerializeToString(), unset.SerializeToString())

  def test_clearing_satellite_count_is_a_prefix_of_the_golden(self):
    sample = surface_fix_pb2.SurfaceFix()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("satellite_count")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertNotEqual(full, sample.SerializeToString())

  def test_unknown_source_number_is_preserved_and_not_accepted(self):
    sample = surface_fix_pb2.SurfaceFix()
    _fill_nominal(sample)
    wire = sample.SerializeToString().replace(
        b"\x28\x01\x30\x0c", b"\x28\x63\x30\x0c"
    )
    parsed = surface_fix_pb2.SurfaceFix()
    parsed.ParseFromString(wire)
    self.assertEqual(parsed.source, 99)
    self.assertEqual(parsed.SerializeToString(), wire)
    assessment = surface_fix_policy.assess_surface_fix(_view_of(parsed))
    self.assertEqual(
        assessment.error, surface_fix_policy.SurfaceFixError.SOURCE
    )
    self.assertEqual(
        assessment.source, surface_fix_policy.SurfaceFixSourceKind.UNRECOGNIZED
    )
    self.assertFalse(assessment.accepted)

  def test_unknown_field_is_preserved(self):
    sample = surface_fix_pb2.SurfaceFix()
    _fill_nominal(sample)
    golden = sample.SerializeToString()
    with_unknown = golden + b"\xa0\x06\x07"
    parsed = surface_fix_pb2.SurfaceFix()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.position_x_m, 12.5)
    self.assertEqual(parsed.satellite_count, 12)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_textproto_gnss_example_matches_the_fixture(self):
    parsed = surface_fix_pb2.SurfaceFix()
    text_format.Parse(_load_example("surface_fix_gnss.textproto"), parsed)
    coded = surface_fix_pb2.SurfaceFix()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )

  def test_textproto_acoustic_example_is_accepted(self):
    parsed = surface_fix_pb2.SurfaceFix()
    text_format.Parse(_load_example("surface_fix_acoustic.textproto"), parsed)
    self.assertEqual(
        parsed.source, surface_fix_pb2.SurfaceFix.FIX_SOURCE_ACOUSTIC
    )
    self.assertEqual(parsed.beacon_count, 4)
    self.assertFalse(parsed.HasField("satellite_count"))
    self.assertEqual(parsed.horizontal_accuracy_m, 0.5)
    self.assertEqual(parsed.velocity_z_m_s, -0.125)
    self.assertTrue(
        surface_fix_policy.assess_surface_fix(_view_of(parsed)).accepted
    )
    truth = fake_surface_fix.SurfaceFixTruth(
        frame_id="acoustic_array",
        source=2,
        satellite_count_present=False,
        beacon_count_present=True,
        beacon_count=4,
        horizontal_accuracy_present=True,
        horizontal_accuracy_m=0.5,
        vertical_accuracy_present=True,
        vertical_accuracy_m=1.0,
        velocity_present=True,
        velocity_x_m_s=0.5,
        velocity_y_m_s=0.0,
        velocity_z_m_s=-0.125,
    )
    faked = fake_surface_fix.FakeSurfaceFix().measure(truth)
    self.assertEqual(parsed.SerializeToString(), faked.SerializeToString())


if __name__ == "__main__":
  unittest.main()
