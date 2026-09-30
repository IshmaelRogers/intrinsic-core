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

"""Serialization and textproto tests for PressureDepthMeasurement."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import fake_pressure_depth
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import pressure_depth_pb2
from intrinsic.hardware.marine import pressure_depth_policy

from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of the default FakePressureDepth / _fill_nominal().
# Keep in sync with pressure_depth_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000f03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000d03f00000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e671100000000006a"
    "08411900000000000024402002290000000000049040"
)


def _fill_covariance(matrix):
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[pressure_depth_policy.PRESSURE_VARIANCE_SLOT] = 1.0
  values[pressure_depth_policy.DEPTH_VARIANCE_SLOT] = 0.25
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
  sample.pressure_pa = 200000.0
  sample.depth_m = 10.0
  sample.depth_provenance = (
      pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_FROM_PRESSURE
  )
  sample.fluid_density_kg_m3 = 1025.0


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
  return pressure_depth_policy.PressureDepthMeasurementView(
      health=health_view,
      pressure_present=sample.HasField("pressure_pa"),
      pressure_pa=sample.pressure_pa,
      depth_present=sample.HasField("depth_m"),
      depth_m=sample.depth_m,
      provenance_present=sample.HasField("depth_provenance"),
      depth_provenance=sample.depth_provenance,
      density_present=sample.HasField("fluid_density_kg_m3"),
      fluid_density_kg_m3=sample.fluid_density_kg_m3,
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


class PressureDepthSerializationTest(unittest.TestCase):

  def test_contract_shape_and_unchanged_validity(self):
    descriptor = pressure_depth_pb2.PressureDepthMeasurement.DESCRIPTOR
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
    message = pressure_depth_pb2.PressureDepthMeasurement
    self.assertEqual(message.DEPTH_PROVENANCE_UNSPECIFIED, 0)
    self.assertEqual(message.DEPTH_PROVENANCE_DIRECT, 1)
    self.assertEqual(message.DEPTH_PROVENANCE_FROM_PRESSURE, 2)

  def test_defaults_are_empty_and_opt_in(self):
    sample = pressure_depth_pb2.PressureDepthMeasurement()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("health"))
    self.assertFalse(sample.HasField("pressure_pa"))
    self.assertFalse(sample.HasField("depth_m"))
    self.assertFalse(sample.HasField("depth_provenance"))
    self.assertFalse(sample.HasField("fluid_density_kg_m3"))

  def test_nominal_round_trip_matches_golden_and_fake(self):
    sample = pressure_depth_pb2.PressureDepthMeasurement()
    _fill_nominal(sample)
    faked = fake_pressure_depth.FakePressureDepth().measure()
    self.assertEqual(sample.SerializeToString(), faked.SerializeToString())
    self.assertEqual(sample.SerializeToString().hex(), _NOMINAL_GOLDEN_HEX)
    parsed = pressure_depth_pb2.PressureDepthMeasurement()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )
    self.assertEqual(parsed.health.header.frame_id, "sensor")
    self.assertEqual(parsed.pressure_pa, 200000.0)
    self.assertEqual(parsed.depth_m, 10.0)
    self.assertEqual(
        parsed.depth_provenance,
        pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_FROM_PRESSURE,
    )
    self.assertEqual(parsed.fluid_density_kg_m3, 1025.0)
    self.assertEqual(
        parsed.health.covariance.values[
            pressure_depth_policy.PRESSURE_VARIANCE_SLOT
        ],
        1.0,
    )
    self.assertEqual(
        parsed.health.covariance.values[
            pressure_depth_policy.DEPTH_VARIANCE_SLOT
        ],
        0.25,
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(_view_of(parsed)).accepted
    )

  def test_presence_is_distinct_from_default_values(self):
    unset = pressure_depth_pb2.PressureDepthMeasurement()
    specified = pressure_depth_pb2.PressureDepthMeasurement()
    specified.depth_provenance = (
        pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_UNSPECIFIED
    )
    self.assertFalse(unset.HasField("depth_provenance"))
    self.assertTrue(specified.HasField("depth_provenance"))
    self.assertNotEqual(
        unset.SerializeToString(), specified.SerializeToString()
    )
    pressure = pressure_depth_pb2.PressureDepthMeasurement()
    pressure.pressure_pa = 0.0
    self.assertNotEqual(
        pressure.SerializeToString(),
        pressure_depth_pb2.PressureDepthMeasurement().SerializeToString(),
    )
    depth = pressure_depth_pb2.PressureDepthMeasurement()
    depth.depth_m = 0.0
    self.assertTrue(depth.HasField("depth_m"))
    density = pressure_depth_pb2.PressureDepthMeasurement()
    density.fluid_density_kg_m3 = 0.0
    self.assertTrue(density.HasField("fluid_density_kg_m3"))

  def test_clearing_density_is_a_prefix_of_the_golden(self):
    sample = pressure_depth_pb2.PressureDepthMeasurement()
    _fill_nominal(sample)
    full = sample.SerializeToString()
    sample.ClearField("fluid_density_kg_m3")
    self.assertTrue(full.startswith(sample.SerializeToString()))
    self.assertNotEqual(full, sample.SerializeToString())

  def test_unknown_field_and_provenance_are_preserved(self):
    sample = pressure_depth_pb2.PressureDepthMeasurement()
    _fill_nominal(sample)
    golden = sample.SerializeToString()
    with_unknown = golden + b"\xa0\x06\x07"
    parsed = pressure_depth_pb2.PressureDepthMeasurement()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(
        parsed.depth_provenance,
        pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_FROM_PRESSURE,
    )
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    parsed.depth_provenance = 100
    reparsed = pressure_depth_pb2.PressureDepthMeasurement()
    reparsed.ParseFromString(parsed.SerializeToString())
    self.assertEqual(reparsed.depth_provenance, 100)
    self.assertEqual(
        pressure_depth_policy.classify_depth_provenance(
            reparsed.HasField("depth_provenance"), reparsed.depth_provenance
        ),
        pressure_depth_policy.DepthProvenanceKind.UNRECOGNIZED,
    )

  def test_textproto_example_matches_the_fixture(self):
    parsed = pressure_depth_pb2.PressureDepthMeasurement()
    text_format.Parse(
        _load_example("pressure_depth_from_pressure.textproto"), parsed
    )
    coded = pressure_depth_pb2.PressureDepthMeasurement()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )


if __name__ == "__main__":
  unittest.main()
