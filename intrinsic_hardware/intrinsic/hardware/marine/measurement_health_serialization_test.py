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

"""Serialization and textproto tests for marine measurement health."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

# Canonical serialization of _fill_nominal(). Keep in sync with
# measurement_health_serialization_test.cc.
_NOMINAL_GOLDEN_HEX = (
    "0a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f73656e736f"
    "722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3020aa0"
    "02000000000000d03f000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000d03f00000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000d03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000b03f000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000b03f00000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000000000b0"
    "3f2a0d0a077072696d617279120208012a080a06616964696e67"
)


def _nominal_covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[0] = 0.25
  values[7] = 0.25
  values[14] = 0.25
  values[21] = 0.0625
  values[28] = 0.0625
  values[35] = 0.0625
  return values


def _fill_header(header):
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "nav_sensor"
  header.frame_id = "sensor"
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _fill_nominal(sample):
  _fill_header(sample.header)
  sample.state = measurement_health_pb2.MeasurementHealth.VALID
  sample.quality = 0.75
  sample.covariance.values.extend(_nominal_covariance())
  primary = sample.sources.add()
  primary.source_id = "primary"
  primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
  sample.sources.add().source_id = "aiding"


def _view_of(sample):
  covariance = (
      tuple(sample.covariance.values) if sample.HasField("covariance") else ()
  )
  sources = tuple(
      measurement_health_policy.SourceHealthView(
          source.source_id,
          source.HasField("validity"),
          source.validity.state,
      )
      for source in sample.sources
  )
  header = sample.header
  return measurement_health_policy.MeasurementHealthView(
      header_present=sample.HasField("header"),
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      header_validity_present=header.HasField("validity"),
      header_validity_state=header.validity.state,
      state_present=sample.HasField("state"),
      state=sample.state,
      quality_present=sample.HasField("quality"),
      quality=sample.quality,
      covariance_present=sample.HasField("covariance"),
      covariance=covariance,
      sources=sources,
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


class MeasurementHealthSerializationTest(unittest.TestCase):

  def test_no_sensor_payload_on_the_contract(self):
    descriptor = measurement_health_pb2.MeasurementHealth.DESCRIPTOR
    self.assertEqual(descriptor.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(len(descriptor.file.message_types_by_name), 1)
    self.assertEqual(
        descriptor.fields_by_name["header"].message_type.full_name,
        "intrinsic_proto.embodiment.StampedHeader",
    )
    self.assertEqual(
        descriptor.fields_by_name["covariance"].message_type.full_name,
        "intrinsic_proto.vehicle.Matrix6",
    )
    self.assertEqual(
        descriptor.fields_by_name["sources"].message_type.full_name,
        "intrinsic_proto.vehicle.SourceHealth",
    )
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))
    self.assertIsNone(validity.values_by_name.get("DEGRADED"))
    for name in (
        "dvl",
        "imu",
        "ins",
        "depth",
        "pressure",
        "altitude",
        "altimeter",
        "gnss",
        "gps",
        "sonar",
        "robot_type",
        "embodiment",
        "payload",
    ):
      self.assertIsNone(descriptor.fields_by_name.get(name))

  def test_defaults_are_empty_and_opt_in(self):
    sample = measurement_health_pb2.MeasurementHealth()
    self.assertEqual(sample.SerializeToString(), b"")
    self.assertFalse(sample.HasField("header"))
    self.assertFalse(sample.HasField("state"))
    self.assertFalse(sample.HasField("quality"))
    self.assertFalse(sample.HasField("covariance"))
    self.assertEqual(len(sample.sources), 0)

  def test_nominal_round_trip_is_stable(self):
    sample = measurement_health_pb2.MeasurementHealth()
    _fill_nominal(sample)
    hex_bytes = sample.SerializeToString().hex()
    self.assertEqual(hex_bytes, _NOMINAL_GOLDEN_HEX)
    parsed = measurement_health_pb2.MeasurementHealth()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_GOLDEN_HEX))
    self.assertEqual(
        parsed.SerializeToString(), bytes.fromhex(_NOMINAL_GOLDEN_HEX)
    )
    self.assertEqual(parsed.header.frame_id, "sensor")
    self.assertEqual(
        parsed.state, measurement_health_pb2.MeasurementHealth.VALID
    )
    self.assertAlmostEqual(parsed.quality, 0.75)
    self.assertEqual(
        len(parsed.covariance.values), vehicle_contract_policy.COVARIANCE_VALUES
    )
    self.assertTrue(parsed.sources[0].HasField("validity"))
    self.assertFalse(parsed.sources[1].HasField("validity"))
    assessment = measurement_health_policy.assess_measurement_health(
        _view_of(parsed)
    )
    self.assertTrue(assessment.accepted)

  def test_unknown_state_and_zero_quality_are_present(self):
    unset = measurement_health_pb2.MeasurementHealth()
    unknown = measurement_health_pb2.MeasurementHealth()
    unknown.state = measurement_health_pb2.MeasurementHealth.UNKNOWN
    self.assertFalse(unset.HasField("state"))
    self.assertTrue(unknown.HasField("state"))
    self.assertNotEqual(unset.SerializeToString(), unknown.SerializeToString())
    zero_quality = measurement_health_pb2.MeasurementHealth()
    zero_quality.quality = 0.0
    self.assertTrue(zero_quality.HasField("quality"))
    with_zero = zero_quality.SerializeToString()
    zero_quality.ClearField("quality")
    self.assertNotEqual(zero_quality.SerializeToString(), with_zero)

  def test_zero_covariance_is_not_the_unknown_encoding(self):
    absent = measurement_health_pb2.MeasurementHealth()
    zeros = measurement_health_pb2.MeasurementHealth()
    zeros.covariance.values.extend(
        [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
    )
    self.assertFalse(absent.HasField("covariance"))
    self.assertTrue(zeros.HasField("covariance"))
    self.assertNotEqual(absent.SerializeToString(), zeros.SerializeToString())
    self.assertFalse(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(
                True, list(zeros.covariance.values)
            )
        )
    )

  def test_stamp_uses_the_existing_time_policy(self):
    sample = measurement_health_pb2.MeasurementHealth()
    _fill_nominal(sample)
    age = stamped_header_policy.monotonic_age_seconds(
        (sample.header.source_time.seconds, sample.header.source_time.nanos),
        (sample.header.receive_time.seconds, sample.header.receive_time.nanos),
        sample.header.clock_domain,
    )
    self.assertEqual(age, 0.75)
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            (
                sample.header.source_time.seconds,
                sample.header.source_time.nanos,
            ),
            (
                sample.header.receive_time.seconds,
                sample.header.receive_time.nanos,
            ),
            stamped_header_policy.CLOCK_DOMAIN_UTC,
        )
    )
    self.assertTrue(stamped_header_policy.sequence_advances(41, 42))
    self.assertFalse(
        frame_policy.frame_id_matches(sample.header.frame_id, "enu")
    )

  def test_textproto_examples_match_the_fixtures(self):
    parsed = measurement_health_pb2.MeasurementHealth()
    text_format.Parse(_load_example("measurement_health.textproto"), parsed)
    coded = measurement_health_pb2.MeasurementHealth()
    _fill_nominal(coded)
    self.assertEqual(parsed.SerializeToString(), coded.SerializeToString())

    unknown = measurement_health_pb2.MeasurementHealth()
    text_format.Parse(
        _load_example("measurement_health_unknown_covariance.textproto"),
        unknown,
    )
    self.assertFalse(unknown.HasField("covariance"))
    coded_unknown = measurement_health_pb2.MeasurementHealth()
    _fill_nominal(coded_unknown)
    coded_unknown.ClearField("covariance")
    self.assertEqual(
        unknown.SerializeToString(), coded_unknown.SerializeToString()
    )
    assessment = measurement_health_policy.assess_measurement_health(
        _view_of(unknown)
    )
    self.assertTrue(assessment.accepted)


if __name__ == "__main__":
  unittest.main()
