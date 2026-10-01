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

"""Serialization and textproto tests for forward-looking sonar frames."""

import math
import os
import unittest

from google.protobuf import text_format

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.perception.proto.sonar import fls_frame_pb2
from intrinsic.perception.sonar import fls_frame_contract_policy as policy

_Error = policy.FlsFrameContractError
_Kind = stamped_header_policy.ValidityKind

# Canonical serialization of _fill_inline_nominal(), which is also
# examples/fls_frame_inline_nominal.textproto. Keep in sync with
# fls_frame_contract_serialization_test.cc.
_INLINE_NOMINAL_GOLDEN_HEX = (
    "0a380815120b0894e2cfaa061080e59a771a060895e2cfaa062205666c735f302a09666c735f"
    "736f6e617232096d6f6e6f746f6e69633a02080112280804100819000000000000e03f210000"
    "00000000444029000000000000e0bf31000000000000e03f1a320a04626f64791209666c735f"
    "736f6e61721a1f0a1209000000000000f83f19000000000000d0bf120921000000000000f03f"
    "2100000000007097402a83010a8001000000000000003e0000803e0000c03e0000003f000020"
    "3f0000403f0000603f0000803f0000903f0000a03f0000b03f0000c03f0000d03f0000e03f00"
    "00f03f0000004000000840000010400000184000002040000028400000304000003840000040"
    "4000004840000050400000584000006040000068400000704000007840a206160a0b736f6e61"
    "725f6d6f64656c120766697874757265"
)

_FRAME = fls_frame_pb2.ForwardLookingSonarFrame


def _fill_header(header, frame_id="fls_sonar"):
  header.sequence = 21
  header.source_time.seconds = 1700000020
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000021
  header.source_id = "fls_0"
  header.frame_id = frame_id
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID


def _fill_geometry(geometry):
  geometry.num_beams = 4
  geometry.num_range_bins = 8
  geometry.min_range_m = 0.5
  geometry.max_range_m = 40
  geometry.min_bearing_rad = -0.5
  geometry.max_bearing_rad = 0.5


def _fill_calibration(calibration):
  calibration.parent_frame_id = "body"
  calibration.sensor_frame_id = "fls_sonar"
  calibration.pose_parent_from_sensor.position.x = 1.5
  calibration.pose_parent_from_sensor.position.z = -0.25
  calibration.pose_parent_from_sensor.orientation.w = 1


def _fill_inline_nominal():
  frame = _FRAME()
  _fill_header(frame.header)
  _fill_geometry(frame.geometry)
  _fill_calibration(frame.calibration)
  frame.sound_speed_m_s = 1500
  frame.inline_samples.intensity.extend(0.125 * i for i in range(32))
  frame.metadata["sonar_model"] = "fixture"
  return frame


def _fill_blob_nominal():
  frame = _FRAME()
  _fill_header(frame.header)
  _fill_geometry(frame.geometry)
  _fill_calibration(frame.calibration)
  frame.sound_speed_m_s = 1500
  frame.blob_reference.blob_id = "fls-blob-0001"
  frame.blob_reference.content_type = "application/octet-stream"
  frame.blob_reference.byte_size = 128
  return frame


def _view_of(message):
  payload = message.WhichOneof("payload")
  if payload == "inline_samples":
    mode = policy.FlsPayloadMode.INLINE
  elif payload == "blob_reference":
    mode = policy.FlsPayloadMode.BLOB
  else:
    mode = policy.FlsPayloadMode.NONE
  geometry = message.geometry
  calibration = message.calibration
  pose = calibration.pose_parent_from_sensor
  blob = message.blob_reference
  return policy.FlsFrameView(
      header_present=message.HasField("header"),
      validity_present=message.header.HasField("validity"),
      validity_state=message.header.validity.state,
      frame_id=message.header.frame_id,
      geometry=policy.FlsGeometryView(
          num_beams=geometry.num_beams,
          num_range_bins=geometry.num_range_bins,
          min_range_m=geometry.min_range_m,
          max_range_m=geometry.max_range_m,
          min_bearing_rad=geometry.min_bearing_rad,
          max_bearing_rad=geometry.max_bearing_rad,
      ),
      calibration=policy.FlsCalibrationView(
          present=message.HasField("calibration"),
          parent_frame_id=calibration.parent_frame_id,
          sensor_frame_id=calibration.sensor_frame_id,
          pose_present=calibration.HasField("pose_parent_from_sensor"),
          position=(pose.position.x, pose.position.y, pose.position.z),
          orientation=(
              pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w,
          ),
      ),
      sound_speed_m_s=message.sound_speed_m_s,
      payload_mode=mode,
      intensity=tuple(message.inline_samples.intensity),
      blob_id=blob.blob_id,
      blob_byte_size_present=blob.HasField("byte_size"),
      blob_byte_size=blob.byte_size,
      metadata_present=len(message.metadata) > 0,
  )


def _assess(message):
  return policy.assess_forward_looking_sonar_frame(_view_of(message))


def _round_trip(message):
  parsed = _FRAME()
  parsed.ParseFromString(message.SerializeToString())
  return parsed


def _load_example(name):
  root = os.environ.get("TEST_SRCDIR", "")
  matches = []
  for dirpath, _, filenames in os.walk(root):
    if name in filenames:
      matches.append(os.path.join(dirpath, name))
  if not matches:
    raise AssertionError("missing %s under %s" % (name, root))
  matches.sort(key=lambda path: ("examples" not in path, len(path)))
  with open(matches[0], encoding="utf-8") as handle:
    return handle.read()


def _parse_example(name):
  parsed = _FRAME()
  text_format.Parse(_load_example(name), parsed)
  return parsed


class FlsFrameContractSerializationTest(unittest.TestCase):

  def test_package_and_locked_field_numbers(self):
    descriptor = _FRAME.DESCRIPTOR
    self.assertEqual(
        descriptor.file.package, "intrinsic_proto.perception.sonar"
    )
    self.assertEqual(
        {field.name: field.number for field in descriptor.fields},
        {
            "header": 1,
            "geometry": 2,
            "calibration": 3,
            "sound_speed_m_s": 4,
            "inline_samples": 5,
            "blob_reference": 6,
            "metadata": 100,
        },
    )
    self.assertEqual(
        [field.name for field in descriptor.oneofs_by_name["payload"].fields],
        ["inline_samples", "blob_reference"],
    )
    messages = descriptor.file.message_types_by_name
    numbers = lambda name: {
        field.name: field.number for field in messages[name].fields
    }
    self.assertEqual(
        numbers("FlsGeometry"),
        {
            "num_beams": 1,
            "num_range_bins": 2,
            "min_range_m": 3,
            "max_range_m": 4,
            "min_bearing_rad": 5,
            "max_bearing_rad": 6,
        },
    )
    self.assertEqual(
        numbers("FlsCalibration"),
        {
            "parent_frame_id": 1,
            "sensor_frame_id": 2,
            "pose_parent_from_sensor": 3,
        },
    )
    self.assertEqual(numbers("FlsInlineSamples"), {"intensity": 1})
    self.assertEqual(
        numbers("SonarPayloadBlobReference"),
        {"blob_id": 1, "content_type": 2, "byte_size": 3},
    )

  def test_scope_is_fls_only(self):
    descriptor = _FRAME.DESCRIPTOR
    self.assertEqual(
        set(descriptor.file.message_types_by_name),
        {
            "FlsGeometry",
            "FlsCalibration",
            "FlsInlineSamples",
            "SonarPayloadBlobReference",
            "ForwardLookingSonarFrame",
        },
    )
    self.assertEqual(len(descriptor.file.enum_types_by_name), 0)
    self.assertIsNone(descriptor.fields_by_name.get("robot_type"))

  def test_default_is_zero_bytes_and_not_accepted(self):
    empty = _FRAME()
    self.assertEqual(empty.SerializeToString(), b"")
    self.assertEqual(empty.ByteSize(), 0)
    self.assertFalse(empty.HasField("header"))
    self.assertFalse(empty.HasField("geometry"))
    self.assertFalse(empty.HasField("calibration"))
    self.assertIsNone(empty.WhichOneof("payload"))
    self.assertEqual(empty.sound_speed_m_s, 0)
    self.assertEqual(len(empty.metadata), 0)
    parsed = _FRAME()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    assessment = _assess(parsed)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_golden_round_trip(self):
    frame = _fill_inline_nominal()
    golden = bytes.fromhex(_INLINE_NOMINAL_GOLDEN_HEX)
    self.assertEqual(frame.SerializeToString().hex(), golden.hex())
    parsed = _FRAME()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.SerializeToString(), golden)
    self.assertEqual(parsed.header.frame_id, "fls_sonar")
    self.assertEqual(parsed.geometry.num_beams, 4)
    self.assertEqual(parsed.geometry.num_range_bins, 8)
    self.assertEqual(parsed.sound_speed_m_s, 1500)
    self.assertEqual(parsed.WhichOneof("payload"), "inline_samples")
    self.assertEqual(len(parsed.inline_samples.intensity), 32)
    self.assertEqual(parsed.inline_samples.intensity[31], 3.875)
    self.assertEqual(parsed.metadata["sonar_model"], "fixture")
    assessment = _assess(parsed)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_blob_round_trip(self):
    frame = _fill_blob_nominal()
    parsed = _round_trip(frame)
    self.assertEqual(parsed.SerializeToString(), frame.SerializeToString())
    self.assertEqual(parsed.WhichOneof("payload"), "blob_reference")
    self.assertEqual(len(parsed.inline_samples.intensity), 0)
    self.assertTrue(_assess(parsed).accepted)

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_INLINE_NOMINAL_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA8, 0x06, 0x07))
    parsed = _FRAME()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.header.frame_id, "fls_sonar")
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    self.assertTrue(_assess(parsed).accepted)

  def test_unknown_field_in_nested_message_is_preserved(self):
    geometry = fls_frame_pb2.FlsGeometry()
    _fill_geometry(geometry)
    nested = geometry.SerializeToString() + bytes((0x78, 0x01))
    raw = bytes((0x12, len(nested))) + nested
    parsed = _FRAME()
    parsed.ParseFromString(raw)
    self.assertEqual(parsed.geometry.num_beams, 4)
    self.assertEqual(parsed.SerializeToString(), raw)

  def test_metadata_shuffle_stays_accepted(self):
    first = _fill_inline_nominal()
    first.metadata["a"] = "1"
    first.metadata["note"] = ""
    second = _fill_inline_nominal()
    second.metadata["note"] = ""
    second.metadata["a"] = "1"
    self.assertEqual(dict(first.metadata), dict(second.metadata))
    first_assessment = _assess(first)
    second_assessment = _assess(second)
    self.assertTrue(first_assessment.accepted)
    self.assertEqual(first_assessment, second_assessment)
    first.metadata[""] = "nan"
    self.assertTrue(_assess(first).accepted)

  def test_metadata_alone_is_engaged_and_rejected(self):
    frame = _FRAME()
    frame.metadata["k"] = "v"
    assessment = _assess(frame)
    self.assertIs(assessment.error, _Error.MISSING_FRAME)
    self.assertFalse(assessment.accepted)

  def test_payload_oneof_switches_modes(self):
    frame = _fill_inline_nominal()
    frame.blob_reference.blob_id = "fls-blob-0001"
    self.assertEqual(frame.WhichOneof("payload"), "blob_reference")
    self.assertFalse(frame.HasField("inline_samples"))
    self.assertTrue(_assess(frame).accepted)
    frame.inline_samples.intensity.append(0.0)
    self.assertEqual(frame.WhichOneof("payload"), "inline_samples")
    self.assertFalse(frame.HasField("blob_reference"))
    self.assertIs(_assess(frame).error, _Error.PAYLOAD_MISMATCH)

  def test_selected_empty_arms_survive_the_wire(self):
    frame = _fill_inline_nominal()
    frame.inline_samples.SetInParent()
    frame.inline_samples.ClearField("intensity")
    parsed = _round_trip(frame)
    self.assertEqual(parsed.WhichOneof("payload"), "inline_samples")
    self.assertIs(_assess(parsed).error, _Error.PAYLOAD_MISMATCH)
    frame = _fill_blob_nominal()
    frame.blob_reference.Clear()
    frame.blob_reference.SetInParent()
    parsed = _round_trip(frame)
    self.assertEqual(parsed.WhichOneof("payload"), "blob_reference")
    self.assertIs(_assess(parsed).error, _Error.BLOB_REFERENCE)

  def test_missing_payload_after_the_wire(self):
    frame = _fill_inline_nominal()
    frame.ClearField("payload")
    self.assertIs(_assess(_round_trip(frame)).error, _Error.MISSING_PAYLOAD)

  def test_blob_byte_size_presence(self):
    frame = _fill_blob_nominal()
    frame.blob_reference.ClearField("byte_size")
    parsed = _round_trip(frame)
    self.assertFalse(parsed.blob_reference.HasField("byte_size"))
    self.assertTrue(_assess(parsed).accepted)
    frame.blob_reference.byte_size = 0
    zero = _round_trip(frame)
    self.assertTrue(zero.blob_reference.HasField("byte_size"))
    self.assertNotEqual(zero.SerializeToString(), parsed.SerializeToString())
    self.assertIs(_assess(zero).error, _Error.BLOB_REFERENCE)
    frame.blob_reference.content_type = ""
    frame.blob_reference.byte_size = 1
    self.assertTrue(_assess(_round_trip(frame)).accepted)

  def test_sound_speed_survives_the_wire(self):
    for value in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
      frame = _fill_blob_nominal()
      frame.sound_speed_m_s = value
      parsed = _round_trip(frame)
      error = _assess(parsed).error
      self.assertIs(error, _Error.SOUND_SPEED, msg=str(value))

  def test_non_finite_sample_survives_the_wire(self):
    for value in (float("nan"), float("inf"), float("-inf")):
      frame = _fill_inline_nominal()
      frame.inline_samples.intensity[7] = value
      parsed = _round_trip(frame)
      self.assertFalse(math.isfinite(parsed.inline_samples.intensity[7]))
      self.assertIs(_assess(parsed).error, _Error.NON_FINITE)

  def test_missing_calibration_pose_is_a_calibration_defect(self):
    frame = _fill_blob_nominal()
    frame.calibration.ClearField("pose_parent_from_sensor")
    self.assertTrue(frame.HasField("calibration"))
    self.assertIs(_assess(_round_trip(frame)).error, _Error.CALIBRATION)

  def test_calibration_zero_quaternion_on_the_wire(self):
    frame = _fill_blob_nominal()
    frame.calibration.pose_parent_from_sensor.orientation.Clear()
    parsed = _round_trip(frame)
    self.assertTrue(parsed.calibration.HasField("pose_parent_from_sensor"))
    self.assertIs(_assess(parsed).error, _Error.QUATERNION)

  def test_validity_distinctions_survive_the_wire(self):
    frame = _fill_blob_nominal()
    frame.header.ClearField("validity")
    absent = _assess(_round_trip(frame))
    self.assertIs(absent.validity, _Kind.ABSENT)
    self.assertFalse(absent.accepted)
    frame.header.validity.state = stamped_header_pb2.Validity.STATE_INVALID
    invalid = _assess(_round_trip(frame))
    self.assertIs(invalid.validity, _Kind.INVALID)
    self.assertIs(invalid.error, _Error.NONE)
    self.assertFalse(invalid.accepted)
    frame.header.validity.state = stamped_header_pb2.Validity.STATE_UNSPECIFIED
    frame.header.validity.SetInParent()
    unspecified = _assess(_round_trip(frame))
    self.assertIs(unspecified.validity, _Kind.UNSPECIFIED)
    self.assertFalse(unspecified.accepted)

  def test_valid_textproto_fixtures(self):
    inline = _parse_example("fls_frame_inline_nominal.textproto")
    self.assertEqual(
        inline.SerializeToString(),
        _fill_inline_nominal().SerializeToString(),
    )
    self.assertEqual(
        inline.SerializeToString(),
        bytes.fromhex(_INLINE_NOMINAL_GOLDEN_HEX),
    )
    self.assertEqual(inline.WhichOneof("payload"), "inline_samples")
    self.assertEqual(len(inline.inline_samples.intensity), 32)
    self.assertEqual(inline.sound_speed_m_s, 1500)
    self.assertTrue(inline.HasField("calibration"))
    self.assertEqual(inline.metadata["sonar_model"], "fixture")
    blob = _parse_example("fls_frame_blob_nominal.textproto")
    self.assertEqual(
        blob.SerializeToString(), _fill_blob_nominal().SerializeToString()
    )
    self.assertEqual(blob.WhichOneof("payload"), "blob_reference")
    self.assertEqual(blob.geometry, inline.geometry)
    self.assertEqual(blob.blob_reference.blob_id, "fls-blob-0001")
    self.assertEqual(len(blob.inline_samples.intensity), 0)
    for frame in (inline, blob):
      assessment = _assess(frame)
      self.assertIs(assessment.error, _Error.NONE)
      self.assertIs(assessment.validity, _Kind.VALID)
      self.assertTrue(assessment.accepted)

  def test_invalid_textproto_fixtures(self):
    cases = (
        ("fls_frame_bad_sound_speed.textproto", _Error.SOUND_SPEED),
        ("fls_frame_payload_mismatch.textproto", _Error.PAYLOAD_MISMATCH),
        ("fls_frame_bad_frame.textproto", _Error.MISSING_FRAME),
        ("fls_frame_empty_blob_id.textproto", _Error.BLOB_REFERENCE),
    )
    for name, error in cases:
      with self.subTest(name=name):
        assessment = _assess(_parse_example(name))
        self.assertIs(assessment.error, error)
        self.assertIs(assessment.validity, _Kind.VALID)
        self.assertFalse(assessment.accepted)

  def test_invalid_fixtures_differ_from_nominal_by_one_defect(self):
    bad_sound_speed = _parse_example("fls_frame_bad_sound_speed.textproto")
    self.assertLessEqual(bad_sound_speed.sound_speed_m_s, 0)
    bad_frame = _parse_example("fls_frame_bad_frame.textproto")
    self.assertEqual(bad_frame.header.frame_id, "")
    mismatch = _parse_example("fls_frame_payload_mismatch.textproto")
    geometry = mismatch.geometry
    self.assertNotEqual(
        len(mismatch.inline_samples.intensity),
        geometry.num_beams * geometry.num_range_bins,
    )
    empty_blob = _parse_example("fls_frame_empty_blob_id.textproto")
    self.assertEqual(empty_blob.WhichOneof("payload"), "blob_reference")
    self.assertEqual(empty_blob.blob_reference.blob_id, "")


if __name__ == "__main__":
  unittest.main()
