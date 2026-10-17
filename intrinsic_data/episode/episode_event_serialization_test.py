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

"""Serialization, golden, and compatibility tests for EpisodeEvent.

Keep in sync with episode_event_serialization_test.cc.
"""

import os
import unittest

from google.protobuf import message as message_lib
from google.protobuf import text_format
from intrinsic.episode.proto import episode_event_pb2
from intrinsic_data.episode import episode_event_policy as policy

_Error = policy.EpisodeEventError
_Kind = policy.PayloadKind
_Event = episode_event_pb2.EpisodeEvent

# SHA-256 of marine_parameter_sampler_seed_42.golden.txt (1530 bytes).
_PINNED_SEED_42_DIGEST = (
    "93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a"
)
_SEED_42_TEXT_BYTES = 1530
_SNAPSHOT = "6ceb1af0dca7d2b01d4e876fe699fb8c5d5709d5a4938ea3f46e8d355d3b33de"

_PAYLOADS = (
    ("safety_decision", _Kind.SAFETY_DECISION),
    ("vehicle_trajectory", _Kind.VEHICLE_TRAJECTORY),
    ("inference_envelope", _Kind.INFERENCE_ENVELOPE),
    ("inference_result", _Kind.INFERENCE_RESULT),
    ("metadata", _Kind.EPISODE_METADATA),
)

_V2_NAME = "episode_event_v2_additive_unknown_payload.binpb"
_UNKNOWN_PAYLOAD_TAG = 536870000
_UNKNOWN_ENVELOPE_TAG = 536870001

_WIRE_TYPE_VARINT = 0
_WIRE_TYPE_FIXED64 = 1
_WIRE_TYPE_LENGTH_DELIMITED = 2
_WIRE_TYPE_FIXED32 = 5
_EXPECTED_WIRE_TYPES = {
    "uint32": _WIRE_TYPE_VARINT,
    "string": _WIRE_TYPE_LENGTH_DELIMITED,
    "message": _WIRE_TYPE_LENGTH_DELIMITED,
}


def _runfile(*candidates):
  root = os.environ.get("TEST_SRCDIR", "")
  workspace = os.environ.get("TEST_WORKSPACE", "")
  for relative in candidates:
    for prefix in (
        os.path.join(root, workspace),
        os.path.join(root, "_main"),
        os.path.join(root, "intrinsic_apis+"),
        os.path.join(root, workspace, "external", "intrinsic_apis+"),
    ):
      path = os.path.join(prefix, relative)
      if os.path.isfile(path):
        return path
  suffix = os.sep + candidates[0]
  for dirpath, _, filenames in os.walk(root):
    for name in filenames:
      path = os.path.join(dirpath, name)
      if path.endswith(suffix):
        return path
  raise AssertionError("missing %s under %s" % (candidates[0], root))


def _read(path, mode):
  with open(path, mode) as handle:
    return handle.read()


def _testdata_path(name):
  return _runfile("intrinsic_data/episode/testdata/" + name)


def _golden_bytes(payload):
  return _read(_testdata_path("episode_event_v1_%s.binpb" % payload), "rb")


def _golden_text(payload):
  return _read(_testdata_path("episode_event_v1_%s.textproto" % payload), "r")


def _example_text(payload):
  return _read(
      _runfile(
          "intrinsic/episode/proto/examples/episode_event_v1_%s.textproto"
          % payload
      ),
      "r",
  )


def _serialize(message):
  return message.SerializeToString(deterministic=True)


def _parse(data):
  message = _Event()
  message.ParseFromString(data)
  return message


def _varint(value):
  out = bytearray()
  while True:
    byte = value & 0x7F
    value >>= 7
    if value:
      out.append(byte | 0x80)
    else:
      out.append(byte)
      return bytes(out)


def _read_varint(data, offset):
  value = 0
  shift = 0
  while True:
    byte = data[offset]
    offset += 1
    value |= (byte & 0x7F) << shift
    if not byte & 0x80:
      return value, offset
    shift += 7


def _has_unknown_top_level_fields(message):
  """Python upb does not implement UnknownFields(), so scan the wire.

  Only top-level fields count, like the C++ UnknownFieldSet of the message
  itself. A known number with the wrong wire type is unknown to the parser.
  """
  data = message.SerializeToString()
  fields = message.DESCRIPTOR.fields_by_number
  offset = 0
  while offset < len(data):
    key, offset = _read_varint(data, offset)
    number, wire_type = key >> 3, key & 7
    field = fields.get(number)
    if field is None:
      return True
    type_name = {
        field.TYPE_UINT32: "uint32",
        field.TYPE_STRING: "string",
        field.TYPE_MESSAGE: "message",
    }[field.type]
    if _EXPECTED_WIRE_TYPES[type_name] != wire_type:
      return True
    if wire_type == _WIRE_TYPE_VARINT:
      _, offset = _read_varint(data, offset)
    elif wire_type == _WIRE_TYPE_LENGTH_DELIMITED:
      length, offset = _read_varint(data, offset)
      offset += length
    elif wire_type == _WIRE_TYPE_FIXED64:
      offset += 8
    elif wire_type == _WIRE_TYPE_FIXED32:
      offset += 4
    else:
      return True
  return False


def _payload_case(message):
  name = message.WhichOneof("payload")
  if name is None:
    return policy.PAYLOAD_CASE_NOT_SET
  return message.DESCRIPTOR.fields_by_name[name].number


def _view(message):
  header = message.header
  return policy.EpisodeEventView(
      header_present=message.HasField("header"),
      source_id=header.source_id,
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      schema_version=message.schema_version,
      snapshot_id=message.snapshot_id,
      config_digest=message.config_digest,
      provenance_present=message.HasField("provenance"),
      payload_case=_payload_case(message),
      has_unknown_fields=_has_unknown_top_level_fields(message),
      marine_randomization_metadata=(
          message.episode_metadata.marine_randomization_metadata
          if message.HasField("episode_metadata")
          else ""
      ),
  )


def _assess(message):
  return policy.assess_episode_event(_view(message))


def _golden(payload):
  return _parse(_golden_bytes(payload))


class EpisodeEventSerializationTest(unittest.TestCase):

  def test_payload_case_numbers_match_the_generated_schema(self):
    fields = _Event.DESCRIPTOR.fields_by_name
    self.assertEqual(
        fields["safety_decision"].number, policy.PAYLOAD_CASE_SAFETY_DECISION
    )
    self.assertEqual(
        fields["vehicle_trajectory"].number,
        policy.PAYLOAD_CASE_VEHICLE_TRAJECTORY,
    )
    self.assertEqual(
        fields["inference_envelope"].number,
        policy.PAYLOAD_CASE_INFERENCE_ENVELOPE,
    )
    self.assertEqual(
        fields["inference_result"].number,
        policy.PAYLOAD_CASE_INFERENCE_RESULT,
    )
    self.assertEqual(
        fields["episode_metadata"].number,
        policy.PAYLOAD_CASE_EPISODE_METADATA,
    )

  def test_e1_round_trip_per_payload(self):
    for payload, kind in _PAYLOADS:
      with self.subTest(payload=payload):
        data = _golden_bytes(payload)
        message = _parse(data)
        assessment = _assess(message)
        self.assertIs(assessment.error, _Error.NONE)
        self.assertIs(assessment.payload_kind, kind)
        self.assertFalse(assessment.skip)
        self.assertEqual(_serialize(message), data)

  def test_e1_text_fixture_serializes_to_the_golden_bytes(self):
    for payload, _ in _PAYLOADS:
      with self.subTest(payload=payload):
        message = _Event()
        text_format.Parse(_golden_text(payload), message)
        self.assertEqual(_serialize(message), _golden_bytes(payload))

  def test_e1_proto_package_examples_match_the_testdata_text(self):
    for payload, _ in _PAYLOADS:
      with self.subTest(payload=payload):
        self.assertEqual(_example_text(payload), _golden_text(payload))

  def test_e2_metadata_event_preserves_every_field(self):
    message = _parse(_serialize(_golden("metadata")))
    header = message.header
    self.assertEqual(header.sequence, 5)
    self.assertEqual(header.source_time.seconds, 1700000105)
    self.assertEqual(header.source_time.nanos, 250000000)
    self.assertEqual(header.receive_time.seconds, 1700000105)
    self.assertEqual(header.receive_time.nanos, 251000000)
    self.assertEqual(header.source_id, "episode_recorder")
    self.assertEqual(header.frame_id, "world_enu")
    self.assertEqual(header.clock_domain, "monotonic")
    self.assertEqual(header.validity.state, 1)
    self.assertEqual(message.schema_version, 1)
    self.assertEqual(message.snapshot_id, _SNAPSHOT)
    self.assertEqual(message.config_digest, _PINNED_SEED_42_DIGEST)
    self.assertTrue(message.HasField("provenance"))
    self.assertEqual(message.provenance.model_id, "marine_parameter_sampler")
    self.assertEqual(message.provenance.model_version, "v1")
    self.assertEqual(
        message.provenance.digest, "sha256:" + _PINNED_SEED_42_DIGEST
    )
    self.assertEqual(message.WhichOneof("payload"), "episode_metadata")

  def test_e2_unset_provenance_stays_unset(self):
    for payload, _ in _PAYLOADS:
      if payload == "metadata":
        continue
      with self.subTest(payload=payload):
        message = _parse(_serialize(_golden(payload)))
        self.assertFalse(message.HasField("provenance"))

  def test_e3_empty_message_is_not_engaged(self):
    message = _parse(b"")
    self.assertEqual(message.ByteSize(), 0)
    self.assertEqual(_serialize(message), b"")
    assessment = _assess(message)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertFalse(assessment.engaged)
    self.assertFalse(assessment.skip)

  def test_e3_single_event_is_accepted(self):
    assessment = _assess(_golden("safety_decision"))
    self.assertIs(assessment.error, _Error.NONE)
    self.assertTrue(assessment.engaged)

  def test_e4_every_error_row(self):
    def mutated(edit, payload="safety_decision"):
      message = _golden(payload)
      edit(message)
      return _assess(message).error

    def set_nanos(field, nanos):
      def edit(message):
        getattr(message.header, field).nanos = nanos

      return edit

    self.assertIs(
        mutated(lambda m: m.ClearField("header")), _Error.MISSING_HEADER
    )
    self.assertIs(
        mutated(lambda m: setattr(m.header, "source_id", "")), _Error.SOURCE_ID
    )
    self.assertIs(
        mutated(lambda m: setattr(m.header, "frame_id", "")),
        _Error.MISSING_FRAME,
    )
    self.assertIs(
        mutated(lambda m: m.header.ClearField("source_time")),
        _Error.SOURCE_TIME,
    )
    self.assertIs(
        mutated(lambda m: m.header.ClearField("receive_time")),
        _Error.RECEIVE_TIME,
    )
    for nanos in (-1, 1000000000):
      with self.subTest(nanos=nanos):
        self.assertIs(
            mutated(set_nanos("source_time", nanos)), _Error.SOURCE_TIME
        )
        self.assertIs(
            mutated(set_nanos("receive_time", nanos)), _Error.RECEIVE_TIME
        )
    self.assertIs(
        mutated(lambda m: setattr(m, "schema_version", 0)),
        _Error.SCHEMA_VERSION,
    )
    bad_hex = (_SNAPSHOT.upper(), _SNAPSHOT[:63], _SNAPSHOT + "a", "g" * 64)
    for bad in bad_hex:
      with self.subTest(bad=bad):
        self.assertIs(
            mutated(lambda m: setattr(m, "snapshot_id", bad)),
            _Error.SNAPSHOT_ID,
        )
        self.assertIs(
            mutated(lambda m: setattr(m, "config_digest", bad)),
            _Error.CONFIG_DIGEST,
        )
    self.assertIs(
        mutated(lambda m: m.ClearField("payload")), _Error.MISSING_PAYLOAD
    )

  def test_e4_metadata_digest_mismatch(self):
    def mutated(edit):
      message = _golden("metadata")
      edit(message)
      return _assess(message).error

    def flip_first_digit(message):
      text = message.episode_metadata.marine_randomization_metadata
      index = text.index("seed=42") + len("seed=")
      message.episode_metadata.marine_randomization_metadata = (
          text[:index] + "3" + text[index + 1 :]
      )

    self.assertIs(mutated(flip_first_digit), _Error.CONFIG_DIGEST_MISMATCH)
    self.assertIs(
        mutated(lambda m: setattr(m, "config_digest", "")),
        _Error.CONFIG_DIGEST_MISMATCH,
    )
    self.assertIs(
        mutated(
            lambda m: setattr(
                m.episode_metadata, "marine_randomization_metadata", ""
            )
        ),
        _Error.CONFIG_DIGEST_MISMATCH,
    )

  def test_e5_truncated_bytes_report_a_parse_failure(self):
    for payload, _ in _PAYLOADS:
      data = _golden_bytes(payload)
      failures = 0
      for cut in range(1, len(data) + 1):
        message = _Event()
        try:
          message.ParseFromString(data[: len(data) - cut])
        except message_lib.DecodeError:
          failures += 1
          continue
        _assess(message)
      with self.subTest(payload=payload):
        self.assertGreater(failures, 0)
        with self.assertRaises(message_lib.DecodeError):
          _Event().ParseFromString(data[:-1])

  def test_e6_v2_additive_golden_is_skipped_and_preserved(self):
    data = _read(_testdata_path(_V2_NAME), "rb")
    message = _parse(data)
    view = _view(message)
    self.assertEqual(view.payload_case, policy.PAYLOAD_CASE_NOT_SET)
    self.assertTrue(view.has_unknown_fields)
    assessment = policy.assess_episode_event(view)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.UNKNOWN)
    self.assertTrue(assessment.skip)
    self.assertEqual(message.SerializeToString(), data)
    self.assertEqual(_serialize(message), data)

  def test_e6_v2_golden_is_the_v1_metadata_envelope_plus_unknown_fields(self):
    expected = _golden("metadata")
    expected.ClearField("episode_metadata")
    data = _serialize(expected)
    data += _varint((_UNKNOWN_PAYLOAD_TAG << 3) | 2)
    data += _varint(len(b"future_payload")) + b"future_payload"
    data += _varint((_UNKNOWN_ENVELOPE_TAG << 3) | 0) + _varint(7)
    self.assertEqual(_read(_testdata_path(_V2_NAME), "rb"), data)

  def test_e6_unknown_envelope_field_keeps_a_known_payload(self):
    data = _golden_bytes("metadata")
    data += _varint((_UNKNOWN_ENVELOPE_TAG << 3) | 0) + _varint(7)
    message = _parse(data)
    assessment = _assess(message)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.EPISODE_METADATA)
    self.assertFalse(assessment.skip)
    self.assertEqual(message.SerializeToString(), data)

  def test_e7_v1_goldens_parse_identically_from_text_and_bytes(self):
    for payload, _ in _PAYLOADS:
      with self.subTest(payload=payload):
        from_text = _Event()
        text_format.Parse(_golden_text(payload), from_text)
        self.assertEqual(from_text, _golden(payload))

  def test_e7_newer_schema_version_on_a_v1_payload_is_accepted(self):
    message = _golden("safety_decision")
    message.schema_version = 2
    reparsed = _parse(_serialize(message))
    self.assertIs(_assess(reparsed).error, _Error.NONE)
    self.assertEqual(reparsed.schema_version, 2)

  def test_e8_pinned_digest_of_the_seed_42_text(self):
    text = _golden("metadata").episode_metadata.marine_randomization_metadata
    self.assertEqual(len(text.encode("ascii")), _SEED_42_TEXT_BYTES)
    self.assertTrue(text.startswith("marine_randomization_metadata_v1\n"))
    self.assertEqual(policy.compute_config_digest(text), _PINNED_SEED_42_DIGEST)
    self.assertEqual(_golden("metadata").config_digest, _PINNED_SEED_42_DIGEST)

  def test_e8_one_byte_text_change_is_a_mismatch(self):
    message = _golden("metadata")
    text = message.episode_metadata.marine_randomization_metadata
    message.episode_metadata.marine_randomization_metadata = text[:-1] + "x"
    self.assertIs(_assess(message).error, _Error.CONFIG_DIGEST_MISMATCH)
    message.episode_metadata.marine_randomization_metadata = text + "\n"
    self.assertIs(_assess(message).error, _Error.CONFIG_DIGEST_MISMATCH)


if __name__ == "__main__":
  unittest.main()
