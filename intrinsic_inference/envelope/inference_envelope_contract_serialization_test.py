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

"""Serialization and textproto tests for the inference envelope."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.embodiment import stamped_header_policy
from intrinsic.inference.proto import inference_envelope_pb2
from intrinsic_inference.envelope import inference_envelope_contract_policy as policy

_Error = policy.InferenceEnvelopeContractError
_Kind = stamped_header_policy.ValidityKind
_Envelope = inference_envelope_pb2.InferenceEnvelope
_Result = inference_envelope_pb2.InferenceResult

# Canonical serialization of examples/inference_envelope_nominal.textproto.
# Keep in sync with inference_envelope_contract_serialization_test.cc.
_NOMINAL_ENVELOPE_GOLDEN_HEX = (
    "0a3b081512060880e2cfaa061a0a0880e2cfaa0610c0843d2209706c616e6e6572"
    "5f302a09776f726c645f656e7532096d6f6e6f746f6e69633a020801121f0a130a"
    "0e706f73655f657374696d61746f7212013312087265712d30303031182a224030"
    "313233343536373839616263646566303132333435363738396162636465663031"
    "3233343536373839616263646566303132333435363738396162636465662a0c08"
    "80e2cfaa061080cab5ee013202080239cdccccccccccec3f419a9999999999a93f"
    "4a477368613235363a616261626162616261626162616261626162616261626162"
    "616261626162616261626162616261626162616261626162616261626162616261"
    "62616261626162525c0a0e706f73655f657374696d61746f721201331a47736861"
    "3235363a6566656665666566656665666566656665666566656665666566656665"
    "666566656665666566656665666566656665666566656665666566656665666566"
    "6566a206130a0870726f6475636572120766697874757265"
)


def _hex_to_bytes(text):
  return bytes.fromhex("".join(text.split()))


def _common_view(message, input_digest, provenance):
  header = message.header
  return policy.InferenceCommonView(
      header_present=message.HasField("header"),
      validity_present=header.HasField("validity"),
      validity_state=header.validity.state,
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      state_epoch=message.state_epoch,
      world_snapshot_id=message.world_snapshot_id,
      deadline_present=message.HasField("deadline"),
      deadline=(message.deadline.seconds, message.deadline.nanos),
      validity_horizon_present=message.HasField("validity_horizon"),
      validity_horizon=(
          message.validity_horizon.seconds,
          message.validity_horizon.nanos,
      ),
      confidence_present=message.HasField("confidence"),
      confidence=message.confidence,
      uncertainty_present=message.HasField("uncertainty"),
      uncertainty=message.uncertainty,
      input_digest=input_digest,
      provenance_present=message.HasField("provenance"),
      provenance_model_id=provenance.model_id,
      metadata_present=len(message.metadata) > 0,
  )


def _oip_view(message, field):
  oip = getattr(message, field)
  return policy.OipIdentifierView(
      message_set=message.HasField(field),
      model_name=oip.model.model_name,
      model_version=oip.model.model_version,
      request_id=oip.request_id,
  )


def _envelope_view(message):
  return policy.InferenceEnvelopeView(
      common=_common_view(message, message.input_digest, message.provenance),
      oip_request=_oip_view(message, "oip_request"),
  )


def _result_view(message):
  return policy.InferenceResultView(
      common=_common_view(message, message.input_digest, message.provenance),
      oip_result=_oip_view(message, "oip_result"),
      output_digest=message.output_digest,
  )


def _assess_envelope(message):
  return policy.assess_inference_envelope(_envelope_view(message))


def _assess_result(message):
  return policy.assess_inference_result(_result_view(message))


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


def _parse(message_type, name):
  parsed = message_type()
  text_format.Parse(_load_example(name), parsed)
  return parsed


def _parse_envelope(name):
  return _parse(_Envelope, name)


class InferenceEnvelopeFixtureTest(unittest.TestCase):

  def test_nominal_envelope_is_accepted(self):
    assessment = _assess_envelope(
        _parse_envelope("inference_envelope_nominal.textproto")
    )
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_nominal_result_is_accepted(self):
    assessment = _assess_result(
        _parse(_Result, "inference_result_nominal.textproto")
    )
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_deadline_equal_creation_is_accepted(self):
    message = _parse_envelope(
        "inference_envelope_deadline_equal_creation.textproto"
    )
    self.assertEqual(
        (message.deadline.seconds, message.deadline.nanos),
        (message.header.source_time.seconds, message.header.source_time.nanos),
    )
    assessment = _assess_envelope(message)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)

  def test_invalid_fixtures_report_locked_errors(self):
    cases = (
        ("inference_envelope_deadline_before_creation", _Error.DEADLINE),
        ("inference_envelope_bad_confidence", _Error.CONFIDENCE),
        ("inference_envelope_bad_uncertainty", _Error.UNCERTAINTY),
        ("inference_envelope_bad_snapshot_id", _Error.SNAPSHOT_ID),
        ("inference_envelope_empty_oip_request_id", _Error.OIP_IDENTIFIER),
    )
    for name, expected in cases:
      with self.subTest(name=name):
        assessment = _assess_envelope(_parse_envelope(name + ".textproto"))
        self.assertIs(assessment.error, expected)
        # STATE_VALID does not repair a structural defect.
        self.assertIs(assessment.validity, _Kind.VALID)
        self.assertFalse(assessment.accepted)

  def test_fixtures_survive_a_wire_round_trip(self):
    for name in (
        "inference_envelope_nominal",
        "inference_envelope_deadline_equal_creation",
        "inference_envelope_deadline_before_creation",
        "inference_envelope_bad_confidence",
        "inference_envelope_bad_uncertainty",
        "inference_envelope_bad_snapshot_id",
        "inference_envelope_empty_oip_request_id",
    ):
      with self.subTest(name=name):
        original = _parse_envelope(name + ".textproto")
        reparsed = _Envelope.FromString(original.SerializeToString())
        self.assertEqual(original, reparsed)
        self.assertEqual(_assess_envelope(original), _assess_envelope(reparsed))


class InferenceEnvelopeWireTest(unittest.TestCase):

  def test_locked_field_numbers(self):
    envelope = {f.name: f.number for f in _Envelope.DESCRIPTOR.fields}
    self.assertEqual(
        envelope,
        {
            "header": 1,
            "oip_request": 2,
            "state_epoch": 3,
            "world_snapshot_id": 4,
            "deadline": 5,
            "validity_horizon": 6,
            "confidence": 7,
            "uncertainty": 8,
            "input_digest": 9,
            "provenance": 10,
            "metadata": 100,
        },
    )
    result = {f.name: f.number for f in _Result.DESCRIPTOR.fields}
    self.assertEqual(
        result,
        {
            "header": 1,
            "oip_result": 2,
            "state_epoch": 3,
            "world_snapshot_id": 4,
            "deadline": 5,
            "validity_horizon": 6,
            "confidence": 7,
            "uncertainty": 8,
            "input_digest": 9,
            "output_digest": 10,
            "provenance": 11,
            "metadata": 100,
        },
    )
    for message_type in (
        inference_envelope_pb2.OipRequestIdentifier,
        inference_envelope_pb2.OipResultIdentifier,
    ):
      self.assertEqual(
          {f.name: f.number for f in message_type.DESCRIPTOR.fields},
          {"model": 1, "request_id": 2},
      )
    self.assertEqual(
        {
            f.name: f.number
            for f in inference_envelope_pb2.OipModelIdentifier.DESCRIPTOR.fields
        },
        {"model_name": 1, "model_version": 2},
    )

  def test_package_and_imports_exclude_oip_payload(self):
    descriptor = _Envelope.DESCRIPTOR.file
    self.assertEqual(descriptor.package, "intrinsic_proto.inference")
    self.assertCountEqual(
        [dep.name for dep in descriptor.dependencies],
        [
            "google/protobuf/duration.proto",
            "google/protobuf/timestamp.proto",
            "intrinsic/embodiment/proto/model_provenance.proto",
            "intrinsic/embodiment/proto/stamped_header.proto",
        ],
    )

  def test_default_messages_serialize_to_zero_bytes(self):
    self.assertEqual(_Envelope().SerializeToString(), b"")
    self.assertEqual(_Result().SerializeToString(), b"")
    self.assertEqual(inference_envelope_pb2.OipModelIdentifier().ByteSize(), 0)
    self.assertEqual(
        inference_envelope_pb2.OipRequestIdentifier().ByteSize(), 0
    )
    self.assertEqual(inference_envelope_pb2.OipResultIdentifier().ByteSize(), 0)

  def test_default_messages_are_not_engaged(self):
    self.assertFalse(_assess_envelope(_Envelope()).accepted)
    self.assertIs(_assess_envelope(_Envelope()).error, _Error.NONE)
    self.assertFalse(_assess_result(_Result()).accepted)
    self.assertIs(_assess_result(_Result()).error, _Error.NONE)

  def test_optional_doubles_track_presence(self):
    message = _Envelope()
    self.assertFalse(message.HasField("confidence"))
    self.assertFalse(message.HasField("uncertainty"))
    message.confidence = 0.0
    message.uncertainty = 0.0
    self.assertTrue(message.HasField("confidence"))
    self.assertTrue(message.HasField("uncertainty"))
    # Tag plus fixed64 each. A set zero is not an unset field.
    self.assertEqual(message.ByteSize(), 18)
    reparsed = _Envelope.FromString(message.SerializeToString())
    self.assertTrue(reparsed.HasField("confidence"))
    self.assertTrue(reparsed.HasField("uncertainty"))
    reparsed.ClearField("confidence")
    self.assertFalse(reparsed.HasField("confidence"))

  def test_unknown_fields_round_trip(self):
    original = _parse_envelope("inference_envelope_nominal.textproto")
    # Field 900 varint = 7, then field 901 length-delimited = "ext".
    unknown = bytes.fromhex("a03807" "aa3803657874")
    wire = original.SerializeToString() + unknown
    parsed = _Envelope.FromString(wire)
    self.assertEqual(parsed.input_digest, original.input_digest)
    self.assertEqual(parsed.SerializeToString(), wire)
    self.assertTrue(_assess_envelope(parsed).accepted)

  def test_unknown_fields_in_nested_messages_round_trip(self):
    original = _parse_envelope("inference_envelope_nominal.textproto")
    nested = _Envelope.FromString(original.SerializeToString())
    nested.oip_request.model.MergeFromString(bytes.fromhex("a03807"))
    nested.provenance.MergeFromString(bytes.fromhex("b03809"))
    wire = nested.SerializeToString()
    reparsed = _Envelope.FromString(wire)
    self.assertEqual(reparsed.SerializeToString(), wire)
    self.assertTrue(_assess_envelope(reparsed).accepted)

  def test_unknown_fields_round_trip_on_result(self):
    original = _parse(_Result, "inference_result_nominal.textproto")
    wire = original.SerializeToString() + bytes.fromhex("a03807")
    parsed = _Result.FromString(wire)
    self.assertEqual(parsed.SerializeToString(), wire)
    self.assertTrue(_assess_result(parsed).accepted)

  def test_metadata_round_trips(self):
    original = _parse_envelope("inference_envelope_nominal.textproto")
    original.metadata["second"] = "value"
    reparsed = _Envelope.FromString(original.SerializeToString())
    self.assertEqual(dict(reparsed.metadata), dict(original.metadata))
    self.assertTrue(_assess_envelope(reparsed).accepted)

  def test_metadata_alone_engages(self):
    message = _Envelope()
    message.metadata["k"] = "v"
    assessment = _assess_envelope(message)
    self.assertIs(assessment.error, _Error.MISSING_FRAME)
    self.assertFalse(assessment.accepted)

  def test_nominal_envelope_golden_bytes(self):
    message = _parse_envelope("inference_envelope_nominal.textproto")
    self.assertEqual(
        message.SerializeToString(deterministic=True).hex(),
        _NOMINAL_ENVELOPE_GOLDEN_HEX,
    )
    parsed = _Envelope.FromString(_hex_to_bytes(_NOMINAL_ENVELOPE_GOLDEN_HEX))
    self.assertEqual(parsed, message)


if __name__ == "__main__":
  unittest.main()
