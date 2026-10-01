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

"""Serialization and textproto tests for multimodal observation bundles."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.perception.proto.sonar import observation_bundle_pb2
from intrinsic.perception.sonar import observation_bundle_builder as builder
from intrinsic.perception.sonar import observation_bundle_contract_policy as policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2

_Error = policy.ObservationBundleContractError
_Kind = stamped_header_policy.ValidityKind
_Bundle = observation_bundle_pb2.MultimodalObservationBundle
_Modality = observation_bundle_pb2.ObservationModality
_Reason = observation_bundle_pb2.AbsentModalityReason

# Canonical serialization of examples/observation_bundle_sonar_only.textproto.
# Keep in sync with observation_bundle_contract_serialization_test.cc.
_SONAR_ONLY_GOLDEN_HEX = (
    "0a36080712060880e2cfaa061a0b0880e2cfaa061080c2d72f220862756e646c655f"
    "302a0473796e6332096d6f6e6f746f6e69633a0208011205108084af5f1a660a3808"
    "0b12060880e2cfaa061a0b0880e2cfaa061080dac4092205666c735f302a09666c73"
    "5f736f6e617232096d6f6e6f746f6e69633a020801120e666c732d6672616d652d30"
    "3030311a176170706c69636174696f6e2f782d666c732d6672616d65208001420408"
    "021001420408031002420408041003420408051005a206140a0662756e646c65120a"
    "736f6e61725f6f6e6c79"
)


def _slot_view(slot):
  header = slot.header
  return policy.ObservationSlotView(
      reference_id=slot.reference_id,
      content_type=slot.content_type,
      byte_size_present=slot.HasField("byte_size"),
      byte_size=slot.byte_size,
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      clock_domain=header.clock_domain,
  )


def _view_of(message):
  header = message.header
  state = message.vehicle_state
  return policy.ObservationBundleView(
      header_present=message.HasField("header"),
      validity_present=header.HasField("validity"),
      validity_state=header.validity.state,
      frame_id=header.frame_id,
      clock_domain=header.clock_domain,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      max_skew_present=message.HasField("max_skew"),
      max_skew=(message.max_skew.seconds, message.max_skew.nanos),
      fls=_slot_view(message.fls),
      sss=_slot_view(message.sss),
      optical=_slot_view(message.optical),
      point_cloud=_slot_view(message.point_cloud),
      vehicle_state=policy.StateReferenceView(
          message_set=message.HasField("vehicle_state"),
          frame_id=state.header.frame_id,
          source_time_present=state.header.HasField("source_time"),
          source_time=(
              state.header.source_time.seconds,
              state.header.source_time.nanos,
          ),
          receive_time_present=state.header.HasField("receive_time"),
          receive_time=(
              state.header.receive_time.seconds,
              state.header.receive_time.nanos,
          ),
          clock_domain=state.header.clock_domain,
          state_epoch=state.state_epoch,
          world_snapshot_id=state.world_snapshot_id,
      ),
      absent=tuple(
          policy.AbsentModalityEntryView(
              modality=entry.modality, reason=entry.reason
          )
          for entry in message.absent
      ),
      metadata_present=len(message.metadata) > 0,
  )


def _assess(message):
  return policy.assess_multimodal_observation_bundle(_view_of(message))


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
  parsed = _Bundle()
  text_format.Parse(_load_example(name), parsed)
  return parsed


def _sonar_only_parts():
  fls = builder.ObservationSlotParts(
      sequence=11,
      source_seconds=1700000000,
      set_receive=True,
      receive_seconds=1700000000,
      receive_nanos=20000000,
      source_id="fls_0",
      frame_id="fls_sonar",
      clock_domain="monotonic",
      set_validity=True,
      validity_state=stamped_header_pb2.Validity.STATE_VALID,
      reference_id="fls-frame-0001",
      content_type="application/x-fls-frame",
      set_byte_size=True,
      byte_size=128,
  )
  header = builder.ObservationSlotParts(
      sequence=7,
      source_seconds=1700000000,
      set_receive=True,
      receive_seconds=1700000000,
      receive_nanos=100000000,
      source_id="bundle_0",
      frame_id="sync",
      clock_domain="monotonic",
      set_validity=True,
      validity_state=stamped_header_pb2.Validity.STATE_VALID,
  )
  return builder.ObservationBundleParts(
      set_header=True,
      header=header,
      set_max_skew=True,
      max_skew_nanos=policy.FIXTURE_MAX_SKEW_NANOS,
      set_fls=True,
      fls=fls,
      absent=builder.absent_parts(
          (
              (
                  _Modality.OBSERVATION_MODALITY_SONAR_SSS,
                  _Reason.ABSENT_MODALITY_REASON_NOT_CONFIGURED,
              ),
              (
                  _Modality.OBSERVATION_MODALITY_OPTICAL,
                  _Reason.ABSENT_MODALITY_REASON_SENSOR_OFFLINE,
              ),
              (
                  _Modality.OBSERVATION_MODALITY_POINT_CLOUD,
                  _Reason.ABSENT_MODALITY_REASON_OUT_OF_RANGE,
              ),
              (
                  _Modality.OBSERVATION_MODALITY_VEHICLE_STATE,
                  _Reason.ABSENT_MODALITY_REASON_INTENTIONALLY_OMITTED,
              ),
          )
      ),
      metadata=(("bundle", "sonar_only"),),
  )


class ObservationBundleSerializationTest(unittest.TestCase):

  def test_package_and_locked_field_numbers(self):
    descriptor = _Bundle.DESCRIPTOR
    self.assertEqual(
        descriptor.file.package, "intrinsic_proto.perception.sonar"
    )
    self.assertEqual(
        {field.name: field.number for field in descriptor.fields},
        {
            "header": 1,
            "max_skew": 2,
            "fls": 3,
            "sss": 4,
            "optical": 5,
            "point_cloud": 6,
            "vehicle_state": 7,
            "absent": 8,
            "metadata": 100,
        },
    )
    self.assertIsNone(descriptor.fields_by_name.get("robot_type"))
    slot = observation_bundle_pb2.ObservationSlot.DESCRIPTOR
    self.assertEqual(
        {field.name: field.number for field in slot.fields},
        {"header": 1, "reference_id": 2, "content_type": 3, "byte_size": 4},
    )
    state = observation_bundle_pb2.StateReference.DESCRIPTOR
    self.assertEqual(
        {field.name: field.number for field in state.fields},
        {"header": 1, "state_epoch": 2, "world_snapshot_id": 3},
    )
    absent = observation_bundle_pb2.AbsentModalityEntry.DESCRIPTOR
    self.assertEqual(
        {field.name: field.number for field in absent.fields},
        {"modality": 1, "reason": 2},
    )
    self.assertEqual(
        set(descriptor.file.message_types_by_name),
        {
            "AbsentModalityEntry",
            "MultimodalObservationBundle",
            "ObservationSlot",
            "StateReference",
        },
    )
    self.assertEqual(len(descriptor.file.enum_types_by_name), 2)
    self.assertEqual(_Modality.OBSERVATION_MODALITY_SONAR_FLS, 1)
    self.assertEqual(_Reason.ABSENT_MODALITY_REASON_DROPPED_FOR_SKEW, 4)
    dependency_names = {item.name for item in descriptor.file.dependencies}
    self.assertIn("google/protobuf/duration.proto", dependency_names)
    self.assertIn(
        "intrinsic/embodiment/proto/stamped_header.proto", dependency_names
    )
    for name in dependency_names:
      self.assertNotIn("fls_frame", name)
      self.assertNotIn("sss_frame", name)
      self.assertNotIn("/v1/", name)

  def test_default_is_zero_bytes_and_not_accepted(self):
    empty = _Bundle()
    self.assertEqual(empty.SerializeToString(), b"")
    self.assertEqual(empty.ByteSize(), 0)
    self.assertFalse(empty.HasField("header"))
    self.assertFalse(empty.HasField("max_skew"))
    self.assertFalse(empty.HasField("fls"))
    self.assertFalse(empty.HasField("vehicle_state"))
    self.assertEqual(len(empty.absent), 0)
    parsed = _Bundle()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    assessment = _assess(parsed)
    self.assertEqual(assessment.error, _Error.NONE)
    self.assertEqual(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_sonar_only_golden_round_trip(self):
    parsed = _parse_example("observation_bundle_sonar_only.textproto")
    golden = bytes.fromhex(_SONAR_ONLY_GOLDEN_HEX)
    self.assertEqual(parsed.SerializeToString(), golden)
    again = _Bundle()
    again.ParseFromString(golden)
    self.assertEqual(again.SerializeToString(), golden)
    self.assertEqual(again.header.frame_id, "sync")
    self.assertEqual(again.fls.reference_id, "fls-frame-0001")
    self.assertEqual(again.max_skew.nanos, policy.FIXTURE_MAX_SKEW_NANOS)
    self.assertEqual(again.metadata["bundle"], "sonar_only")
    self.assertFalse(again.HasField("sss"))
    self.assertFalse(again.HasField("optical"))
    assessment = _assess(again)
    self.assertEqual(assessment.error, _Error.NONE)
    self.assertEqual(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)
    self.assertTrue(assessment.measured_skew_present)
    self.assertEqual(assessment.measured_skew, (0, 0))

  def test_builder_matches_sonar_only_fixture(self):
    built = builder.build_multimodal_observation_bundle(_sonar_only_parts())
    parsed = _parse_example("observation_bundle_sonar_only.textproto")
    self.assertEqual(built.SerializeToString(), parsed.SerializeToString())
    self.assertEqual(
        built.SerializeToString(), bytes.fromhex(_SONAR_ONLY_GOLDEN_HEX)
    )
    self.assertTrue(_assess(built).accepted)
    self.assertEqual(built.fls.reference_id, "fls-frame-0001")
    self.assertFalse(built.HasField("sss"))
    self.assertFalse(built.HasField("optical"))
    self.assertFalse(built.HasField("point_cloud"))
    self.assertFalse(built.HasField("vehicle_state"))

  def test_builder_does_not_rewrite_a_bad_snapshot(self):
    parts = builder.ObservationBundleParts(
        set_header=True,
        header=builder.ObservationSlotParts(
            frame_id="sync",
            clock_domain="monotonic",
            set_validity=True,
            validity_state=stamped_header_pb2.Validity.STATE_VALID,
        ),
        set_max_skew=True,
        max_skew_nanos=policy.FIXTURE_MAX_SKEW_NANOS,
        vehicle_state=builder.StateReferenceParts(
            set=True,
            stamp=builder.ObservationSlotParts(
                frame_id="body",
                clock_domain="monotonic",
                source_seconds=1700000000,
            ),
            world_snapshot_id="not-a-hex",
            state_epoch=0,
        ),
    )
    built = builder.build_multimodal_observation_bundle(parts)
    self.assertEqual(built.vehicle_state.world_snapshot_id, "not-a-hex")
    self.assertEqual(built.vehicle_state.state_epoch, 0)
    self.assertEqual(_assess(built).error, _Error.SNAPSHOT_ID)

  def test_unknown_field_is_preserved(self):
    with_unknown = bytearray(bytes.fromhex(_SONAR_ONLY_GOLDEN_HEX))
    with_unknown.extend(b"\xa8\x06\x07")
    parsed = _Bundle()
    parsed.ParseFromString(bytes(with_unknown))
    self.assertEqual(parsed.header.frame_id, "sync")
    self.assertEqual(parsed.SerializeToString(), bytes(with_unknown))
    self.assertTrue(_assess(parsed).accepted)

  def test_unknown_nested_field_is_preserved(self):
    slot = observation_bundle_pb2.ObservationSlot(reference_id="fls-frame-0001")
    nested = bytearray(slot.SerializeToString())
    nested.extend(b"\x78\x01")
    raw = bytearray(b"\x1a")
    raw.append(len(nested))
    raw.extend(nested)
    parsed = _Bundle()
    parsed.ParseFromString(bytes(raw))
    self.assertEqual(parsed.fls.reference_id, "fls-frame-0001")
    self.assertEqual(parsed.SerializeToString(), bytes(raw))

  def test_fixtures_match_locked_results(self):
    cases = (
        ("observation_bundle_sonar_only.textproto", _Error.NONE, True),
        ("observation_bundle_camera_only.textproto", _Error.NONE, True),
        ("observation_bundle_all_modalities.textproto", _Error.NONE, True),
        (
            "observation_bundle_excessive_skew.textproto",
            _Error.EXCESSIVE_SKEW,
            False,
        ),
        (
            "observation_bundle_frame_mismatch.textproto",
            _Error.FRAME_MISMATCH,
            False,
        ),
        (
            "observation_bundle_time_reversal.textproto",
            _Error.TIME_REVERSAL,
            False,
        ),
        (
            "observation_bundle_missing_absent_reason.textproto",
            _Error.ABSENT_LIST,
            False,
        ),
        (
            "observation_bundle_bad_snapshot_id.textproto",
            _Error.SNAPSHOT_ID,
            False,
        ),
    )
    for name, error, accepted in cases:
      assessment = _assess(_parse_example(name))
      self.assertEqual(assessment.error, error, name)
      self.assertEqual(assessment.validity, _Kind.VALID, name)
      self.assertEqual(assessment.accepted, accepted, name)
    skew = _assess(
        _parse_example("observation_bundle_excessive_skew.textproto")
    )
    self.assertTrue(skew.measured_skew_present)
    self.assertEqual(skew.measured_skew, (0, policy.FIXTURE_MAX_SKEW_NANOS + 1))
    all_modalities = _assess(
        _parse_example("observation_bundle_all_modalities.textproto")
    )
    self.assertEqual(all_modalities.measured_skew, (0, 180000000))
    self.assertTrue(all_modalities.accepted)

  def test_deterministic_bytes(self):
    first = builder.build_multimodal_observation_bundle(_sonar_only_parts())
    second = builder.build_multimodal_observation_bundle(_sonar_only_parts())
    self.assertEqual(first.SerializeToString(), second.SerializeToString())
    left = _assess(first)
    right = _assess(second)
    self.assertEqual(left, right)
    first.metadata["note"] = ""
    first.metadata["a"] = "1"
    second.metadata["a"] = "1"
    second.metadata["note"] = ""
    self.assertTrue(_assess(first).accepted)
    self.assertTrue(_assess(second).accepted)
    self.assertEqual(_assess(first).error, _assess(second).error)


if __name__ == "__main__":
  unittest.main()
