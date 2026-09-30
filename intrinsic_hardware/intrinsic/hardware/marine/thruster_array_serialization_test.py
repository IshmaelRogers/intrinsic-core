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

"""Serialization and textproto tests for the thruster array."""

import os
import unittest

from google.protobuf import text_format
from intrinsic.hardware.marine import fake_thruster_array
from intrinsic.hardware.marine import thruster_array_pb2
from intrinsic.hardware.marine import thruster_array_policy

from intrinsic.embodiment.proto import stamped_header_pb2

# Canonical serialization of the nominal command and of
# FakeThrusterArray().apply(that command) with the default config.
# Keep in sync with thruster_array_serialization_test.cc.
_NOMINAL_COMMAND_GOLDEN_HEX = (
    "0a3c082a120b0880e2cfaa061080e59a771a060881e2cfaa06220e74687275737465"
    "725f61727261792a04626f647932096d6f6e6f746f6e69633a02080112150a0a7375"
    "7267655f706f7274110000000000002440121a0a0f73757267655f73746172626f61"
    "72641100000000000014c012140a09737761795f666f726511000000000000000012"
    "130a08737761795f61667411000000000000104012150a0a68656176655f666f7265"
    "11000000000000204012140a0968656176655f6166741100000000000000c0"
)
_NOMINAL_FEEDBACK_GOLDEN_HEX = (
    "0a3c082a120b0880e2cfaa061080e59a771a060881e2cfaa06220e74687275737465"
    "725f61727261792a04626f647932096d6f6e6f746f6e69633a02080112340a0a7375"
    "7267655f706f72741100000000000024401900000000000024402000280031000000"
    "000000f03f39000000000000f03f12390a0f73757267655f73746172626f61726411"
    "00000000000014c01900000000000014c02000280031000000000000f03f39000000"
    "000000f03f12330a09737761795f666f726511000000000000000019000000000000"
    "00002000280031000000000000f03f39000000000000f03f12320a08737761795f61"
    "66741100000000000010401900000000000010402000280031000000000000f03f39"
    "000000000000f03f12340a0a68656176655f666f7265110000000000002040190000"
    "0000000020402000280031000000000000f03f39000000000000f03f12330a096865"
    "6176655f6166741100000000000000c01900000000000000c0200028003100000000"
    "0000f03f39000000000000f03f"
)

_SLOT_NAMES = (
    "surge_port",
    "surge_starboard",
    "sway_fore",
    "sway_aft",
    "heave_fore",
    "heave_aft",
)
_SLOT_THRUST_N = (10.0, -5.0, 0.0, 4.0, 8.0, -2.0)


def _fill_nominal_command(command):
  header = command.header
  header.sequence = 42
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001
  header.source_id = "thruster_array"
  header.frame_id = "body"
  header.clock_domain = "monotonic"
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  for name, thrust in zip(_SLOT_NAMES, _SLOT_THRUST_N):
    element = command.thrusters.add()
    element.name = name
    element.thrust_n = thrust


def _header_view(message):
  header = message.header
  return thruster_array_policy.ThrusterHeaderView(
      header_present=message.HasField("header"),
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      header_validity_present=header.HasField("validity"),
      header_validity_state=header.validity.state,
  )


def _assess_command(message):
  elements = tuple(
      thruster_array_policy.ThrusterCommandElementView(
          name_present=element.HasField("name"),
          name=element.name,
          thrust_present=element.HasField("thrust_n"),
          thrust_n=element.thrust_n,
          enable_present=element.HasField("enable"),
          enable=element.enable,
      )
      for element in message.thrusters
  )
  return thruster_array_policy.assess_thruster_array_command(
      thruster_array_policy.ThrusterArrayCommandView(
          header=_header_view(message), thrusters=elements
      )
  )


def _assess_feedback(message):
  elements = tuple(
      thruster_array_policy.ThrusterFeedbackElementView(
          name_present=element.HasField("name"),
          name=element.name,
          commanded_thrust_present=element.HasField("commanded_thrust_n"),
          commanded_thrust_n=element.commanded_thrust_n,
          measured_thrust_present=element.HasField("measured_thrust_n"),
          measured_thrust_n=element.measured_thrust_n,
          saturated_present=element.HasField("saturated"),
          saturated=element.saturated,
          health_present=element.HasField("health"),
          health=element.health,
          health_derate_present=element.HasField("health_derate"),
          health_derate=element.health_derate,
          efficiency_present=element.HasField("efficiency"),
          efficiency=element.efficiency,
      )
      for element in message.thrusters
  )
  return thruster_array_policy.assess_thruster_array_feedback(
      thruster_array_policy.ThrusterArrayFeedbackView(
          header=_header_view(message), thrusters=elements
      )
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


class ThrusterArraySerializationTest(unittest.TestCase):

  def test_contract_shape_and_health_wire_numbers(self):
    command = thruster_array_pb2.ThrusterArrayCommand.DESCRIPTOR
    feedback = thruster_array_pb2.ThrusterArrayFeedback.DESCRIPTOR
    self.assertEqual(command.file.package, "intrinsic_proto.hardware.marine")
    self.assertEqual(command.fields_by_name["header"].number, 1)
    self.assertEqual(command.fields_by_name["thrusters"].number, 2)
    self.assertEqual(feedback.fields_by_name["header"].number, 1)
    self.assertIsNone(command.fields_by_name.get("wrench"))
    self.assertEqual(
        command.fields_by_name["header"].message_type.full_name,
        "intrinsic_proto.embodiment.StampedHeader",
    )
    for name in ("name", "thrust_n", "enable"):
      self.assertTrue(
          command.fields_by_name["thrusters"]
          .message_type.fields_by_name[name]
          .has_presence,
          name,
      )
    health = thruster_array_pb2.ThrusterHealth.DESCRIPTOR
    self.assertEqual(
        [(value.name, value.number) for value in health.values],
        [
            ("THRUSTER_HEALTH_NOMINAL", 0),
            ("THRUSTER_HEALTH_DISABLED", 1),
            ("THRUSTER_HEALTH_DERATED", 2),
            ("THRUSTER_HEALTH_STUCK_OFF", 3),
            ("THRUSTER_HEALTH_FAILED", 4),
        ],
    )
    self.assertIsNone(health.values_by_name.get("THRUSTER_HEALTH_UNSPECIFIED"))
    validity = stamped_header_pb2.Validity.State.DESCRIPTOR
    self.assertEqual(len(validity.values), 3)
    self.assertIsNone(validity.values_by_name.get("STATE_DEGRADED"))

  def test_defaults_are_empty_and_opt_in(self):
    self.assertEqual(
        thruster_array_pb2.ThrusterArrayCommand().SerializeToString(), b""
    )
    self.assertEqual(
        thruster_array_pb2.ThrusterArrayFeedback().SerializeToString(), b""
    )

  def test_presence_is_distinct_from_default_values(self):
    thrust = thruster_array_pb2.ThrusterCommandElement()
    self.assertFalse(thrust.HasField("thrust_n"))
    thrust.thrust_n = 0.0
    self.assertTrue(thrust.HasField("thrust_n"))
    self.assertNotEqual(thrust.SerializeToString(), b"")

    enable = thruster_array_pb2.ThrusterCommandElement()
    self.assertFalse(enable.HasField("enable"))
    enable.enable = False
    self.assertTrue(enable.HasField("enable"))
    self.assertNotEqual(enable.SerializeToString(), b"")

    health = thruster_array_pb2.ThrusterFeedbackElement()
    self.assertFalse(health.HasField("health"))
    health.health = thruster_array_pb2.THRUSTER_HEALTH_NOMINAL
    self.assertTrue(health.HasField("health"))
    self.assertNotEqual(health.SerializeToString(), b"")

    saturated = thruster_array_pb2.ThrusterFeedbackElement()
    saturated.saturated = False
    self.assertTrue(saturated.HasField("saturated"))
    self.assertNotEqual(saturated.SerializeToString(), b"")

  def test_nominal_round_trip_matches_golden_and_fake(self):
    command = thruster_array_pb2.ThrusterArrayCommand()
    _fill_nominal_command(command)
    command_hex = command.SerializeToString().hex()
    self.assertEqual(command_hex, _NOMINAL_COMMAND_GOLDEN_HEX, command_hex)
    parsed_command = thruster_array_pb2.ThrusterArrayCommand()
    parsed_command.ParseFromString(bytes.fromhex(_NOMINAL_COMMAND_GOLDEN_HEX))
    self.assertEqual(
        parsed_command.SerializeToString(),
        bytes.fromhex(_NOMINAL_COMMAND_GOLDEN_HEX),
    )
    self.assertTrue(_assess_command(parsed_command).accepted)

    feedback = fake_thruster_array.FakeThrusterArray().apply(command)
    feedback_hex = feedback.SerializeToString().hex()
    self.assertEqual(feedback_hex, _NOMINAL_FEEDBACK_GOLDEN_HEX, feedback_hex)
    parsed = thruster_array_pb2.ThrusterArrayFeedback()
    parsed.ParseFromString(bytes.fromhex(_NOMINAL_FEEDBACK_GOLDEN_HEX))
    self.assertEqual(parsed.header.sequence, 42)
    self.assertEqual(parsed.header.frame_id, "body")
    self.assertEqual(len(parsed.thrusters), 6)
    self.assertEqual(parsed.thrusters[2].measured_thrust_n, 0.0)
    self.assertEqual(
        parsed.thrusters[0].health, thruster_array_pb2.THRUSTER_HEALTH_NOMINAL
    )
    self.assertTrue(_assess_feedback(parsed).accepted)

  def test_unknown_field_and_health_are_preserved(self):
    command = thruster_array_pb2.ThrusterArrayCommand()
    _fill_nominal_command(command)
    golden = command.SerializeToString()
    with_unknown = golden + b"\xa0\x06\x07"
    parsed = thruster_array_pb2.ThrusterArrayCommand()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.thrusters[0].name, "surge_port")
    self.assertEqual(parsed.SerializeToString(), with_unknown)

    feedback = fake_thruster_array.FakeThrusterArray().apply(command)
    feedback.thrusters[0].health = 99
    reparsed = thruster_array_pb2.ThrusterArrayFeedback()
    reparsed.ParseFromString(feedback.SerializeToString())
    self.assertEqual(reparsed.thrusters[0].health, 99)
    assessment = _assess_feedback(reparsed)
    self.assertEqual(
        assessment.error, thruster_array_policy.ThrusterArrayError.HEALTH
    )
    self.assertFalse(assessment.accepted)

  def test_textproto_examples_match_the_fixture(self):
    parsed_command = thruster_array_pb2.ThrusterArrayCommand()
    text_format.Parse(
        _load_example("thruster_array_command_nominal.textproto"),
        parsed_command,
    )
    coded = thruster_array_pb2.ThrusterArrayCommand()
    _fill_nominal_command(coded)
    self.assertEqual(
        parsed_command.SerializeToString(), coded.SerializeToString()
    )
    self.assertTrue(_assess_command(parsed_command).accepted)

    parsed_feedback = thruster_array_pb2.ThrusterArrayFeedback()
    text_format.Parse(
        _load_example("thruster_array_feedback_nominal.textproto"),
        parsed_feedback,
    )
    faked = fake_thruster_array.FakeThrusterArray().apply(coded)
    self.assertEqual(
        parsed_feedback.SerializeToString(), faked.SerializeToString()
    )
    self.assertTrue(_assess_feedback(parsed_feedback).accepted)


if __name__ == "__main__":
  unittest.main()
